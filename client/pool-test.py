#!/usr/bin/env python3
"""
Fixed-command pool test for the real sub.

Sends ONE degree of freedom at a fixed level for a fixed duration, through the SAME
thruster mix and packet format as the manual driver, then returns to neutral so the sub
glides to rest. This mirrors the Unity sim's fixed-command test exactly, so real and sim
runs are directly comparable.

Protocol for each run:
  1. Park the sub at rest, mark its start position.
  2. Run this script; it counts down, prints GO, drives for --duration, prints STOP.
  3. The sub coasts to rest. Measure start -> rest (this matches the sim's TOTAL distance).

Examples:
  python pool_test.py surge --dir + --duration 3
  python pool_test.py strafe --dir - --duration 3
  python pool_test.py yaw --dir + --duration 3 --capture-sensors yaw_run1.csv
  python pool_test.py heave --dir - --duration 3 --capture-sensors dive_run1.csv --wifi
"""

import argparse
import configparser
import os
import socket
import struct
import sys
import threading
import time

NEUTRAL = 1500
LIGHT_OFF = 1100


def clamp(x):
    return max(-1.0, min(1.0, x))


def mix(surge, strafe, heave, yaw):
    # Identical to the manual driver's mix.
    fl = clamp(surge - strafe - yaw)
    fr = clamp(surge + strafe + yaw)
    rl = clamp(surge + strafe - yaw)
    rr = clamp(surge - strafe + yaw)
    v1 = clamp(heave)
    v2 = clamp(-heave)
    return fl, fr, rl, rr, v1, v2


def to_pwm(x, amp):
    return int(NEUTRAL + x * amp)


def thruster_packet(thrusters, amp):
    fl, fr, rl, rr, v1, v2 = thrusters
    return struct.pack(
        "<7H",
        to_pwm(fl, amp),
        to_pwm(fr, amp),
        to_pwm(rl, amp),
        to_pwm(rr, amp),
        to_pwm(v1, amp),
        to_pwm(v2, amp),
        LIGHT_OFF,
    )


def neutral_packet():
    return struct.pack("<7H", *([NEUTRAL] * 6), LIGHT_OFF)


def load_config(mode):
    path = os.path.expanduser(".rov_server_creds")
    cfg = configparser.ConfigParser()
    if not os.path.exists(path) or not cfg.read(path):
        sys.exit(f"✗ ERROR: Config file not found or empty at '{path}'")
    try:
        rov_ip = cfg[mode]["rov_ip"]
        thruster_port = cfg.getint("DEFAULT", "thruster_port")
    except (KeyError, configparser.NoSectionError) as e:
        sys.exit(f"✗ ERROR: Missing section or key in config: {e}")
    return (rov_ip, thruster_port), cfg


def capture_sensors(port, out_path, stop_event):
    """Record raw IMU/depth packets with timestamps for later decoding.
    (Format unknown here, so this logs raw bytes as hex — decode offline.)"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    s.settimeout(0.2)
    t0 = time.time()
    with open(out_path, "w") as f:
        f.write("t_seconds,length,hex\n")
        while not stop_event.is_set():
            try:
                data, _ = s.recvfrom(4096)
            except socket.timeout:
                continue
            f.write(f"{time.time() - t0:.4f},{len(data)},{data.hex()}\n")
    s.close()


def main():
    ap = argparse.ArgumentParser(
        description="Fixed-command pool test for the real sub."
    )
    ap.add_argument("motion", choices=["surge", "strafe", "heave", "yaw"])
    ap.add_argument(
        "--dir", choices=["+", "-"], default="+", help="direction (default +)"
    )
    ap.add_argument(
        "--duration", type=float, default=3.0, help="seconds (match the sim)"
    )
    ap.add_argument(
        "--level", type=float, default=1.0, help="command magnitude 0..1 (default 1.0)"
    )
    ap.add_argument(
        "--amp",
        type=int,
        default=100,
        help="PWM amplitude, like the driver's AMP (default 100)",
    )
    ap.add_argument(
        "--rate", type=float, default=50.0, help="send rate in Hz (default 50)"
    )
    ap.add_argument(
        "--countdown", type=int, default=3, help="countdown seconds before GO"
    )
    ap.add_argument(
        "--wifi", action="store_true", help="use wifi config instead of lan"
    )
    ap.add_argument(
        "--capture-sensors",
        metavar="FILE",
        default=None,
        help="also record raw IMU/depth packets to FILE",
    )
    args = ap.parse_args()

    mode = "wifi" if args.wifi else "lan"
    addr, cfg = load_config(mode)

    sign = 1.0 if args.dir == "+" else -1.0
    level = clamp(abs(args.level)) * sign

    cmd = {"surge": 0.0, "strafe": 0.0, "heave": 0.0, "yaw": 0.0}
    cmd[args.motion] = level
    thr = mix(cmd["surge"], cmd["strafe"], cmd["heave"], cmd["yaw"])

    print(f"✓ Target {addr[0]}:{addr[1]}  ({mode})")
    print(
        f"  motion = {args.motion}{args.dir}   level = {abs(level):.2f}   "
        f"amp = {args.amp}   duration = {args.duration:.1f}s"
    )
    print(
        f"  thrusters: fl={thr[0]:+.2f} fr={thr[1]:+.2f} rl={thr[2]:+.2f} "
        f"rr={thr[3]:+.2f} v1={thr[4]:+.2f} v2={thr[5]:+.2f}"
    )

    stop_event = threading.Event()
    cap_thread = None
    if args.capture_sensors:
        port = cfg.getint("DEFAULT", "imu_and_depth_port")
        cap_thread = threading.Thread(
            target=capture_sensors,
            args=(port, args.capture_sensors, stop_event),
            daemon=True,
        )
        cap_thread.start()
        print(
            f"  recording raw sensor packets on port {port} -> {args.capture_sensors}"
        )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(neutral_packet(), addr)  # establish link / ensure neutral

    dt = 1.0 / args.rate
    packet = thruster_packet(thr, args.amp)

    try:
        for i in range(args.countdown, 0, -1):
            print(f"  starting in {i}...", flush=True)
            time.sleep(1.0)
        print(">>> GO  (sub is now driving)")

        t_next = time.time()
        t_end = t_next + args.duration
        while time.time() < t_end:
            sock.sendto(packet, addr)
            t_next += dt
            delay = t_next - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                t_next = time.time()

        print(">>> STOP (command ended; sub glides to rest — measure start to rest)")
    except KeyboardInterrupt:
        print("\n! Interrupted — sending neutral.")
    finally:
        # Send neutral for ~0.5 s so the sub definitely stops.
        for _ in range(int(0.5 / dt) + 1):
            sock.sendto(neutral_packet(), addr)
            time.sleep(dt)
        stop_event.set()
        if cap_thread:
            cap_thread.join(timeout=1.0)
        sock.close()
        print("✓ Neutral sent. Done.")


if __name__ == "__main__":
    main()
