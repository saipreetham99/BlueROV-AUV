#!/usr/bin/env python3
"""
Combined ROV server (runs on the Raspberry Pi).

  * receives 7-channel thruster/light UDP packets and drives the PCA9685
  * streams the camera to the client over UDP

Both run on their own threads, so a camera fault never stops thruster control
(the safety-critical loop keeps running). Reads .rov_server_creds; --wifi picks the
wifi section. Ctrl+C neutralises and disables outputs cleanly.
"""

import argparse
import configparser
import os
import signal
import socket
import struct
import sys
import threading
import time

import cv2
from pca9685 import PCA9685

# --- thruster / light ---
NEUTRAL_PULSE = 1500
LIGHT_OFF_PULSE = 1100
THRUSTER_CHANNELS = range(6)
LIGHT_CHANNEL = 9
THR_LOOP_DT = 0.02  # 50 Hz socket timeout
THR_TIMEOUT = 0.5  # neutral after this many seconds without packets

# --- video ---
CHUNK = 60_000  # keep UDP packets < 65507 bytes

pca = PCA9685()
pca.set_pwm_frequency(50)
pca.output_enable()

stop_event = threading.Event()


def neutral_all():
    for ch in THRUSTER_CHANNELS:
        pca.pwm[ch] = NEUTRAL_PULSE
    pca.pwm[LIGHT_CHANNEL] = LIGHT_OFF_PULSE


def safe_shutdown(*_):
    print("\nExecuting safe shutdown...")
    stop_event.set()
    neutral_all()
    time.sleep(0.05)
    pca.output_disable()
    print("Shutdown complete.")


signal.signal(signal.SIGINT, safe_shutdown)
signal.signal(signal.SIGHUP, safe_shutdown)


def load_config():
    path = os.path.expanduser(".rov_server_creds")
    cfg = configparser.ConfigParser()
    if not os.path.exists(path) or not cfg.read(path):
        sys.exit(f"✗ ERROR: Config file not found or empty at '{path}'")
    return cfg


def thruster_loop(cfg):
    port = cfg.getint("DEFAULT", "thruster_port")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", port))
    sock.settimeout(THR_LOOP_DT)
    print(f"[thrusters] listening on *:{port}")

    last_rx = time.time()
    fmt, size = "<7H", 14
    try:
        while not stop_event.is_set():
            try:
                data, _ = sock.recvfrom(size)
                if len(data) == size:
                    fl, fr, rl, rr, v1, v2, light = struct.unpack(fmt, data)
                    pca.pwm[0] = fl
                    pca.pwm[1] = fr
                    pca.pwm[2] = rl
                    pca.pwm[3] = rr
                    pca.pwm[4] = v1
                    pca.pwm[5] = v2
                    pca.pwm[LIGHT_CHANNEL] = light
                    last_rx = time.time()
            except socket.timeout:
                if time.time() - last_rx > THR_TIMEOUT:
                    for ch in range(6):
                        pca.pwm[ch] = NEUTRAL_PULSE
                    last_rx = time.time()
    finally:
        sock.close()
        print("[thrusters] stopped")


def video_loop(cfg, mode, source, chunk, quality_override):
    try:
        client_ip = cfg[mode]["client_ip"]
    except (KeyError, configparser.NoSectionError) as e:
        print(f"[video] config error ({e}); video disabled")
        return
    port = cfg.getint("DEFAULT", "video_port")
    quality = (
        quality_override
        if quality_override is not None
        else cfg.getint("DEFAULT", "video_quality", fallback=75)
    )

    cam = cv2.VideoCapture(source)
    if not cam.isOpened():
        print(f"[video] cannot open camera {source}; video disabled")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    enc = [cv2.IMWRITE_JPEG_QUALITY, quality]
    fid = 0
    print(f"[video] streaming to {client_ip}:{port} @ {quality}%")
    try:
        while not stop_event.is_set():
            ok, frame = cam.read()
            if not ok:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode(".jpg", frame, enc)
            if not ok:
                continue
            data = buf.tobytes()
            blocks = (len(data) - 1) // chunk + 1
            for idx in range(blocks):
                part = data[idx * chunk : idx * chunk + chunk]
                header = struct.pack("!HHH", fid & 0xFFFF, blocks, idx)
                sock.sendto(header + part, (client_ip, port))
            fid += 1
            time.sleep(0.01)
    finally:
        cam.release()
        sock.close()
        print("[video] stopped")


def main():
    ap = argparse.ArgumentParser(description="Combined ROV server (thrusters + video).")
    ap.add_argument("--wifi", action="store_true", help="use wifi config section")
    ap.add_argument("--source", type=int, default=0, help="camera index")
    ap.add_argument(
        "--quality", type=int, default=None, help="JPEG quality (else config)"
    )
    ap.add_argument("--chunk", type=int, default=CHUNK)
    ap.add_argument("--no-video", action="store_true", help="run thrusters only")
    args = ap.parse_args()

    mode = "wifi" if args.wifi else "lan"
    cfg = load_config()
    print(f"✓ Loaded '{mode}' settings")

    neutral_all()

    threads = [threading.Thread(target=thruster_loop, args=(cfg,), daemon=True)]
    if not args.no_video:
        threads.append(
            threading.Thread(
                target=video_loop,
                args=(cfg, mode, args.source, args.chunk, args.quality),
                daemon=True,
            )
        )
    for t in threads:
        t.start()

    print("Server running. Press Ctrl+C to stop.")
    try:
        while not stop_event.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        safe_shutdown()
    time.sleep(0.15)


if __name__ == "__main__":
    main()
