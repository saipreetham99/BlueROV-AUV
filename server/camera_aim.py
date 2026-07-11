#!/usr/bin/env python3
"""
camera_aim.py  -  aim the camera servo (PCA9685 channel 15).

Runs on the Pi (needs the pca9685 driver + hardware powered).
Type a pulse and watch the camera move; adjust until it points where you
want. Thrusters are held neutral (stopped); no other channel is touched.

Commands (press Enter after each):
    <number>   set an absolute pulse, e.g.  1500
    w / +      nudge up by the step
    s / -      nudge down by the step
    c          center (1500)
    step <n>   change the nudge size, e.g.  step 10
    q          quit (leaves the camera holding its current aim)

Servo pulse is clamped to 1000-2000 us. Center is ~1500.
"""

import argparse

from pca9685 import PCA9685

NEUTRAL = 1500
LIGHT_OFF = 1100
THRUSTER_CHANNELS = range(6)
SERVO_MIN, SERVO_MAX, CENTER = 1000, 2000, 1500


def clamp(v):
    return max(SERVO_MIN, min(SERVO_MAX, v))


def main():
    ap = argparse.ArgumentParser(
        description="Aim the camera servo on a PCA9685 channel."
    )
    ap.add_argument(
        "--channel", type=int, default=15, help="camera servo channel (default 15)"
    )
    ap.add_argument(
        "--start", type=int, default=CENTER, help="starting pulse (default 1500)"
    )
    ap.add_argument(
        "--step", type=int, default=25, help="nudge size in us (default 25)"
    )
    args = ap.parse_args()

    ch = args.channel
    pulse = clamp(args.start)
    step = args.step

    pca = PCA9685()
    pca.set_pwm_frequency(50)
    pca.output_enable()
    # hold thrusters stopped; do not touch anything except the camera channel
    for c in THRUSTER_CHANNELS:
        pca.pwm[c] = NEUTRAL
    pca.pwm[ch] = pulse

    print(f"Camera aim on channel {ch}. Start pulse {pulse}, step {step}.")
    print("Type a number, w/+ up, s/- down, c center, 'step N', q quit.\n")

    try:
        while True:
            raw = input(f"[ch{ch} = {pulse}] > ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            elif raw in ("w", "+"):
                pulse = clamp(pulse + step)
            elif raw in ("s", "-"):
                pulse = clamp(pulse - step)
            elif raw == "c":
                pulse = CENTER
            elif raw.startswith("step"):
                parts = raw.split()
                if len(parts) == 2 and parts[1].isdigit():
                    step = int(parts[1])
                    print(f"    step is now {step}")
                else:
                    print("    usage: step <number>")
                continue
            elif raw.lstrip("-").isdigit():
                pulse = clamp(int(raw))
            elif raw == "":
                continue
            else:
                print("    ? number, w/+, s/-, c, 'step N', or q")
                continue
            pca.pwm[ch] = pulse
    except KeyboardInterrupt:
        print()
    finally:
        # Leave output ENABLED so the servo keeps holding the aim.
        # Thrusters stay neutral (stopped), which is safe.
        for c in THRUSTER_CHANNELS:
            pca.pwm[c] = NEUTRAL
        pca.pwm[ch] = pulse
        print(f"\nCamera left aimed at pulse {pulse} on channel {ch}.")
        print("Output stays enabled so it holds. Thrusters are neutral (safe).")


if __name__ == "__main__":
    main()
