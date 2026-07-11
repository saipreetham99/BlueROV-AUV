#!/usr/bin/env python3
"""
light_finder.py  -  find which PCA9685 channel the light is wired to.

Runs on the Pi (needs the pca9685 driver + hardware powered).
Tests one channel at a time by blinking it between the light's OFF (1100)
and ON (1900) pulse. Watch the light; the channel where it flashes is the
one to set as LIGHT_CHANNEL in rov-server.py.

Thruster channels 0-5 are SKIPPED by default so no thruster spins.

Examples:
    python light_finder.py                # test channels 6..15
    python light_finder.py --channel 9    # test only channel 9
    python light_finder.py --start 6 --end 11
"""

import argparse
import time

from pca9685 import PCA9685

NEUTRAL = 1500
LIGHT_OFF = 1100
LIGHT_ON = 1900
THRUSTER_CHANNELS = range(6)  # 0-5 are thrusters; skipped unless overridden


def safe_state(pca):
    """Thrusters held neutral (stopped), every other channel off."""
    for ch in range(16):
        pca.pwm[ch] = NEUTRAL if ch in THRUSTER_CHANNELS else LIGHT_OFF


def blink(pca, ch, times, period):
    for _ in range(times):
        pca.pwm[ch] = LIGHT_ON
        time.sleep(period / 2)
        pca.pwm[ch] = LIGHT_OFF
        time.sleep(period / 2)


def main():
    ap = argparse.ArgumentParser(
        description="Find the PCA9685 channel the light is on."
    )
    ap.add_argument(
        "--start", type=int, default=6, help="first channel to test (default 6)"
    )
    ap.add_argument(
        "--end", type=int, default=15, help="last channel to test (default 15)"
    )
    ap.add_argument(
        "--channel", type=int, default=None, help="test only this one channel"
    )
    ap.add_argument(
        "--blinks", type=int, default=6, help="blinks per channel (default 6)"
    )
    ap.add_argument(
        "--period", type=float, default=0.5, help="seconds per blink (default 0.5)"
    )
    ap.add_argument(
        "--include-thrusters",
        action="store_true",
        help="also test channels 0-5 (WARNING: thrusters WILL spin)",
    )
    args = ap.parse_args()

    pca = PCA9685()
    pca.set_pwm_frequency(50)
    pca.output_enable()
    safe_state(pca)

    if args.channel is not None:
        channels = [args.channel]
    else:
        channels = list(range(args.start, args.end + 1))
        if not args.include_thrusters:
            channels = [c for c in channels if c not in THRUSTER_CHANNELS]

    print("Light finder - watch the light and note where it flashes.")
    print(f"Channels to test: {channels}")
    print("Thrusters are held neutral. Ctrl+C to stop.\n")

    found = None
    try:
        for ch in channels:
            input(f">>> Channel {ch}: press Enter, then watch the light...")
            print(f"    blinking channel {ch} ...")
            blink(pca, ch, args.blinks, args.period)
            safe_state(pca)
            ans = (
                input(f"    Did the light flash on channel {ch}? [y/N/q]: ")
                .strip()
                .lower()
            )
            if ans == "y":
                found = ch
                break
            if ans == "q":
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        safe_state(pca)
        time.sleep(0.05)
        pca.output_disable()

    if found is not None:
        print(f"\n*** Light is on channel {found} ***")
        print(f"Set  LIGHT_CHANNEL = {found}  in rov-server.py")
    else:
        print("\nNo channel identified. Try a wider range (--start/--end),")
        print("or --include-thrusters if the light might share a low channel")
        print("(WARNING: that will spin thrusters - keep clear / props off).")


if __name__ == "__main__":
    main()
