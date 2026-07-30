import os
import sys

import cv2


def extract_frames(video_path, out_dir, step=1):
    if step < 1:
        raise ValueError(f"step must be >= 1, got {step}")

    os.makedirs(out_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"could not open {video_path}")

    name = os.path.splitext(os.path.basename(video_path))[0]
    frame_idx = 0
    saved = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx % step == 0:
            out_path = os.path.join(out_dir, f"{name}_frame_{saved:05d}.jpg")
            cv2.imwrite(out_path, frame)
            saved += 1
        frame_idx += 1

    cap.release()
    print(f"{video_path}: read {frame_idx} frames, saved {saved} to {out_dir}")


if __name__ == "__main__":
    args = sys.argv[1:]

    step = 1
    if len(args) > 1 and args[-1].isdigit():
        step = int(args.pop())

    if not args:
        print("Usage: python extract_frames.py <video1> [video2 ...] [step]")
        sys.exit(1)

    for v in args:
        extract_frames(v, os.path.splitext(v)[0] + "_frames", step)
