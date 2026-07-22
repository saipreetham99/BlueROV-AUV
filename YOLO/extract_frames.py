import cv2
import os
import sys


def extract_frames(video_path, out_dir, fps_target=1):
    os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    video_fps = cap.get(cv2.CAP_PROP_FPS)
    step = max(int(round(video_fps / fps_target)), 1)

    frame_idx = 0
    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx % step == 0:
            name = os.path.splitext(os.path.basename(video_path))[0]
            out_path = os.path.join(out_dir, f"{name}_frame_{saved:05d}.jpg")
            cv2.imwrite(out_path, frame)
            saved += 1
        frame_idx += 1

    cap.release()
    print(f"{video_path}: saved {saved} frames to {out_dir}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python extract_frames.py <video1.mp4> <video2.mp4> [fps_target]")
        sys.exit(1)

    videos = sys.argv[1:3]
    fps_target = float(sys.argv[3]) if len(sys.argv) > 3 else 1

    for v in videos:
        out_dir = os.path.splitext(v)[0] + "_frames"
        extract_frames(v, out_dir, fps_target)
