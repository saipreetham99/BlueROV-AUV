import argparse
import time
import cv2
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "weights"
    )  # your trained .pt, e.g. runs/detect/train/weights/best.pt
    ap.add_argument("video")  # path to a video file, or "0" for webcam
    ap.add_argument("--conf", type=float, default=0.80)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default=None)  # e.g. "0" for GPU, "cpu"; None = auto
    args = ap.parse_args()

    model = YOLO(args.weights)

    source = 0 if args.video == "0" else args.video
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"could not open: {args.video}")

    # Get video FPS for realtime sync
    vid_fps = cap.get(cv2.CAP_PROP_FPS)
    if vid_fps <= 0:
        vid_fps = 30.0  # Fallback
    frame_delay_sec = 1.0 / vid_fps
    is_webcam = args.video == "0"

    win = "yolo26 inference (q to quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    start_wall_time = time.time()
    frame_count = 0

    while True:
        loop_start_time = time.time()

        ok, frame = cap.read()
        if not ok:
            break

        frame_count += 1

        results = model.predict(
            frame,
            conf=args.conf,
            imgsz=args.imgsz,
            device=args.device,
            verbose=False,
        )
        annotated = results[0].plot()

        # Extract YOLO inference latency (provided by Ultralytics)
        latency_ms = results[0].speed.get("inference", 0.0)

        # Calculate processing FPS (excluding artificial delays)
        work_time = time.time() - loop_start_time
        fps = 1.0 / max(work_time, 1e-6)

        # Draw FPS
        cv2.putText(
            annotated,
            f"{fps:5.1f} FPS",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        # Draw Latency
        cv2.putText(
            annotated,
            f"{latency_ms:.1f} ms Latency",
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        cv2.imshow(win, annotated)

        # Realtime Sync Logic
        expected_time = start_wall_time + (frame_count * frame_delay_sec)
        current_time = time.time()

        if not is_webcam:
            if current_time < expected_time:
                # Inference is faster than realtime video, wait to slow down
                sleep_ms = int((expected_time - current_time) * 1000)
                if cv2.waitKey(max(1, sleep_ms)) & 0xFF == ord("q"):
                    break
            else:
                # Inference is slower than realtime video, skip frames to catch up
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                frames_to_skip = int((current_time - expected_time) / frame_delay_sec)
                for _ in range(frames_to_skip):
                    if cap.grab():
                        frame_count += 1
                    else:
                        break
        else:
            # Webcams naturally run in realtime
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
