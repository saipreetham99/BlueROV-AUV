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
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default=None)  # e.g. "0" for GPU, "cpu"; None = auto
    args = ap.parse_args()

    model = YOLO(args.weights)

    source = 0 if args.video == "0" else args.video
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"could not open: {args.video}")

    win = "yolo26 inference (q to quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    prev = time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        results = model.predict(
            frame,
            conf=args.conf,
            imgsz=args.imgsz,
            device=args.device,
            verbose=False,
        )
        annotated = results[0].plot()  # draws boxes + labels onto the frame

        now = time.time()
        fps = 1.0 / max(now - prev, 1e-6)
        prev = now
        cv2.putText(
            annotated,
            f"{fps:5.1f} FPS",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        cv2.imshow(win, annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
