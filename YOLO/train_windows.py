#!/usr/bin/env python3
"""
Train YOLO26n on a Roboflow-exported dataset -- on Windows with an NVIDIA GPU
(e.g. an RTX 4060 laptop).

One-time setup (a venv keeps torch/ultralytics out of your system Python):
    py -m venv .venv
    .venv\\Scripts\\activate
    pip install -U ultralytics

IMPORTANT -- get the CUDA build of torch, or the 4060 sits idle:
    On Windows, a plain "pip install ultralytics" often pulls the CPU-only
    torch. Install the CUDA build explicitly (check the current command at
    https://pytorch.org -- pick your CUDA version; cu124 works on recent GPUs):
        pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
    Verify it sees the GPU:
        python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
    That should print  True  and your card name (e.g. NVIDIA GeForce RTX 4060 ...).

Then train:
    python train.py --data C:\\path\\to\\roboflow\\data.yaml

Your Roboflow download (export in a YOLO / PyTorch format) is a folder holding
data.yaml plus train/ valid/ (and maybe test/) subfolders. Point --data at that
data.yaml -- it lists the image paths and your class names, which is all YOLO needs.

When it finishes, the trained weights land at:
    runs\\detect\\<name>\\weights\\best.pt
which is exactly the file the client wants:
    python rov_client.py --weights runs\\detect\\<name>\\weights\\best.pt
"""

import argparse
import os


def pick_device():
    """cuda (NVIDIA GPU) if available, else cpu."""
    import torch

    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def main():
    ap = argparse.ArgumentParser(
        description="Train YOLO26n on a Roboflow dataset (Windows + NVIDIA GPU)."
    )
    ap.add_argument("--data", required=True, help="path to the Roboflow data.yaml")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)  # matches the 640x480 pipeline
    ap.add_argument(
        "--batch", type=int, default=16, help="lower to 8 or 4 if you run out of VRAM"
    )
    ap.add_argument(
        "--model", default="yolo26n.pt", help="downloads automatically on first use"
    )
    ap.add_argument(
        "--name", default="rov_yolo26n", help="run folder name under runs/detect/"
    )
    args = ap.parse_args()

    from ultralytics import YOLO

    device = pick_device()
    print(f"[train] device={device}  model={args.model}  data={args.data}")
    if device == "cpu":
        print(
            "[train] note: no CUDA GPU detected -> training on CPU (much slower). "
            "If you have a 4060, install the CUDA build of torch (see the header)."
        )
    else:
        import torch

        print(f"[train] GPU: {torch.cuda.get_device_name(0)}")

    # Start from the COCO-pretrained nano model and fine-tune on your classes.
    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        name=args.name,
        verbose=False,
    )

    best = os.path.join("runs", "detect", args.name, "weights", "best.pt")
    print(f"\n[train] done -> {best}")
    print(f"[train] run it with:  python rov_client.py --weights {best}")


if __name__ == "__main__":
    main()
