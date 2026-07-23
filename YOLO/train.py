#!/usr/bin/env python3
"""
Train YOLO26n on a Roboflow-exported dataset -- cross-platform.

Auto-picks the fastest device available:
    cuda  -- NVIDIA GPU (Linux / Windows, e.g. RTX 2060 / 4060 laptop)
    mps   -- Apple Silicon GPU (macOS)
    cpu   -- fallback (works everywhere, just slower)

One-time setup (a venv keeps torch/ultralytics out of your system Python):
    macOS / Linux:
        python3 -m venv .venv
        source .venv/bin/activate
        pip install -U ultralytics
    Windows:
        py -m venv .venv
        .venv\\Scripts\\activate
        pip install -U ultralytics

NVIDIA GPU note:
    Linux -- the standard torch wheel bundles CUDA, so the install above usually
      just works. You need a working driver (check with:  nvidia-smi).
    Windows -- a plain install often pulls the CPU-only torch. If the GPU isn't
      used, install the CUDA build explicitly (pick your CUDA version at
      https://pytorch.org; cu124 works on recent drivers):
        pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
    Verify either one sees the GPU:
        python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
    (macOS uses MPS automatically -- nothing extra to install.)

Then train:
    python train.py --data /path/to/roboflow/data.yaml        (macOS / Linux)
    python train.py --data C:\\path\\to\\roboflow\\data.yaml    (Windows)

Your Roboflow download (export in a YOLO / PyTorch format) is a folder holding
data.yaml plus train/ valid/ (and maybe test/) subfolders. Point --data at that
data.yaml -- it lists the image paths and your class names, which is all YOLO needs.

When it finishes, the trained weights land at:
    runs/detect/<name>/weights/best.pt
which is exactly the file the client wants:
    python rov_client.py --weights runs/detect/<name>/weights/best.pt
"""

import argparse
import os

# Apple Silicon only: let any op MPS doesn't implement fall back to CPU instead
# of erroring out mid-training. Harmless (ignored) on Linux / Windows.
os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")


def pick_device():
    """Fastest available: cuda (NVIDIA) -> mps (Apple Silicon) -> cpu."""
    import torch

    if torch.cuda.is_available():
        return "cuda"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def main():
    ap = argparse.ArgumentParser(
        description="Train YOLO26n on a Roboflow dataset (cuda/mps/cpu auto)."
    )
    ap.add_argument("--data", required=True, help="path to the Roboflow data.yaml")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)  # matches the 640x480 pipeline
    ap.add_argument(
        "--batch", type=int, default=16, help="lower to 8 or 4 if you run out of memory"
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
            "[train] note: no GPU detected -> training on CPU (much slower). "
            "NVIDIA: check 'nvidia-smi' and that torch is the CUDA build (see header)."
        )
    elif device == "cuda":
        import torch

        print(f"[train] GPU: {torch.cuda.get_device_name(0)}")
    else:  # mps
        print("[train] GPU: Apple Silicon (MPS)")

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
    print(
        f"[train] run it with:  python rov-client-with-state-machine.py --weights {best}"
    )


if __name__ == "__main__":
    main()
