"""
YOLOv8 Trash Detection - Unified Pipeline (Train, Test/Predict, Validate, Webcam)
================================================================================
Classes (10):
  0: battery
  1: can
  2: cardboard
  3: drink carton
  4: glass bottle
  5: paper
  6: plastic bag
  7: plastic bottle
  8: plastic bottle cap
  9: pop tab

Usage examples:
  1. Train / Finetune:
     python main.py --mode train --epochs 25 --batch 16 --model yolov8n.pt

  2. Test on a single image or folder:
     python main.py --mode predict --source valid/images/001.jpg --weights runs/detect/trash_finetune/weights/best.pt

  3. Real-time Webcam detection:
     python main.py --mode webcam --weights runs/detect/trash_finetune/weights/best.pt

  4. Validate model performance (mAP, Precision, Recall):
     python main.py --mode val --weights runs/detect/trash_finetune/weights/best.pt
"""

import os
import sys
import argparse
from pathlib import Path
import cv2
import torch
from ultralytics import YOLO

# Project root directory
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name == "src" else SCRIPT_DIR
ROOT_DIR = PROJECT_ROOT

DATA_YAML = str(ROOT_DIR / "dataset" / "data.yaml") if (ROOT_DIR / "dataset" / "data.yaml").exists() else str(ROOT_DIR / "data.yaml")
DEFAULT_PROJECT = str(ROOT_DIR / "runs" / "detect")
DEFAULT_WEIGHTS = str(ROOT_DIR / "models" / "best_trash_yolov8m.pt")

def detect_device() -> str:
    """Check for CUDA GPU availability and return '0' or 'cpu'."""
    if torch.cuda.is_available():
        gpu = torch.cuda.get_device_name(0)
        print(f"[Device] Using GPU: {gpu}")
        return "0"
    print("[Device] Using CPU")
    return "cpu"

def find_best_weights(preferred_weights: str = None) -> str:
    """Find the best available model weights."""
    candidates = [
        preferred_weights,
        str(ROOT_DIR / "models" / "best_trash_yolov8m.pt"),
        str(ROOT_DIR / "weights" / "best_trash_yolov8m.pt"),
        str(ROOT_DIR / "models" / "best_degradable_yolov8.pt"),
        str(ROOT_DIR / "weights" / "best_degradable_yolov8.pt"),
        str(ROOT_DIR / "runs" / "detect" / "deep_trash_yolov8m" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "trash_max_accuracy" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "trash_yolov8m_ultra_acc" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "trash_finetune" / "weights" / "best.pt"),
        str(ROOT_DIR / "models" / "roboflow_banana_weights.pt"),
        str(ROOT_DIR / "models" / "yolov8n.pt"),
        str(ROOT_DIR / "best.pt"),
        "yolov8m.pt",
        "yolov8n.pt"
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return "yolov8n.pt"

# ============================================================================
# 1. Training Mode
# ============================================================================
def run_train(args):
    """Run YOLOv8 training/fine-tuning on the dataset."""
    device = args.device or detect_device()
    print(f"\n[Training] Initializing base model: {args.model}")
    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=device,
        project=args.project,
        name=args.name,
        lr0=args.lr0,
        workers=args.workers
    )
    return results

# ============================================================================
# 2. Prediction / Inference Mode
# ============================================================================
def run_predict(args):
    """Run inference on an image, video, or directory of images."""
    weights = find_best_weights(args.weights)
    print(f"\n[Inference] Loading model: {weights}")
    model = YOLO(weights)

    source = args.source
    if not source:
        # Pick a sample image from dataset/valid/images or valid/images
        val_images_dir = ROOT_DIR / "dataset" / "valid" / "images"
        if not val_images_dir.exists():
            val_images_dir = ROOT_DIR / "valid" / "images"
        if val_images_dir.exists():
            images = list(val_images_dir.glob("*.jpg")) + list(val_images_dir.glob("*.png"))
            if images:
                source = str(images[0])
                print(f"[Inference] No source specified, using sample validation image: {source}")
        if not source:
            print("[Error] Please specify --source <path_to_image_or_folder>")
            return

    print(f"[Inference] Running detection on: {source}")
    device = args.device or detect_device()
    results = model.predict(
        source=source,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        device=device,
        save=True,
        save_txt=False,
        project=args.project,
        name=args.predict_name
    )

    print("\n--- Detection Results ---")
    for r in results:
        boxes = r.boxes
        print(f"File: {r.path} | Detected Objects: {len(boxes)}")
        for box in boxes:
            cls_id = int(box.cls[0])
            cls_name = model.names[cls_id]
            conf = float(box.conf[0])
            xyxy = [round(float(coord), 2) for coord in box.xyxy[0].tolist()]
            print(f"  - {cls_name} ({conf*100:.1f}%) at {xyxy}")

    save_dir = results[0].save_dir if results else "runs/detect"
    print(f"\n[Inference] Output saved to: {save_dir}")

# ============================================================================
# 3. Real-time Webcam Mode
# ============================================================================
def run_webcam(args):
    """Run real-time detection on webcam stream."""
    import time
    weights = find_best_weights(args.weights)
    print(f"\n[Webcam] Loading model: {weights}")
    model = YOLO(weights)

    camera_id = int(args.camera_id)
    print(f"[Webcam] Opening camera (ID: {camera_id})...")
    
    # Use DirectShow backend on Windows for faster initialization
    if os.name == 'nt':
        cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(camera_id)

    # Fallback to standard capture if DirectShow didn't open
    if not cap.isOpened():
        cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print(f"[Error] Could not open camera with ID {camera_id}. Check camera permissions or try --camera_id 1.")
        return

    # Set camera resolution (optional: 1280x720 or 640x480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    device = args.device or detect_device()
    current_conf = args.conf

    print("\n" + "=" * 55)
    print(" Live Webcam Trash Detection Active!")
    print("=" * 55)
    print(" Controls:")
    print("   [q] or [ESC] : Exit webcam")
    print("   [+] or [=]   : Increase confidence threshold (+0.05)")
    print("   [-]          : Decrease confidence threshold (-0.05)")
    print("   [s]          : Save snapshot image to disk")
    print("=" * 55 + "\n")

    prev_time = time.time()
    fps = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[Webcam] Failed to grab frame. Exiting...")
                break

            # Calculate FPS
            curr_time = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(curr_time - prev_time, 1e-5))
            prev_time = curr_time

            # Model inference
            results = model.predict(frame, conf=current_conf, imgsz=args.imgsz, device=device, verbose=False)
            annotated_frame = results[0].plot()

            # Draw HUD info overlay
            info_text = f"FPS: {fps:.1f} | Conf: {current_conf:.2f} | Model: {Path(weights).name}"
            cv2.putText(annotated_frame, info_text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
            cv2.putText(annotated_frame, info_text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("YOLOv8 Trash Detection - Live Webcam", annotated_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                break
            elif key == ord('+') or key == ord('='):
                current_conf = min(0.95, round(current_conf + 0.05, 2))
                print(f"[Controls] Confidence threshold set to: {current_conf:.2f}")
            elif key == ord('-') or key == ord('_'):
                current_conf = max(0.10, round(current_conf - 0.05, 2))
                print(f"[Controls] Confidence threshold set to: {current_conf:.2f}")
            elif key == ord('s') or key == ord('S'):
                snap_name = f"snapshot_{int(time.time())}.jpg"
                cv2.imwrite(snap_name, annotated_frame)
                print(f"[Snapshot] Saved to {snap_name}")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[Webcam] Stream closed.")

# ============================================================================
# 4. Validation Mode
# ============================================================================
def run_val(args):
    """Run validation benchmark on validation set."""
    weights = find_best_weights(args.weights)
    print(f"\n[Validation] Loading model: {weights}")
    model = YOLO(weights)
    device = args.device or detect_device()

    metrics = model.val(
        data=args.data,
        split="val",
        imgsz=args.imgsz,
        device=device,
        project=args.project,
        name="val_evaluation"
    )

    print("\n" + "=" * 50)
    print(" Validation Metrics Summary")
    print("=" * 50)
    print(f" mAP50        : {metrics.box.map50:.4f}")
    print(f" mAP50-95     : {metrics.box.map:.4f}")
    print(f" Precision    : {metrics.box.mp:.4f}")
    print(f" Recall       : {metrics.box.mr:.4f}")
    print("=" * 50)

# ============================================================================
# Main Entry Point
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="YOLOv8 Trash Detection - Unified Pipeline")
    parser.add_argument("--mode", type=str, default="train", choices=["train", "predict", "webcam", "val"],
                        help="Operating mode: train, predict, webcam, or val")

    # Common parameters
    parser.add_argument("--weights", type=str, default=None, help="Weights path for inference or validation")
    parser.add_argument("--device", type=str, default=None, help="Device ('0', 'cpu', or leave empty for auto)")
    parser.add_argument("--imgsz", type=int, default=640, help="Image resolution")
    parser.add_argument("--project", type=str, default=DEFAULT_PROJECT, help="Output directory")

    # Training parameters
    parser.add_argument("--data", type=str, default=DATA_YAML, help="Path to data.yaml")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Base model for training (e.g. yolov8n.pt)")
    parser.add_argument("--epochs", type=int, default=60, help="Number of training epochs")
    parser.add_argument("--batch", type=int, default=16, help="Training batch size")
    parser.add_argument("--name", type=str, default="trash_finetune", help="Experiment name")
    parser.add_argument("--workers", type=int, default=2, help="DataLoader workers")
    parser.add_argument("--lr0", type=float, default=0.01, help="Initial learning rate")

    # Inference / Webcam parameters
    parser.add_argument("--source", type=str, default=None, help="Source path (image, video, folder) for prediction")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold")
    parser.add_argument("--camera_id", type=int, default=0, help="Webcam device ID (default: 0)")
    parser.add_argument("--predict_name", type=str, default="predict_results", help="Prediction output folder name")

    args = parser.parse_args()

    if args.mode == "train":
        run_train(args)
    elif args.mode == "predict":
        run_predict(args)
    elif args.mode == "webcam":
        run_webcam(args)
    elif args.mode == "val":
        run_val(args)

if __name__ == "__main__":
    main()
