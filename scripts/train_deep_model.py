"""
Deep YOLOv8 Training Engine - Maximum Accuracy & Deep Feature Extraction
========================================================================
- Model Architecture: YOLOv8 Medium (YOLOv8m) - 25.9M Parameters
- Dataset: 12 Classes (All Non-Degradable + Degradable Banana Peels + Cups)
- Optimizer: AdamW with Cosine Learning Rate Annealing
- Hardware: Auto-tuned for NVIDIA GeForce RTX 3050 GPU (FP16 AMP, Batch 8)
- Deep Augmentation: Mosaic, Mixup, Color Jitter, Scaling & Affine transforms
"""

import os
import sys
import shutil
import time
import argparse
from pathlib import Path
import torch
from ultralytics import YOLO


def play_completion_alert():
    try:
        import winsound
        for freq in [800, 1000, 1200, 1600]:
            winsound.Beep(freq, 150)
            time.sleep(0.05)
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(description="Deep YOLOv8 Trash Detection Training Engine")
    parser.add_argument("--epochs", type=int, default=25, help="Deep training epochs (default: 25)")
    parser.add_argument("--batch", type=int, default=8, help="Batch size (default: 8, optimal for 6GB GPU)")
    parser.add_argument("--imgsz", type=int, default=640, help="Input resolution (default: 640)")
    parser.add_argument("--patience", type=int, default=10, help="Early stopping patience (default: 10)")
    args = parser.parse_args()

    SCRIPT_DIR = Path(__file__).resolve().parent
    PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name in ("src", "scripts", "tests") else SCRIPT_DIR
    ROOT_DIR = PROJECT_ROOT
    DATA_YAML = ROOT_DIR / "dataset" / "data.yaml" if (ROOT_DIR / "dataset" / "data.yaml").exists() else ROOT_DIR / "data.yaml"
    MODELS_DIR = ROOT_DIR / "models"
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    WEIGHTS_DIR = ROOT_DIR / "weights" if (ROOT_DIR / "weights").exists() else MODELS_DIR
    RUNS_DIR = ROOT_DIR / "runs" / "detect"
    EXPERIMENT = "deep_trash_yolov8m"

    if not DATA_YAML.exists():
        raise FileNotFoundError(f"data.yaml not found at: {DATA_YAML}")

    # Hardware Detection
    if torch.cuda.is_available():
        gpu_name = torch.cuda.get_device_name(0)
        vram_gb = torch.cuda.get_device_properties(0).total_memory / (1024 ** 3)
        device = "0"
        print(f"\n[GPU DETECTED] {gpu_name} ({vram_gb:.1f} GB VRAM) - CUDA Enabled")
        batch_size = args.batch
        workers = 2
    else:
        device = "cpu"
        batch_size = 4
        workers = 0
        print("\n[WARNING] GPU not detected. Running on CPU.")

    # Starting Weights Selection (fine-tune on existing best or fresh yolov8m)
    candidate_weights = [
        MODELS_DIR / "best_trash_yolov8m.pt",
        WEIGHTS_DIR / "best_trash_yolov8m.pt",
        MODELS_DIR / "roboflow_banana_weights.pt",
        ROOT_DIR / "yolov8m.pt",
        "yolov8m.pt"
    ]
    base_model = "yolov8m.pt"
    for cand in candidate_weights:
        if cand.exists():
            base_model = str(cand)
            break

    print("=" * 75)
    print("      🚀 DEEP NEURAL NETWORK TRAINING: 12-CLASS SMART WASTE AI 🚀")
    print("=" * 75)
    print(f"  • Architecture    : YOLOv8m (Medium Backbone - 25.9M Parameters)")
    print(f"  • Base Weights    : {base_model}")
    print(f"  • Dataset Config  : {DATA_YAML}")
    print(f"  • Deep Epochs     : {args.epochs}")
    print(f"  • Batch Size      : {batch_size} (Workers: {workers})")
    print(f"  • Resolution      : {args.imgsz}x{args.imgsz}")
    print(f"  • Optimizer       : AdamW (Cosine Annealing LR)")
    print(f"  • Precision       : FP16 Mixed Precision (AMP Enabled)")
    print("=" * 75 + "\n")

    model = YOLO(base_model)

    print("[Phase 1/3] Launching Deep Neural Network Optimization...\n")
    results = model.train(
        data=str(DATA_YAML),
        epochs=args.epochs,
        batch=batch_size,
        imgsz=args.imgsz,
        device=device,
        workers=workers,
        project=str(RUNS_DIR),
        name=EXPERIMENT,
        exist_ok=True,
        save=True,
        save_period=5,
        # Optimizer & Learning Rate Dynamics
        optimizer="AdamW",
        lr0=0.001,
        lrf=0.01,
        cos_lr=True,
        warmup_epochs=3,
        weight_decay=0.0005,
        patience=args.patience,
        # Loss Weighting for precise bounding boxes
        box=7.5,
        cls=1.2,
        dfl=1.5,
        # Deep Data Augmentation
        mosaic=1.0,
        mixup=0.15,
        copy_paste=0.20,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=12.0,
        translate=0.1,
        scale=0.3,
        fliplr=0.5,
        close_mosaic=10,
        verbose=True,
        plots=True,
        amp=True
    )

    print("\n[Phase 2/3] Evaluating Deep Model Metrics on Validation Set...\n")
    best_weights_path = Path(results.save_dir) / "weights" / "best.pt"
    if not best_weights_path.exists():
        best_weights_path = Path(results.save_dir) / "weights" / "last.pt"

    val_model = YOLO(str(best_weights_path))
    val_results = val_model.val(
        data=str(DATA_YAML),
        split="val",
        imgsz=args.imgsz,
        device=device,
        plots=True
    )

    print("\n[Phase 3/3] Deploying Best Weights to Active Project Controllers...\n")
    active_target = MODELS_DIR / "best_trash_yolov8m.pt"
    degradable_target = MODELS_DIR / "best_degradable_yolov8.pt"
    backup_file = MODELS_DIR / "best_trash_yolov8m.pt.bak"

    if best_weights_path.exists():
        if active_target.exists():
            shutil.copy2(active_target, backup_file)
            print(f"  [Backup] Preserved previous weights at: {backup_file.name}")
        shutil.copy2(best_weights_path, active_target)
        shutil.copy2(best_weights_path, degradable_target)
        if WEIGHTS_DIR != MODELS_DIR:
            try:
                shutil.copy2(best_weights_path, WEIGHTS_DIR / "best_trash_yolov8m.pt")
                shutil.copy2(best_weights_path, WEIGHTS_DIR / "best_degradable_yolov8.pt")
            except Exception: pass
        print(f"  [Deployed] Active Controller Weights updated: {active_target}")
        print(f"  [Deployed] Degradable Sorter Weights updated: {degradable_target}")
    else:
        print(f"  [Warning] Could not locate best.pt at {best_weights_path}")

    print("\n" + "=" * 75)
    print("  🏆 DEEP TRAINING COMPLETE - BENCHMARK METRICS SUMMARY 🏆")
    print("=" * 75)
    try:
        box = val_results.box
        print(f"  • Mean Average Precision (mAP@50)     : {box.map50 * 100:.2f}%")
        print(f"  • Overall Precision (mAP@50-95)       : {box.map * 100:.2f}%")
        print(f"  • Class Precision                     : {box.mp * 100:.2f}%")
        print(f"  • Class Recall                        : {box.mr * 100:.2f}%\n")
        print("  --- PER-CLASS ACCURACY BREAKDOWN ---")
        names_dict = val_results.names
        for i, (cls_idx, name) in enumerate(names_dict.items()):
            if i < len(box.maps):
                print(f"  [{cls_idx:02d}] {name:<22} : {box.maps[i]*100:.1f}%")
    except Exception as e:
        print(f"  (Metrics summary error: {e})")
    print(f"\n  • Saved Model File                    : {active_target}")
    print("=" * 75 + "\n")

    play_completion_alert()


if __name__ == "__main__":
    main()
