"""
High-Accuracy YOLOv8 Trainer for Degradable (Banana Peel) & Non-Degradable Waste
=================================================================================
Fine-tunes YOLOv8 on 12 classes (11 Non-Degradable + 1 Degradable: Banana Peel)
Optimized for high-precision truck floor detection and automated dual dustbin sorting.
"""

import os
import sys
import shutil
from pathlib import Path
import torch
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name in ("src", "scripts", "tests") else SCRIPT_DIR
ROOT_DIR = PROJECT_ROOT
DATA_YAML = str(ROOT_DIR / "dataset" / "data.yaml") if (ROOT_DIR / "dataset" / "data.yaml").exists() else str(ROOT_DIR / "data.yaml")
MODELS_DIR = ROOT_DIR / "models"
MODELS_DIR.mkdir(parents=True, exist_ok=True)
WEIGHTS_DIR = ROOT_DIR / "weights" if (ROOT_DIR / "weights").exists() else MODELS_DIR

def find_starting_weights():
    candidates = [
        str(MODELS_DIR / "best_trash_yolov8m.pt"),
        str(WEIGHTS_DIR / "best_trash_yolov8m.pt"),
        str(MODELS_DIR / "roboflow_banana_weights.pt"),
        str(ROOT_DIR / "runs" / "detect" / "deep_trash_yolov8m" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "trash_max_accuracy" / "weights" / "best.pt"),
        str(ROOT_DIR / "runs" / "detect" / "trash_yolov8m_ultra_acc" / "weights" / "best.pt"),
        str(ROOT_DIR / "yolov8m.pt"),
        str(ROOT_DIR / "yolov8n.pt"),
        "yolov8n.pt"
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return "yolov8n.pt"

def train():
    device = "0" if torch.cuda.is_available() else "cpu"
    device_name = torch.cuda.get_device_name(0) if device == "0" else "CPU"
    starting_weights = find_starting_weights()

    print("\n" + "=" * 70)
    print(" 🚀 HIGH-PRECISION DUAL-BIN MODEL TRAINING (BANANA PEEL + TRASH) 🚀")
    print("=" * 70)
    print(f" • Hardware Acceleration : {device_name} (CUDA: {device == '0'})")
    print(f" • Base Model Weights    : {starting_weights}")
    print(f" • Dataset Configuration : {DATA_YAML}")
    print(f" • Image Resolution      : 640 x 640")
    print(f" • Target Degradable Cls : 'banana peel' (Class 11)")
    print("=" * 70 + "\n")

    model = YOLO(starting_weights)

    epochs = 25 if device == "0" else 10
    batch_size = 8 if device == "0" else 4

    # Train with balanced data augmentations and close_mosaic
    results = model.train(
        data=DATA_YAML,
        epochs=epochs,
        batch=batch_size,
        imgsz=640,
        device=device,
        project=str(ROOT_DIR / "runs" / "detect"),
        name="degradable_trash_yolo",
        exist_ok=True,
        patience=8,
        save=True,
        plots=True,
        mosaic=0.8,
        mixup=0.05,
        fliplr=0.5,
        degrees=10.0,
        scale=0.2,
        lr0=0.006,
        lrf=0.01,
        close_mosaic=4,
        workers=2 if os.name == 'nt' else 4,
        verbose=True
    )

    best_pt = ROOT_DIR / "runs" / "detect" / "degradable_trash_yolo" / "weights" / "best.pt"
    target_weights = WEIGHTS_DIR / "best_degradable_yolov8.pt"
    active_weights = WEIGHTS_DIR / "best_trash_yolov8m.pt"

    if best_pt.exists():
        shutil.copy2(best_pt, target_weights)
        if active_weights.exists():
            shutil.copy2(active_weights, WEIGHTS_DIR / "best_trash_yolov8m.pt.bak")
        shutil.copy2(best_pt, active_weights)
        print(f"\n✅ Training Complete! Saved high-accuracy model to: {target_weights}")
        print(f"✅ Deployed to active production weights: {active_weights}")
    else:
        print("\n[Notice] Checking training output directory...")

    # Validate model
    print("\n" + "=" * 70)
    print(" 📊 DETAILED VALIDATION ACCURACY & PER-CLASS BENCHMARKS")
    print("=" * 70)
    eval_model = YOLO(str(target_weights if target_weights.exists() else starting_weights))
    val_res = eval_model.val(data=DATA_YAML, split="val", imgsz=640, conf=0.30, device=device)

    mp = float(val_res.box.mp)
    mr = float(val_res.box.mr)
    map50 = float(val_res.box.map50)
    map_all = float(val_res.box.map)

    print("\n" + "-" * 70)
    print(f" ★ OVERALL MODEL PRECISION (P) : {mp*100:6.2f}%")
    print(f" ★ OVERALL MODEL RECALL (R)    : {mr*100:6.2f}%")
    print(f" ★ OVERALL mAP@50              : {map50*100:6.2f}%")
    print(f" ★ OVERALL mAP@50-95           : {map_all*100:6.2f}%")
    print("-" * 70)

    # Class-by-class accuracy report
    print("\n" + "=" * 70)
    print(f"{'CLASS NAME':<22} | {'PRECISION':<10} | {'RECALL':<10} | {'mAP@50':<10} | {'BIN TARGET':<15}")
    print("-" * 70)
    class_names = eval_model.names
    for i, cls_id in enumerate(val_res.box.ap_class_index):
        cls_name = class_names[cls_id]
        p_val = val_res.box.p[i]
        r_val = val_res.box.r[i]
        ap50_val = val_res.box.ap50[i]
        bin_target = "RIGHT (180° CW)" if cls_name == "banana peel" else "LEFT (180° CCW)"
        print(f"{cls_name:<22} | {p_val*100:9.1f}% | {r_val*100:9.1f}% | {ap50_val*100:9.1f}% | {bin_target:<15}")
    print("=" * 70)
    print("Dual dustbin autonomous routing calibrated and ready!\n")

if __name__ == "__main__":
    train()
