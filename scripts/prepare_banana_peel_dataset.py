"""
Banana Peel & Degradable Waste Dataset Builder
==============================================
1. Updates data.yaml to include 'banana peel' (class 11, total nc=12).
2. Fixes path in data.yaml to be portable (current directory).
3. Downloads & curates banana peel images from open datasets / web sources.
4. Auto-labels banana peels using YOLO pretrained vision (COCO class 46: banana/peel).
5. Supports importing any downloaded Roboflow dataset zip or folder.
6. Applies dataset augmentations to ensure robust detection on truck floor.
"""

import os
import sys
import glob
import shutil
import urllib.request
from pathlib import Path
import yaml
import cv2
import numpy as np

import zipfile
import tempfile

ROOT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT_DIR.parent if ROOT_DIR.name in ("src", "scripts", "tests") else ROOT_DIR

if (PROJECT_ROOT / "dataset" / "data.yaml").exists():
    DATASET_DIR = PROJECT_ROOT / "dataset"
elif (ROOT_DIR / "dataset" / "data.yaml").exists():
    DATASET_DIR = ROOT_DIR / "dataset"
elif (ROOT_DIR / "data.yaml").exists():
    DATASET_DIR = ROOT_DIR
else:
    DATASET_DIR = PROJECT_ROOT

DATA_YAML = DATASET_DIR / "data.yaml"
TRAIN_IMG = DATASET_DIR / "train" / "images"
TRAIN_LBL = DATASET_DIR / "train" / "labels"
VAL_IMG   = DATASET_DIR / "valid" / "images"
VAL_LBL   = DATASET_DIR / "valid" / "labels"

TRAIN_IMG.mkdir(parents=True, exist_ok=True)
TRAIN_LBL.mkdir(parents=True, exist_ok=True)
VAL_IMG.mkdir(parents=True, exist_ok=True)
VAL_LBL.mkdir(parents=True, exist_ok=True)

# Curated open banana peel and decaying banana waste image URLs
CURATED_BANANA_PEEL_URLS = [
    # Banana peels on ground / floor / table / waste bins
    "https://images.unsplash.com/photo-1528825871115-3581a5387919?w=800",
    "https://images.unsplash.com/photo-1571771894821-ce9b6c11b08e?w=800",
    "https://images.unsplash.com/photo-1566393028639-d108a42c46a7?w=800",
    "https://images.unsplash.com/photo-1481349518771-20055b2a7b24?w=800",
    "https://images.unsplash.com/photo-1587132137056-bfbf0166836e?w=800",
    "https://images.unsplash.com/photo-1594489428504-5c0c480a15fd?w=800",
    "https://images.unsplash.com/photo-1603052875302-d376b7c0638a?w=800",
    "https://images.unsplash.com/photo-1550258987-190a2d41a8ba?w=800",
    "https://images.unsplash.com/photo-1543218024-57a70143c369?w=800",
    "https://images.unsplash.com/photo-1588776814546-1ffcf47267a5?w=800",
    "https://images.unsplash.com/photo-1525607551316-4a8e16d1f9ba?w=800",
    "https://images.unsplash.com/photo-1574856344991-aaa31b6f4ce3?w=800",
    "https://images.unsplash.com/photo-1610832958506-aa56368176cf?w=800",
    "https://images.unsplash.com/photo-1518843875459-f738682238a6?w=800",
    "https://images.unsplash.com/photo-1563861826100-9cb868fdbe1c?w=800",
    "https://images.unsplash.com/photo-1596040033229-a9821ebd058d?w=800",
    "https://images.unsplash.com/photo-1587486913049-53fc88980cfc?w=800",
    "https://images.unsplash.com/photo-1506806732259-39c2d0268443?w=800",
    "https://images.unsplash.com/photo-1546548970-71785318a17b?w=800",
    "https://images.unsplash.com/photo-1576045057995-568f588f82fb?w=800"
]

def update_data_yaml():
    """Ensure data.yaml is portable and has 'banana peel' as class 11."""
    with open(DATA_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # Use portable relative path
    data["path"] = "."
    data["train"] = "train/images"
    data["val"] = "valid/images"
    data["test"] = "valid/images"

    classes = data.get("names", [])
    if "banana peel" not in classes and "banana_peel" not in classes:
        classes.append("banana peel")
        data["names"] = classes
        data["nc"] = len(classes)
        with open(DATA_YAML, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        root_yaml = PROJECT_ROOT / "data.yaml"
        if root_yaml.exists() and root_yaml.resolve() != Path(DATA_YAML).resolve():
            try:
                with open(root_yaml, "r", encoding="utf-8") as f:
                    r_data = yaml.safe_load(f)
                r_data["nc"] = len(classes)
                r_data["names"] = classes
                with open(root_yaml, "w", encoding="utf-8") as f:
                    yaml.safe_dump(r_data, f, sort_keys=False)
            except Exception: pass
        print(f"[OK] data.yaml updated: Added 'banana peel' (Class index: {len(classes)-1})")
    else:
        banana_idx = classes.index("banana peel") if "banana peel" in classes else classes.index("banana_peel")
        print(f"[OK] data.yaml already contains banana peel (Class index: {banana_idx})")

    return classes

def import_roboflow_folder(rf_path: str):
    """Import and merge any downloaded Roboflow dataset folder or ZIP into train/valid."""
    rf_input = Path(rf_path)
    if not rf_input.exists():
        print(f"[Warning] Roboflow path does not exist: {rf_path}")
        return 0

    temp_extract_dir = None
    if rf_input.is_file() and rf_input.suffix.lower() == ".zip":
        print(f"\n[Roboflow Import] Extracting ZIP archive: {rf_input.name}...")
        temp_extract_dir = tempfile.mkdtemp(prefix="rf_banana_")
        with zipfile.ZipFile(rf_input, 'r') as zip_ref:
            zip_ref.extractall(temp_extract_dir)
        rf_dir = Path(temp_extract_dir)
    else:
        rf_dir = rf_input

    print(f"\n[Roboflow Import] Scanning {rf_dir} for YOLO images & labels...")
    imported = 0
    banana_class_id = 11

    # First check standard Roboflow splits: train, valid, test
    found_splits = False
    for split in ["train", "valid", "test"]:
        split_dir = rf_dir / split
        if not split_dir.exists():
            # Check nested under a parent directory
            candidates = list(rf_dir.glob(f"*/{split}"))
            if candidates:
                split_dir = candidates[0]
            else:
                continue

        found_splits = True
        dest_img_dir = VAL_IMG if split == "valid" else TRAIN_IMG
        dest_lbl_dir = VAL_LBL if split == "valid" else TRAIN_LBL

        img_files = []
        for ext in ["*.jpg", "*.jpeg", "*.png"]:
            if (split_dir / "images").exists():
                img_files.extend(list((split_dir / "images").glob(ext)))
            else:
                img_files.extend(list(split_dir.glob(ext)))

        for img_p in img_files:
            lbl_p = (split_dir / "labels" / f"{img_p.stem}.txt")
            if not lbl_p.exists():
                lbl_p = split_dir / f"{img_p.stem}.txt"

            target_img_name = f"rf_banana_{img_p.name}"
            target_lbl_name = f"rf_banana_{img_p.stem}.txt"

            # Copy image
            shutil.copy2(img_p, dest_img_dir / target_img_name)

            # Rewrite labels to map to class 11 (banana peel)
            if lbl_p.exists():
                with open(lbl_p, "r", encoding="utf-8") as lf:
                    lines = lf.readlines()
                new_lines = []
                for line in lines:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        parts[0] = str(banana_class_id)
                        new_lines.append(" ".join(parts) + "\n")
                with open(dest_lbl_dir / target_lbl_name, "w", encoding="utf-8") as nlf:
                    nlf.writelines(new_lines)
            else:
                # Default centered box if no label file
                with open(dest_lbl_dir / target_lbl_name, "w", encoding="utf-8") as nlf:
                    nlf.write(f"{banana_class_id} 0.500000 0.500000 0.600000 0.600000\n")
            imported += 1

    # If no standard splits, import all images directly (80% train, 20% val)
    if not found_splits:
        all_imgs = []
        for ext in ["*.jpg", "*.jpeg", "*.png"]:
            all_imgs.extend(list(rf_dir.rglob(ext)))

        for idx, img_p in enumerate(all_imgs):
            dest_img_dir = VAL_IMG if (idx % 5 == 0) else TRAIN_IMG
            dest_lbl_dir = VAL_LBL if (idx % 5 == 0) else TRAIN_LBL

            target_img_name = f"rf_banana_{img_p.name}"
            target_lbl_name = f"rf_banana_{img_p.stem}.txt"

            shutil.copy2(img_p, dest_img_dir / target_img_name)

            lbl_candidates = list(rf_dir.rglob(f"{img_p.stem}.txt"))
            if lbl_candidates and lbl_candidates[0].exists():
                with open(lbl_candidates[0], "r", encoding="utf-8") as lf:
                    lines = lf.readlines()
                new_lines = []
                for line in lines:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        parts[0] = str(banana_class_id)
                        new_lines.append(" ".join(parts) + "\n")
                with open(dest_lbl_dir / target_lbl_name, "w", encoding="utf-8") as nlf:
                    nlf.writelines(new_lines)
            else:
                with open(dest_lbl_dir / target_lbl_name, "w", encoding="utf-8") as nlf:
                    nlf.write(f"{banana_class_id} 0.500000 0.500000 0.600000 0.600000\n")
            imported += 1

    if temp_extract_dir:
        shutil.rmtree(temp_extract_dir, ignore_errors=True)

    print(f"✅ Successfully imported {imported} banana peel samples into dataset (Class 11).")
    return imported

def download_and_autolabel_banana_peels(target_count=30):
    """Download banana peel samples and auto-label using YOLO vision model."""
    from ultralytics import YOLO
    detector = YOLO("yolov8n.pt")  # COCO class 46 is banana

    banana_class_id = 11
    downloaded = 0
    print(f"\n[Download] Fetching curated banana peel samples & auto-labeling...")

    headers = {"User-Agent": "Mozilla/5.0"}

    for idx, url in enumerate(CURATED_BANANA_PEEL_URLS[:target_count]):
        try:
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req, timeout=8.0) as resp:
                img_data = resp.read()
            nparr = np.frombuffer(img_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if img is None:
                continue

            h, w = img.shape[:2]
            results = detector.predict(img, verbose=False, conf=0.18)
            boxes = results[0].boxes

            yolo_boxes = []
            for b in boxes:
                cid = int(b.cls[0])
                if cid == 46:  # COCO Banana
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    # Convert to normalized YOLO format (cx, cy, bw, bh)
                    cx = ((x1 + x2) / 2.0) / w
                    cy = ((y1 + y2) / 2.0) / h
                    bw = (x2 - x1) / w
                    bh = (y2 - y1) / h
                    yolo_boxes.append(f"{banana_class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

            # If COCO detector was uncertain, create a centered bounding box for the peel
            if not yolo_boxes:
                yolo_boxes.append(f"{banana_class_id} 0.500000 0.500000 0.650000 0.600000\n")

            # Route 80% to train, 20% to valid
            is_val = (idx % 5 == 0)
            target_img_dir = VAL_IMG if is_val else TRAIN_IMG
            target_lbl_dir = VAL_LBL if is_val else TRAIN_LBL

            base_name = f"banana_peel_{idx:03d}"
            cv2.imwrite(str(target_img_dir / f"{base_name}.jpg"), img)
            with open(target_lbl_dir / f"{base_name}.txt", "w", encoding="utf-8") as f:
                f.writelines(yolo_boxes)

            # Generate floor augmentations (perspective & lighting variation)
            for aug_i in range(2):
                aug_img = img.copy()
                if aug_i == 0:
                    aug_img = cv2.flip(aug_img, 1)  # horizontal flip
                    aug_name = f"banana_peel_{idx:03d}_flip"
                    aug_boxes = []
                    for line in yolo_boxes:
                        parts = line.strip().split()
                        cx = 1.0 - float(parts[1])
                        aug_boxes.append(f"{parts[0]} {cx:.6f} {parts[2]} {parts[3]} {parts[4]}\n")
                else:
                    # Brightness variation simulating floor shadows
                    aug_img = cv2.convertScaleAbs(aug_img, alpha=0.85, beta=-15)
                    aug_name = f"banana_peel_{idx:03d}_dark"
                    aug_boxes = yolo_boxes

                cv2.imwrite(str(TRAIN_IMG / f"{aug_name}.jpg"), aug_img)
                with open(TRAIN_LBL / f"{aug_name}.txt", "w", encoding="utf-8") as f:
                    f.writelines(aug_boxes)

            downloaded += 1
            print(f"  [{idx+1}/{len(CURATED_BANANA_PEEL_URLS)}] Processed banana peel sample -> Saved + Augmented")
        except Exception as e:
            print(f"  [Notice] Could not download image {idx+1}: {e}")

    print(f"\n🎉 Added {downloaded} primary banana peel sets ({downloaded * 3} total augmented images).")
    return downloaded

def main():
    print("=" * 65)
    print(" Banana Peel & Degradable Waste Dataset Builder")
    print("=" * 65)

    classes = update_data_yaml()

    # Check if a custom Roboflow folder was provided as CLI argument
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        import_roboflow_folder(sys.argv[1])
    else:
        download_and_autolabel_banana_peels(target_count=20)

    # Summarize dataset
    train_count = len(list(TRAIN_IMG.glob("*.jpg")))
    val_count = len(list(VAL_IMG.glob("*.jpg")))
    banana_train = len(list(TRAIN_LBL.glob("*banana*.*")))
    banana_val = len(list(VAL_LBL.glob("*banana*.*")))

    print("\n" + "=" * 65)
    print(f"📊 DATASET SUMMARY:")
    print(f" • Total Classes      : {len(classes)} ({', '.join(classes)})")
    print(f" • Degradable Class   : Class 11 -> '{classes[11] if len(classes)>11 else 'banana peel'}'")
    print(f" • Total Train Images : {train_count} (Banana peel sets: {banana_train})")
    print(f" • Total Valid Images : {val_count} (Banana peel sets: {banana_val})")
    print("=" * 65)
    print("Ready to train! Run: python train_degradable_model.py\n")

if __name__ == "__main__":
    main()
