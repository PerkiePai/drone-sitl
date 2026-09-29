"""Reshape the COCO Annotator export into RF-DETR's expected layout:

datasets_rfdetr/
  train/_annotations.coco.json + hardlinked images
  valid/_annotations.coco.json + hardlinked images

Reuses the exact same train/val split as the YOLO run (train.txt/val.txt) so
the two models are trained and evaluated on identical data.
"""

import json
import os
from pathlib import Path

ROOT = Path(__file__).parent
COCO_JSON = ROOT / "datasets" / "Truck4800-2.json"
IMAGES_DIR = ROOT / "datasets" / "images"
OUT_ROOT = ROOT / "datasets_rfdetr"

with open(COCO_JSON) as f:
    coco = json.load(f)

images_by_id = {img["id"]: img for img in coco["images"]}
anns_by_image = {}
for ann in coco["annotations"]:
    anns_by_image.setdefault(ann["image_id"], []).append(ann)

annotated_ids = set(anns_by_image.keys())


def file_names_from_list(list_path):
    names = set()
    for line in Path(list_path).read_text().splitlines():
        line = line.strip()
        if line:
            names.add(Path(line).name)
    return names


train_names = file_names_from_list(ROOT / "datasets" / "train.txt")
val_names = file_names_from_list(ROOT / "datasets" / "val.txt")

categories = coco["categories"]


def build_split(out_dir, file_names):
    out_dir.mkdir(parents=True, exist_ok=True)
    new_images = []
    new_annotations = []
    next_ann_id = 1

    for img_id in sorted(annotated_ids):
        img = images_by_id[img_id]
        if img["file_name"] not in file_names:
            continue

        src = IMAGES_DIR / img["file_name"]
        dst = out_dir / img["file_name"]
        if not dst.exists():
            os.link(src, dst)

        new_images.append(
            {
                "id": img_id,
                "file_name": img["file_name"],
                "width": img["width"],
                "height": img["height"],
            }
        )
        for ann in anns_by_image[img_id]:
            new_annotations.append(
                {
                    "id": next_ann_id,
                    "image_id": img_id,
                    "category_id": ann["category_id"],
                    "bbox": ann["bbox"],
                    "area": ann["area"],
                    "iscrowd": 0,
                    "segmentation": [],
                }
            )
            next_ann_id += 1

    coco_out = {
        "images": new_images,
        "annotations": new_annotations,
        "categories": categories,
    }
    with open(out_dir / "_annotations.coco.json", "w") as f:
        json.dump(coco_out, f)

    print(f"{out_dir.name}: {len(new_images)} images, {len(new_annotations)} boxes -> {out_dir}")


build_split(OUT_ROOT / "train", train_names)
build_split(OUT_ROOT / "valid", val_names)
