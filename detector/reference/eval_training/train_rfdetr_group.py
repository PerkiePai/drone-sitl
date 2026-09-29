"""Q-D — เทรน RF-DETR Medium บน group split (แบ่งตามคันรถ ไม่รั่ว)

จุดประสงค์
---------
RF-DETR ตัวที่ deploy อยู่ได้ AP50 = 1.0000 เป๊ะทั้ง 7 เงื่อนไขบน group_test
ซึ่งเป็นสัญญาณ memorization — แต่ checkpoint นั้นเทรนด้วยรถครบ 50 คัน
ทำให้ group_test **รั่วสำหรับมัน** จึงแยกไม่ออกว่า "ทนจริง" หรือ "จำรถได้"

สคริปต์นี้เทรนใหม่บน **40 คัน** แล้ววัดบน **5 คันที่ไม่เคยเห็น**
เพื่อให้เทียบกับ YOLO honest baseline ได้อย่างยุติธรรม

⚠️ คัดลอก hyperparameter จาก train_rfdetr_medium.py **ครบทุกตัว**
   เปลี่ยนแค่ dataset_dir + output_dir เพื่อให้ delta ที่เห็นมาจาก
   "split ที่ไม่รั่ว" อย่างเดียว ไม่ปนกับการจูนอย่างอื่น

รัน (ใน .venv_rfdetr)
--------------------
    source ~/Tiger/DroneDetectionProject/.venv_rfdetr/bin/activate
    cd ~/Tiger/DroneDetectionProject
    python3 link_rfdetr_group_images.py     # <-- ต้องรันก่อน ครั้งเดียว
    tmux new -s rfdetr
    python3 train_rfdetr_group.py 2>&1 | tee train_rfdetr_group.log
"""

import json
from pathlib import Path

from rfdetr import RFDETRMedium

HERE = Path(__file__).resolve().parent
DATASET_DIR = str(HERE / "datasets_rfdetr_group")                       # <-- เปลี่ยนแค่นี้
OUTPUT_DIR  = str(HERE / "runs_rfdetr" / "truck_group_quality_aug")     # <-- และนี่

# เหมือน train_rfdetr_medium.py ทุกตัวอักษร
AUG_CONFIG = {
    "HorizontalFlip": {"p": 0.5},
    "Blur": {"blur_limit": (3, 9), "p": 0.15},
    "MotionBlur": {"blur_limit": (3, 15), "p": 0.15},
    "GaussNoise": {"std_range": (0.04, 0.15), "p": 0.3},
    "ISONoise": {"color_shift": (0.01, 0.05), "intensity": (0.1, 0.5), "p": 0.2},
    "ImageCompression": {"quality_range": (30, 85), "p": 0.35},
    "RandomBrightnessContrast": {"brightness_limit": 0.3, "contrast_limit": 0.3, "p": 0.4},
    "RandomGamma": {"gamma_limit": (70, 130), "p": 0.2},
    "CLAHE": {"p": 0.05},
}

TRAIN_KWARGS = dict(
    dataset_dir=DATASET_DIR,
    output_dir=OUTPUT_DIR,
    class_names=["Truck"],
    epochs=150,
    batch_size=8,
    grad_accum_steps=2,
    num_workers=8,
    early_stopping=True,
    early_stopping_patience=15,
    aug_config=AUG_CONFIG,
    augmentation_backend="cpu",
    device="cuda",
    run_test=True,          # ประเมินบน test split ตอนจบ -> ได้ mAP ซื่อสัตย์ในlog
)


def _dump_kwargs() -> None:
    """เก็บหลักฐานว่าเปลี่ยนแค่ dataset_dir + output_dir"""
    out = Path(OUTPUT_DIR); out.mkdir(parents=True, exist_ok=True)
    (out / "train_kwargs.json").write_text(json.dumps({
        "model": "RFDETRMedium(num_classes=1, resolution=640)",
        "train_kwargs": {k: v for k, v in TRAIN_KWARGS.items() if k != "aug_config"},
        "aug_config": AUG_CONFIG,
        "note": "เหมือน train_rfdetr_medium.py ทุกอย่าง ยกเว้น dataset_dir + output_dir",
    }, indent=2, ensure_ascii=False))


def main():
    _dump_kwargs()
    model = RFDETRMedium(num_classes=1, resolution=640)
    model.train(**TRAIN_KWARGS)


if __name__ == "__main__":
    main()
