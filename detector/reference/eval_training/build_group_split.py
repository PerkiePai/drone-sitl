#!/usr/bin/env python3
"""Build train/val/test splits at the LEVEL OF THE TRUCK, not the frame.

Why this script exists
----------------------
The existing pipeline (`shuffle_dataset.py` -> `build_yolo_8010_split.py`) pools
all 4811 frames from the 50 per-truck folders and shuffles them at the *frame*
level before slicing 80/10/10. Because every truck contributes 96-98 near-identical
frames (one truck orbited by the camera), that shuffle scatters frames of the same
physical truck across train AND test. Measured consequence: 466/466 test frames
show a truck that was also seen during training. Every metric computed on that
split measures memorisation, not generalisation.

This script instead partitions the 50 trucks into disjoint groups and assigns
every frame to the split of its truck. No truck appears in more than one split,
so the held-out set finally measures "can the model find a truck it never saw
during training" (albeit still the same 3D asset in the same scene -- see
DESIGN.md G1 vs G2).

Inputs (all already in datasets/ -- no images or re-labelling needed)
-------------------------------------------------------------------
  manifest.csv      new_name,source_folder,source_file
                    4811 rows, source_folder in Truck2_01..Truck2_50
  Truck4800-2.json  COCO export from COCO Annotator
                    images:      4667 records, ALL of them annotated
                    annotations: 4767 records, bbox = [x,y,w,h] absolute px
                    categories:  [{id: 1, name: "Truck"}]

Caveats baked into the code
---------------------------
  * The `annotated` and `category_ids` fields on COCO image records are wrong
    (images with annotations still say annotated=false). "Does this image have a
    box" is derived ONLY from the annotations list.
  * 4811 - 4667 = 144 frames are absent from the COCO file entirely. Those are
    the negatives (truck out of frame / fully occluded). They are never dropped;
    they go to their own list for false-positive measurement in Phase 0.2.

Outputs
-------
  {out_dir}/splits/group_train.txt        positive frames, one abs path per line
  {out_dir}/splits/group_val.txt
  {out_dir}/splits/group_test.txt
  {out_dir}/splits/group_negatives.txt    all 144 no-box frames
  {out_dir}/splits/split_report.json      trucks per split + leakage check
  {out_dir}/truck_group.yaml              Ultralytics data config
  {out_dir}/../datasets_rfdetr_group/{train,valid,test}/_annotations.coco.json
                                          COCO subset per split, ids re-indexed

Deterministic: same --seed + --mode always yields byte-identical output.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
from dataclasses import dataclass, field
from pathlib import Path

TOTAL_FRAMES = 4811
TOTAL_ANNOTATIONS = 4767
N_TRUCKS = 50

HERE = Path(__file__).resolve().parent
DEFAULT_OUT_DIR = HERE / "datasets"


# --------------------------------------------------------------------------- #
# config
# --------------------------------------------------------------------------- #
@dataclass
class Config:
    out_dir: Path
    images_dir: Path
    manifest: Path
    coco: Path
    seed: int = 42
    mode: str = "shuffle"  # "shuffle" | "sequential"
    train_trucks: int = 40
    val_trucks: int = 5
    test_trucks: int = 5

    rfdetr_dir: Path = field(init=False)
    splits_dir: Path = field(init=False)

    def __post_init__(self) -> None:
        self.splits_dir = self.out_dir / "splits"
        self.rfdetr_dir = self.out_dir.parent / "datasets_rfdetr_group"
        if self.train_trucks + self.val_trucks + self.test_trucks != N_TRUCKS:
            raise SystemExit(
                f"train+val+test trucks must sum to {N_TRUCKS}, got "
                f"{self.train_trucks}+{self.val_trucks}+{self.test_trucks}"
            )
        if self.mode not in ("shuffle", "sequential"):
            raise SystemExit(f"unknown --mode {self.mode!r}")


# --------------------------------------------------------------------------- #
# load
# --------------------------------------------------------------------------- #
def load_manifest(path: Path) -> dict[str, str]:
    """new_name -> source_folder (the truck the frame belongs to)."""
    frame_to_truck: dict[str, str] = {}
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_to_truck[row["new_name"]] = row["source_folder"]
    if len(frame_to_truck) != TOTAL_FRAMES:
        raise SystemExit(
            f"manifest has {len(frame_to_truck)} rows, expected {TOTAL_FRAMES}"
        )
    return frame_to_truck


def load_coco(path: Path) -> dict:
    with path.open() as f:
        coco = json.load(f)
    n_img = len(coco["images"])
    n_ann = len(coco["annotations"])
    if n_ann != TOTAL_ANNOTATIONS:
        raise SystemExit(
            f"COCO has {n_ann} annotations, expected {TOTAL_ANNOTATIONS}"
        )
    print(f"[load] manifest frames={TOTAL_FRAMES}  coco images={n_img}  "
          f"coco annotations={n_ann}  negatives={TOTAL_FRAMES - n_img}")
    return coco


# --------------------------------------------------------------------------- #
# split
# --------------------------------------------------------------------------- #
def partition_trucks(cfg: Config) -> dict[str, list[str]]:
    trucks = sorted({f"Truck2_{i:02d}" for i in range(1, N_TRUCKS + 1)})
    if cfg.mode == "shuffle":
        random.Random(cfg.seed).shuffle(trucks)
    # sequential: keep sorted order

    a = cfg.train_trucks
    b = a + cfg.val_trucks
    split = {
        "train": sorted(trucks[:a]),
        "val": sorted(trucks[a:b]),
        "test": sorted(trucks[b:]),
    }
    return split


def assert_no_truck_overlap(split_trucks: dict[str, list[str]]) -> dict[str, list[str]]:
    s = {k: set(v) for k, v in split_trucks.items()}
    overlaps = {
        "train_val": sorted(s["train"] & s["val"]),
        "train_test": sorted(s["train"] & s["test"]),
        "val_test": sorted(s["val"] & s["test"]),
    }
    for name, ov in overlaps.items():
        assert not ov, f"truck leak between {name}: {ov}"
    covered = s["train"] | s["val"] | s["test"]
    assert len(covered) == N_TRUCKS, f"only {len(covered)}/{N_TRUCKS} trucks covered"
    return overlaps


# --------------------------------------------------------------------------- #
# assemble per-split frame + annotation records
# --------------------------------------------------------------------------- #
@dataclass
class SplitData:
    trucks: list[str]
    positive_frames: list[str]              # file_name, sorted
    coco_images: list[dict]                 # re-indexed
    coco_annotations: list[dict]            # re-indexed

    @property
    def n_frames(self) -> int:
        return len(self.positive_frames)

    @property
    def n_annotations(self) -> int:
        return len(self.coco_annotations)


def build_split_data(
    split_trucks: dict[str, list[str]],
    frame_to_truck: dict[str, str],
    coco: dict,
) -> tuple[dict[str, SplitData], list[str]]:
    truck_of_split = {
        truck: name for name, trucks in split_trucks.items() for truck in trucks
    }

    # old image id -> record, and file_name -> old image id
    img_by_id = {img["id"]: img for img in coco["images"]}
    anns_by_img: dict[int, list[dict]] = {}
    for ann in coco["annotations"]:
        anns_by_img.setdefault(ann["image_id"], []).append(ann)

    coco_file_names = {img["file_name"] for img in coco["images"]}

    # negatives: in manifest but not in COCO at all
    negatives = sorted(fn for fn in frame_to_truck if fn not in coco_file_names)

    # bucket positive frames by split
    buckets: dict[str, list[str]] = {"train": [], "val": [], "test": []}
    for img in coco["images"]:
        fn = img["file_name"]
        truck = frame_to_truck.get(fn)
        if truck is None:
            raise SystemExit(f"COCO image {fn!r} not found in manifest")
        buckets[truck_of_split[truck]].append(fn)

    out: dict[str, SplitData] = {}
    for name in ("train", "val", "test"):
        frames = sorted(buckets[name])
        new_images: list[dict] = []
        new_anns: list[dict] = []
        next_img_id = 1
        next_ann_id = 1
        # deterministic: iterate frames in sorted file_name order
        fn_to_oldid = {img["file_name"]: img["id"] for img in coco["images"]}
        for fn in frames:
            old_id = fn_to_oldid[fn]
            src = img_by_id[old_id]
            new_images.append({
                "id": next_img_id,
                "file_name": fn,
                "width": src.get("width", 1280),
                "height": src.get("height", 720),
            })
            for ann in sorted(anns_by_img.get(old_id, []), key=lambda a: a["id"]):
                new_anns.append({
                    "id": next_ann_id,
                    "image_id": next_img_id,
                    "category_id": ann["category_id"],
                    "bbox": [float(x) for x in ann["bbox"]],
                    "area": float(ann.get("area", ann["bbox"][2] * ann["bbox"][3])),
                    "iscrowd": int(ann.get("iscrowd", 0)),
                    "segmentation": ann.get("segmentation", []),
                })
                next_ann_id += 1
            next_img_id += 1
        out[name] = SplitData(
            trucks=split_trucks[name],
            positive_frames=frames,
            coco_images=new_images,
            coco_annotations=new_anns,
        )
    return out, negatives


# --------------------------------------------------------------------------- #
# write
# --------------------------------------------------------------------------- #
def _abs_lines(frames: list[str], images_dir: Path) -> str:
    return "\n".join(str(images_dir / fn) for fn in frames) + "\n"


def write_outputs(
    cfg: Config,
    split_data: dict[str, SplitData],
    negatives: list[str],
    overlaps: dict[str, list[str]],
    categories: list[dict],
) -> dict:
    cfg.splits_dir.mkdir(parents=True, exist_ok=True)

    yolo_names = {"train": "group_train.txt", "val": "group_val.txt", "test": "group_test.txt"}
    for name, fname in yolo_names.items():
        (cfg.splits_dir / fname).write_text(
            _abs_lines(split_data[name].positive_frames, cfg.images_dir)
        )
    (cfg.splits_dir / "group_negatives.txt").write_text(
        _abs_lines(negatives, cfg.images_dir)
    )

    # Ultralytics yaml
    yaml_text = (
        "# Auto-generated by build_group_split.py -- group (per-truck) split.\n"
        f"# seed={cfg.seed} mode={cfg.mode}\n"
        f"path: {cfg.out_dir}\n"
        "train: splits/group_train.txt\n"
        "val: splits/group_val.txt\n"
        "test: splits/group_test.txt\n"
        "names:\n"
        "  0: Truck\n"
    )
    (cfg.out_dir / "truck_group.yaml").write_text(yaml_text)

    # RF-DETR COCO subsets
    rfdetr_split_dir = {"train": "train", "val": "valid", "test": "test"}
    for name, sub in rfdetr_split_dir.items():
        d = cfg.rfdetr_dir / sub
        d.mkdir(parents=True, exist_ok=True)
        payload = {
            "images": split_data[name].coco_images,
            "annotations": split_data[name].coco_annotations,
            "categories": categories,
        }
        (d / "_annotations.coco.json").write_text(json.dumps(payload, indent=2))

    # report
    report = {
        "generator": "build_group_split.py",
        "seed": cfg.seed,
        "mode": cfg.mode,
        "truck_counts": {
            "train": cfg.train_trucks,
            "val": cfg.val_trucks,
            "test": cfg.test_trucks,
        },
        "splits": {
            name: {
                "n_trucks": len(sd.trucks),
                "trucks": sd.trucks,
                "n_frames": sd.n_frames,
                "n_annotations": sd.n_annotations,
            }
            for name, sd in split_data.items()
        },
        "negatives": {"n_frames": len(negatives)},
        "leakage_check": {
            "truck_overlap": overlaps,
            "passed": all(len(v) == 0 for v in overlaps.values()),
        },
        "totals": {
            "frames": sum(sd.n_frames for sd in split_data.values()) + len(negatives),
            "annotations": sum(sd.n_annotations for sd in split_data.values()),
            "expected_frames": TOTAL_FRAMES,
            "expected_annotations": TOTAL_ANNOTATIONS,
        },
    }
    (cfg.splits_dir / "split_report.json").write_text(json.dumps(report, indent=2))
    return report


# --------------------------------------------------------------------------- #
# acceptance checks
# --------------------------------------------------------------------------- #
def run_asserts(report: dict) -> None:
    assert report["leakage_check"]["passed"], "truck overlap detected"
    assert report["totals"]["frames"] == TOTAL_FRAMES, (
        f'frame total {report["totals"]["frames"]} != {TOTAL_FRAMES}'
    )
    assert report["totals"]["annotations"] == TOTAL_ANNOTATIONS, (
        f'annotation total {report["totals"]["annotations"]} != {TOTAL_ANNOTATIONS}'
    )


def print_summary(report: dict, negatives: list[str]) -> None:
    print("\n" + "=" * 78)
    print(f"  group split  |  seed={report['seed']}  mode={report['mode']}")
    print("=" * 78)
    hdr = f"{'split':<12}{'trucks':>8}{'frames':>9}{'anns':>8}   trucks"
    print(hdr)
    print("-" * 78)
    for name in ("train", "val", "test"):
        s = report["splits"][name]
        truck_str = ", ".join(t.replace("Truck2_", "") for t in s["trucks"])
        print(f"{name:<12}{s['n_trucks']:>8}{s['n_frames']:>9}{s['n_annotations']:>8}   {truck_str}")
    print(f"{'negatives':<12}{'-':>8}{len(negatives):>9}{'0':>8}   (spread across all trucks)")
    print("-" * 78)
    t = report["totals"]
    print(f"{'TOTAL':<12}{N_TRUCKS:>8}{t['frames']:>9}{t['annotations']:>8}")
    print("=" * 78)
    lk = report["leakage_check"]
    status = "PASS -- no truck in more than one split" if lk["passed"] else "FAIL"
    print(f"leakage check: {status}")


# --------------------------------------------------------------------------- #
def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--mode", choices=["shuffle", "sequential"], default="shuffle")
    p.add_argument("--train-trucks", type=int, default=40)
    p.add_argument("--val-trucks", type=int, default=5)
    p.add_argument("--test-trucks", type=int, default=5)
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR,
                   help="the datasets/ dir that holds manifest.csv (default: %(default)s)")
    p.add_argument("--images-dir", type=Path, default=None,
                   help="dir the .txt paths point at (default: {out-dir}/images)")
    p.add_argument("--manifest", type=Path, default=None,
                   help="default: {out-dir}/manifest.csv")
    p.add_argument("--coco", type=Path, default=None,
                   help="default: {out-dir}/Truck4800-2.json")
    args = p.parse_args()

    out_dir = args.out_dir.resolve()
    cfg = Config(
        out_dir=out_dir,
        images_dir=(args.images_dir or (out_dir / "images")).resolve(),
        manifest=(args.manifest or (out_dir / "manifest.csv")).resolve(),
        coco=(args.coco or (out_dir / "Truck4800-2.json")).resolve(),
        seed=args.seed,
        mode=args.mode,
        train_trucks=args.train_trucks,
        val_trucks=args.val_trucks,
        test_trucks=args.test_trucks,
    )

    for pth in (cfg.manifest, cfg.coco):
        if not pth.exists():
            raise SystemExit(f"missing input: {pth}")
    if not cfg.images_dir.exists():
        print(f"[warn] images dir does not exist yet: {cfg.images_dir}\n"
              f"       .txt lists will still be written with that prefix.")

    frame_to_truck = load_manifest(cfg.manifest)
    coco = load_coco(cfg.coco)

    split_trucks = partition_trucks(cfg)
    overlaps = assert_no_truck_overlap(split_trucks)
    split_data, negatives = build_split_data(split_trucks, frame_to_truck, coco)

    report = write_outputs(cfg, split_data, negatives, overlaps, coco["categories"])
    run_asserts(report)
    print_summary(report, negatives)

    print(f"\nwrote:")
    print(f"  {cfg.splits_dir}/group_{{train,val,test}}.txt")
    print(f"  {cfg.splits_dir}/group_negatives.txt")
    print(f"  {cfg.splits_dir}/split_report.json")
    print(f"  {cfg.out_dir}/truck_group.yaml")
    print(f"  {cfg.rfdetr_dir}/{{train,valid,test}}/_annotations.coco.json")


if __name__ == "__main__":
    main()
