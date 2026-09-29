#!/usr/bin/env python3
"""Run one model through eval_common.run_evaluation and save the honest numbers.

Task 0.3a (sanity) vs 0.3b (baseline)
------------------------------------
Running an EXISTING checkpoint (Env2aug.pt / checkpoint_best_ema.pth) through this
script is a SANITY CHECK ONLY -- those weights were trained on data containing all
50 trucks, so even the new per-truck test split is still leaked for them. Pass
--leaked to stamp that into the output JSON so nobody mistakes it for a baseline.
The real baseline comes from a model retrained on group_train.txt (train_yolo_group.py).

Usage
-----
  python run_eval.py --model yolo   --weights ../Env2aug.pt \
      --split-list datasets/splits/group_test.txt \
      --gt-coco   datasets_rfdetr_group/test/_annotations.coco.json \
      --negatives datasets/splits/group_negatives.txt \
      --out results/eval__yolo_env2aug__LEAKED.json \
      --tag "yolo_env2aug_ckpt_old" --leaked
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from eval_common import (
    DEFAULT_CONDITIONS,
    make_rfdetr_predict_fn,
    make_yolo_predict_fn,
    run_evaluation,
)


def _read_list(path: Path) -> list[Path]:
    return [Path(line) for line in path.read_text().splitlines() if line.strip()]


def _fmt_md(results: dict) -> str:
    cols = ["condition", "AP50", "AP75", "AP50-95", "P@.25", "R@.25", "FP/img", "FP/neg"]
    lines = ["| " + " | ".join(cols) + " |", "|" + "|".join(["---"] * len(cols)) + "|"]
    for cond, m in results.items():
        lines.append("| " + " | ".join([
            cond,
            f"{m['AP50']:.4f}", f"{m['AP75']:.4f}", f"{m['AP50_95']:.4f}",
            f"{m['precision']:.4f}", f"{m['recall']:.4f}",
            f"{m['fp_per_image']:.3f}", f"{m['fp_per_negative_image']:.3f}",
        ]) + " |")
    return "\n".join(lines)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", choices=["yolo", "rfdetr"], required=True)
    p.add_argument("--weights", type=Path, required=True)
    p.add_argument("--split-list", type=Path, required=True)
    p.add_argument("--gt-coco", type=Path, required=True)
    p.add_argument("--negatives", type=Path, default=None)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--tag", type=str, required=True)
    p.add_argument("--leaked", action="store_true",
                   help="stamp leakage_warning: this checkpoint saw all 50 trucks in training")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--conf-operating", type=float, default=0.25)
    p.add_argument("--imgsz", type=int, default=1280, help="YOLO only")
    p.add_argument("--limit", type=int, default=0,
                   help="ทดสอบเร็ว: ใช้แค่ N ภาพแรก (0 = ทั้งหมด)")
    p.add_argument("--conditions", default="",
                   help="เลือกเงื่อนไข คั่นด้วย , เช่น clean,combo_worst_case (ว่าง = ครบ 7)")
    p.add_argument("--device", default=None,
                   help="e.g. 'cuda', '0', 'cpu'. YOLO: passed to predict(). RF-DETR: passed to from_checkpoint().")
    args = p.parse_args()

    image_paths = _read_list(args.split_list)
    negative_paths = _read_list(args.negatives) if args.negatives else None
    gt_coco = json.loads(args.gt_coco.read_text())

    if args.limit:
        keep = {p.name for p in image_paths[:args.limit]}
        image_paths = image_paths[:args.limit]
        gt_coco = {
            "images": [i for i in gt_coco["images"] if i["file_name"] in keep],
            "annotations": [a for a in gt_coco["annotations"]
                            if a["image_id"] in {i["id"] for i in gt_coco["images"]
                                                 if i["file_name"] in keep}],
            "categories": gt_coco["categories"],
        }
        if negative_paths:
            negative_paths = negative_paths[:args.limit]
        print(f"[run_eval] LIMIT: {len(image_paths)} pos / "
              f"{len(negative_paths) if negative_paths else 0} neg")

    conds = DEFAULT_CONDITIONS
    if args.conditions:
        want = [c.strip() for c in args.conditions.split(",") if c.strip()]
        bad = [c for c in want if c not in DEFAULT_CONDITIONS]
        if bad:
            raise SystemExit(f"ไม่รู้จักเงื่อนไข: {bad} | มีให้เลือก: {list(DEFAULT_CONDITIONS)}")
        conds = {c: DEFAULT_CONDITIONS[c] for c in want}

    if args.model == "yolo":
        predict_fn = make_yolo_predict_fn(str(args.weights), imgsz=args.imgsz, device=args.device)
    else:
        predict_fn = make_rfdetr_predict_fn(str(args.weights), device=(args.device or "cuda"))

    t0 = time.time()
    results = run_evaluation(
        predict_fn, image_paths, gt_coco,
        negative_paths=negative_paths,
        conditions=conds,
        seed=args.seed,
        conf_operating=args.conf_operating,
    )
    elapsed = time.time() - t0

    leakage_warning = (
        "LEAKED: this checkpoint was trained on data containing all 50 trucks; "
        "the per-truck test split does not hold out anything for it. Sanity check only."
        if args.leaked else
        "clean: this checkpoint was trained only on the group_train trucks."
    )

    payload = {
        "tag": args.tag,
        "model": args.model,
        "weights": str(args.weights),
        "split_list": str(args.split_list),
        "gt_coco": str(args.gt_coco),
        "negatives": str(args.negatives) if args.negatives else None,
        "n_images": len(image_paths),
        "n_negative_images": len(negative_paths) if negative_paths else 0,
        "seed": args.seed,
        "conf_operating": args.conf_operating,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "elapsed_sec": round(elapsed, 1),
        "leakage_warning": leakage_warning,
        "results": results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2))

    print(f"\n# {args.tag}   ({args.model}, {len(image_paths)} images, {elapsed:.0f}s)")
    print(f"_{leakage_warning}_\n")
    print(_fmt_md(results))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
