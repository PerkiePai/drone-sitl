#!/usr/bin/env python3
"""Shared, honest evaluation for the drone truck detector.

Why this module exists
----------------------
Every eval script in this project (`eval_yolo_8010.py:83`, `eval_rfdetr_8010.py:74`,
`test_robustness.py`, `test_robustness_rfdetr.py`) scores robustness like this:

    pred = model.predict(degraded, conf=0.25)
    if len(pred.boxes) > 0:
        hits[condition] += 1

That counts an image as a "hit" if the model emits *any* box *anywhere*. It never
checks whether the box overlaps a real truck, never checks how many boxes, never
counts false positives. A model that emits one garbage box on every frame scores
a perfect 466/466 -- which is exactly why RF-DETR's reported robustness is
466/466 on every condition while YOLO (which has objectness + NMS that suppress
weak boxes) reads 277/466 on the same combo. Those two numbers are not
comparable; they measure box-emission habits, not accuracy.

This module replaces hit-rate with:
  * AP50 / AP75 / AP50-95   -- via pycocotools COCOeval (reference implementation),
    computed on an in-memory dataset, no temp files (disk has been at 97%).
  * precision / recall / fp_per_image at a fixed operating threshold (0.25),
    via greedy IoU matching.
  * fp_per_negative_image   -- on frames that contain no truck at all.

It also fixes a reproducibility bug: the original `degrade_noise` calls
`np.random.normal` with no seed, so re-running an eval gives different numbers.
Here every degradation that needs randomness takes an explicit
`np.random.Generator` derived from (seed, image_index, condition_name).

The six degradation functions are lifted verbatim (same kernel sizes, same
constants) from the scripts above so new numbers stay comparable to old ones --
only the RNG plumbing changed.
"""

from __future__ import annotations

import contextlib
import hashlib
import io
import time
import warnings
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import cv2
import numpy as np

# predict_fn: takes a BGR uint8 image, returns (xyxy float[N,4], scores float[N])
PredictFn = Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]]
# condition_fn: takes a BGR uint8 image + an RNG, returns a BGR uint8 image
ConditionFn = Callable[[np.ndarray, np.random.Generator], np.ndarray]

DETECT_THRESHOLD = 0.001   # predict_fn must emit down to here so AP sees the full PR curve
IOU_MATCH = 0.5            # for precision/recall/FP bookkeeping


# --------------------------------------------------------------------------- #
# degradation functions  (constants identical to the originals)
# --------------------------------------------------------------------------- #
def degrade_clean(img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    return img


def degrade_blur(img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    return cv2.GaussianBlur(img, (15, 15), 0)


def degrade_motion_blur(img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    k = np.zeros((21, 21))
    k[10, :] = 1.0 / 21
    return cv2.filter2D(img, -1, k)


def degrade_noise(img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Gaussian sigma=25, additive. Original used np.random.normal with no seed;
    here the caller-supplied Generator makes it reproducible."""
    noise = rng.normal(0, 25, img.shape).astype(np.float32)
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def degrade_compression(img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 15])
    return cv2.imdecode(enc, cv2.IMREAD_COLOR)


def degrade_lowlight(img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    return np.clip(img.astype(np.float32) * 0.35, 0, 255).astype(np.uint8)


def degrade_combo(img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    img = degrade_blur(img, rng)
    img = degrade_noise(img, rng)
    img = degrade_compression(img, rng)
    return degrade_lowlight(img, rng)


DEFAULT_CONDITIONS: dict[str, ConditionFn] = {
    "clean": degrade_clean,
    "blur": degrade_blur,
    "motion_blur": degrade_motion_blur,
    "noise": degrade_noise,
    "heavy_compression": degrade_compression,
    "low_light": degrade_lowlight,
    "combo_worst_case": degrade_combo,
}


# --------------------------------------------------------------------------- #
# rng derivation
# --------------------------------------------------------------------------- #
def _stable_int(text: str) -> int:
    return int.from_bytes(hashlib.sha256(text.encode()).digest()[:8], "big")


def make_rng(seed: int, image_index: int, condition_name: str) -> np.random.Generator:
    """Deterministic per (seed, image, condition) so an eval re-run is byte-identical."""
    return np.random.default_rng((seed, image_index, _stable_int(condition_name)))


# --------------------------------------------------------------------------- #
# geometry
# --------------------------------------------------------------------------- #
def iou_matrix(boxes_a: np.ndarray, boxes_b: np.ndarray) -> np.ndarray:
    """boxes in xyxy. Returns (len(a), len(b)) IoU."""
    if len(boxes_a) == 0 or len(boxes_b) == 0:
        return np.zeros((len(boxes_a), len(boxes_b)), dtype=np.float32)
    a = boxes_a[:, None, :]
    b = boxes_b[None, :, :]
    ix1 = np.maximum(a[..., 0], b[..., 0])
    iy1 = np.maximum(a[..., 1], b[..., 1])
    ix2 = np.minimum(a[..., 2], b[..., 2])
    iy2 = np.minimum(a[..., 3], b[..., 3])
    iw = np.clip(ix2 - ix1, 0, None)
    ih = np.clip(iy2 - iy1, 0, None)
    inter = iw * ih
    area_a = np.clip(a[..., 2] - a[..., 0], 0, None) * np.clip(a[..., 3] - a[..., 1], 0, None)
    area_b = np.clip(b[..., 2] - b[..., 0], 0, None) * np.clip(b[..., 3] - b[..., 1], 0, None)
    union = area_a + area_b - inter
    return np.where(union > 0, inter / union, 0.0).astype(np.float32)


def greedy_match(
    det_xyxy: np.ndarray,
    det_scores: np.ndarray,
    gt_xyxy: np.ndarray,
    iou_thr: float = IOU_MATCH,
) -> tuple[int, int, int]:
    """Return (tp, fp, fn). Detections already filtered to the operating threshold.
    Highest score first; each GT can be claimed once."""
    order = np.argsort(-det_scores)
    gt_taken = np.zeros(len(gt_xyxy), dtype=bool)
    ious = iou_matrix(det_xyxy[order], gt_xyxy) if len(det_xyxy) else np.zeros((0, len(gt_xyxy)))
    tp = fp = 0
    for row in range(len(order)):
        cand = -1
        best = iou_thr
        for g in range(len(gt_xyxy)):
            if gt_taken[g]:
                continue
            if ious[row, g] >= best:
                best = ious[row, g]
                cand = g
        if cand >= 0:
            gt_taken[cand] = True
            tp += 1
        else:
            fp += 1
    fn = int((~gt_taken).sum())
    return tp, fp, fn


# --------------------------------------------------------------------------- #
# COCO AP  (in-memory, no temp files)
# --------------------------------------------------------------------------- #
def _coco_from_dict(dataset: dict) -> Any:
    from pycocotools.coco import COCO

    with contextlib.redirect_stdout(io.StringIO()):
        c = COCO()
        c.dataset = dataset
        c.createIndex()
    return c


def coco_ap(
    gt_dataset: dict,
    detections: list[dict],
    image_ids: Sequence[int],
) -> dict[str, float]:
    """detections: [{image_id, category_id, bbox:[x,y,w,h], score}, ...] (xywh)."""
    if not detections:
        return {"AP50": 0.0, "AP75": 0.0, "AP50_95": 0.0}

    from pycocotools.cocoeval import COCOeval

    gt = _coco_from_dict(gt_dataset)
    with contextlib.redirect_stdout(io.StringIO()):
        dt = gt.loadRes(detections)
        ev = COCOeval(gt, dt, iouType="bbox")
        ev.params.imgIds = list(image_ids)
        ev.evaluate()
        ev.accumulate()
        ev.summarize()
    return {
        "AP50_95": float(ev.stats[0]),
        "AP50": float(ev.stats[1]),
        "AP75": float(ev.stats[2]),
    }


# --------------------------------------------------------------------------- #
# main entry
# --------------------------------------------------------------------------- #
def _index_gt(gt_coco: dict) -> tuple[dict[str, int], dict[int, np.ndarray]]:
    """file_name -> image_id, and image_id -> gt boxes xyxy."""
    fname_to_id = {img["file_name"]: img["id"] for img in gt_coco["images"]}
    gt_by_id: dict[int, list[list[float]]] = {img["id"]: [] for img in gt_coco["images"]}
    for ann in gt_coco["annotations"]:
        x, y, w, h = ann["bbox"]
        gt_by_id.setdefault(ann["image_id"], []).append([x, y, x + w, y + h])
    return fname_to_id, {k: np.asarray(v, dtype=np.float32).reshape(-1, 4) for k, v in gt_by_id.items()}


def run_evaluation(
    predict_fn: PredictFn,
    image_paths: Sequence[Path],
    gt_coco: dict,
    negative_paths: Sequence[Path] | None = None,
    conditions: dict[str, ConditionFn] | None = None,
    seed: int = 0,
    conf_operating: float = 0.25,
    verbose: bool = True,
    progress_every: int = 20,
) -> dict[str, dict[str, float]]:
    """Evaluate `predict_fn` on every condition. Nothing is written to disk.

    predict_fn MUST return detections down to ~{DETECT_THRESHOLD} confidence; AP
    needs the full precision/recall curve, so do not pre-threshold at 0.25.
    """
    conditions = conditions or DEFAULT_CONDITIONS
    fname_to_id, gt_by_id = _index_gt(gt_coco)
    categories = gt_coco["categories"]
    cat_id = categories[0]["id"]

    image_paths = [Path(p) for p in image_paths]
    negative_paths = [Path(p) for p in (negative_paths or [])]

    # cache source images once
    src_pos = [(p, cv2.imread(str(p))) for p in image_paths]
    missing = [str(p) for p, im in src_pos if im is None]
    if missing:
        raise SystemExit(f"could not read {len(missing)} images, e.g. {missing[:3]}")
    src_neg = [(p, cv2.imread(str(p))) for p in negative_paths]
    neg_missing = [str(p) for p, im in src_neg if im is None]
    if neg_missing:
        raise SystemExit(f"could not read {len(neg_missing)} negative images, e.g. {neg_missing[:3]}")

    results: dict[str, dict[str, float]] = {}

    n_cond = len(conditions)
    total_infer = n_cond * (len(src_pos) + len(src_neg))
    done_infer = 0
    t_start = time.time()
    if verbose:
        print(f"[eval] {len(src_pos)} positives + {len(src_neg)} negatives x {n_cond} conditions"
              f" = {total_infer:,} inferences", flush=True)

    for ci, (cond_name, cond_fn) in enumerate(conditions.items(), 1):
        detections: list[dict] = []
        tp_tot = fp_tot = fn_tot = 0
        t_cond = time.time()
        if verbose:
            print(f"[eval] ({ci}/{n_cond}) {cond_name}", flush=True)

        for i, (path, img) in enumerate(src_pos):
            rng = make_rng(seed, i, cond_name)
            degraded = cond_fn(img.copy(), rng)
            xyxy, scores = predict_fn(degraded)
            xyxy = np.asarray(xyxy, dtype=np.float32).reshape(-1, 4)
            scores = np.asarray(scores, dtype=np.float32).reshape(-1)

            image_id = fname_to_id[path.name]
            for (x1, y1, x2, y2), s in zip(xyxy, scores):
                detections.append({
                    "image_id": image_id,
                    "category_id": cat_id,
                    "bbox": [float(x1), float(y1), float(x2 - x1), float(y2 - y1)],
                    "score": float(s),
                })

            keep = scores >= conf_operating
            tp, fp, fn = greedy_match(xyxy[keep], scores[keep], gt_by_id.get(image_id, np.zeros((0, 4))))
            tp_tot += tp
            fp_tot += fp
            fn_tot += fn

            done_infer += 1
            if verbose and progress_every and (i + 1) % progress_every == 0:
                el = time.time() - t_start
                rate = done_infer / max(el, 1e-9)
                eta = (total_infer - done_infer) / max(rate, 1e-9)
                print(f"\r        pos {i+1}/{len(src_pos)} | total {done_infer}/{total_infer}"
                      f" | {rate:.1f} img/s | ETA ~{eta/60:.1f} min      ", end="", flush=True)

        ap = coco_ap(
            {"images": gt_coco["images"], "annotations": gt_coco["annotations"], "categories": categories},
            detections,
            [img["id"] for img in gt_coco["images"]],
        )

        precision = tp_tot / (tp_tot + fp_tot) if (tp_tot + fp_tot) else 0.0
        recall = tp_tot / (tp_tot + fn_tot) if (tp_tot + fn_tot) else 0.0
        fp_per_image = fp_tot / len(src_pos) if src_pos else 0.0

        # negatives: every box above the operating threshold is a false positive
        fp_neg = 0
        for j, (path, img) in enumerate(src_neg):
            rng = make_rng(seed, 10_000_000 + j, cond_name)
            degraded = cond_fn(img.copy(), rng)
            _, scores = predict_fn(degraded)
            scores = np.asarray(scores, dtype=np.float32).reshape(-1)
            fp_neg += int((scores >= conf_operating).sum())

            done_infer += 1
            if verbose and progress_every and (j + 1) % progress_every == 0:
                el = time.time() - t_start
                rate = done_infer / max(el, 1e-9)
                eta = (total_infer - done_infer) / max(rate, 1e-9)
                print(f"\r        neg {j+1}/{len(src_neg)} | total {done_infer}/{total_infer}"
                      f" | {rate:.1f} img/s | ETA ~{eta/60:.1f} min      ", end="", flush=True)
        fp_per_negative_image = fp_neg / len(src_neg) if src_neg else 0.0

        results[cond_name] = {
            **ap,
            "precision": round(precision, 6),
            "recall": round(recall, 6),
            "fp_per_image": round(fp_per_image, 6),
            "fp_per_negative_image": round(fp_per_negative_image, 6),
            "n_images": len(src_pos),
            "n_negative_images": len(src_neg),
        }

        if verbose:
            r = results[cond_name]
            print(f"\r        {cond_name:<18} AP50 {r['AP50']:.4f} | AP50-95 {r['AP50_95']:.4f}"
                  f" | R {r['recall']:.3f} | FP/img {r['fp_per_image']:.3f}"
                  f" | FP/neg {r['fp_per_negative_image']:.3f}"
                  f"   ({time.time()-t_cond:.0f}s)", flush=True)

    if verbose:
        print(f"[eval] done in {(time.time()-t_start)/60:.1f} min", flush=True)
    return results


# --------------------------------------------------------------------------- #
# model adapters  (lazy imports: YOLO and RF-DETR live in different venvs)
# --------------------------------------------------------------------------- #
def make_yolo_predict_fn(
    model_path: str,
    imgsz: int = 1280,
    conf: float = DETECT_THRESHOLD,
    device: str | int | None = None,
) -> PredictFn:
    from ultralytics import YOLO

    model = YOLO(model_path)

    def _predict(img_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        r = model.predict(img_bgr, imgsz=imgsz, conf=conf, device=device, verbose=False)[0]
        if r.boxes is None or len(r.boxes) == 0:
            return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32)
        return (
            r.boxes.xyxy.cpu().numpy().astype(np.float32),
            r.boxes.conf.cpu().numpy().astype(np.float32),
        )

    return _predict


def make_rfdetr_predict_fn(
    checkpoint_path: str,
    threshold: float = DETECT_THRESHOLD,
    device: str = "cuda",
    optimize: bool = False,
) -> PredictFn:
    """optimize=False by default: optimize_for_inference() is deprecated, floods the
    console with TracerWarnings, and spends a long time on torch.jit.trace before the
    first image is even processed."""
    from rfdetr import from_checkpoint

    warnings.filterwarnings("ignore", category=FutureWarning)
    warnings.filterwarnings("ignore", category=UserWarning)

    try:
        model = from_checkpoint(checkpoint_path, device=device)
    except TypeError:  # older/newer signature without device kwarg
        model = from_checkpoint(checkpoint_path)
    if optimize:
        with contextlib.suppress(Exception):
            model.optimize_for_inference()

    def _predict(img_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)  # same as ros_yolo.py
        det = model.predict(rgb, threshold=threshold)
        if det is None or len(det) == 0:
            return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32)
        return (
            np.asarray(det.xyxy, dtype=np.float32).reshape(-1, 4),
            np.asarray(det.confidence, dtype=np.float32).reshape(-1),
        )

    return _predict


__all__ = [
    "run_evaluation",
    "make_yolo_predict_fn",
    "make_rfdetr_predict_fn",
    "DEFAULT_CONDITIONS",
    "make_rng",
    "iou_matrix",
    "greedy_match",
    "coco_ap",
]
