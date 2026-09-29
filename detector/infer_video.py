"""Run the detector over a video file -> annotated mp4 + detections.jsonl (no ROS).

    python infer_video.py input.mp4 [--out out.mp4] [--jsonl dets.jsonl] [--conf 0.25] [--every 1]

detections.jsonl: one line per processed frame, same shape the drone pipeline wrote:
    {"frame": 12, "t": 0.4, "infer_ms": 14.6, "n": 1, "boxes": [[x1, y1, x2, y2, conf, "Truck"], ...]}
"""

import argparse
import json
import os
import time

import cv2

from truck_detector import TruckDetector


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--out", default=None)
    ap.add_argument("--jsonl", default=None)
    ap.add_argument("--conf", type=float, default=None)
    ap.add_argument("--every", type=int, default=1, help="process every Nth frame (others copied unannotated)")
    a = ap.parse_args()

    stem = os.path.splitext(a.video)[0]
    out_path, jsonl_path = a.out or stem + "_det.mp4", a.jsonl or stem + "_det.jsonl"
    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {a.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    det = TruckDetector()
    n = hits = 0
    t0 = time.time()
    with open(jsonl_path, "w") as jf:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            n += 1
            if (n - 1) % max(1, a.every) == 0:
                dets, ms = det.predict_timed(frame, conf=a.conf)
                hits += bool(dets)
                jf.write(json.dumps(dict(frame=n, t=round((n - 1) / fps, 3), infer_ms=round(ms, 2), n=len(dets),
                                         boxes=[[d["x1"], d["y1"], d["x2"], d["y2"], d["confidence"], d["class_name"]]
                                                for d in dets])) + "\n")
                frame = det.annotate(frame, dets)
            writer.write(frame)
    cap.release()
    writer.release()
    print(f"{n} frames, {hits} with detections, {n / max(time.time() - t0, 1e-9):.1f} fps -> {out_path}, {jsonl_path}")


if __name__ == "__main__":
    main()
