"""truck_detector.py - RF-DETR truck detector, no ROS / no Isaac Sim needed.

Usage (Python):
    import cv2
    from truck_detector import TruckDetector

    det = TruckDetector()                      # loads model/rfdetr_truck_medium_ema.pth
    img = cv2.imread("samples/truck_0005.png")  # BGR, as cv2 returns it
    for d in det.predict(img, conf=0.25):
        print(d["class_name"], d["confidence"], d["x1"], d["y1"], d["x2"], d["y2"])
    cv2.imwrite("out.jpg", det.annotate(img, det.predict(img)))

Contract:
  - input : numpy uint8 image, HxWx3. Default channel order is BGR (cv2.imread / cv2.VideoCapture);
            pass input_format="rgb" if you already have RGB (PIL, most web decoders).
  - output: list of dicts, pixel coordinates in the ORIGINAL image size (x1,y1 top-left, x2,y2 bottom-right),
            confidence 0..1, class_name "Truck". Sorted by confidence, highest first.
  - The model rescales internally (resolution 640), so any image size works; the training frames were 1280x720.

Thread safety: one TruckDetector instance is NOT safe to call from several threads at once
(use one lock, or one instance per worker process). server/app.py takes a lock.
"""

import os
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_WEIGHTS = os.path.join(HERE, "model", "rfdetr_truck_medium_ema.pth")
DEFAULT_CONF = 0.25          # same threshold the drone pipeline used (ros_detector.py --conf 0.25)
CLASS_NAMES = ("Truck",)


class TruckDetector:
    def __init__(self, weights=None, device=None, conf=DEFAULT_CONF, optimize=True):
        """weights: .pth checkpoint (default: the bundled one) | device: "cuda"/"cpu" (default: cuda if available)"""
        import torch
        from rfdetr import from_checkpoint          # same loader ros_detector.py used

        self.weights = os.path.abspath(weights or os.environ.get("TRUCK_WEIGHTS") or DEFAULT_WEIGHTS)
        if not os.path.isfile(self.weights):
            raise FileNotFoundError(f"weights not found: {self.weights}")
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.conf = float(conf)
        self.model = from_checkpoint(self.weights, device=self.device)
        self.optimized = False
        if optimize:
            try:
                self.model.optimize_for_inference()
                self.optimized = True
            except Exception as e:                    # optional speed-up; the model works without it
                print(f"[TruckDetector] optimize_for_inference skipped: {e}")
        self.lock = threading.Lock()                  # callers in threaded servers can use `with det.lock:`

    def predict(self, image, conf=None, input_format="bgr"):
        """Detect trucks in one image -> list of dicts (see module docstring)."""
        if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("image must be a numpy HxWx3 array")
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if input_format == "bgr" else image
        threshold = self.conf if conf is None else float(conf)
        det = self.model.predict(rgb, threshold=threshold)
        names = det.data.get("class_name", []) if getattr(det, "data", None) else []
        out = []
        for i in range(len(det)):
            x1, y1, x2, y2 = (float(v) for v in det.xyxy[i])
            out.append(dict(
                x1=round(x1, 1), y1=round(y1, 1), x2=round(x2, 1), y2=round(y2, 1),
                confidence=round(float(det.confidence[i]), 4),
                class_name=str(names[i]) if i < len(names) else CLASS_NAMES[0],
            ))
        out.sort(key=lambda d: -d["confidence"])
        return out

    def predict_timed(self, image, conf=None, input_format="bgr"):
        """Same as predict() but returns (detections, inference_ms)."""
        t = time.perf_counter()
        res = self.predict(image, conf=conf, input_format=input_format)
        return res, (time.perf_counter() - t) * 1000.0

    @staticmethod
    def annotate(image, detections, color=(0, 200, 255), thickness=2):
        """Draw boxes + "Truck 0.93" labels on a copy of a BGR image."""
        out = image.copy()
        for d in detections:
            p1, p2 = (int(d["x1"]), int(d["y1"])), (int(d["x2"]), int(d["y2"]))
            cv2.rectangle(out, p1, p2, color, thickness)
            label = f'{d["class_name"]} {d["confidence"]:.2f}'
            (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            y0 = max(p1[1], th + base)
            cv2.rectangle(out, (p1[0], y0 - th - base), (p1[0] + tw + 4, y0), color, -1)
            cv2.putText(out, label, (p1[0] + 2, y0 - base), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        return out


if __name__ == "__main__":          # quick CLI:  python truck_detector.py image.jpg [conf]
    import json
    import sys

    if len(sys.argv) < 2:
        raise SystemExit("usage: python truck_detector.py <image> [conf]")
    img = cv2.imread(sys.argv[1])
    if img is None:
        raise SystemExit(f"cannot read image: {sys.argv[1]}")
    d = TruckDetector()
    res, ms = d.predict_timed(img, conf=float(sys.argv[2]) if len(sys.argv) > 2 else None)
    print(json.dumps(dict(device=d.device, infer_ms=round(ms, 1), detections=res), indent=1))
