"""web_feed.py -- operator-side truck detector for the web UI.

Reads the Isaac MJPEG camera feed, runs TruckDetector on every frame, and
POSTs the boxes to joystick-server's /detector/boxes. The page's detection
view (web/js/detections.js) then draws them -- no agent run needed.

    detector/web_feed.py [--cam detect] [--conf 0.5] [--hz 15]

Needs the rfdetr environment (e.g. /home/innovation/Tiger/.venv/bin/python);
joystick-server itself stays free of torch. Boxes expire server-side after
1 s, so killing this script just makes the boxes disappear.
"""
import argparse
import json
import os
import sys
import time
import urllib.request

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from truck_detector import TruckDetector  # noqa: E402


def to_wire(dets):
    """TruckDetector dicts -> the harness/agent box shape the UI already draws."""
    return [{"class_name": d["class_name"], "confidence": d["confidence"],
             "bbox": [d["x1"], d["y1"], d["x2"], d["y2"]]} for d in dets]


def post(url, boxes):
    req = urllib.request.Request(url, json.dumps({"boxes": boxes}).encode(),
                                 {"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=2).read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video-port", type=int, default=8080)
    ap.add_argument("--server", default="http://127.0.0.1:8090")
    ap.add_argument("--cam", default="detect", help="MJPEG path: detect | chase | ...")
    ap.add_argument("--conf", type=float, default=0.5,
                    help="0.5 not 0.25: the model gives ~0.8 false boxes per empty image")
    ap.add_argument("--hz", type=float, default=15.0, help="max detections per second")
    a = ap.parse_args()

    feed = f"http://127.0.0.1:{a.video_port}/{a.cam}"
    sink = f"{a.server}/detector/boxes"
    det = TruckDetector(conf=a.conf)
    print(f">>> detector on {det.device}; {feed} -> {sink}", flush=True)
    det.predict(cv2.imread(os.path.join(os.path.dirname(__file__), "samples", "truck_0005.png")))  # warm-up

    period, last = 1.0 / a.hz, 0.0
    while True:
        cap = cv2.VideoCapture(feed)
        if not cap.isOpened():
            print(f">>> no feed at {feed}, retrying", flush=True)
            time.sleep(2)
            continue
        while True:
            ok, frame = cap.read()
            if not ok:
                break                      # feed dropped -> reconnect
            if time.monotonic() - last < period:
                continue                   # keep draining so we stay on the newest frame
            last = time.monotonic()
            try:
                post(sink, to_wire(det.predict(frame)))
            except OSError as e:
                print(f">>> post failed: {e}", flush=True)
                time.sleep(1)
        cap.release()


if __name__ == "__main__":
    main()
