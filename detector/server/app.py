"""Minimal HTTP API around TruckDetector (FastAPI). Starting point for the web integration.

Run (from the package root):
    pip install -r requirements-web.txt
    uvicorn server.app:app --host 0.0.0.0 --port 8000        # ONE worker per GPU (the model is loaded per process)

Endpoints:
    GET  /health                      -> status, device, weights, classes
    POST /detect?conf=0.25            -> multipart field "file" (jpg/png)  ->  JSON detections
    POST /detect/annotated?conf=0.25  -> same input -> image/jpeg with boxes drawn

Env vars: TRUCK_WEIGHTS (path to .pth), TRUCK_CONF (default threshold), TRUCK_DEVICE (cuda|cpu), MAX_UPLOAD_MB (default 20).
"""

import os
import sys

import cv2
import numpy as np
from fastapi import FastAPI, File, HTTPException, Query, UploadFile
from fastapi.responses import Response

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from truck_detector import CLASS_NAMES, DEFAULT_CONF, TruckDetector  # noqa: E402

MAX_UPLOAD_BYTES = int(float(os.environ.get("MAX_UPLOAD_MB", "20")) * 1024 * 1024)

app = FastAPI(title="Truck detector", version="1.0")
detector = None


@app.on_event("startup")
def _load():
    global detector
    detector = TruckDetector(device=os.environ.get("TRUCK_DEVICE"),
                             conf=float(os.environ.get("TRUCK_CONF", DEFAULT_CONF)))
    # warm-up so the first real request is not slow
    detector.predict(np.zeros((720, 1280, 3), np.uint8))


async def _decode(file: UploadFile):
    data = await file.read(MAX_UPLOAD_BYTES + 1)
    if len(data) > MAX_UPLOAD_BYTES:
        raise HTTPException(413, f"file larger than {MAX_UPLOAD_BYTES // 1024 // 1024} MB")
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)     # -> BGR, drops alpha
    if img is None:
        raise HTTPException(400, "not a decodable image (send jpg/png)")
    return img


@app.get("/health")
def health():
    return dict(status="ok" if detector else "loading", device=getattr(detector, "device", None),
                weights=os.path.basename(getattr(detector, "weights", "")), classes=list(CLASS_NAMES),
                default_conf=getattr(detector, "conf", None))


@app.post("/detect")
async def detect(file: UploadFile = File(...), conf: float = Query(None, ge=0.0, le=1.0)):
    img = await _decode(file)
    with detector.lock:                                   # one inference at a time per process
        dets, ms = detector.predict_timed(img, conf=conf)
    h, w = img.shape[:2]
    return dict(image=dict(width=w, height=h), conf=conf if conf is not None else detector.conf,
                count=len(dets), infer_ms=round(ms, 1), detections=dets)


@app.post("/detect/annotated")
async def detect_annotated(file: UploadFile = File(...), conf: float = Query(None, ge=0.0, le=1.0)):
    img = await _decode(file)
    with detector.lock:
        dets = detector.predict(img, conf=conf)
    ok, buf = cv2.imencode(".jpg", detector.annotate(img, dets), [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise HTTPException(500, "jpeg encode failed")
    return Response(buf.tobytes(), media_type="image/jpeg", headers={"X-Detections": str(len(dets))})
