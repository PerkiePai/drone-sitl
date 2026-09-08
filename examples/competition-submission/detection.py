"""detection.py -- everything about the detection model, nothing about
flying. Loads a .pt weights file and runs inference on frames handed to
it by an Agent's on_frame. See ../../docs/superpowers/specs/
2026-09-08-competitor-submission-docker-design.md Decision 3 for why this
is a separate file from the Agent.

Reference shape only: swap _load()/detect() for whatever your model
actually needs (a custom torchvision model, ONNX runtime, ...). The
Detection/Detector split -- one small dataclass out, one class in -- is
what matters here, not this exact model format.

Assumes an ultralytics-style YOLO checkpoint since that's the most common
shape for a car/person detector at this scale; not a repo dependency,
just what this example was written against.
"""
from dataclasses import dataclass

import torch


@dataclass
class Detection:
    class_name: str
    confidence: float
    bbox: tuple            # (x1, y1, x2, y2) pixels in the source image


class Detector:
    """One model. Competitors using more than one model (e.g. a car
    detector and a person detector) just instantiate this twice -- see
    the design doc's Decision 3. No multi-weight/ensemble API here on
    purpose."""

    def __init__(self, weights_path, device=None):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self._load(weights_path)

    def _load(self, weights_path):
        from ultralytics import YOLO           # noqa: E402 -- deferred, optional dep
        model = YOLO(weights_path)
        model.to(self.device)
        return model

    def detect(self, image):
        """image: RGB numpy array, exactly what on_frame receives. Returns
        a list of Detection, empty if nothing crossed the model's own
        confidence floor. Never raises on an empty/None image -- caller
        (on_frame) is expected to skip calling this when image is None."""
        results = self.model.predict(image, device=self.device, verbose=False)
        out = []
        for r in results:
            for box in r.boxes:
                out.append(Detection(
                    class_name=r.names[int(box.cls)],
                    confidence=float(box.conf),
                    bbox=tuple(float(v) for v in box.xyxy[0]),
                ))
        return out
