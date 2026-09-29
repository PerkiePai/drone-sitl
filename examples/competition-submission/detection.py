"""detection.py -- everything about the detection model, nothing about
flying. Loads a checkpoint and runs inference on frames handed to it by an
Agent's on_frame. See ../../docs/superpowers/specs/
2026-09-08-competitor-submission-docker-design.md Decision 3 for why this
is a separate file from the Agent.

Reference shape only: swap _load()/detect() for whatever your model
actually needs (a custom torchvision model, ONNX runtime, ...). The
Detection/Detector split -- one small dataclass out, one class in -- is
what matters here, not this exact model format.

Wraps an RF-DETR checkpoint (rfdetr.from_checkpoint) since that's what
this example was actually trained and validated against (RF-DETR Medium,
single class "Truck"); not a repo dependency, just what weights.pt is.
"""
from dataclasses import dataclass

CONF_THRESHOLD = 0.25


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
        import torch
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self._load(weights_path)

    def _load(self, weights_path):
        from rfdetr import from_checkpoint   # noqa: E402 -- deferred, optional dep
        return from_checkpoint(weights_path, device=self.device)

    def detect(self, image):
        """image: RGB numpy array, exactly what on_frame receives. Returns
        a list of Detection, empty if nothing crossed CONF_THRESHOLD. Never
        raises on an empty/None image -- caller (on_frame) is expected to
        skip calling this when image is None."""
        result = self.model.predict(image, threshold=CONF_THRESHOLD)
        out = []
        for bbox, confidence, class_id in zip(result.xyxy, result.confidence, result.class_id):
            out.append(Detection(
                class_name=self.model.class_names[int(class_id)],
                confidence=float(confidence),
                bbox=tuple(float(v) for v in bbox),
            ))
        return out
