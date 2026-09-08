"""Mock Detector -- no real model, no torch/ultralytics. Proves the
agent.py/detection.py wiring (weights "loading", the detect() call shape,
the camera-switch-on-a-hit path in full_flight_agent.py) without needing a
real trained model, GPU, or heavy pip install. See
docs/superpowers/specs/2026-09-08-competitor-submission-docker-design.md.

Same Detection/Detector shape as examples/competition-submission/detection.py
-- this is a stand-in for that file's _load()/detect(), not a different API.
"""
from dataclasses import dataclass


@dataclass
class Detection:
    class_name: str
    confidence: float
    bbox: tuple            # (x1, y1, x2, y2) pixels in the source image


class Detector:
    """Fakes a detector: "loads" weights (just proves the file made it into
    the image) and reports one confident detection every FRAMES_BETWEEN_HITS
    calls, so the on_frame -> camera-switch -> orbit path in
    full_flight_agent.py fires deterministically instead of depending on
    the sim actually rendering something recognizable."""

    FRAMES_BETWEEN_HITS = 15

    def __init__(self, weights_path, device=None):
        with open(weights_path) as f:
            self._marker = f.read().strip()
        self._calls = 0

    def detect(self, image):
        self._calls += 1
        if self._calls % self.FRAMES_BETWEEN_HITS == 0:
            return [Detection(class_name="mock-target", confidence=0.99,
                              bbox=(0.0, 0.0, 10.0, 10.0))]
        return []
