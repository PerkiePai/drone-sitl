"""The derotation attitude must be one the accelerometer cannot corrupt.

`MahonyState`'s gravity correction treats the accelerometer reading as the
gravity direction. An accelerometer measures SPECIFIC FORCE, so while the
aircraft accelerates horizontally that reading is tilted by atan(a/g) from
true down and the correction pulls the attitude estimate toward it.

That error would be tolerable in the absolute attitude alone. It is not
tolerable in the INTER-FRAME rotation the flow solve derotates by: an attitude
error changing at w rad/s subtracts a rotation that did not happen, and at
height h the solve reads the leftover flow as a translation of h*w m/s. At the
49 m hover this project flies, the ~0.4 rad/s the correction can reach is
metres per second of motion that never occurred -- fabricated out of a still
hover, and then flown for real by a position controller trying to cancel it.

Measured on run 20260822-poshold: predicted false velocity from the tilt rate
h*d(atan(a/g))/dt came to mean 5.75 m/s against an observed estimator error
rate of 6.69 m/s, over 1135 samples of the GPS-denied hold.
"""
import importlib.util
import os
import sys

import cv2
import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)


def _load_pipeline():
    spec = importlib.util.spec_from_file_location(
        "pipeline_streaming", os.path.join(ROOT, "pipeline-streaming.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# A nadir camera looking straight down with FLU body axes: the pipeline's own
# convention (drone_setup_px4_cesium.py builds R_CtoI the same way).
R_CTOI = np.array([[0.0, -1.0, 0.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0]])
META = {"K": [[318.84, 0.0, 480.0], [0.0, 318.84, 300.0], [0.0, 0.0, 1.0]],
        "R_CtoI": R_CTOI.tolist(), "heading_deg": 90.0}

HOVER_ALT_M = 49.0
LEVEL_FRD_ACCEL = [0.0, 0.0, -9.81]
"""Specific force at rest and level: FRD z is down, specific force points up."""
ACCELERATING_FRD_ACCEL = [5.0, 0.0, -9.81]
"""The same airframe accelerating forward at 5 m/s^2. Nothing has rotated --
the gyro below says so -- but this reading is tilted 27 deg from true down."""


def _texture():
    """A frame with enough corners for the LK tracker to lock onto."""
    rng = np.random.default_rng(7)
    img = rng.integers(0, 255, size=(60, 96), dtype=np.uint8)
    img = cv2.resize(img, (960, 600), interpolation=cv2.INTER_LINEAR)
    return cv2.imencode(".jpg", img)[1].tobytes()


def _hovering_estimator(ps):
    est = ps.Estimator(META)
    est.on_baro({"alt_m": 0.0})
    est.on_baro({"alt_m": HOVER_ALT_M})
    return est


def test_the_mahony_attitude_is_corrupted_by_horizontal_acceleration():
    """The sensor reality this exists to work around, asserted so the fix
    below cannot be mistaken for having removed it."""
    ps = _load_pipeline()
    est = _hovering_estimator(ps)
    for _ in range(400):                       # 2 s at 200 Hz, not rotating
        est.on_imu({"w": [0.0, 0.0, 0.0], "a": ACCELERATING_FRD_ACCEL}, 0.005)

    R = est.state.R_flu()
    tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
    assert tilt > 10.0, f"expected the gravity term to tilt the estimate, got {tilt:.1f} deg"


def test_a_still_camera_under_acceleration_solves_no_translation():
    """The whole bug in one test. Two IDENTICAL frames -- the camera has not
    moved and has not rotated -- with a lying accelerometer in between. The
    solved position must not move."""
    ps = _load_pipeline()
    est = _hovering_estimator(ps)
    jpg = _texture()

    est.on_frame({"jpg": jpg, "ts_ns": 0, "frame": 0})
    for _ in range(23):                        # one 8.7 fps frame interval
        est.on_imu({"w": [0.0, 0.0, 0.0], "a": ACCELERATING_FRD_ACCEL}, 0.005)
    est.on_frame({"jpg": jpg, "ts_ns": 115_000_000, "frame": 1})

    moved = float(np.hypot(est.pos[0], est.pos[1]))
    assert est.n_solved == 1, "the frame pair did not solve at all"
    assert moved < 0.5, f"fabricated {moved:.1f} m of translation from a still camera"


def test_a_still_camera_at_rest_solves_no_translation():
    """The control: with an honest accelerometer the same pair already solves
    ~zero, so the test above is measuring the gravity term and nothing else."""
    ps = _load_pipeline()
    est = _hovering_estimator(ps)
    jpg = _texture()

    est.on_frame({"jpg": jpg, "ts_ns": 0, "frame": 0})
    for _ in range(23):
        est.on_imu({"w": [0.0, 0.0, 0.0], "a": LEVEL_FRD_ACCEL}, 0.005)
    est.on_frame({"jpg": jpg, "ts_ns": 115_000_000, "frame": 1})

    assert est.n_solved == 1
    assert float(np.hypot(est.pos[0], est.pos[1])) < 0.5


def test_a_real_rotation_is_still_derotated():
    """The fix must not throw the derotation away. A camera that genuinely
    yaws between two frames sees its features sweep; if that rotation is not
    subtracted the solve reads the sweep as translation."""
    ps = _load_pipeline()
    est = _hovering_estimator(ps)
    img = cv2.imdecode(np.frombuffer(_texture(), np.uint8), cv2.IMREAD_GRAYSCALE)
    rot = cv2.warpAffine(
        img, cv2.getRotationMatrix2D((480.0, 300.0), -2.0, 1.0), (960, 600))

    est.on_frame({"jpg": _texture(), "ts_ns": 0, "frame": 0})
    # 2 deg about body-down over the interval, honestly reported by the gyro.
    for _ in range(23):
        est.on_imu({"w": [0.0, 0.0, np.radians(2.0) / 0.115],
                    "a": LEVEL_FRD_ACCEL}, 0.005)
    est.on_frame({"jpg": cv2.imencode(".jpg", rot)[1].tobytes(),
                  "ts_ns": 115_000_000, "frame": 1})

    assert est.n_solved == 1
    moved = float(np.hypot(est.pos[0], est.pos[1]))
    assert moved < 2.0, f"a pure yaw was read as {moved:.1f} m of translation"
