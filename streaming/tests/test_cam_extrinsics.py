"""The derived down_cam extrinsic must match a real recorded calibration.

vio-streamer.py derives the extrinsic from authored stage geometry instead of
measuring it. That derivation is only trustworthy if it reproduces what the
recorder actually measured on a live stage -- which is what these tests pin.
"""
import glob
import json
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import cam_extrinsics  # noqa: E402

DOWN_IMG_ROLL_DEG = -90.0
"""drone_setup_px4_cesium.py:66. The whole down_cam extrinsic, when rigid."""


def recorded_calibs():
    """Every dataset whose calib claims a constant (VIO-valid) extrinsic."""
    out = []
    for d in sorted(glob.glob(os.path.expanduser("~/vio_dataset/*"))):
        path = os.path.join(d, "cam_calib.json")
        if not os.path.isfile(path):
            continue
        try:
            with open(path) as fh:
                calib = json.load(fh)
        except (OSError, ValueError):
            continue
        if calib.get("extrinsic_is_constant") and "extrinsic_body_to_cam" in calib:
            out.append((os.path.basename(d), calib))
    return out


def test_analytic_extrinsic_is_a_quarter_turn_about_the_optical_axis():
    R = cam_extrinsics.analytic_R_body_cam(DOWN_IMG_ROLL_DEG)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert cam_extrinsics.rot_angle_deg(R, np.eye(3)) == pytest.approx(90.0)
    assert np.allclose(R @ [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], atol=1e-12)


def test_pipeline_convention_fix_matches_load_dataset():
    """load_dataset:75 post-multiplies by diag(1,-1,-1); nothing else."""
    R = cam_extrinsics.analytic_R_body_cam(DOWN_IMG_ROLL_DEG)
    assert np.allclose(cam_extrinsics.to_pipeline_R_CtoI(R),
                       R @ np.diag([1.0, -1.0, -1.0]), atol=1e-15)


def test_flu_to_frd_is_self_inverse():
    assert np.allclose(cam_extrinsics.FLU_TO_FRD @ cam_extrinsics.FLU_TO_FRD,
                       np.eye(3), atol=1e-15)


@pytest.mark.skipif(not recorded_calibs(),
                    reason="no recorded cam_calib.json with a constant extrinsic")
def test_derived_extrinsic_matches_every_recorded_calibration():
    """The check vio-streamer.py runs live at startup, run offline instead.

    Its 1 deg gate is the same one; observed agreement is ~0.01 deg, so a
    failure here means the mount geometry moved, not that the bar is tight.
    """
    analytic = cam_extrinsics.analytic_R_body_cam(DOWN_IMG_ROLL_DEG)
    for name, calib in recorded_calibs():
        qw, qx, qy, qz = calib["extrinsic_body_to_cam"]["quaternion_wxyz"]
        measured = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        err = cam_extrinsics.rot_angle_deg(analytic, measured)
        assert err < 1.0, f"{name}: derived extrinsic is {err:.3f} deg off"
