"""Unit tests for flow_odometry.MahonyState. No dataset, no sim."""
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from flow_odometry import MahonyState  # noqa: E402

GRAV_FRD_LEVEL = np.array([0.0, 0.0, -9.81])
"""Specific force read by a level FRD accelerometer at rest.

FRD's z axis points DOWN, and an accelerometer at rest measures specific force,
which points UP -- so the reading is negative on z.
"""


def test_identity_seed_holds_still_with_no_input():
    m = MahonyState(np.eye(3), Kp=0.0)
    R = m.update([0.0, 0.0, 0.0], GRAV_FRD_LEVEL, 0.005)
    assert np.allclose(R, np.eye(3), atol=1e-12)


def test_gyro_integrates_about_z():
    m = MahonyState(np.eye(3), Kp=0.0)
    for _ in range(100):
        m.update([0.0, 0.0, 1.0], GRAV_FRD_LEVEL, 0.01)   # 1 rad/s for 1 s
    ang = Rot.from_matrix(m.R).as_rotvec()
    assert ang[2] == pytest.approx(1.0, abs=1e-3)


def test_gravity_correction_levels_a_tilted_seed():
    """The whole point of the accelerometer term: a wrong initial roll/pitch is
    pulled back toward gravity, while yaw -- which gravity cannot observe --
    keeps whatever error it started with.

    The seed is a LEVEL attitude perturbed in roll, NOT np.eye(3) perturbed in
    roll. As an FRD->ENU rotation np.eye(3) is upside down (the singularity
    test below spells this out), so a seed near it converges toward roll=180 --
    correct filter behaviour measured against the wrong stable point.
    """
    m = MahonyState.from_heading(90.0, Kp=1.0)     # level, facing East: R_flu = I
    m.R = m.R @ Rot.from_euler("x", 10.0, degrees=True).as_matrix()
    for _ in range(2000):
        m.update([0.0, 0.0, 0.0], GRAV_FRD_LEVEL, 0.005)
    roll = Rot.from_matrix(m.R_flu()).as_euler("xyz", degrees=True)[0]
    assert abs(roll) < 1.0


def test_from_heading_seeds_yaw_and_is_level():
    """Compass 0=N/90=E, ENU yaw 0=E measured counter-clockwise, so the two
    differ by yaw_enu = 90 - heading."""
    m = MahonyState.from_heading(90.0)              # due East
    yaw = np.arctan2(m.R_flu()[1, 0], m.R_flu()[0, 0])
    assert yaw == pytest.approx(0.0, abs=1e-9)
    m = MahonyState.from_heading(0.0)               # due North
    yaw = np.arctan2(m.R_flu()[1, 0], m.R_flu()[0, 0])
    assert yaw == pytest.approx(np.pi / 2, abs=1e-9)


def test_from_heading_is_not_the_identity_singularity():
    """np.eye(3) as an FRD->ENU seed puts the body upside down: FRD z (down)
    lands on ENU z (up), so v_pred is ANTIPARALLEL to the measured gravity
    direction, their cross product is zero, and the correction that is supposed
    to level the filter is dead on arrival. from_heading must not do that."""
    m = MahonyState.from_heading(0.0)
    v_pred = m.R.T @ np.array([0.0, 0.0, 9.81])
    v_meas = GRAV_FRD_LEVEL / np.linalg.norm(GRAV_FRD_LEVEL)
    assert np.linalg.norm(np.cross(v_meas, v_pred / np.linalg.norm(v_pred))) < 1e-9
    assert v_pred[2] < 0            # agrees with the reading, not opposed to it


def test_dt_outlier_is_clamped_like_the_batch_loop():
    """flow_odometry.py:164 substitutes 0.004 s for a non-positive or >0.1 s
    gap. Streaming sees those on ZMQ reconnects, so the guard must live in
    update(), not in the batch caller."""
    a = MahonyState(np.eye(3), Kp=0.0)
    a.update([0.0, 0.0, 1.0], GRAV_FRD_LEVEL, 5.0)
    b = MahonyState(np.eye(3), Kp=0.0)
    b.update([0.0, 0.0, 1.0], GRAV_FRD_LEVEL, 0.004)
    assert np.allclose(a.R, b.R)
