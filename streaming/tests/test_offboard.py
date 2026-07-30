"""Unit tests for streaming/offboard.py. No PX4 and no Isaac Sim required."""
import os
import sys

import pytest
from unittest.mock import MagicMock

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import offboard  # noqa: E402


# --- axes_to_body_velocity -------------------------------------------------

def test_forward_is_positive_vx():
    assert offboard.axes_to_body_velocity({"fwd"}, 2.0, 1.0) == (2.0, 0.0, 0.0)


def test_backward_is_negative_vx():
    assert offboard.axes_to_body_velocity({"back"}, 2.0, 1.0) == (-2.0, 0.0, 0.0)


def test_climb_is_negative_vz_because_ned_down_is_positive():
    assert offboard.axes_to_body_velocity({"up"}, 2.0, 1.0) == (0.0, 0.0, -1.0)


def test_descend_is_positive_vz():
    assert offboard.axes_to_body_velocity({"down"}, 2.0, 1.0) == (0.0, 0.0, 1.0)


def test_nothing_held_is_hover():
    assert offboard.axes_to_body_velocity(set(), 2.0, 1.0) == (0.0, 0.0, 0.0)


def test_forward_and_climb_combine():
    assert offboard.axes_to_body_velocity({"fwd", "up"}, 2.0, 1.0) == (2.0, 0.0, -1.0)


def test_opposing_directions_cancel():
    assert offboard.axes_to_body_velocity(
        {"fwd", "back", "up", "down"}, 2.0, 1.0) == (0.0, 0.0, 0.0)


def test_vy_is_always_zero_because_strafe_is_out_of_scope():
    for d in offboard.DIRECTIONS:
        assert offboard.axes_to_body_velocity({d}, 2.0, 1.0)[1] == 0.0


def test_local_constants_match_pymavlink():
    """offboard.py hard-codes these for readability. If pymavlink's dialect
    ever disagrees, fail here rather than silently send a wrong command."""
    from pymavlink.dialects.v20 import common as m
    assert offboard.MAV_FRAME_BODY_NED == m.MAV_FRAME_BODY_NED
    assert offboard.MAV_CMD_DO_SET_MODE == m.MAV_CMD_DO_SET_MODE
    assert offboard.MAV_CMD_COMPONENT_ARM_DISARM == m.MAV_CMD_COMPONENT_ARM_DISARM
    assert offboard.MAV_PARAM_TYPE_INT32 == m.MAV_PARAM_TYPE_INT32
    assert offboard.MAV_PARAM_TYPE_REAL32 == m.MAV_PARAM_TYPE_REAL32
    assert (offboard.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            == m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
