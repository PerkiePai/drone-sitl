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


# --- CommandState ----------------------------------------------------------

def test_state_reports_held_velocity():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    assert s.velocity(now=100.1) == (2.0, 0.0, 0.0)


def test_state_release_returns_to_hover():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    s.set("fwd", False, now=100.2)
    assert s.velocity(now=100.3) == (0.0, 0.0, 0.0)


def test_watchdog_zeroes_stale_input():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    assert s.velocity(now=100.4) == (2.0, 0.0, 0.0)   # still fresh
    assert s.velocity(now=101.0) == (0.0, 0.0, 0.0)   # stale -> hover


def test_touch_keeps_a_held_direction_alive():
    """The page pings every 150 ms while a button is down; without that the
    watchdog would cut hold-to-move off after watchdog_s."""
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    s.touch(now=100.4)
    assert s.velocity(now=100.7) == (2.0, 0.0, 0.0)


def test_clear_drops_everything():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    s.clear()
    assert s.velocity(now=100.1) == (0.0, 0.0, 0.0)


def test_held_reports_current_set():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    s.set("up", True, now=100.0)
    assert s.held() == {"fwd", "up"}


def test_out_of_scope_direction_is_rejected():
    s = offboard.CommandState()
    with pytest.raises(ValueError):
        s.set("left", True)


# --- decode_px4_mode -------------------------------------------------------

def test_decode_offboard_mode():
    assert offboard.decode_px4_mode(6 << 16) == "OFFBOARD"


def test_decode_auto_takeoff_mode():
    assert offboard.decode_px4_mode((2 << 24) | (4 << 16)) == "AUTO.TAKEOFF"


def test_decode_auto_land_mode():
    assert offboard.decode_px4_mode((6 << 24) | (4 << 16)) == "AUTO.LAND"


def test_decode_posctl_mode():
    assert offboard.decode_px4_mode(3 << 16) == "POSCTL"


def test_decode_unknown_mode_is_readable_not_a_crash():
    assert "99" in offboard.decode_px4_mode(99 << 16)
