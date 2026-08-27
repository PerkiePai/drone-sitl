"""Unit tests for streaming/offboard.py. No PX4 and no Isaac Sim required."""
import math
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
    assert s.command(now=100.1) == (2.0, 0.0, 0.0, 0.0)


def test_state_release_returns_to_hover():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    s.set("fwd", False, now=100.2)
    assert s.command(now=100.3) == (0.0, 0.0, 0.0, 0.0)


def test_watchdog_zeroes_stale_input():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    assert s.command(now=100.4) == (2.0, 0.0, 0.0, 0.0)   # still fresh
    assert s.command(now=101.0) == (0.0, 0.0, 0.0, 0.0)   # stale -> hover


def test_touch_keeps_a_held_direction_alive():
    """The page pings every 150 ms while a button is down; without that the
    watchdog would cut hold-to-move off after watchdog_s."""
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    s.touch(now=100.4)
    assert s.command(now=100.7) == (2.0, 0.0, 0.0, 0.0)


def test_clear_drops_everything():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    s.clear()
    assert s.command(now=100.1) == (0.0, 0.0, 0.0, 0.0)


def test_held_reports_current_set():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
    s.set("fwd", True, now=100.0)
    s.set("up", True, now=100.0)
    assert s.held() == {"fwd", "up"}


def test_out_of_scope_direction_is_rejected():
    s = offboard.CommandState()
    with pytest.raises(ValueError):
        s.set("strafe", True)


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


# --- OffboardLink ----------------------------------------------------------

def test_send_velocity_uses_body_ned_frame_and_velocity_mask():
    conn = MagicMock()
    offboard.OffboardLink(conn).send_velocity(2.0, 0.0, -1.0)
    args, _ = conn.mav.set_position_target_local_ned_send.call_args
    (_ms, _sys, _comp, frame, mask,
     _x, _y, _z, vx, vy, vz, _ax, _ay, _az, _yaw, yaw_rate) = args
    assert frame == offboard.MAV_FRAME_BODY_NED
    assert mask == offboard.VEL_YAWRATE_TYPE_MASK
    assert (vx, vy, vz) == (2.0, 0.0, -1.0)
    assert yaw_rate == 0.0      # zero yaw_rate is what holds the heading


def test_offboard_sets_custom_main_mode_6():
    conn = MagicMock()
    offboard.OffboardLink(conn).offboard()
    args, _ = conn.mav.command_long_send.call_args
    _sys, _comp, command, _conf, p1, p2, p3, _p4, _p5, _p6, _p7 = args
    assert command == offboard.MAV_CMD_DO_SET_MODE
    assert p1 == float(offboard.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    assert p2 == float(offboard.PX4_MAIN_MODE_OFFBOARD)
    assert p3 == 0.0


def test_takeoff_uses_auto_takeoff_submode():
    conn = MagicMock()
    offboard.OffboardLink(conn).takeoff()
    args, _ = conn.mav.command_long_send.call_args
    assert args[5] == float(offboard.PX4_MAIN_MODE_AUTO)
    assert args[6] == float(offboard.PX4_SUB_MODE_AUTO_TAKEOFF)


def test_land_uses_auto_land_submode():
    conn = MagicMock()
    offboard.OffboardLink(conn).land()
    args, _ = conn.mav.command_long_send.call_args
    assert args[5] == float(offboard.PX4_MAIN_MODE_AUTO)
    assert args[6] == float(offboard.PX4_SUB_MODE_AUTO_LAND)


def test_arm_then_disarm():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.arm()
    args, _ = conn.mav.command_long_send.call_args
    assert args[2] == offboard.MAV_CMD_COMPONENT_ARM_DISARM
    assert args[4] == 1.0
    link.disarm()
    args, _ = conn.mav.command_long_send.call_args
    assert args[4] == 0.0


def test_rc_loss_exception_param_is_sent_as_int32():
    conn = MagicMock()
    offboard.OffboardLink(conn).set_param(
        "COM_RCL_EXCEPT", offboard.COM_RCL_EXCEPT_OFFBOARD,
        offboard.MAV_PARAM_TYPE_INT32)
    args, _ = conn.mav.param_set_send.call_args
    _sys, _comp, param_id, value, param_type = args
    assert param_id == b"COM_RCL_EXCEPT"
    assert value == 4.0
    assert param_type == offboard.MAV_PARAM_TYPE_INT32


def test_bind_target_adopts_ids_from_heartbeat():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    msg = MagicMock()
    msg.get_srcSystem.return_value = 3
    msg.get_srcComponent.return_value = 7
    link.bind_target(msg)
    assert (link.target_system, link.target_component) == (3, 7)


# --- yaw (left/right turn the aircraft; they do NOT strafe) -----------------

def test_yaw_right_is_positive_rate():
    """NED yaw is positive clockwise seen from above, so right turn > 0."""
    assert offboard.axes_to_yaw_rate({"yaw_right"}, 0.5) == 0.5


def test_yaw_left_is_negative_rate():
    assert offboard.axes_to_yaw_rate({"yaw_left"}, 0.5) == -0.5


def test_opposing_yaw_cancels():
    assert offboard.axes_to_yaw_rate({"yaw_left", "yaw_right"}, 0.5) == 0.0


def test_no_turn_held_is_zero_yaw_rate():
    assert offboard.axes_to_yaw_rate(set(), 0.5) == 0.0


def test_yaw_does_not_produce_any_translation():
    """Turning must not sneak in sideways motion -- the whole point of putting
    yaw on left/right instead of strafe."""
    assert offboard.axes_to_body_velocity({"yaw_left"}, 2.0, 1.0) == (0.0, 0.0, 0.0)
    assert offboard.axes_to_body_velocity({"yaw_right"}, 2.0, 1.0) == (0.0, 0.0, 0.0)


def test_command_combines_forward_climb_and_turn():
    import math
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0)
    for d in ("fwd", "up", "yaw_right"):
        s.set(d, True, now=100.0)
    vx, vy, vz, yr = s.command(now=100.1)
    assert (vx, vy, vz) == (2.0, 0.0, -1.0)
    assert math.isclose(yr, math.radians(45.0))


def test_watchdog_also_stops_the_turn():
    """A stale link must not leave the aircraft spinning."""
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0)
    s.set("yaw_right", True, now=100.0)
    assert s.command(now=101.0) == (0.0, 0.0, 0.0, 0.0)


def test_send_velocity_forwards_yaw_rate():
    conn = MagicMock()
    offboard.OffboardLink(conn).send_velocity(1.0, 0.0, 0.0, yaw_rate=0.75)
    args, _ = conn.mav.set_position_target_local_ned_send.call_args
    assert args[15] == 0.75          # yaw_rate is the last field


def test_send_velocity_defaults_to_holding_heading():
    conn = MagicMock()
    offboard.OffboardLink(conn).send_velocity(1.0, 0.0, 0.0)
    args, _ = conn.mav.set_position_target_local_ned_send.call_args
    assert args[15] == 0.0


def test_position_constants_match_pymavlink():
    from pymavlink.dialects.v20 import common as m
    assert (offboard.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            == m.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
    # position and yaw USED; velocity, acceleration and yaw_rate ignored
    expected = (m.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | m.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    assert offboard.POS_YAW_TYPE_MASK == expected == 2552
    assert not (offboard.POS_YAW_TYPE_MASK
                & m.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
    assert not (offboard.POS_YAW_TYPE_MASK
                & m.POSITION_TARGET_TYPEMASK_X_IGNORE)


def test_send_position_global_scales_degrees_to_1e7():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.send_position_global(40.7128, -74.0060, 12.0, 90.0)
    args = conn.mav.set_position_target_global_int_send.call_args[0]
    assert args[3] == offboard.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert args[4] == offboard.POS_YAW_TYPE_MASK
    assert args[5] == 407128000        # lat_int
    assert args[6] == -740060000       # lon_int
    assert args[7] == 12.0             # alt, relative to home


def test_send_position_global_converts_yaw_to_radians():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.send_position_global(40.0, -74.0, 5.0, 90.0)
    args = conn.mav.set_position_target_global_int_send.call_args[0]
    assert abs(args[14] - math.pi / 2) < 1e-9    # yaw
    assert args[15] == 0.0                       # yaw_rate, masked off


def test_send_position_global_zeroes_the_masked_fields():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.send_position_global(40.0, -74.0, 5.0, 0.0)
    args = conn.mav.set_position_target_global_int_send.call_args[0]
    assert args[8:14] == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)   # vx..afz


# --- send_velocity_world (agent VelocityWorld) ---------------------------

def test_local_ned_constant_matches_pymavlink():
    from pymavlink.dialects.v20 import common as m
    assert offboard.MAV_FRAME_LOCAL_NED == m.MAV_FRAME_LOCAL_NED


def test_send_velocity_world_uses_local_ned_frame_and_passes_ned_through():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.send_velocity_world(2.0, -1.0, -1.5, yaw_rate=0.3)

    (_ms, _sys, _comp, frame, mask,
     _x, _y, _z, vx, vy, vz, _ax, _ay, _az, _yaw, yaw_rate) = \
        conn.mav.set_position_target_local_ned_send.call_args[0]
    assert frame == offboard.MAV_FRAME_LOCAL_NED
    assert mask == offboard.VEL_YAWRATE_TYPE_MASK
    assert (vx, vy, vz) == (2.0, -1.0, -1.5)      # already NED; not re-flipped
    assert yaw_rate == 0.3
