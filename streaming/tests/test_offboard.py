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

def test_full_forward_pitch_is_positive_vx():
    assert offboard.axes_to_body_velocity(1.0, 0.0, 0.0, 2.0, 1.5, 1.0) == (2.0, 0.0, 0.0)


def test_full_back_pitch_is_negative_vx():
    assert offboard.axes_to_body_velocity(-1.0, 0.0, 0.0, 2.0, 1.5, 1.0) == (-2.0, 0.0, 0.0)


def test_pitch_is_proportional_to_deflection():
    assert offboard.axes_to_body_velocity(0.5, 0.0, 0.0, 2.0, 1.5, 1.0) == (1.0, 0.0, 0.0)


def test_full_right_roll_is_positive_vy():
    assert offboard.axes_to_body_velocity(0.0, 1.0, 0.0, 2.0, 1.5, 1.0) == (0.0, 1.5, 0.0)


def test_full_left_roll_is_negative_vy():
    assert offboard.axes_to_body_velocity(0.0, -1.0, 0.0, 2.0, 1.5, 1.0) == (0.0, -1.5, 0.0)


def test_roll_is_proportional_to_deflection():
    assert offboard.axes_to_body_velocity(0.0, 0.5, 0.0, 2.0, 1.5, 1.0) == (0.0, 0.75, 0.0)


def test_full_up_thrust_is_negative_vz_because_ned_down_is_positive():
    assert offboard.axes_to_body_velocity(0.0, 0.0, 1.0, 2.0, 1.5, 1.0) == (0.0, 0.0, -1.0)


def test_full_down_thrust_is_positive_vz():
    assert offboard.axes_to_body_velocity(0.0, 0.0, -1.0, 2.0, 1.5, 1.0) == (0.0, 0.0, 1.0)


def test_zero_axes_is_hover():
    assert offboard.axes_to_body_velocity(0.0, 0.0, 0.0, 2.0, 1.5, 1.0) == (0.0, 0.0, 0.0)


def test_pitch_roll_and_thrust_combine():
    assert offboard.axes_to_body_velocity(1.0, 0.5, 1.0, 2.0, 1.5, 1.0) == (2.0, 0.75, -1.0)


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


# --- axes_to_yaw_rate -------------------------------------------------------

def test_full_right_yaw_is_positive_rate():
    """NED yaw is positive clockwise seen from above, so right turn > 0."""
    assert offboard.axes_to_yaw_rate(1.0, 0.5) == 0.5


def test_full_left_yaw_is_negative_rate():
    assert offboard.axes_to_yaw_rate(-1.0, 0.5) == -0.5


def test_yaw_is_proportional_to_deflection():
    assert offboard.axes_to_yaw_rate(0.5, 0.5) == 0.25


def test_zero_yaw_is_zero_rate():
    assert offboard.axes_to_yaw_rate(0.0, 0.5) == 0.0


# --- CommandState ----------------------------------------------------------

def test_state_reports_stick_velocity():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)   # up-drag = full forward pitch
    assert s.command(now=100.1) == (2.0, 0.0, 0.0, 0.0)


def test_state_scales_with_partial_deflection():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -0.5, now=100.0)
    assert s.command(now=100.1) == (1.0, 0.0, 0.0, 0.0)


def test_state_release_returns_to_hover():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)
    s.set_stick("right", 0.0, 0.0, now=100.2)
    assert s.command(now=100.3) == (0.0, 0.0, 0.0, 0.0)


def test_right_stick_x_is_roll():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 1.0, 0.0, now=100.0)
    assert s.command(now=100.1) == (0.0, 1.5, 0.0, 0.0)


def test_left_stick_drives_thrust_and_yaw():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0, speed_right=1.5)
    s.set_stick("left", 1.0, -1.0, now=100.0)    # x=yaw right, up-drag=climb
    vx, vy, vz, yr = s.command(now=100.1)
    assert (vx, vy, vz) == (0.0, 0.0, -1.0)
    assert math.isclose(yr, math.radians(45.0))


def test_watchdog_zeroes_stale_input():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)
    assert s.command(now=100.4) == (2.0, 0.0, 0.0, 0.0)   # still fresh
    assert s.command(now=101.0) == (0.0, 0.0, 0.0, 0.0)   # stale -> hover


def test_watchdog_also_stops_the_turn():
    """A stale link must not leave the aircraft spinning."""
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0, speed_right=1.5)
    s.set_stick("left", 1.0, 0.0, now=100.0)
    assert s.command(now=101.0) == (0.0, 0.0, 0.0, 0.0)


def test_touch_keeps_a_stick_alive():
    """The page pings every 150 ms while a stick is off-center; without
    that the watchdog would cut hold-to-move off after watchdog_s."""
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)
    s.touch(now=100.4)
    assert s.command(now=100.7) == (2.0, 0.0, 0.0, 0.0)


def test_clear_drops_everything():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)
    s.clear()
    assert s.command(now=100.1) == (0.0, 0.0, 0.0, 0.0)


def test_deadzone_snaps_small_deflection_to_zero():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.02, -0.03, now=100.0)
    assert s.command(now=100.1) == (0.0, 0.0, 0.0, 0.0)


def test_out_of_range_deflection_is_clamped():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 2.0, -3.0, now=100.0)
    assert s.command(now=100.1) == (2.0, 1.5, 0.0, 0.0)


def test_unknown_stick_is_rejected():
    s = offboard.CommandState()
    with pytest.raises(ValueError):
        s.set_stick("middle", 0.0, 0.0)


def test_set_stick_returns_true_on_rest_to_active_edge():
    s = offboard.CommandState()
    assert s.set_stick("right", 0.0, -1.0, now=100.0) is True


def test_set_stick_returns_false_while_already_active():
    s = offboard.CommandState()
    assert s.set_stick("right", 0.0, -1.0, now=100.0) is True
    assert s.set_stick("right", 0.0, -0.5, now=100.1) is False


def test_set_stick_returns_true_again_after_returning_to_rest():
    s = offboard.CommandState()
    assert s.set_stick("right", 0.0, -1.0, now=100.0) is True
    assert s.set_stick("right", 0.0, 0.0, now=100.1) is False   # back to rest, not an edge
    assert s.set_stick("right", 0.0, -1.0, now=100.2) is True   # rest -> active again


def test_command_combines_forward_climb_and_turn():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)   # pitch forward
    s.set_stick("left", 1.0, -1.0, now=100.0)    # yaw right + climb
    vx, vy, vz, yr = s.command(now=100.1)
    assert (vx, vy, vz) == (2.0, 0.0, -1.0)
    assert math.isclose(yr, math.radians(45.0))


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
