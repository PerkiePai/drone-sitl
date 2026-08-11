"""Unit tests for streaming/vision_bridge.py. No PX4, no Isaac Sim."""
import math
import os
import sys

import pytest
from unittest.mock import MagicMock

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import offboard  # noqa: E402
import vision_bridge  # noqa: E402


@pytest.fixture
def conn():
    c = MagicMock()
    c.mav = MagicMock()
    return c


# --- ENU -> NED ------------------------------------------------------------

def test_enu_to_ned_swaps_east_north_and_flips_up():
    assert vision_bridge.enu_to_ned(1.0, 2.0, 3.0) == (2.0, 1.0, -3.0)


def test_enu_to_ned_is_its_own_inverse():
    """Swap-two-and-negate-the-third is an involution, so the NED->ENU
    direction needs no second function: applying this one twice is identity."""
    n, e, d = vision_bridge.enu_to_ned(4.0, -5.0, 6.0)
    assert (n, e, d) == (-5.0, 4.0, -6.0)
    assert vision_bridge.enu_to_ned(n, e, d) == (4.0, -5.0, 6.0)


# --- VISION_POSITION_ESTIMATE ---------------------------------------------

def test_send_converts_enu_to_ned_and_yaw_to_radians(conn):
    s = vision_bridge.VisionPositionSender(conn)
    assert s.send(vision_bridge.VisionPose(
        ts_ns=1_000_000_000, x=1.0, y=2.0, z=3.0,
        roll=0.0, pitch=0.0, yaw=math.pi / 2), now=1.0)
    args = conn.mav.vision_position_estimate_send.call_args[0]
    assert args[1:4] == (2.0, 1.0, -3.0)          # x=N, y=E, z=Down
    assert args[6] == pytest.approx(math.pi / 2)   # yaw already radians


def test_send_stamps_microseconds_not_nanoseconds(conn):
    s = vision_bridge.VisionPositionSender(conn)
    s.send(vision_bridge.VisionPose(ts_ns=2_500_000_000, x=0, y=0, z=0,
                                    roll=0, pitch=0, yaw=0), now=1.0)
    assert conn.mav.vision_position_estimate_send.call_args[0][0] == 2_500_000


def test_stale_pose_is_dropped_not_repeated(conn):
    """A frozen-but-plausible position is worse than none: EKF2 has failsafes
    for a lost source and none for a lying one."""
    s = vision_bridge.VisionPositionSender(conn, max_age_s=0.5)
    pose = vision_bridge.VisionPose(ts_ns=0, x=0, y=0, z=0,
                                    roll=0, pitch=0, yaw=0)
    assert s.send(pose, now=100.0, received_at=99.9)
    conn.mav.vision_position_estimate_send.reset_mock()
    assert not s.send(pose, now=100.0, received_at=99.0)
    conn.mav.vision_position_estimate_send.assert_not_called()


def test_none_pose_sends_nothing(conn):
    s = vision_bridge.VisionPositionSender(conn)
    assert not s.send(None, now=1.0)
    conn.mav.vision_position_estimate_send.assert_not_called()


# --- EKF2 params -----------------------------------------------------------

def test_gps_fusion_is_disabled():
    assert dict((n, v) for n, v, _ in
                vision_bridge.EKF2_VISION_PARAMS)["EKF2_GPS_CTRL"] == 0


def test_vision_claims_horizontal_position_and_yaw_only():
    """EKF2_EV_CTRL bit0=horizontal position, bit3=yaw (ekf2_params.c:687).
    Bit1 (vertical position) stays CLEAR because flow-odom takes altitude
    straight from the barometer (flow_odometry.py:455) -- setting it would feed
    baro back as an independent vision observation while EKF2 already fuses
    baro directly, double-counting one sensor. Bit2 (velocity) stays clear
    because VISION_POSITION_ESTIMATE carries no velocity."""
    ctrl = dict((n, v) for n, v, _ in
                vision_bridge.EKF2_VISION_PARAMS)["EKF2_EV_CTRL"]
    assert ctrl & 0b0001            # horizontal position
    assert ctrl & 0b1000            # yaw
    assert not ctrl & 0b0010        # NOT vertical position
    assert not ctrl & 0b0100        # NOT velocity


def test_height_reference_stays_on_the_barometer():
    """EKF2_HGT_REF value 0 = Barometric pressure (ekf2_params.c:657)."""
    assert dict((n, v) for n, v, _ in
                vision_bridge.EKF2_VISION_PARAMS)["EKF2_HGT_REF"] == 0


def test_vision_noise_is_looser_than_px4_defaults():
    """Defaults are 0.1 m / 0.1 rad -- far too tight for drifting flow-odom;
    EKF2 would reject its own vision source as inconsistent."""
    p = dict((n, v) for n, v, _ in vision_bridge.EKF2_VISION_PARAMS)
    assert p["EKF2_EVP_NOISE"] > 0.1
    assert p["EKF2_EVA_NOISE"] > 0.1
    assert p["EKF2_EV_NOISE_MD"] == 1     # use the params, not a reported variance


# --- the phase split -------------------------------------------------------

def test_the_phases_partition_the_whole_param_set():
    """Every param belongs to exactly one phase, and the phases add up to the
    profile. A param in two phases would be applied twice with different
    timing; one in none would silently never be applied at all."""
    phases = (vision_bridge.EKF2_BOOT_PARAMS
              + vision_bridge.EKF2_FUSION_PARAMS
              + vision_bridge.EKF2_GPS_DENIED_PARAMS)
    names = [n for n, _, _ in phases]
    assert len(names) == len(set(names)), names
    assert set(names) == {n for n, _, _ in vision_bridge.EKF2_VISION_PARAMS}


def test_only_reboot_required_params_are_in_the_boot_phase():
    """The boot phase costs a PX4 restart, so nothing rides along in it that
    did not have to."""
    assert ({n for n, _, _ in vision_bridge.EKF2_BOOT_PARAMS}
            == set(vision_bridge.HGT_REF_NEEDS_REBOOT))


def test_the_fusion_phase_is_safe_to_apply_with_gnss_on():
    """Phase 1 must be flyable on GPS: it turns vision ON without taking
    anything away, so the aircraft can climb to where the camera can see."""
    assert "EKF2_GPS_CTRL" not in {n for n, _, _
                                   in vision_bridge.EKF2_FUSION_PARAMS}


def test_cutting_gnss_is_a_phase_of_its_own():
    assert [n for n, _, _ in vision_bridge.EKF2_GPS_DENIED_PARAMS] == \
        ["EKF2_GPS_CTRL"]


def test_reboot_sets_the_boot_params_before_restarting(conn):
    """Reboot first and the param is still unset when PX4 reads it -- the whole
    sequence would be a no-op that looks like it worked."""
    link = offboard.OffboardLink(conn)
    order = []
    link.set_param = lambda name, value, ptype: order.append(("param", name))
    link.reboot_autopilot = lambda: order.append(("reboot", None))

    vision_bridge.reboot_for_boot_params(link)

    assert order == [("param", "EKF2_HGT_REF"), ("reboot", None)], order


def test_reboot_autopilot_sends_the_documented_command(conn):
    """MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN with param1=1 -- param1=0 would be a
    shutdown, which is not recoverable without relaunching the sim."""
    offboard.OffboardLink(conn).reboot_autopilot()
    args = conn.mav.command_long_send.call_args[0]
    assert args[2] == offboard.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    assert args[4] == pytest.approx(1.0)


def test_apply_params_sends_every_param_with_its_declared_type(conn):
    link = offboard.OffboardLink(conn)
    vision_bridge.apply_ekf2_vision_params(link)
    sent = {c[0][2].decode(): (c[0][3], c[0][4])
            for c in conn.mav.param_set_send.call_args_list}
    assert len(sent) == len(vision_bridge.EKF2_VISION_PARAMS)
    for name, value, ptype in vision_bridge.EKF2_VISION_PARAMS:
        assert sent[name] == (pytest.approx(float(value)), ptype)


# --- SET_GPS_GLOBAL_ORIGIN -------------------------------------------------

def test_origin_uses_degE7_and_millimetres(conn):
    """SET_GPS_GLOBAL_ORIGIN: latitude/longitude degE7, altitude MILLIMETRES."""
    link = offboard.OffboardLink(conn)
    vision_bridge.send_gps_global_origin(link, 13.66156872, 100.298235, 12.5)
    args = conn.mav.set_gps_global_origin_send.call_args[0]
    assert args[1] == 136615687
    assert args[2] == 1002982350
    assert args[3] == 12500
