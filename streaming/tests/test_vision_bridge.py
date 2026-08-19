"""Unit tests for streaming/vision_bridge.py. No PX4, no Isaac Sim."""
import math
import os
import struct
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


@pytest.mark.parametrize("heading_deg", [0.0, 45.0, 90.0, 180.0, 270.0])
def test_enu_yaw_to_ned_recovers_the_compass_heading(heading_deg):
    """The estimator's yaw comes from MahonyState.from_heading, which builds it
    as radians(90 - heading_deg) (flow_odometry.py:157). Converting back has to
    return the compass heading it was made from -- for EVERY heading, not just
    45 deg, which is the one bearing where the raw ENU number happened to be
    right and where a single-case test would have passed."""
    yaw_enu = math.radians(90.0 - heading_deg)
    ned = vision_bridge.enu_yaw_to_ned(yaw_enu)
    assert math.degrees(ned) == pytest.approx(
        math.degrees(vision_bridge.wrap_pi(math.radians(heading_deg))))


def test_enu_yaw_to_ned_is_its_own_inverse():
    """Like enu_to_ned: reflecting about 45 deg twice is identity, so there is
    no separate NED->ENU function to keep in step with this one."""
    for yaw in (0.0, 0.3, -2.1, 3.0):
        assert vision_bridge.enu_yaw_to_ned(
            vision_bridge.enu_yaw_to_ned(yaw)) == pytest.approx(yaw)


def test_enu_attitude_to_ned_matches_the_rotation_matrix_form():
    """The closed form against the definition it is derived from:
    R_frd_ned = S @ R_flu_enu @ B. Pure Python so the test suite keeps needing
    no scipy -- 3x3 matrices are small enough to multiply by hand."""
    def rot_xyz(r, p, y):
        cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                                  math.sin(p), math.cos(y), math.sin(y))
        return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr]]

    S = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]      # ENU -> NED
    B = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]     # FLU -> FRD
    for roll, pitch, yaw in [(0.1, 0.2, 0.3), (-0.4, 0.15, 2.5),
                             (0.05, -0.3, -1.9), (0.0, 0.0, math.pi / 2)]:
        R = rot_xyz(roll, pitch, yaw)
        M = [[sum(S[i][k] * R[k][m] * B[m][j] for k in range(3)
                  for m in range(3) if B[m][j])
              for j in range(3)] for i in range(3)]
        # Euler angles read back out of the NED/FRD matrix, aerospace 3-2-1.
        want = (math.atan2(M[2][1], M[2][2]),
                -math.asin(max(-1.0, min(1.0, M[2][0]))),
                math.atan2(M[1][0], M[0][0]))
        got = vision_bridge.enu_attitude_to_ned(roll, pitch, yaw)
        assert got == pytest.approx(want, abs=1e-12)


# --- VISION_POSITION_ESTIMATE ---------------------------------------------

def test_send_converts_position_and_attitude_into_ned(conn):
    """The pose is ENU/FLU on the wire and NED/FRD in the message.

    Position was converted from the start; the ATTITUDE half was not, until
    2026-08-13. An ENU yaw of pi/2 is a drone pointing NORTH, and PX4 must be
    told 0, not pi/2 -- see enu_yaw_to_ned.
    """
    s = vision_bridge.VisionPositionSender(conn)
    assert s.send(vision_bridge.VisionPose(
        ts_ns=1_000_000_000, x=1.0, y=2.0, z=3.0,
        roll=0.1, pitch=0.2, yaw=math.pi / 2), now=1.0)
    args = conn.mav.vision_position_estimate_send.call_args[0]
    assert args[1:4] == (2.0, 1.0, -3.0)           # x=N, y=E, z=Down
    assert args[4] == pytest.approx(0.1)           # roll survives
    assert args[5] == pytest.approx(-0.2)          # pitch flips
    assert args[6] == pytest.approx(0.0)           # ENU east-relative -> NED north-relative


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


def test_vision_pose_has_no_ground_truth_fields():
    """The fence that replaces D4's process separation (ADR-0004).

    Ground truth now crosses into this process, carried on the SAME `vio` ZMQ
    message as the estimate, purely for joystick-server.py to score aircraft
    excursion. VisionPositionSender.send() only ever reads a VisionPose --
    never the raw ZMQ dict -- so as long as VisionPose itself cannot carry a
    gt_* field, nothing here can leak into what PX4 is told. This is the
    structural guarantee: if a future change ever adds one to the dataclass,
    this test is what notices."""
    fields = vision_bridge.VisionPose.__dataclass_fields__
    assert not any(name.startswith("gt") for name in fields), fields


# --- frame alignment -------------------------------------------------------

def _pose(x, y, z=10.0, yaw=math.pi / 2):
    return vision_bridge.VisionPose(ts_ns=0, x=x, y=y, z=z,
                                    roll=0.0, pitch=0.0, yaw=yaw)


def test_identity_alignment_is_a_plain_frame_conversion():
    """With no alignment taken, nothing is invented: the transform is exactly
    the ENU->NED conversion and no more."""
    n, e, d, _, _, yaw = vision_bridge.IDENTITY_ALIGNMENT.to_px4_ned(
        _pose(1.0, 2.0, 3.0, yaw=math.pi / 2))
    assert (n, e, d) == (2.0, 1.0, -3.0)
    assert yaw == pytest.approx(0.0)
    assert vision_bridge.IDENTITY_ALIGNMENT.offset_m() == 0.0


def test_alignment_puts_the_pose_exactly_on_px4():
    """The defining property. A vision frame 30 m adrift and 10 deg rotated is
    the 2026-08-12 handover; after aligning, the transformed pose IS PX4's."""
    pose = _pose(30.0, -12.0, z=48.0, yaw=math.radians(70.0))
    px4 = vision_bridge.Px4Pose(north=5.0, east=-3.0, down=-49.0,
                                yaw=math.radians(30.0))
    align = vision_bridge.align_to_px4(pose, px4)
    n, e, d, _, _, yaw = align.to_px4_ned(pose)
    assert (n, e, d) == pytest.approx((px4.north, px4.east, px4.down))
    assert yaw == pytest.approx(px4.yaw)


def test_alignment_preserves_motion_it_did_not_measure():
    """Rigid, not a snap-to-PX4. Vision travel after the alignment must still
    arrive as travel of the same LENGTH -- otherwise the transform would be
    quietly rescaling the one signal EKF2 has left once GNSS is gone."""
    a, b = _pose(30.0, -12.0), _pose(37.0, -4.0)
    px4 = vision_bridge.Px4Pose(north=5.0, east=-3.0, down=-49.0, yaw=0.4)
    align = vision_bridge.align_to_px4(a, px4)
    na, ea, _, _, _, _ = align.to_px4_ned(a)
    nb, eb, _, _, _, _ = align.to_px4_ned(b)
    assert math.hypot(nb - na, eb - ea) == pytest.approx(
        math.hypot(b.x - a.x, b.y - a.y))


def test_alignment_rotates_travel_with_the_yaw_it_corrected():
    """A translation-only fix would leave the frames rotated against each
    other, so every metre flown after the handover would point a few degrees
    wrong and the error would grow with distance. Vision travelling due ENU-east
    under a 90 deg yaw correction must come out along a correspondingly rotated
    bearing, not along raw east."""
    a, b = _pose(0.0, 0.0, yaw=math.pi / 2), _pose(10.0, 0.0, yaw=math.pi / 2)
    # Vision says north (ENU yaw pi/2), PX4 says east (NED yaw pi/2).
    px4 = vision_bridge.Px4Pose(north=0.0, east=0.0, down=0.0, yaw=math.pi / 2)
    align = vision_bridge.align_to_px4(a, px4)
    assert align.yaw == pytest.approx(math.pi / 2)
    nb, eb, _, _, _, _ = align.to_px4_ned(b)
    # Raw ENU east == NED (0, 10); rotated 90 deg clockwise it is (-10, 0).
    assert (nb, eb) == pytest.approx((-10.0, 0.0))


def test_offset_m_reports_the_error_that_would_have_been_inherited():
    pose = _pose(0.0, 0.0, yaw=math.pi / 2)
    px4 = vision_bridge.Px4Pose(north=30.0, east=40.0, down=0.0, yaw=0.0)
    assert vision_bridge.align_to_px4(pose, px4).offset_m() == pytest.approx(50.0)


def test_sender_applies_the_alignment_to_every_later_estimate(conn):
    s = vision_bridge.VisionPositionSender(conn)
    px4 = vision_bridge.Px4Pose(north=100.0, east=-50.0, down=-49.0, yaw=0.0)
    s.realign(_pose(0.0, 0.0, z=49.0, yaw=math.pi / 2), px4)
    assert s.realigned == 1
    assert s.send(_pose(0.0, 0.0, z=49.0, yaw=math.pi / 2), now=1.0)
    args = conn.mav.vision_position_estimate_send.call_args[0]
    assert args[1:3] == pytest.approx((100.0, -50.0))


def test_realigning_replaces_rather_than_accumulates(conn):
    """Each transition solves the transform afresh against PX4. Composing them
    would fold a stale offset into the new one and leave the handover carrying
    exactly the error it exists to remove."""
    s = vision_bridge.VisionPositionSender(conn)
    pose = _pose(10.0, 10.0)
    s.realign(pose, vision_bridge.Px4Pose(north=1.0, east=1.0, down=0.0, yaw=0.0))
    s.realign(pose, vision_bridge.Px4Pose(north=7.0, east=8.0, down=0.0, yaw=0.0))
    assert s.realigned == 2
    n, e, _, _, _, _ = s.alignment.to_px4_ned(pose)
    assert (n, e) == pytest.approx((7.0, 8.0))


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


def test_boot_phase_enables_flight_logging_from_boot_to_shutdown():
    """SDLOG_MODE=2 (logger/params.c:68) so a flight is captured even if arming
    fails partway -- the whole point of ADR-0003, which found no ulog had ever
    survived a VIO session because Pegasus deletes the rootfs on exit."""
    p = dict((n, v) for n, v, _ in vision_bridge.EKF2_BOOT_PARAMS)
    assert p["SDLOG_MODE"] == 2
    assert p["SDLOG_PROFILE"] == 131          # bit0 default + bit1 EKF2 replay + bit7 CV
    for name in ("SDLOG_MODE", "SDLOG_PROFILE"):
        assert dict((n, t) for n, _, t in vision_bridge.EKF2_BOOT_PARAMS)[name] \
            == offboard.MAV_PARAM_TYPE_INT32


def test_the_fusion_phase_is_safe_to_apply_with_gnss_on():
    """Phase 1 must be flyable on GPS: it turns vision ON without taking
    anything away, so the aircraft can climb to where the camera can see."""
    assert "EKF2_GPS_CTRL" not in {n for n, _, _
                                   in vision_bridge.EKF2_FUSION_PARAMS}


def test_cutting_gnss_is_a_phase_of_its_own():
    assert [n for n, _, _ in vision_bridge.EKF2_GPS_DENIED_PARAMS] == \
        ["EKF2_GPS_CTRL"]


def test_reboot_sets_the_boot_params_before_restarting(conn):
    """Reboot first and the params are still unset when PX4 reads them -- the
    whole sequence would be a no-op that looks like it worked."""
    link = offboard.OffboardLink(conn)
    order = []
    link.set_param = lambda name, value, ptype: order.append(("param", name))
    link.reboot_autopilot = lambda: order.append(("reboot", None))

    vision_bridge.reboot_for_boot_params(link)

    assert order == [("param", n) for n, _, _ in vision_bridge.EKF2_BOOT_PARAMS] \
        + [("reboot", None)], order


def test_reboot_autopilot_sends_the_documented_command(conn):
    """MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN with param1=1 -- param1=0 would be a
    shutdown, which is not recoverable without relaunching the sim."""
    offboard.OffboardLink(conn).reboot_autopilot()
    args = conn.mav.command_long_send.call_args[0]
    assert args[2] == offboard.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    assert args[4] == pytest.approx(1.0)


def _decode(wire, ptype):
    """Undo the wire encoding, so tests assert the value PX4 will actually
    store rather than the number that happened to be passed in."""
    if ptype == offboard.MAV_PARAM_TYPE_INT32:
        return struct.unpack("<i", struct.pack("<f", wire))[0]
    return wire


def test_apply_params_sends_every_param_with_its_declared_type(conn):
    link = offboard.OffboardLink(conn)
    vision_bridge.apply_ekf2_vision_params(link)
    sent = {c[0][2].decode(): (c[0][3], c[0][4])
            for c in conn.mav.param_set_send.call_args_list}
    assert len(sent) == len(vision_bridge.EKF2_VISION_PARAMS)
    for name, value, ptype in vision_bridge.EKF2_VISION_PARAMS:
        wire, sent_type = sent[name]
        assert sent_type == ptype
        assert _decode(wire, ptype) == pytest.approx(value)


def test_int_params_go_on_the_wire_as_their_bit_pattern(conn):
    """PX4 reinterprets PARAM_SET's float field as int32 for an INT32 param
    (mavlink_parameters.cpp:134), so the wire must carry the bits of the
    integer, not the integer converted to a float.

    Sending float(9) sets the param to 1091567616 and PX4 accepts it silently.
    Measured live 2026-08-11: this is why EKF2_EV_CTRL was never 9 and vision
    fusion never switched on."""
    link = offboard.OffboardLink(conn)
    link.set_param("EKF2_EV_CTRL", 9, offboard.MAV_PARAM_TYPE_INT32)
    wire = conn.mav.param_set_send.call_args[0][3]

    assert struct.unpack("<i", struct.pack("<f", wire))[0] == 9
    assert wire != 9.0, "sending the number itself is the bug"


def test_zero_valued_int_params_are_unchanged_by_the_encoding(conn):
    """0.0f and int 0 share a bit pattern. This is why EKF2_GPS_CTRL=0 worked
    all along while every non-zero int param did not -- the params that
    disabled things worked and only the ones that enabled things were dead."""
    link = offboard.OffboardLink(conn)
    link.set_param("EKF2_GPS_CTRL", 0, offboard.MAV_PARAM_TYPE_INT32)
    assert conn.mav.param_set_send.call_args[0][3] == 0.0


def test_float_params_are_sent_as_their_value(conn):
    """REAL32 params are NOT bit-reinterpreted -- only INT32 is."""
    link = offboard.OffboardLink(conn)
    link.set_param("EKF2_EVP_NOISE", 0.5, offboard.MAV_PARAM_TYPE_REAL32)
    assert conn.mav.param_set_send.call_args[0][3] == pytest.approx(0.5)


# --- MPC_XY_* gains (Task 10) -----------------------------------------------

def test_mpc_defaults_match_px4s_own_stock_values():
    """mc_pos_control_params.c:270,282,295,307 -- PX4's own shipped values,
    and D2's `baseline` candidate row."""
    defaults = dict((n, v) for n, v, _ in vision_bridge.MPC_XY_DEFAULTS)
    assert defaults == {
        "MPC_XY_P": 0.95,
        "MPC_XY_VEL_P_ACC": 1.8,
        "MPC_XY_VEL_I_ACC": 0.4,
        "MPC_XY_VEL_D_ACC": 0.2,
    }


def test_revert_mpc_gains_sends_exactly_the_four_defaults(conn):
    link = offboard.OffboardLink(conn)
    vision_bridge.revert_mpc_gains(link)
    sent = {c[0][2].decode(): (c[0][3], c[0][4])
            for c in conn.mav.param_set_send.call_args_list}
    assert len(sent) == len(vision_bridge.MPC_XY_DEFAULTS)
    for name, value, ptype in vision_bridge.MPC_XY_DEFAULTS:
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
