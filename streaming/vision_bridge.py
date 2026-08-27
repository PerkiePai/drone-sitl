"""VISION_POSITION_ESTIMATE, the EKF2 vision param set, and the GPS origin.

Imported by joystick-server.py, which owns the only MAVLink connection in this
system. Same contract as OffboardLink: SETPOINT-THREAD ONLY -- pymavlink
connections are not thread-safe.

PX4 v1.14.3. Every param value below was read out of ~/PX4-Autopilot, with the
source line on each one, per the convention in offboard.py:7-9.
"""
import math
from dataclasses import dataclass

from offboard import MAV_PARAM_TYPE_INT32, MAV_PARAM_TYPE_REAL32

# EKF2 vision fusion. Applied only under --vision: EKF2_GPS_CTRL=0 is a
# deliberate, safety-relevant act, never a default. Each param is documented on
# the phase that applies it.
#
# The set is split into PHASES, because applying it as one block does not
# work and the failure is an unbounded descent rather than an error. Measured
# live 2026-08-11 (SESSION.md): EKF2_GPS_CTRL=0 takes effect immediately while
# EKF2_HGT_REF=0 does not, leaving EKF2 with GNSS fusion off and its height
# reference still on GPS. Reported altitude ran 49 m -> -22 m and still falling,
# with AUTO.LAND latched and both `offboard` and `disarm` refused.

EKF2_GPS_CTRL_DEFAULT = 7
"""PX4's own default (ekf2_params.c:706) -- all GNSS aiding on.

Named rather than inlined because restoring it is a safety act, not a tidy-up:
it is what stops a previous GPS-denied session leaving the next run to boot
with GNSS already disabled.
"""

EV_DELAY_MS_MAX = 300.0
"""PX4's own ceiling on EKF2_EV_DELAY (ekf2_params.c:146, `@max 300`).

Stated as a constant because the flown pipeline lag is LARGER than it -- see
EKF2_EV_DELAY in EKF2_BOOT_PARAMS -- so the ceiling is a fact about what this
knob can and cannot buy back, not an input-validation detail.
"""

EKF2_BOOT_PARAMS = (
    ("EKF2_HGT_REF", 0, MAV_PARAM_TYPE_INT32),
    ("EKF2_EV_DELAY", 0.0, MAV_PARAM_TYPE_REAL32),
    ("SDLOG_MODE", 2, MAV_PARAM_TYPE_INT32),
    ("SDLOG_PROFILE", 131, MAV_PARAM_TYPE_INT32),
)
"""Phase 0 -- params PX4 only re-reads at boot, so PX4 must be restarted after
these are set. Not just EKF2 vision params despite the name -- SDLOG_* rides
along because phase 0 already pays for a reboot (ADR-0003).

`EKF2_HGT_REF` ekf2_params.c:657, default 1 (GPS). 0 = barometric, which is
where the height genuinely comes from. *** @reboot_required true
(ekf2_params.c:656). *** A PARAM_SET at runtime updates the STORED value -- QGC
will show 0 -- but EKF2 does not re-read its height reference until PX4
restarts.

`SDLOG_MODE` logger/params.c:68, default 0 (armed until disarm). 2 = boot until
shutdown, so a flight is captured even if it never arms cleanly.
@reboot_required true (logger/params.c:65).

`EKF2_EV_DELAY` ekf2_params.c:151, default 0 ms, `@max 300`, *** and
@reboot_required true (ekf2_params.c:148) *** -- which is why it is HERE and
not in the phase-1b fusion set. Applied at connect time it would store and do
nothing, exactly as EKF2_HGT_REF did on 2026-08-11.

It is how long ago the vision measurement actually describes. Left at 0, EKF2
treats every VISION_POSITION_ESTIMATE as a measurement of where the aircraft is
at the instant the message lands, and it is not: it is at best a solve of a
frame captured a while earlier. PX4 makes that worse than it looks -- the
`usec` field VisionPositionSender sends is DISCARDED, because Timesync never
converges on this link (Timesync.cpp:127-136, and Run 2 below), so PX4 stamps
each estimate with its own arrival time. This param is the only remaining way
to tell EKF2 the measurement is old.

**The default stays 0 because the flown lag does not fit in the param.**
Measured 2026-08-22 against ground truth from the estimator's own payload,
where gt and the estimate are the same frame: the position estimate trails
truth by ~0.7-1.2 s of sim time. 300 ms is the ceiling, so this knob can
model at most a third of it. Set it from `--ev-delay-ms` for the experiment;
do not read a partial improvement as the fix.

`SDLOG_PROFILE` logger/params.c:147, default 1 (mission messages only). 131 =
bit0 (1, default set) + bit1 (2, full-rate EKF2 replay) + bit7 (128, computer
vision) -- 1+2+128=131. Replay topics unlock `src/modules/replay` on the
resulting ulog (ADR-0003); computer-vision topics carry the vision fusion
innovations this whole system exists to measure. @reboot_required true
(logger/params.c:144).
"""

EKF2_FUSION_PARAMS = (
    ("EKF2_EV_CTRL", 9, MAV_PARAM_TYPE_INT32),
    ("EKF2_EV_NOISE_MD", 1, MAV_PARAM_TYPE_INT32),
    ("EKF2_EVP_NOISE", 3.0, MAV_PARAM_TYPE_REAL32),
    ("EKF2_EVA_NOISE", 0.2, MAV_PARAM_TYPE_REAL32),
)
"""Phase 1b -- turn vision fusion ON while GNSS is still on. Safe in flight.

Deliberately separable from phase 2 so the aircraft can climb on GPS with the
vision source already being fused and observable before anything is taken away.
At sites where the nadir camera cannot see the ground from the pad -- which is
every Cesium-tile site tested so far -- that climb is not optional.

  EKF2_EV_CTRL      ekf2_params.c:687, default 15. bit0 horizontal position,
                    bit1 vertical position, bit2 3D velocity, bit3 yaw.
                    9 = horizontal position + yaw, which is exactly what
                    flow-odom measures. Vertical is deliberately NOT claimed:
                    altitude comes straight from the barometer
                    (flow_odometry.py:455), and EKF2 already fuses baro
                    directly (EKF2_BARO_CTRL default 1), so setting bit1 would
                    feed one sensor in twice and read as spurious agreement.
  EKF2_EV_NOISE_MD  ekf2_params.c:814, default 0. 1 = use the noise params
                    below rather than a reported variance we do not compute.
  EKF2_EVP_NOISE    ekf2_params.c:837, default 0.1 m. 3.0 -- see below.
  EKF2_EVA_NOISE    ekf2_params.c:857, default 0.1 rad -- far too tight for a
                    drifting estimator; EKF2 would reject its own vision
                    source as inconsistent.

`EKF2_EVP_NOISE` was 0.5 until 2026-08-13, and 0.5 is a statement that the
vision position is good to half a metre. Flown, it is not. Once GNSS is cut and
the control loop CLOSES on the estimate, flow-odom starts producing position
jumps of 8-13 m between consecutive samples -- with 550-600 healthy inliers, so
not a tracking dropout. At 0.5 EKF2 believes each jump and flies at it, the
aircraft lurches, the camera sweeps, tracking degrades, and the next solve is
worse. Measured 2026-08-13: 0.33 m drift at the cut (a clean handover) to
12 m within 15 frames, and the aircraft ran 165 m off.

3.0 m is chosen to sit ABOVE that jump amplitude, so EKF2 filters the jumps
instead of chasing them, while vision still constrains position far better than
dead reckoning. It is a statement about what this estimator actually delivers
in flight, not a tuning knob to relax until the symptom goes away.
"""

EKF2_GPS_FLIGHT_PARAMS = (
    ("EKF2_EV_CTRL", 0, MAV_PARAM_TYPE_INT32),
    ("EKF2_GPS_CTRL", EKF2_GPS_CTRL_DEFAULT, MAV_PARAM_TYPE_INT32),
)
"""Phase 1 -- assert ordinary GPS flight, explicitly. Applied every startup.

**PX4 parameters persist across runs and across reboots**, so not setting a
param is NOT the same as it being off. Without this, two things carry over from
a previous GPS-denied session:

  EKF2_EV_CTRL=9   vision fused over a blind pad camera -> PX4 refuses to arm,
                   `Preflight Fail: Yaw estimate error`
  EKF2_GPS_CTRL=0  **the next run boots GPS-denied on the ground**, which is
                   the more dangerous of the two by a wide margin

Both were observed live 2026-08-11: deferring the fusion params in code changed
nothing, because the previous run had already saved EKF2_EV_CTRL=9.
"""

EKF2_GPS_DENIED_PARAMS = (
    ("EKF2_GPS_CTRL", 0, MAV_PARAM_TYPE_INT32),
)
"""Phase 2 -- the GPS-denied moment itself. Kept alone and explicit.

`EKF2_GPS_CTRL` ekf2_params.c:706, default 7. 0 disables ALL GNSS fusion. This
is the only irreversible-feeling step in the sequence and the one worth being
able to point at, so it is not buried in a block of six.
"""

EKF2_GNSS_RESTORE_PARAMS = (
    ("EKF2_GPS_CTRL", EKF2_GPS_CTRL_DEFAULT, MAV_PARAM_TYPE_INT32),
)
"""Phase 2, undone. The abort, and the exact inverse of EKF2_GPS_DENIED_PARAMS.

Returns to PHASE 1B -- GNSS on with vision still fused -- rather than to
phase 1, because that is the state the cut was taken from and the one it should
fall back to. Vision keeps streaming and stays observable, so the operator can
watch it recover and cut again once it looks good, which is the whole recovery
workflow. `apply_ekf2_gps_flight_params` is the other direction, for a startup
that must assert ordinary GPS flight from an unknown prior state.

**Deliberately ungated, unlike the cut.** Every refusal in `_go_gps_denied`
exists because cutting GNSS onto a bad vision source can put the aircraft in
the ground. Restoring GNSS has no such failure mode: the worst case is that a
healthy source is added to a flight that was managing without it. A recovery
control that can refuse is not a recovery control.

Nothing here re-enables anything vision-side, so it is safe to send at any
time, including when GNSS was never cut.
"""

EKF2_VISION_PARAMS = EKF2_BOOT_PARAMS + EKF2_FUSION_PARAMS + EKF2_GPS_DENIED_PARAMS
"""Every param the vision profile touches. The phases above are how they are
APPLIED; this is what the whole profile amounts to."""

HGT_REF_NEEDS_REBOOT = tuple(name for name, _, _ in EKF2_BOOT_PARAMS)
"""Params that PX4 only re-reads at boot.

Setting these on a running PX4 is not enough: the value sticks, the behaviour
does not change. Kept as data so the caller can act rather than silently
believing a param it can see took effect.
"""

MPC_XY_DEFAULTS = (
    ("MPC_XY_P", 0.95, MAV_PARAM_TYPE_REAL32),
    ("MPC_XY_VEL_P_ACC", 1.8, MAV_PARAM_TYPE_REAL32),
    ("MPC_XY_VEL_I_ACC", 0.4, MAV_PARAM_TYPE_REAL32),
    ("MPC_XY_VEL_D_ACC", 0.2, MAV_PARAM_TYPE_REAL32),
)
"""PX4's own stock position-controller gains (mc_pos_control_params.c:270,
282,295,307) -- none @reboot_required, unlike EKF2_HGT_REF, confirmed by their
absence from that file's docblocks. This is Task 10's `baseline` candidate
(docs/superpowers/specs/2026-08-14-mpc-xy-gain-sweep-design.md, D2) and also
what `revert_mpc_gains` restores to.
"""

DEFAULT_MAX_AGE_S = 0.5
"""How long an estimate may go unrefreshed before it is dropped, in SIM SECONDS.

The unit is the whole point. This budget answers "how long has the aircraft been
flying on a position nobody has confirmed", and the aircraft flies in sim time:
PX4 SITL is lockstepped to Isaac, so at sim_rate 0.11 one simulated second takes
nine wall seconds and the airframe does not care.

Measured in wall time -- as this was until 2026-08-12 -- it turns into a
self-inflicted failure exactly when the sim bogs down. The estimator keeps
producing perfectly good estimates, they arrive further apart in wall time than
the budget allows, and every one is dropped as stale at the precise moment they
are the only position source EKF2 has. Observed: `dropped_stale` 15 -> 179 while
`sim_rate` fell 0.56 -> 0.11, and the aircraft diverged and crashed.

The caller supplies both `now` and `received_at`; it is responsible for their
being on the same clock, and for that clock being sim time when one is available.
"""


@dataclass(frozen=True)
class VisionPose:
    """One estimate, in the estimator's ENU frame. Angles in radians.

    The angles are the estimator's, which means body-FLU-in-world-ENU
    (pipeline-streaming.py:130-134, off `MahonyState.R_flu`). They are NOT the
    NED/FRD angles VISION_POSITION_ESTIMATE carries -- see
    `enu_attitude_to_ned`, which is the only correct way to read them.
    """
    ts_ns: int
    x: float          # East
    y: float          # North
    z: float          # Up
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class Px4Pose:
    """Where PX4 believes it is, in its own local NED frame. Radians.

    Assembled by the caller from LOCAL_POSITION_NED (position) and ATTITUDE
    (yaw). Exists so `align_to_px4` takes one argument that cannot be
    half-populated, rather than four loose floats in an order nobody remembers.
    """
    north: float
    east: float
    down: float
    yaw: float


def wrap_pi(angle):
    """An angle folded into (-pi, pi]. Applied to every yaw this module emits."""
    return math.remainder(angle, math.tau)


def enu_to_ned(e, n, u):
    """(East, North, Up) -> (North, East, Down)."""
    return (n, e, -u)


def enu_yaw_to_ned(yaw):
    """ENU yaw (0 = East, counter-clockwise) -> NED yaw (0 = North, clockwise).

    `pi/2 - yaw`, and like `enu_to_ned` it is its own inverse.

    **This conversion was missing until 2026-08-13, and it is not cosmetic.**
    The estimator reports yaw as the heading of the body-forward axis measured
    from EAST, counter-clockwise (pipeline-streaming.py:132,
    `arctan2(R_flu[1,0], R_flu[0,0])`). VISION_POSITION_ESTIMATE is defined in
    NED, so PX4 reads that same number as degrees clockwise from NORTH. Sent
    raw, a drone pointing north (ENU yaw 90 deg) tells EKF2 it is pointing east.
    The error is `90 - 2*heading` degrees, so it vanishes at heading 45 and is
    worst -- a full 180 deg -- pointing south. With `EKF2_EV_CTRL` bit3 set
    (EKF2_FUSION_PARAMS) that yaw is fused, so EKF2 was being told the aircraft
    faced somewhere it did not, and every horizontal correction it derived from
    vision was applied in the wrong direction.
    """
    return wrap_pi(math.pi / 2 - yaw)


def enu_attitude_to_ned(roll, pitch, yaw):
    """Body-FLU-in-ENU euler angles -> body-FRD-in-NED, which is what VPE carries.

    Roll survives untouched, pitch flips sign, yaw goes through
    `enu_yaw_to_ned`. That falls out of `R_frd_ned = S @ R_flu_enu @ B` with
    S the ENU->NED swap and B = diag(1, -1, -1) the FLU->FRD flip; verified
    exact against the matrix form over random attitudes
    (test_enu_attitude_to_ned_matches_the_rotation_matrix_form).

    Pitch matters even though EKF2 is not asked to fuse EV attitude beyond yaw:
    PX4 rebuilds a quaternion from all three angles and takes the yaw out of
    that, so a mirrored pitch corrupts the one component that IS fused.
    """
    return (roll, -pitch, enu_yaw_to_ned(yaw))


@dataclass(frozen=True)
class FrameAlignment:
    """A rigid transform from the estimator's frame into PX4's local NED.

    A drifting odometry source and PX4's own estimate are two different frames
    that happen to be described in the same units. Nothing keeps their origins
    together, so by the time the aircraft has climbed they disagree -- measured
    23-37 m after a 3 m/s climb to 49 m, against 0.36-0.61 m in a hover
    (SESSION.md, run 4). Handing EKF2 that frame unchanged is what made
    GPS-denied flight diverge: the aircraft chases the offset, which moves the
    camera, which feeds more drift.

    So the stream is transformed rather than trusted. Yaw is rotated as well as
    position, because a translation-only fix leaves the two frames rotated
    against each other and every subsequent metre of vision travel then points
    a few degrees wrong -- an error that grows with distance flown, which is
    exactly the flight this has to survive.

    `down` is carried for completeness and is very nearly zero in practice:
    both sides take height from the same barometer (flow_odometry.py:455,
    EKF2_HGT_REF=0) and `EKF2_EV_CTRL` bit1 is deliberately unset, so EKF2 does
    not fuse the vertical component anyway.

    The identity is the honest default: with no alignment taken, `to_px4_ned`
    is a pure frame conversion and nothing is invented.
    """
    yaw: float = 0.0
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0

    def to_px4_ned(self, pose):
        """A VisionPose as (north, east, down, roll, pitch, yaw) for PX4."""
        n, e, d = enu_to_ned(pose.x, pose.y, pose.z)
        roll, pitch, yaw = enu_attitude_to_ned(pose.roll, pose.pitch, pose.yaw)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.north + c * n - s * e,
                self.east + s * n + c * e,
                self.down + d,
                roll, pitch, wrap_pi(yaw + self.yaw))

    def offset_m(self):
        """How far this shifts the vision frame horizontally, in metres.

        The headline number: it is the error EKF2 would otherwise have
        inherited at the handover, so it belongs in the telemetry row and in
        the log line rather than being computed and thrown away.
        """
        return math.hypot(self.north, self.east)


IDENTITY_ALIGNMENT = FrameAlignment()
"""No alignment taken yet -- `to_px4_ned` is then a plain ENU->NED conversion."""


def align_to_px4(pose, px4):
    """The FrameAlignment that puts `pose` exactly at `px4`.

    Solves `to_px4_ned(pose) == px4` for the transform, so the vision stream is
    continuous with PX4's own estimate at the instant it is taken and
    GPS-denied flight starts from zero error instead of from 30 m.

    Taken at the two phase transitions ONLY -- never continuously. Re-solving
    this every tick would feed EKF2 its own state back as an independent
    measurement: innovations would sit at zero by construction, EKF2 would gain
    confidence from a measurement carrying no information, and a completely
    broken estimator would look perfect right up until GNSS was cut. Between
    the transitions the vision source stays independent and observable, which
    is the entire reason the profile has a phase 1b at all.
    """
    n, e, d = enu_to_ned(pose.x, pose.y, pose.z)
    dyaw = wrap_pi(px4.yaw - enu_yaw_to_ned(pose.yaw))
    c, s = math.cos(dyaw), math.sin(dyaw)
    return FrameAlignment(yaw=dyaw,
                          north=px4.north - (c * n - s * e),
                          east=px4.east - (s * n + c * e),
                          down=px4.down - d)


def _apply(link, params):
    for name, value, ptype in params:
        link.set_param(name, value, ptype)


def apply_ekf2_vision_params(link):
    """Push the whole vision param set in one go. Setpoint thread only.

    Correct ONLY when PX4 will be rebooted afterwards, or was booted with the
    phase-0 params already in place. Applying this to a running PX4 and then
    flying is the failure described at EKF2_BOOT_PARAMS. Prefer the phased
    calls below; this is kept for the case where a reboot follows.
    """
    _apply(link, EKF2_VISION_PARAMS)


def boot_params(ev_delay_ms=None):
    """EKF2_BOOT_PARAMS with EKF2_EV_DELAY overridden, clamped to PX4's max.

    Kept separate from the applier so a caller -- or a test -- can see exactly
    what phase 0 is about to send without a link.
    """
    if ev_delay_ms is None:
        return EKF2_BOOT_PARAMS
    delay = max(0.0, min(float(ev_delay_ms), EV_DELAY_MS_MAX))
    return tuple((name, delay if name == "EKF2_EV_DELAY" else value, ptype)
                 for name, value, ptype in EKF2_BOOT_PARAMS)


def apply_ekf2_boot_params(link, ev_delay_ms=None):
    """Phase 0. Setpoint thread only. Requires a reboot to take effect."""
    _apply(link, boot_params(ev_delay_ms))


def apply_ekf2_gps_flight_params(link):
    """Phase 1: assert ordinary GPS flight with vision NOT fused.

    Setpoint thread only. Must run every startup -- see EKF2_GPS_FLIGHT_PARAMS
    for why leaving these alone is not the same as them being off.
    """
    _apply(link, EKF2_GPS_FLIGHT_PARAMS)


def apply_ekf2_fusion_params(link):
    """Phase 1b: fuse vision alongside GNSS. Setpoint thread only.

    Gated by the caller on the estimate actually tracking something -- see
    EKF2_GPS_FLIGHT_PARAMS for what fusing a blind camera costs.
    """
    _apply(link, EKF2_FUSION_PARAMS)


def apply_ekf2_gps_denied_params(link):
    """Phase 2: cut GNSS. Setpoint thread only.

    Separate from phase 1 on purpose -- this is the step that can put the
    aircraft in the ground if the vision source is not already known good.
    """
    _apply(link, EKF2_GPS_DENIED_PARAMS)


def apply_ekf2_gnss_restore_params(link):
    """Undo phase 2: turn GNSS fusion back on. Setpoint thread only.

    The abort. Takes no preconditions and cannot refuse -- see
    EKF2_GNSS_RESTORE_PARAMS for why that asymmetry with the cut is deliberate.
    """
    _apply(link, EKF2_GNSS_RESTORE_PARAMS)


def revert_mpc_gains(link):
    """D4: RESTORE GNSS reverts MPC_XY_* too, symmetric with EKF2_GPS_CTRL.

    Whatever gain set was active during a GPS-denied hold must not carry into
    a route or manual flight afterward. Called from `_go_gps_restore`
    alongside `apply_ekf2_gnss_restore_params` -- like that revert, this one
    can never make things worse, so it takes no gate and no precondition.
    """
    _apply(link, MPC_XY_DEFAULTS)


def reboot_for_boot_params(link, ev_delay_ms=None):
    """Set the @reboot_required params and restart PX4. Setpoint thread only.

    PX4 refuses a reboot while armed, so this is a pre-flight act by
    construction. The caller is expected to re-apply the remaining phases once
    PX4 comes back -- joystick-server.py already re-sends its startup params on
    a heartbeat gap, which is exactly what a reboot looks like from outside.
    """
    apply_ekf2_boot_params(link, ev_delay_ms)
    link.reboot_autopilot()


def send_gps_global_origin(link, lat_deg, lon_deg, alt_m):
    """Anchor PX4's global frame. Setpoint thread only.

    Mandatory with GNSS off: VISION_POSITION_ESTIMATE carries local NED offsets
    and no lat/lon, so without this PX4 has no absolute reference. Three shipped
    features fail SILENTLY without it -- the map marker (GLOBAL_POSITION_INT),
    waypoint flight (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, offboard.py:222-238) and
    the FLY gate (HOME_POSITION, joystick-server.py:73-76).

    Altitude is MILLIMETRES here, unlike every other altitude in this codebase.
    """
    link.conn.mav.set_gps_global_origin_send(
        link.target_system,
        int(round(lat_deg * 1e7)),
        int(round(lon_deg * 1e7)),
        int(round(alt_m * 1000.0)))


class VisionPositionSender:
    """Streams VISION_POSITION_ESTIMATE. Setpoint thread only.

    One message per setpoint tick, which is FASTER than estimates arrive: the
    setpoint loop runs at ~32 Hz and the estimator solves at ~14.7 Hz, so the
    same pose goes out more than once. Measured on the flown ulogs of
    2026-08-22 (`logs/20260822-twohold`, `logs/20260822-openloop98b`): 15627
    messages carrying 7010 distinct poses, 2.23 repeats each, max 4, 55% of
    all traffic. Identical in both, open loop and closed.

    That is not free. PX4 stamps each arrival with its own clock (see
    EKF2_EV_DELAY), so a repeat is presented to EKF2 as a fresh, independent
    measurement of the current instant while actually describing a frame up to
    120 ms older -- a delay that VARIES sample to sample, which no constant
    EKF2_EV_DELAY can model -- and 2.23 identical samples at EKF2_EVP_NOISE
    also understate the variance by about the same factor.

    `send_repeats=False` sends only when the estimate has moved on.
    **Default True, because the shipped behaviour is the flown one** and the
    repeats are the smaller half of the problem: they are worth ~120 ms
    against the ~0.7-1.2 s of lag measured end to end. Fly it before believing
    it, at 98 m AND at 49 m -- the 49 m companion is the test the gyro-bias
    gate failed.
    """

    def __init__(self, conn, max_age_s=DEFAULT_MAX_AGE_S, send_repeats=True):
        self.conn = conn
        self.max_age_s = max_age_s
        self.send_repeats = send_repeats
        self.sent = 0
        self.dropped_stale = 0
        # Ticks on which the pose had not changed since the last one sent.
        # Counted whether or not they were suppressed, so the two
        # configurations are readable from the same field.
        self.repeats = 0
        # pose.ts_ns of the last estimate actually sent, and its capture time
        # in seconds -- the sim clock the streamer stamped the FRAME with
        # (vio-streamer.py:242). Against the send clock this is the end-to-end
        # pipeline lag, less a constant PX4-boot offset; logged rather than
        # differenced here because only the caller knows both clocks.
        self._last_sent_ts = None
        self.last_capture_s = None
        # Identity until the caller takes an alignment at a phase transition.
        self.alignment = IDENTITY_ALIGNMENT
        self.realigned = 0

    def realign(self, pose, px4):
        """Re-solve the vision->PX4 transform against `px4`. Returns it.

        Called at the phase transitions by the caller, which owns the phases;
        see `align_to_px4` for why it must not be called every tick.
        """
        self.alignment = align_to_px4(pose, px4)
        self.realigned += 1
        # A new transform makes the SAME pose a different NED position, so the
        # next send is new information even if the estimate has not moved on.
        self._last_sent_ts = None
        return self.alignment

    def send(self, pose, now, received_at=None):
        """Send one estimate. Returns True if it went out.

        A pose older than max_age_s is DROPPED rather than repeated. Repeating a
        frozen position keeps EKF2 confident about a place the drone is not,
        and EKF2 has failsafes for a lost vision source but none for a lying
        one.

        `now` and `received_at` must be on the SAME clock, and that clock should
        be sim time wherever the caller can get it -- see DEFAULT_MAX_AGE_S for
        what measuring this in wall time costs. This function cannot check that
        for itself, which is why the requirement is stated rather than enforced.

        With `send_repeats=False` a pose already sent is not sent again. See
        the class docstring for what the repeats cost EKF2.
        """
        if pose is None:
            return False
        if received_at is not None and (now - received_at) > self.max_age_s:
            self.dropped_stale += 1
            return False
        if pose.ts_ns == self._last_sent_ts:
            self.repeats += 1
            if not self.send_repeats:
                return False
        n, e, d, roll, pitch, yaw = self.alignment.to_px4_ned(pose)
        self.conn.mav.vision_position_estimate_send(
            int(pose.ts_ns // 1000),        # usec
            float(n), float(e), float(d),
            float(roll), float(pitch), float(yaw))
        self._last_sent_ts = pose.ts_ns
        self.last_capture_s = pose.ts_ns / 1e9
        self.sent += 1
        return True
