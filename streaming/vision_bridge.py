"""VISION_POSITION_ESTIMATE, the EKF2 vision param set, and the GPS origin.

Imported by joystick-server.py, which owns the only MAVLink connection in this
system. Same contract as OffboardLink: SETPOINT-THREAD ONLY -- pymavlink
connections are not thread-safe.

PX4 v1.14.3. Every param value below was read out of ~/PX4-Autopilot, with the
source line on each one, per the convention in offboard.py:7-9.
"""
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

EKF2_BOOT_PARAMS = (
    ("EKF2_HGT_REF", 0, MAV_PARAM_TYPE_INT32),
)
"""Phase 0 -- @reboot_required, so PX4 must be restarted after these are set.

`EKF2_HGT_REF` ekf2_params.c:657, default 1 (GPS). 0 = barometric, which is
where the height genuinely comes from. *** @reboot_required true
(ekf2_params.c:656). *** A PARAM_SET at runtime updates the STORED value -- QGC
will show 0 -- but EKF2 does not re-read its height reference until PX4
restarts.
"""

EKF2_FUSION_PARAMS = (
    ("EKF2_EV_CTRL", 9, MAV_PARAM_TYPE_INT32),
    ("EKF2_EV_NOISE_MD", 1, MAV_PARAM_TYPE_INT32),
    ("EKF2_EVP_NOISE", 0.5, MAV_PARAM_TYPE_REAL32),
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
  EKF2_EVP_NOISE    ekf2_params.c:837, default 0.1 m -- far too tight for a
  EKF2_EVA_NOISE    ekf2_params.c:857, default 0.1 rad -- drifting estimator;
                    EKF2 would reject its own vision source as inconsistent.
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

EKF2_VISION_PARAMS = EKF2_BOOT_PARAMS + EKF2_FUSION_PARAMS + EKF2_GPS_DENIED_PARAMS
"""Every param the vision profile touches. The phases above are how they are
APPLIED; this is what the whole profile amounts to."""

HGT_REF_NEEDS_REBOOT = tuple(name for name, _, _ in EKF2_BOOT_PARAMS)
"""Params that PX4 only re-reads at boot.

Setting these on a running PX4 is not enough: the value sticks, the behaviour
does not change. Kept as data so the caller can act rather than silently
believing a param it can see took effect.
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
    """One estimate, in the estimator's ENU frame. Angles in radians."""
    ts_ns: int
    x: float          # East
    y: float          # North
    z: float          # Up
    roll: float
    pitch: float
    yaw: float


def enu_to_ned(e, n, u):
    """(East, North, Up) -> (North, East, Down)."""
    return (n, e, -u)


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


def apply_ekf2_boot_params(link):
    """Phase 0. Setpoint thread only. Requires a reboot to take effect."""
    _apply(link, EKF2_BOOT_PARAMS)


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


def reboot_for_boot_params(link):
    """Set the @reboot_required params and restart PX4. Setpoint thread only.

    PX4 refuses a reboot while armed, so this is a pre-flight act by
    construction. The caller is expected to re-apply the remaining phases once
    PX4 comes back -- joystick-server.py already re-sends its startup params on
    a heartbeat gap, which is exactly what a reboot looks like from outside.
    """
    apply_ekf2_boot_params(link)
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
    """Streams VISION_POSITION_ESTIMATE. Setpoint thread only."""

    def __init__(self, conn, max_age_s=DEFAULT_MAX_AGE_S):
        self.conn = conn
        self.max_age_s = max_age_s
        self.sent = 0
        self.dropped_stale = 0

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
        """
        if pose is None:
            return False
        if received_at is not None and (now - received_at) > self.max_age_s:
            self.dropped_stale += 1
            return False
        x, y, z = enu_to_ned(pose.x, pose.y, pose.z)
        self.conn.mav.vision_position_estimate_send(
            int(pose.ts_ns // 1000),        # usec
            float(x), float(y), float(z),
            float(pose.roll), float(pose.pitch), float(pose.yaw))
        self.sent += 1
        return True
