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
# deliberate, safety-relevant act, never a default.
#
#   EKF2_GPS_CTRL     ekf2_params.c:706, default 7. 0 disables ALL GNSS fusion.
#   EKF2_EV_CTRL      ekf2_params.c:687, default 15. bit0 horizontal position,
#                     bit1 vertical position, bit2 3D velocity, bit3 yaw.
#                     9 = horizontal position + yaw, which is exactly what
#                     flow-odom measures. Vertical is deliberately NOT claimed:
#                     altitude comes straight from the barometer
#                     (flow_odometry.py:455), and EKF2 already fuses baro
#                     directly (EKF2_BARO_CTRL default 1), so setting bit1 would
#                     feed one sensor in twice and read as spurious agreement.
#   EKF2_HGT_REF      ekf2_params.c:657, default 1 (GPS). 0 = barometric, which
#                     is where the height genuinely comes from.
#                     *** @reboot_required true (ekf2_params.c:656). ***
#                     A PARAM_SET at runtime updates the STORED value -- QGC
#                     will show 0 -- but EKF2 does not re-read its height
#                     reference until PX4 restarts. See HGT_REF_NEEDS_REBOOT.
#   EKF2_EV_NOISE_MD  ekf2_params.c:814, default 0. 1 = use the noise params
#                     below rather than a reported variance we do not compute.
#   EKF2_EVP_NOISE    ekf2_params.c:837, default 0.1 m -- far too tight for a
#   EKF2_EVA_NOISE    ekf2_params.c:857, default 0.1 rad -- drifting estimator;
#                     EKF2 would reject its own vision source as inconsistent.
EKF2_VISION_PARAMS = (
    ("EKF2_GPS_CTRL", 0, MAV_PARAM_TYPE_INT32),
    ("EKF2_EV_CTRL", 9, MAV_PARAM_TYPE_INT32),
    ("EKF2_HGT_REF", 0, MAV_PARAM_TYPE_INT32),
    ("EKF2_EV_NOISE_MD", 1, MAV_PARAM_TYPE_INT32),
    ("EKF2_EVP_NOISE", 0.5, MAV_PARAM_TYPE_REAL32),
    ("EKF2_EVA_NOISE", 0.2, MAV_PARAM_TYPE_REAL32),
)

HGT_REF_NEEDS_REBOOT = ("EKF2_HGT_REF",)
"""Params that PX4 only re-reads at boot.

Setting these on a running PX4 is not enough: the value sticks, the behaviour
does not change. They have to be in place BEFORE the flight controller starts,
which for SITL means the startup script rather than a PARAM_SET at connect
time. Kept as data so the caller can warn rather than silently believing a
param it can see took effect.
"""

DEFAULT_MAX_AGE_S = 0.5


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


def apply_ekf2_vision_params(link):
    """Push the vision param set. Setpoint thread only."""
    for name, value, ptype in EKF2_VISION_PARAMS:
        link.set_param(name, value, ptype)


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
        one. `received_at` is wall time (time.monotonic) at which the estimate
        arrived over ZMQ -- not pose.ts_ns, which is SIM time and runs at its
        own rate under lockstep.
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
