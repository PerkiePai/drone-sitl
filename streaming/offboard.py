"""PX4 OFFBOARD velocity control for the web-joystick PoC.

Four commands only: climb, descend, forward, backward. Strafe and yaw are
deliberately out of scope -- see
docs/superpowers/specs/2026-07-30-joystick-offboard-design.md.

Every MAVLink constant below was read out of ~/PX4-Autopilot rather than
recalled; the source file and line are on each one. test_offboard.py also
asserts they agree with pymavlink's dialect.
"""
import math
import threading
import time

# SET_POSITION_TARGET_LOCAL_NED coordinate frame. PX4 rotates vx/vy by yaw
# only and passes vz through as world-down (mavlink_receiver.cpp:970-991),
# which is exactly what we want: "forward" tracks the nose, "up" stays up
# regardless of attitude.
MAV_FRAME_BODY_NED = 8

# World-frame velocity setpoints, for the agent API's VelocityWorld. Unlike
# BODY_NED, PX4 applies no yaw rotation -- vx/vy are world north/east and vz is
# world-down (mavlink_receiver.cpp:970-991).
MAV_FRAME_LOCAL_NED = 1

# type_mask: ignore position (bits 0-2), ignore acceleration (bits 6-8),
# ignore yaw (bit 10). USE velocity (bits 3-5) and yaw_rate (bit 11 clear).
# yaw_rate = 0 holds the heading; non-zero turns. PX4 applies yawspeed outside
# its frame switch (mavlink_receiver.cpp:1025), so this works in BODY_NED.
VEL_YAWRATE_TYPE_MASK = 1479

# Global-frame position setpoints, for autonomous waypoints. PX4 projects
# lat/lon to local NED itself, using the estimator's own reference
# (mavlink_receiver.cpp:1063-1093), so nothing on this side owns a map
# projection and our idea of a position cannot drift from PX4's.
#
# Altitude is relative to HOME. PX4 requires home_position.valid_alt and
# returns SILENTLY without it (mavlink_receiver.cpp:1107-1110), which is why
# the server gates FLY on having received HOME_POSITION.
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

# type_mask: ignore velocity (bits 3-5), acceleration (bits 6-8) and yaw_rate
# (bit 11). USE position (bits 0-2 clear) and yaw (bit 10 clear), so the nose
# -- and therefore the camera -- points along the leg being flown.
POS_YAW_TYPE_MASK = 2552

MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

# PX4 custom modes (px4_custom_mode.h:44-67). MAV_CMD_DO_SET_MODE takes these
# as SEPARATE params -- param1=base_mode, param2=main, param3=sub
# (Commander.cpp:737-738) -- not bit-packed into one custom_mode field. The
# bit-packed form appears only in HEARTBEAT, decoded by decode_px4_mode().
PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_SUB_MODE_AUTO_TAKEOFF = 2
PX4_SUB_MODE_AUTO_LAND = 6

# RC-loss exception bitmask (commander_params.c:808-820); bit 2 = Offboard.
# Without this, PX4's default NAV_RCL_ACT=2 fires a Return-mode failsafe in
# SITL -- where there is no RC transmitter -- and flies the drone away
# mid-demo.
COM_RCL_EXCEPT_OFFBOARD = 4

DIRECTIONS = ("fwd", "back", "yaw_left", "yaw_right", "up", "down")

DEFAULT_YAW_RATE_DPS = 45.0


def axes_to_body_velocity(held, speed_fwd, speed_up):
    """Map held joystick directions to a body-NED velocity setpoint.

    NED means +vx is nose-forward and +vz is DOWN, so climbing is negative vz.
    vy is always 0.0 -- sideways strafe is out of scope; the left/right buttons
    turn the aircraft instead (see axes_to_yaw_rate). Opposing directions
    cancel.
    """
    vx = 0.0
    if "fwd" in held:
        vx += speed_fwd
    if "back" in held:
        vx -= speed_fwd
    vz = 0.0
    if "up" in held:
        vz -= speed_up
    if "down" in held:
        vz += speed_up
    return vx, 0.0, vz


def axes_to_yaw_rate(held, yaw_rate_rps):
    """Map held turn directions to a yaw rate in rad/s.

    NED yaw is positive clockwise viewed from above, so turning right is
    positive. Opposing directions cancel.

    Because setpoints go out in BODY_NED, turning also rotates what "forward"
    means -- PX4 resolves vx against the current heading every tick, so
    forward keeps tracking the nose with no extra work here.
    """
    rate = 0.0
    if "yaw_right" in held:
        rate += yaw_rate_rps
    if "yaw_left" in held:
        rate -= yaw_rate_rps
    return rate


class CommandState:
    """Thread-safe held-direction set with a staleness watchdog.

    The web thread writes; the setpoint thread reads. If the browser stops
    talking -- crash, wifi drop, backgrounded tab -- velocity decays to zero
    instead of latching the last command at full speed.
    """

    def __init__(self, speed_fwd=2.0, speed_up=1.0, watchdog_s=0.5,
                 yaw_rate_dps=DEFAULT_YAW_RATE_DPS):
        self.speed_fwd = speed_fwd
        self.speed_up = speed_up
        self.watchdog_s = watchdog_s
        self.yaw_rate_rps = math.radians(yaw_rate_dps)
        self._lock = threading.Lock()
        self._held = set()
        self._last_input = 0.0

    def set(self, direction, pressed, now=None):
        if direction not in DIRECTIONS:
            raise ValueError(
                f"unknown direction {direction!r}; expected one of {DIRECTIONS}")
        with self._lock:
            if pressed:
                self._held.add(direction)
            else:
                self._held.discard(direction)
            self._last_input = time.monotonic() if now is None else now

    def touch(self, now=None):
        """Keepalive. The watchdog measures time since the last message of any
        kind, and holding a button produces exactly one message, so the page
        must refresh the stamp periodically."""
        with self._lock:
            self._last_input = time.monotonic() if now is None else now

    def clear(self):
        with self._lock:
            self._held.clear()

    def held(self):
        with self._lock:
            return set(self._held)

    def command(self, now=None):
        """Everything one setpoint needs: (vx, vy, vz, yaw_rate).

        Single method rather than separate velocity/yaw getters so both are
        read under one lock against one staleness check -- otherwise a
        watchdog expiry between two calls could zero the velocity while
        leaving the aircraft still turning.
        """
        now = time.monotonic() if now is None else now
        with self._lock:
            held = set(self._held)
            last = self._last_input
        if held and (now - last) > self.watchdog_s:
            return 0.0, 0.0, 0.0, 0.0
        vx, vy, vz = axes_to_body_velocity(held, self.speed_fwd, self.speed_up)
        return vx, vy, vz, axes_to_yaw_rate(held, self.yaw_rate_rps)


PX4_MODE_NAMES = {
    (1, 0): "MANUAL", (2, 0): "ALTCTL", (3, 0): "POSCTL",
    (4, 1): "AUTO.READY", (4, 2): "AUTO.TAKEOFF", (4, 3): "AUTO.LOITER",
    (4, 4): "AUTO.MISSION", (4, 5): "AUTO.RTL", (4, 6): "AUTO.LAND",
    (5, 0): "ACRO", (6, 0): "OFFBOARD", (7, 0): "STABILIZED",
}


def decode_px4_mode(custom_mode):
    """HEARTBEAT.custom_mode -> readable PX4 mode name.

    PX4 packs main mode at bits 16-23 and sub mode at bits 24-31. Unknown
    combinations render as mode(main.sub) so an unexpected failsafe shows up
    in the UI as something specific rather than as a blank.
    """
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    name = PX4_MODE_NAMES.get((main, sub))
    if name is None:
        name = PX4_MODE_NAMES.get((main, 0), f"mode({main}.{sub})")
    return name


class OffboardLink:
    """Every MAVLink write for the joystick PoC.

    Single-threaded by contract: pymavlink connections are not thread-safe,
    so only the setpoint thread may call these methods.
    """

    def __init__(self, conn):
        self.conn = conn
        self.target_system = 1
        self.target_component = 1

    def bind_target(self, msg):
        """Adopt sysid/compid from a received HEARTBEAT."""
        self.target_system = msg.get_srcSystem()
        self.target_component = msg.get_srcComponent()

    def send_velocity(self, vx, vy, vz, yaw_rate=0.0):
        self.conn.mav.set_position_target_local_ned_send(
            0,                                  # time_boot_ms (PX4 ignores)
            self.target_system, self.target_component,
            MAV_FRAME_BODY_NED,
            VEL_YAWRATE_TYPE_MASK,
            0.0, 0.0, 0.0,                      # x, y, z    -- masked off
            vx, vy, vz,
            0.0, 0.0, 0.0,                      # afx, afy, afz -- masked off
            0.0,                                # yaw        -- masked off
            yaw_rate)                           # rad/s; 0 holds heading

    def send_velocity_world(self, vn, ve, vd, yaw_rate=0.0):
        """World-frame velocity: vn north, ve east, vd DOWN (NED). The caller
        has already flipped the API's +up to vd -- see agent_control.py. Same
        setpoint-thread-only contract as send_velocity."""
        self.conn.mav.set_position_target_local_ned_send(
            0,                                  # time_boot_ms (PX4 ignores)
            self.target_system, self.target_component,
            MAV_FRAME_LOCAL_NED,
            VEL_YAWRATE_TYPE_MASK,
            0.0, 0.0, 0.0,                      # x, y, z    -- masked off
            vn, ve, vd,
            0.0, 0.0, 0.0,                      # afx, afy, afz -- masked off
            0.0,                                # yaw        -- masked off
            yaw_rate)                           # rad/s

    def send_position_global(self, lat, lon, rel_alt_m, yaw_deg):
        """Fly to a lat/lon at an altitude relative to home, nose on yaw_deg.

        Same contract as send_velocity: setpoint-thread only. Altitude is
        RELATIVE to home, not AMSL -- matching the altitude the UI displays.
        """
        self.conn.mav.set_position_target_global_int_send(
            0,                                  # time_boot_ms (PX4 ignores)
            self.target_system, self.target_component,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            POS_YAW_TYPE_MASK,
            int(lat * 1e7), int(lon * 1e7),     # degrees -> 1e7 fixed point
            float(rel_alt_m),
            0.0, 0.0, 0.0,                      # vx, vy, vz    -- masked off
            0.0, 0.0, 0.0,                      # afx, afy, afz -- masked off
            math.radians(yaw_deg),
            0.0)                                # yaw_rate      -- masked off

    def _command_long(self, command, *params):
        padded = list(params) + [0.0] * (7 - len(params))
        self.conn.mav.command_long_send(
            self.target_system, self.target_component, command, 0, *padded)

    def set_mode(self, main_mode, sub_mode=0):
        self._command_long(
            MAV_CMD_DO_SET_MODE,
            float(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(main_mode),
            float(sub_mode))

    def arm(self):
        self._command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self._command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0)

    def takeoff(self):
        """AUTO.TAKEOFF climbs to MIS_TAKEOFF_ALT, set at startup."""
        self.set_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_TAKEOFF)

    def land(self):
        self.set_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LAND)

    def offboard(self):
        self.set_mode(PX4_MAIN_MODE_OFFBOARD)

    def set_param(self, name, value, param_type):
        self.conn.mav.param_set_send(
            self.target_system, self.target_component,
            name.encode() if isinstance(name, str) else name,
            float(value), param_type)
