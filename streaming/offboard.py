"""PX4 OFFBOARD velocity control for the web-joystick PoC.

Proportional dual-stick control: pitch, roll, yaw, thrust, all continuous
in [-1, 1] -- see docs/superpowers/specs/2026-09-07-joystick-analog-sticks-design.md
(supersedes the four-command, strafe-out-of-scope version described in
docs/superpowers/specs/2026-07-30-joystick-offboard-design.md).

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

DEFAULT_YAW_RATE_DPS = 45.0

DEADZONE = 0.05


def _clamp(v):
    return max(-1.0, min(1.0, v))


def axes_to_body_velocity(pitch, roll, thrust, speed_fwd, speed_right, speed_up):
    """Map proportional stick deflection to a body-NED velocity setpoint.

    pitch/roll/thrust are each in [-1, 1]: full-back to full-forward
    (pitch), full-left to full-right (roll), full-down to full-up
    (thrust). NED means +vx is nose-forward, +vy is right, and +vz is
    DOWN, so climbing (+thrust) is negative vz. Callers (CommandState)
    own clamping and the deadzone; this function only scales.
    """
    vx = pitch * speed_fwd
    vy = roll * speed_right
    vz = -thrust * speed_up
    return vx, vy, vz


def axes_to_yaw_rate(yaw, yaw_rate_rps):
    """yaw in [-1, 1] -> rad/s. Positive yaw turns right (NED yaw is
    positive clockwise viewed from above) -- CommandState.set_stick maps
    a rightward stick drag to positive yaw, so the signs already agree
    here with no flip needed."""
    return yaw * yaw_rate_rps


class CommandState:
    """Thread-safe proportional stick state with a staleness watchdog.

    The web thread writes; the setpoint thread reads. If the browser stops
    talking -- crash, wifi drop, backgrounded tab -- velocity decays to zero
    instead of latching the last command at full speed.
    """

    def __init__(self, speed_fwd=2.0, speed_up=1.0, watchdog_s=0.5,
                 yaw_rate_dps=DEFAULT_YAW_RATE_DPS, speed_right=1.5):
        self.speed_fwd = speed_fwd
        self.speed_up = speed_up
        self.speed_right = speed_right
        self.watchdog_s = watchdog_s
        self.yaw_rate_rps = math.radians(yaw_rate_dps)
        self._lock = threading.Lock()
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0
        self._thrust = 0.0
        self._last_input = 0.0

    def _is_active(self):
        return bool(self._pitch or self._roll or self._yaw or self._thrust)

    def set_stick(self, stick, x, y, now=None):
        """Update one stick's pair of axes from raw DOM-offset x/y in
        [-1, 1] (right/down positive) -- the client sends screen
        coordinates and never applies flight sign conventions itself.

        Left stick: x -> yaw, y -> thrust (up-drag climbs, so thrust is
        -y). Right stick: x -> roll, y -> pitch (up-drag pitches
        forward, so pitch is -y too). Roll needs no flip: dragging right
        (+x) strafes right, and body-NED +vy is right.

        Returns True exactly when this call moves the whole command from
        all-axes-at-rest to at least one axis off-center (past the
        deadzone) -- the edge callers use to trigger mission-pause /
        agent-abort on manual takeover.
        """
        x = _clamp(x)
        y = _clamp(y)
        if abs(x) < DEADZONE:
            x = 0.0
        if abs(y) < DEADZONE:
            y = 0.0
        with self._lock:
            was_active = self._is_active()
            if stick == "left":
                self._yaw = x
                self._thrust = -y
            elif stick == "right":
                self._roll = x
                self._pitch = -y
            else:
                raise ValueError(f"unknown stick {stick!r}; expected 'left' or 'right'")
            self._last_input = time.monotonic() if now is None else now
            is_active = self._is_active()
        return (not was_active) and is_active

    def touch(self, now=None):
        """Keepalive. The watchdog measures time since the last message of
        any kind, and dragging sends a message only on change, so the page
        must refresh the stamp periodically while a stick is off-center."""
        with self._lock:
            self._last_input = time.monotonic() if now is None else now

    def clear(self):
        with self._lock:
            self._pitch = self._roll = self._yaw = self._thrust = 0.0

    def command(self, now=None):
        """Everything one setpoint needs: (vx, vy, vz, yaw_rate).

        Single method rather than separate velocity/yaw getters so both are
        read under one lock against one staleness check -- otherwise a
        watchdog expiry between two calls could zero the velocity while
        leaving the aircraft still turning.
        """
        now = time.monotonic() if now is None else now
        with self._lock:
            pitch, roll, yaw, thrust = self._pitch, self._roll, self._yaw, self._thrust
            last = self._last_input
        if (pitch or roll or yaw or thrust) and (now - last) > self.watchdog_s:
            return 0.0, 0.0, 0.0, 0.0
        vx, vy, vz = axes_to_body_velocity(
            pitch, roll, thrust, self.speed_fwd, self.speed_right, self.speed_up)
        return vx, vy, vz, axes_to_yaw_rate(yaw, self.yaw_rate_rps)


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
