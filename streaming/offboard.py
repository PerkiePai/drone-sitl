"""PX4 OFFBOARD velocity control for the web-joystick PoC.

Four commands only: climb, descend, forward, backward. Strafe and yaw are
deliberately out of scope -- see
docs/superpowers/specs/2026-07-30-joystick-offboard-design.md.

Every MAVLink constant below was read out of ~/PX4-Autopilot rather than
recalled; the source file and line are on each one. test_offboard.py also
asserts they agree with pymavlink's dialect.
"""
import threading
import time

# SET_POSITION_TARGET_LOCAL_NED coordinate frame. PX4 rotates vx/vy by yaw
# only and passes vz through as world-down (mavlink_receiver.cpp:970-991),
# which is exactly what we want: "forward" tracks the nose, "up" stays up
# regardless of attitude.
MAV_FRAME_BODY_NED = 8

# type_mask: ignore position (bits 0-2), ignore acceleration (bits 6-8),
# ignore yaw (bit 10). USE velocity (bits 3-5) and yaw_rate (bit 11 clear).
# Sending yaw_rate = 0 with yaw ignored is what holds the heading.
VEL_YAWRATE_TYPE_MASK = 1479

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

DIRECTIONS = ("fwd", "back", "up", "down")


def axes_to_body_velocity(held, speed_fwd, speed_up):
    """Map held joystick directions to a body-NED velocity setpoint.

    NED means +vx is nose-forward and +vz is DOWN, so climbing is negative vz.
    vy is always 0.0 -- strafe is out of scope. Opposing directions cancel.
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


class CommandState:
    """Thread-safe held-direction set with a staleness watchdog.

    The web thread writes; the setpoint thread reads. If the browser stops
    talking -- crash, wifi drop, backgrounded tab -- velocity decays to zero
    instead of latching the last command at full speed.
    """

    def __init__(self, speed_fwd=2.0, speed_up=1.0, watchdog_s=0.5):
        self.speed_fwd = speed_fwd
        self.speed_up = speed_up
        self.watchdog_s = watchdog_s
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

    def velocity(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            held = set(self._held)
            last = self._last_input
        if held and (now - last) > self.watchdog_s:
            return 0.0, 0.0, 0.0
        return axes_to_body_velocity(held, self.speed_fwd, self.speed_up)


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

    def send_velocity(self, vx, vy, vz):
        self.conn.mav.set_position_target_local_ned_send(
            0,                                  # time_boot_ms (PX4 ignores)
            self.target_system, self.target_component,
            MAV_FRAME_BODY_NED,
            VEL_YAWRATE_TYPE_MASK,
            0.0, 0.0, 0.0,                      # x, y, z    -- masked off
            vx, vy, vz,
            0.0, 0.0, 0.0,                      # afx, afy, afz -- masked off
            0.0,                                # yaw        -- masked off
            0.0)                                # yaw_rate=0 -> hold heading

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
