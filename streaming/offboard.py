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
