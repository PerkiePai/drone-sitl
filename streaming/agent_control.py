"""The agent's velocity setpoint holder.

Mirrors offboard.CommandState. The /agent/control WebSocket handler (web
thread) writes; SetpointLoop reads command() once per tick. A child that
stops sending decays to hover after watchdog_s -- a stuck velocity is a
hazard, same as a stuck joystick button.

Route does NOT come here -- it goes through waypoints.Mission, exactly like
the operator's map route. This holds only flight() (translated to a body
velocity via offboard.axes_to_body_velocity/axes_to_yaw_rate -- the same
conversion CommandState already does for the human joystick UI, see
docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md
Decision 3) and the internal hold() safety latch.

UNITS IN: m/s and deg/s, `up` positive. UNITS OUT: m/s and rad/s, NED (down
positive). The +up -> NED flip is here and nowhere else.
"""
import math
import threading
import time

import offboard


class AgentControl:
    def __init__(self, watchdog_s=0.5):
        self.watchdog_s = watchdog_s
        self._lock = threading.Lock()
        self._kind = None          # None | "body"
        self._vec = (0.0, 0.0, 0.0, 0.0)
        self._held = False         # hold(): latched, not watchdogged
        self._stamp = 0.0

    def _set(self, kind, vec, held, now):
        with self._lock:
            self._kind = kind
            self._vec = vec
            self._held = held
            self._stamp = time.monotonic() if now is None else now

    def set_velocity_body(self, forward, right, up, yaw_rate, now=None):
        self._set("body", (forward, right, -up, math.radians(yaw_rate)),
                  False, now)

    def set_flight(self, left_x, left_y, right_x, right_y, *,
                   speed_fwd, speed_right, speed_up, yaw_rate_dps, now=None):
        """flight()'s four stick axes -> a body velocity, reusing exactly
        the conversion the human joystick UI already uses. axes_to_body_velocity
        returns vz in NED (down positive); set_velocity_body expects +up and
        flips it itself, so undo that flip here before handing it over."""
        vx, vy, vz_ned = offboard.axes_to_body_velocity(
            right_y, right_x, left_y, speed_fwd, speed_right, speed_up)
        yaw_rate_dps_value = left_x * yaw_rate_dps
        self.set_velocity_body(vx, vy, -vz_ned, yaw_rate_dps_value, now)

    def hold(self, now=None):
        self._set("body", (0.0, 0.0, 0.0, 0.0), True, now)

    def clear(self):
        with self._lock:
            self._kind = None
            self._held = False

    def active(self):
        with self._lock:
            return self._kind is not None

    def command(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._kind is None:
                return None
            a, b, c, d = self._vec
            if not self._held and (now - self._stamp) > self.watchdog_s:
                a = b = c = d = 0.0
            return (self._kind, a, b, c, d)
