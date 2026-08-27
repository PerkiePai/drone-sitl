"""The agent's velocity setpoint holder.

Mirrors offboard.CommandState. The /agent/control WebSocket handler (web
thread) writes; SetpointLoop reads command() once per tick. A child that
stops sending decays to hover after watchdog_s -- a stuck velocity is a
hazard, same as a stuck joystick button.

Goto and Route do NOT come here -- they go through waypoints.Mission, exactly
like the operator's map route. This holds only Velocity, VelocityWorld and
the explicit Hold.

UNITS IN: m/s and deg/s, `up` positive. UNITS OUT: m/s and rad/s, NED (down
positive). The +up -> NED flip is here and nowhere else.
"""
import math
import threading
import time


class AgentControl:
    def __init__(self, watchdog_s=0.5):
        self.watchdog_s = watchdog_s
        self._lock = threading.Lock()
        self._kind = None          # None | "body" | "world"
        self._vec = (0.0, 0.0, 0.0, 0.0)
        self._held = False         # a Hold(): latched, not watchdogged
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

    def set_velocity_world(self, north, east, up, yaw_rate, now=None):
        self._set("world", (north, east, -up, math.radians(yaw_rate)),
                  False, now)

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
