"""Waypoint sequencing for autonomous missions.

Pure logic: no MAVLink, no I/O. The setpoint loop asks advance() for a target
every tick and sends whatever it gets back; everything about "which waypoint
are we on" lives here and nowhere else.

Distances are great-circle. Legs in this application are tens to hundreds of
metres, where a flat approximation would also work -- haversine is used because
it is the same handful of lines and has no latitude at which it quietly stops
being true.

Design: docs/superpowers/specs/2026-07-31-map-waypoints-design.md
"""
import math
import threading

EARTH_RADIUS_M = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance between two lat/lon points, in metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing, in compass degrees (0 = N, 90 = E).

    "Initial" matters: on a long leg the bearing changes as you fly it. Legs
    here are short enough that it does not, but the loop recomputes every tick
    anyway, so the aircraft tracks the true course either way.
    """
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(p2)
    x = (math.cos(p1) * math.sin(p2)
         - math.sin(p1) * math.cos(p2) * math.cos(dlam))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


class Mission:
    """Waypoint sequencer: owns the route, the altitude and the progress.

    Thread-safe by construction. The web thread calls load/fly/pause/clear;
    the setpoint thread calls advance() twenty times a second. The lock lives
    here so no caller has to remember it.

    States:
      IDLE     nothing planned, or planned but not started
      RUNNING  flying the route
      PAUSED   operator took manual control; route retained
      DONE     past the last waypoint, holding it
    """

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    DONE = "DONE"

    def __init__(self, arrival_radius_m=2.0):
        # Not zero: a position setpoint is a target, not a guarantee, and PX4
        # settles within about a metre. A zero radius hangs on waypoint 1
        # forever.
        self.arrival_radius_m = arrival_radius_m
        self._lock = threading.Lock()
        self._points = []
        self._alt_m = 0.0
        self._index = 0
        self._state = self.IDLE
        self._hold_yaw = 0.0
        self._dist_m = None

    def load(self, points, alt_m):
        """Replace the route. Never starts it -- planning must not fly."""
        with self._lock:
            self._points = [(float(lat), float(lon)) for lat, lon in points]
            self._alt_m = float(alt_m)
            self._index = 0
            self._state = self.IDLE
            self._dist_m = None

    def fly(self):
        """Start, or resume from where a pause left off.

        From DONE this re-flies the loaded route from waypoint 1, which is what
        makes repeating a survey one button press.
        """
        with self._lock:
            if not self._points:
                return
            if self._state == self.DONE:
                self._index = 0
            self._state = self.RUNNING

    def pause(self):
        with self._lock:
            if self._state == self.RUNNING:
                self._state = self.PAUSED

    def clear(self):
        with self._lock:
            self._points = []
            self._index = 0
            self._state = self.IDLE
            self._dist_m = None

    def status(self):
        """Snapshot for telemetry. `index` is 0-based; the UI adds 1."""
        with self._lock:
            return {"state": self._state,
                    "index": self._index,
                    "count": len(self._points),
                    "dist_m": self._dist_m}
