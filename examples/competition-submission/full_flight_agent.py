"""full_flight_agent.py -- REFERENCE EXAMPLE, not runnable against this
repo's sandbox today (see single_flight.py's docstring for why). Flies a
toy lawnmower sweep entirely on flight() -- deliberately not Route, which
is still unresolved, see
docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md's
open Route-vs-ATE question. Integrates a detection pipeline: nadir camera
by default, detection.Detector runs in on_frame, and a confident hit
switches to the oblique camera and orbits briefly before resuming the
sweep where it left off.

detection.py owns everything about the model; this file owns everything
about flying. See ../../docs/superpowers/specs/
2026-09-08-competitor-submission-docker-design.md for the Docker
packaging this pairs with.
"""
import math

from competition import Agent, Command, flight

from detection import Detector

EARTH_R = 6371000.0

SWEEP_ALT_M = 40.0
LEG_SPACING_M = 60.0
LEG_LENGTH_M = 150.0
LEGS = 3
ARRIVAL_RADIUS_M = 5.0
DETECT_CONFIDENCE = 0.6
ORBIT_SECONDS = 6.0


def _lawnmower(spacing_m, legs, length_m):
    """Toy boustrophedon pattern as (north_m, east_m) offsets from launch.
    A real submission would derive this from arena.bounds -- kept simple
    here so the example stays about flight()+detection, not coverage
    planning."""
    pts = []
    for i in range(legs):
        north = i * spacing_m
        pts.append((north, 0.0))
        pts.append((north, length_m if i % 2 == 0 else -length_m))
    return pts


def _offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / EARTH_R
    dlon = east_m / (EARTH_R * max(math.cos(math.radians(lat)), 1e-6))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def _bearing_body(cur_lat, cur_lon, cur_heading_deg, tgt_lat, tgt_lon):
    """Same Decision-5 rotation as single_flight.py -- duplicated rather
    than shared to keep each example file self-contained and copy-pasteable."""
    phi1, phi2 = math.radians(cur_lat), math.radians(tgt_lat)
    dlambda = math.radians(tgt_lon - cur_lon)
    y = math.sin(dlambda) * math.cos(phi2)
    x = (math.cos(phi1) * math.sin(phi2)
         - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda))
    bearing = math.degrees(math.atan2(y, x))
    dphi = math.radians(tgt_lat - cur_lat)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    dist = 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))
    north = dist * math.cos(math.radians(bearing))
    east = dist * math.sin(math.radians(bearing))
    h = math.radians(cur_heading_deg)
    rx = north * math.sin(h) + east * math.cos(h)
    ry = north * math.cos(h) - east * math.sin(h)
    return rx, ry, dist


class FullFlightAgent(Agent):
    """Sweep for a target on the nadir camera; on a confident hit, switch
    to the oblique camera and orbit briefly, then resume the sweep."""

    def on_start(self, arena):
        self.detector = Detector("weights.pt")     # baked into the image -- see Dockerfile
        self.waypoints = _lawnmower(LEG_SPACING_M, LEGS, LEG_LENGTH_M)
        self.wp_index = 0
        self.origin = None
        self.phase = "sweep"          # sweep -> orbit -> sweep ...
        self.orbit_start = None
        return Command(camera="nadir", flight=flight(0, 0, 0, 0))

    def on_tick(self, state):
        if state.lat is None:
            return None
        if self.origin is None:
            self.origin = (state.lat, state.lon)

        if self.phase == "orbit":
            if state.time_elapsed - self.orbit_start > ORBIT_SECONDS:
                self.phase = "sweep"
                return Command(camera="nadir", flight=flight(0, 0, 0, 0))
            return Command(flight=flight(0.4, 0, 0, 0))   # slow yaw -- crude orbit

        return self._sweep_step(state)

    def on_frame(self, image, state):
        if image is None or self.phase != "sweep":
            return None
        hits = [d for d in self.detector.detect(image)
                if d.confidence >= DETECT_CONFIDENCE]
        if not hits:
            return None
        self.phase = "orbit"
        self.orbit_start = state.time_elapsed
        return Command(camera="oblique", flight=flight(0, 0, 0, 0))

    def _sweep_step(self, state):
        if self.wp_index >= len(self.waypoints):
            return Command(flight=flight(0, 0, 0, 0))   # sweep complete -- hold
        north, east = self.waypoints[self.wp_index]
        tgt_lat, tgt_lon = _offset_latlon(*self.origin, north, east)
        rx, ry, dist = _bearing_body(state.lat, state.lon, state.heading, tgt_lat, tgt_lon)
        if dist < ARRIVAL_RADIUS_M:
            self.wp_index += 1
            return Command(flight=flight(0, 0, 0, 0))
        alt_err = SWEEP_ALT_M - state.alt_agl
        mag = math.hypot(rx, ry) or 1.0
        push = min(1.0, dist / 20.0)
        climb = max(-1.0, min(1.0, alt_err * 0.5))
        return Command(flight=flight(0, climb, rx / mag * push, ry / mag * push))
