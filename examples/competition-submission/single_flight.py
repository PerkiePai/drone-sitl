"""single_flight.py -- the smallest possible flight() script: hover while
acquiring position, fly to a fixed offset waypoint, then spin 360 in
place. Runs against competition/ as it stands; needs no deps beyond the
package itself.

flight(left_x, left_y, right_x, right_y), each in [-1, 1]:
    left_y  = thrust   (+1 = full climb)
    left_x  = yaw       (+1 = full right / clockwise)
    right_y = pitch      (+1 = full forward)
    right_x = roll        (+1 = full right / strafe)
Full deflection on a translation axis = 5 m/s. See
docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md
Decisions 3-5 for the full rationale -- this file is that spec's "A -> B,
then spin" worked example, promoted to a complete, self-contained file.
"""
import math

from competition import Agent, Command, flight

EARTH_R = 6371000.0

TARGET_ALT_M = 15.0
TARGET_OFFSET_M = (40.0, 25.0)   # (north, east) from wherever the flight starts
ARRIVAL_RADIUS_M = 2.0
SPIN_SECONDS = 4.0


def _offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / EARTH_R
    dlon = east_m / (EARTH_R * max(math.cos(math.radians(lat)), 1e-6))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def _bearing_body(cur_lat, cur_lon, cur_heading_deg, tgt_lat, tgt_lon):
    """World bearing/distance to (tgt_lat, tgt_lon), rotated into
    flight()'s body frame -- Decision 5's rotation, spelled out here
    rather than imported since the real spec leaves it to each script."""
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
    rx = north * math.sin(h) + east * math.cos(h)     # body-right
    ry = north * math.cos(h) - east * math.sin(h)     # body-forward
    return rx, ry, dist


class SingleFlightAgent(Agent):
    """hover -> fly to a 47 m offset waypoint at 15 m alt -> spin -> hold."""

    def on_start(self, arena):
        self.target = None          # set on the first tick, once we know where we are
        self.phase = "fly"          # fly -> spin -> done
        self.spin_start = None
        return Command(flight=flight(0, 0, 0, 0))   # hover while acquiring position

    def on_tick(self, state):
        if state.lat is None:
            return None              # no position yet -- keep hovering

        if self.target is None:
            north, east = TARGET_OFFSET_M
            self.target = _offset_latlon(state.lat, state.lon, north, east)

        if self.phase == "fly":
            return self._fly_step(state)
        if self.phase == "spin":
            return self._spin_step(state)
        return None                  # done -- keep the last command (hover)

    def _fly_step(self, state):
        rx, ry, dist = _bearing_body(state.lat, state.lon, state.heading, *self.target)
        alt_err = TARGET_ALT_M - state.alt_agl
        if dist < ARRIVAL_RADIUS_M and abs(alt_err) < 1.0:
            self.phase = "spin"
            self.spin_start = state.time_elapsed
            return Command(flight=flight(0, 0, 0, 0))
        mag = math.hypot(rx, ry) or 1.0
        push = min(1.0, dist / 15.0)          # decelerate on approach
        climb = max(-1.0, min(1.0, alt_err * 0.5))
        return Command(flight=flight(0, climb, rx / mag * push, ry / mag * push))

    def _spin_step(self, state):
        if state.time_elapsed - self.spin_start > SPIN_SECONDS:
            self.phase = "done"
            return Command(flight=flight(0, 0, 0, 0))
        return Command(flight=flight(1.0, 0, 0, 0))   # left_x = yaw, full right turn
