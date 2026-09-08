"""Full sortie: everything flight() can do, in one uploadable script.

RUN SCRIPT takes off to 5 m, then this flies:

    climb to 50 m AGL -> nadir cam -> forward 50 m -> oblique cam -> spin
    360 in place -> 50 m due west (a world bearing rotated into flight()'s
    body frame) -> descend to the ground.

flight() is stateless, so every leg re-issues its setpoint on each on_tick.
Distance and yaw are integrated from `state` -- there is no "travelled"
field. The agent API has no land command; descending flight() until
alt_agl is ~0 puts it down, then the operator presses DISARM.
"""
import math

from competition import Agent, Command, flight

CLIMB_ALT = 50.0     # m AGL to reach before the forward leg
LEG_DIST = 50.0      # m for the forward and west legs
YAW_DEG_TOTAL = 360.0
CLIMB_THRUST = 0.6   # left_y -- well under full deflection (5 m/s), gentle
CRUISE_PITCH = 0.6   # right_y
YAW_STICK = 0.6      # left_x
DESCENT_THRUST = -0.4


def _metres(lat1, lon1, lat2, lon2):
    """Flat-earth distance in metres -- exact enough over tens of metres."""
    dlat = (lat2 - lat1) * 111_320.0
    dlon = (lon2 - lon1) * 111_320.0 * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


class FullSortie(Agent):
    def on_start(self, arena):
        self.phase = "climb"
        self.anchor = None       # (lat, lon) captured at the start of a leg
        self.last_heading = None
        self.yaw_turned = 0.0
        return Command(flight=flight(0, CLIMB_THRUST, 0, 0), camera="oblique")

    def on_tick(self, state):
        return getattr(self, f"_{self.phase}")(state)

    # -- phases -----------------------------------------------------------

    def _climb(self, state):
        if state.alt_agl < CLIMB_ALT:
            return Command(flight=flight(0, CLIMB_THRUST, 0, 0))
        print(f"reached {state.alt_agl:.1f} m AGL -- forward leg")
        self.anchor = (state.lat, state.lon)
        self.phase = "forward"
        return Command(flight=flight(0, 0, 0, CRUISE_PITCH), camera="nadir")

    def _forward(self, state):
        if state.lat is None:
            return None
        if _metres(*self.anchor, state.lat, state.lon) < LEG_DIST:
            return Command(flight=flight(0, 0, 0, CRUISE_PITCH))
        print("forward 50 m done -- spinning 360")
        self.last_heading = state.heading
        self.yaw_turned = 0.0
        self.phase = "spin"
        return Command(flight=flight(YAW_STICK, 0, 0, 0), camera="oblique")

    def _spin(self, state):
        step = (state.heading - self.last_heading + 180.0) % 360.0 - 180.0
        self.yaw_turned += max(step, 0.0)
        self.last_heading = state.heading
        if self.yaw_turned < YAW_DEG_TOTAL:
            return Command(flight=flight(YAW_STICK, 0, 0, 0))
        print("360 done -- heading west")
        self.anchor = (state.lat, state.lon)
        self.phase = "west"
        return self._west(state)

    def _west(self, state):
        if state.lat is None:
            return None
        if _metres(*self.anchor, state.lat, state.lon) < LEG_DIST:
            # World "west" rotated into flight()'s body frame -- Decision 5
            # of the flight() spec, spelled out here rather than imported.
            h = math.radians(state.heading)
            north, east = 0.0, -CRUISE_PITCH
            rx = north * math.sin(h) + east * math.cos(h)
            ry = north * math.cos(h) - east * math.sin(h)
            return Command(flight=flight(0, 0, rx, ry))
        print("west 50 m done -- descending")
        self.phase = "landing"
        return Command(flight=flight(0, DESCENT_THRUST, 0, 0))

    def _landing(self, state):
        if state.alt_agl is not None and state.alt_agl > 0.5:
            return Command(flight=flight(0, DESCENT_THRUST, 0, 0))
        print("on the ground -- press DISARM")
        return Command(flight=flight(0, 0, 0, 0))
