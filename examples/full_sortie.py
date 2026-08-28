"""Full sortie: every flight + camera primitive in one uploadable script.

RUN SCRIPT takes off to 5 m, then this flies:

    oblique cam -> climb to 50 m AGL -> nadir cam -> 50 m forward (body
    Velocity) -> oblique cam -> yaw 360 in place -> 50 m due west
    (VelocityWorld) -> Goto(alt=0) to put it on the ground.

Velocity / VelocityWorld are stateless, so every leg re-issues its setpoint
on each on_tick. Distance and yaw are integrated from `state` -- there is no
"travelled" field. The agent API has no land command; a Goto to alt 0 lands
it, then the operator presses DISARM.
"""
import math

from competition import Agent, Command, Goto, Hold, Velocity, VelocityWorld

CLIMB_ALT = 50.0     # m AGL to reach before the forward leg
LEG_DIST = 50.0      # m for the forward and west legs
CLIMB_RATE = 3.0     # m/s
CRUISE = 3.0         # m/s (server clamps MPC_XY_VEL_MAX to 3)
YAW_RATE = 30.0      # deg/s, clockwise seen from above


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
        return Command(flight=VelocityWorld(up=CLIMB_RATE), camera="oblique")

    def on_tick(self, state):
        return getattr(self, f"_{self.phase}")(state)

    # -- phases -----------------------------------------------------------

    def _climb(self, state):
        if state.alt_agl < CLIMB_ALT:
            return Command(flight=VelocityWorld(up=CLIMB_RATE))
        print(f"reached {state.alt_agl:.1f} m AGL -- forward leg")
        self.anchor = (state.lat, state.lon)
        self.phase = "forward"
        return Command(flight=Velocity(forward=CRUISE), camera="nadir")

    def _forward(self, state):
        if state.lat is None:
            return None
        if _metres(*self.anchor, state.lat, state.lon) < LEG_DIST:
            return Command(flight=Velocity(forward=CRUISE))
        print("forward 50 m done -- spinning 360")
        self.last_heading = state.heading
        self.yaw_turned = 0.0
        self.phase = "spin"
        return Command(flight=Velocity(yaw_rate=YAW_RATE), camera="oblique")

    def _spin(self, state):
        step = (state.heading - self.last_heading + 180.0) % 360.0 - 180.0
        self.yaw_turned += max(step, 0.0)
        self.last_heading = state.heading
        if self.yaw_turned < 360.0:
            return Command(flight=Velocity(yaw_rate=YAW_RATE))
        print("360 done -- heading west")
        self.anchor = (state.lat, state.lon)
        self.phase = "west"
        return Command(flight=VelocityWorld(east=-CRUISE))

    def _west(self, state):
        if state.lat is None:
            return None
        if _metres(*self.anchor, state.lat, state.lon) < LEG_DIST:
            return Command(flight=VelocityWorld(east=-CRUISE))
        print("west 50 m done -- landing via Goto(alt=0)")
        self.phase = "landing"
        return Command(flight=Goto(state.lat, state.lon, alt=0.0))

    def _landing(self, state):
        return None

    def on_arrival(self, state):
        print("on the ground -- press DISARM")
        return Command(flight=Hold())
