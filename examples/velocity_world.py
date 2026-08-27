"""World-frame VelocityWorld: fly due north at 3 m/s for 8 s, then hold.

north/east are compass directions -- heading does not matter. Same stateless
re-issue-every-tick pattern as velocity.py.
"""
from competition import Agent, Command, Hold, VelocityWorld


class NorthThenHold(Agent):
    def on_tick(self, state):
        if state.time_elapsed < 8.0:
            return Command(flight=VelocityWorld(north=3.0))
        return Command(flight=Hold())
