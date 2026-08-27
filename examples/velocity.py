"""Body-frame Velocity: fly forward at 3 m/s for 8 seconds, then hold.

Velocity is stateless -- it decays to a hover if you stop sending it -- so
on_tick re-issues it every call. Upload this and press RUN.
"""
from competition import Agent, Command, Hold, Velocity


class ForwardThenHold(Agent):
    def on_tick(self, state):
        if state.time_elapsed < 8.0:
            return Command(flight=Velocity(forward=3.0))
        return Command(flight=Hold())
