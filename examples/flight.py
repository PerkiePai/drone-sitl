"""flight(): the drone's one flight primitive. Fly forward at full speed for
8 seconds, then hold in place.

flight() is stateless -- it decays to a hover if you stop sending it -- so
on_tick re-issues it every call. Re-issuing flight(0, 0, 0, 0) is what
"hold" means now: see
docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md
Decision 2 (Hold was a bare zero-command, not worth its own type).
"""
from competition import Agent, Command, flight


class ForwardThenHover(Agent):
    def on_tick(self, state):
        if state.time_elapsed < 8.0:
            return Command(flight=flight(0, 0, 0, 1.0))   # right_y = pitch forward
        return Command(flight=flight(0, 0, 0, 0))
