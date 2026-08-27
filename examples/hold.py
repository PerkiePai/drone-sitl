"""Hold: the simplest possible agent. Take off, then just sit there.

Useful as a baseline -- if the drone drifts while running this, the problem
is upstream of any script.
"""
from competition import Agent, Command, Hold


class JustHover(Agent):
    def on_start(self, arena):
        return Command(flight=Hold())
