"""The base class every uploaded script subclasses.

The harness owns the loop and calls these; a script never writes `while True`.
Every callback is a no-op returning None ("keep doing what you were doing"),
so a script overrides only what it needs. Full contract:
docs/competition-api.md section 1.
"""


class Agent:
    def on_start(self, arena):
        """Once, before anything flies. Return the first Command, or None."""
        return None

    def on_frame(self, image, state):
        """~5 Hz. `image` is a numpy array from the selected camera, or None
        until the first frame arrives. Heavy perception goes here."""
        return None

    def on_tick(self, state):
        """~20 Hz. No image. Cheap steering only."""
        return None

    def on_arrival(self, state):
        """A single-waypoint Route finished."""
        return None

    def on_waypoint(self, index, state):
        """Route waypoint `index` (0-based) was reached."""
        return None

    def on_route_complete(self, state):
        """The last Route waypoint was reached."""
        return None
