"""Camera switching: hover, and flip nadir <-> oblique every 5 seconds,
logging the frame size each time on_frame delivers one.

Shows that Command(camera=...) changes what on_frame receives without
touching the flight path.
"""
from competition import Agent, Command, Hold


class CameraToggle(Agent):
    def on_start(self, arena):
        self.flipped_at = 0.0
        return Command(flight=Hold(), camera="nadir")

    def on_frame(self, image, state):
        shape = None if image is None else getattr(image, "shape", None)
        print(f"t={state.time_elapsed:5.1f}s camera={state.camera} frame={shape}")
        if state.time_elapsed - self.flipped_at >= 5.0:
            self.flipped_at = state.time_elapsed
            want = "oblique" if state.camera == "nadir" else "nadir"
            return Command(camera=want)
