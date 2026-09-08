"""The flight and camera commands an Agent returns from its callbacks.

Mirrors docs/competition-api.md section 2 for the subset this sandbox
implements: flight() and Route. Inspect and the perception helpers are out
of scope -- see docs/superpowers/specs/2026-08-27-website-agent-upload-design.md.
`flight()` replaced Velocity/VelocityWorld/Goto/Hold -- see
docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md.

Every command is a frozen dataclass validated on construction. UNITS: metres,
degrees. `flight()`'s four axes are normalized floats in [-1, 1]; the
translation to an actual m/s velocity happens once, server-side, in
streaming/agent_control.py.
"""
from dataclasses import dataclass

_NUM = (int, float)


def _check_numbers(obj, names):
    for name in names:
        v = getattr(obj, name)
        if isinstance(v, bool) or not isinstance(v, _NUM):
            raise TypeError(
                f"{type(obj).__name__}.{name} must be a number, got {v!r}")


def _check_unit_range(obj, names):
    for name in names:
        v = getattr(obj, name)
        if not -1.0 <= v <= 1.0:
            raise ValueError(
                f"{type(obj).__name__}.{name} must be in [-1, 1], got {v!r}")


@dataclass(frozen=True)
class flight:
    """The one flight primitive: two virtual joysticks, body-frame,
    normalized. Stateless: re-issue it every tick to hold it (the server
    watchdog decays it to hover otherwise), exactly like Velocity did.

        flight(left_x, left_y, right_x, right_y)   # each in [-1, 1]
            left_y  = thrust   (+1 = full climb)
            left_x  = yaw       (+1 = full right / clockwise)
            right_y = pitch      (+1 = full forward)
            right_x = roll        (+1 = full right / strafe)

    Full deflection on a translation axis = 5 m/s. See
    docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md
    Decisions 3-5 for the full rationale.
    """
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0

    def __post_init__(self):
        names = ("left_x", "left_y", "right_x", "right_y")
        _check_numbers(self, names)
        _check_unit_range(self, names)


@dataclass(frozen=True)
class Route:
    """Fly a list of (lat, lon) in order at one altitude. Stateful. Progress
    arrives as on_waypoint / on_route_complete."""
    waypoints: tuple
    alt: float
    speed: float | None = None

    def __post_init__(self):
        try:
            pts = tuple((float(a), float(b)) for a, b in self.waypoints)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Route waypoints must be (lat, lon) pairs: "
                             f"{exc}") from None
        object.__setattr__(self, "waypoints", pts)
        if not pts:
            raise ValueError("Route needs at least one waypoint")
        _check_numbers(self, ("alt",))
        if self.speed is not None and (isinstance(self.speed, bool)
                                       or not isinstance(self.speed, _NUM)
                                       or self.speed <= 0):
            raise ValueError(f"Route speed must be a positive number, "
                             f"got {self.speed!r}")


FLIGHT_TYPES = (flight, Route)
CAMERAS = ("nadir", "oblique")


@dataclass(frozen=True)
class Command:
    """What every callback returns (or None to keep doing what it was doing).
    Both fields optional: Command(camera="oblique") switches cameras without
    touching the flight path."""
    flight: object = None
    camera: str | None = None

    def __post_init__(self):
        if self.flight is not None and not isinstance(self.flight, FLIGHT_TYPES):
            raise TypeError(
                "Command.flight must be one of "
                f"{[t.__name__ for t in FLIGHT_TYPES]}, "
                f"got {type(self.flight).__name__}")
        if self.camera is not None and self.camera not in CAMERAS:
            raise ValueError(
                f"Command.camera must be one of {CAMERAS}, got {self.camera!r}")
        if self.flight is None and self.camera is None:
            raise ValueError("Command must set flight, camera, or both")
