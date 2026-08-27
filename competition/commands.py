"""The flight and camera commands an Agent returns from its callbacks.

Mirrors docs/competition-api.md section 2 for the subset this sandbox
implements: Velocity (body + world), Goto, Route, Hold. Inspect and the
perception helpers are out of scope -- see
docs/superpowers/specs/2026-08-27-website-agent-upload-design.md.

Every command is a frozen dataclass validated on construction. UNITS: metres,
m/s, degrees, deg/s. `up` is POSITIVE UP in both frames; the NED sign flip
happens once, server-side, in streaming/agent_control.py.
"""
from dataclasses import dataclass

_NUM = (int, float)


def _check_numbers(obj, names):
    for name in names:
        v = getattr(obj, name)
        if isinstance(v, bool) or not isinstance(v, _NUM):
            raise TypeError(
                f"{type(obj).__name__}.{name} must be a number, got {v!r}")


@dataclass(frozen=True)
class Velocity:
    """Body-frame velocity. `forward` tracks the nose. Stateless: re-issue it
    every tick to hold it (the server watchdog decays it to hover otherwise)."""
    forward: float = 0.0
    right: float = 0.0
    up: float = 0.0
    yaw_rate: float = 0.0

    def __post_init__(self):
        _check_numbers(self, ("forward", "right", "up", "yaw_rate"))


@dataclass(frozen=True)
class VelocityWorld:
    """World-frame velocity. north/east instead of forward/right; heading is
    irrelevant. Stateless, same as Velocity."""
    north: float = 0.0
    east: float = 0.0
    up: float = 0.0
    yaw_rate: float = 0.0

    def __post_init__(self):
        _check_numbers(self, ("north", "east", "up", "yaw_rate"))


@dataclass(frozen=True)
class Goto:
    """Fly to one point; altitude is metres above the launch point. Stateful:
    survives a silent script. Completion arrives as on_arrival."""
    lat: float
    lon: float
    alt: float
    speed: float | None = None

    def __post_init__(self):
        _check_numbers(self, ("lat", "lon", "alt"))
        if self.speed is not None and (isinstance(self.speed, bool)
                                       or not isinstance(self.speed, _NUM)
                                       or self.speed <= 0):
            raise ValueError(f"Goto speed must be a positive number, "
                             f"got {self.speed!r}")


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


@dataclass(frozen=True)
class Hold:
    """Station-keep at the current position. Stateful and safe."""


FLIGHT_TYPES = (Velocity, VelocityWorld, Goto, Route, Hold)
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
