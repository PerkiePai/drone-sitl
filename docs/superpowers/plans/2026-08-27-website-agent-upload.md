# Website Agent Upload — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an Agent panel to the joystick website that uploads a competition-API control script, flies it in a child process, and returns control to the operator on the first manual input.

**Architecture:** A new `competition/` package is what uploaded scripts import (`Agent`, `Command`, `Velocity`, `VelocityWorld`, `Goto`, `Route`, `Hold`). A new `agent_runner.py` child process loads the script, runs the callback loop (`competition/harness.py`), pulls camera frames from the Isaac MJPEG server, and sends `Command`s to `joystick-server.py` over a `/agent/control` WebSocket. The server applies them through a new `AgentControl` object and its existing `SetpointLoop`, which gains a third setpoint source: mission ▸ agent ▸ manual.

**Tech Stack:** Python 3.12, `pymavlink`, `fastapi` + `uvicorn` + `websockets` (already in the `drone` conda env), `numpy` + `Pillow` for frame decode (already present), vanilla ES modules on the web side, `pytest`.

**Spec:** `docs/superpowers/specs/2026-08-27-website-agent-upload-design.md` — read it alongside this plan.

## Global Constraints

- **No new flags on `joystick-server.py`.** Running it stays `conda run -n drone python joystick-server.py` with nothing appended. New config is module constants: `AGENT_UPLOAD_DIR = <repo>/logs/agents/`, `ARENA_RADIUS_M = 500.0`, `AGENT_TIME_LIMIT_S = None`.
- **No new Python dependencies.** In particular no `python-multipart`: the upload endpoint takes a raw body, not a multipart form.
- **The child process never imports `pymavlink` and never opens a MAVLink socket.** Its only outward effect is JSON on the `/agent/control` WebSocket.
- **`SetpointLoop` stays the sole owner of the MAVLink connection.** All agent flight goes through `AgentControl` / the mission, read by `SetpointLoop` on its own thread.
- **`up` is positive-up everywhere in `competition/` and every example script.** The NED sign flip happens once, in `streaming/agent_control.py` (and `OffboardLink.send_velocity_world`).
- **Units:** metres, m/s, degrees, deg/s. Heading 0 = north, positive clockwise.
- **Test isolation:** tests that bind sockets must not use ports 8090 / 14540 / 14580 (real SITL) — follow the existing `test_web_ui.py` / `test_offboard_loop.py` port choices.
- **Conda env:** run every test with `conda run -n drone python -m pytest ...`.
- Commit messages end with `Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>`.

---

## File Structure

**New:**

| File | Responsibility |
|---|---|
| `competition/__init__.py` | Public re-exports: `Agent`, `Command`, `Velocity`, `VelocityWorld`, `Goto`, `Route`, `Hold` |
| `competition/commands.py` | The frozen command dataclasses + validation |
| `competition/agent.py` | `Agent` base class — all callbacks no-op returning `None` |
| `competition/state.py` | `State`, `RouteInfo`, `Arena` value objects + `from_telemetry` / `around` builders |
| `competition/harness.py` | `Harness` — owns the loop, dispatches callbacks with per-call timeouts, synthesizes arrival events, translates `Command`→channel messages |
| `competition/frames.py` | `MjpegFrames` — one background MJPEG connection, decodes the latest JPEG to a numpy array, `select(camera)` switches endpoints |
| `competition/tests/__init__.py` | empty — makes the test dir a package |
| `competition/tests/test_commands.py` | command construction + validation |
| `competition/tests/test_state.py` | `State`/`Arena` from a telemetry dict |
| `competition/tests/test_harness.py` | loop cadence, timeout-abandons-call, event synthesis, command translation, camera switching |
| `competition/tests/test_frames.py` | JPEG decode + endpoint switching against a local fake MJPEG server |
| `agent_runner.py` | Child-process entrypoint: load script, find the `Agent` subclass, wire `Harness` to a real WebSocket + `MjpegFrames`, forward stdout/stderr |
| `streaming/agent_control.py` | `AgentControl` — thread-safe agent setpoint holder, +up→NED on write, 0.5 s velocity watchdog, stateful `Hold` |
| `streaming/tests/test_agent_control.py` | `AgentControl` behaviour |
| `web/js/agent.js` | Agent panel: upload, run/stop, log pane, camera-swap on telemetry |
| `examples/velocity.py` … `examples/camera.py` | Six uploadable example agents, one per primitive |

**Modified:**

| File | Change |
|---|---|
| `streaming/offboard.py` | `OffboardLink.send_velocity_world`; parse `ATTITUDE` (roll/pitch) and `GLOBAL_POSITION_INT` world velocity into telemetry |
| `streaming/tests/test_offboard.py` | tests for the two `offboard.py` additions |
| `joystick-server.py` | telemetry `roll_deg`/`pitch_deg`/`vn`/`ve`/`agent`; `AgentControl` wiring; `SetpointLoop` source priority; `POST /agent/upload`, `GET /agent/list`; `/agent/control` WebSocket; `/ws` `agent` run/stop; the ARM→TAKEOFF→OFFBOARD run state machine + child spawn/kill |
| `streaming/tests/test_offboard_loop.py` | source-priority tests |
| `streaming/tests/test_web_ui.py` | upload/list/run-refused tests |
| `web/index.html` | the `#agent` panel markup |
| `web/js/main.js` | import + wire `agent.js` |
| `web/css/app.css` | `#agent` panel styles |
| `RUN-WEBSITE.md` | new "Fly an uploaded script" section + manual test list |
| `SESSION.md` | how to run the feature |

---

## Task 1: `competition/` command + agent surface

**Files:**
- Create: `competition/__init__.py`
- Create: `competition/commands.py`
- Create: `competition/agent.py`
- Create: `competition/tests/__init__.py`
- Test: `competition/tests/test_commands.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `competition.commands.Velocity(forward=0.0, right=0.0, up=0.0, yaw_rate=0.0)` — frozen dataclass
  - `competition.commands.VelocityWorld(north=0.0, east=0.0, up=0.0, yaw_rate=0.0)` — frozen dataclass
  - `competition.commands.Goto(lat, lon, alt, speed=None)` — frozen dataclass
  - `competition.commands.Route(waypoints, alt, speed=None)` — frozen; `waypoints` normalized to `tuple[tuple[float, float], ...]`
  - `competition.commands.Hold()` — frozen, no fields
  - `competition.commands.Command(flight=None, camera=None)` — frozen; `camera ∈ {"nadir", "oblique"}`
  - `competition.commands.FLIGHT_TYPES` = `(Velocity, VelocityWorld, Goto, Route, Hold)`
  - `competition.commands.CAMERAS` = `("nadir", "oblique")`
  - `competition.agent.Agent` — base class with `on_start(self, arena)`, `on_frame(self, image, state)`, `on_tick(self, state)`, `on_arrival(self, state)`, `on_waypoint(self, index, state)`, `on_route_complete(self, state)`, each returning `None`
  - `from competition import Agent, Command, Velocity, VelocityWorld, Goto, Route, Hold`

- [ ] **Step 1: Write the failing test**

`competition/tests/test_commands.py`:

```python
"""competition.commands -- construction and validation.

A malformed command is a bug in the uploaded script; it must raise where the
script writes it, not fly. No PX4, no server.
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition import (  # noqa: E402
    Agent, Command, Velocity, VelocityWorld, Goto, Route, Hold,
)


def test_velocity_defaults_to_all_zero_hover():
    v = Velocity()
    assert (v.forward, v.right, v.up, v.yaw_rate) == (0.0, 0.0, 0.0, 0.0)


def test_velocity_is_frozen():
    v = Velocity(forward=1.0)
    with pytest.raises(Exception):
        v.forward = 2.0


def test_velocity_rejects_non_numeric():
    with pytest.raises(TypeError):
        Velocity(forward="fast")


def test_velocity_world_has_north_east_not_forward_right():
    v = VelocityWorld(north=2.0, east=-1.0)
    assert v.north == 2.0 and v.east == -1.0
    assert not hasattr(v, "forward")


def test_goto_speed_must_be_positive_when_given():
    Goto(1.0, 2.0, 30.0)              # None speed is fine
    Goto(1.0, 2.0, 30.0, speed=5.0)
    with pytest.raises(ValueError):
        Goto(1.0, 2.0, 30.0, speed=0.0)


def test_route_needs_at_least_one_waypoint():
    with pytest.raises(ValueError):
        Route(waypoints=[], alt=30.0)


def test_route_normalizes_waypoints_to_tuple_of_float_pairs():
    r = Route(waypoints=[[1, 2], (3, 4)], alt=30.0)
    assert r.waypoints == ((1.0, 2.0), (3.0, 4.0))


def test_hold_takes_no_fields():
    Hold()


def test_command_accepts_a_flight_command():
    c = Command(flight=Velocity(forward=3.0))
    assert isinstance(c.flight, Velocity)
    assert c.camera is None


def test_command_accepts_camera_only():
    c = Command(camera="oblique")
    assert c.flight is None and c.camera == "oblique"


def test_command_rejects_unknown_camera():
    with pytest.raises(ValueError):
        Command(camera="telephoto")


def test_command_rejects_a_non_command_flight():
    with pytest.raises(TypeError):
        Command(flight=(0, 0, 0))


def test_command_must_set_something():
    with pytest.raises(ValueError):
        Command()


def test_agent_callbacks_all_default_to_none():
    a = Agent()
    assert a.on_start(None) is None
    assert a.on_frame(None, None) is None
    assert a.on_tick(None) is None
    assert a.on_arrival(None) is None
    assert a.on_waypoint(0, None) is None
    assert a.on_route_complete(None) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `conda run -n drone python -m pytest competition/tests/test_commands.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'competition'`

- [ ] **Step 3: Write `competition/commands.py`**

```python
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
```

- [ ] **Step 4: Write `competition/agent.py`**

```python
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
        """A Goto finished."""
        return None

    def on_waypoint(self, index, state):
        """Route waypoint `index` (0-based) was reached."""
        return None

    def on_route_complete(self, state):
        """The last Route waypoint was reached."""
        return None
```

- [ ] **Step 5: Write `competition/__init__.py` and `competition/tests/__init__.py`**

`competition/__init__.py`:

```python
"""Competitor-facing drone control API (sandbox subset).

    from competition import Agent, Command, Velocity, Route, Goto, Hold

    class MyAgent(Agent):
        def on_start(self, arena):
            return Command(flight=Velocity(forward=3.0))

Full surface: docs/competition-api.md. What this sandbox implements and what it
leaves out: docs/superpowers/specs/2026-08-27-website-agent-upload-design.md.
"""
from competition.agent import Agent
from competition.commands import (
    Command, Goto, Hold, Route, Velocity, VelocityWorld,
)

__all__ = [
    "Agent", "Command", "Velocity", "VelocityWorld", "Goto", "Route", "Hold",
]
```

`competition/tests/__init__.py`: empty file.

- [ ] **Step 6: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest competition/tests/test_commands.py -v`
Expected: PASS (14 tests)

- [ ] **Step 7: Commit**

```bash
git add competition/__init__.py competition/commands.py competition/agent.py \
        competition/tests/__init__.py competition/tests/test_commands.py
git commit -m "feat(competition): Agent base class and flight/camera commands

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 2: `competition/state.py` — `State` and `Arena`

**Files:**
- Create: `competition/state.py`
- Test: `competition/tests/test_state.py`

**Interfaces:**
- Consumes: nothing from earlier tasks (pure value objects).
- Produces:
  - `competition.state.RouteInfo(state, index, count, distance_m)` — frozen
  - `competition.state.State` — frozen, fields: `lat, lon, alt_agl, vx, vy, vz, ground_speed, heading, roll, pitch, camera, time_elapsed, time_remaining, route`
  - `State.from_telemetry(telem: dict, *, camera: str, time_elapsed: float, time_limit: float | None) -> State`
  - `competition.state.Arena(bounds, time_limit)` — frozen; `bounds = (lat_min, lon_min, lat_max, lon_max)`
  - `Arena.around(lat: float, lon: float, radius_m: float, time_limit: float | None) -> Arena`
- Telemetry keys read: `lat`, `lon`, `alt_m`, `vn`, `ve`, `vz`, `gs`, `heading_deg`, `roll_deg`, `pitch_deg`, `mission` (`{state, index, count, dist_m}`). Missing keys default to `0.0` / `None` / `"IDLE"`.

- [ ] **Step 1: Write the failing test**

`competition/tests/test_state.py`:

```python
"""competition.state -- State/Arena value objects built from the server's
telemetry dict. No PX4, no server."""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition.state import Arena, State  # noqa: E402

TELEM = {
    "lat": 13.6, "lon": 100.3, "alt_m": 42.0,
    "vn": 1.5, "ve": -0.5, "vz": 0.2, "gs": 1.58,
    "heading_deg": 270.0, "roll_deg": 3.0, "pitch_deg": -12.0,
    "mission": {"state": "RUNNING", "index": 2, "count": 4, "dist_m": 37.0},
}


def test_state_maps_every_field():
    s = State.from_telemetry(TELEM, camera="nadir", time_elapsed=10.0,
                             time_limit=None)
    assert s.lat == 13.6 and s.lon == 100.3
    assert s.alt_agl == 42.0
    assert (s.vx, s.vy, s.vz) == (1.5, -0.5, 0.2)
    assert s.ground_speed == 1.58
    assert s.heading == 270.0
    assert (s.roll, s.pitch) == (3.0, -12.0)
    assert s.camera == "nadir"
    assert s.time_elapsed == 10.0
    assert s.time_remaining is None
    assert s.route.state == "RUNNING" and s.route.index == 2
    assert s.route.count == 4 and s.route.distance_m == 37.0


def test_time_remaining_is_clamped_and_computed_when_limited():
    s = State.from_telemetry(TELEM, camera="nadir", time_elapsed=250.0,
                             time_limit=300.0)
    assert s.time_remaining == 50.0
    s2 = State.from_telemetry(TELEM, camera="nadir", time_elapsed=999.0,
                              time_limit=300.0)
    assert s2.time_remaining == 0.0


def test_missing_keys_default_rather_than_raise():
    s = State.from_telemetry({}, camera="oblique", time_elapsed=0.0,
                             time_limit=None)
    assert s.lat is None and s.lon is None
    assert s.alt_agl == 0.0 and s.vz == 0.0
    assert s.route.state == "IDLE" and s.route.count == 0
    assert s.route.distance_m is None


def test_arena_around_is_a_box_centred_on_the_point():
    a = Arena.around(13.0, 100.0, radius_m=500.0, time_limit=None)
    lat_min, lon_min, lat_max, lon_max = a.bounds
    assert lat_min < 13.0 < lat_max
    assert lon_min < 100.0 < lon_max
    # ~500 m north/south is ~0.00449 deg latitude
    assert math.isclose((lat_max - lat_min) / 2, 500.0 / 111_320.0, rel_tol=1e-3)
    assert a.time_limit is None


def test_arena_longitude_span_widens_toward_the_equator():
    near_eq = Arena.around(1.0, 0.0, 500.0, None).bounds
    high_lat = Arena.around(60.0, 0.0, 500.0, None).bounds
    assert (near_eq[3] - near_eq[1]) < (high_lat[3] - high_lat[1])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `conda run -n drone python -m pytest competition/tests/test_state.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'competition.state'`

- [ ] **Step 3: Write `competition/state.py`**

```python
"""Value objects handed to every Agent callback.

State is a read-only snapshot of the aircraft, rebuilt each tick from the
server's telemetry dict. Arena is fixed for the run.

UNITS match docs/competition-api.md section 3.3: metres, m/s, degrees.
Velocity is +up. Heading 0 = north, positive clockwise.
"""
import math
from dataclasses import dataclass

_DEG_LAT_M = 111_320.0


@dataclass(frozen=True)
class RouteInfo:
    state: str            # "IDLE" | "RUNNING" | "PAUSED" | "DONE"
    index: int            # 0-based waypoint currently targeted
    count: int
    distance_m: float | None


@dataclass(frozen=True)
class State:
    lat: float | None
    lon: float | None
    alt_agl: float
    vx: float             # world north, m/s, +N
    vy: float             # world east,  m/s, +E
    vz: float             # +up, m/s
    ground_speed: float
    heading: float        # deg, 0 = N, +CW
    roll: float           # deg
    pitch: float          # deg
    camera: str
    time_elapsed: float   # s since on_start
    time_remaining: float | None
    route: RouteInfo

    @classmethod
    def from_telemetry(cls, telem, *, camera, time_elapsed, time_limit):
        m = telem.get("mission") or {}
        return cls(
            lat=telem.get("lat"),
            lon=telem.get("lon"),
            alt_agl=float(telem.get("alt_m", 0.0)),
            vx=float(telem.get("vn", 0.0)),
            vy=float(telem.get("ve", 0.0)),
            vz=float(telem.get("vz", 0.0)),
            ground_speed=float(telem.get("gs", 0.0)),
            heading=float(telem.get("heading_deg", 0.0)),
            roll=float(telem.get("roll_deg", 0.0)),
            pitch=float(telem.get("pitch_deg", 0.0)),
            camera=camera,
            time_elapsed=float(time_elapsed),
            time_remaining=(None if time_limit is None
                            else max(0.0, float(time_limit) - float(time_elapsed))),
            route=RouteInfo(
                state=m.get("state", "IDLE"),
                index=int(m.get("index", 0)),
                count=int(m.get("count", 0)),
                distance_m=m.get("dist_m"),
            ),
        )


@dataclass(frozen=True)
class Arena:
    bounds: tuple         # (lat_min, lon_min, lat_max, lon_max)
    time_limit: float | None

    @classmethod
    def around(cls, lat, lon, radius_m, time_limit):
        dlat = radius_m / _DEG_LAT_M
        dlon = radius_m / (_DEG_LAT_M * max(math.cos(math.radians(lat)), 1e-6))
        return cls(bounds=(lat - dlat, lon - dlon, lat + dlat, lon + dlon),
                   time_limit=time_limit)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest competition/tests/test_state.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Commit**

```bash
git add competition/state.py competition/tests/test_state.py
git commit -m "feat(competition): State and Arena value objects for callbacks

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 3: `competition/harness.py` — the callback loop

**Files:**
- Create: `competition/harness.py`
- Test: `competition/tests/test_harness.py`

**Interfaces:**
- Consumes:
  - `competition.commands` — `Command`, `Velocity`, `VelocityWorld`, `Goto`, `Route`, `Hold`
  - `competition.state` — `State`, `Arena`
- Produces:
  - `competition.harness.Harness(agent, channel, frames, arena, *, now=time.monotonic, sleep=time.sleep, log=print)`
  - `Harness.run()` — blocks until `stop()` or the arena time limit; drives the loop
  - `Harness.stop()` — sets an `Event`; `run()` returns after the current tick
  - Constants: `TICK_HZ = 20.0`, `FRAME_HZ = 5.0`, `BUDGET_FRAME_S = 0.200`, `BUDGET_TICK_S = 0.050`, `BUDGET_EVENT_S = 0.200`
  - `channel` contract (duck-typed): `channel.send(msg: dict) -> None`, `channel.telemetry() -> dict`
  - `frames` contract: `frames.latest() -> "np.ndarray | None"`, `frames.select(camera: str) -> None`
  - Channel message shapes emitted: `{"type":"velocity","forward","right","up","yaw_rate"}`, `{"type":"velocity_world","north","east","up","yaw_rate"}`, `{"type":"goto","lat","lon","alt","speed"}`, `{"type":"route","points":[[lat,lon],...],"alt","speed"}`, `{"type":"hold"}`, `{"type":"camera","camera"}`

- [ ] **Step 1: Write the failing test**

`competition/tests/test_harness.py`:

```python
"""competition.harness -- the loop the harness owns so the script doesn't.

Driven by a virtual clock so cadence assertions are exact. Callbacks that
overrun are tested with real threads and a small real budget.
"""
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition import Agent, Command, Goto, Hold, Route, Velocity, VelocityWorld  # noqa: E402
from competition.harness import Harness  # noqa: E402
from competition.state import Arena  # noqa: E402


class FakeChannel:
    def __init__(self, telem=None):
        self.sent = []
        self._telem = telem or {"mission": {"state": "IDLE", "index": 0,
                                            "count": 0, "dist_m": None}}
        self.lock = threading.Lock()

    def send(self, msg):
        with self.lock:
            self.sent.append(msg)

    def telemetry(self):
        with self.lock:
            return dict(self._telem)

    def set_telemetry(self, telem):
        with self.lock:
            self._telem = telem

    def types(self):
        with self.lock:
            return [m["type"] for m in self.sent]


class FakeFrames:
    def __init__(self):
        self.selected = "nadir"
        self.image = "FRAME"

    def latest(self):
        return self.image

    def select(self, camera):
        self.selected = camera


class VirtualClock:
    """now()/sleep() that advance a shared counter -- no wall time passes."""
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 0.0)


def _run_for(harness, clock, seconds, step=0.001):
    """Spin the harness on its own thread, advancing the virtual clock until
    `seconds` of virtual time have elapsed, then stop it."""
    th = threading.Thread(target=harness.run, daemon=True)
    th.start()
    # The harness sleeps via clock.sleep, which advances clock.t. Give it wall
    # time to make progress while we watch the virtual clock.
    deadline = time.monotonic() + 5.0
    while clock.t < seconds and time.monotonic() < deadline:
        time.sleep(0.005)
    harness.stop()
    th.join(timeout=2.0)
    assert not th.is_alive(), "harness.run did not return after stop()"


def test_on_start_result_is_sent_first():
    class A(Agent):
        def on_start(self, arena):
            return Command(flight=Velocity(forward=5.0))

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.01)
    assert ch.sent[0] == {"type": "velocity", "forward": 5.0, "right": 0.0,
                          "up": 0.0, "yaw_rate": 0.0}


def test_on_tick_runs_at_20hz_and_on_frame_at_5hz():
    counts = {"tick": 0, "frame": 0}

    class A(Agent):
        def on_tick(self, state):
            counts["tick"] += 1

        def on_frame(self, image, state):
            counts["frame"] += 1
            assert image == "FRAME"

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 1.0)
    # ~5 frame ticks and ~15 plain ticks in a second; allow slack for the
    # exact boundary tick.
    assert 4 <= counts["frame"] <= 6, counts
    assert 13 <= counts["tick"] <= 17, counts
    assert counts["frame"] + counts["tick"] >= 19


def test_a_callback_that_overruns_its_budget_is_abandoned():
    """Real threads, small real budget. The slow on_tick must not block the
    loop, and its (late) return value must never reach the channel."""
    class A(Agent):
        def __init__(self):
            self.calls = 0

        def on_start(self, arena):
            return Command(flight=Velocity(forward=1.0))

        def on_tick(self, state):
            self.calls += 1
            time.sleep(0.5)                       # >> BUDGET_TICK_S
            return Command(flight=Velocity(forward=99.0))  # must be dropped

    ch = FakeChannel()
    agent = A()
    h = Harness(agent, ch, FakeFrames(), Arena.around(0, 0, 500, None),
                log=lambda *a: None)             # real clock here
    th = threading.Thread(target=h.run, daemon=True)
    th.start()
    time.sleep(0.4)
    h.stop()
    th.join(timeout=2.0)
    forwards = [m["forward"] for m in ch.sent if m["type"] == "velocity"]
    assert forwards, "nothing sent"
    assert all(f == 1.0 for f in forwards), f"a 99.0 leaked through: {forwards}"


def test_none_return_sends_nothing():
    class A(Agent):
        def on_tick(self, state):
            return None

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.5)
    assert ch.sent == []


def test_camera_switch_emits_a_camera_message_and_selects_the_source():
    class A(Agent):
        def on_start(self, arena):
            return Command(camera="oblique")

    ch, clock, frames = FakeChannel(), VirtualClock(), FakeFrames()
    h = Harness(A(), ch, frames, Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.01)
    assert {"type": "camera", "camera": "oblique"} in ch.sent
    assert frames.selected == "oblique"


def test_repeated_same_camera_is_not_re_emitted():
    class A(Agent):
        def on_tick(self, state):
            return Command(camera="nadir")       # already the default

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.5)
    assert "camera" not in ch.types()


def test_route_waypoint_and_completion_events_fire_from_mission_telemetry():
    seen = []

    class A(Agent):
        def on_start(self, arena):
            return Command(flight=Route(waypoints=[(0, 0), (1, 1), (2, 2)],
                                        alt=30.0))

        def on_waypoint(self, index, state):
            seen.append(("wp", index))

        def on_route_complete(self, state):
            seen.append(("done", None))

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    th = threading.Thread(target=h.run, daemon=True)
    th.start()
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "RUNNING", "index": 0, "count": 3}})
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "RUNNING", "index": 2, "count": 3}})
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "DONE", "index": 2, "count": 3}})
    time.sleep(0.05)
    h.stop()
    th.join(timeout=2.0)
    assert ("wp", 0) in seen and ("wp", 1) in seen
    assert ("done", None) in seen


def test_single_point_goto_completion_is_on_arrival_not_route_complete():
    seen = []

    class A(Agent):
        def on_start(self, arena):
            return Command(flight=Goto(1.0, 2.0, 30.0))

        def on_arrival(self, state):
            seen.append("arrived")

        def on_route_complete(self, state):
            seen.append("route_complete")

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    th = threading.Thread(target=h.run, daemon=True)
    th.start()
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "RUNNING", "index": 0, "count": 1}})
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "DONE", "index": 0, "count": 1}})
    time.sleep(0.05)
    h.stop()
    th.join(timeout=2.0)
    assert seen == ["arrived"]


def test_velocity_world_up_is_passed_through_positive():
    """The NED flip is server-side. The harness must send +up verbatim."""
    class A(Agent):
        def on_start(self, arena):
            return Command(flight=VelocityWorld(north=2.0, up=1.0))

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.01)
    msg = next(m for m in ch.sent if m["type"] == "velocity_world")
    assert msg["north"] == 2.0 and msg["up"] == 1.0


def test_time_limit_ends_the_run_with_a_hold():
    class A(Agent):
        def on_tick(self, state):
            return Command(flight=Velocity(forward=1.0))

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, 0.3),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    th = threading.Thread(target=h.run, daemon=True)
    th.start()
    deadline = time.monotonic() + 5.0
    while th.is_alive() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not th.is_alive(), "run() did not return at the time limit"
    assert ch.sent[-1] == {"type": "hold"}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `conda run -n drone python -m pytest competition/tests/test_harness.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'competition.harness'`

- [ ] **Step 3: Write `competition/harness.py`**

```python
"""The loop the harness owns so the competitor doesn't.

Given an Agent, a control channel and a frame source, this calls the Agent's
callbacks at the right rates, turns each returned Command into a channel
message, and synthesizes arrival events from mission telemetry.

A callback that overruns its budget is abandoned -- run on a worker thread,
joined with a timeout; the previous command stays in force. That is the
contract in docs/competition-api.md section 1. A genuinely wedged callback
leaves a daemon thread parked forever; Stop (killing the process) is the
backstop for that.
"""
import threading
import time

from competition.commands import (
    Command, Goto, Hold, Route, Velocity, VelocityWorld,
)
from competition.state import State

TICK_HZ = 20.0
FRAME_HZ = 5.0
BUDGET_FRAME_S = 0.200
BUDGET_TICK_S = 0.050
BUDGET_EVENT_S = 0.200
_OVERRUN_LOG_EVERY_S = 5.0


def _call_with_timeout(fn, budget_s, *args):
    """(status, payload): ("ok", result) | ("overrun", None) | ("error", exc)."""
    box = {}

    def target():
        try:
            box["result"] = fn(*args)
        except BaseException as exc:          # reported upward, not swallowed
            box["error"] = exc

    t = threading.Thread(target=target, daemon=True)
    t.start()
    t.join(budget_s)
    if t.is_alive():
        return "overrun", None
    if "error" in box:
        return "error", box["error"]
    return "ok", box.get("result")


class Harness:
    def __init__(self, agent, channel, frames, arena, *,
                 now=time.monotonic, sleep=time.sleep, log=print):
        self.agent = agent
        self.channel = channel          # .send(dict); .telemetry() -> dict
        self.frames = frames            # .latest() -> ndarray|None; .select(name)
        self.arena = arena
        self._now = now
        self._sleep = sleep
        self._log = log
        self._camera = "nadir"
        self._started_at = None
        self._last_frame_at = None
        self._prev_mission = {"state": "IDLE", "index": 0, "count": 0}
        self._last_overrun_log = -1e9
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    # -- command translation ------------------------------------------------

    def _emit(self, cmd):
        if cmd is None:
            return
        if not isinstance(cmd, Command):
            self._log(f"[harness] callback returned {type(cmd).__name__}, "
                      f"expected Command or None -- ignored")
            return
        if cmd.camera is not None and cmd.camera != self._camera:
            self._camera = cmd.camera
            self.frames.select(cmd.camera)
            self.channel.send({"type": "camera", "camera": cmd.camera})
        f = cmd.flight
        if isinstance(f, Velocity):
            self.channel.send({"type": "velocity", "forward": f.forward,
                               "right": f.right, "up": f.up,
                               "yaw_rate": f.yaw_rate})
        elif isinstance(f, VelocityWorld):
            self.channel.send({"type": "velocity_world", "north": f.north,
                               "east": f.east, "up": f.up,
                               "yaw_rate": f.yaw_rate})
        elif isinstance(f, Goto):
            self.channel.send({"type": "goto", "lat": f.lat, "lon": f.lon,
                               "alt": f.alt, "speed": f.speed})
        elif isinstance(f, Route):
            self.channel.send({"type": "route",
                               "points": [[a, b] for a, b in f.waypoints],
                               "alt": f.alt, "speed": f.speed})
        elif isinstance(f, Hold):
            self.channel.send({"type": "hold"})

    # -- dispatch ----------------------------------------------------------

    def _dispatch(self, fn, budget, *args):
        status, payload = _call_with_timeout(fn, budget, *args)
        if status == "overrun":
            now = self._now()
            if now - self._last_overrun_log > _OVERRUN_LOG_EVERY_S:
                self._log(f"[harness] {fn.__name__} overran "
                          f"{budget * 1000:.0f} ms -- call abandoned, "
                          f"previous command holds")
                self._last_overrun_log = now
            return None
        if status == "error":
            raise payload
        return payload

    def _state(self):
        return State.from_telemetry(
            self.channel.telemetry(), camera=self._camera,
            time_elapsed=self._now() - self._started_at,
            time_limit=self.arena.time_limit)

    # -- events ----------------------------------------------------------

    def _fire_events(self, state):
        m = self.channel.telemetry().get("mission") or {}
        prev = self._prev_mission
        cur = {"state": m.get("state", "IDLE"),
               "index": int(m.get("index", 0)),
               "count": int(m.get("count", 0))}
        if cur["count"] > 1 and cur["index"] > prev["index"]:
            for i in range(prev["index"], cur["index"]):
                self._emit(self._dispatch(self.agent.on_waypoint,
                                          BUDGET_EVENT_S, i, state))
        if prev["state"] == "RUNNING" and cur["state"] == "DONE":
            if cur["count"] <= 1:
                self._emit(self._dispatch(self.agent.on_arrival,
                                          BUDGET_EVENT_S, state))
            else:
                self._emit(self._dispatch(self.agent.on_route_complete,
                                          BUDGET_EVENT_S, state))
        self._prev_mission = cur

    # -- loop ----------------------------------------------------------

    def run(self):
        self._started_at = self._now()
        self._last_frame_at = self._started_at - 1.0 / FRAME_HZ
        self._emit(self._dispatch(self.agent.on_start, BUDGET_EVENT_S,
                                  self.arena))
        tick_dt = 1.0 / TICK_HZ
        frame_dt = 1.0 / FRAME_HZ
        next_tick = self._now()
        while not self._stop.is_set():
            state = self._state()
            self._fire_events(state)
            if (self.arena.time_limit is not None
                    and state.time_elapsed >= self.arena.time_limit):
                self._log("[harness] time limit reached -- holding")
                self.channel.send({"type": "hold"})
                return
            now = self._now()
            if now - self._last_frame_at >= frame_dt:
                self._last_frame_at = now
                self._emit(self._dispatch(self.agent.on_frame, BUDGET_FRAME_S,
                                          self.frames.latest(), state))
            else:
                self._emit(self._dispatch(self.agent.on_tick, BUDGET_TICK_S,
                                          state))
            next_tick += tick_dt
            nap = next_tick - self._now()
            if nap > 0:
                self._sleep(nap)
            else:
                next_tick = self._now()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest competition/tests/test_harness.py -v`
Expected: PASS (10 tests). If `test_on_tick_runs_at_20hz_and_on_frame_at_5hz` is flaky under load, widen the bounds one unit — the invariant is "~5 frames, ~15 ticks, never both in a tick", not exact counts.

- [ ] **Step 5: Commit**

```bash
git add competition/harness.py competition/tests/test_harness.py
git commit -m "feat(competition): the callback loop with per-call timeouts and event synthesis

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 4: `competition/frames.py` — MJPEG frame source

**Files:**
- Create: `competition/frames.py`
- Test: `competition/tests/test_frames.py`

**Interfaces:**
- Consumes: nothing from earlier tasks. Uses `numpy`, `PIL.Image`, `urllib.request`.
- Produces:
  - `competition.frames.MjpegFrames(host: str, port: int, *, camera: str = "nadir")`
  - `MjpegFrames.CAMERA_PATHS = {"nadir": "down", "oblique": "detect"}`
  - `MjpegFrames.start() -> None` — begins the background reader thread
  - `MjpegFrames.latest() -> "np.ndarray | None"` — HxWx3 uint8 RGB, or `None` before the first frame
  - `MjpegFrames.select(camera: str) -> None` — switch endpoint; next `latest()` may be `None` briefly
  - `MjpegFrames.stop() -> None`

- [ ] **Step 1: Write the failing test**

`competition/tests/test_frames.py`:

```python
"""competition.frames -- decode the latest JPEG from a multipart MJPEG stream.

Runs against a local fake MJPEG server, no Isaac. The fake serves the exact
`multipart/x-mixed-replace; boundary=frame` shape
drone_setup_px4_cesium.py's camera server produces.
"""
import io
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition.frames import MjpegFrames  # noqa: E402


def _jpeg(color):
    buf = io.BytesIO()
    Image.new("RGB", (16, 12), color).save(buf, format="JPEG")
    return buf.getvalue()


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        name = self.path.strip("/")
        color = {"down": (255, 0, 0), "detect": (0, 255, 0)}.get(name)
        if color is None:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            for _ in range(200):
                jpeg = _jpeg(color)
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, ValueError):
            return


def _server():
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd, httpd.server_address[1]


def _wait(fn, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = fn()
        if v is not None:
            return v
        time.sleep(0.02)
    raise AssertionError("condition never held")


def test_latest_is_none_before_the_first_frame_then_an_rgb_array():
    httpd, port = _server()
    try:
        f = MjpegFrames("127.0.0.1", port, camera="nadir")
        assert f.latest() is None
        f.start()
        img = _wait(f.latest)
        assert isinstance(img, np.ndarray)
        assert img.shape == (12, 16, 3) and img.dtype == np.uint8
        # nadir -> /down -> red
        assert img[0, 0, 0] > 200 and img[0, 0, 1] < 60
        f.stop()
    finally:
        httpd.shutdown()


def test_select_switches_the_endpoint():
    httpd, port = _server()
    try:
        f = MjpegFrames("127.0.0.1", port, camera="nadir")
        f.start()
        _wait(f.latest)
        f.select("oblique")            # -> /detect -> green
        _wait(lambda: (im := f.latest()) is not None
              and im[0, 0, 1] > 200 and im[0, 0, 0] < 60 or None)
        f.stop()
    finally:
        httpd.shutdown()


def test_stop_ends_the_reader_thread():
    httpd, port = _server()
    try:
        f = MjpegFrames("127.0.0.1", port)
        f.start()
        _wait(f.latest)
        f.stop()
        time.sleep(0.2)
        assert not f._thread.is_alive()
    finally:
        httpd.shutdown()


def test_a_dead_endpoint_leaves_latest_none_without_raising():
    f = MjpegFrames("127.0.0.1", 1, camera="nadir")   # nothing listening
    f.start()
    time.sleep(0.5)
    assert f.latest() is None
    f.stop()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `conda run -n drone python -m pytest competition/tests/test_frames.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'competition.frames'`

- [ ] **Step 3: Write `competition/frames.py`**

```python
"""One MJPEG connection to the Isaac camera server, latest frame decoded.

drone_setup_px4_cesium.py serves each camera at
`http://<host>:8080/<name>` as `multipart/x-mixed-replace; boundary=frame`.
This holds one connection to the selected camera, parses the multipart
stream, and keeps only the most recent JPEG decoded to a numpy RGB array --
`on_frame` wants the freshest frame, never a backlog.

Never raises to the caller: a missing stream just means `latest()` stays
None and the flight continues.
"""
import io
import threading
import urllib.request

import numpy as np
from PIL import Image

_MARKER = b"\xff\xd8"       # JPEG SOI; the multipart parser keys off Content-Length


class MjpegFrames:
    CAMERA_PATHS = {"nadir": "down", "oblique": "detect"}

    def __init__(self, host, port, *, camera="nadir"):
        self.host = host
        self.port = port
        self._camera = camera
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._generation = 0            # bumped on select() to retire the reader
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def select(self, camera):
        if camera not in self.CAMERA_PATHS:
            return
        with self._lock:
            self._camera = camera
            self._latest = None
            self._generation += 1

    def latest(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    # -- internals -------------------------------------------------------

    def _url(self):
        with self._lock:
            path = self.CAMERA_PATHS[self._camera]
            gen = self._generation
        return f"http://{self.host}:{self.port}/{path}", gen

    def _run(self):
        while not self._stop.is_set():
            url, gen = self._url()
            try:
                self._read_stream(url, gen)
            except Exception:            # noqa: BLE001 -- retry, never propagate
                pass
            self._stop.wait(0.5)         # backoff before reconnecting

    def _read_stream(self, url, gen):
        with urllib.request.urlopen(url, timeout=5.0) as resp:
            buf = b""
            while not self._stop.is_set():
                if self._current_generation() != gen:
                    return               # select() moved us elsewhere
                chunk = resp.read(4096)
                if not chunk:
                    return
                buf += chunk
                buf = self._drain(buf, gen)

    def _current_generation(self):
        with self._lock:
            return self._generation

    def _drain(self, buf, gen):
        """Pull complete `Content-Length`-delimited JPEG parts out of buf."""
        while True:
            header_end = buf.find(b"\r\n\r\n")
            if header_end == -1:
                return buf
            header = buf[:header_end].lower()
            marker = b"content-length:"
            i = header.find(marker)
            if i == -1:
                # No length -- drop everything up to the next boundary.
                nxt = buf.find(b"--frame", header_end)
                if nxt == -1:
                    return buf[header_end:]
                buf = buf[nxt:]
                continue
            length = int(header[i + len(marker):header.split(b"\r\n")[
                header[:i].count(b"\r\n")].__len__() and i + len(marker):]
                .split(b"\r\n")[0])
            start = header_end + 4
            if len(buf) < start + length:
                return buf
            self._decode(buf[start:start + length], gen)
            buf = buf[start + length:]

    def _decode(self, jpeg, gen):
        try:
            img = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))
        except Exception:                # noqa: BLE001
            return
        with self._lock:
            if self._generation == gen:
                self._latest = img
```

> **Note for the implementer:** the `length = int(...)` line above is deliberately
> ugly to avoid a regex import and is easy to get wrong. Replace it with the
> straightforward version:
> ```python
> for line in header.split(b"\r\n"):
>     if line.startswith(b"content-length:"):
>         length = int(line.split(b":", 1)[1])
>         break
> else:
>     length = None
> ```
> and handle `length is None` by skipping to the next `--frame`. Keep whichever
> is clearer; the test is the contract.

- [ ] **Step 4: Simplify `_drain` per the note, run tests to verify they pass**

Run: `conda run -n drone python -m pytest competition/tests/test_frames.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add competition/frames.py competition/tests/test_frames.py
git commit -m "feat(competition): MjpegFrames -- latest decoded frame from the camera server

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 5: `streaming/agent_control.py` — `AgentControl`

**Files:**
- Create: `streaming/agent_control.py`
- Test: `streaming/tests/test_agent_control.py`

**Interfaces:**
- Consumes: nothing from earlier tasks (mirrors `offboard.CommandState`).
- Produces:
  - `streaming.agent_control.AgentControl(watchdog_s: float = 0.5)`
  - `AgentControl.set_velocity_body(forward, right, up, yaw_rate)` — stores NED `(vx, vy, vz, yaw_rate)`: `vx=forward`, `vy=right`, `vz=-up`, `yaw_rate` in **rad/s** (converted from deg/s here)
  - `AgentControl.set_velocity_world(north, east, up, yaw_rate)` — stores NED world `(vn, ve, vd=-up, yaw_rate rad/s)`
  - `AgentControl.hold()` — latch zero body velocity, watchdog disabled (explicit, stateful)
  - `AgentControl.clear()` — back to `None` (no agent command)
  - `AgentControl.command(now=None) -> tuple | None` — returns `("body", vx, vy, vz, yaw_rate)`, `("world", vn, ve, vd, yaw_rate)`, or `None`. A stale velocity (older than `watchdog_s`, and not a `hold()`) returns the same kind with all-zero components.
  - `AgentControl.active() -> bool` — True if `command()` would return non-`None`
- Note: `Goto`/`Route` do **not** go through `AgentControl` — the `/agent/control` handler routes those to `loop_thread.load_mission(...)` + `submit("mission_fly")`, exactly like the web mission path.

- [ ] **Step 1: Write the failing test**

`streaming/tests/test_agent_control.py`:

```python
"""streaming.agent_control.AgentControl -- the agent's setpoint holder.

Mirrors offboard.CommandState: the /agent/control handler writes, the
setpoint thread reads once per tick, a silent child decays to hover. The
+up -> NED-down flip lives here and nowhere else.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))

from agent_control import AgentControl  # noqa: E402


def test_no_command_by_default():
    ac = AgentControl()
    assert ac.command() is None
    assert ac.active() is False


def test_body_velocity_flips_up_to_ned_down_and_converts_yaw_to_radians():
    ac = AgentControl(watchdog_s=10.0)
    ac.set_velocity_body(forward=3.0, right=1.0, up=2.0, yaw_rate=90.0)
    kind, vx, vy, vz, yaw = ac.command(now=0.0)
    assert kind == "body"
    assert (vx, vy) == (3.0, 1.0)
    assert vz == -2.0                       # +up -> NED down
    assert math.isclose(yaw, math.radians(90.0))
    assert ac.active() is True


def test_world_velocity_flips_up_and_keeps_north_east():
    ac = AgentControl(watchdog_s=10.0)
    ac.set_velocity_world(north=2.0, east=-1.0, up=1.5, yaw_rate=0.0)
    kind, vn, ve, vd, yaw = ac.command(now=0.0)
    assert kind == "world"
    assert (vn, ve) == (2.0, -1.0)
    assert vd == -1.5


def test_velocity_decays_to_zero_after_the_watchdog():
    ac = AgentControl(watchdog_s=0.5)
    ac.set_velocity_body(forward=3.0, right=0.0, up=0.0, yaw_rate=0.0, now=0.0)
    assert ac.command(now=0.4)[1] == 3.0
    stale = ac.command(now=1.0)
    assert stale == ("body", 0.0, 0.0, 0.0, 0.0)   # still "body", just zeroed
    assert ac.active() is True                       # still latched, just hover


def test_hold_is_zero_body_velocity_and_is_not_watchdogged():
    ac = AgentControl(watchdog_s=0.5)
    ac.hold(now=0.0)
    assert ac.command(now=100.0) == ("body", 0.0, 0.0, 0.0, 0.0)


def test_clear_drops_back_to_no_command():
    ac = AgentControl()
    ac.set_velocity_body(1.0, 0.0, 0.0, 0.0)
    ac.clear()
    assert ac.command() is None
    assert ac.active() is False


def test_switching_from_world_to_body_replaces_not_merges():
    ac = AgentControl(watchdog_s=10.0)
    ac.set_velocity_world(north=5.0, east=0.0, up=0.0, yaw_rate=0.0)
    ac.set_velocity_body(forward=1.0, right=0.0, up=0.0, yaw_rate=0.0)
    assert ac.command(now=0.0)[0] == "body"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `conda run -n drone python -m pytest streaming/tests/test_agent_control.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'agent_control'`

- [ ] **Step 3: Write `streaming/agent_control.py`**

```python
"""The agent's velocity setpoint holder.

Mirrors offboard.CommandState. The /agent/control WebSocket handler (web
thread) writes; SetpointLoop reads command() once per tick. A child that
stops sending decays to hover after watchdog_s -- a stuck velocity is a
hazard, same as a stuck joystick button.

Goto and Route do NOT come here -- they go through waypoints.Mission, exactly
like the operator's map route. This holds only Velocity, VelocityWorld and
the explicit Hold.

UNITS IN: m/s and deg/s, `up` positive. UNITS OUT: m/s and rad/s, NED (down
positive). The +up -> NED flip is here and nowhere else.
"""
import math
import threading
import time


class AgentControl:
    def __init__(self, watchdog_s=0.5):
        self.watchdog_s = watchdog_s
        self._lock = threading.Lock()
        self._kind = None          # None | "body" | "world"
        self._vec = (0.0, 0.0, 0.0, 0.0)
        self._held = False         # a Hold(): latched, not watchdogged
        self._stamp = 0.0

    def _set(self, kind, vec, held, now):
        with self._lock:
            self._kind = kind
            self._vec = vec
            self._held = held
            self._stamp = time.monotonic() if now is None else now

    def set_velocity_body(self, forward, right, up, yaw_rate, now=None):
        self._set("body", (forward, right, -up, math.radians(yaw_rate)),
                  False, now)

    def set_velocity_world(self, north, east, up, yaw_rate, now=None):
        self._set("world", (north, east, -up, math.radians(yaw_rate)),
                  False, now)

    def hold(self, now=None):
        self._set("body", (0.0, 0.0, 0.0, 0.0), True, now)

    def clear(self):
        with self._lock:
            self._kind = None
            self._held = False

    def active(self):
        with self._lock:
            return self._kind is not None

    def command(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._kind is None:
                return None
            a, b, c, d = self._vec
            if not self._held and (now - self._stamp) > self.watchdog_s:
                a = b = c = d = 0.0
            return (self._kind, a, b, c, d)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_agent_control.py -v`
Expected: PASS (7 tests)

- [ ] **Step 5: Commit**

```bash
git add streaming/agent_control.py streaming/tests/test_agent_control.py
git commit -m "feat(streaming): AgentControl -- watchdogged agent velocity holder

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 6: `streaming/offboard.py` — world velocity + attitude telemetry

**Files:**
- Modify: `streaming/offboard.py`
- Test: `streaming/tests/test_offboard.py`

**Interfaces:**
- Consumes: nothing new.
- Produces:
  - `offboard.MAV_FRAME_LOCAL_NED = 1`
  - `offboard.OffboardLink.send_velocity_world(vn, ve, vd, yaw_rate=0.0)` — `SET_POSITION_TARGET_LOCAL_NED` with `MAV_FRAME_LOCAL_NED`, `VEL_YAWRATE_TYPE_MASK`, `(vn, ve, vd)` as velocity, `yaw_rate` rad/s. **Caller passes NED already** (down positive) — the +up flip happened in `AgentControl`.
- Note: attitude parsing (`roll_deg`, `pitch_deg`) and world velocity (`vn`, `ve`) into telemetry is done in `joystick-server.py`'s `SetpointLoop._drain_mavlink` (Task 7), not here — `offboard.py` has no telemetry dict. This task only adds `send_velocity_world` + its constant.

- [ ] **Step 1: Write the failing test**

Append to `streaming/tests/test_offboard.py`:

```python
def test_local_ned_constant_matches_pymavlink():
    from pymavlink.dialects.v20 import common as m
    assert offboard.MAV_FRAME_LOCAL_NED == m.MAV_FRAME_LOCAL_NED


def test_send_velocity_world_uses_local_ned_frame_and_passes_ned_through():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.target_system, link.target_component = 1, 1
    link.send_velocity_world(2.0, -1.0, -1.5, yaw_rate=0.3)

    args = conn.mav.set_position_target_local_ned_send.call_args.args
    # signature: (time, sys, comp, frame, mask, x,y,z, vx,vy,vz, afx,afy,afz, yaw, yawrate)
    frame, mask = args[3], args[4]
    vx, vy, vz = args[8], args[9], args[10]
    yaw_rate = args[15]
    assert frame == offboard.MAV_FRAME_LOCAL_NED
    assert mask == offboard.VEL_YAWRATE_TYPE_MASK
    assert (vx, vy, vz) == (2.0, -1.0, -1.5)      # already NED; not re-flipped
    assert yaw_rate == 0.3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `conda run -n drone python -m pytest streaming/tests/test_offboard.py -k world -v`
Expected: FAIL — `AttributeError: module 'offboard' has no attribute 'MAV_FRAME_LOCAL_NED'`

- [ ] **Step 3: Add the constant and method to `streaming/offboard.py`**

Below `MAV_FRAME_BODY_NED = 8` (line ~19):

```python
# World-frame velocity setpoints, for the agent API's VelocityWorld. PX4
# passes vx/vy through as world north/east and vz as world-down
# (mavlink_receiver.cpp:970-991) -- no yaw rotation, unlike BODY_NED.
MAV_FRAME_LOCAL_NED = 1
```

In `OffboardLink`, directly after `send_velocity`:

```python
    def send_velocity_world(self, vn, ve, vd, yaw_rate=0.0):
        """World-frame velocity: vn north, ve east, vd DOWN (NED). The caller
        has already flipped the API's +up to vd -- see agent_control.py. Same
        setpoint-thread-only contract as send_velocity."""
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.target_system, self.target_component,
            MAV_FRAME_LOCAL_NED,
            VEL_YAWRATE_TYPE_MASK,
            0.0, 0.0, 0.0,
            vn, ve, vd,
            0.0, 0.0, 0.0,
            0.0,
            yaw_rate)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_offboard.py -v`
Expected: PASS (all existing + 2 new)

- [ ] **Step 5: Commit**

```bash
git add streaming/offboard.py streaming/tests/test_offboard.py
git commit -m "feat(offboard): send_velocity_world for the agent VelocityWorld command

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 7: `SetpointLoop` — attitude telemetry + agent setpoint source

**Files:**
- Modify: `joystick-server.py` (`SetpointLoop.__init__`, `_drain_mavlink`, `run`, new `set_agent_*` passthroughs)
- Test: `streaming/tests/test_offboard_loop.py`

**Interfaces:**
- Consumes:
  - `streaming.agent_control.AgentControl` (Task 5)
  - `offboard.OffboardLink.send_velocity_world` (Task 6)
- Produces:
  - `SetpointLoop.__init__` gains `agent_control` positional-or-keyword param (an `AgentControl`); default `None` → the loop constructs its own so existing callers/tests still work.
  - `SetpointLoop.agent_control` attribute
  - Telemetry dict gains: `"roll_deg": 0.0`, `"pitch_deg": 0.0`, `"vn": 0.0`, `"ve": 0.0`
  - `_drain_mavlink` handles `ATTITUDE` → `roll_deg`, `pitch_deg` (degrees); `GLOBAL_POSITION_INT` → `vn = msg.vx/100.0`, `ve = msg.vy/100.0` (cm/s → m/s)
  - `run()` setpoint selection becomes: mission target → else `agent_control.command()` → else `state.command()`. `("body", …)` → `send_velocity`; `("world", …)` → `send_velocity_world`. `cmd_vx` / `cmd_yaw_rate` telemetry reflect whichever source won (for the world case, report the horizontal magnitude in `cmd_vx`).

- [ ] **Step 1: Write the failing tests**

Append to `streaming/tests/test_offboard_loop.py`:

```python
import streaming_agent_control_shim  # noqa  -- see note; delete if not needed


def test_attitude_message_populates_roll_and_pitch_telemetry():
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 20}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)

    class Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

        def get_type(self):
            return "ATTITUDE"

    # pymavlink ATTITUDE carries radians
    import math as _m
    loop._handle_attitude(Msg(roll=_m.radians(4.0), pitch=_m.radians(-11.0),
                              yaw=0.0))
    t = loop.telemetry()
    assert abs(t["roll_deg"] - 4.0) < 1e-6
    assert abs(t["pitch_deg"] + 11.0) < 1e-6


def test_agent_body_velocity_is_sent_when_no_mission_and_no_manual():
    js = _load_server()
    from agent_control import AgentControl
    port = FAKE_PX4_PORT + 21
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        ac = AgentControl(watchdog_s=10.0)
        loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0),
                               rate_hz=20.0, agent_control=ac)
        loop.start()
        ac.set_velocity_body(forward=4.0, right=0.0, up=0.0, yaw_rate=0.0)

        seen = _collect(px4, 1.0)
        assert len(seen) >= 10
        assert last := seen[-1]
        assert last.coordinate_frame == offboard.MAV_FRAME_BODY_NED
        assert abs(last.vx - 4.0) < 1e-6
    finally:
        px4.close()


def test_agent_world_velocity_uses_the_local_ned_frame():
    js = _load_server()
    from agent_control import AgentControl
    port = FAKE_PX4_PORT + 22
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        ac = AgentControl(watchdog_s=10.0)
        loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0),
                               rate_hz=20.0, agent_control=ac)
        loop.start()
        ac.set_velocity_world(north=3.0, east=0.0, up=0.0, yaw_rate=0.0)

        seen = _collect(px4, 1.0)
        assert seen[-1].coordinate_frame == offboard.MAV_FRAME_LOCAL_NED
        assert abs(seen[-1].vx - 3.0) < 1e-6
    finally:
        px4.close()


def test_mission_target_beats_agent_velocity():
    js = _load_server()
    from agent_control import AgentControl
    port = FAKE_PX4_PORT + 23
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        ac = AgentControl(watchdog_s=10.0)
        loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0),
                               rate_hz=20.0, agent_control=ac)
        loop._handle_global_position(_fake_gpi(400000000, -740000000))
        loop.load_mission([[40.0010, -74.0]], 12.0)
        loop.mission.fly()
        ac.set_velocity_body(9.0, 0.0, 0.0, 0.0)
        loop.start()

        seen = _collect(px4, 1.0, kind="SET_POSITION_TARGET_GLOBAL_INT")
        assert len(seen) >= 10, "mission did not win over the agent velocity"
    finally:
        px4.close()


def test_expired_agent_velocity_sends_zero_not_the_manual_command():
    js = _load_server()
    from agent_control import AgentControl
    port = FAKE_PX4_PORT + 24
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        ac = AgentControl(watchdog_s=0.2)
        state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
        state.set("fwd", True)               # manual also held
        loop = js.SetpointLoop(conn, state, rate_hz=20.0, agent_control=ac)
        ac.set_velocity_body(5.0, 0.0, 0.0, 0.0)   # will go stale
        loop.start()
        time.sleep(0.6)                        # agent watchdog expires

        seen = _collect(px4, 0.5)
        # agent is still "active" (latched), so it -- zeroed -- still wins over
        # manual; vx must be ~0, never the manual 2.0.
        assert abs(seen[-1].vx) < 1e-6, f"manual leaked past a latched agent"
    finally:
        px4.close()
```

> Delete the `import streaming_agent_control_shim` line — it was a placeholder;
> `from agent_control import AgentControl` works because `test_offboard_loop.py`
> already does `sys.path.insert(0, os.path.join(ROOT, "streaming"))`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_offboard_loop.py -k "agent or attitude" -v`
Expected: FAIL — `TypeError: __init__() got an unexpected keyword argument 'agent_control'` / `AttributeError: '_handle_attitude'`

- [ ] **Step 3: Modify `joystick-server.py` `SetpointLoop`**

In the imports block (after `import waypoints`):

```python
import agent_control  # noqa: E402
```

`__init__` signature — add `agent_control=None` after `arrival_radius=2.0`:

```python
    def __init__(self, conn, state, rate_hz=20.0, takeoff_alt=5.0, warmup_s=1.0,
                 mission_speed=3.0, arrival_radius=2.0, agent_control=None):
        super().__init__(daemon=True)
        ...
        self.agent_control = agent_control or agent_control_module_default()
```

Simplest: `self.agent_control = agent_control if agent_control is not None else agent_control.AgentControl()` — but the param name shadows the module. Rename the param to `agent_ctl`:

```python
    def __init__(self, conn, state, rate_hz=20.0, takeoff_alt=5.0, warmup_s=1.0,
                 mission_speed=3.0, arrival_radius=2.0, agent_ctl=None):
        ...
        self.agent_control = agent_ctl if agent_ctl is not None \
            else agent_control.AgentControl()
```

> The tests pass `agent_control=ac` as a keyword. Rename the tests' keyword to
> `agent_ctl=ac` OR keep the param `agent_control` and import the module as
> `import agent_control as agent_control_mod`. **Pick keeping the test keyword
> `agent_control=` and import the module `as agentctl`:**
> ```python
> import agent_control as agentctl  # noqa: E402
> ...
> self.agent_control = agent_control if agent_control is not None \
>     else agentctl.AgentControl()
> ```
> Keep it consistent with whatever the test file uses.

Add to the `self._telem` dict literal:

```python
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "vn": 0.0,
            "ve": 0.0,
```

Add a handler method:

```python
    def _handle_attitude(self, msg):
        """ATTITUDE carries radians; the API and UI want degrees."""
        with self._telem_lock:
            self._telem["roll_deg"] = math.degrees(msg.roll)
            self._telem["pitch_deg"] = math.degrees(msg.pitch)
```

In `_drain_mavlink`, add branches:

```python
            elif kind == "ATTITUDE":
                self._handle_attitude(msg)
```

and inside the existing `GLOBAL_POSITION_INT` branch (currently just calls
`_handle_global_position`), also capture world velocity:

```python
            elif kind == "GLOBAL_POSITION_INT":
                self._handle_global_position(msg)
                with self._telem_lock:
                    self._telem["vn"] = msg.vx / 100.0    # cm/s -> m/s
                    self._telem["ve"] = msg.vy / 100.0
```

In `run()`, replace the source dispatch:

```python
            lat, lon = self._position()
            target = self.mission.advance(lat, lon)
            if target is not None:
                wp_lat, wp_lon, wp_alt, wp_yaw = target
                self.link.send_position_global(wp_lat, wp_lon, wp_alt, wp_yaw)
                vx, yaw_rate = 0.0, 0.0
            else:
                agent_cmd = self.agent_control.command()
                if agent_cmd is not None:
                    kind, a, b, c, d = agent_cmd
                    if kind == "world":
                        self.link.send_velocity_world(a, b, c, d)
                    else:
                        self.link.send_velocity(a, b, c, d)
                    vx, yaw_rate = math.hypot(a, b), d
                else:
                    vx, vy, vz, yaw_rate = self.state.command()
                    self.link.send_velocity(vx, vy, vz, yaw_rate)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_offboard_loop.py streaming/tests/test_offboard.py -v`
Expected: PASS (all existing + new)

- [ ] **Step 5: Commit**

```bash
git add joystick-server.py streaming/tests/test_offboard_loop.py
git commit -m "feat(server): agent setpoint source + roll/pitch/world-velocity telemetry

SetpointLoop now picks mission > agent > manual each tick. ATTITUDE and
GLOBAL_POSITION_INT world velocity feed the telemetry the agent State reads.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 8: `POST /agent/upload` and `GET /agent/list`

**Files:**
- Modify: `joystick-server.py` (`build_app`, module constants near `ROOT`)
- Test: `streaming/tests/test_web_ui.py`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces:
  - Module constant `AGENT_UPLOAD_DIR = os.path.join(ROOT, "logs", "agents")`, `os.makedirs(..., exist_ok=True)` at import.
  - Module constant `AGENT_MAX_BYTES = 256 * 1024`.
  - `POST /agent/upload?name=<file.py>` — body is raw bytes. 400 if `name` missing, not `*.py`, contains `/` or `\` or `..`, or body > `AGENT_MAX_BYTES` or not valid UTF-8. On success writes `AGENT_UPLOAD_DIR/<stem>-<YYYYmmdd-HHMMSS>.py`, returns `{"stored": "<filename>"}`.
  - `GET /agent/list` — `{"files": ["name3.py", "name2.py", ...]}` newest first (by mtime).
  - Helper `_safe_agent_name(name: str) -> str | None` (module level) — returns the sanitized base name or `None`.

- [ ] **Step 1: Write the failing tests**

Append to `streaming/tests/test_web_ui.py`:

```python
def test_agent_upload_accepts_a_py_file_and_lists_it(server):
    import urllib.request
    src = b"from competition import Agent\nclass A(Agent):\n    pass\n"
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=myagent.py",
        data=src, method="POST",
        headers={"Content-Type": "text/x-python"})
    with urllib.request.urlopen(req) as r:
        body = json.loads(r.read())
    assert body["stored"].startswith("myagent-") and body["stored"].endswith(".py")

    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/agent/list") as r:
        files = json.loads(r.read())["files"]
    assert body["stored"] in files


def test_agent_upload_rejects_a_non_py_name(server):
    import urllib.request
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=evil.sh",
        data=b"rm -rf /", method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_agent_upload_rejects_a_path_traversal_name(server):
    import urllib.request
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=../../etc/x.py",
        data=b"x = 1", method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_agent_upload_rejects_an_oversize_body(server):
    import urllib.request
    big = b"# " + b"x" * (256 * 1024 + 10)
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=big.py",
        data=big, method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400
```

Also add `import urllib.error` at the top of the file if not present (it is
imported transitively via `urllib.request`, but be explicit).

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -k agent_upload -v`
Expected: FAIL — 404 (route not defined) → `HTTPError: 404` (the "accepts" test fails on the 404 not being JSON)

- [ ] **Step 3: Modify `joystick-server.py`**

Near the top, after `ROOT = ...`:

```python
AGENT_UPLOAD_DIR = os.path.join(ROOT, "logs", "agents")
AGENT_MAX_BYTES = 256 * 1024
os.makedirs(AGENT_UPLOAD_DIR, exist_ok=True)


def _safe_agent_name(name):
    """A base filename ending .py with no path parts, or None."""
    if not name or not name.endswith(".py"):
        return None
    if name != os.path.basename(name) or "/" in name or "\\" in name \
            or ".." in name:
        return None
    return name
```

In `build_app`, add imports `Request` and `PlainTextResponse` to the FastAPI
import line, and add routes before `app.mount(...)`:

```python
    @app.post("/agent/upload")
    async def agent_upload(request: Request):
        from fastapi.responses import JSONResponse
        name = request.query_params.get("name", "")
        safe = _safe_agent_name(name)
        if safe is None:
            return JSONResponse({"detail": "name must be a bare *.py filename"},
                                status_code=400)
        body = await request.body()
        if len(body) > AGENT_MAX_BYTES:
            return JSONResponse({"detail": f"file over {AGENT_MAX_BYTES} bytes"},
                                status_code=400)
        try:
            text = body.decode("utf-8")
        except UnicodeDecodeError:
            return JSONResponse({"detail": "file is not valid UTF-8 text"},
                                status_code=400)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        stored = f"{safe[:-3]}-{stamp}.py"
        with open(os.path.join(AGENT_UPLOAD_DIR, stored), "w") as fh:
            fh.write(text)
        return JSONResponse({"stored": stored})

    @app.get("/agent/list")
    def agent_list():
        from fastapi.responses import JSONResponse
        try:
            entries = [e for e in os.scandir(AGENT_UPLOAD_DIR)
                       if e.is_file() and e.name.endswith(".py")]
        except FileNotFoundError:
            entries = []
        entries.sort(key=lambda e: e.stat().st_mtime, reverse=True)
        return JSONResponse({"files": [e.name for e in entries]})
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -v`
Expected: PASS (all existing + 4 new)

- [ ] **Step 5: Commit**

```bash
git add joystick-server.py streaming/tests/test_web_ui.py
git commit -m "feat(server): POST /agent/upload and GET /agent/list (raw body, no multipart)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 9: `/agent/control` WebSocket

**Files:**
- Modify: `joystick-server.py` (`build_app`)
- Test: `streaming/tests/test_web_ui.py`

**Interfaces:**
- Consumes:
  - `SetpointLoop.agent_control` (Task 7), `SetpointLoop.load_mission`, `SetpointLoop.submit` (existing), `SetpointLoop.telemetry` (existing)
  - `AgentControl.set_velocity_body`, `set_velocity_world`, `hold`, `clear` (Task 5)
- Produces:
  - `@app.websocket("/agent/control")` — the child connects here.
    - On accept: send `{"type":"arena","origin":[lat,lon]|null,"radius_m":ARENA_RADIUS_M,"time_limit":AGENT_TIME_LIMIT_S}` — `origin` is `(telem["lat"], telem["lon"])` if `home_valid` else `null`; the child waits for a non-null origin by reading telemetry frames.
    - Push the server telemetry dict at 5 Hz (reuse `_push_telemetry`).
    - Inbound messages routed:
      - `{"type":"velocity", forward,right,up,yaw_rate}` → `agent_control.set_velocity_body(...)`
      - `{"type":"velocity_world", north,east,up,yaw_rate}` → `agent_control.set_velocity_world(...)`
      - `{"type":"goto", lat,lon,alt,speed}` → `load_mission([[lat,lon]], alt)`; `submit("mission_fly")`
      - `{"type":"route", points, alt, speed}` → `load_mission(points, alt)`; `submit("mission_fly")`
      - `{"type":"hold"}` → `mission.clear()` if a mission is loaded; `agent_control.hold()`
      - `{"type":"camera", camera}` → store on the loop (`loop_thread.agent_camera = camera`) for telemetry; no MAVLink effect
    - On disconnect: leave `agent_control` as-is (the `/ws` stop path or the watchdog handles cleanup — a brief control-socket blip must not drop a latched Hold).
  - `SetpointLoop` gains attribute `agent_camera = "nadir"` (plain attribute, set from this handler; read into telemetry `agent` block in Task 10).
- Note: `speed` on `goto`/`route` is accepted and currently ignored (the server clamps `MPC_XY_VEL_MAX` at startup to `mission_speed`); document that in the handler comment. Not worth a per-mission param change for the PoC.

- [ ] **Step 1: Write the failing test**

Append to `streaming/tests/test_web_ui.py`:

```python
def test_agent_control_socket_sends_arena_then_applies_a_velocity(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(
                f"ws://127.0.0.1:{WEB_PORT}/agent/control") as ws:
            first = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert first["type"] == "arena"
            assert "radius_m" in first and "time_limit" in first

            await ws.send(json.dumps({"type": "velocity", "forward": 3.0,
                                      "right": 0.0, "up": 1.0,
                                      "yaw_rate": 0.0}))
            # telemetry keeps flowing on the same socket
            for _ in range(10):
                t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
                if t.get("type") != "arena":
                    break
            assert "streaming_s" in t

    asyncio.run(exercise())


def test_agent_control_route_message_loads_and_flies_a_mission(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(
                f"ws://127.0.0.1:{WEB_PORT}/agent/control") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)   # arena
            await ws.send(json.dumps({
                "type": "route",
                "points": [[40.0, -74.0], [40.001, -74.0]],
                "alt": 20.0, "speed": None}))
            t = await _telem_where(ws, lambda t: (t.get("mission") or {}).get(
                "count") == 2)
            assert t["mission"]["state"] == "RUNNING"

    asyncio.run(exercise())
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -k agent_control -v`
Expected: FAIL — the `/agent/control` upgrade is refused (404 / connection rejected)

- [ ] **Step 3: Add the handler to `build_app`**

```python
    @app.websocket("/agent/control")
    async def agent_control_ws(sock: WebSocket):
        await sock.accept()
        t0 = loop_thread.telemetry()
        origin = ([t0["lat"], t0["lon"]] if t0.get("home_valid")
                  and t0.get("lat") is not None else None)
        await sock.send_text(json.dumps({
            "type": "arena", "origin": origin,
            "radius_m": ARENA_RADIUS_M, "time_limit": AGENT_TIME_LIMIT_S}))
        pusher = asyncio.create_task(_push_telemetry(sock, loop_thread))
        ac = loop_thread.agent_control
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                kind = msg.get("type")
                if kind == "velocity":
                    ac.set_velocity_body(msg["forward"], msg["right"],
                                         msg["up"], msg["yaw_rate"])
                elif kind == "velocity_world":
                    ac.set_velocity_world(msg["north"], msg["east"],
                                          msg["up"], msg["yaw_rate"])
                elif kind == "goto":
                    # speed is accepted but the server clamps MPC_XY_VEL_MAX
                    # to --mission-speed at startup; per-leg speed is not wired.
                    loop_thread.load_mission([[msg["lat"], msg["lon"]]],
                                             msg["alt"])
                    loop_thread.submit("mission_fly")
                elif kind == "route":
                    loop_thread.load_mission(msg["points"], msg["alt"])
                    loop_thread.submit("mission_fly")
                elif kind == "hold":
                    loop_thread.submit("mission_clear")
                    ac.hold()
                elif kind == "camera":
                    loop_thread.agent_camera = msg.get("camera", "nadir")
        except (WebSocketDisconnect, json.JSONDecodeError, KeyError, ValueError):
            pass
        finally:
            pusher.cancel()
            # Deliberately NOT clearing agent_control here: a control-socket
            # blip must not drop a latched Hold. The /ws stop path and the
            # velocity watchdog own cleanup.
```

Add `ARENA_RADIUS_M = 500.0` and `AGENT_TIME_LIMIT_S = None` to the module
constants near `AGENT_UPLOAD_DIR`. Add `self.agent_camera = "nadir"` in
`SetpointLoop.__init__`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -v`
Expected: PASS (all + 2 new)

- [ ] **Step 5: Commit**

```bash
git add joystick-server.py streaming/tests/test_web_ui.py
git commit -m "feat(server): /agent/control WebSocket -- arena bootstrap, commands, telemetry

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 10: `agent_runner.py` + `/ws` run/stop + run state machine

**Files:**
- Create: `agent_runner.py`
- Modify: `joystick-server.py` (`/ws` handler `agent` action, new `AgentRun` helper class, telemetry `agent` block, `_push_telemetry` already reused)
- Test: `streaming/tests/test_web_ui.py`, plus a standalone `agent_runner` unit test file `streaming/tests/test_agent_runner.py`

**Interfaces:**
- Consumes:
  - `competition.harness.Harness` (Task 3), `competition.frames.MjpegFrames` (Task 4), `competition.state.Arena` (Task 2)
  - `/agent/control` WebSocket (Task 9)
  - `SetpointLoop.telemetry`, `SetpointLoop.submit`, `AgentControl.clear`, `SetpointLoop.agent_camera`
- Produces:
  - `agent_runner.load_agent(path: str) -> type` — imports the file, returns the single `Agent` subclass; raises `RuntimeError` with a clear message on 0 or >1.
  - `agent_runner.main()` — CLI: `--file` (required), `--host` (default `127.0.0.1`), `--port` (default `8090`), `--video-port` (default `8080`). Connects the control socket, waits for an `arena` frame with a non-null `origin`, builds `Arena.around(...)`, starts `MjpegFrames`, runs `Harness`. Exit codes: `0` clean stop, `1` agent raised, `2` load error.
  - `agent_runner.WsChannel` — adapts the control WebSocket to the harness `channel` contract (`.send(dict)` queues a text frame; a background task drains the queue and also reads telemetry frames into `.telemetry()`).
  - In `joystick-server.py`: `class AgentRun` — owns the child `subprocess.Popen`, the arm→takeoff→offboard state machine (driven by a method `tick(telem)` called from the setpoint loop's telemetry cadence OR from a small asyncio task), and `state` ∈ `{"idle","arming","running","stopped","error"}`. A ring buffer `log: collections.deque(maxlen=40)` fed from the child's stdout via a reader thread.
  - Telemetry `agent` block (Task spec §"Telemetry gains"): `{"state","file","camera","log":[...]}`.
  - `/ws` inbound `{"type":"agent","action":"run","file":"<name>"}` and `{"type":"agent","action":"stop"}`.
  - `/ws` `axis` handler: when `pressed` and `agent_run.state in {"arming","running"}` → `agent_run.stop("manual takeover")`.

- [ ] **Step 1: Write the failing tests**

`streaming/tests/test_agent_runner.py`:

```python
"""agent_runner.load_agent -- find exactly one Agent subclass in a file."""
import os
import sys
import textwrap

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

import agent_runner  # noqa: E402


def _write(tmp_path, body):
    p = tmp_path / "script.py"
    p.write_text(textwrap.dedent(body))
    return str(p)


def test_loads_the_single_agent_subclass(tmp_path):
    path = _write(tmp_path, """
        from competition import Agent, Command, Velocity
        class Mine(Agent):
            def on_tick(self, state):
                return Command(flight=Velocity(forward=1.0))
    """)
    cls = agent_runner.load_agent(path)
    assert cls.__name__ == "Mine"


def test_rejects_a_file_with_no_agent(tmp_path):
    path = _write(tmp_path, "x = 1\n")
    with pytest.raises(RuntimeError, match="no Agent subclass"):
        agent_runner.load_agent(path)


def test_rejects_a_file_with_two_agents(tmp_path):
    path = _write(tmp_path, """
        from competition import Agent
        class A(Agent): pass
        class B(Agent): pass
    """)
    with pytest.raises(RuntimeError, match="more than one"):
        agent_runner.load_agent(path)


def test_a_syntax_error_in_the_script_is_reported(tmp_path):
    path = _write(tmp_path, "def broken(\n")
    with pytest.raises(RuntimeError):
        agent_runner.load_agent(path)
```

Append to `streaming/tests/test_web_ui.py` (end-to-end, no Isaac — the child
will fail to reach `:8080` for frames but flight still works):

```python
def test_ws_agent_run_is_refused_when_link_is_down(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({"type": "agent", "action": "run",
                                      "file": "whatever.py"}))
            t = await _telem_where(ws, lambda t: "agent" in t)
            assert t["agent"]["state"] in ("idle", "error")
            # never "arming"/"running" without a MAVLink link
            for _ in range(6):
                t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
                assert t["agent"]["state"] in ("idle", "error")

    asyncio.run(exercise())


def test_ws_telemetry_carries_an_agent_block_from_the_start(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert t["agent"] == {"state": "idle", "file": None,
                                  "camera": None, "log": []}

    asyncio.run(exercise())
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_agent_runner.py streaming/tests/test_web_ui.py -k "agent_run or agent_block or load" -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'agent_runner'`; `KeyError: 'agent'`

- [ ] **Step 3: Write `agent_runner.py`**

```python
#!/usr/bin/env python3
"""Child process that flies an uploaded competition Agent.

Spawned by joystick-server.py once the aircraft is in OFFBOARD. Loads the
script, runs competition.harness.Harness, pulls frames from the Isaac camera
server, and streams Commands back to joystick-server.py over
/agent/control. Never imports pymavlink; its only outward effect is JSON on
that socket.

    agent_runner.py --file logs/agents/foo-20260827-120000.py

Runnable by hand for debugging; joystick-server.py fills in --file/--host/
--port so the operator never types a flag.
"""
import argparse
import asyncio
import importlib.util
import inspect
import json
import queue
import sys
import threading
import traceback

ROOT = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
sys.path.insert(0, ROOT)

from competition import Agent                       # noqa: E402
from competition.frames import MjpegFrames          # noqa: E402
from competition.harness import Harness             # noqa: E402
from competition.state import Arena                 # noqa: E402

import websockets                                   # noqa: E402


def load_agent(path):
    """Import `path` and return its single Agent subclass."""
    spec = importlib.util.spec_from_file_location("_uploaded_agent", path)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except BaseException as exc:                     # syntax error, import error
        raise RuntimeError(f"could not load {path}: {exc!r}") from exc
    found = [obj for _, obj in inspect.getmembers(module, inspect.isclass)
             if issubclass(obj, Agent) and obj is not Agent
             and obj.__module__ == module.__name__]
    if not found:
        raise RuntimeError(f"{path}: no Agent subclass found -- your class must "
                           f"be `class MyAgent(Agent):`")
    if len(found) > 1:
        raise RuntimeError(f"{path}: more than one Agent subclass "
                           f"({', '.join(c.__name__ for c in found)}) -- "
                           f"upload exactly one")
    return found[0]


class WsChannel:
    """Adapts the /agent/control WebSocket to the Harness channel contract."""

    def __init__(self, ws, loop):
        self._ws = ws
        self._loop = loop
        self._telem = {}
        self._lock = threading.Lock()

    def telemetry(self):
        with self._lock:
            return dict(self._telem)

    def _absorb(self, frame):
        with self._lock:
            self._telem = frame

    def send(self, msg):
        # Called from the harness worker threads; hop to the asyncio loop.
        asyncio.run_coroutine_threadsafe(
            self._ws.send(json.dumps(msg)), self._loop)


async def _run(args):
    uri = f"ws://{args.host}:{args.port}/agent/control"
    async with websockets.connect(uri) as ws:
        loop = asyncio.get_running_loop()
        channel = WsChannel(ws, loop)

        # First frame is the arena bootstrap; wait for a non-null origin.
        origin = None
        radius = 500.0
        time_limit = None
        while origin is None:
            frame = json.loads(await ws.recv())
            if frame.get("type") == "arena":
                radius = frame.get("radius_m", 500.0)
                time_limit = frame.get("time_limit")
                origin = frame.get("origin")
            # else: a telemetry frame before home was valid; keep waiting.
        arena = Arena.around(origin[0], origin[1], radius, time_limit)

        frames = MjpegFrames(args.host, args.video_port, camera="nadir")
        frames.start()

        agent_cls = load_agent(args.file)
        harness = Harness(agent_cls(), channel, frames, arena,
                          log=lambda m: print(m, flush=True))

        # Pump telemetry frames into the channel on this loop.
        async def pump():
            try:
                async for raw in ws:
                    frame = json.loads(raw)
                    if frame.get("type") != "arena":
                        channel._absorb(frame)
            except websockets.ConnectionClosed:
                harness.stop()

        pump_task = asyncio.create_task(pump())
        try:
            await loop.run_in_executor(None, harness.run)
        finally:
            harness.stop()
            frames.stop()
            pump_task.cancel()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--file", required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--video-port", type=int, default=8080)
    args = ap.parse_args()

    try:
        load_agent(args.file)                        # fail fast, before flying
    except RuntimeError as exc:
        print(f"[agent_runner] {exc}", flush=True)
        sys.exit(2)

    try:
        asyncio.run(_run(args))
    except Exception:                                # noqa: BLE001
        traceback.print_exc()
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Add the run machinery to `joystick-server.py`**

Imports: `import collections`, `import subprocess`, `import signal`.

New class (module level, after `SetpointLoop`):

```python
class AgentRun:
    """The lifecycle of one uploaded-script flight, on the web-async side.

    Arm -> takeoff -> settle -> offboard -> spawn the child. First manual
    input, Stop, or child exit tears it down. Reads the telemetry the loop
    already produces; never touches MAVLink itself.
    """

    def __init__(self, loop_thread, python_exe, host, port, video_port):
        self.loop_thread = loop_thread
        self.python_exe = python_exe
        self.host, self.port, self.video_port = host, port, video_port
        self.state = "idle"
        self.file = None
        self._proc = None
        self._phase = None
        self._log = collections.deque(maxlen=40)
        self._lock = threading.Lock()

    def snapshot(self):
        with self._lock:
            return {"state": self.state, "file": self.file,
                    "camera": (self.loop_thread.agent_camera
                               if self.state in ("arming", "running") else None),
                    "log": list(self._log)}

    def run(self, filename):
        path = os.path.join(AGENT_UPLOAD_DIR, os.path.basename(filename))
        telem = self.loop_thread.telemetry()
        if not telem.get("connected"):
            self._note("RUN refused: no MAVLink link")
            self.state = "error"
            return
        if self.state in ("arming", "running"):
            self._note("RUN refused: an agent is already running")
            return
        if not os.path.isfile(path):
            self._note(f"RUN refused: {filename} not found")
            self.state = "error"
            return
        self.file = filename
        self.state = "arming"
        self._phase = "arm"
        self._note(f"arming for {filename}")
        self.loop_thread.submit("arm")

    def stop(self, why="stopped"):
        if self.state not in ("arming", "running"):
            return
        self._note(f"stop: {why}")
        p = self._proc
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                p.kill()
        self._proc = None
        self.loop_thread.submit("mission_clear")
        self.loop_thread.agent_control.clear()
        self.state = "stopped"
        self._phase = None

    def tick(self):
        """Called ~5 Hz from the telemetry pusher. Advances the state machine
        and reaps the child."""
        if self.state == "arming":
            self._advance_preflight()
        elif self.state == "running":
            if self._proc and self._proc.poll() is not None:
                code = self._proc.returncode
                self._proc = None
                self.loop_thread.agent_control.hold()
                self.state = "error" if code else "stopped"
                self._note(f"agent exited ({code})")

    def _advance_preflight(self):
        t = self.loop_thread.telemetry()
        if self._phase == "arm" and t.get("armed"):
            self._phase = "takeoff"
            self._note("takeoff")
            self.loop_thread.submit("takeoff")
        elif self._phase == "takeoff" and t.get("alt_m", 0.0) > 1.0 \
                and abs(t.get("vz", 9.0)) < 0.2 and t.get("ready_for_offboard"):
            self._phase = "offboard"
            self._note("offboard")
            self.loop_thread.submit("offboard")
        elif self._phase == "offboard" and t.get("mode") == "OFFBOARD":
            self._phase = None
            self._spawn()

    def _spawn(self):
        path = os.path.join(AGENT_UPLOAD_DIR, os.path.basename(self.file))
        self._proc = subprocess.Popen(
            [self.python_exe, "-u", os.path.join(ROOT, "agent_runner.py"),
             "--file", path, "--host", self.host, "--port", str(self.port),
             "--video-port", str(self.video_port)],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        threading.Thread(target=self._drain_child, daemon=True).start()
        self.state = "running"
        self._note("agent running")

    def _drain_child(self):
        for line in self._proc.stdout:
            self._note(line.rstrip())

    def _note(self, msg):
        with self._lock:
            self._log.append(msg)
        print(f">>> agent: {msg}")
```

Wire it in `main()` — after `loop_thread.start()`:

```python
    agent_run = AgentRun(loop_thread, sys.executable, "127.0.0.1", args.port,
                         args.video_port)
```

Pass `agent_run` into `build_app(...)` (add the parameter). In `build_app`:

- In `_push_telemetry`, the telemetry dict must gain the `agent` block. Cleanest:
  change `_push_telemetry` to take `agent_run` and merge:
  ```python
  async def _push_telemetry(sock, loop_thread, agent_run=None, hz=5.0):
      try:
          while True:
              t = loop_thread.telemetry()
              if agent_run is not None:
                  agent_run.tick()
                  t["agent"] = agent_run.snapshot()
              await sock.send_text(json.dumps(t))
              await asyncio.sleep(1.0 / hz)
      except Exception:
          pass
  ```
  Update both `create_task(_push_telemetry(...))` call sites: `/ws` passes
  `agent_run`; `/agent/control` passes `None` (the child does not need the
  agent block and `tick()` must run on exactly one cadence).
- In the `/ws` handler, handle the new message:
  ```python
  elif kind == "agent":
      action = msg.get("action")
      if action == "run":
          agent_run.run(msg.get("file", ""))
      elif action == "stop":
          agent_run.stop("stop button")
  ```
- In the `/ws` `axis` branch, after the existing `mission_pause` submit:
  ```python
  if msg.get("pressed"):
      loop_thread.submit("mission_pause")
      agent_run.stop("manual takeover")
  ```
- Add `"agent"` to the initial telemetry so `test_ws_telemetry_carries_an_agent_block_from_the_start` passes even before the first `tick()` — simplest is that `snapshot()` on a fresh `AgentRun` already returns `{"state":"idle","file":None,"camera":None,"log":[]}`, which it does.

- [ ] **Step 5: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_agent_runner.py streaming/tests/test_web_ui.py -v`
Expected: PASS. `test_ws_agent_run_is_refused_when_link_is_down` confirms no arming without a link.

- [ ] **Step 6: Commit**

```bash
git add agent_runner.py joystick-server.py streaming/tests/test_agent_runner.py \
        streaming/tests/test_web_ui.py
git commit -m "feat: agent_runner child process + Run/Stop + preflight state machine

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 11: Web UI — the Agent panel

**Files:**
- Modify: `web/index.html`, `web/js/main.js`, `web/css/app.css`
- Create: `web/js/agent.js`
- Test: `streaming/tests/test_web_ui.py` (static-serve smoke)

**Interfaces:**
- Consumes: `/agent/upload`, `/agent/list` (Task 8); `/ws` `{type:"agent",action:...}` (Task 10); telemetry `agent` block (Task 10); `send` from `web/js/ws.js` (the same helper `controls.js` uses — `send(o)` JSON-stringifies and writes to the socket).
- Produces:
  - `web/js/agent.js` exports `initAgent()` and `paintAgent(telem)`.
  - `web/index.html` gains `<div id="agent">` after `#status`.
  - `main.js` calls `initAgent()` once and `paintAgent(t)` per telemetry frame.

- [ ] **Step 1: Write `web/js/agent.js`**

```javascript
// The Agent panel: upload a control script, Run it, watch its log.
// Flight goes through the same /ws socket every other control uses.
import { send } from './ws.js';

const el = id => document.getElementById(id);
let lastLogLen = 0;

async function refreshList(selected) {
  const r = await fetch('/agent/list');
  const { files } = await r.json();
  const sel = el('a-file');
  sel.innerHTML = '';
  for (const f of files) {
    const o = document.createElement('option');
    o.value = o.textContent = f;
    sel.appendChild(o);
  }
  if (selected) sel.value = selected;
}

async function upload(file) {
  const text = await file.text();
  const r = await fetch(
    `/agent/upload?name=${encodeURIComponent(file.name)}`,
    { method: 'POST', headers: { 'Content-Type': 'text/x-python' }, body: text });
  if (!r.ok) {
    const { detail } = await r.json().catch(() => ({ detail: r.statusText }));
    el('a-state').textContent = `upload failed: ${detail}`;
    return;
  }
  const { stored } = await r.json();
  await refreshList(stored);
}

export function initAgent() {
  refreshList();
  el('a-upload').addEventListener('change', e => {
    if (e.target.files[0]) upload(e.target.files[0]);
    e.target.value = '';
  });
  el('a-run').addEventListener('click', () => {
    const file = el('a-file').value;
    if (file) {
      el('a-log').textContent = '';
      lastLogLen = 0;
      send({ type: 'agent', action: 'run', file });
    }
  });
  el('a-stop').addEventListener('click',
    () => send({ type: 'agent', action: 'stop' }));
}

export function paintAgent(t) {
  const a = t.agent;
  if (!a) return;
  el('a-state').textContent = a.state + (a.camera ? ` · ${a.camera}` : '');
  const running = a.state === 'arming' || a.state === 'running';
  el('a-run').disabled = !(t.connected) || running;
  el('a-stop').disabled = !running;
  if (a.log.length !== lastLogLen) {
    el('a-log').textContent = a.log.join('\n');
    el('a-log').scrollTop = el('a-log').scrollHeight;
    lastLogLen = a.log.length;
  }
  // Swap the main video feed to the agent's selected camera while it flies.
  if (a.camera) {
    const path = a.camera === 'oblique' ? 'detect' : 'down';
    const v = el('video');
    const want = `http://${location.hostname}:${v.dataset.port || 8080}/${path}`;
    if (v.src !== want) v.src = want;
  }
}
```

> If `main.js` sets `#video` `src` from `/config`, stash the port:
> `el('video').dataset.port = c.video_port;` in that fetch handler, and on agent
> stop let `main.js` restore `/detect`. Keep the restore simple: when
> `a.state` leaves running and `!a.camera`, set `#video` back to `/detect`.

- [ ] **Step 2: Add the panel to `web/index.html`**

After `<div id="status"></div>`:

```html
  <div id="agent">
    <div class="a-row">
      <select id="a-file" title="uploaded scripts"></select>
      <label class="a-file-btn">upload<input id="a-upload" type="file" accept=".py"></label>
      <button id="a-run" disabled>RUN SCRIPT</button>
      <button id="a-stop" disabled>STOP</button>
      <span id="a-state">idle</span>
    </div>
    <pre id="a-log"></pre>
  </div>
```

- [ ] **Step 3: Wire `web/js/main.js`**

```javascript
import { initAgent, paintAgent } from './agent.js';
// ...
initAgent();
// inside onMessage, alongside the other paint calls:
paintAgent(t);
```

- [ ] **Step 4: Style in `web/css/app.css`**

```css
#agent { margin: 8px; font: 13px/1.4 system-ui, sans-serif; }
#agent .a-row { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; }
#agent select { max-width: 42vw; }
#agent .a-file-btn { border: 1px solid #888; border-radius: 4px; padding: 3px 8px; cursor: pointer; }
#agent .a-file-btn input { display: none; }
#agent #a-state { color: #9cf; }
#a-log {
  margin-top: 6px; height: 9em; overflow-y: auto; white-space: pre-wrap;
  background: #111; color: #cde; padding: 6px; border-radius: 4px;
  font: 12px/1.35 ui-monospace, Menlo, monospace;
}
```

- [ ] **Step 5: Add a static-serve smoke test**

Append to `streaming/tests/test_web_ui.py`:

```python
def test_agent_js_and_panel_are_served(server):
    import urllib.request
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/js/agent.js") as r:
        assert r.status == 200 and b"initAgent" in r.read()
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/") as r:
        assert b'id="agent"' in r.read()
```

- [ ] **Step 6: Run tests + eyeball the page**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -v`
Expected: PASS.
Then: start the server with no Isaac (`conda run -n drone python joystick-server.py &`), open `http://127.0.0.1:8090/`, confirm the panel renders, upload one of the `examples/` files (Task 12), confirm it appears in the dropdown. Kill the server.

- [ ] **Step 7: Commit**

```bash
git add web/index.html web/js/main.js web/js/agent.js web/css/app.css \
        streaming/tests/test_web_ui.py
git commit -m "feat(web): Agent panel -- upload, run, stop, live log

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 12: `examples/` — one agent per flight primitive

**Files:**
- Create: `examples/velocity.py`, `examples/velocity_world.py`, `examples/goto.py`, `examples/route.py`, `examples/hold.py`, `examples/camera.py`
- Test: `competition/tests/test_examples.py`

**Interfaces:**
- Consumes: the full `competition` public API (Tasks 1–2) and `agent_runner.load_agent` (Task 10).
- Produces: six uploadable files, each with exactly one `Agent` subclass.

- [ ] **Step 1: Write the failing test**

`competition/tests/test_examples.py`:

```python
"""Every examples/ file must load as exactly one Agent and its callbacks must
not raise on a plausible State."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

import agent_runner  # noqa: E402
from competition.state import Arena, State  # noqa: E402

EXAMPLES = os.path.join(ROOT, "examples")
FILES = sorted(f for f in os.listdir(EXAMPLES) if f.endswith(".py"))

TELEM = {"lat": 13.6, "lon": 100.3, "alt_m": 30.0, "vn": 0.0, "ve": 0.0,
         "vz": 0.0, "gs": 0.0, "heading_deg": 0.0, "roll_deg": 0.0,
         "pitch_deg": 0.0,
         "mission": {"state": "IDLE", "index": 0, "count": 0, "dist_m": None}}


@pytest.mark.parametrize("fname", FILES)
def test_example_loads_as_one_agent(fname):
    cls = agent_runner.load_agent(os.path.join(EXAMPLES, fname))
    assert cls is not None


@pytest.mark.parametrize("fname", FILES)
def test_example_callbacks_do_not_raise(fname):
    cls = agent_runner.load_agent(os.path.join(EXAMPLES, fname))
    agent = cls()
    arena = Arena.around(13.6, 100.3, 500.0, None)
    st = State.from_telemetry(TELEM, camera="nadir", time_elapsed=1.0,
                              time_limit=None)
    agent.on_start(arena)
    agent.on_tick(st)
    agent.on_frame(None, st)
    agent.on_arrival(st)
    agent.on_waypoint(0, st)
    agent.on_route_complete(st)


def test_all_six_primitives_are_covered():
    assert set(FILES) == {"velocity.py", "velocity_world.py", "goto.py",
                          "route.py", "hold.py", "camera.py"}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `conda run -n drone python -m pytest competition/tests/test_examples.py -v`
Expected: FAIL — `FileNotFoundError: examples` or the coverage assertion.

- [ ] **Step 3: Write the six examples**

`examples/velocity.py`:

```python
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
```

`examples/velocity_world.py`:

```python
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
```

`examples/goto.py`:

```python
"""Goto: fly to a point ~40 m north of the launch position, then hold.

Goto is stateful -- issue it once and forget it; on_arrival fires when the
aircraft gets there. 40 m north is about 0.00036 degrees of latitude.
"""
from competition import Agent, Command, Goto, Hold


class GoNorth40(Agent):
    def on_start(self, arena):
        lat_min, lon_min, lat_max, lon_max = arena.bounds
        centre_lat = (lat_min + lat_max) / 2
        centre_lon = (lon_min + lon_max) / 2
        self.target = (centre_lat + 40 / 111_320.0, centre_lon)
        return Command(flight=Goto(self.target[0], self.target[1], alt=30.0))

    def on_arrival(self, state):
        print(f"arrived at {state.lat:.6f}, {state.lon:.6f}")
        return Command(flight=Hold())
```

`examples/route.py`:

```python
"""Route: fly a ~60 m box around the launch point at 25 m, logging progress.

Route runs closed-loop on the simulator host. on_waypoint fires per corner,
on_route_complete at the end -- then this holds the last point.
"""
from competition import Agent, Command, Hold, Route


class BoxSurvey(Agent):
    def on_start(self, arena):
        lat_min, lon_min, lat_max, lon_max = arena.bounds
        clat = (lat_min + lat_max) / 2
        clon = (lon_min + lon_max) / 2
        d = 30 / 111_320.0
        box = [(clat + d, clon - d), (clat + d, clon + d),
               (clat - d, clon + d), (clat - d, clon - d)]
        return Command(flight=Route(waypoints=box, alt=25.0))

    def on_waypoint(self, index, state):
        print(f"reached waypoint {index}")

    def on_route_complete(self, state):
        print("route complete -- holding")
        return Command(flight=Hold())
```

`examples/hold.py`:

```python
"""Hold: the simplest possible agent. Take off, then just sit there.

Useful as a baseline -- if the drone drifts while running this, the problem
is upstream of any script.
"""
from competition import Agent, Command, Hold


class JustHover(Agent):
    def on_start(self, arena):
        return Command(flight=Hold())
```

`examples/camera.py`:

```python
"""Camera switching: hover, and flip nadir <-> oblique every 5 seconds,
logging the frame size each time on_frame delivers one.

Shows that Command(camera=...) changes what on_frame receives without
touching the flight path.
"""
from competition import Agent, Command, Hold


class CameraToggle(Agent):
    def on_start(self, arena):
        self.want = "nadir"
        self.flipped_at = 0.0
        return Command(flight=Hold(), camera="nadir")

    def on_frame(self, image, state):
        shape = None if image is None else getattr(image, "shape", None)
        print(f"t={state.time_elapsed:5.1f}s camera={state.camera} frame={shape}")
        if state.time_elapsed - self.flipped_at >= 5.0:
            self.flipped_at = state.time_elapsed
            self.want = "oblique" if state.camera == "nadir" else "nadir"
            return Command(camera=self.want)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest competition/tests/test_examples.py -v`
Expected: PASS (13 parametrized + 1)

- [ ] **Step 5: Commit**

```bash
git add examples/ competition/tests/test_examples.py
git commit -m "feat(examples): one uploadable agent per flight primitive

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Task 13: Docs + full-suite gate + manual flight checklist

**Files:**
- Modify: `RUN-WEBSITE.md`, `SESSION.md`
- No code.

**Interfaces:** none.

- [ ] **Step 1: Run the entire test suite**

Run:
```bash
conda run -n drone python -m pytest competition/ streaming/tests/ sim/tests/ -q
```
Expected: all green. Fix any regression before writing docs. Record the count.

- [ ] **Step 2: Add a section to `RUN-WEBSITE.md`**

After section 7 ("Fly a route"), add section 7A "Fly an uploaded script":

```markdown
## 7A. Fly an uploaded script

The **Agent** panel below the command buttons runs a Python control script
written against the competition API (`docs/competition-api.md`). The script
is a callback class, not a loop — the harness calls `on_tick`, `on_frame`
and the event callbacks and flies whatever `Command` they return.

### Step 1 — pick or upload a script

`examples/` ships one script per flight primitive: `velocity.py`,
`velocity_world.py`, `goto.py`, `route.py`, `hold.py`, `camera.py`. They
appear in the dropdown on a fresh box. To add your own, click **upload** and
choose a `.py` file — it is stored under `logs/agents/` and selected.

### Step 2 — RUN

**RUN SCRIPT** arms, takes off to 5 m, enters OFFBOARD, and starts the
script — you do not press ARM/TAKEOFF/OFFBOARD yourself. The state readout
goes `arming → running`. The log pane shows the script's `print()` output
and any harness messages.

### Step 3 — take over

**Touch the pad or press any flight key** and the script is killed
immediately — you are flying manually. There is no resume; press **RUN
SCRIPT** again to restart it from the top. **STOP** does the same without
needing to fly.

### When it goes wrong

| Symptom | Cause |
|---|---|
| RUN SCRIPT greyed out | `link` is down (see 9.1), or a script is already running — press STOP |
| state goes straight to `error`, log shows a traceback | the script raised. The drone holds position; take over or LAND |
| state `error`, log says "no Agent subclass" / "more than one" | your file needs exactly one `class X(Agent):` |
| script runs but the drone barely moves | the `sim` figure — same as 9.4. Check `speed` in telemetry |
| `on_frame` frame is always `None` in the log | the Isaac camera server on :8080 is unreachable; flight still works |
| closed the browser mid-run and the drone kept flying | by design — the child runs server-side. Reload to reconnect |

### Manual test checklist (run once after any change here)

1. Upload and RUN each of the six `examples/`; confirm the motion each
   describes and the expected log lines.
2. RUN `examples/route.py`, press **W** mid-route: the child dies within
   ~0.5 s, the route stops, manual works.
3. Upload a file whose `on_tick` does `raise RuntimeError("boom")`: the
   drone hovers, the log shows the traceback, state is `error`.
4. RUN `examples/hold.py`, close the browser tab, reopen it: telemetry
   shows the agent still `running`.
```

- [ ] **Step 3: Add a note to `SESSION.md`**

Prepend a dated section:

```markdown
# Agent panel — uploading a control script (2026-08-27)

The website can now fly an uploaded competition-API `Agent`. See
RUN-WEBSITE.md section 7A. Design/plan:
docs/superpowers/specs/2026-08-27-website-agent-upload-design.md and
docs/superpowers/plans/2026-08-27-website-agent-upload.md.

Run it exactly as before — no new flags:

    ./sim/launch-sitl.sh
    conda run -n drone python joystick-server.py
    # browse to :8090, use the Agent panel

The agent runs as a child process (`agent_runner.py`) talking to the server
over `/agent/control`; it never touches MAVLink. First manual input kills it.

Tests: `conda run -n drone python -m pytest competition/ streaming/tests/ -q`
```

- [ ] **Step 4: Commit**

```bash
git add RUN-WEBSITE.md SESSION.md
git commit -m "docs: how to fly an uploaded script + manual test checklist

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

- [ ] **Step 5: Manual flight against Isaac (not automatable)**

With Isaac + PX4 running (`./sim/launch-sitl.sh`), work through the
RUN-WEBSITE.md §7A manual test checklist. This is the real acceptance gate —
the unit suite proves the parts, this proves the flight. Report results.

---

## Self-Review

**1. Spec coverage:**

| Spec section | Task |
|---|---|
| `competition/` package (Agent, commands) | 1 |
| `State` / `Arena` from telemetry | 2 |
| `Harness` loop, timeouts, event synthesis, `Command`→channel | 3 |
| Frame decode + camera switch (`on_frame` image) | 4 |
| `AgentControl` (+up→NED, watchdog, Hold) | 5 |
| `send_velocity_world` | 6 |
| `SetpointLoop` source priority (mission ▸ agent ▸ manual); roll/pitch/world-vel telemetry | 7 |
| `POST /agent/upload`, `GET /agent/list` (raw body, no multipart) | 8 |
| `/agent/control` WebSocket (arena bootstrap, commands, telemetry push) | 9 |
| `agent_runner.py` child; `/ws` run/stop; ARM→TAKEOFF→OFFBOARD; first-manual-input kill; telemetry `agent` block; child stdout→log | 10 |
| Web Agent panel (`agent.js`, markup, css, camera swap) | 11 |
| `examples/` one per primitive | 12 |
| `RUN-WEBSITE.md` §7A + `SESSION.md` + manual checklist | 13 |
| No new flags / no new deps / child never speaks MAVLink | Global Constraints + Tasks 8–10 |
| AD1 child process | 10 |
| AD2 child never speaks MAVLink | 10 (`agent_runner.py` imports no pymavlink) |
| AD3 reuse `waypoints.Mission` for Route/Goto | 9 (`load_mission`+`mission_fly`), 3 (event synthesis) |
| AD4 Run auto-sequences preflight | 10 (`AgentRun._advance_preflight`) |
| AD5 first manual input kills, no resume | 10 (`axis` branch → `agent_run.stop`) |
| AD6 +up flip in one place | 5 (`AgentControl`), 6 (`send_velocity_world` takes NED) |
| AD7 no Inspect/roi/raycast/lawnmower/submit | scope — not built |
| AD8 nadir→/down, oblique→/detect | 4 (`CAMERA_PATHS`), 11 (video swap) |
| AD9 no new flags | Global Constraints, Tasks 8–9 (module constants) |
| Error handling table | 10 (`AgentRun` states), 3 (overrun log), 4 (frames never raise), 13 (documented) |

Gaps: none. The spec's `state.vx/vy` (world horizontal) and `alt_amsl` — `vx/vy`
are covered (Task 7 `vn`/`ve`); `alt_amsl` is **dropped** as not worth a
`GLOBAL_POSITION_INT.alt` wire for the PoC — `alt_agl` is what every example
needs. If a reviewer wants it, it is one line in Task 7 and one field in Task 2.

**2. Placeholder scan:** The `_drain` `length = int(...)` line in Task 4 Step 3
is explicitly flagged with a correct replacement in Step 4 — not a placeholder,
a "write it the clear way" instruction with the code given. Task 11 uses `send`
from `web/js/ws.js` — the verified export name (`controls.js` imports the same).
No "TODO"/"handle edge cases"/"similar to Task N" anywhere.

**3. Type consistency:**
- `AgentControl.command()` returns `(kind, a, b, c, d)` — used identically in
  Task 5 tests and Task 7 `run()`.
- Channel message shapes in Task 3 (`_emit`) match the `/agent/control` inbound
  routing in Task 9.
- `MjpegFrames.CAMERA_PATHS` keys (`nadir`/`oblique`) match `competition.commands.CAMERAS`
  (Task 1) and the harness `_camera` default (Task 3) and the web swap (Task 11).
- `AgentRun.snapshot()` dict (`state`/`file`/`camera`/`log`) matches the telemetry
  assertion in Task 10 and `paintAgent` reads in Task 11.
- `SetpointLoop.__init__` new param: Task 7 Step 3 explicitly resolves the
  module-name/param-name clash — implementer must keep the test keyword and the
  code in sync (call it out in review).

---

## Execution Handoff

Two execution options:

**1. Subagent-Driven (recommended)** — a fresh subagent per task, two-stage review between tasks, fast iteration.

**2. Inline Execution** — tasks executed in this session via `superpowers:executing-plans`, batched with review checkpoints.

Tasks 1–6 are independent of each other except 3←(1,2) and are ideal for the subagent flow. Tasks 7–11 are sequential (each builds on the server changes before it). Task 12 needs 1–2 and 10. Task 13 is last.
