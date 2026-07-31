# Implementation plan: satellite map + autonomous waypoint missions

**Spec:** `docs/superpowers/specs/2026-07-31-map-waypoints-design.md`
**Branch:** `feat/map-waypoints` (off `feat/joystick-offboard`)
**Date:** 2026-07-31
**Baseline:** 42 tests passing in ~3 s (`conda run -n drone python -m pytest streaming/tests -q`)

## Goal

A satellite map panel on the existing flight page showing the drone's live
position and heading, with click-to-plan waypoint routes flown autonomously
through the same OFFBOARD stream that already carries manual joystick input —
so touching the pad takes over in ~0.65 s with no mode switch.

Done means: hover manually, click four points on the satellite imagery, press
FLY, watch the drone fly the route on both map and camera, press FORWARD
mid-leg and be flying by hand within a second with the route still drawn, press
RESUME, watch it finish and hold at the last waypoint.

## Architecture summary

```
WS 'axis' pressed ──► queue: mission_pause ──┐
                                             ▼
        ┌────────── SetpointLoop ──── wp = mission.advance(lat, lon) ────┐
        │  wp     ──► send_position_global(wp.lat, wp.lon, alt, yaw)     │──► PX4
        │  None   ──► send_velocity(*state.command())  ← hover when idle │
        └────────────────────────────────────────────────────────────────┘
                                    ▲
        GLOBAL_POSITION_INT (50 Hz) ─┘  HOME_POSITION (0.5 Hz)
```

One setpoint per tick, as today. The loop dispatches on whether `advance()`
yields a target — never on mission state — so a `DONE` mission keeps holding
its last waypoint instead of drifting on a zero-velocity hover.

## Tech stack

Python 3 in the `drone` conda env; pymavlink, FastAPI, uvicorn, websockets
(all already installed). Leaflet 1.9.4 vendored into `web/vendor/`. Esri World
Imagery basemap, no API key. pytest for all automated tests — **no PX4 and no
Isaac Sim required for Tasks 1-10**.

## Global constraints

- **The setpoint stream must never gap.** Every tick sends exactly one setpoint.
  A gap drops PX4 out of OFFBOARD. This is the load-bearing invariant from the
  joystick PoC and nothing in this plan may break it.
- **Only the setpoint thread touches `conn`.** pymavlink connections are not
  thread-safe. Web-thread work goes on the existing command queue, or onto
  `Mission`, which carries its own lock and touches no MAVLink.
- **`SetpointLoop(conn, state, rate_hz=...)` must keep working positionally** —
  `test_offboard_loop.py:46` constructs it that way. New parameters are
  keyword-with-default, appended.
- **Never nest `_telem_lock` and `Mission._lock`.** After Task 7 there are two
  locks and the setpoint thread touches both every tick. Each call site takes
  one, releases it, then takes the other — `_position()` before `advance()`,
  `mission.status()` before the `_telem_lock` block, `_note_mode()` releasing
  before `mission.pause()`. Held in this order there is no cycle and no
  deadlock; nesting them would introduce one.
- **No new Python dependencies.**
- Every task ends green: `conda run -n drone python -m pytest streaming/tests -q`.

## Task list

| # | Task | Files |
|---|---|---|
| 0 | Branch + vendor Leaflet | `web/vendor/` |
| 1 | `haversine_m` + `bearing_deg` | `streaming/waypoints.py` |
| 2 | `Mission` state machine | `streaming/waypoints.py` |
| 3 | `Mission.advance()` sequencer | `streaming/waypoints.py` |
| 4 | `send_position_global` | `streaming/offboard.py` |
| 5 | Position + home telemetry | `joystick-server.py` |
| 6 | Mission speed clamp | `joystick-server.py` |
| 7 | Mission ownership + dispatch | `joystick-server.py` |
| 8 | WS mission protocol + pause edge | `joystick-server.py` |
| 9 | Map panel + live drone marker | `web/index.html` |
| 10 | Waypoint planning + mission controls | `web/index.html` |
| 11 | Manual acceptance in Isaac Sim | — |

---

## Task 0 — Branch and vendor Leaflet

**Creates:** `web/vendor/leaflet.js`, `web/vendor/leaflet.css`
**Modifies:** `joystick-server.py` (static mount)

Leaflet is vendored rather than loaded from a CDN so a CDN outage cannot take
out the flight UI. Marker **images** are deliberately not vendored — every
marker in this UI is a `divIcon` or `circleMarker`, so the sprite files are
never requested and cannot 404.

- [ ] Branch:
```bash
cd /home/innovation/pai/drone-sitl
git checkout -b feat/map-waypoints
```

- [ ] Download Leaflet 1.9.4:
```bash
mkdir -p web/vendor
curl -sL https://unpkg.com/leaflet@1.9.4/dist/leaflet.js  -o web/vendor/leaflet.js
curl -sL https://unpkg.com/leaflet@1.9.4/dist/leaflet.css -o web/vendor/leaflet.css
ls -l web/vendor/
```
Expected: `leaflet.js` ~147 KB, `leaflet.css` ~14 KB.

- [ ] Sanity-check the download is really Leaflet, not an error page:
```bash
head -c 120 web/vendor/leaflet.js; echo; grep -c "leaflet-container" web/vendor/leaflet.css
```
Expected: a `/* @preserve Leaflet 1.9.4 ... */` banner, and a non-zero count.

- [ ] In `joystick-server.py`, add the import inside `build_app`, next to the
      existing FastAPI imports:
```python
    from fastapi.staticfiles import StaticFiles
```

- [ ] In `build_app`, immediately after `app = FastAPI()`:
```python
    app.mount("/vendor",
              StaticFiles(directory=os.path.join(ROOT, "web", "vendor")),
              name="vendor")
```

- [ ] Verify the mount serves:
```bash
conda run -n drone python joystick-server.py --port 8099 --mavlink udpin:127.0.0.1:14598 &
sleep 4 && curl -s -o /dev/null -w "%{http_code} %{size_download}\n" http://127.0.0.1:8099/vendor/leaflet.js
kill %1
```
Expected: `200 150659` (or thereabouts — a 200 with a six-figure size).

- [ ] Commit:
```
chore(map): vendor Leaflet 1.9.4 and serve it from /vendor

Not a CDN: the flight page has no external dependencies today and a CDN
outage taking out the UI is worse than a pinned library. Marker images are
not vendored because every marker is a divIcon or circleMarker.
```

---

## Task 1 — `haversine_m` and `bearing_deg`

**Creates:** `streaming/waypoints.py`, `streaming/tests/test_waypoints.py`

**Interface produced:**
```python
haversine_m(lat1, lon1, lat2, lon2) -> float    # metres
bearing_deg(lat1, lon1, lat2, lon2) -> float    # compass deg, 0=N, 90=E
```

- [ ] Write the failing test — `streaming/tests/test_waypoints.py`:
```python
"""Unit tests for waypoint sequencing. No PX4, no Isaac, no threads."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import waypoints  # noqa: E402


def test_one_degree_of_latitude_is_about_111_km():
    d = waypoints.haversine_m(0.0, 0.0, 1.0, 0.0)
    assert abs(d - 111194.9) < 1.0


def test_one_degree_of_longitude_at_the_equator_matches_latitude():
    assert abs(waypoints.haversine_m(0.0, 0.0, 0.0, 1.0)
               - waypoints.haversine_m(0.0, 0.0, 1.0, 0.0)) < 1.0


def test_a_short_local_leg_is_metres_not_degrees():
    """The scale that actually matters: a 0.001 deg step is ~111 m."""
    d = waypoints.haversine_m(40.0, -74.0, 40.001, -74.0)
    assert abs(d - 111.19) < 0.1


def test_distance_to_the_same_point_is_zero():
    assert waypoints.haversine_m(40.0, -74.0, 40.0, -74.0) == 0.0


def test_distance_is_symmetric():
    a = waypoints.haversine_m(40.0, -74.0, 40.01, -74.01)
    b = waypoints.haversine_m(40.01, -74.01, 40.0, -74.0)
    assert abs(a - b) < 1e-9


def test_bearing_of_the_four_cardinal_directions():
    assert waypoints.bearing_deg(0.0, 0.0, 1.0, 0.0) == 0.0      # north
    assert waypoints.bearing_deg(0.0, 0.0, 0.0, 1.0) == 90.0     # east
    assert waypoints.bearing_deg(0.0, 0.0, -1.0, 0.0) == 180.0   # south
    assert waypoints.bearing_deg(0.0, 0.0, 0.0, -1.0) == 270.0   # west


def test_bearing_northeast_is_about_45_but_not_exactly():
    """Great-circle, not flat: a NE leg starts at 44.996 deg, not 45."""
    assert abs(waypoints.bearing_deg(0.0, 0.0, 1.0, 1.0) - 45.0) < 0.01


def test_bearing_is_always_a_positive_compass_angle():
    for lat, lon in [(1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]:
        b = waypoints.bearing_deg(0.0, 0.0, lat, lon)
        assert 0.0 <= b < 360.0
```

- [ ] Verify red:
```bash
conda run -n drone python -m pytest streaming/tests/test_waypoints.py -q
```
Expected: collection error — `ModuleNotFoundError: No module named 'waypoints'`.

- [ ] Create `streaming/waypoints.py`:
```python
"""Waypoint sequencing for autonomous missions.

Pure logic: no MAVLink, no I/O. The setpoint loop asks advance() for a target
every tick and sends whatever it gets back; everything about "which waypoint
are we on" lives here and nowhere else.

Distances are great-circle. Legs in this application are tens to hundreds of
metres, where a flat approximation would also work -- haversine is used because
it is the same handful of lines and has no latitude at which it quietly stops
being true.

Design: docs/superpowers/specs/2026-07-31-map-waypoints-design.md
"""
import math
import threading

EARTH_RADIUS_M = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance between two lat/lon points, in metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing, in compass degrees (0 = N, 90 = E).

    "Initial" matters: on a long leg the bearing changes as you fly it. Legs
    here are short enough that it does not, but the loop recomputes every tick
    anyway, so the aircraft tracks the true course either way.
    """
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(p2)
    x = (math.cos(p1) * math.sin(p2)
         - math.sin(p1) * math.cos(p2) * math.cos(dlam))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
```

- [ ] Verify green:
```bash
conda run -n drone python -m pytest streaming/tests/test_waypoints.py -q
```
Expected: `8 passed`.

- [ ] Commit:
```
feat(waypoints): add haversine distance and great-circle bearing

Pure functions, no I/O. Bearing feeds the yaw field of the position
setpoint so the nose -- and therefore the camera -- points along the leg.
```

---

## Task 2 — `Mission` state machine

**Modifies:** `streaming/waypoints.py`, `streaming/tests/test_waypoints.py`

**Interface produced:**
```python
class Mission:
    IDLE, RUNNING, PAUSED, DONE = "IDLE", "RUNNING", "PAUSED", "DONE"
    Mission(arrival_radius_m=2.0)
    load(points, alt_m)   # points = [(lat, lon), ...]  -> IDLE
    fly()                 # IDLE|PAUSED|DONE -> RUNNING (no-op if no points)
    pause()               # RUNNING -> PAUSED only
    clear()               # -> IDLE, route dropped
    status()              # -> {"state", "index", "count", "dist_m"}  index 0-based
```

`Mission` carries its own `threading.Lock`. The web thread calls `load`; the
setpoint thread calls `advance`. Putting the lock inside the class means no
caller has to remember it.

- [ ] Append to `streaming/tests/test_waypoints.py`:
```python
ROUTE = [(40.0000, -74.0000), (40.0010, -74.0000), (40.0010, -74.0010)]


def test_a_fresh_mission_is_idle_and_empty():
    m = waypoints.Mission()
    s = m.status()
    assert s["state"] == waypoints.Mission.IDLE
    assert s["count"] == 0
    assert s["dist_m"] is None


def test_load_stores_the_route_but_does_not_start_it():
    """Planning must never move the aircraft -- FLY is a separate press."""
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    s = m.status()
    assert s["state"] == waypoints.Mission.IDLE
    assert s["count"] == 3
    assert s["index"] == 0


def test_fly_starts_a_loaded_route():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    assert m.status()["state"] == waypoints.Mission.RUNNING


def test_fly_with_no_waypoints_is_a_no_op():
    m = waypoints.Mission()
    m.fly()
    assert m.status()["state"] == waypoints.Mission.IDLE


def test_pause_only_applies_to_a_running_mission():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.pause()
    assert m.status()["state"] == waypoints.Mission.IDLE   # not PAUSED
    m.fly()
    m.pause()
    assert m.status()["state"] == waypoints.Mission.PAUSED


def test_pause_is_idempotent():
    """Every axis press submits a pause; holding a direction sends one, but
    tapping four buttons sends four. They must not stack into anything."""
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.pause(); m.pause(); m.pause()
    assert m.status()["state"] == waypoints.Mission.PAUSED


def test_fly_after_pause_resumes_without_losing_progress():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.advance(40.0010, -74.0000)      # arrive at wp 1 -> index steps to 1
    m.pause()
    m.fly()
    assert m.status()["state"] == waypoints.Mission.RUNNING
    assert m.status()["index"] == 1   # resumed, not restarted


def test_clear_drops_the_route_and_returns_to_idle():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.clear()
    s = m.status()
    assert s["state"] == waypoints.Mission.IDLE
    assert s["count"] == 0


def test_load_replaces_a_previous_route_and_resets_progress():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.advance(40.0010, -74.0000)
    m.load([(41.0, -75.0)], 20.0)
    s = m.status()
    assert s["count"] == 1
    assert s["index"] == 0
    assert s["state"] == waypoints.Mission.IDLE
```

- [ ] Verify red:
```bash
conda run -n drone python -m pytest streaming/tests/test_waypoints.py -q
```
Expected: `AttributeError: module 'waypoints' has no attribute 'Mission'`.

- [ ] Append to `streaming/waypoints.py`:
```python
class Mission:
    """Waypoint sequencer: owns the route, the altitude and the progress.

    Thread-safe by construction. The web thread calls load/fly/pause/clear;
    the setpoint thread calls advance() twenty times a second. The lock lives
    here so no caller has to remember it.

    States:
      IDLE     nothing planned, or planned but not started
      RUNNING  flying the route
      PAUSED   operator took manual control; route retained
      DONE     past the last waypoint, holding it
    """

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    DONE = "DONE"

    def __init__(self, arrival_radius_m=2.0):
        # Not zero: a position setpoint is a target, not a guarantee, and PX4
        # settles within about a metre. A zero radius hangs on waypoint 1
        # forever.
        self.arrival_radius_m = arrival_radius_m
        self._lock = threading.Lock()
        self._points = []
        self._alt_m = 0.0
        self._index = 0
        self._state = self.IDLE
        self._hold_yaw = 0.0
        self._dist_m = None

    def load(self, points, alt_m):
        """Replace the route. Never starts it -- planning must not fly."""
        with self._lock:
            self._points = [(float(lat), float(lon)) for lat, lon in points]
            self._alt_m = float(alt_m)
            self._index = 0
            self._state = self.IDLE
            self._dist_m = None

    def fly(self):
        """Start, or resume from where a pause left off.

        From DONE this re-flies the loaded route from waypoint 1, which is what
        makes repeating a survey one button press.
        """
        with self._lock:
            if not self._points:
                return
            if self._state == self.DONE:
                self._index = 0
            self._state = self.RUNNING

    def pause(self):
        with self._lock:
            if self._state == self.RUNNING:
                self._state = self.PAUSED

    def clear(self):
        with self._lock:
            self._points = []
            self._index = 0
            self._state = self.IDLE
            self._dist_m = None

    def status(self):
        """Snapshot for telemetry. `index` is 0-based; the UI adds 1."""
        with self._lock:
            return {"state": self._state,
                    "index": self._index,
                    "count": len(self._points),
                    "dist_m": self._dist_m}
```

- [ ] Verify green (`advance` is still missing, so the two tests that call it
      will fail — that is expected and Task 3 fixes them):
```bash
conda run -n drone python -m pytest streaming/tests/test_waypoints.py -q 2>&1 | tail -5
```
Expected: `2 failed, 15 passed` — both failures `AttributeError: 'Mission' object has no attribute 'advance'`.

- [ ] Commit:
```
feat(waypoints): add Mission state machine

IDLE/RUNNING/PAUSED/DONE with its own lock, because the web thread loads
routes while the setpoint thread reads them. load() never starts a route:
planning must not move the aircraft.
```

---

## Task 3 — `Mission.advance()`, the sequencer

**Modifies:** `streaming/waypoints.py`, `streaming/tests/test_waypoints.py`

**Interface produced:**
```python
Mission.advance(lat, lon) -> (lat, lon, alt_m, yaw_deg) | None
```

This return contract *is* the setpoint loop's entire dispatch rule:

| State | Returns |
|---|---|
| `IDLE`, `PAUSED` | `None` |
| no route loaded | `None` |
| `lat` or `lon` is `None` | `None` |
| `RUNNING` | active waypoint + bearing to it |
| `DONE` | **last** waypoint + the final leg's stored bearing |

- [ ] Append to `streaming/tests/test_waypoints.py`:
```python
def test_advance_returns_nothing_when_idle():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    assert m.advance(40.0, -74.0) is None


def test_advance_returns_nothing_when_paused():
    """The whole point of pause: the loop falls through to the manual path."""
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.pause()
    assert m.advance(40.0, -74.0) is None


def test_advance_returns_nothing_without_a_position_fix():
    """Steering toward a waypoint from an unknown position is worse than
    hovering. The mission stays RUNNING and picks up when position returns."""
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    assert m.advance(None, None) is None
    assert m.status()["state"] == waypoints.Mission.RUNNING


def test_advance_returns_nothing_when_no_route_is_loaded():
    m = waypoints.Mission()
    assert m.advance(40.0, -74.0) is None


def test_running_targets_the_first_waypoint_with_its_altitude():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    lat, lon, alt, yaw = m.advance(40.0, -74.0)
    assert (lat, lon) == ROUTE[0]
    assert alt == 12.0


def test_the_nose_points_along_the_leg():
    """wp 1 is due north of the start, so yaw should be ~0 deg."""
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    _, _, _, yaw = m.advance(40.0, -74.0)
    assert abs(yaw) < 0.5


def test_arriving_within_the_radius_steps_to_the_next_waypoint():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    lat, lon, _, _ = m.advance(40.0000, -74.0000)
    assert (lat, lon) == ROUTE[0]
    lat, lon, _, _ = m.advance(40.0010, -74.0000)   # sitting on wp 1
    assert (lat, lon) == ROUTE[1]


def test_a_waypoint_just_outside_the_radius_is_not_reached():
    """2.0 m radius; 0.00005 deg of latitude is ~5.6 m."""
    m = waypoints.Mission(arrival_radius_m=2.0)
    m.load(ROUTE, 12.0)
    m.fly()
    lat, lon, _, _ = m.advance(40.00005, -74.0000)
    assert (lat, lon) == ROUTE[0]
    assert m.status()["index"] == 0


def test_several_waypoints_inside_the_radius_are_consumed_at_once():
    """Two clicks a metre apart must not take two ticks to clear."""
    m = waypoints.Mission(arrival_radius_m=2.0)
    m.load([(40.0, -74.0), (40.000001, -74.0), (40.0010, -74.0)], 12.0)
    m.fly()
    lat, lon, _, _ = m.advance(40.0, -74.0)
    assert (lat, lon) == (40.0010, -74.0)


def test_passing_the_last_waypoint_finishes_the_mission():
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    m.advance(40.0, -74.0)
    m.advance(40.0010, -74.0)
    assert m.status()["state"] == waypoints.Mission.DONE


def test_done_keeps_holding_the_last_waypoint():
    """The regression this exists to catch: dispatching on state == RUNNING
    would drop a finished mission onto a zero-velocity hover, which drifts in
    wind. DONE must still yield a position target."""
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    m.advance(40.0, -74.0)
    m.advance(40.0010, -74.0)
    target = m.advance(40.0010, -74.0)
    assert target is not None
    lat, lon, alt, _ = target
    assert (lat, lon) == (40.0010, -74.0)
    assert alt == 12.0


def test_the_held_yaw_does_not_wander_once_holding():
    """Bearing to a point you are sitting on is numerically meaningless and
    would yaw the aircraft randomly on the spot. The final leg's bearing is
    stored on arrival, not recomputed."""
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    m.advance(40.0, -74.0)            # flying north, yaw ~0
    m.advance(40.0010, -74.0)         # arrive -> DONE
    yaws = [m.advance(40.0010 + 1e-7 * i, -74.0)[3] for i in range(5)]
    assert len(set(yaws)) == 1
    assert abs(yaws[0]) < 0.5         # still the northward course


def test_distance_to_the_active_waypoint_is_reported():
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    m.advance(40.0, -74.0)
    assert abs(m.status()["dist_m"] - 111.19) < 0.1


def test_flying_again_after_done_restarts_from_waypoint_one():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    for lat, lon in ROUTE:
        m.advance(lat, lon)
    m.advance(*ROUTE[-1])
    assert m.status()["state"] == waypoints.Mission.DONE
    m.fly()
    assert m.status()["index"] == 0
    lat, lon, _, _ = m.advance(40.0, -74.0)
    assert (lat, lon) == ROUTE[0]
```

- [ ] Verify red:
```bash
conda run -n drone python -m pytest streaming/tests/test_waypoints.py -q 2>&1 | tail -3
```
Expected: `16 failed, 15 passed` — all failures `AttributeError: ... 'advance'`.

- [ ] Append `advance` to `Mission` in `streaming/waypoints.py`:
```python
    def advance(self, lat, lon):
        """The target for this tick, or None if the loop should fly manually.

        Returning None -- rather than raising or holding some sentinel -- is
        what lets the setpoint loop stay ignorant of the state machine: it
        sends a position setpoint when it gets a target and a velocity
        setpoint when it does not.
        """
        with self._lock:
            # No fix means no navigation. Stay RUNNING: nothing about the
            # operator's intent changed, so the route resumes when GPS returns.
            if lat is None or lon is None or not self._points:
                return None

            if self._state == self.RUNNING:
                while self._index < len(self._points):
                    wp_lat, wp_lon = self._points[self._index]
                    dist = haversine_m(lat, lon, wp_lat, wp_lon)
                    if dist > self.arrival_radius_m:
                        self._dist_m = dist
                        # Recorded every tick while the target is far enough
                        # away for a bearing to mean something. On arrival the
                        # last good value is what DONE holds.
                        self._hold_yaw = bearing_deg(lat, lon, wp_lat, wp_lon)
                        return (wp_lat, wp_lon, self._alt_m, self._hold_yaw)
                    # Inside the radius: consume it and look at the next one in
                    # the same tick, so two nearby clicks do not cost two ticks.
                    self._index += 1
                self._state = self.DONE
                self._index = len(self._points) - 1
                self._dist_m = 0.0

            if self._state == self.DONE:
                wp_lat, wp_lon = self._points[-1]
                return (wp_lat, wp_lon, self._alt_m, self._hold_yaw)

            return None
```

- [ ] Verify green:
```bash
conda run -n drone python -m pytest streaming/tests/test_waypoints.py -q
```
Expected: `31 passed`.

- [ ] Full suite still green:
```bash
conda run -n drone python -m pytest streaming/tests -q
```
Expected: `73 passed`.

- [ ] Commit:
```
feat(waypoints): add advance(), the whole sequencer

Its return contract is the setpoint loop's dispatch rule, so the loop needs
no knowledge of the state machine. DONE still yields the last waypoint:
dispatching on state == RUNNING would drop a finished mission onto a
zero-velocity hover, which drifts in wind.
```

---

## Task 4 — `send_position_global`

**Modifies:** `streaming/offboard.py`, `streaming/tests/test_offboard.py`

**Interface produced:**
```python
offboard.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
offboard.POS_YAW_TYPE_MASK = 2552
OffboardLink.send_position_global(lat, lon, rel_alt_m, yaw_deg)
```

- [ ] Append to `streaming/tests/test_offboard.py`:
```python
def test_position_constants_match_pymavlink():
    from pymavlink.dialects.v20 import common as m
    assert (offboard.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            == m.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
    # position and yaw USED; velocity, acceleration and yaw_rate ignored
    expected = (m.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | m.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    assert offboard.POS_YAW_TYPE_MASK == expected == 2552
    assert not (offboard.POS_YAW_TYPE_MASK
                & m.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
    assert not (offboard.POS_YAW_TYPE_MASK
                & m.POSITION_TARGET_TYPEMASK_X_IGNORE)


def test_send_position_global_scales_degrees_to_1e7():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.send_position_global(40.7128, -74.0060, 12.0, 90.0)
    args = conn.mav.set_position_target_global_int_send.call_args[0]
    assert args[3] == offboard.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert args[4] == offboard.POS_YAW_TYPE_MASK
    assert args[5] == 407128000        # lat_int
    assert args[6] == -740060000       # lon_int
    assert args[7] == 12.0             # alt, relative to home


def test_send_position_global_converts_yaw_to_radians():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.send_position_global(40.0, -74.0, 5.0, 90.0)
    args = conn.mav.set_position_target_global_int_send.call_args[0]
    assert abs(args[14] - math.pi / 2) < 1e-9    # yaw
    assert args[15] == 0.0                       # yaw_rate, masked off


def test_send_position_global_zeroes_the_masked_fields():
    conn = MagicMock()
    link = offboard.OffboardLink(conn)
    link.send_position_global(40.0, -74.0, 5.0, 0.0)
    args = conn.mav.set_position_target_global_int_send.call_args[0]
    assert args[8:14] == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)   # vx..afz
```

> If `test_offboard.py` does not already `import math`, add it to the imports
> at the top of that file.

- [ ] Verify red:
```bash
conda run -n drone python -m pytest streaming/tests/test_offboard.py -q 2>&1 | tail -3
```
Expected: `4 failed` — `AttributeError: module 'offboard' has no attribute 'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT'`.

- [ ] In `streaming/offboard.py`, add after the `VEL_YAWRATE_TYPE_MASK` block:
```python
# Global-frame position setpoints, for autonomous waypoints. PX4 projects
# lat/lon to local NED itself, using the estimator's own reference
# (mavlink_receiver.cpp:1063-1093), so nothing on this side owns a map
# projection and our idea of a position cannot drift from PX4's.
#
# Altitude is relative to HOME. PX4 requires home_position.valid_alt and
# returns SILENTLY without it (mavlink_receiver.cpp:1107-1110), which is why
# the server gates FLY on having received HOME_POSITION.
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

# type_mask: ignore velocity (bits 3-5), acceleration (bits 6-8) and yaw_rate
# (bit 11). USE position (bits 0-2 clear) and yaw (bit 10 clear), so the nose
# -- and therefore the camera -- points along the leg being flown.
POS_YAW_TYPE_MASK = 2552
```

- [ ] Add to `OffboardLink`, immediately after `send_velocity`:
```python
    def send_position_global(self, lat, lon, rel_alt_m, yaw_deg):
        """Fly to a lat/lon at an altitude relative to home, nose on yaw_deg.

        Same contract as send_velocity: setpoint-thread only. Altitude is
        RELATIVE to home, not AMSL -- matching the altitude the UI displays.
        """
        self.conn.mav.set_position_target_global_int_send(
            0,                                  # time_boot_ms (PX4 ignores)
            self.target_system, self.target_component,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            POS_YAW_TYPE_MASK,
            int(lat * 1e7), int(lon * 1e7),     # degrees -> 1e7 fixed point
            float(rel_alt_m),
            0.0, 0.0, 0.0,                      # vx, vy, vz    -- masked off
            0.0, 0.0, 0.0,                      # afx, afy, afz -- masked off
            math.radians(yaw_deg),
            0.0)                                # yaw_rate      -- masked off
```

- [ ] Verify green:
```bash
conda run -n drone python -m pytest streaming/tests -q
```
Expected: `77 passed`.

- [ ] Commit:
```
feat(offboard): add global-frame position setpoints for waypoints

PX4 does the lat/lon projection itself (mavlink_receiver.cpp:1063-1093), so
waypoints go out as raw degrees and no map projection lives on this side.
Mask 2552 keeps yaw in play so the nose tracks the leg.
```

---

## Task 5 — Position and home telemetry

**Modifies:** `joystick-server.py`, `streaming/tests/test_offboard_loop.py`

**Interface produced:** telemetry gains `lat`, `lon`, `home_valid`;
`heading_deg` now comes from `GLOBAL_POSITION_INT.hdg`.
`SetpointLoop._position()` returns `(lat, lon)` for the loop.

- [ ] Append to `streaming/tests/test_offboard_loop.py`:
```python
def test_heading_ignores_the_unknown_sentinel():
    """GLOBAL_POSITION_INT.hdg is centidegrees, but 65535 means UNKNOWN.
    Dividing that by 100 points the map arrow at 655 deg on every frame
    before a heading estimate exists."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 6}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)

    class Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

        def get_type(self):
            return "GLOBAL_POSITION_INT"

    loop._handle_global_position(
        Msg(lat=407128000, lon=-740060000, hdg=9000))
    assert loop.telemetry()["heading_deg"] == 90.0

    loop._handle_global_position(
        Msg(lat=407128000, lon=-740060000, hdg=65535))
    assert loop.telemetry()["heading_deg"] == 90.0      # unchanged, not 655.35


def test_position_telemetry_starts_null_and_fills_in():
    """The map must show 'waiting for position' rather than centring on
    lat/lon 0,0 in the Gulf of Guinea."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 7}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)

    t = loop.telemetry()
    assert t["lat"] is None and t["lon"] is None
    assert t["home_valid"] is False
    assert loop._position() == (None, None)

    class Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

        def get_type(self):
            return "GLOBAL_POSITION_INT"

    loop._handle_global_position(
        Msg(lat=407128000, lon=-740060000, hdg=0))
    assert abs(loop.telemetry()["lat"] - 40.7128) < 1e-7
    assert abs(loop.telemetry()["lon"] + 74.0060) < 1e-7
    assert loop._position() == (loop.telemetry()["lat"],
                                loop.telemetry()["lon"])
```

- [ ] Verify red:
```bash
conda run -n drone python -m pytest streaming/tests/test_offboard_loop.py -q 2>&1 | tail -3
```
Expected: `2 failed` — `AttributeError: 'SetpointLoop' object has no attribute '_handle_global_position'`.

- [ ] In `joystick-server.py`, add to the `self._telem` dict, after
      `"heading_deg": 0.0,`:
```python
            # Map feed. None until the first GLOBAL_POSITION_INT, so the page
            # can say "waiting for position" instead of centring on 0,0.
            "lat": None,
            "lon": None,
            # PX4 needs a valid home altitude to accept GLOBAL_RELATIVE_ALT
            # setpoints and returns SILENTLY without one
            # (mavlink_receiver.cpp:1107-1110). FLY is gated on this.
            "home_valid": False,
```

- [ ] Add these two methods to `SetpointLoop`, after `telemetry()`:
```python
    def _position(self):
        """Current (lat, lon), either may be None. For the setpoint thread."""
        with self._telem_lock:
            return self._telem["lat"], self._telem["lon"]

    def _handle_global_position(self, msg):
        """GLOBAL_POSITION_INT -> map position and heading.

        Heading comes from here rather than ATTITUDE so the map arrow and the
        telemetry row are the same number and cannot disagree.
        """
        with self._telem_lock:
            self._telem["lat"] = msg.lat / 1e7
            self._telem["lon"] = msg.lon / 1e7
            # hdg is centidegrees 0-35999, with 65535 meaning UNKNOWN. Keep
            # the last good heading rather than reporting 655 degrees.
            if msg.hdg != 65535:
                self._telem["heading_deg"] = msg.hdg / 100.0
```

- [ ] In `_drain_mavlink`, **replace** the whole `elif kind == "ATTITUDE":`
      branch (and its two body lines) with:
```python
            elif kind == "GLOBAL_POSITION_INT":
                self._handle_global_position(msg)
            elif kind == "HOME_POSITION":
                with self._telem_lock:
                    self._telem["home_valid"] = True
```

- [ ] Verify green:
```bash
conda run -n drone python -m pytest streaming/tests -q
```
Expected: `79 passed`.

- [ ] Commit:
```
feat(telemetry): publish lat/lon and home validity for the map

Heading moves from ATTITUDE to GLOBAL_POSITION_INT.hdg so the map arrow and
the telemetry row cannot disagree. hdg 65535 means UNKNOWN and is discarded:
dividing it by 100 points the arrow at 655 degrees.
```

---

## Task 6 — Mission speed clamp

**Modifies:** `joystick-server.py`, `streaming/tests/test_offboard_loop.py`

The pad flies at 2 m/s; PX4's default `MPC_XY_VEL_MAX` is 12 m/s
(`mc_pos_control_params.c:412`). Without this, FLY sends the aircraft off six
times faster than anything the operator has seen it do.

**Interface produced:** `--mission-speed` (default 3.0),
`--arrival-radius` (default 2.0), `SetpointLoop(..., mission_speed=3.0,
arrival_radius=2.0)`.

- [ ] Append to `streaming/tests/test_offboard_loop.py`:
```python
def test_startup_params_clamp_the_mission_speed():
    """The pad does 2 m/s; PX4's default position-setpoint ceiling is 12.
    Pressing FLY must not be a step change in how the aircraft behaves."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 8}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0),
                           rate_hz=20.0, mission_speed=3.0)
    sent = {}
    loop.link.set_param = lambda name, value, ptype: sent.__setitem__(name, value)

    loop._send_startup_params()

    assert sent["MPC_XY_VEL_MAX"] == 3.0
    assert sent["COM_RCL_EXCEPT"] == offboard.COM_RCL_EXCEPT_OFFBOARD
    assert "MIS_TAKEOFF_ALT" in sent
```

- [ ] Verify red:
```bash
conda run -n drone python -m pytest streaming/tests/test_offboard_loop.py -q 2>&1 | tail -3
```
Expected: `1 failed` — `TypeError: __init__() got an unexpected keyword argument 'mission_speed'`.

- [ ] Change the `SetpointLoop.__init__` signature (append keyword args only —
      `test_offboard_loop.py:46` passes `conn, state, rate_hz` positionally):
```python
    def __init__(self, conn, state, rate_hz=20.0, takeoff_alt=5.0, warmup_s=1.0,
                 mission_speed=3.0, arrival_radius=2.0):
```

- [ ] Add to the body of `__init__`, after `self.takeoff_alt = takeoff_alt`:
```python
        self.mission_speed = mission_speed
```

- [ ] Add to `_send_startup_params`, before `self._params_sent = True`:
```python
        # PX4's default MPC_XY_VEL_MAX is 12 m/s (mc_pos_control_params.c:412)
        # against the pad's 2 m/s. Unclamped, FLY would send the aircraft off
        # six times faster than anything the operator has seen it do.
        self.link.set_param("MPC_XY_VEL_MAX", self.mission_speed,
                            offboard.MAV_PARAM_TYPE_REAL32)
```

- [ ] Update the print in `_send_startup_params` to mention it:
```python
        print(f">>> params: COM_RCL_EXCEPT=4 (offboard exempt from RC-loss "
              f"failsafe), MIS_TAKEOFF_ALT={self.takeoff_alt}, "
              f"MPC_XY_VEL_MAX={self.mission_speed}")
```

- [ ] Add the two CLI arguments in `main()`, after `--takeoff-alt`:
```python
    ap.add_argument("--mission-speed", type=float, default=3.0,
                    help="waypoint cruise, m/s. Clamps PX4's MPC_XY_VEL_MAX, "
                         "whose 12 m/s default dwarfs the pad's 2 m/s")
    ap.add_argument("--arrival-radius", type=float, default=2.0,
                    help="metres; a waypoint counts as reached inside this")
```

- [ ] Pass them through where `SetpointLoop` is constructed in `main()`:
```python
    loop_thread = SetpointLoop(conn, state, args.rate, args.takeoff_alt,
                               args.offboard_warmup, args.mission_speed,
                               args.arrival_radius)
```

- [ ] Verify green:
```bash
conda run -n drone python -m pytest streaming/tests -q
```
Expected: `80 passed`.

- [ ] Commit:
```
feat(joystick): clamp mission speed via MPC_XY_VEL_MAX at startup

Default 3 m/s against PX4's 12. The pad does 2 m/s, so an unclamped FLY
would violate an intuition built entirely on manual flight, on one press,
toward a point that may be off-screen.
```

---

## Task 7 — Mission ownership and the dispatch

**Modifies:** `joystick-server.py`, `streaming/tests/test_offboard_loop.py`

**Interface produced:** `SetpointLoop.mission`, `SetpointLoop.load_mission`,
`submit()` accepting `mission_fly` / `mission_pause` / `mission_clear`, and the
`advance()`-based dispatch in `run()`.

- [ ] Append to `streaming/tests/test_offboard_loop.py`:
```python
def test_a_running_mission_sends_global_position_setpoints():
    """The dispatch: a target from advance() means a position setpoint, not
    the velocity one the manual path sends."""
    js = _load_server()
    port = FAKE_PX4_PORT + 9
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
        loop = js.SetpointLoop(conn, state, rate_hz=20.0)
        loop._handle_global_position(
            type("M", (), {"lat": 400000000, "lon": -740000000, "hdg": 0,
                           "get_type": lambda s: "GLOBAL_POSITION_INT"})())
        loop.load_mission([[40.0010, -74.0]], 12.0)
        loop.mission.fly()
        loop.start()

        seen = _collect(px4, 1.0, kind="SET_POSITION_TARGET_GLOBAL_INT")
        assert len(seen) >= 10, f"expected >=10 position setpoints, got {len(seen)}"
        last = seen[-1]
        assert last.coordinate_frame == offboard.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        assert last.type_mask == offboard.POS_YAW_TYPE_MASK
        assert last.lat_int == 400010000
        assert abs(last.alt - 12.0) < 1e-4
    finally:
        px4.close()


def test_pausing_a_mission_hands_control_back_to_the_joystick():
    """Takeover: after a pause the very next setpoints are velocity ones
    carrying the held direction, and the stream never stops."""
    js = _load_server()
    port = FAKE_PX4_PORT + 10
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
        loop = js.SetpointLoop(conn, state, rate_hz=20.0)
        loop._handle_global_position(
            type("M", (), {"lat": 400000000, "lon": -740000000, "hdg": 0,
                           "get_type": lambda s: "GLOBAL_POSITION_INT"})())
        loop.load_mission([[40.0010, -74.0]], 12.0)
        loop.mission.fly()
        loop.start()
        _collect(px4, 0.5, kind="SET_POSITION_TARGET_GLOBAL_INT")

        state.set("fwd", True)
        loop.submit("mission_pause")

        seen = _collect(px4, 1.0, kind="SET_POSITION_TARGET_LOCAL_NED")
        assert len(seen) >= 10, f"joystick did not take over: {len(seen)} sent"
        assert abs(seen[-1].vx - 2.0) < 1e-6
        assert loop.mission.status()["state"] == "PAUSED"
    finally:
        px4.close()


def test_leaving_offboard_auto_pauses_a_running_mission():
    """PX4 accepts and discards setpoints outside OFFBOARD
    (mavlink_receiver.cpp:1163). A mission left RUNNING there would look fine
    and do nothing."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 11}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)
    loop.load_mission([[40.0010, -74.0]], 12.0)
    loop.mission.fly()

    loop._note_mode("AUTO.LAND")
    assert loop.mission.status()["state"] == "PAUSED"
```

- [ ] Verify red:
```bash
conda run -n drone python -m pytest streaming/tests/test_offboard_loop.py -q 2>&1 | tail -3
```
Expected: `3 failed` — `AttributeError: 'SetpointLoop' object has no attribute 'load_mission'`.

- [ ] Add the import at the top of `joystick-server.py`, next to `import offboard`:
```python
import waypoints  # noqa: E402
```

- [ ] Add to `SetpointLoop.__init__`, after `self.link = offboard.OffboardLink(conn)`:
```python
        self.mission = waypoints.Mission(arrival_radius)
```

- [ ] Add to the `self._telem` dict, after the `"home_valid": False,` entry:
```python
            "mission": {"state": "IDLE", "index": 0, "count": 0,
                        "dist_m": None},
```

- [ ] Add the mission command map as a class attribute on `SetpointLoop`,
      immediately above `def __init__`:
```python
    # Mission verbs go through the same queue as arm/takeoff so that nothing
    # but the setpoint thread ever mutates flight state mid-tick.
    MISSION_COMMANDS = {"mission_fly": "fly",
                        "mission_pause": "pause",
                        "mission_clear": "clear"}
```

- [ ] Replace `submit()` with:
```python
    def submit(self, name):
        """Called from the web thread. Queue only -- never touches `conn`."""
        if (name in ("arm", "disarm", "takeoff", "land", "offboard")
                or name in self.MISSION_COMMANDS):
            self.commands.put(name)
```

- [ ] Replace `_run_command()` with:
```python
    def _run_command(self, name):
        method = self.MISSION_COMMANDS.get(name)
        if method is not None:
            getattr(self.mission, method)()
        else:
            getattr(self.link, name)()
        print(f">>> command: {name}")
```

- [ ] Add these two methods to `SetpointLoop`, after `_handle_global_position`:
```python
    def load_mission(self, points, alt_m):
        """Called from the web thread. Mission carries its own lock and
        touches no MAVLink, so this needs neither the queue nor _telem_lock."""
        self.mission.load(points, alt_m)

    def _note_mode(self, mode):
        """Record PX4's actual mode, auto-pausing a mission that has lost its
        only means of flying.

        PX4 accepts setpoints outside OFFBOARD and then discards them
        (mavlink_receiver.cpp:1163), so a mission left RUNNING after a mode
        change would report progress it is not making.
        """
        with self._telem_lock:
            self._telem["mode"] = mode
        if mode != "OFFBOARD":
            self.mission.pause()
```

- [ ] In `_drain_mavlink`, in the `HEARTBEAT` branch, remove
      `self._telem["mode"] = mode` from inside the `with self._telem_lock:`
      block and call `self._note_mode(mode)` after that block closes:
```python
            if kind == "HEARTBEAT":
                self.link.bind_target(msg)
                armed = bool(msg.base_mode & offboard.MAV_MODE_FLAG_SAFETY_ARMED)
                mode = offboard.decode_px4_mode(msg.custom_mode)
                with self._telem_lock:
                    self._telem["connected"] = True
                    self._telem["armed"] = armed
                self._note_mode(mode)
                if not self._params_sent:
                    self._send_startup_params()
```

- [ ] Replace the two setpoint lines in `run()` — currently
      `vx, vy, vz, yaw_rate = self.state.command()` followed by
      `self.link.send_velocity(...)` — with the dispatch:
```python
            # The whole autonomous/manual split. A target from advance() means
            # fly the route; None means the operator has it. Either way exactly
            # one setpoint goes out this tick -- a gap drops PX4 out of
            # OFFBOARD.
            lat, lon = self._position()
            target = self.mission.advance(lat, lon)
            if target is not None:
                wp_lat, wp_lon, wp_alt, wp_yaw = target
                self.link.send_position_global(wp_lat, wp_lon, wp_alt, wp_yaw)
                vx, yaw_rate = 0.0, 0.0
            else:
                vx, vy, vz, yaw_rate = self.state.command()
                self.link.send_velocity(vx, vy, vz, yaw_rate)
```

- [ ] Publish mission status each tick — add inside the existing
      `with self._telem_lock:` block in `run()`, after
      `self._telem["streaming_s"] = streaming_s`:
```python
                self._telem["mission"] = mission_status
```
      and immediately **above** that `with` block, add:
```python
            mission_status = self.mission.status()
```

- [ ] Verify green:
```bash
conda run -n drone python -m pytest streaming/tests -q
```
Expected: `83 passed`.

- [ ] Commit:
```
feat(joystick): fly waypoint missions through the existing setpoint loop

The loop dispatches on whether Mission.advance() yields a target, never on
mission state, so a finished route keeps holding its last waypoint. Leaving
OFFBOARD auto-pauses: PX4 discards setpoints in other modes silently.
```

---

## Task 8 — WebSocket mission protocol and the pause edge

**Modifies:** `joystick-server.py`, `streaming/tests/test_web_ui.py`

**Interface produced:**
`{"type":"mission","action":"fly"|"pause"|"clear","points":[[lat,lon],…],"alt":N}`
and `/config` gaining `mission_speed`.

The `fly` action is overloaded by design: **`points` present → load then fly**
(FLY, new route from waypoint 1); **`points` absent → fly only** (RESUME,
continue the loaded route).

- [ ] Append to `streaming/tests/test_web_ui.py`:
```python
def test_config_exposes_mission_speed_for_eta(server):
    """The map shows ETA to the next waypoint, which is a lie unless it uses
    the speed the server actually clamped PX4 to."""
    import urllib.request
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/config") as r:
        cfg = json.loads(r.read())
    assert cfg["video_port"] == 8080
    assert cfg["mission_speed"] == 3.0


def test_mission_can_be_planned_flown_paused_and_cleared_over_the_socket(server):
    """Full protocol round-trip against a live uvicorn, no PX4 needed."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)

            await ws.send(json.dumps({
                "type": "mission", "action": "fly",
                "points": [[40.0, -74.0], [40.001, -74.0]], "alt": 12.0}))
            t = await _telem_where(ws, lambda t: t["mission"]["count"] == 2)
            assert t["mission"]["state"] == "RUNNING"

            await ws.send(json.dumps({"type": "mission", "action": "pause"}))
            t = await _telem_where(ws, lambda t: t["mission"]["state"] == "PAUSED")

            # RESUME: same action, no points -- continues the loaded route
            await ws.send(json.dumps({"type": "mission", "action": "fly"}))
            t = await _telem_where(ws, lambda t: t["mission"]["state"] == "RUNNING")
            assert t["mission"]["count"] == 2      # route retained

            await ws.send(json.dumps({"type": "mission", "action": "clear"}))
            t = await _telem_where(ws, lambda t: t["mission"]["count"] == 0)
            assert t["mission"]["state"] == "IDLE"

    asyncio.run(exercise())


def test_pressing_a_direction_pauses_a_running_mission(server):
    """The takeover edge. Polling held() at 20 Hz would miss a press-release
    inside one tick and keep flying the route; a WS message cannot be missed."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({
                "type": "mission", "action": "fly",
                "points": [[40.0, -74.0]], "alt": 12.0}))
            await _telem_where(ws, lambda t: t["mission"]["state"] == "RUNNING")

            await ws.send(json.dumps({"type": "axis", "dir": "fwd",
                                      "pressed": True}))
            t = await _telem_where(ws, lambda t: t["mission"]["state"] == "PAUSED")
            assert t["mission"]["count"] == 1     # route retained, not cleared

    asyncio.run(exercise())


def test_releasing_a_direction_does_not_pause(server):
    """Only pressed=True pauses. If releases paused too, the mission would
    re-pause forever and RESUME could never take."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({
                "type": "mission", "action": "fly",
                "points": [[40.0, -74.0]], "alt": 12.0}))
            await _telem_where(ws, lambda t: t["mission"]["state"] == "RUNNING")

            await ws.send(json.dumps({"type": "axis", "dir": "fwd",
                                      "pressed": False}))
            for _ in range(4):
                t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert t["mission"]["state"] == "RUNNING"

    asyncio.run(exercise())
```

- [ ] Add this helper to `streaming/tests/test_web_ui.py`, above the tests:
```python
async def _telem_where(ws, predicate, tries=25):
    """Telemetry is pushed at 5 Hz and commands are applied asynchronously, so
    wait for a frame that satisfies the predicate rather than assuming the
    very next one does."""
    last = None
    for _ in range(tries):
        last = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
        if predicate(last):
            return last
    raise AssertionError(f"condition never held; last telemetry: {last}")
```

- [ ] Verify red:
```bash
conda run -n drone python -m pytest streaming/tests/test_web_ui.py -q 2>&1 | tail -3
```
Expected: `4 failed` — `KeyError: 'mission_speed'` and assertion failures on
`t["mission"]`.

- [ ] In `build_app`, change the signature and the `/config` route:
```python
def build_app(loop_thread, state, video_port, mission_speed):
```
```python
    @app.get("/config")
    def config():
        return JSONResponse({"video_port": video_port,
                             "mission_speed": mission_speed})
```

- [ ] Update the `uvicorn.run` call in `main()`:
```python
    uvicorn.run(build_app(loop_thread, state, args.video_port,
                          args.mission_speed),
                host="0.0.0.0", port=args.port, log_level="warning")
```

- [ ] In the `/ws` handler, replace the message-dispatch block with:
```python
                if kind == "axis":
                    state.set(msg["dir"], bool(msg.get("pressed")))
                    # An EDGE, not a level. Polling held() at 20 Hz would miss
                    # a press-release inside one tick -- the drone would twitch
                    # and the mission would keep flying. Only presses pause: if
                    # releases did too, the mission would re-pause forever and
                    # RESUME could never take.
                    if msg.get("pressed"):
                        loop_thread.submit("mission_pause")
                elif kind == "cmd":
                    loop_thread.submit(msg["name"])
                elif kind == "mission":
                    action = msg.get("action")
                    if action == "fly":
                        # Overloaded on purpose: points present means FLY (new
                        # route from waypoint 1), points absent means RESUME
                        # (continue the loaded one). Start and continue are the
                        # same transition, so they are the same action.
                        if "points" in msg:
                            loop_thread.load_mission(msg["points"],
                                                     msg.get("alt", 0.0))
                        loop_thread.submit("mission_fly")
                    elif action == "pause":
                        loop_thread.submit("mission_pause")
                    elif action == "clear":
                        loop_thread.submit("mission_clear")
                elif kind == "ping":
                    state.touch()
```

- [ ] Verify green:
```bash
conda run -n drone python -m pytest streaming/tests -q
```
Expected: `87 passed`.

- [ ] Commit:
```
feat(joystick): add the WebSocket mission protocol

fly/pause/clear, with `fly` overloaded: points present starts a new route,
points absent resumes the loaded one. An axis press submits mission_pause on
the same queue, so takeover is an edge that cannot be missed.
```

---

## Task 9 — Map panel and live drone marker

**Modifies:** `web/index.html`

Camera and map side by side on desktop, stacked on a phone. No behaviour
changes to the pad, telemetry row or command buttons.

- [ ] Add the Leaflet stylesheet in `<head>`, above the existing `<style>`:
```html
<link rel="stylesheet" href="/vendor/leaflet.css">
```

- [ ] Add to the existing `<style>` block:
```css
  #panels { display:flex; gap:10px; flex-wrap:wrap; justify-content:center;
            width:100%; }
  #video { width:min(92vw,640px); aspect-ratio:16/10; background:#000;
           border:1px solid #333; object-fit:cover; }
  #mapwrap { width:min(92vw,640px); display:flex; flex-direction:column;
             gap:6px; }
  #map { width:100%; aspect-ratio:16/10; background:#000;
         border:1px solid #333; }
  /* Leaflet ships light-themed controls that glare on a dark flight page. */
  .leaflet-container { background:#000; font:12px system-ui,sans-serif; }
  .leaflet-control-attribution { background:rgba(0,0,0,.6); color:#888; }
  .leaflet-control-attribution a { color:#aaa; }
  /* The drone: a triangle rotated to the live heading. A divIcon, not an
     image marker -- vendored Leaflet has no image sprites to 404 on. */
  .drone-icon { display:flex; align-items:center; justify-content:center; }
  .drone-arrow { width:0; height:0; border-left:9px solid transparent;
                 border-right:9px solid transparent; border-bottom:22px solid #6f6;
                 filter:drop-shadow(0 0 3px #000); transform-origin:50% 65%; }
  .wp-icon { display:flex; align-items:center; justify-content:center;
             width:22px; height:22px; border-radius:50%; background:#fc6;
             color:#111; font:bold 12px system-ui,sans-serif;
             border:2px solid #111; cursor:pointer; }
  .wp-icon.active { background:#6f6; }
  #mapbar { display:flex; gap:6px; align-items:center; flex-wrap:wrap;
            justify-content:center; }
  #mapbar button { padding:9px 14px; border-radius:8px; border:1px solid #444;
                   background:#222; color:#eee; font-size:14px; }
  #mapbar button:disabled { opacity:.35; }
  #mapbar input { width:62px; padding:7px; border-radius:6px;
                  border:1px solid #444; background:#222; color:#eee;
                  font-size:14px; }
  #mstat { min-height:1.3em; font-size:13px; text-align:center; color:#9a9a9a;
           font-variant-numeric:tabular-nums; }
```

- [ ] Replace the single `<img id="video" …>` line in `<body>` with:
```html
  <div id="panels">
    <img id="video" alt="sim camera feed">
    <div id="mapwrap">
      <div id="map"></div>
      <div id="mapbar">
        <label>alt <input id="m-alt" type="number" step="1" value="5"></label>
        <button id="m-fly" disabled>FLY</button>
        <button id="m-pause" disabled>PAUSE</button>
        <button id="m-clear" disabled>CLEAR</button>
      </div>
      <div id="mstat">click the map to add waypoints</div>
    </div>
  </div>
```

- [ ] Load Leaflet before the existing inline `<script>`:
```html
<script src="/vendor/leaflet.js"></script>
```

- [ ] Add to the top of the inline `<script>`, after `const el = id => …`:
```js
// ---- map ------------------------------------------------------------------
// Esri World Imagery: no API key, and the same place on Earth as the Cesium
// tiles the drone is flying over, because drone_setup_px4_cesium.py sets the
// PX4 GPS origin from the Cesium georeference.
const map = L.map('map', {zoomControl: true}).setView([0, 0], 2);
L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {maxZoom: 19, attribution: 'Imagery &copy; Esri'}).addTo(map);

const droneIcon = L.divIcon({
  className: 'drone-icon', iconSize: [24, 24], iconAnchor: [12, 12],
  html: '<div class="drone-arrow"></div>'});
const droneMarker = L.marker([0, 0], {icon: droneIcon, interactive: false});
let droneOnMap = false;
let centredOnce = false;      // first fix centres; after that the operator pans

function paintDrone(t) {
  if (t.lat === null || t.lon === null) return;
  const p = [t.lat, t.lon];
  if (!droneOnMap) { droneMarker.addTo(map); droneOnMap = true; }
  droneMarker.setLatLng(p);
  if (!centredOnce) { map.setView(p, 18); centredOnce = true; }
  const arrow = droneMarker.getElement()
    && droneMarker.getElement().querySelector('.drone-arrow');
  // The triangle is drawn pointing north, so heading is a plain rotation.
  if (arrow) arrow.style.transform = `rotate(${t.heading_deg}deg)`;
}
```

- [ ] Call it from `paint(t)`, immediately before `checkPending(t)`:
```js
  paintDrone(t);
```

- [ ] Verify the page loads and the map initialises — start the server, open
      `http://<box-ip>:8090/` in a browser, and check the JS console:
```bash
conda run -n drone python joystick-server.py --port 8099 --mavlink udpin:127.0.0.1:14598 &
sleep 4
curl -s http://127.0.0.1:8099/ | grep -c "leaflet\|id=\"map\""
kill %1
```
Expected: `3` or more (the stylesheet link, the script tag, the map div).
In the browser: satellite tiles render, no console errors, and telemetry shows
`link down` because no PX4 is attached — which is correct for this check.

- [ ] Commit:
```
feat(web): add a satellite map panel with a live drone marker

Camera and map side by side, stacking on a phone. The drone is a divIcon
triangle rotated to heading -- not an image marker, because vendored Leaflet
has no sprites and they would 404.
```

---

## Task 10 — Waypoint planning and mission controls

**Modifies:** `web/index.html`

The route lives in a browser array until FLY, so planning costs the aircraft
nothing and can be done mid-hover.

- [ ] Add to the inline `<script>`, after the `paintDrone` function:
```js
// ---- route planning -------------------------------------------------------
// Client-side until FLY: planning must never move the aircraft.
let route = [];                  // [[lat, lon], ...]
let wpMarkers = [];
let routeLine = null;
let missionSpeed = 3.0;          // overwritten from /config
let mission = {state: 'IDLE', index: 0, count: 0, dist_m: null};

function drawRoute() {
  wpMarkers.forEach(m => map.removeLayer(m));
  wpMarkers = route.map((p, i) => {
    const active = mission.state === 'RUNNING' && i === mission.index;
    const m = L.marker(p, {icon: L.divIcon({
      className: '', iconSize: [22, 22], iconAnchor: [11, 11],
      html: `<div class="wp-icon${active ? ' active' : ''}">${i + 1}</div>`})});
    m.on('click', ev => {
      L.DomEvent.stop(ev);       // do not also drop a new waypoint here
      route.splice(i, 1);
      drawRoute();
      paintMission();
    });
    return m.addTo(map);
  });
  if (routeLine) map.removeLayer(routeLine);
  routeLine = route.length > 1
    ? L.polyline(route, {color: '#fc6', weight: 2, dashArray: '6 5'}).addTo(map)
    : null;
}

map.on('click', e => {
  route.push([e.latlng.lat, e.latlng.lng]);
  drawRoute();
  paintMission();
});

// Why FLY is disabled, in the order the operator would hit them. Two of these
// fail SILENTLY inside PX4 -- a mission with no home altitude, or one flown
// outside OFFBOARD, is accepted and discarded -- so naming them here is the
// difference between a diagnosis and a wasted afternoon.
function flyBlocker(t) {
  if (!t.connected) return 'no MAVLink link';
  if (t.lat === null) return 'waiting for position fix';
  if (!t.home_valid) return 'PX4 home position not set yet';
  if (t.mode !== 'OFFBOARD') return `not in OFFBOARD (mode is ${t.mode})`;
  if (!route.length && !mission.count) return 'no waypoints — click the map';
  return null;
}

function paintMission(t) {
  const running = mission.state === 'RUNNING';
  const paused = mission.state === 'PAUSED';
  el('m-fly').textContent = paused ? 'RESUME' : 'FLY';
  el('m-pause').disabled = !running;
  el('m-clear').disabled = !route.length && !mission.count;

  if (!t) return;                // called from a map click; no fresh telemetry
  const blocker = flyBlocker(t);
  el('m-fly').disabled = running || blocker !== null;
  el('m-fly').title = blocker || '';

  if (running || mission.state === 'DONE') {
    const eta = mission.dist_m !== null
      ? ` · ${Math.round(mission.dist_m / missionSpeed)} s` : '';
    el('mstat').textContent = mission.state === 'DONE'
      ? `route complete — holding waypoint ${mission.count}`
      : `flying waypoint ${mission.index + 1}/${mission.count} · `
        + `${mission.dist_m === null ? '--' : mission.dist_m.toFixed(0)} m${eta}`;
  } else if (paused) {
    el('mstat').textContent =
      `paused at waypoint ${mission.index + 1}/${mission.count} — RESUME to continue`;
  } else if (route.length) {
    el('mstat').textContent = blocker
      ? `${route.length} waypoint${route.length > 1 ? 's' : ''} — ${blocker}`
      : `${route.length} waypoint${route.length > 1 ? 's' : ''} — ready to fly`;
  } else {
    el('mstat').textContent = 'click the map to add waypoints';
  }
}

el('m-fly').addEventListener('click', () => {
  if (mission.state === 'PAUSED') {
    send({type: 'mission', action: 'fly'});          // resume; route retained
  } else {
    send({type: 'mission', action: 'fly', points: route,
          alt: parseFloat(el('m-alt').value)});
  }
});
el('m-pause').addEventListener('click',
  () => send({type: 'mission', action: 'pause'}));
el('m-clear').addEventListener('click', () => {
  send({type: 'mission', action: 'clear'});
  route = [];
  drawRoute();
  paintMission();
});
```

- [ ] Default the altitude field to the drone's current altitude — add to
      `paint(t)`, immediately before `paintDrone(t)`:
```js
  // Climb manually to a height that looks right on camera, then plan: the
  // route defaults to where you already are, with no typing.
  if (!altTouched && t.alt_m > 0.5) el('m-alt').value = t.alt_m.toFixed(0);
```
      and declare the flag plus its listener next to the route state:
```js
let altTouched = false;
el('m-alt').addEventListener('input', () => { altTouched = true; });
```

- [ ] Track mission state from telemetry — add to `paint(t)`, immediately
      after `paintDrone(t)`:
```js
  const wasIndex = mission.index, wasState = mission.state;
  mission = t.mission;
  // Only redraw when the highlight actually moves; every frame would fight
  // the operator's clicks.
  if (mission.index !== wasIndex || mission.state !== wasState) drawRoute();
  paintMission(t);
```

- [ ] Pick up `mission_speed` — extend the existing `/config` fetch:
```js
fetch('/config').then(r => r.json()).then(c => {
  el('video').src = `http://${location.hostname}:${c.video_port}/detect`;
  missionSpeed = c.mission_speed;
});
```

- [ ] Verify the suite is still green and the page still serves:
```bash
conda run -n drone python -m pytest streaming/tests -q
```
Expected: `87 passed`.

- [ ] Browser check, no PX4 needed: open the page, click four points on the
      map. Expected: numbered amber markers joined by a dashed line, status
      reads `4 waypoints — no MAVLink link`, FLY disabled with that reason as
      its tooltip. Click marker 2. Expected: it disappears and the rest
      renumber to 1-2-3.

- [ ] Commit:
```
feat(web): plan waypoint routes on the map and fly them

Route stays client-side until FLY, so planning costs the aircraft nothing.
FLY names the reason it is disabled: two of its preconditions fail silently
inside PX4, so an unexplained dead button would cost an afternoon.
```

---

## Task 11 — Manual acceptance in Isaac Sim

**Modifies:** nothing. This is the flight test.

- [ ] Confirm `drone_setup_px4_cesium.py` has `ADD_WIND = False` (line ~41) and
      `STREAM_CAMERAS = True` (line ~65), then run it in Isaac Sim
      (Window → Script Editor → paste → Ctrl+Enter) and press Play.

- [ ] Start the server and watch for the new param line:
```bash
conda run -n drone python joystick-server.py
```
Expected: `>>> params: COM_RCL_EXCEPT=4 …, MIS_TAKEOFF_ALT=5.0, MPC_XY_VEL_MAX=3.0`

- [ ] Open `http://<box-ip>:8090/`. Expected: satellite imagery centres on the
      drone within a second or two, green triangle pointing at its heading,
      status `link up`.

- [ ] **Map tracks reality** — ARM → TAKEOFF → OFFBOARD, then hold TURN R.
      Expected: the triangle rotates on the map in step with the Isaac
      viewport. This is the check that heading is wired to the right field.

- [ ] **Manual first, then plan** — fly forward and climb to ~12 m. Expected:
      the alt field follows to `12`. Click four waypoints, roughly a 100 m box
      starting near the drone.

- [ ] **FLY** — press it. Expected: the drone yaws toward waypoint 1 and
      departs at ~3 m/s; `speed` telemetry settles near 3.0, not 12; status
      counts down metres and seconds; the active marker turns green and steps
      through the route.

- [ ] **Takeover** — mid-leg, press and hold FORWARD. Expected: within about a
      second the drone is flying your command, status reads `paused at waypoint
      i/n`, the route stays drawn on the map, and PX4 never leaves OFFBOARD
      (mode stays green throughout — this is the property the whole design
      exists for).

- [ ] **RESUME** — press it. Expected: the drone returns to the waypoint it was
      heading for and carries on.

- [ ] **Completion** — let it finish. Expected: `route complete — holding
      waypoint 4`, and the drone holds station rather than drifting.

- [ ] **Re-fly** — press FLY again. Expected: it flies the same route from
      waypoint 1.

- [ ] **Gate check** — press LAND mid-mission. Expected: mode leaves OFFBOARD,
      the mission auto-pauses, the drone lands, and FLY is disabled with
      `not in OFFBOARD (mode is AUTO.LAND)` as its tooltip.

- [ ] Update the spec status line to `implemented and flown 2026-07-31`, and add
      any surprises to `docs/joystick-guide.md`.

- [ ] Commit:
```
docs: mark map + waypoint missions flown

Manual acceptance passed in Isaac Sim: map tracks heading, manual-then-plan
flow works, takeover holds OFFBOARD throughout, completed route holds
station.
```

---

## Plan validation

**Spec coverage** — every section of
`2026-07-31-map-waypoints-design.md` maps to a task:

| Spec requirement | Task |
|---|---|
| `haversine_m`, `bearing_deg` | 1 |
| `Mission` states + `advance()` contract table | 2, 3 |
| `DONE` holds last waypoint, stored yaw | 3 |
| `None` without a position fix | 3 |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, mask 2552 | 4 |
| `send_position_global` | 4 |
| `lat`/`lon`/`home_valid` telemetry, `hdg` 65535 | 5 |
| `MPC_XY_VEL_MAX` clamp | 6 |
| `advance()`-based dispatch | 7 |
| Mode-drift auto-pause | 7 |
| `fly`/`pause`/`clear`, `fly` overload | 8 |
| Axis-press pause edge, presses only | 8 |
| Vendored Leaflet, Esri basemap | 0, 9 |
| Map panel, rotating drone marker | 9 |
| Click to add/remove waypoints, alt defaulting | 10 |
| FLY gating naming the blocker | 10 |
| ETA at mission speed | 10 |
| Manual acceptance script | 11 |

**Naming consistency** — checked across all tasks:
`haversine_m`, `bearing_deg`, `Mission.load/fly/pause/clear/advance/status`,
`OffboardLink.send_position_global(lat, lon, rel_alt_m, yaw_deg)`,
`SetpointLoop._position/_handle_global_position/_note_mode/load_mission/mission`,
`MISSION_COMMANDS`, `POS_YAW_TYPE_MASK`,
`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`. `advance()` returns a 4-tuple
`(lat, lon, alt_m, yaw_deg)` everywhere it appears.

**Test count** — 42 at baseline → 73 (Task 3) → 77 (4) → 79 (5) → 80 (6) →
83 (7) → 87 (8).

**Not covered by automated tests, by choice:** tile rendering, marker rotation
and the two-column layout are visual and are verified in Tasks 9-11 by eye.
Everything that can silently fly the aircraft wrong is under pytest.
