# Design: satellite map + autonomous waypoint missions

**Status:** implemented and flown 2026-07-31 -- full mission cycle
(fly, take over, resume, complete, re-fly, clear) verified against PX4 in
Isaac Sim; 88 automated tests pass
**Date:** 2026-07-31
**Builds on:** `2026-07-30-joystick-offboard-design.md` (implemented and flown)

## Goal

Add a satellite map panel to the existing flight page showing the drone's live
position and heading, let the operator plan a route by clicking on it, and fly
that route autonomously — **without giving up instant manual takeover**.

Success is: hover manually, click four points on the satellite imagery, press
FLY, watch the drone fly the route on both the map and the camera feed, then
touch the pad and be flying it by hand inside a second.

This is the manual+autonomous sibling of **V2a — commanding** in
`2026-07-14-roadmap.md`. The joystick PoC's spec said `streaming/offboard.py`
was intended to grow into V2a's `SetpointSender`; this is that growth. It keeps
velocity setpoints for manual and adds position setpoints for autonomous, both
through the one 20 Hz stream that already exists.

## Non-goals (YAGNI)

Per-waypoint altitude, RTL, survey/grid pattern generation, geofencing, mission
save/load, terrain following, obstacle avoidance, multi-vehicle, offline map
tiles, and mission upload to PX4's own `AUTO.MISSION` are all out of scope.

Explicitly rejected during design, with reasons, so they are not re-litigated:

- **PX4 `AUTO.MISSION` upload** — PX4 would own navigation, and QGC would show
  the same mission. Rejected because manual takeover would require a mode
  switch out of `AUTO.MISSION` into `OFFBOARD`, which costs a re-warmup
  (setpoints must stream above 2 Hz *before* PX4 accepts the mode) at exactly
  the moment the operator wants control now. Sharing one OFFBOARD stream makes
  takeover a change of packet contents, not a change of mode.
- **Nearest-waypoint mission start** — avoids flying backwards to waypoint 1
  when the operator has drifted down the route manually. Rejected because the
  same drawn route would then fly differently depending on where the aircraft
  happens to be hovering, which is a bad property for a repeatable survey.
  Missions always start at waypoint 1.
- **Auto-resume on button release** — rejected because releasing a button would
  silently re-engage autonomy. Resuming is an explicit press.

## Architecture

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

**One setpoint still goes out every tick.** The only change is which message.
The loop never asks "is manual held?" — it asks "does the mission have a target
for me?". Manual input is what *removes* that target, via a latched pause, and
it stays removed until the operator presses RESUME.

### Why the dispatch is on `advance()`, not on `state == RUNNING`

Dispatching on `RUNNING` would drop a *finished* mission back onto the velocity
path, hovering on zeros. Zero-velocity hover drifts in wind; a position setpoint
does not — and re-enabling this sim's 5 m/s gusts is the intended next demo. A
completed route must keep holding its last waypoint, so `DONE` still yields a
target.

`advance()` therefore returns a target in `RUNNING` (the active waypoint) and in
`DONE` (the last waypoint, with the final leg's bearing held), and `None` in
`IDLE` and `PAUSED`. The loop needs no knowledge of the state machine at all.

This preserves the invariant the joystick PoC depends on: the setpoint stream
never has a gap, so PX4 never drops out of OFFBOARD.

### Why the pause is an edge, not a level

Dispatching on `CommandState.held()` being non-empty each tick would resume the
mission the instant the operator released the button — the rejected auto-resume
behaviour, arrived at by accident.

The pause is therefore triggered by the **arrival of an `axis` message with
`pressed=true`** in the WebSocket handler, which submits `mission_pause` on the
existing command queue. Polling `held()` at 20 Hz would miss a press-release
inside 50 ms: the drone would twitch and the mission would keep flying. A
WebSocket message cannot be missed.

Only `pressed=true` pauses. Release messages must not, or the mission would
re-pause forever.

### Pause and command land on the same tick

`SetpointLoop.run()` drains the command queue *before* reading held directions
(`joystick-server.py:141-148`). So a mid-mission FORWARD press executes
`mission_pause` and then reads `fwd` as held within the same 50 ms tick. The
drone transitions straight from flying the route to flying the operator's
command — it does not hover first and then start moving.

## Verified constants

Every value below was read out of `~/PX4-Autopilot` or pymavlink's dialect, not
recalled.

| Constant | Value | Source |
|---|---|---|
| `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` | 6 | pymavlink `common`; handled at `mavlink_receiver.cpp:1097` |
| `POS_YAW_TYPE_MASK` | 2552 | vel 8+16+32, accel 64+128+256, yaw_rate 2048 — position and yaw **used** |
| `MPC_XY_VEL_MAX` default | 12.0 m/s | `mc_pos_control_params.c:412` |
| `MPC_Z_VEL_MAX_UP` default | 3.0 m/s | `mc_pos_control_params.c:230` |
| `MPC_ACC_HOR_MAX` default | 5.0 m/s² | `mc_pos_control_params.c:599` |
| `GLOBAL_POSITION_INT` stream rate | 50 Hz | `mavlink_main.cpp:1472` (`MAVLINK_MODE_ONBOARD`) |
| `HOME_POSITION` stream rate | 0.5 Hz | `mavlink_main.cpp` (`MAVLINK_MODE_ONBOARD`) |

Three findings from PX4 source that materially shaped this design:

**1. PX4 does the lat/lon → local NED projection itself.**
`handle_message_set_position_target_global_int` (`mavlink_receiver.cpp:1063`)
builds a `MapProjection` from the estimator's own reference and projects the
target (`:1093`). We therefore send waypoints as raw lat/lon and own **no**
projection code — removing any chance of our idea of a position drifting from
PX4's.

**2. A missing home altitude fails silently.** For
`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, PX4 requires `home_position.valid_alt` and
**returns without error or log** if it is false (`mavlink_receiver.cpp:1107-1110`).
A mission launched before home is set would do nothing, with no diagnostic. FLY
is therefore gated on having received `HOME_POSITION`.

**3. Setpoints outside OFFBOARD are accepted and discarded.** PX4 publishes
`offboard_control_mode` from any valid setpoint but only publishes the
trajectory setpoint when `nav_state == NAVIGATION_STATE_OFFBOARD`
(`mavlink_receiver.cpp:1163`). A mission flown in any other mode would look
perfectly sent and do nothing. FLY is gated on mode.

### Mission speed must be clamped

The manual pad flies at 2 m/s (`--speed-fwd`, `joystick-server.py:225`). PX4's
default ceiling for position setpoints is `MPC_XY_VEL_MAX` = 12 m/s — **six
times faster than anything the operator has seen this aircraft do** — with
climb at 3 m/s against the pad's 1 m/s.

Pressing FLY would therefore violate an intuition built entirely on the pad, on
one button press, toward a point that may be off-screen. The server sets
`MPC_XY_VEL_MAX = --mission-speed` (default **3.0 m/s**) at startup, alongside
the two params it already sets (`joystick-server.py:91-98`): brisk enough that a
100 m leg is not tedious, close enough to 2 m/s that FLY feels like the same
aircraft.

`MPC_XY_CRUISE` (5 m/s, `:321`) governs PX4's own auto modes rather than
offboard setpoints, so `MPC_XY_VEL_MAX` is the binding limit here.

### Takeover latency

| Stage | Time |
|---|---|
| Button press → WS message on LAN | ~1–2 ms |
| Message → next setpoint tick | ≤50 ms (20 Hz, `joystick-server.py:232`) |
| PX4 decelerating from 3 m/s at `MPC_ACC_HOR_MAX` | 0.6 s, ~0.9 m |

**~0.65 s and under a metre of overshoot** from press to the drone obeying.

## Components

### `streaming/waypoints.py` (new)

Pure logic, zero I/O, no MAVLink, no threads — fully unit-testable without PX4
or Isaac.

```python
EARTH_RADIUS_M = 6371000.0

def haversine_m(lat1, lon1, lat2, lon2) -> float
def bearing_deg(lat1, lon1, lat2, lon2) -> float    # 0=N, 90=E, compass
```

```python
class Mission:
    """Waypoint sequencer. Owns route, altitude and progress; no I/O."""
    IDLE, RUNNING, PAUSED, DONE = "IDLE", "RUNNING", "PAUSED", "DONE"

    def __init__(self, arrival_radius_m=2.0)
    def load(self, points, alt_m)   # points = [(lat, lon), ...]; -> IDLE
    def fly(self)                   # IDLE|PAUSED|DONE -> RUNNING
    def pause(self)                 # RUNNING -> PAUSED; no-op otherwise
    def clear(self)                 # -> IDLE, route dropped
    def advance(self, lat, lon)     # -> (lat, lon, alt, yaw_deg) or None
    def status(self)                # -> dict for telemetry
```

`advance()` is the whole sequencer: it measures distance to the active
waypoint, steps the index when inside `arrival_radius_m`, transitions to `DONE`
past the last one, and returns the target plus the bearing to it so the nose
points along the leg.

Its return contract is the loop's entire dispatch rule:

| State | `advance()` returns |
|---|---|
| `IDLE` | `None` — nothing planned |
| `PAUSED` | `None` — the operator is flying |
| `RUNNING` | active waypoint + bearing to it |
| `DONE` | **last** waypoint + the final leg's bearing, held indefinitely |
| any state, `lat`/`lon` is `None` | `None` — cannot navigate without a position |

`DONE` retaining a target is what makes a finished route hold station rather
than drift. The final bearing is stored when the last waypoint is reached, not
recomputed — bearing to a point you are already sitting on is numerically
meaningless and would make the aircraft yaw randomly on the spot.

`fly()` from `DONE` re-flies the loaded route from waypoint 1. `fly()` with no
waypoints is a no-op that stays `IDLE`.

**Arrival radius is 2.0 m**, not zero: a position setpoint is a target, not a
guarantee, and PX4 settles within a metre or so. Zero would hang on waypoint 1
forever.

### `streaming/offboard.py` (extend)

Additive only — nothing existing changes.

```python
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
POS_YAW_TYPE_MASK = 2552

class OffboardLink:
    def send_position_global(self, lat, lon, rel_alt_m, yaw_deg): ...
```

`send_position_global` emits `SET_POSITION_TARGET_GLOBAL_INT` with lat/lon as
`int(deg * 1e7)`, altitude relative to home, and yaw in radians. It is called
only from the setpoint thread, preserving the single-threaded-by-contract rule
on the class.

### `joystick-server.py` (extend `SetpointLoop`)

- `_drain_mavlink` gains two message types: `GLOBAL_POSITION_INT` →
  `lat`/`lon`/`heading_deg` telemetry, and `HOME_POSITION` → `home_valid`.
- `_send_startup_params` gains `MPC_XY_VEL_MAX = --mission-speed`.
- The loop owns a `Mission`. `run()` dispatches on whether
  `mission.advance(lat, lon)` yields a target, never on the state directly.
- `submit()`'s whitelist gains `mission_pause`, `mission_fly`, `mission_clear`.
- A new `load_mission(points, alt)` is called from the WS handler; it mutates
  the `Mission` under a lock rather than going through the queue, because it
  carries a payload and touches no MAVLink.

The `fly` action is overloaded by design, and the rule is: **`points` present →
`load()` then `fly()`; `points` absent → `fly()` only.** FLY sends points and
starts a new route from waypoint 1; RESUME omits them and continues the loaded
route from where it paused. One action, because "start" and "continue" are the
same transition on the state machine and splitting them would let the two drift
apart.

`heading_deg` moves from `ATTITUDE` to `GLOBAL_POSITION_INT.hdg` so the map
arrow and the telemetry row cannot disagree, and `ATTITUDE` handling is
dropped. `hdg` is centidegrees 0–35999, with **65535 meaning unknown** — that
value must be discarded rather than divided by 100, or the arrow will point at
655° on every frame before a heading estimate exists.

### `web/index.html` (extend)

Two-column desktop layout — camera left, map right — stacking on a phone.
Existing pad, telemetry row and command buttons are unchanged.

New map panel:

- Leaflet map, satellite basemap, drone marker as a rotating arrow at `hdg`
- Click empty map → append numbered waypoint marker + extend polyline
- Click existing marker → remove it, renumber the rest
- **ALT** field (metres), defaulting to current altitude on first focus
- **FLY / PAUSE / RESUME / CLEAR** buttons
- Status line: mission state, `waypoint i/n`, distance to active waypoint, and
  ETA at `--mission-speed`

The route lives in a browser array until FLY. Planning therefore costs the
aircraft nothing and can be done mid-hover.

**Leaflet is vendored into `web/vendor/`, not loaded from a CDN.** The page has
no external dependencies today, and a CDN outage taking out the flight UI is a
worse failure than a stale pinned library. FastAPI gains a `StaticFiles` mount.

Basemap is **Esri World Imagery** (`server.arcgisonline.com/ArcGIS/rest/services/
World_Imagery/MapServer/tile/{z}/{y}/{x}`) — no API key, verified reachable from
this box. Attribution is required and rendered in the map corner.

The satellite imagery and the Cesium tiles are the same place on Earth because
`drone_setup_px4_cesium.py:669` sets the PX4 GPS origin from the Cesium
georeference. The map is therefore meaningful with no extra alignment work.

## Data flow

### Planning and flying

1. Operator clicks the map → browser appends `[lat, lon]` to a local array
2. **FLY** → WS `{"type":"mission","action":"fly","points":[[lat,lon],…],"alt":12.0}`
3. Server → `Mission.load(points, alt)` then `Mission.fly()` → `RUNNING`
4. Next 20 Hz tick → `advance(lat, lon)` → `(wp_lat, wp_lon, 12.0, 47.3)`
5. → `SET_POSITION_TARGET_GLOBAL_INT(frame=6, type_mask=2552)`
6. PX4 projects to local NED, position controller flies the leg
7. Inside 2 m of the waypoint → index steps → next leg
8. Past the last waypoint → `DONE`, holds position there

### Taking over

1. `pointerdown ▶` → WS `{"type":"axis","dir":"fwd","pressed":true}`
2. Server → `CommandState.set("fwd", True)` **and** `submit("mission_pause")`
3. Next tick: queue drains → `Mission.pause()` → `PAUSED`
4. Same tick: `state.command()` → `(2.0, 0, 0, 0)` → `send_velocity`
5. Release → zeros → hover. Mission stays `PAUSED`; route stays on the map
6. **RESUME** → WS `{"type":"mission","action":"fly"}` → continues from the
   waypoint it was heading for

## Command sequence

| UI action | Effect |
|---|---|
| map click | client-side only; nothing sent |
| FLY | `mission` action `fly` with points + alt |
| PAUSE | `mission` action `pause` |
| RESUME | `mission` action `fly`, no points — resumes the loaded route |
| CLEAR | `mission` action `clear`; route dropped, state `IDLE` |
| any pad press | implicit `mission_pause` alongside the axis message |

Existing ARM / TAKEOFF / OFFBOARD / LAND / DISARM are unchanged.

## Telemetry additions

```json
{
  "lat": 40.7128, "lon": -74.0060, "home_valid": true,
  "mission": {"state": "RUNNING", "index": 2, "count": 4, "dist_m": 37.4}
}
```

Pushed at the existing 5 Hz. `lat`/`lon` are `null` until the first
`GLOBAL_POSITION_INT`.

## Error handling

- **FLY gating** — disabled unless *all* of: `connected`, `lat`/`lon` non-null,
  `home_valid`, `mode == "OFFBOARD"`, and at least one waypoint. The button's
  title attribute names the first unmet condition, because two of these
  (`home_valid`, mode) fail *silently* inside PX4 and would otherwise present as
  "I pressed FLY and nothing happened".
- **No position yet** — the map shows "waiting for position" instead of
  centring on lat/lon 0,0 in the Gulf of Guinea.
- **Position lost mid-mission** — `advance()` returns `None` without a
  `lat`/`lon`, so the loop falls back to the velocity path and the drone hovers
  rather than steering toward a waypoint from an unknown position. The mission
  stays `RUNNING` and resumes navigating the moment position returns; it is not
  paused, because nothing about the operator's intent changed.
- **Mission continues without a browser** — mission state lives on the server,
  so a WS disconnect mid-mission does *not* stop the route. Deliberate: the
  input watchdog exists because a stuck manual key is a hazard, whereas an
  autonomous route completing unattended is correct. Reloading the page
  reconnects to a mission still in progress.
- **`state.clear()` on disconnect does not pause** — it sends no `axis`
  message, so it cannot latch a pause. This is what makes the point above work.
- **Mode drift mid-mission** — if PX4 leaves OFFBOARD for any reason, telemetry
  reports the real mode, the UI turns red, and the mission is auto-paused
  rather than left `RUNNING` while its setpoints are silently discarded.
- **Empty route** — `fly()` with no waypoints is a no-op; FLY is disabled
  anyway.
- **Setpoint continuity** — unchanged and still the load-bearing invariant. The
  velocity path remains the fallback on every tick where the mission is not
  running, so there is never a gap.
- **LAND and DISARM are never gated**, and both work mid-mission. LAND changes
  mode out of OFFBOARD, which auto-pauses the mission by the mode-drift rule.

## Isaac Sim side

**No changes.** `drone_setup_px4_cesium.py:669` already sets the PX4 GPS origin
from the Cesium georeference, which is the only thing this feature needs from
it. `GLOBAL_POSITION_INT` and `HOME_POSITION` are default streams on the
onboard link and require no configuration.

Wind remains disabled (`ADD_WIND = False`) for bring-up, for the same reason as
the joystick PoC. Re-enabling it afterwards is a good second demo: holding a
route through 5 m/s gusts is exactly what position setpoints do better than
velocity ones.

## Testing

**Unit** — `streaming/tests/test_waypoints.py`, pytest in the `drone` env, no
PX4 and no Isaac. Covers: `haversine_m` against known distances (1° of latitude
≈ 111.19 km; a short local leg), `bearing_deg` for the four cardinal directions
and a diagonal, arrival stepping the index at the radius boundary, `DONE` past
the last waypoint, empty-route `fly()` being a no-op, `pause()` from `RUNNING`
only, and `fly()` from `DONE` re-flying from waypoint 1.

The `advance()` return contract gets a test per row of its table, since it *is*
the dispatch rule: `None` in `IDLE` and `PAUSED`, `None` when `lat`/`lon` is
`None` in any state, the active waypoint in `RUNNING`, and — the one that would
otherwise regress silently — **the last waypoint still returned in `DONE`, with
a bearing that does not change once holding.**

**Unit** — extend `streaming/tests/test_offboard.py`: `POS_YAW_TYPE_MASK` and
`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` agree with pymavlink's dialect, and
`send_position_global` emits frame 6, mask 2552, and lat/lon scaled by 1e7.

**Loop** — extend `streaming/tests/test_offboard_loop.py` against the existing
fake PX4: a `RUNNING` mission emits `SET_POSITION_TARGET_GLOBAL_INT`; an
`axis pressed` message pauses it and the next tick emits
`SET_POSITION_TARGET_LOCAL_NED` instead; setpoint rate stays above 2 Hz across
the transition.

**Web** — extend `streaming/tests/test_web_ui.py`: map panel elements and the
four mission buttons exist, and the vendored Leaflet assets are present.

**Manual acceptance** — with Isaac Sim running: ARM → TAKEOFF → OFFBOARD, fly
manually to ~12 m, plan a 4-point route on the satellite map, press FLY, watch
the drone fly it on both map and camera, press FORWARD mid-leg and be flying
manually within a second with the route still drawn, then RESUME and watch it
finish and hold at the last waypoint.

## Dependencies

No new Python packages. `fastapi.staticfiles.StaticFiles` is already part of
FastAPI. Leaflet (~150 KB of JS + CSS) is vendored into `web/vendor/` and
committed.
