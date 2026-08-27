# Design: upload a control script to the website and fly it

**Status:** design under discussion — no implementation yet
**Date:** 2026-08-27
**Branch:** `feat/joystick-autopilot-poc`

## The idea

Add an **Agent** panel to the joystick website. The operator picks a Python
file, clicks **Upload**, then **Run**. The website arms, takes off, enters
OFFBOARD, and hands the aircraft to the uploaded script. The script is written
against the **competition control API** (`docs/competition-api.md`): a callback
class, not a `while` loop. First touch of the manual pad kills the script and
returns control to the operator.

The end state is a folder of small example scripts — one per flight-control
primitive — that the operator uploads to exercise each part of the API against
the running SITL.

### Why

`docs/competition-api.md` describes the API competitors will write to. This repo
is the R&D sandbox for that competition. Today the only way to fly a script here
is `autopilot_poc.py`, which replays browser button presses — the *opposite*
shape from the competition's inversion-of-control API. This feature makes the
website a practice environment: write a competition-style `Agent`, upload it,
watch it fly, iterate.

## Scope

**In:**

- `competition/` package: the `Agent` base class, `Command`, and the flight
  commands `Velocity`, `VelocityWorld`, `Goto`, `Route`, `Hold`.
- `on_frame(image, state)` fed a decoded camera frame; `Command(camera=...)`
  switches which of the sim's existing cameras feeds `on_frame` and the web
  video panel.
- `state` built from the telemetry the server already has.
- Callbacks: `on_start`, `on_frame` (~5 Hz), `on_tick` (~20 Hz), `on_arrival`,
  `on_waypoint`, `on_route_complete`. A slow callback is abandoned; the previous
  command holds.
- The agent runs in a **child process**. A crash or hang cannot take down
  `joystick-server.py`. Wall-clock kill switch on Run/Stop.
- Run auto-sequences ARM → TAKEOFF → wait for altitude → OFFBOARD.
- First manual axis press (or Stop, or child exit) kills the child.
- `examples/`: one minimal `Agent` per flight primitive, plus one camera example.

**Out (belongs to the "full competition sandbox", not this PoC):**

- `Inspect` — needs standoff-geometry solving.
- `request_roi` — needs a lossless crop path from Isaac.
- `pixel_to_ground` / `ground_to_pixel` / `is_visible` — need a terrain
  raycaster with live camera pose.
- `arena.lawnmower` — sweep-pattern generator.
- `self.submit` — no scoring in the sandbox.
- Any sandboxing beyond process isolation (rlimits, containers, restricted
  users). Matches the repo's existing "trusted LAN, no auth" stance
  (`RUN-WEBSITE.md` §11).

## Architecture

```
Browser ──/ws──►  joystick-server.py  ──queue──►  SetpointLoop  ──MAVLink──►  PX4
  Agent panel        POST /agent/upload            (owns conn)      14540
  Run / Stop         /ws  {type:agent}                  │
  log pane           /agent/control  ◄──────────┐       │  one setpoint/tick:
                          (WebSocket)           │       │  mission ▸ agent ▸ manual
                                                │       ▼
                          ┌─────────────────────┴──────────────┐
                          │ agent_runner.py  (child process)   │
                          │  imports the uploaded file         │
                          │  runs the harness loop:            │
                          │   on_start / on_frame / on_tick /  │
                          │   on_arrival / on_waypoint / ...    │
                          │  pulls JPEG frames from :8080       │
                          │  Command → JSON on /agent/control   │
                          │  stdout/stderr → server → log pane  │
                          └────────────────────────────────────┘
```

### `competition/` package

What uploaded scripts `import`. Pure Python, no MAVLink, independently testable.

| Module | Contents |
|---|---|
| `competition/__init__.py` | re-exports `Agent`, `Command`, `Velocity`, `VelocityWorld`, `Goto`, `Route`, `Hold` |
| `competition/commands.py` | the command dataclasses; frozen, validated in `__post_init__` |
| `competition/agent.py` | `Agent` base class — every callback a no-op returning `None` |
| `competition/state.py` | `State` and `Arena` value objects passed to callbacks |
| `competition/harness.py` | `Harness` — owns the loop, dispatches callbacks, enforces per-call timeouts, turns `Command`s into control-channel messages |

**Command shapes** (subset of `competition-api.md` §2.1, units unchanged):

```python
Velocity(forward=0.0, right=0.0, up=0.0, yaw_rate=0.0)     # body frame, m/s + deg/s
VelocityWorld(north=0.0, east=0.0, up=0.0, yaw_rate=0.0)   # world frame
Goto(lat, lon, alt, speed=None)                            # → on_arrival
Route(waypoints, alt, speed=None)                          # → on_waypoint / on_route_complete
Hold()                                                     # station-keep
Command(flight=None, camera=None)                          # camera ∈ {"nadir","oblique"}
```

`up` is **positive up in both frames** — matches `competition-api.md`, opposite
of the NED convention `offboard.py` uses internally. The translation to NED
happens server-side, in one place.

**`State`** — populated from the server's telemetry dict each tick:

| Field | Source |
|---|---|
| `lat`, `lon` | `telem["lat"]`, `telem["lon"]` |
| `alt_agl` | `telem["alt_m"]` (height above launch, which is AGL at this flat site) |
| `vz` | `telem["vz"]` (+up) |
| `ground_speed` | `telem["gs"]` |
| `heading` | `telem["heading_deg"]` |
| `camera` | tracked by the runner |
| `time_elapsed` | wall seconds since `on_start` |
| `time_remaining` | `arena.time_limit - time_elapsed`, or `None` if unlimited |
| `route` | `.state`, `.index`, `.count`, `.distance_m` from `telem["mission"]` |

`vx`/`vy` (world horizontal components), `roll`, `pitch`, `alt_amsl` are **not**
in the server telemetry today. Adding them is a small `SetpointLoop._drain_mavlink`
change (`ATTITUDE` for roll/pitch, `GLOBAL_POSITION_INT.vx/vy` for world
velocity). Include it — the competition `state` has them and example scripts
that check `state.pitch` before trusting a frame should work here too.

**`Arena`** — passed to `on_start`:

```python
arena.bounds        # (lat_min, lon_min, lat_max, lon_max) — a box around the
                    # site origin, ± AGENT_ARENA_RADIUS_M (default 500 m)
arena.time_limit    # seconds, or None. From --agent-time-limit (default: none)
```

No `lawnmower` (out of scope). The site origin comes from `sim/sites.py` via a
new `--site` flag on the server, defaulting to `bangkok-survey-040`.

### `Harness` — the loop

Runs inside `agent_runner.py`. One instance per run.

```
on_start(arena) ─────────────────────────► first Command (or None → Hold)
loop at 20 Hz:
    state = build_state(latest_telemetry)
    if a route/goto arrival was detected since last tick:
        dispatch on_arrival / on_waypoint / on_route_complete
    if 5 Hz tick due and a fresh frame is available:
        cmd = call_with_timeout(on_frame, 200 ms, image, state)
    else:
        cmd = call_with_timeout(on_tick, 50 ms, state)
    if cmd is not None:
        send(cmd) on /agent/control
    # None → send nothing; server holds the last agent command
```

- **Timeouts.** Each callback runs on a worker thread with a `join(budget)`. On
  overrun the result is dropped and the loop continues — the previous command
  stays in force server-side. This matches `competition-api.md` §1 ("call
  abandoned, previous command holds") without needing to kill the callback
  thread; a genuinely wedged callback is caught by the Run/Stop kill switch.
- **Events.** The harness watches `telem["mission"]`: `index` increasing →
  `on_waypoint(index)`; `state` going `RUNNING → DONE` → `on_route_complete`; a
  `Goto` (a one-point mission) reaching `DONE` → `on_arrival`. Synthesized from
  telemetry because `waypoints.Mission` exposes progress but not events.
- **Frames.** A background thread holds one MJPEG connection to the currently
  selected camera on `:8080`, decoding the latest JPEG to a numpy array
  (`PIL` + `numpy`, both in the `drone` env). `Command(camera=...)` tears down
  and reopens on the other endpoint. `on_frame` gets `None` until the first
  frame arrives.

### `agent_runner.py` — the child process

```
agent_runner.py --file <path> --host 127.0.0.1 --port 8090 \
                --control-path /agent/control [--time-limit N] [--arena-radius M]
```

1. `importlib` the uploaded file in its own module namespace.
2. Find exactly one `Agent` subclass. Zero or many → print an error, exit 2.
3. Open `ws://host:port/agent/control`. Read the `arena` bootstrap frame the
   server sends on connect (site origin, limits).
4. Run `Harness`. Telemetry arrives on the same socket (server pushes it).
5. On any unhandled exception in the agent: print the traceback, send a final
   `Hold`, exit 1.
6. `stdout`/`stderr` are line-buffered and inherited by the server, which
   forwards them to the browser log pane.

The child never touches MAVLink and never imports `pymavlink`. Its only
outward effect is JSON on the control socket.

### `joystick-server.py` changes

**New HTTP:**

- `POST /agent/upload` (multipart, one `.py` file) — reject non-`.py` and files
  over 256 KiB; save to `AGENT_UPLOAD_DIR` (a scratch dir, `--agent-dir`,
  default `<repo>/logs/agents/`) under a timestamped name; return
  `{"stored": "<name>"}`.
- `GET /agent/list` — the uploaded files, newest first, for a picker.

**New WebSocket `/agent/control`** — the child connects here.

- On connect the server sends `{"type":"arena", ...}` (site origin, bounds,
  time limit).
- The server pushes the same telemetry dict it pushes on `/ws`, at the same
  5 Hz, so the child needs only one socket.
- Inbound messages: `{"type":"velocity"|"velocity_world"|"goto"|"route"|"hold"|
  "camera", ...}`. The server applies each to `AgentControl` / the mission /
  the camera selection.

**`/ws` gains** `{"type":"agent","action":"run","file":"<name>"}` and
`{"type":"agent","action":"stop"}`:

- `run`: refuse unless `link` is up and no agent is already running. Set agent
  state `arming`. Enqueue `arm`, `takeoff`; when `alt_m` settles and
  `ready_for_offboard`, enqueue `offboard`; when `mode == OFFBOARD`, spawn
  `agent_runner.py` and set state `running`. This sequencing logic is a small
  state machine on the web-async side driven by the telemetry it already
  receives — it does not go in `SetpointLoop`.
- `stop` / first `axis` press with `pressed` / child process exit: SIGTERM the
  child (SIGKILL after 2 s), `agent_control.clear()`, mission `clear()` if the
  agent had loaded one, `state.clear()`, set agent state `stopped` (or `error`
  if the child exited non-zero).

**New `AgentControl`** (`streaming/agent_control.py`) — mirrors
`offboard.CommandState`:

- Thread-safe. The `/agent/control` handler (web thread) writes; `SetpointLoop`
  reads once per tick.
- Holds one of: a body velocity `(vx, vy, vz, yaw_rate)` in **NED** (translated
  on write from the API's +up), a world velocity, or `None`.
- 0.5 s watchdog on velocity commands, exactly like `CommandState` — a silent
  child decays to hover. `Goto`/`Route`/`Hold` are stateful (they go through
  the mission or a latched hold) and are not watchdogged.

**`SetpointLoop.run()` source priority** — one setpoint per tick, in order:

1. `mission.advance()` returns a target → fly it (covers both operator routes
   *and* agent `Route`/`Goto`).
2. else `agent_control.command()` returns a velocity → send it
   (`send_velocity` for body, a world-frame variant for `VelocityWorld`).
3. else `state.command()` → manual pad (unchanged).

Manual (3) is last, but a manual *press* also triggers `stop` up in the `/ws`
handler, so in practice the agent is already gone by the time its command would
lose. The ordering is the backstop, not the mechanism.

**`Hold`** from the agent = latch a zero body velocity in `AgentControl` with
the watchdog disabled (it is an explicit, stateful choice, not silence).

**Telemetry gains:**

```json
"agent": {
  "state": "idle|arming|running|stopped|error",
  "file": "<name or null>",
  "camera": "nadir|oblique|null",
  "log": ["last N lines from the child"]
}
```

**`VelocityWorld` server support:** `OffboardLink.send_velocity_world(vn, ve,
vu, yaw_rate)` — same `SET_POSITION_TARGET_LOCAL_NED` as `send_velocity` but
`MAV_FRAME_LOCAL_NED` (frame 1) instead of `MAV_FRAME_BODY_NED`, and `vu`
negated to NED-down. One small method next to the existing one.

### Web UI

New panel below `#cmds` in `index.html`:

```html
<div id="agent">
  <select id="a-file"></select>
  <input id="a-upload" type="file" accept=".py">
  <button id="a-run" disabled>RUN SCRIPT</button>
  <button id="a-stop" disabled>STOP</button>
  <span id="a-state">idle</span>
  <pre id="a-log"></pre>
</div>
```

`web/js/agent.js`:

- `POST` the chosen file to `/agent/upload`, refresh `#a-file` from
  `/agent/list`.
- `RUN` → `ws.send({type:"agent",action:"run",file})`; `STOP` →
  `{type:"agent",action:"stop"}`.
- From each telemetry frame: paint `#a-state`, append new `agent.log` lines to
  `#a-log`, and when `agent.camera` changes swap `#video` `src` to the matching
  endpoint (`nadir`→`/down`, `oblique`→`/detect`).
- `RUN` enabled when `link` up and `agent.state` in `{idle, stopped, error}`;
  `STOP` enabled when `arming` or `running`.

`main.js` wires the panel; `app.css` styles it (monospace log, fixed height,
scroll).

### `examples/`

One minimal `Agent` per flight primitive. Each is a complete uploadable file.

| File | Demonstrates |
|---|---|
| `examples/velocity.py` | `on_tick` returns `Velocity(forward=2.0)` every tick; stops (`Hold`) after `state.time_elapsed > 8` |
| `examples/velocity_world.py` | `VelocityWorld(north=2.0)` for a few seconds, then `Hold` |
| `examples/goto.py` | `on_start` returns `Goto` to a point 40 m north of start; `on_arrival` returns `Hold` and logs |
| `examples/route.py` | `on_start` returns `Route` over a 4-point box; logs each `on_waypoint` and `on_route_complete` |
| `examples/hold.py` | `on_start` returns `Hold`; does nothing else — the "does it just sit there" check |
| `examples/camera.py` | `on_start` selects `nadir`; every 5 s toggles `nadir`/`oblique` in `on_frame` and logs the frame shape |

These double as the manual test script for the feature and as the seed content
for the `#a-file` picker on a fresh box.

## Data flow — one run

1. Operator uploads `route.py`. Server stores it, returns the name.
2. Operator clicks **RUN SCRIPT**. `link` is up → server sets `agent.state =
   arming`, enqueues `arm` then `takeoff`.
3. Telemetry shows `armed`, then `alt_m ≈ 5`, `vz ≈ 0`, `ready_for_offboard`.
   Server enqueues `offboard`.
4. `mode == OFFBOARD` → server spawns `agent_runner.py --file <route.py>`.
   `agent.state = running`.
5. Child connects to `/agent/control`, gets the `arena` frame, calls
   `on_start(arena)` → returns `Route(...)`. Child sends `{type:"route",
   points:[...], alt:...}`.
6. Server `mission.load(...)`, `mission.fly()`. `SetpointLoop` flies the route
   (source 1).
7. Each waypoint: server telemetry `mission.index` ticks up → child synthesizes
   `on_waypoint(i)` → script logs a line → line appears in `#a-log`.
8. `mission.state → DONE` → child calls `on_route_complete` → script returns
   `Hold()` → child sends `{type:"hold"}` → server latches zero velocity, clears
   the mission.
9. Operator presses **W**. `/ws` `axis` handler SIGTERMs the child,
   `agent_control.clear()`, `state.clear()`. `agent.state = stopped`. Manual
   control is live.

## Error handling

| Failure | Behaviour |
|---|---|
| Uploaded file is not `.py` / too big | `POST /agent/upload` → 400, panel shows the message |
| File has no `Agent` subclass, or more than one | The import check runs in the child, which is spawned only after OFFBOARD — so the drone is already hovering. Child prints the error, exits 2. Server latches `Hold`, sets `agent.state = error`, leaves it hovering for the operator to take over or land. |
| Agent raises mid-flight | Child prints traceback, sends `Hold`, exits 1. Server latches the hold, `agent.state = error`, drone hovers. Operator takes over or lands. |
| Callback overruns its budget | Result dropped, previous command holds. Logged once per 5 s to avoid spam. |
| Child stops sending (wedged, killed) | `AgentControl` watchdog zeroes velocity after 0.5 s → hover. A latched `Hold`/`Route` persists (stateful). Server notices the child PID is gone within a tick → `stop` path. |
| `/agent/control` socket drops but child alive | Child retries connect 3×; then exits 1. |
| Isaac `:8080` frame stream unavailable | `on_frame` gets `None`; harness logs it once. Flight still works. |
| Operator clicks RUN with `link` down | Refused; panel says "no MAVLink link". |
| RUN while an agent is already running | Refused; `STOP` first. |
| Manual takeover mid-`Route` | `stop` path clears the mission too, so the route does not keep flying after the child is gone. |

## Testing

**`competition/tests/` — the package, no server, no Isaac:**

- `test_commands.py` — construction, validation (`Route` needs ≥1 waypoint,
  speeds non-negative), frozen-ness.
- `test_harness.py` — against a fake control channel and a scripted telemetry
  stream:
  - `on_start` result is sent as the first command.
  - `on_frame` called at ~5 Hz, `on_tick` at ~20 Hz, never both in one tick.
  - a callback that sleeps past its budget is abandoned; the previous command
    is what the channel last saw.
  - `on_frame` returning `None` sends nothing.
  - `mission.index` increments in telemetry → `on_waypoint` fires with the right
    index; `RUNNING→DONE` → `on_route_complete`.
  - `Command(camera=...)` emits a `camera` message and updates `state.camera`.
  - `Velocity(up=1.0)` reaches the channel as +up (the NED flip is server-side,
    not here).
- `test_state.py` — `State`/`Arena` built from a telemetry dict; `time_remaining`
  `None` when unlimited.

**`streaming/tests/` — server units, no Isaac:**

- `test_agent_control.py` — `AgentControl`: body/world velocity round-trips with
  the +up→NED flip; 0.5 s watchdog zeroes velocity; `Hold` is not watchdogged;
  `clear()` drops to `None`.
- `test_offboard_loop.py` (extend) — source priority: mission target beats agent
  velocity beats manual; agent velocity with a live watchdog is sent; expired
  agent velocity sends zero, not manual.
- `test_web_ui.py` (extend) — `POST /agent/upload` accepts a `.py`, rejects
  `.txt` and oversize; `/agent/list` returns newest first; `/ws`
  `{type:agent,action:run}` with `link` down is refused.

**Manual, against Isaac + PX4 (documented in `RUN-WEBSITE.md`):**

- Upload and run each of the six `examples/`; confirm the described motion and
  log output.
- Run `examples/route.py`, press **W** mid-route: child dies within ~0.5 s,
  route stops, manual works.
- Upload a file that raises in `on_tick`: drone hovers, `#a-log` shows the
  traceback, `agent.state = error`.
- Close the browser tab mid-run: the child keeps flying (server-side), reload
  reconnects to the running agent. (Matches the existing mission behaviour in
  `RUN-WEBSITE.md` §7.)

## Decisions

### AD1 — Child process, not a thread

A thread sharing `joystick-server.py`'s interpreter means an uploaded `while
True: pass` hangs the server and the setpoint loop with it, and `exec`'d code
can reach into the MAVLink connection. A child process is killable, has its own
GIL, and its only interface is a JSON socket. The cost — an IPC channel and
frame-fetch duplication — is small and buys a hard isolation line. (User choice.)

### AD2 — The child never speaks MAVLink

All flight goes: child → `/agent/control` → `AgentControl`/mission →
`SetpointLoop` → the one MAVLink connection. `SetpointLoop` stays the sole owner
of `conn` (its existing contract). The child cannot send a setpoint the server
did not translate and rate-limit.

### AD3 — Reuse `waypoints.Mission` for `Route` and `Goto`

`Route` is a mission; `Goto` is a one-point mission; both already fly
closed-loop on the server at 20 Hz, which is exactly what `competition-api.md`
§2.1 says `Route` should do ("runs entirely on the simulator host"). No new
path-following code. Arrival *events* are synthesized by the harness from
mission telemetry because `Mission` tracks progress but does not emit events —
cheaper than adding an event queue to `Mission` and its lock.

### AD4 — Run auto-sequences ARM → TAKEOFF → OFFBOARD

The competition harness assumes an airborne aircraft; `on_start` returns a
flight command immediately. Requiring the operator to pre-fly would mean every
run starts with four manual clicks and a "why is RUN greyed out" moment. The
sequencing is a small async state machine reading telemetry the server already
has — it does **not** go into `SetpointLoop`, which stays a dumb setpoint pump.
(User choice.)

### AD5 — First manual input kills the agent; no resume

An agent is a program you abort, not a route you pause. "Pause and resume" would
mean freezing a running callback loop and its internal state consistently —
fragile, and not what the competition does either. Stop is a clean kill; run it
again to restart from `on_start`. (User choice.)

### AD6 — `+up` in the API, NED inside; flip in exactly one place

`competition-api.md` is emphatic that `up` is positive in both frames ("the sign
flip is a reliable source of bugs"). `offboard.py` is NED internally. The
conversion lives in `AgentControl` on write (and `send_velocity_world`), nowhere
else. The `competition/` package and every example script only ever see +up.

### AD7 — No sweep generator, no perception helpers, no submit

`arena.lawnmower`, `pixel_to_ground`, `request_roi`, `Inspect`, `self.submit`
are the "full competition sandbox" — each needs Isaac-side work (lossless crop
path, terrain raycaster, standoff solver) that dwarfs this feature. Excluded
now; the package layout leaves room to add them later without changing the
`Agent`/`Command` surface. (User choice: "flight + camera image".)

### AD8 — One camera at a time, mapped to existing feeds

The sim already serves `/down` (nadir), `/detect` (forward gimbal), `/chase`.
The API exposes `nadir` → `/down` and `oblique` → `/detect`, matching the
competition's two-camera model. No new render products. `Command(camera=...)`
switches which stream the harness decodes for `on_frame` and which the web
`#video` shows.

## Files

**New:**

- `competition/__init__.py`, `competition/commands.py`, `competition/agent.py`,
  `competition/state.py`, `competition/harness.py`
- `competition/tests/test_commands.py`, `test_harness.py`, `test_state.py`
- `agent_runner.py`
- `streaming/agent_control.py`
- `streaming/tests/test_agent_control.py`
- `web/js/agent.js`
- `examples/velocity.py`, `velocity_world.py`, `goto.py`, `route.py`, `hold.py`,
  `camera.py`

**Edited:**

- `joystick-server.py` — upload/list routes, `/agent/control` WebSocket, `/ws`
  agent actions, run state machine, telemetry `agent` block, `--site` /
  `--agent-dir` / `--agent-time-limit` / `--agent-arena-radius` flags
- `streaming/offboard.py` — `send_velocity_world`; `ATTITUDE` (roll/pitch) and
  world velocity into `_drain_mavlink` / telemetry
- `streaming/waypoints.py` — no change expected; note if `advance` needs a
  distinct-source hint
- `web/index.html`, `web/js/main.js`, `web/css/app.css` — the panel
- `streaming/tests/test_offboard_loop.py`, `test_web_ui.py` — extend
- `RUN-WEBSITE.md` — a new section for the Agent panel and the manual test list
- `SESSION.md` — how to run it

## Open questions

None blocking. `--agent-time-limit` defaults to unlimited; revisit if example
scripts want a clock to read.
