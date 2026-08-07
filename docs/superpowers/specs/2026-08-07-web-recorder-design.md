# Design: record VIO datasets from the web UI

Add a RECORD button to the flight page that starts and stops
`vio-recorder-pai.py` inside Isaac Sim, writing a full dataset to
`~/vio_dataset/<timestamp>/`, with live status on the page.

Sequenced **before** `2026-08-07-vio-gps-denied-design.md`: that plan's estimator
is developed against recorded datasets (its Task 1 Step 1.3 needs an AHRS
baseline, its Task 5 Step 5.2 replays a dataset offline), so being able to capture
one without hand-driving the Script Editor comes first.

## Why this is not just a button

Two facts about the current setup, both measured rather than assumed:

**The recorder already produces silent failures.** The most recent run,
`~/vio_dataset/20260802_151016/`, contains `imu.csv`, `poses.csv`, `frames.csv`
and `baro.csv` at **zero bytes** and **zero images** — only a directory and a
`cam_calib.json`. That is the takeoff gate doing exactly what it was written to
do (`vio-recorder-pai.py:361-372`): it armed, the drone never climbed 0.5 m, and
nothing was written. Nothing in the console distinguishes "armed, waiting" from
"recording" at a glance, so the run looked fine until someone opened it.

**The disk cannot absorb many more runs.** 1.8 T total with **180 G free (90%
used)**; `~/vio_dataset` is **217 G** over 24 runs; typical runs are **22–30 G**.
That is roughly **seven** more recordings. A one-click recorder without a guard
turns a full disk into a routine outcome — and a disk that fills mid-flight
corrupts the run *and* destabilises the sim and PX4 with it.

So the design is really: a button, a truthful status readout, and a guard.

## Architecture

```
browser                joystick-server.py            Isaac Sim (Kit)
┌──────────┐  WS /ws   ┌──────────────────┐  HTTP    ┌─────────────────────────┐
│ RECORD   │──────────▶│  :8090           │─────────▶│ sim/recorder_control.py │
│ status   │◀──────────│  proxy + 1 Hz    │  :8091   │  HTTP thread: ENQUEUE   │
└──────────┘ telemetry │  status poll     │◀─────────│  main thread: EXEC      │
                       └──────────────────┘  JSON    │    vio-recorder-pai.py  │
                                                     │      ↓                  │
                                                     │  ~/vio_dataset/<ts>/    │
                                                     └─────────────────────────┘
```

Three processes because the boundary is forced: the recorder needs Pegasus sensor
callbacks and the replicator annotators, which exist only inside Kit; the web
server lives in the `drone` conda env and cannot reach them.

## Decisions

### R1 — The HTTP thread enqueues; the main thread executes

`ThreadingHTTPServer` hands each request to a worker thread. That thread must
**never** exec the recorder: doing so installs a physics callback and touches USD
and replicator from a non-main thread, which Kit does not support.

So the handler validates, enqueues a command, and returns. A callback on
`omni.kit.app.get_app().get_update_event_stream()` drains the queue on the main
thread and does the work.

This is the same discipline `joystick-server.py` already uses for its own
cross-thread commands — the web thread only ever calls `submit()`, which puts to a
queue the setpoint thread drains (`joystick-server.py:139-151`). Reusing the shape
means one concurrency model to reason about, not two.

Serving HTTP from inside Kit is likewise not new here: `drone_setup_px4_cesium.py:537`
already runs a `ThreadingHTTPServer` for the MJPEG camera feed on 8080. Port 8091
(8080 = video, 8090 = web UI), overridable with `SITL_RECORDER_PORT`.

### R2 — The proxy runs on the web thread, NOT the setpoint queue

Every existing command from the page (`arm`, `takeoff`, `offboard`, `land`,
mission verbs) is routed through `SetpointLoop.submit()` onto the setpoint thread.
That exists for one reason: those commands touch the MAVLink connection, and
pymavlink connections are not thread-safe (`streaming/offboard.py:196-198`).

**Recording touches no MAVLink at all**, so it must not take that path. An HTTP
request to Kit can block for hundreds of milliseconds — Kit's main thread may be
mid-frame — and the setpoint thread cannot afford that: a gap in the setpoint
stream drops PX4 out of OFFBOARD (`joystick-server.py:216-219`). Routing a
recording command through the setpoint queue would mean **pressing RECORD could
drop the aircraft out of offboard control.**

The record commands are therefore handled directly in the async WebSocket handler
and never enter `submit()`.

### R3 — stdlib HTTP, no new dependency

The `drone` env has no `httpx`, `aiohttp` or `requests`. Rather than add one for
three small calls, use `urllib.request` wrapped in `asyncio.to_thread()` — async
where it must be, no dependency, short timeouts.

### R4 — The recorder is exec'd unmodified

`vio-recorder-pai.py` stays byte-identical to `~/pai/drone-vio/vio-recorder-pai.py`.
The control server reads its source and `exec(compile(...), ns)` into a namespace
it keeps — exactly how `bootstrap.py:90-98` runs the setup script.

Everything needed is reachable through that namespace afterwards, because the
recorder's `_on_phys` closure resolves module globals **dynamically**:

- `ns["TAKEOFF_ALT_M"] = 0.0` after exec disables the takeoff gate (R5). Verified:
  a function exec'd into a namespace sees later mutations of that namespace.
- `ns["st"]` is the live counters dict (`vio-recorder-pai.py:340-341`) —
  `frame`, `n_frame`, `dropped`, `img_every`, `armed`.
- `ns["imgq"]` gives writer-queue depth; `ns["_VIO_REC"]` gives `dir` and the file
  handles the stop sequence needs.

Stop is the sequence the recorder itself documents (`vio-recorder-pai.py:460-461`):
put the sentinel, remove the `vio_rec` physics callback, close the files.

Re-exec is already a safe restart: the recorder cleans up a previous run at
`:159-169`.

### R5 — RECORD records; it does not arm

`ns["TAKEOFF_ALT_M"] = 0.0`, so data flows the moment the button is pressed.

The button *is* the trigger; a second, invisible condition on top of it is what
produced the zero-byte run. The cost is a few seconds of ground frames at the head
of each dataset, which is harmless — `flow_odometry.run()` already takes
`skip_frames` (`flow_odometry.py:334-338`) and `pipeline.py` the same.

The gate's other failure mode disappears with it: pressing RECORD while already
airborne re-baselines `ground_z` to the current altitude
(`vio-recorder-pai.py:363-364`), so recording would not start until the drone
climbed a *further* 0.5 m.

### R6 — Free space is a gate, not a warning

- **< 50 G free: RECORD is disabled**, with the reason on the page. Two runs of
  headroom, so a flight cannot start only to die halfway and leave a truncated
  dataset.
- **< 100 G free: amber warning**, recording still allowed.
- Free space and the live size of the current run are both shown.

Checked at start time in the control server (authoritative — it is on the machine
doing the writing) and surfaced through status so the button can be disabled
before it is pressed rather than erroring after.

### R7 — Status reports whether data is actually landing

The zero-byte run is the requirement here. Status carries counters, not just a
state name:

```json
{
  "state": "offline|idle|starting|recording|error",
  "run_dir": "/home/innovation/vio_dataset/20260807_143000",
  "elapsed_s": 12.4,
  "frames": 2480,
  "images": 186,
  "dropped": 0,
  "queue": 3,
  "bytes": 18234567,
  "cameras": ["cam0"],
  "free_bytes": 193273528320,
  "error": null
}
```

The page derives a **stalled** condition: `state == "recording"` but `images` has
not advanced for >3 s. That is the honest signal — "recording" alone is what lied
last time.

`bytes` is the run directory's size, sampled on the status tick rather than
accumulated, so it stays true even if a writer thread dies.

## Components

### `sim/recorder_control.py` (new)

Kit-side. Owns the HTTP server, the command queue, the update-stream callback, the
recorder namespace, and the state machine.

| endpoint | method | behaviour |
|---|---|---|
| `/record/start` | POST | 409 if already recording; 507 if below the free-space floor; 503 if the timeline is not playing; else enqueue and return `202` |
| `/record/stop` | POST | idempotent — 200 whether or not it was recording |
| `/record/status` | GET | the JSON above; always 200 so the page can tell "offline" from "broken" |

Start must verify the recorder actually installed itself: if `ns.get("_VIO_REC")`
is absent after exec, the recorder printed its own diagnostic (no drone, no IMU,
no `down_cam` — `vio-recorder-pai.py:147-152`) and did nothing. That is an
`error` state with the reason, not a success.

Recording requires the timeline to be playing — Pegasus only streams sensor data
from the play event (`sim/bootstrap.py:192-195`), and physics callbacks do not
fire otherwise.

### `sim/bootstrap.py` (modified)

Start the control server after the setup script, before/around Play. Advisory like
every other post-setup step (`bootstrap.py:108-124`): a recorder server that fails
to bind must not cost you the sim.

### `joystick-server.py` (modified)

- `--recorder-url`, default `http://127.0.0.1:8091`.
- A 1 Hz background task polling `/record/status` into a cached dict; folded into
  the existing 5 Hz telemetry push as a `rec` block. Polling Kit at telemetry rate
  would be wasteful and the numbers do not change that fast.
- WS handling for `{type: "record", action: "start"|"stop"}`, handled inline (R2).
- Unreachable control server ⇒ `state: "offline"`, RECORD disabled. Normal before
  Isaac is up; it is a state, not an error.

### `web/` (modified)

A RECORD button and a status line near the existing command row
(`web/index.html:56-62`), painted by `web/js/telemetry.js` in the established
`good`/`wait`/`bad` vocabulary (`telemetry.js:84-105`).

Shown while recording: elapsed, images, size, free space. Red on `stalled`,
`dropped > 0`, or low disk. Disabled with a reason when offline or below the floor.

## Error handling

| condition | behaviour |
|---|---|
| control server unreachable | `offline`; button disabled; not an error |
| timeline stopped | 503 with reason; button disabled |
| no drone / IMU / `down_cam` | `error` + the recorder's own reason |
| free space below floor | 507; button disabled with the number shown |
| start while recording | 409; no second directory created |
| stop while idle | 200, no-op |
| writer queue saturating | `dropped` climbs; page turns red (already counted at `vio-recorder-pai.py:431`) |
| Kit dies mid-recording | poll fails ⇒ `offline`; the partial dataset stays on disk |

## Testing

### Automated — `conda run -n drone pytest sim/tests/ streaming/tests/ -q`

`sim/tests/` already exists with `test_sites.py`. None of the below needs Isaac:
the control module's logic is separated from its Kit bindings, and the recorder
namespace is a plain dict in tests.

- State machine: every transition, including start-while-recording and
  stop-while-idle.
- Free-space gate: refuses below the floor, allows above, and the threshold is
  actually applied to the dataset root rather than the CWD.
- Start with a namespace missing `_VIO_REC` ⇒ `error`, not `recording`.
- Status shaping from a fake `st`/`imgq`/`_VIO_REC`.
- HTTP layer against a fake queue: correct codes for 409 / 503 / 507.
- **The HTTP handler must not execute anything** — assert that a start request
  leaves the recorder unexec'd until the update callback runs (R1).
- `joystick-server` proxy against a fake control server: offline handling,
  timeouts, and that record commands never enter `submit()` (R2).
- Web UI in the existing `streaming/tests/test_web_ui.py` style.

### Manual

1. `./sim/launch-sitl.sh`, then `curl -X POST localhost:8091/record/start` —
   proves the Kit side alone, before any UI exists.
2. `curl localhost:8091/record/status` — counters advancing.
3. Press RECORD on the page; confirm images land in `~/vio_dataset/<ts>/`.
4. Fly a short flight; STOP; confirm the dataset loads:
   `python -c "from flow_odometry import load_dataset; print(len(load_dataset('<dir>')[2]))"`.
   **This is the real acceptance bar** — a dataset the pipeline cannot load is not
   a recording.
5. Stop Isaac mid-recording; confirm the page shows `offline` and the partial run
   survives.

## Out of scope

- Recording without Isaac (from MJPEG + MAVLink). It would produce a
  different, worse dataset — 640×400 stream frames instead of 960×600 VIO
  captures, and EKF output instead of ground-truth poses.
- Dataset browsing, playback, or deletion from the page. The disk needs pruning,
  but that is a separate tool and a destructive one.
- Any change to `vio-recorder-pai.py` (R4).
- `cam1`/fpv recording — the recorder already handles it when the camera is on the
  stage; nothing here needs to know.
- Compression, retention policy, or automatic pruning.
