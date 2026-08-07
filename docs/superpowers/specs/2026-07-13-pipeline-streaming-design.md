# Design: streaming VIO pipeline → PX4/QGC (Isaac Sim)

> **Architecture superseded (2026-08-07)** by `2026-08-07-vio-gps-denied-design.md`,
> which keeps this document's goal, scope and error-handling analysis but revises
> how the estimate reaches PX4 (via `joystick-server.py`, the sole MAVLink owner)
> and corrects `EKF2_HGT_REF=EV` — flow-odom takes altitude from the barometer, so
> vision must not also claim to supply it.

## Goal

Feed `pipeline.py`'s GPS-denied position estimate into PX4 SITL (spawned by
`drone_setup_px4_cesium-pai.py` in Isaac Sim) as an external vision source, so
QGroundControl can arm and fly the drone using vision-only positioning
(EKF2 fusing `VISION_POSITION_ESTIMATE`, GPS fusion disabled) instead of the
simulator's ground-truth GPS.

`pipeline.py` today is strictly offline/batch: it loads a fully-recorded
dataset directory (`frames.csv`, `imu.csv`, `poses.csv`, ...) and iterates a
pre-loaded `recs` list, using GT only for final scoring. This design adds a
live-streaming counterpart that consumes sensor data as it's produced inside
Isaac Sim and produces a position estimate frame-by-frame, in real time.

## Scope (v1)

- **Flow-odom only.** LK optical-flow odometry (the batch pipeline's Layer 1)
  ported to consume one frame at a time. SIFT+LightGlue DSMAC and the
  relief-fix (Layer 2, Exp10) are explicitly **out of scope for v1** — they're
  a fast-follow once the base streaming + MAVLink loop is validated live.
- **No rangefinder/AGL sensor on this rig.** Depth falls back to baro-derived
  height above takeoff, the same fallback path `pipeline.py` already has for
  datasets without `lidar.csv`. This is a known accuracy ceiling (see
  `frontend/CLAUDE.md`'s AGL findings — baro-only depth is ~4× scale error at
  altitude), carried into v1 as-is and not re-litigated here.
- **No GT.** The streaming pipeline has no ground truth available live;
  initial position comes from a CLI-provided takeoff lat/lon, not `recs[0]["gt"]`.

## Architecture

```
Isaac Sim process                          drone conda env (external process)
┌─────────────────────────┐                ┌──────────────────────────────────┐
│ vio-streamer-pai.py      │  ZMQ PUB/SUB   │ pipeline-streaming.py            │
│  - Pegasus IMU callback  │───tcp:5556────▶│  - MahonyState (incremental AHRS)│
│  - Pegasus Baro callback │  topics:       │  - LK flow-odom step (reused     │
│  - down_cam capture      │   imu, baro,   │    from pipeline.py functions)   │
│    (JPEG bytes inline,   │   frame        │  - pos integration (ENU)         │
│    no disk write)        │                │  - MAVLink bridge ─────┐         │
└─────────────────────────┘                └─────────────────────────┼─────────┘
                                                                       │ UDP
                                                          VISION_POSITION_ESTIMATE
                                                                       │
                                                             ┌─────────▼─────────┐
                                                             │ PX4 SITL (Pegasus) │
                                                             │  EKF2_EV_CTRL on   │
                                                             │  EKF2_GPS_CTRL off │
                                                             └─────────┬─────────┘
                                                                       │ MAVLink
                                                                 ┌─────▼─────┐
                                                                 │    QGC    │
                                                                 └───────────┘
```

Two separate processes, chosen deliberately:

1. **Isaac Sim's embedded Python** (running `vio-streamer-pai.py` in the
   Script Editor) has zero-copy access to Pegasus sensor callbacks but is not
   guaranteed to have `torch`/`lightglue` installed, and heavy matching work
   there risks stalling the sim's render/physics loop.
2. **A standalone external process** (`pipeline-streaming.py`, `conda run -n
   drone`) reuses the exact stack `pipeline.py` already runs on, isolated from
   Isaac Sim's process.

They're bridged over **ZeroMQ PUB/SUB on localhost TCP** — low latency,
decouples producer/consumer rates, trivial to add more subscribers later
(e.g. a live plotter).

`vio-streamer-pai.py` is a **new, separate script** from `vio-recorder-pai.py`
(not a modification of it) — recording-to-disk and live-streaming stay fully
decoupled; each independently hooks the same Pegasus sensor callbacks.

## Components

### 1. `vio-streamer-pai.py` (new, Isaac Sim Script Editor)

Mirrors `vio-recorder-pai.py`'s sensor wiring (Pegasus `IMU`/`Barometer`
callbacks, down_cam capture) but instead of writing CSV/PNG to disk, ZMQ-PUBs
each sample as msgpack:

| topic | rate | payload |
|---|---|---|
| `imu`   | ~200 Hz  | `ts_ns, wx,wy,wz, ax,ay,az` |
| `baro`  | ~200 Hz  | `ts_ns, pressure_altitude_m` |
| `frame` | ~12.5 Hz | `frame_idx, ts_ns, jpg_bytes` (JPEG-encoded inline, no disk write) |

No AGL/rangefinder topic in v1 (see Scope).

### 2. `pipeline-streaming.py` (new, external process, `conda run -n drone`)

ZMQ SUB consumer:

- **`MahonyState`** class — the per-tick math already inside
  `compute_ahrs_attitude` (`frontend/flow-odom/flow_odometry.py:115`),
  extracted into an incremental object (`update(gyro, acc, mag, dt) -> R`)
  instead of the batch-over-`rows` loop it's currently locked into. This is a
  refactor of existing, already-validated math, not new logic — the unit test
  in the plan asserts it reproduces the batch function's output bit-for-bit.
- On each `frame` message: decode JPEG, run `_detect`/`_track_lk`/
  `_solve_translation` (imported directly from `pipeline.py`, unchanged)
  between this and the last *kept* frame, respecting `--stride` by
  subsampling the incoming stream rather than indexing a pre-loaded list.
- Integrates `dC` into a running ENU `pos`, initialized from `--start_lat`/
  `--start_lon` CLI args.
- Every step, converts `pos` + yaw to NED and sends `VISION_POSITION_ESTIMATE`
  over a dedicated pymavlink UDP connection to PX4's companion port.

### 3. MAVLink bridge (module inside `pipeline-streaming.py`)

`mavutil.mavlink_connection('udpout:127.0.0.1:14540')` — PX4's onboard/
companion endpoint, distinct from QGC's own GCS link (typically 14550) and
from `PX4MavlinkBackendConfig`'s `connection_baseport=4560`, which is the
*simulator*-interface port Pegasus itself uses, not this one. The plan
includes a step to confirm the actual companion port against Pegasus's PX4
SITL startup config before wiring it in.

ENU→NED conversion (`x_ned = N, y_ned = E, z_ned = -Up`), then
`vision_position_estimate_send(usec, x, y, z, roll, pitch, yaw)`.

### 4. PX4 param doc

A short params list/file: `EKF2_EV_CTRL` bits (horizontal position, vertical
position, yaw fusion), `EKF2_GPS_CTRL=0`, `EKF2_HGT_REF=EV`. Loaded once via
QGC's parameter editor so EKF2 actually trusts vision over Pegasus's
simulated GPS.

## Data flow / error handling

- **Dropped/out-of-order camera frames:** `pipeline-streaming.py` tracks the
  last-processed `frame_idx`; a gap just means a longer baseline for the next
  LK step (mirrors how `pipeline.py` already tolerates variable strides) — no
  special recovery needed.
- **ZMQ SUB disconnect / Isaac Sim restart:** reconnect loop with backoff; on
  reconnect, reset the flow-odom `prev` frame and `MahonyState` from the next
  `imu`/`frame` pair seen — no persisted state across a sim restart (matches
  the batch pipeline's behavior of starting fresh per run).
- **MAVLink send failures (PX4 not up yet):** skip the send and log; no
  buffering, since EKF2 has no use for a stale position anyway. Resume once
  PX4 accepts packets.

## Testing

### Automated (`conda run -n drone pytest`)

- `MahonyState` incremental output matches `compute_ahrs_attitude`'s batch
  output bit-for-bit on a recorded dataset's `imu.csv` fixture.
- ZMQ msgpack framing round-trip for all three topics (`imu`, `baro`, `frame`).
- ENU→NED conversion correctness against known vectors.
- `VISION_POSITION_ESTIMATE` packing against a mocked `mavutil` connection
  (assert fields/units, e.g. NED axes and radian yaw).

### Manual validation checklist (live Isaac Sim + QGC)

**1. Start the sim environment**
- Open Isaac Sim with the Cesium stage loaded, sim **stopped**.
- Run `drone_setup_px4_cesium-pai.py` in the Script Editor.
- ✅ Console prints `>>> PX4 drone spawned` and `>>> Done. Press Play, then connect QGC.`

**2. Press Play, connect QGC normally (baseline)**
- Press Play. Open QGC, let it connect to PX4 over its usual MAVLink link.
- ✅ QGC shows the drone at the expected lat/lon, GPS-locked, arms normally —
  isolates any later failure to the new streaming path, not a regression in
  the base setup.

**3. Launch the ZMQ publisher**
- Run `vio-streamer-pai.py` in the Script Editor.
- ✅ Console shows ZMQ PUB bound (e.g. `tcp://*:5556`) and per-topic tick
  counters incrementing (`imu`, `baro`, `frame`).

**4. Launch the streaming pipeline**
- `conda run -n drone python pipeline-streaming.py --start_lat <lat> --start_lon <lon>`
- ✅ Logs "ZMQ connected", then per-step flow-odom output (drift, LK inlier
  count) at roughly camera rate ÷ stride.
- ✅ Logs MAVLink connect to the companion UDP port and a running
  `VISION_POSITION_ESTIMATE` send counter/rate.

**5. Confirm PX4 is actually fusing vision, not ignoring it**
- Load the EKF2 params (component 4) via QGC's parameter editor, reboot PX4.
- In QGC's MAVLink Inspector, watch `ESTIMATOR_STATUS` / `LOCAL_POSITION_NED`.
- ✅ Position matches the streamed vision estimate, not sim ground truth —
  nudge/fly the drone and confirm the QGC map position tracks the *vision*
  estimate, including its drift, not GT.

**6. Fly it**
- Arm and take off in QGC using vision-only positioning (GPS fusion disabled).
- ✅ Holds position/altitude reasonably and responds to stick/waypoint
  commands — the real pass/fail bar; a jumpy or diverging estimate shows up
  immediately as oscillation or drift-away in manual/position-hold mode.

**7. Failure-mode checks**
- Stop `pipeline-streaming.py` mid-flight — ✅ PX4/QGC reports a stale/lost
  vision source gracefully (failsafe) rather than lurching from a stuck old
  estimate.
- Restart it — ✅ reconnects and resumes sending without restarting Isaac Sim
  or PX4.

## Explicitly out of scope (v1)

- DSMAC / relief-fix corrections (fast-follow).
- Real rangefinder/AGL sensor integration (depends on hardware/sim sensor
  availability — separate effort per `project-next-recording.md` memory).
- Persisting/recovering estimator state across a streamer or PX4 restart.
- Automated end-to-end (Isaac Sim + PX4 + QGC) test — validated manually per
  the checklist above.
- Any drone-commanded autonomy (v1 is estimator-only: it feeds PX4 a position,
  it never issues a flight command). See v2 below.

## V2 (future): autonomous waypoint flight + adaptive altitude search

v1 only makes `pipeline-streaming.py` a passive sensor: it estimates position
and hands it to EKF2, but PX4/QGC still decides where the drone goes. V2
changes that — the streaming process also becomes a **mission commander**,
issuing flight commands over the same MAVLink companion link it already uses
for `VISION_POSITION_ESTIMATE`. This is a materially different risk profile
(a bug here now moves the aircraft, not just corrupts a position estimate),
so it's staged as two sub-phases, each independently shippable and
validated live before moving to the next:

### V2a — fly-to-lat/lon (build first)

- User supplies a target lat/lon (CLI arg or a simple runtime input).
- `pipeline-streaming.py` sends `SET_POSITION_TARGET_GLOBAL_INT` (or
  `MAV_CMD_NAV_WAYPOINT` in guided mode) over the existing companion
  connection once PX4 is armed and vision-fusing (the v1 loop already
  running).
- No new estimator logic — this is a thin addition: one outbound command
  type on a connection that already exists. Requires PX4 to be in a mode
  that accepts companion-computer setpoints (guided/offboard) — a manual
  checklist step to confirm which mode QGC needs to be left in before this
  will actually move the drone, distinct from v1's manual checklist (which
  never commands, only estimates).
- Safety bound: PX4/QGC's own geofence and failsafe config are the only
  bounds until v2b adds any pipeline-side ones — no new pipeline-side limits
  needed for straight-line goto, since PX4's own navigation already handles
  the flight path.

### V2b — climb-and-search on repeated DSMAC failure (after V2a is flying)

- Requires DSMAC (not just flow-odom) to already be ported into the
  streaming pipeline — i.e., this depends on the "DSMAC / relief-fix
  corrections" fast-follow listed above, not on v1 alone.
- `pipeline-streaming.py` tracks consecutive DSMAC-fix rejections (the
  batch pipeline already counts attempts/accepts — same counter, reused).
  When consecutive rejects cross a threshold, it issues an altitude-adjust
  setpoint (climb), re-attempts DSMAC at the new altitude on the next fix
  cadence, and stops climbing once a fix clears the confidence gate.
- Needs pipeline-side safety limits independent of whatever PX4/QGC already
  enforce: a max climb rate, a hard altitude ceiling, and a "give up, hold
  the last accepted fix, stop climbing" fallback — otherwise a genuinely
  featureless area (e.g. Exp10's ds5, which had almost no exploitable
  relief signal at any altitude tried) would have the drone climb
  indefinitely chasing a fix that will never come.
- Open question carried over from Exp10, relevant here: whether climbing
  actually helps DSMAC's match rate on canopy/repetitive terrain, or only
  changes scale/GSD without fixing the underlying "no correspondable
  structure" problem Exp08 found — this needs its own small validation pass
  (does accept rate vs. altitude actually trend up on a real failure case?)
  before building the closed-loop climb behavior, not assumed from the
  design.

## Addendum (2026-07-14): initial state, origin, and V2a mechanics

### Initial state / origin (applies to v1 and up)

`VISION_POSITION_ESTIMATE` carries **only local NED offsets** from the vision
origin — no lat/lon. With GPS off, PX4 has no absolute anchor unless one is given.
So:

- The user inputs initial **position** via `--start_lat/--start_lon/--start_alt`
  (v1's "log-only" note is superseded — these are the one georef anchor once
  commanding exists). Initial **heading** must also be provided (`--start_yaw` or
  `takeoff.json`), because the estimator's initial attitude was otherwise a
  degenerate identity seed.
- Push the anchor to PX4 once with `SET_GPS_GLOBAL_ORIGIN(lat, lon, alt)`. Benefits:
  PX4 can map global↔local, the QGC map places the drone, and targets may be sent
  as local NED *or* lat/lon interchangeably.
- The anchor must be consistent across three things or the drone flies to the
  wrong place: the `SET_GPS_GLOBAL_ORIGIN` value, the ENU origin the estimate
  drifts from, and the origin targets are converted against.

### V2a mechanics (concrete)

- PX4 flies the path; the pipeline only names a destination — no pipeline-side
  guidance. PX4's geofence/failsafe are the only bounds.
- Message: `SET_POSITION_TARGET_LOCAL_NED` with a type_mask using x/y/z (+yaw),
  ignoring velocity/accel/yaw_rate. Sent **every loop** (OFFBOARD needs >2 Hz).
- `DO_REPOSITION` is a single-shot alternative that skips the offboard heartbeat,
  but does NOT extend to V2b's continuous altitude commanding — use OFFBOARD
  streaming if V2b is the goal.
- Mode switch to OFFBOARD is a safety boundary: operator flips it in QGC for first
  bring-up; auto `MAV_CMD_DO_SET_MODE` only after V2b's safety bounds exist.

### Middleware decision

Stay on **pymavlink** (matches the non-ROS stack, works on Windows). Revisit
**PX4-ROS2 / uXRCE-DDS** (not MAVROS) only if the project moves to Ubuntu *and*
grows into a multi-node autonomy stack.
