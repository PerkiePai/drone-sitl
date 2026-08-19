# Design: GPS-denied flight on pipeline VIO, flown from the web UI

Supersedes the architecture of `2026-07-13-pipeline-streaming-design.md` (and its
2026-07-14 addendum). The **goal** is unchanged — fly the SITL drone positioned by
this repo's own flow-odometry instead of the simulator's GPS. What changed is
everything the estimate has to plug into: that spec assumed QGroundControl was the
cockpit and that `pipeline-streaming.py` would own its own MAVLink link to PX4.
Neither is true any more.

## Why this revision exists

Five things landed between 2026-07-13 and today that invalidate parts of the old
plan:

1. **`joystick-server.py` is the cockpit and owns MAVLink.** It binds
   `udpin:0.0.0.0:14540` and its `SetpointLoop` is documented as *"Sole owner of
   the MAVLink connection"* (`joystick-server.py:34-41`), a contract restated on
   `OffboardLink` (`streaming/offboard.py:196-198`). The old plan's
   `udpout:127.0.0.1:14540` collides with that bind.
2. **The web UI already commands the aircraft** — arm/takeoff/offboard/land, a
   Leaflet map, waypoint routes, 5 Hz telemetry. The old spec staged this as
   future work "V2a"; it shipped.
3. **`sim/` replaced the Script Editor.** `launch-sitl.sh` → `bootstrap.py --exec`
   authors the stage and presses Play. Nothing is pasted by hand.
4. **`sim/sites.py` holds the georeference truth** — lat/lon/height, spawn, heading
   (`sim/sites.py:96-118`). The old plan's `--start_lat/--start_lon/--start_yaw`
   CLI args would be a second, drift-prone copy of it.
5. **All dependencies are already installed** in the `drone` env: `zmq 27.1.0`,
   `msgpack 1.2.1`, `cv2 4.13.0`, `scipy 1.17.1`, `pymavlink 2.4.49`, `pytest`.
   The old plan's Task 0 is a no-op.

## The camera-mount problem (found during design; drives D3)

`down_cam` is **not rigidly attached to the airframe**. It hangs off
`/World/down_mount`, a top-level Xform re-driven on every app update
(`drone_setup_px4_cesium.py:326-347`):

- position copies the drone body exactly (plus `DOWN_Z_OFFSET`);
- orientation is the body attitude times a constant image roll, passed through a
  **first-order low-pass** — `DOWN_VIB_DAMP = True`, `DOWN_VIB_TAU_S = 0.05`
  (`:68-69`), standing in for the silicone anti-vibration mount a gimbal-less
  global-shutter camera needs on a real airframe.

Flow-odom assumes a **constant** body→camera extrinsic:
`R_wc0 = r0["R_wb"] @ R_CtoI` (`flow_odometry.py:403-405`). Under a 50 ms
attitude lag that assumption fails whenever the drone rotates. The damage is not
cosmetic: `R_c1c0` is precisely what `_solve_translation` uses to predict where a
feature would land under rotation alone, and it attributes whatever is left over
to translation (`pipeline.py:242-247`). An attitude error therefore converts
directly into **phantom translation**. At the ZED's focal length
(2.2 mm / 3.45 µm ≈ 638 px) 1° of error is ~11 px of apparent shift, which at
survey altitude is metres of invented motion per frame.

Nothing in the existing code would catch it: the recorder tests for a stabilized
rig with `"down_gimbal" in cam_path` (`vio-recorder-pai.py:145`), and this camera's
path says `down_mount`, so it records a takeoff-snapshot extrinsic as if the mount
were rigid.

**This would have looked exactly like estimator drift.** Isolating it is worth more
than the realism it costs during bring-up.

## Architecture

```
Isaac Sim (Kit, --exec)          drone conda env                  drone conda env
┌──────────────────────┐  ZMQ    ┌─────────────────────┐  ZMQ    ┌────────────────────┐
│ vio-streamer.py      │ :5556   │ pipeline-streaming  │ :5557   │ joystick-server.py │
│  IMU / Baro callbacks│────────▶│  MahonyState (AHRS) │────────▶│  SOLE MAVLink owner│
│  down_cam capture    │ imu     │  LK flow-odom       │ vio     │  VISION_POSITION_  │
│  GT pose (scoring)   │ baro    │  ENU integration    │ pose +  │    ESTIMATE @ 20 Hz│
│  render-sync step()  │ frame   │  drift vs GT        │ health  │  EKF2 params       │
│  asserts rigid mount │ gt/meta │                     │         │  web telemetry     │
└──────────────────────┘         └─────────────────────┘         └─────────┬──────────┘
                                                                            │ udpin:14540
                                                                     ┌──────▼──────┐
                                                                     │  PX4 SITL   │
                                                                     │ EV on/GPS off│
                                                                     └─────────────┘
```

Three processes, because each boundary is forced:

- **Isaac ↔ estimator** — Kit's embedded Python has the sensor callbacks but heavy
  CV there stalls the render loop, and PX4 SITL runs in **lockstep** with Isaac
  (`SESSION.md:33-38`), so a stall there stalls the flight controller.
- **estimator ↔ joystick-server** — the estimator needs opencv/scipy and burns CPU
  per frame; the web server must stay responsive at 20 Hz. Keeping them apart also
  means a crashed estimator degrades to "vision stale" rather than taking the
  cockpit down with it.

## Decisions

### D1 — VPE is sent by `joystick-server.py`, not the estimator

`pipeline-streaming.py` publishes its pose on ZMQ `:5557`; `joystick-server.py`
subscribes and emits `VISION_POSITION_ESTIMATE` from the setpoint thread, on the
connection it already owns.

Preserves the sole-owner contract verbatim, and puts VIO health on the page the
operator is already flying from. Alternative considered: the estimator sends to
`udpout:127.0.0.1:14580` on its own socket (no bind conflict, fully decoupled) —
rejected because the estimator would be blind to PX4 and the UI would show nothing
about vision without adding a second channel anyway.

Cost, accepted: vision only flows while the web server runs. Since the web server
*is* how you fly, that is not a real loss.

### D2 — The origin comes from `sim/sites.py`

The estimator takes `lat/lon/height/heading` from the site config, not CLI args.
The 2026-07-14 addendum's warning stands and is the whole reason: the
`SET_GPS_GLOBAL_ORIGIN` value, the ENU origin the estimate drifts from, and the
origin waypoint targets are resolved against **must be the same numbers**, or the
aircraft flies somewhere else. One source, imported by both sides.

The streamer publishes the origin on a `meta` topic so the estimator never
hard-codes it and a mismatch is impossible rather than merely unlikely.

### D3 — v1 requires a rigid mount; the streamer asserts it

Runs are launched with `DRONE_SETUP_DOWN_VIB_DAMP=False` (the override block at
`drone_setup_px4_cesium.py:90-109` makes this a one-variable change). The mount
attitude then equals body attitude times the constant image roll, and the extrinsic
is genuinely constant.

The streamer **reads the live value and publishes it on `meta`**; the estimator
refuses to start when damping is on unless `--allow-damped-mount` is passed. An
assumption this load-bearing should fail loudly, not silently degrade — the whole
point is that its symptom is indistinguishable from estimator drift.

This also settles **how `R_CtoI` is obtained**, which is not obvious here: the
recorder derives extrinsics by composing local transforms up to a `body` ancestor
(`vio-recorder-pai.py:231-239`), but `/World/down_mount/down_cam` has no `body`
ancestor at all — it is a sibling of the drone, re-driven per frame. Walking that
hierarchy yields the camera's pose relative to the *mount*, which is identity, not
the extrinsic.

With a rigid mount the extrinsic is instead known **analytically**: mount attitude
is body attitude composed with `DOWN_IMG_ROLL_DEG` (`drone_setup_px4_cesium.py:291`,
applied at `:330`), and the camera looks down its local −Z. So `R_CtoI` is that
constant roll times the USD-camera look convention, computed once from the site
config rather than read off the stage. The streamer publishes it on `meta`, and a
startup check compares it against the live prim transform so a stage change cannot
silently invalidate it.

Deferred, not dismissed: modelling the low-pass (or publishing measured camera
attitude) to fly with the realistic soft mount. That is a follow-up with its own
validation, not v1 bring-up.

### D4 — Ground truth rides a separate topic, for scoring only

The streamer publishes Isaac's true pose on `gt`. The estimator uses it for **one
thing**: computing drift against its own estimate, reported as a number on the page.
It never enters the estimate — enforced by keeping the GT subscriber out of the
estimator's update path entirely.

Without this, a drifting estimate and a control problem look identical from the
cockpit. With it, "VIO error 3.2 m" is on screen next to the altitude.

### D5 — EKF2 params are pushed over MAVLink, behind `--vision`

`joystick-server.py:153-166` already sets `COM_RCL_EXCEPT`, `MIS_TAKEOFF_ALT` and
`MPC_XY_VEL_MAX` at startup via `param_set_send`. The EKF2 vision params go the
same way. QGC's parameter editor leaves the design entirely.

Gated behind an explicit `--vision` flag because `EKF2_GPS_CTRL = 0` is a
deliberate, safety-relevant act, not a default.

**Param set** (PX4 **v1.14.3**, values read from source, not recalled):

| param | value | default | why | source |
|---|---|---|---|---|
| `EKF2_GPS_CTRL` | `0` | 7 | disables all GNSS fusion — this *is* "GPS-denied" | `ekf2_params.c:706` |
| `EKF2_EV_CTRL` | `9` | 15 | bit0 horizontal position + bit3 yaw | `ekf2_params.c:687` |
| `EKF2_HGT_REF` | `0` | 1 (GPS) | height reference = **barometer** | `ekf2_params.c:657` |
| `EKF2_EV_NOISE_MD` | `1` | 0 | use the noise params below, not a reported variance we do not compute | `ekf2_params.c:814` |
| `EKF2_EVP_NOISE` | `0.5` | 0.1 | 10 cm is far too tight for drifting flow-odom | `ekf2_params.c:837` |
| `EKF2_EVA_NOISE` | `0.2` | 0.1 | ditto for yaw, which has no compass aiding | `ekf2_params.c:857` |

**`EKF2_EV_CTRL = 9`, not 15, is a substantive choice.** Flow-odom does not measure
vertical position at all: `pos[2] = r1["h"]` takes altitude *straight from the
barometer* (`flow_odometry.py:455`). Setting bit 1 would feed baro-derived height
back to EKF2 as an independent "vision" observation while EKF2 also fuses the same
barometer directly (`EKF2_BARO_CTRL` default 1) — the same sensor counted twice,
which an estimator reads as spurious agreement and over-trusts. So vision supplies
only what it genuinely observes (horizontal position and yaw) and height stays with
the barometer, where it actually comes from.

This corrects the old spec, which called for `EKF2_HGT_REF=EV`.

Bit 2 (3D velocity) stays clear: `VISION_POSITION_ESTIMATE` carries no velocity —
that is `VISION_SPEED_ESTIMATE`, which v1 does not send.

### D6 — `SET_GPS_GLOBAL_ORIGIN` is mandatory, not optional

With `EKF2_GPS_CTRL = 0` PX4 has **no absolute anchor**:
`VISION_POSITION_ESTIMATE` carries local NED offsets only, no lat/lon. Three
things in the shipped UI break without an origin, and all three fail *silently*:

- **The map.** The drone marker is driven by `GLOBAL_POSITION_INT`
  (`joystick-server.py:107-119`), which PX4 cannot produce without a global
  reference.
- **Waypoint flight.** Routes are flown with `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`
  (`streaming/offboard.py:222-238`) and PX4 projects lat/lon against the
  estimator's own reference (`:27-30`). No reference, no projection.
- **The FLY button.** It is gated on `home_valid` (`joystick-server.py:73-76`),
  set by `HOME_POSITION`, which PX4 only publishes once it holds a valid global
  position — and which it *returns silently without*
  (`mavlink_receiver.cpp:1107-1110`, cited in `offboard.py:32-34`).

So `joystick-server.py` sends `SET_GPS_GLOBAL_ORIGIN` once under `--vision`, using
the same `sim/sites.py` numbers as D2, before streaming any vision. That satisfies
the addendum's three-way consistency requirement by construction: one source feeds
the stage, the PX4 origin, and the estimator's ENU frame.

Sent from the setpoint thread like every other write, and re-sent if PX4 restarts
(detected by a `HEARTBEAT` gap), since a rebooted PX4 forgets it.

### D7 — v1 is flow-odom only

No DSMAC, no relief-fix, no climb-and-search. Unchanged from the original spec;
`2026-07-14-roadmap.md` still describes the follow-on order. Baro-derived depth
remains the known accuracy ceiling, carried forward as-is.

## Components

### `vio-streamer.py` (new, repo root, runs inside Kit)

Mirrors the sensor wiring of `vio-recorder-pai.py` (461-line version — Pegasus
`IMU`/`Barometer` callbacks, `down_cam` via replicator, JPEG+grayscale encode) but
PUBs over ZMQ instead of writing to disk. Reuses the recorder's
`rep.orchestrator.step(rt_subframes=1, pause_timeline=False)` render-sync call
(`vio-recorder-pai.py:404-412`), which exists because annotator reads otherwise
return pixels ~1 render period older than their own timestamp.

That call carries a warning the recorder could afford and this cannot: it may be
reentrant-unsafe from inside a physics callback on some Isaac builds. Under lockstep
a hang there stalls the physics step PX4 is blocked on, dropping the aircraft out of
OFFBOARD mid-flight. It gets an explicit verification step, not a copied line.

| topic | rate | payload |
|---|---|---|
| `meta` | 1 Hz | origin lat/lon/height, `K`, `R_CtoI`, image size, `vib_damp`, site name |
| `imu` | ~200 Hz | `ts_ns, wx,wy,wz, ax,ay,az` (FRD body) |
| `baro` | ~200 Hz | `ts_ns, pressure_altitude_m` |
| `frame` | ~15 Hz | `frame_idx, ts_ns, jpg_bytes` (grayscale, q92) |
| `gt` | ~200 Hz | `ts_ns, x,y,z, qx,qy,qz,qw` — scoring only (D4) |

`meta` repeats at 1 Hz rather than being sent once so a late-starting or restarted
estimator converges without restarting the sim.

### `streaming/zmq_proto.py` (new)

msgpack encode/decode for the topics above, plus the `vio` topic on `:5557`. One
module both sides import, so the wire format cannot drift.

### `MahonyState` (new, in `flow_odometry.py`)

The per-tick math inside `compute_ahrs_attitude` (`flow_odometry.py:162-182`)
extracted into an incremental `update(gyro, acc, dt) -> R`. A refactor of validated
math, not new logic; `compute_ahrs_attitude` is rewritten to call it so there is
one implementation, and a regression test pins batch output unchanged.

Cold-start fix carried from the roadmap: seed from the site heading rather than
`np.eye(3)`. The batch version initialises from GT frame 0
(`flow_odometry.py:144`), which is unavailable live, and identity is a 180° roll
singularity for the gravity correction.

### `pipeline-streaming.py` (new, repo root, `conda run -n drone`)

SUB on `:5556`; runs `_detect` / `_track_lk` / `_solve_translation` imported
unchanged from `pipeline.py`; integrates `dC` into ENU position; PUBs pose + health
on `:5557`.

### `streaming/vision_bridge.py` (new)

`VisionPositionSender` (ENU→NED, radian attitude, `vision_position_estimate_send`)
and the EKF2 param set from D5. Imported by `joystick-server.py`; unit-tested
against a mocked connection like `streaming/tests/test_offboard.py` does.

### `joystick-server.py` (modified)

Adds `--vision`: a ZMQ SUB thread storing the latest pose under a lock, a
`VisionPositionSender` call in the existing 20 Hz tick, EKF2 params in
`_send_startup_params`, and a `vio` block in the telemetry dict.

The SUB thread never touches `conn` — it writes to a lock-guarded slot the setpoint
thread reads, exactly as `CommandState` already bridges the web thread to the
setpoint thread.

### Web UI (modified)

A VIO row: fix rate, LK inlier count, **drift vs GT in metres**, seconds since last
vision update, and whether PX4 is fusing vision. Stale vision is styled `bad` — the
same `good`/`wait`/`bad` vocabulary the telemetry bar already uses
(`web/js/telemetry.js:84-105`).

## Data flow / error handling

- **Vision goes stale** (estimator crash, ZMQ drop): the sender stops emitting past
  a freshness threshold rather than repeating a frozen pose. A stale-but-plausible
  position is worse than none — EKF2 has failsafes for a lost source and none for a
  lying one. The UI turns the VIO row red.
- **Estimator restart**: `meta` at 1 Hz re-primes it; flow-odom resets `prev` and
  `MahonyState` from the next frame/IMU pair. No state persists across restarts,
  matching batch behaviour.
- **Dropped/out-of-order frames**: the estimator tracks the last processed
  `frame_idx`; a gap is just a longer LK baseline, which `pipeline.py` already
  tolerates via `--stride`.
- **ZMQ backpressure**: PUB sockets use `CONFLATE`/short HWM on `imu`/`gt` so a slow
  estimator drops old samples instead of accumulating unbounded lag. `frame` keeps a
  small queue — dropping a frame is cheaper than processing a stale one.
- **PX4 not up / not armed**: sends are attempted regardless; EKF2 ignores what it
  cannot use. No buffering — a stale position has no value.

## Testing

### Automated (`conda run -n drone pytest streaming/tests/`)

- `MahonyState` reproduces `compute_ahrs_attitude` batch output on a recorded
  `imu.csv` fixture.
- msgpack round-trip for every topic, including `frame`'s binary payload.
- ENU→NED conversion against known vectors.
- `VISION_POSITION_ESTIMATE` field/unit packing against a mocked connection.
- EKF2 param set asserts the exact names/values/types from D5.
- Freshness gate: a stale pose must **not** be sent.
- The estimator's GT subscriber must not influence the estimate (D4) — asserted by
  running a fixture with and without `gt` and requiring identical output.

### Manual (live)

1. `DRONE_SETUP_DOWN_VIB_DAMP=False ./sim/launch-sitl.sh` → drone spawned, Play
   pressed, MJPEG on 8080.
2. Streamer running → per-topic counters climb; `meta` reports `vib_damp: false`.
3. `pipeline-streaming.py` → per-frame inlier counts; drift vs GT printed.
4. `joystick-server.py --vision` → EKF2 params confirmed; VIO row populates.
5. **Baseline first**: fly on GPS with vision merely *reported* (params not yet
   applied) to confirm nothing regressed, then enable `--vision`.
6. Arm, take off, hold position on vision only. Watch drift vs GT climb; the pass
   bar is holding station, not zero drift.
7. Kill the estimator mid-hover → VIO row red, PX4 failsafes on a lost source
   rather than lurching on a frozen one. Restart → recovers with no sim restart.
8. Fly a short waypoint route from the map on vision only.

## Out of scope

- DSMAC / relief-fix (fast-follow — `2026-07-14-roadmap.md` §3).
- Climb-and-search (`roadmap.md` §5), gated on the validation in §2.
- Modelling the soft-mount low-pass (D3 deferral).
- Rangefinder/AGL — no such sensor on this rig; baro depth is the known ceiling.
- Estimator state persistence across restarts.
- Automated end-to-end sim tests; the live checklist covers that.
