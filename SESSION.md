# Session notes: VIO GPS-denied live bring-up (plan Task 8)

**Plan:** `docs/superpowers/plans/2026-08-07-vio-gps-denied.md`, Task 8
**Status 2026-08-13:** 8.1–8.5 pass. **GPS-denied flight is real and repeatable**
— `cs_gps: False`, `cs_ev_pos: True`, OFFBOARD held on vision alone. It does not
hold station, so **8.6 does not pass**. 8.7–8.9 sit downstream of a stable hover.

Five flights, each fix exposing the next fault. Read the chain top to bottom.

## Running it

```bash
DRONE_SETUP_DOWN_VIB_DAMP=False VIO=1 ./sim/launch-sitl.sh
python joystick-server.py --takeoff-alt 50 --speed-up 3.0
```

Two commands. `joystick-server.py` spawns and supervises `pipeline-streaming.py`,
`--vision` defaults ON, and `--site` defaults to the launcher's. Isaac stays
separate and always will — `vio-streamer.py` runs inside its physics callback.
`--no-vision` gives an ordinary GPS flight; `--no-estimator` keeps vision but
leaves the estimator to you.

**The default reboots PX4 once at startup** (phase 0, `EKF2_HGT_REF`) on every
run, including GPS-only ones. That is the cost of the one-command default, and
the startup banner says so.

Offline: **247/249 tests pass** in the `drone` conda env (the 2 failures are
environmental — see Task 9 note below). They fail in *base* conda for want of
`websockets`/`msgpack` — wrong interpreter, not a real failure. Use
`~/miniconda3/envs/drone/bin/python`.

## Run 1 — `EKF2_HGT_REF` is reboot-required, and not optional

8.1–8.4 passed cleanly. Streamer up at ~14.7 img/s with `meta` verified over the
wire (`site=bangkok-survey-040 vib_damp=False heading=0.0`), zero `poll timeout`
lines all session. Estimator drift vs GT hovering at 49 m: **0.36–0.61 m**, ~595
inliers, 8.8 fps — bounded, not growing. GPS baseline flew normally to 49.2 m and
held station. `sim_rate` held ~0.60 with everything running.

Then `--vision` on an already-booted PX4. `EKF2_GPS_CTRL=0` took effect
immediately; `EKF2_HGT_REF=0` did **not** — it is `@reboot_required`
(`ekf2_params.c:656`), so PX4 stores the new value and QGC shows it while EKF2
keeps its old height reference. GNSS fusion off, height reference still on GPS:
altitude ran 49 → 12 → 0 → **−22 m and still falling** at a constant −0.7 m/s,
`AUTO.LAND` latched, `offboard` and `disarm` both refused.

The estimator was healthy the whole way down (600 inliers, `drift_m` ~2.3,
`dropped_stale: 0`). Not a VIO quality problem — a param-ordering one the code
had already predicted in a comment. The param set is applied at connect time,
which is by construction *after* boot, so under that design it could never work.

The origin itself landed: `GLOBAL_POSITION_INT` kept producing a map position
with GNSS fusion off, and Task 7's telemetry row worked end to end.

## Run 2 — the phase split works; the diagnosis that followed did not

Reflown with the three-phase sequence. **The height blocker is fixed** — PX4
rebooted, params resumed on the fresh instance, the aircraft climbed on GPS to a
stable hover (600 inliers, drift 0.36–0.61 m).

Then `gps_denied` was accepted, **and PX4 immediately fell out of OFFBOARD into
ALTCTL and descended at 0.70 m/s.** Re-commanding OFFBOARD was refused. EKF2 had
not accepted the vision stream as a position source at all.

The write-up blamed VPE timestamps — Isaac sim time against PX4's `hrt` epoch —
and that was **wrong**. From the source: `sync_converged()` requires
`_sequence >= 500` (`Timesync.hpp:76,120`), `_sequence` only advances in the
TIMESYNC round-trip handler (`Timesync.cpp:93`), and nothing in this repo or in
pymavlink sends TIMESYNC. So `sync_stamp()` (`Timesync.cpp:127-136`) was already
returning PX4's own arrival time for every estimate. Sending a corrected
timestamp would have changed nothing.

The lesson is the ordinary one: the one unverified quantity attracted the blame
precisely because every other number looked healthy.

## Run 3 — the real root cause was INT32 param encoding

`set_param` sent INT32 params as `float(value)`. PX4 does not convert — for an
INT32 param it reinterprets the PARAM_SET float field's **raw bytes**
(`param_set(param, &(set.param_value))`, `mavlink_parameters.cpp:134`). So
`float(9)` stored **1091567616**, the bit pattern of `9.0f`, and PX4 accepted it
silently because that is a legal int32. Read back off a live PX4:

| Param | PX4 held | Intended |
|---|---|---|
| `COM_RCL_EXCEPT` | 1082130432 | 4 |
| `EKF2_EV_CTRL` | 1091567616 | 9 |
| `EKF2_EV_NOISE_MD` | 1065353216 | 1 |
| `EKF2_GPS_CTRL`, `EKF2_HGT_REF` | 0 | 0 ✓ |

**Only the zero-valued ones worked**, `0.0f` and int `0` sharing a bit pattern.
That asymmetry hid the bug: every param that *disabled* something worked, every
param that *enabled* something was dead. GNSS really was cut; vision fusion never
switched on. It also means **`COM_RCL_EXCEPT` was never 4 on any flight this repo
has ever made** — OFFBOARD was never exempted from the RC-loss failsafe.

Fixed in `offboard.py:set_param`, which now sends the bit pattern for INT32.
Verified live, and for the first time `cs_ev_pos: True` / `cs_ev_yaw: True`.

**Four faults it then exposed, each invisible until the one before it was fixed:**

1. Fusing a blind camera stops the aircraft arming — `Preflight Fail: Yaw
   estimate error`. Fusion is now deferred until the estimate clears
   `VISION_MIN_INLIERS`, which needs altitude.
2. **PX4 params persist across runs and reboots.** Deferring the fusion params in
   code changed nothing, because the previous run had saved `EKF2_EV_CTRL=9`. Not
   setting a param is not the same as it being off. Startup now *asserts* GPS
   flight; the dangerous direction is a saved `EKF2_GPS_CTRL=0` booting the next
   run GPS-denied on the pad.
3. The param re-send undid fusion mid-flight — `_send_startup_params` is also the
   recovery path. It now re-asserts whichever phase is current.
4. `PX4_RESTART_GAP_S` was wall time against a **sim-time** heartbeat. Lockstepped
   to Isaac, PX4's 1 Hz heartbeat arrives every `1/sim_rate` wall seconds — ~1.8 s
   at 0.55. The old 3 s threshold was 1.5 beats and kept tripping (3). Now 10 s.

**GPS-denied flight then happened for the first time** and held ~40 s before
diverging: altitude 24.9 → −31.3, `vz` to −20 m/s, drift 28 → 352 m. `dropped_stale`
climbed 15 → 179 as `sim_rate` collapsed 0.56 → 0.11 — `VisionPositionSender`
judged staleness on **wall clock** while everything producing the estimates ran on
sim time, so healthy estimates were dropped exactly when they were the only
position source. The third wall-vs-sim-time confusion in this system.

## Run 4 — staleness fixed; the estimator itself exposed

Ageing estimates against PX4's `time_boot_ms` (sim time under lockstep):

| | run 3 (wall clock) | run 4 (sim clock) |
|---|---|---|
| `dropped_stale` | 15 → 179 | **0 throughout** |
| `sim_rate` | 0.56 → 0.11 | 0.41–0.59, no collapse |

The aircraft still diverged, for a different reason. `drift_m` ran 23–37 m at the
cut, then 60 → 318 → 1195 → 2544 m, ground speed to 58 m/s. Two causes:

1. **The estimate had already drifted tens of metres before the handover.** 0.36–0.61 m
   in a hover; 23–37 m after a 3 m/s climb to the same altitude. Flow-odom solves
   translation against a ground plane at barometric height, worst conditioned when
   altitude changes fast.
2. **Nothing realigned the two frames at the cut.** EKF2 was handed a vision frame
   ~30 m from where it believed it was, then followed it — the aircraft chases the
   offset, which moves the camera, which feeds more drift.

(2) was a *design* gap: the two-phase profile was specified before anyone had
flown far enough to see that the phases meet at a discontinuity.

## Run 5 (2026-08-13) — the handover is solved; the failure changed shape

Flown with `vision_bridge.FrameAlignment`, the ENU→NED attitude fix, and
`EKF2_EVP_NOISE` 0.5 → 3.0.

| | run 4 | run 5 |
|---|---|---|
| drift through the climb | 23–37 m | **0.29–0.60 m** |
| realign at fusion start / at the cut | did not exist | 0.2 m, +0.1° / 0.4 m, −0.2° |
| drift at the cut | 23–37 m | **0.46 m** |
| after the cut | 60 → 2544 m, 58 m/s | oscillation, ±5 → ±60 m |
| aircraft | ran 165 m off | stayed in OFFBOARD throughout |

Both realignments had almost nothing left to close. For the first ~40 s after the
cut the aircraft held inside ~1 m with drift 0.42–1.04 m — the first time
vision-only station keeping has looked right.

**The climb-drift collapse was not the alignment.** `drift_m` is the raw estimator
against GT and never sees the transform. The likely cause is the attitude fix:
EKF2 had been fusing a vision yaw wrong by up to 180°, which made the aircraft fly
badly during the climb, which swept the camera and wrecked flow-odom's own input.
The estimator was being blamed for a fault upstream of it. (Run 5 also ran at
8.9 fps against run 4's 5.3, so some of the gain is a fresher sim.)

**What now fails: a slow, growing oscillation** — ~40–60 s period, ±5 m growing to
±60 m, 550–600 inliers, no rejections. A control/estimation instability, not a
tracking failure: the monotonic runaway is gone and nothing diverges at speed.

### The two fixes, and why they were shaped that way

**`FrameAlignment`** is a rigid transform from the estimator's frame into PX4's
local NED, re-solved at fusion start and at the cut, then held. Yaw is rotated as
well as position — a translation-only fix leaves the frames rotated against each
other, and every metre flown after the handover then points a few degrees wrong,
an error that *grows with distance*. It is taken at the transitions **only**:
re-solving every tick would feed EKF2 its own estimate back as an independent
measurement, innovations would sit at zero by construction, and a completely
broken estimator would look perfect right up until GNSS was cut. Both transitions
refuse to proceed without a PX4 pose to align onto.

**`VISION_POSITION_ESTIMATE` was carrying ENU attitude into an NED field.** The
estimator works in ENU/FLU (`pipeline-streaming.py:132`, yaw measured from east,
counter-clockwise); only *position* was being converted. The error is
`90 − 2·heading` degrees — 0 at heading 45°, a full 180° at 135°. `EKF2_EV_CTRL=9`
sets bit3, so that yaw *was* fused: EKF2 was told the aircraft faced somewhere it
did not, and every vision-derived horizontal correction was applied in the wrong
direction. Pitch was mirrored too (FLU vs FRD), and it matters — PX4 rebuilds a
quaternion from all three angles and extracts yaw from that. `enu_attitude_to_ned`
now does the whole conversion, verified against `R_frd_ned = S · R_flu_enu · B`.

The old test passed only because it used `yaw = pi/2` — heading 45°, the one
bearing where the raw ENU number happens to be right. It now sweeps headings.

## Findings that outlived their run

**The nadir camera is blind near the ground at this site.** Parked at 0.5 m AGL
the `/down` frame is mean 1.0, **std 0.00**, **0 features**; hovering at 49 m it
is mean 163.5, std 46.7, **600 inliers**. The camera sits below the Cesium tile
surface at spawn, so **a vision-only takeoff is not possible at
`bangkok-survey-040`** — any GPS-denied profile here must reach altitude first.
Confirmed from both directions: inliers ran 44 → 304 → 440 → 600 during the climb,
and collapsed 600 → 282 → 139 → 37 as the aircraft sank back through the terrain.

This also corrected the older "Isaac render needs a stream client" belief.
**Altitude, not a stream client:** `/detect` rendered real content (mean 143.1,
std 83.5) with no browser attached, while `/down` stayed black *with* an MJPEG
client pulling it. The underlying advice stands and is what caught this — **check
a frame's std is non-zero before trusting any VIO run.**

**`RESTORE GNSS`** applies `EKF2_GPS_CTRL=7` and clears `_gps_denied`, landing
back in phase 1b — GNSS on with vision still fused, the state the cut was taken
from. It is **deliberately ungated**, unlike the cut: every refusal in
`_go_gps_denied` exists because cutting onto a bad source can put the aircraft in
the ground, and restoring has no such failure mode. A recovery control that can
say no is not one. It earns its place because the manual recovery was
`px4-param set EKF2_GPS_CTRL 7` typed into a shell while the aircraft accelerated
away — measured on one run at 180 m off and 3.8 m/s. Clearing `_gps_denied`
matters as much as the param: without it the next heartbeat gap re-asserts the cut.

**`alt` is negative on the pad, and that is correct.** `sites.py` puts the
georeference origin at stage z = 0 and `ground_z` at −25.0, so a drone on the
ground is ~25 m below PX4's local origin. `relative_alt` reads 0.07 m correctly.
(`sim/launch-sitl.sh:39` still says −26.99; that comment is stale.)

**`Preflight Fail: height estimate not stable` is a settling problem.** The
message misleads — the height was steady. The trigger is
`pre_flt_fail_innov_height`, fed by the *baro* innovation against
`_hgt_innov_test_lim = 1.5f`, a compile-time constant
(`PreFlightChecker.hpp:199`) no parameter can relax. EKF2 was initialising while
the drone still settled onto the collision plane, because the one-command launch
reboots PX4 the instant the server starts. Phase 0 now holds until the airframe
has been at rest for `SETTLE_S` **sim** seconds and sends nothing while it waits;
a genuine cold start then gave `baro_vpos` **0.039 m** and ARM in 1.4 s. Being at
rest is a *proxy* for the barometer having settled — if this resurfaces on a
provably still aircraft, gate on the baro innovation itself. The gate applies to
phase 0 only: `_send_startup_params` is also the recovery path, and re-gating it
there would refuse to re-assert vision params on an airborne aircraft.

**Watch `drift_m` after an estimator restart.** It compares the estimator against
a GT anchor set at the ORIGINAL spawn, so a fresh estimator on a drone that has
moved reports the whole displacement as drift (211 m in one test). The frame
alignment handles it correctly — it closed 211.2 m at the cut, which is its job —
but the drift number is meaningless until the run restarts from the pad.

**The bug class to expect here.** Every boundary crossing in this path has
produced a bug: three from the wrong clock (`PX4_RESTART_GAP_S`, the VPE timestamp
theory, the staleness budget), one from the wrong encoding (INT32 params), one
from the wrong frame (ENU into NED). Check units, clock and frame at each hop
rather than assuming the neighbouring code agrees.

## Where the decisions live now

Design-grilled 2026-08-13 after run 5. The calls are ADRs, and the work is
`Task 9` in the plan:

- `docs/adr/0001` — 8.6 is 180 s with a non-growing excursion envelope. 60 s is
  barely one period of the observed mode.
- `docs/adr/0002` — the next flight changes no parameter; it classifies the mode.
- `docs/adr/0003` — **no flight has ever been recorded.** Pegasus runs PX4 in a
  temp dir it deletes on exit. `sim/save-ulog.sh` plus `SDLOG_*` in phase 0 fixes
  it, and unlocks offline EKF2 replay.
- `docs/adr/0004` — nothing computed *aircraft excursion* as opposed to *estimator
  drift*; the 8.6 pass number was not measurable by anything.
- `docs/adr/0005` — the estimator has **no heading reference at all** (`mag=None`
  every tick) while EKF2 has a compass. Corrects the design's premise.
- `docs/adr/0006` — `EKF2_EV_DELAY` is measured on a clock whose rate wanders;
  measure before setting.
- `docs/adr/0007` — PX4-side work ends when Task 8 closes.

`CONTEXT.md` pins the vocabulary these write-ups were overloading — notably
"drift", which meant three different quantities across the runs above.

## If this resurfaces

`alt_m` sinking steadily with `AUTO.LAND` latched and `offboard`/`disarm` both
refused, while the VIO row stays green, is the signature of the height reference
being wrong rather than of a vision failure. Check `EKF2_HGT_REF` took effect
*at boot*, not merely that QGC shows 0.

## Task 9 (2026-08-13) — diagnostic instrumentation implemented, not yet flown

9.1–9.3 landed: `sim/save-ulog.sh`; `SDLOG_MODE=2`/`SDLOG_PROFILE=131` in the
phase-0 boot params; the magnetometer channel wired through at `mag_gain=0.0`
(ADR-0005); ground truth forwarded on the `vio` message for scoring only; and
`joystick-server.py` now computes **aircraft excursion** (ground truth against
the hold point taken at the cut — distinct from `drift_m`, the estimator's own
error, per `CONTEXT.md`) and writes a 20 Hz `logs/<run-name>/run.csv` alongside
whatever `sim/save-ulog.sh <run-name>` copies out. 247/249 tests pass; the 2
failures are pre-existing and environmental, not caused by this work — they
assume Isaac is down, and it was genuinely up and recording during this run
(confirmed by stashing every change and re-running: same 2 failures, same
reasons, on a clean tree).

This landed **while a live session from an earlier context was still running**
— Isaac, PX4, `joystick-server.py --vision`, and the estimator had been up for
~45 minutes, had cut GNSS twice (testing the `RESTORE GNSS` button), and were
holding a stable hover. `sim/save-ulog.sh` was run against that live PX4 as a
correctness check and correctly copied a 394 MB `.ulg` out via `/proc/<pid>/cwd`
— the mechanism is proven. Nothing else was verified live: `vio-streamer.py`'s
magnetometer read only runs inside Kit's Script Editor and was not
re-triggered, and none of the `joystick-server.py`/`pipeline-streaming.py`
changes are loaded by the already-running processes. **The instrumentation
will not appear until the stack is restarted** — a plain restart is enough for
9.2/9.3 (no reboot needed), but 9.1's `SDLOG_*` params are `@reboot_required`
and only take on the *next* phase-0 reboot, i.e. the next full run.

Deliberately did not kill the live session to force that reboot: it was mid
experiment (not the Task 9.4 classification flight) and stable, and forcing a
restart to pick up measurement-only code is exactly the kind of unrequested
disruption this instruction set warns against. **9.4 — the classification
flight — has not been attempted with this instrumentation** and is the next
step once the operator is ready to cycle the stack.

## Run 6 (2026-08-13) — 9.4a flown with the new instrumentation; worse than run 5, but a landing, not a crash

The operator approved cycling the stack. Full restart —
`DRONE_SETUP_DOWN_VIB_DAMP=False VIO=1 ./sim/launch-sitl.sh`, then
`joystick-server.py --takeoff-alt 50 --speed-up 3.0 --run-name 20260813-231224`
— so phase 0's reboot picked up `SDLOG_MODE=2`/`SDLOG_PROFILE=131` for the
first time. Flew the unchanged 9.4a profile (arm, takeoff, offboard, wait for
fusing, settle 20 s, cut, hold **180 s** with no parameter changes, restore,
land) driven over the websocket exactly as the page would send it.

**The cut was clean** — 0.72 m drift, 20.5 m altitude, same signature as every
prior run. **The hold was not.** Aircraft excursion (ground truth against the
hold point, Task 9's new number) grew almost continuously from the start:
47 m by t+30 s, peaking at **247 m at t+58 s**, while altitude fell the whole
time (20.5 m → ~7 m at the peak). `px4_yaw` held within ~2° the entire
180 s — this is **not** the orbit/spiral ADR-0002 named as the heading-fault
signature. It is closer to a one-directional runaway: `n_inliers` stayed
401–600 throughout, `dropped_stale` was 0, `fresh` was `True` on every one of
3604 phase-2 samples — vision tracking was never the problem. `sim_rate` dipped
to 0.172 briefly at t+34 s, after excursion had already reached ~50 m and
climbing, so it reads as a symptom of the sim struggling under the runaway
rather than a trigger for it.

After the peak, excursion partially recovered (247 m → ~186 m by t+68 s) while
still descending, then both altitude and excursion went nearly flat for the
rest of the hold — the scripted flight then restored GNSS and commanded land
exactly per profile.

**First read of the CSV misread this as a crash** — `alt_m` ended at −6.18,
and every previous run's altitude sign convention was internalised as
"negative is bad." It is not: `ground_z = −25.0` (`sim/sites.py`) is a single
flat physics plane across the whole site (not terrain-following,
`sim/stage_builder.py:74-80`), so −6.18 is ~19 m of clearance, not a strike.
Confirmed live after the flight: querying the running server showed
`alt_m: −24.89`, disarmed, `gs`/`vz` ≈ 0 — i.e. sitting cleanly on
`ground_z`, not embedded in it — and the down camera showed the same flat
black this project has always seen near the ground (mean-intensity, zero-std
signature documented earlier in this file). **The aircraft landed itself
safely, ~225 m from the pad, once commanded to.** It was flyable and
controllable throughout; it just held station far worse than run 5's bounded
±5→±60 m oscillation.

Ulog and CSV saved to `logs/20260813-231224/` (`16_14_19.ulg`, 38 MB;
`run.csv`, 7020 rows, 3604 in phase 2) — 9.1d's log-capture path is now
confirmed working against a `SDLOG_PROFILE=131` boot, though
`estimator_aid_src_ev_pos` presence in the ulog itself has not yet been
checked (still open).

**Not yet done:** 9.4b's fuller classification (this write-up is a first read,
not the systematic one), 9.4c (EV delay vs `sim_rate` from the ulog), 9.4d
(magnetometer heading vs Mahony yaw vs GT yaw), 9.4e (offline replay sweep).
Also open: whether this run's severity (247 m peak) is typical of the same
underlying instability as run 5, or an outlier — nothing in Task 9's changes
touches flight dynamics, so the two runs should be sampling the same failure
mode, but only one more flight would confirm that. Isaac/PX4/server were left
running after this analysis, aircraft parked ~225 m off pad, disarmed.

## Task 9.4d (2026-08-14) — heading ruled out

`run.csv`'s 20 Hz stream never carried `gt_yaw` or a magnetometer heading —
only `gt_x`/`gt_y` (position) were forwarded, and the estimator's
magnetometer read (`vio-streamer.py`) is consumed internally by
`MahonyState` and never sent onward. 9.4d as scoped is not answerable from
the CSV. It is answerable from the ulog: `pyulog` (installed into the
`drone` conda env for this) exposes `vehicle_attitude` (EKF2's fused yaw),
`sensor_mag` (raw magnetometer, tilt-compensated here against
`vehicle_attitude` into an independent heading — the actual substitute for
the missing GT yaw), `vehicle_visual_odometry` (the vision yaw PX4 received,
post-`FrameAlignment`), and `estimator_aid_src_ev_yaw` (EKF2's own internal
innovation between its belief and the vision measurement).

Also found while doing this: **`run.csv` for `20260813-231224` is
contaminated.** The orphaned `joystick-server.py`/`pipeline-streaming.py`
pair left running after run 6 (see the "run website" session note above; PIDs
2682059/2682088) kept appending to that same CSV for roughly eleven more
hours — it now has 798,178 rows against the 7,020 run 6 actually produced.
The `.ulg` is unaffected (PX4 stops logging when the flight recorder is
stopped, independent of the CSV writer) and is what all analysis below used.
**Any future read of `run.csv` under this run name needs the real window
sliced out by timestamp first; don't trust row count or tail as the end of
the flight.**

The real cut/restore window, from `EKF2_GPS_CTRL`'s param-change log in the
ulog rather than assumed from the script: fusion started (`EKF2_EV_CTRL=9`)
at t=18.96 s, GNSS was cut at t=50.68 s, restored at t=156.63 s. **The hold
was 106 s, not the scripted 180 s** — `RESTORE GNSS` fired well before the
profile's intended duration, for a reason not yet investigated. Position
excursion computed straight from the ulog (`vehicle_local_position`) peaked
at 267.7 m at t+60.1 s after the cut — consistent with the CSV-and-ground-
truth-based 247 m at t+58 s reported for run 6 above, confirming both are
the same event and that the ulog's timeline lines up with the earlier read.

**Every yaw signal agreed with every other one throughout the entire
106 s hold:**

| Signal | Mean | Std | Max deviation |
|---|---|---|---|
| EKF2 fused yaw vs raw mag heading | −1.55° | 1.41° | 5.4° |
| EV-yaw innovation (EKF2 belief − vision) | −0.53° | 0.54° | 2.0° |
| EV-yaw `test_ratio` (>1 would mean EKF2 doubts vision) | ~0.000 | — | 0.001 |

A 267 m runaway cannot come from a heading error under 2°. **ADR-0002's
heading/orbit hypothesis is ruled out**, with real margin, by an
independent raw-compass cross-check PX4's own self-referential yaw estimate
could not provide on its own — not just "`px4_yaw` looked steady," which was
the limit of run 6's first read. One weak, inconclusive signal: the EV-yaw
innovation's autocorrelation shows a peak near 86 s lag, but the innovation
never exceeds 2° through the whole hold, too small relative to noise to call
that a real oscillator.

This shifts weight decisively onto ADR-0002's other branch — **lag or loop
gain in the position-hold loop**, not heading — as the live hypothesis for
the excursion/oscillation.

## Task 9.4c (2026-08-14) — EV delay does not track sim_rate

ADR-0006 held `EKF2_EV_DELAY` at its default (`0`, PX4's own default —
`ekf2_params.c:151`) until the applied delay could be measured, on the
worry that much of the pipeline's latency is wall-clock CPU work that maps
into sim time scaled by `sim_rate` (observed wandering 0.41–0.65 across this
project's runs) — in which case tuning the param against one snapshot of
system load would be measuring the wrong thing.

`estimator_aid_src_ev_pos`'s `timestamp` (when EKF2 fused the sample) minus
`timestamp_sample` (the sample's own stamp, as this codebase sent it) gives
the actual applied delay, in the same sim clock as everything else PX4
does. Extracted over all 4645 logged samples in run 6's ulog and matched
against `run.csv`'s `sim_rate` column (only the first 5000 rows — see the
9.4d note above on why the rest of that file is contaminated and unusable):

**The delay is flat at 83.4 ms ± 1.4 ms (range 80.0–84.0 ms) across the
entire flight**, phase 1b and the GPS-denied hold alike — while `sim_rate`
itself ranged 0.17 to 0.65 in the same window. Binning delay by `sim_rate`
decile shows no trend at all (every bin: 83.3–83.5 ms mean). Correlation
between delay and `sim_rate`: **+0.018**. Correlation between delay and
`1/sim_rate` (what ADR-0006's hypothesis predicts should be positive):
**−0.013** — indistinguishable from zero, and the wrong sign besides.

**`EKF2_EV_DELAY` is not a sim-rate artifact in this data.** Whatever holds
the delay this steady — most likely PX4's own fixed fusion-horizon
buffering rather than the variable wall-clock cost of JPEG-encode/ZMQ/LK-
solve that motivated the worry — dominates over the rendering-load-driven
component enough that it doesn't show up even across a 4x swing in
`sim_rate` within one flight. Stabilising `sim_rate` would not change this
number.

This does leave a real, small, measured gap: EKF2 is told the delay is 0 ms
while it is actually ~83 ms, so `EKF2_EV_DELAY≈83` is now a legitimately
measured value rather than a guess, if it's worth setting (`@reboot_required`
— goes in phase 0, not the fusion params). But per ADR-0002's own reasoning,
an ~83 ms delay is far too fast to produce a 40–60 s-period oscillation on
its own — it would perturb the position loop at its own (much faster)
bandwidth. So 9.4c neither confirms nor rules out the lag/loop-gain branch;
it just clears `EKF2_EV_DELAY` specifically as a sim-artifact red herring
and leaves a small, real, fixable number on the table for later.

9.4e (offline replay sweep) is still open.

## Task 9.4b (2026-08-14) — straight line, not a spiral: lag/loop-gain confirmed

ADR-0002's test: an orbit or spiral against the hold point indicts heading; a
straight-line back-and-forth indicts lag or loop gain. Classified run 6's
ground-truth track (`gt_x`/`gt_y`) and, separately, PX4's own estimate
(`px4_n`/`px4_e`), both against the hold point taken at the cut, over the
full 106 s hold — bearing angle (unwrapped, relative to the hold point),
radius, and a PCA eigenvalue ratio on the (x, y) cloud (near 1.0 = circular
spread, near 0.0 = a line).

**Binned bearing locks onto a single direction, −173° ± 0.5°, from t+25s
to the end of the hold at t+110s, and never leaves it.** The first 25 s
looks unstable in the raw bearing number (−2° to −306° across a few 5 s
bins) purely because radius was under 16 m there — `atan2` on a
near-origin point swings wildly for centimetre-scale noise, not because
anything was actually rotating; radius in that window only reached 15.9 m.
Once the excursion is large enough for bearing to mean anything, it is
flat. PCA eigenvalue ratio (minor/major axis) is **0.000** on both the
ground-truth and PX4-estimate clouds — as linear a spread as this metric
can report. Excluding the noisy first 25 s and reclassifying from there:
total absolute bearing travel over the remaining 81 s is 186°, half a
turn, against a peak-to-peak radius swing of order 100 m — the opposite of
an orbit, which would rack up many full turns while barely covering
ground radially.

**Radius over time is a damped step response to a false equilibrium, not
a sustained growing oscillation:** 0 → 246.6 m (peak, t+58s) → 191–207 m
(undershoot, t+65-70s) → ~230 m (second, smaller overshoot, t+75-80s) →
settles into a tight 222–228 m band from t+80s onward, holding there for
the rest of the 106 s hold. Two damped cycles, then station-kept — at the
wrong spot, ~225 m from the intended hold point, along one fixed bearing.
This is a materially different shape than the "±5 m growing to ±60 m"
language used for run 5 above; whether that's a difference in the
underlying fault or just a different point in the same fault's transient
is still open (nothing in Task 9 touches flight dynamics, so the working
assumption is still one fault, but this is the first run with the
resolution to say the *shape* isn't identical either).

**Conclusion: heading is not just ruled out by the yaw data (9.4d) — the
track itself doesn't have the shape a heading fault would produce.**
`EKF2_EV_DELAY` is real but too fast to explain a 100+-second transient
(9.4c). What's left, per ADR-0002, is **lag or loop gain in the
position-hold loop**: a roughly constant-direction bias appearing at the
cut, met with an underdamped response that overshoots twice before
settling onto the wrong position rather than the right one.

## Task 9.4e (2026-08-14) — replay sweep: every EV_* candidate killed

PX4's offline replay module (`src/modules/replay`) had never been built in
this checkout. Building it (`replay=<ulog> make px4_sitl_default`, which
configures a separate `build/px4_sitl_default_replay` with
`ORB_USE_PUBLISHER_RULES` and lockstep disabled) surfaced two pre-existing
upstream bugs, neither related to this project's code, both fixed directly
since they blocked the build outright:

- `platforms/posix/src/px4/common/px4_daemon/pxh.cpp` used `uint8_t`
  without including `<cstdint>` — silently relying on a transitive include
  the normal SITL build happens to pull in first, which the replay build's
  different translation-unit ordering doesn't. Added the include.
- `src/lib/matrix/matrix/Matrix.hpp:96` — GCC's `-Werror=array-bounds`
  false-positives on the generic `operator()` accessor for a 1×1 matrix
  instantiation, a known category of GCC false positive on heavily-templated
  fixed-size containers, guarded by an `assert` the compiler can't see at
  `-Werror` optimisation levels. Wrapped the one line in a
  `#pragma GCC diagnostic ignored "-Warray-bounds"` push/pop.

**Mechanics worth recording for next time:** `rc.replay` auto-generates
`replay_params.txt` from the log's initial params via `ulog_params` — a
`pyulog` console script that isn't on PATH inside PX4's minimal replay
shell, so it silently produces an *empty* override file rather than erroring
loud. To override a specific param, pre-create `replay_params.txt` yourself
in `build/px4_sitl_default_replay/rootfs/<instance>/` before starting —
`rc.replay` only generates one if the file doesn't already exist, and any
param named in it is frozen for the whole replay (`Replay::_overridden_params`
skips that param's historical change events from the log entirely, so a
frozen override survives the original flight's own phase transitions
untouched — GPS_CTRL and EV_CTRL's real cut/restore timeline replays
normally as long as you don't put those specific names in the file
yourself). `PX4_SIM_SPEED_FACTOR` had no measurable effect (1 vs 8 gave
identical ~15.6 s wall time for a 217 s log) — this box is CPU-bound well
above real-time regardless, not wall-clock-throttled. One trap: a replayed
ulog's own `changed_parameters` timestamps are wall-clock-at-apply, which
during a compressed CPU-bound replay do **not** preserve original spacing —
the GPS-denied window that took 106 s in the original flight showed as
0.43 s in one replayed ulog's raw parameter-change log. **The individual
topics' own embedded timestamps (`vehicle_local_position.timestamp`,
`sensor_combined.timestamp`, etc.) are unaffected** — confirmed by
`sensor_combined`'s replayed median dt sitting at 4.52 ms, matching the
original IMU rate — so EKF2 itself still integrated over correctly-paced
data. Window boundaries were located from `vehicle_local_position`'s own
span, not from `changed_parameters`.

**Validation:** replayed run 6's own ulog with no overrides and compared
against the original. Peak excursion 266.0 m at t+59.5s, settling to
225.3 m — against the original's 267.7 m at t+60.1s, settling to 225.0 m.
Within 2 m and 0.6 s. The replay reproduces the original EKF2 estimate
closely enough to trust the sweep.

**The sweep**, one factor at a time from the as-flown baseline
(`EKF2_EV_DELAY=0`, `EKF2_EVP_NOISE=3.0`, `EKF2_EV_CTRL=9`):

| Candidate | Peak (m) | Peak @ t+s | Settled (m) | Settled bearing std (deg) |
|---|---|---|---|---|
| baseline, as flown | 266.0 | 59.5 | 225.3 | 0.31 |
| `EKF2_EV_DELAY=83` (9.4c's measured value) | 267.5 | 59.6 | 225.2 | 0.31 |
| `EKF2_EVP_NOISE=0.5` (the pre-2026-08-13 value) | 266.3 | 59.3 | 224.1 | 0.37 |
| `EKF2_EVP_NOISE=6.0` (looser than baseline) | 266.9 | 59.6 | 224.7 | 0.33 |
| `EKF2_EV_CTRL=1` (bit3/yaw fusion off) | 257.7 | 57.3 | 225.9 | 0.32 |

**Every candidate was killed.** All five runs — including the one that
removes yaw fusion entirely — land within about 4% of each other on peak
excursion, within 2.3 s on timing, within 2 m on where they settle, and
within 0.06° on how tightly the bearing locks. Nothing tested moves the
needle.

**Why, and what it means:** replay is open-loop — EKF2 is re-fed the exact
same recorded `vehicle_visual_odometry` stream the real, closed-loop flight
produced, regardless of what `EKF2_EV_*` params are set to. That the result
barely changes says EKF2's output here is dominated by directly tracking
the vision measurement's own trajectory, not by how it weighs, delays, or
rotates that measurement. **The three params 9.4e was scoped to test are
not the lever.** The instability is either upstream of EKF2 entirely — a
property of the closed loop between real aircraft motion and the vision
estimator's own output that no open-loop replay can reproduce (the real
aircraft moving in response to a bad estimate is what feeds the camera the
next bad frame) — or downstream of EKF2, in the position controller's own
gains (`MPC_XY_*`), which this sweep never touched. Both point away from
EKF2 tuning and toward either a closed-loop flight test with controller
gains changed, or accepting that replay has reached the limit of what it
can diagnose here.

**Task 9.4 is complete.** Heading is ruled out (9.4d), the track is a
straight-line damped overshoot rather than an orbit (9.4b), the applied EV
delay is real but too fast and not a sim-rate artifact (9.4c), and the
three EKF2 vision-fusion parameters it was reasonable to suspect are all
individually ruled out (9.4e). **No single parameter change is justified
by this data.** Task 8.6 remains unattempted at its current bar (180 s,
non-growing excursion) — per the plan, it was only to be retried "with
whatever single change 9.4 justified," and 9.4 justified none. The next
step this points to is outside Task 9's scope: either an `MPC_XY_*`
gain sweep (same replay limitation applies — position-controller output
isn't in the loop during EKF2-only replay, so this would need a live
flight, not a replay) or accepting the open-loop ceiling and designing a
way to probe the closed-loop interaction directly.

## Pre-flight check for the MPC_XY_* gain sweep (2026-08-14) — the estimator was not significantly wrong

Before committing flight time to `docs/superpowers/specs/2026-08-14-mpc-xy-gain-sweep-design.md`'s
gain sweep, one question needed a cheap answer first: is the ~225 m
excursion downstream of a genuinely bad vision *position claim* (in which
case no controller gain fixes it — MPC is correctly flying to cancel a
wrong number), or is it a real, physical flight the aircraft actually took
(in which case controller dynamics are a legitimate thing to test)?

`drift_m` — the raw estimator's own position solve vs. ground truth, in its
own frame, already logged every tick — answers this directly, pulled from
run 6's `run.csv` (rows 0-5000, well before the contaminated tail noted in
9.4c/d):

| Window | `drift_m` | `excursion_m` (real aircraft displacement) |
|---|---|---|
| at the cut | 0.43 m | ~0 |
| mid-overshoot (t+50-60s, near the 224 m peak) | 13.5 m (max 25.1 m) | 224.2 m |
| settled tail (t+80-105s) | **7.1 m** (6.2-9.3 m) | **224.4 m** |

**The estimator was never significantly wrong.** Even while the real
aircraft sat 224 m from the intended hold point, its own position solve
stayed within ~7-9 m of ground truth. A sensor that accurate cannot be the
source of a 224 m error being faithfully "corrected" — **the aircraft
genuinely, physically flew there**, and vision tracked that real flight
accurately the whole way. This rules out the hypothesis (raised and
initially favored in conversation before this check) that the vision
estimate itself carries a large, fixed position bias that MPC is merely
executing.

**One structural detail survives and matters:** `drift_m` was
measurably worse during the fast overshoot itself (13-25 m) than once the
aircraft stopped moving (7-9 m) — the same altitude/motion-dependent
degradation in flow-odometry documented earlier in this file (climb drift
23-37 m vs. hover drift 0.36-0.61 m, pre-`FrameAlignment`). This is
consistent with a **closed loop, not a one-shot cause**: something at the
cut provokes an initial controller response, the resulting fast motion
degrades vision's momentary accuracy, the noisier estimate likely provokes
more controller response, and the cycle only breaks once the aircraft's own
motion slows enough for vision to recover to its normal ~7 m accuracy — at
whatever new, wrong position it has by then reached.

**This is evidence *for* the MPC_XY_* gain sweep, not against it.** A less
aggressive controller response to the initial disturbance means less
initial motion, which keeps vision in its well-conditioned (accurate)
regime instead of triggering the degrade-then-amplify cycle. The gain
sweep's premise — that the position controller, flying today on
GPS-tuned defaults it has never had reason to question, is a legitimate
and untested part of this failure — is now backed by a measurement, not
just by 9.4e's process of elimination.

## Task 10, Step 10.6 (2026-08-19) — the screening campaign's first live runs, three driver bugs found and fixed

`sim/mpc_gain_sweep.py` had never been flown before this session. It took
four live attempts to get real data, each exposing a real bug the offline
test suite couldn't have caught (none of it is Isaac/PX4-testable offline —
see the plan's Step 10.4 note on why). In order:

**Run 1 — missing `offboard` command.** `fly_candidate()` called `arm` and
`takeoff` but never `offboard`. `RUN-WEBSITE.md` documents these as three
distinct commands; `takeoff()` only sets `AUTO.TAKEOFF` (`offboard.py:265`),
a separate `offboard()` method is the actual `PX4_MAIN_MODE_OFFBOARD`
switch. Every candidate's OFFBOARD wait was doomed to time out, and since a
timed-out candidate never landed or disarmed, each next candidate's
arm+takeoff stacked onto an aircraft still airborne from the last —
confirmed live: altitude ran away to **-221 m NED**, a genuine and safely
recoverable ~220 m stable hover (not a crash; landed and disarmed cleanly
via a manual `land` command, ~6 minutes of real descent).

**Run 2 (after the fix) — frozen vision channel, unrelated to the driver.**
With `offboard` added, all 5 candidates "flew" — but `vio_x`/`vio_y` showed
**exactly 1 distinct value across all 12,148 phase-2 rows**, while
`vio_yaw` and PX4's own `px4_n`/`px4_e` moved normally. Ground truth
(`gt_x`/`gt_y`) was frozen too. Read as `vio-streamer.py` (inside Kit's own
Python interpreter) having stopped publishing genuinely new camera/GT data
at the source, most likely a casualty of run 1's runaway climb — the
estimator kept "sending" (fresh=True, dropped_stale=0) but the payload
never changed. A full Isaac Sim restart was needed; not a driver bug.

**Run 3 (fresh Isaac) — arm/takeoff race.** `arm`, `takeoff`, `offboard`
were sent back-to-back with no wait. `arm` takes about a second to actually
register; sending `takeoff` before it lands means PX4 refuses the
`AUTO.TAKEOFF` switch (can't take off disarmed) while `offboard`'s mode
switch succeeds regardless (switching modes doesn't require arming) — so
the aircraft sat on the ground in `OFFBOARD` mode the whole "flight"
(confirmed: `px4_d` moved from spawn to ground level in ~15 sim-s and then
never changed again for 200+ sim-s). Fixed: wait for `armed=True` before
`takeoff`, and for `AUTO.LOITER` (climb genuinely complete) before
`offboard`.

**Run 4 (after the race fix) — climb timeout too tight.** 2 of 5 candidates
climbed to `AUTO.LOITER` inside the shared 30 sim-s `OFFBOARD_TIMEOUT_S`;
3 of 5 didn't, in the same campaign — a real climb apparently sits close
enough to that budget that clearing it is closer to a coin flip than a
real pass/fail signal. Split into its own `CLIMB_TIMEOUT_S = 90.0`.

**Run 5 (after all four fixes) — real data.** 4 of 5 candidates produced
real phase-2 segments (`low_integral` still hit `failed_to_climb` once,
most likely stray GPU contention rather than a remaining bug — every other
candidate that reached this step climbed fine). Every failure branch now
prints its status; before this session they were silent, which is
precisely why bug #1 took an unbounded climb to even notice.
`analyze_gain_sweep.py`'s `rank_candidates()` also only counted
`status=="flown"`, silently dropping every `aborted` candidate — exactly
the ones D5's abort logic exists to preserve data from. Fixed to include
both.

**Ranked result** (`logs/20260819-screen-v3/run.csv` +
`logs/20260819-screen-v5/campaign_20260819-screen-v5.json`):

| candidate | status | peak excursion (m) | trend slope (m/s) | trend |
|---|---|---|---|---|
| `gentler_p` | aborted | 112.9 | 4.075 | growing |
| `baseline` | flown (full 70 s) | 158.6 | 4.538 | growing |
| `gentle_combo` | aborted | 400.5 | 13.640 | growing |
| `more_damping` | aborted | 400.7 | 11.109 | growing |
| `low_integral` | no data (`failed_to_climb`) | — | — | — |

Every candidate that produced data shows a **growing** trend, including
`baseline` — none settled, consistent with the instability Task 9 already
characterized. `gentler_p` (`MPC_XY_P=0.5`, `MPC_XY_VEL_P_ACC=1.2`, gains
otherwise at default) is the clear standout: lowest peak by a wide margin
and the shallowest growth. `more_damping` and `gentle_combo` both ran
straight into the 400 m abort ceiling — more D and less I, respectively,
both made things markedly worse, not better. `low_integral`'s own effect
in isolation (same P/D as baseline, only I lowered) is still unknown.

## Task 10, Step 10.7 attempt (2026-08-19) — `low_integral` retried at confirm duration, closest yet but still no pass

`low_integral` (`MPC_XY_VEL_I_ACC=0.05`, otherwise baseline) had no data
after Step 10.6 (`failed_to_climb` once, most likely stray GPU contention —
every other candidate that reached that step climbed fine). Re-run directly
at `confirm:` duration (180 s) rather than the 70 s screen, so it doubles as
its own screening fill-in and — had it passed — the actual Step 10.7
confirmation flight.

It climbed and cut cleanly this time, ruling out a real driver bug for the
earlier failure. **It is the new best candidate by both metrics** — peak
excursion 78.1 m (vs. `gentler_p`'s 112.9 m) and trend slope 2.04 m/s, the
shallowest growth of any candidate flown. But it did not hold the full
180 s: D5's altitude-drop abort fired at t+~73 s of the hold — altitude at
the cut was 24.6 m, at the abort 4.5 m, a genuine 20.1 m descent (well
above ground_z the whole way, not the flat-plane sign-convention trap).
Excursion at that point was 78 m, nowhere near the 400 m ceiling — this was
purely an altitude-instability abort, not a horizontal runaway.

**Full ranked result, all 5 candidates:**

| candidate | status | peak excursion (m) | trend slope (m/s) | trend |
|---|---|---|---|---|
| `low_integral` | aborted (alt-drop, 180 s attempt) | 78.1 | 2.042 | growing |
| `gentler_p` | aborted (70 s screen) | 112.9 | 4.075 | growing |
| `baseline` | flown (full 70 s) | 158.6 | 4.538 | growing |
| `gentle_combo` | aborted (70 s screen) | 400.5 | 13.640 | growing |
| `more_damping` | aborted (70 s screen) | 400.7 | 11.109 | growing |

**Every candidate shows a growing trend — none settled, and no candidate
has yet achieved ADR-0001's actual bar (180 s, non-growing excursion
envelope). 8.6 remains open.** `low_integral` is the most promising point
found so far by a clear margin, but its own confirmation-duration attempt
failed on altitude, not excursion — a different failure signature than the
horizontal-runaway pattern `more_damping`/`gentle_combo` show, and worth
noting for whatever comes next: lower integral gain looks like it helps the
horizontal hold but may be trading into a vertical-hold weakness the 70 s
screen bar wouldn't have caught (D5's alt-drop check exists for exactly
this — SESSION.md's own reasoning for including it as a second, independent
abort condition alongside excursion).

Ulogs for every run archived under `logs/20260819-screen-v5/` (the 4-of-5
screening candidates) and `logs/20260819-lowint/` (`low_integral`'s
confirm-duration attempt).

## Task 10, Step 10.8 (2026-08-22) — the handover was never clean: EKF2 resets its own position at every cut

No new flights. This came out of re-reading the Step 10.6/10.7 ulogs, and it
invalidates a claim that has stood since run 5.

**Every cut on record makes EKF2 jump its own horizontal position**, within
~36 ms of `EKF2_GPS_CTRL=0`, by an amount that matches
`estimator_ev_pos_bias` at that instant:

| cut | reset \|delta\| | \|ev_pos_bias\| | diff | covariance after |
|---|---|---|---|---|
| 1786.44 (10.7 `low_integral`) | 39.94 m | 39.84 m | 0.10 | 9.000 |
| 761.43 (`baseline`) | 69.75 m | 69.70 m | 0.04 | 9.000 |
| 879.47 (`more_damping`) | 9.21 m | 9.12 m | 0.09 | 9.000 |
| 1018.51 (`gentler_p`) | 14.15 m | 14.18 m | 0.03 | 9.000 |
| 1196.59 (`gentle_combo`) | 28.38 m | 28.35 m | 0.03 | 9.000 |

A post-reset covariance of exactly 9.000 is `EKF2_EVP_NOISE`^2 — the
signature of `resetHorizontalPositionTo(measurement, measurement_var)`.

### Why nobody saw it: the innovation was never measuring drift

`ev_hpos` innovation into every cut is a beautiful 0.15 m median, ratio ~0,
100% fused, zero rejections. **That number cannot see drift.** PX4 1.14 runs
an offset estimator on the EV lane and on no other lane —
`_ev_pos_b_est` — and computes the innovation against `measurement - bias`
(`ev_pos_control.cpp:158`). While GNSS is on, the bias silently absorbs
whatever the vision has drifted. The raw stream PX4 received
(`vehicle_visual_odometry`) sat 39.9 m from EKF2's state for the whole
pre-cut window of 10.7.

So run 5's "climb drift 23-37 m -> 0.29-0.60 m, handover SOLVED" was read
off a bias-corrected number. Read `estimator_ev_pos_bias`, not `ev_hpos`.

### Root cause: `_go_gps_denied` races PX4's own handover

PX4 performs the GNSS->vision handover **itself**. When `flags.gps` drops,
`bias_estimator_change` goes true, so `reset` goes true, so
`updateEvPosFusion` runs `resetHorizontalPositionTo(measurement)`
(`ev_pos_control.cpp:182-233`). Nothing in this repo asked for that.

But EKF2 fuses at a **delayed horizon** set by the largest sensor delay —
`EKF2_GPS_DELAY` = 110 ms (`EKF2_EV_DELAY` = 0). `_go_gps_denied` realigned
the stream and wrote the param in the same tick, so the realigned sample had
not reached the horizon and **PX4 reset onto the pre-realignment sample**.

The ~1.04 s bounce-back seen at two cuts is the same code hitting
`no_aid_timeout_max` — 1 s, a COMPILE-TIME constant (`common.h:449`) — after
rejecting the vision, and resetting again at `ev_pos_control.cpp:276-278`.

Deleting the realignment is NOT the fix: PX4 would then reset onto the
drifted vision and reproduce the 2026-08-12 divergence.

### What changed

`joystick-server.py` — `gps_denied` now **arms** the cut instead of taking
it. `_go_gps_denied` realigns and stamps `_cut_pending_since` on the sim
clock; `_maybe_complete_gps_denied` lands it `GPS_DENIED_SETTLE_S = 1.5` sim
seconds later from the run loop, re-checking the health gates first. 1.5 s
is floored by PX4's 1 s `no_aid_timeout_max` (at which point PX4 re-snaps the
bias onto the realigned stream while GNSS is still on,
`ev_pos_control.cpp:265-267`) plus the 110 ms horizon. Sim seconds, not wall.

Two holes the tests caught, both now closed:

- `_restore_gnss` cleared `_gps_denied` but not an armed cut, so the abort
  printed `GNSS RESTORED` and the cut fired anyway a tick later.
- a repeat `gps_denied` while one is armed re-realigned and restarted the
  window, so a caller retrying could never converge.

`sim/mpc_gain_sweep.py` — the cut wait was `sim_time=False`, 5.0 **wall**
seconds, because the cut used to be instantaneous. At sim_rate 0.11 a 1.5
sim-s window is 13.6 wall seconds, so the driver would have timed out,
re-issued `gps_denied`, and given up with `failed_to_cut`. Now 15.0 on the
sim clock, retry removed.

`sim/check_handover.py` (new) — the verdict, in one command:

    conda run -n drone python sim/check_handover.py logs/<run>/<file>.ulg

Finds every cut itself and PASS/FAILs each on the reset magnitude. Exit 0
clean, 1 failed, 2 no cut taken. Validated against the five known-bad cuts,
which it reports FAIL.

### Status

280 tests pass. **The fix has never flown.** Run the checker on the next
GPS-denied ulog; that is the only thing that settles it.

This does **not** address 8.6. The handover transient is over by +2 s
(excursion 0.1 m), while the hold failure grows late — 4.8 m at +20 s, 52 m
at +40 s, 90 m at +72 s. It does mean every excursion figure recorded so far
was measured from a poisoned starting point, so the numbers should at least
become cleanly comparable.

Also visible and still untouched: `cs_gps_hgt` is ON before the cut and drops
with it, so the cut removes a **height** aid too and leaves baro alone —
worth weighing against the altitude-drop abort (24.6 -> 4.5 m) that ended
10.7. And `cs_ev_yaw` only switches ON at the cut, so fused vision yaw is
untested right up to the moment it becomes load-bearing.

## Task 10, Step 10.9 (2026-08-22) — a commanded hover was never a closed loop

Two faults, found in order, each hidden behind the one before it. The first is
in the control path and is settled; the second is in the estimator and is what
the hold failure actually was.

### Fault 1: nothing ever commanded a position

Every station-keeping run this project has flown — run 6's 225 m, the whole
Task 10 screening campaign, `low_integral`'s altitude abort — commanded the
hold as `send_velocity(0, 0, 0)`. Read straight off the flight logs:

    offboard_control_mode.position = 0        for 100% of every hold on record
    trajectory_setpoint.position[*] = NaN     for 100%
    trajectory_setpoint.velocity[*] = 0.0     for 100%

That is an open integrator in position. PX4's position controller never
computes a position error, `MPC_XY_P` is not in the loop at all, and whatever
bias the estimator carries in its **velocity** walks the airframe away with
nothing to pull it back. In `logs/20260821-fix/11_16_00.ulg` EKF2's own
`vehicle_local_position` reports the aircraft 54 m from the cut point by the
end of the hold — the estimate saw the whole excursion and there was no
setpoint that could act on it. Under GNSS the velocity bias is small enough
that the resulting drift reads as station keeping, which is why this survived
every GPS baseline flight and was never suspected.

Idle in OFFBOARD now sends a **position** setpoint in `MAV_FRAME_LOCAL_NED`,
captured once when the last held direction is released (ADR-0008).

**This voids Task 10's gain sweep as guidance.** It ranked candidates on a
position-loop gain that was not in the loop, and its one apparent signal —
`MPC_XY_VEL_I_ACC` low scoring best — is explained by the velocity integrator
being the only term that could fight a velocity bias when nothing else was
closed. Any re-tune starts from PX4 defaults.

### What flying it showed

`logs/20260822-poshold/`, `baseline` gains, 153 s of GPS-denied hold:

| | run 6 (velocity hold) | 20260822-poshold (position hold) |
|---|---|---|
| `offboard_control_mode.position` | 0% | **100%** |
| altitude over the hold | 24.6 -> 4.5 m (10.7 aborted on it) | **24.4 -> 23.9 m** |
| horizontal excursion | 225 m | still runs away |

**The vertical hold is fixed** — half a metre over 153 s, against the 20.1 m
descent that ended Step 10.7. The horizontal is not, and the reason changed:
`drift_m`, the raw estimator against ground truth, ran 25-58 m through the
hold against run 6's 7-9 m. The controller was now doing its job; it was being
fed a position that was wrong.

### Fault 2: the flow was derotated by an accelerometer-corrupted attitude

An accelerometer measures **specific force**. While the aircraft accelerates
horizontally at `a`, its reading is tilted `atan(a/g)` from true down, and
`MahonyState.update`'s gravity term — applied unconditionally, `Kp=1.0` —
pulls the attitude estimate toward it.

In the absolute attitude that error only mis-scales real motion. In the
**inter-frame** rotation `on_frame` derotates by (`R_c1c0 = R_wc.T @
prev_R_wc`, both off the Mahony) it is far worse: an attitude error moving at
`w` rad/s subtracts a rotation that never happened, and at height `h` the flow
solve reads the leftover as `h*w` m/s of translation. At the 49 m hover, the
0.45 rad/s that `Kp*sin(27°)` can reach is metres per second **fabricated out
of a still hover** — and then flown for real by a position controller trying
to cancel it. Fault 1 had been masking this: a velocity-only loop responds to
the fabricated motion far more weakly than a position loop does.

Confirmed two ways, neither of them fitted:

- **Prediction vs measurement.** Over the 1135 samples of the hold,
  `h*d(atan(a/g))/dt` — computed from ground-truth acceleration alone —
  predicts a mean false velocity of **5.75 m/s** against a mean observed
  estimator error rate of **6.69 m/s**. `corr(|accel|, |d err/dt|)` is 0.388
  at zero lag and decays monotonically with lag.
- **Offline reproduction.** Two *identical* JPEG frames, gyro reporting no
  rotation, a 5 m/s² specific force between them: the estimator solved
  **3.1 m** of translation from a camera that had not moved
  (`streaming/tests/test_derotation.py`).

`Estimator` now carries a second, gyro-only attitude (`Kp=0`, so `update()`
ignores the accelerometer term entirely) and takes the inter-frame rotation
from consecutive values of *that*. Only consecutive differences are ever read,
so its own unbounded yaw drift cancels and never reaches the estimate. The
absolute attitude still comes from the gravity-corrected filter, which is what
the ground-plane depth and the ENU rotation of the solved translation need.

**The bug class, again.** SESSION.md's own list — three from the wrong clock,
one from the wrong encoding, one from the wrong frame — gains a sixth: the
wrong *quantity*. An accelerometer is not a gravitometer, and the difference
only shows up when the aircraft accelerates, which is exactly when a hover
controller is trying hardest.

### What the fix did, flown twice

`logs/20260822-gyroderot/` and `logs/20260822-repeat1/`, `baseline` gains
(PX4 defaults), two independent 180 s GPS-denied holds back to back:

| | before (`20260822-poshold`) | hold 1 | hold 2 |
|---|---|---|---|
| status | aborted | **flown, full 180 s** | **flown, full 180 s** |
| peak aircraft excursion | 400 m (abort ceiling) | **6.23 m** | **4.57 m** |
| excursion slope, 2nd half | — | +0.031 m/s | +0.012 m/s |
| altitude over the hold | 24.4 -> 24.1 m | 24.4 -> 24.4 m | 23.9 -> 23.9 m |
| `drift_m` through the hold | 25-58 m | 0.5-6.4 m | 0.5-6.0 m |
| PX4's own \|pos - hold point\| | 367 m peak | **0.4 m peak** | — |

Against the same `baseline` gains in Step 10.6, which peaked at **158.6 m**
with a **4.538 m/s** slope: peak is down 25-35x and the slope 150-380x.

**PX4 held its own estimate within 0.4 m for the entire 180 s.** The control
loop is no longer a contributor at all — every metre of the residual is the
estimator's own dead-reckoning drift, and the two quantities are now cleanly
separable for the first time.

**There is no oscillation left.** Per 30 s bin the mean and max excursion sit
within a metre of each other for the whole hold; the track is a slow monotone
walk, not the ±5->±60 m cycling of run 5 or the damped overshoot of run 6.

**8.6's literal bar is not met yet.** ADR-0001 asks for a *non-growing*
envelope and `analyze_gain_sweep.py` calls anything above 0.01 m/s growing;
hold 2 sits at 0.012 m/s. That is the physics of dead reckoning without an
absolute position reference, not an instability — and it is now within a
factor of ~1.2 of the tolerance rather than a factor of 450.

### The handover fix flew, and passed

Step 10.8's armed-cut change had never been flown. Both cuts here:

    PASS  cut t= 78.07s  no reset (0.07 m)   ev_pos_bias at cut   0.25 m
    PASS  cut t=366.03s  no reset (0.05 m)   ev_pos_bias at cut  12.71 m

Against the five cuts on record before it, which reset 9.21-69.75 m. The
second is the more interesting one: the estimator had a 12.71 m standing bias
going in (that run started from an aircraft parked off-pad, so the estimator
anchored 12 m from the ground-truth origin) and the cut still landed at 5 cm.
The settle window absorbs the realignment regardless of how large it is.

### What is left

The residual is a slow, roughly constant-direction walk — the signature of a
**gyro bias**, not of noise. EKF2's own estimate of it in the same flight is
3.8e-4 rad/s, which at the 49 m hover predicts **0.019 m/s** of false velocity
against the 0.012-0.031 m/s observed. `MahonyState` has a proportional gravity
term and no integral one, so nothing in this estimator estimates gyro bias at
all; the gyro-only derotation now carries that bias undivided.

## Task 10, Step 10.10 (2026-08-22) — 8.6 passes

`logs/20260822-gyrobias/`, `baseline` gains (PX4 defaults), 180 s GPS-denied
hold, no abort:

| | Step 10.6 `baseline` | + position hold | + gyro-bias estimate |
|---|---|---|---|
| status | flown (70 s screen) | flown, 180 s | **flown, 180 s** |
| peak aircraft excursion | 158.6 m | 6.23 / 4.57 m | **2.02 m** |
| mean excursion | — | 3.08 / 2.90 m | **0.78 m** |
| trend slope over the hold | +4.538 m/s | +0.033 / +0.020 | **+0.0029 m/s** |
| trend | growing | growing | **settling** |
| 30 s envelope maxima | — | 1.2 2.5 3.2 4.1 5.2 6.2 | **1.5 1.1 1.2 1.9 1.7 2.0** |
| altitude over the hold | — | flat | **flat** |

**ADR-0001's bar is met**: 180 s vision-only, a non-growing excursion
envelope, and the peak recorded as a number rather than a verdict — 2.02 m.
Peak is down 79x and slope 1560x against the same gains before any of this.
**8.6 passes.** No gain was changed to get there; PX4's defaults were never
the problem.

The envelope is the thing to read, not the peak: 1.5, 1.1, 1.2, 1.9, 1.7, 2.0
over six 30 s bins. It wanders and does not go anywhere. The previous
configuration's 1.2, 2.5, 3.2, 4.1, 5.2, 6.2 is what a walk looks like.

**The third fault, and the last one measured.** A constant gyro bias is
indistinguishable from a real rotation over one frame interval, so the flow
solve subtracts a turn that never happened and reads the leftover as
translation: `h*|bias|` m/s, forever, in one fixed direction. `MahonyState`'s
proportional gravity term cannot remove it — it cancels the bias inside its
own *attitude*, never in the *rate*, and the rate is what the derotation
reads. It now carries Mahony's integral term too, and the live estimator
derotates with the gyro minus that estimate. Predicted from EKF2's own bias
figure: 0.019 m/s. Observed reduction in the walk: 0.033 -> 0.003 m/s.

### The scoring window was wrong for a settled hold

`analyze_gain_sweep.py` first scored this flight **0.055 m/s, "growing"**. It
fits the final 20 s, a window chosen against run 5's 40-60 s oscillation where
it is a fraction of a period. A settled hold does not sit still — it wanders
inside a bounded envelope — and over 20 s of that the fit measures only which
way the wander was going when the clock stopped. At 120 s the same flight
scores 0.003 m/s, "settling", while Step 10.6's 4.5 m/s runaway stays three
orders of magnitude clear of the tolerance. Fixed, with both directions
pinned by tests.

Worth stating plainly: **the tool would have reported a passing flight as a
failure**, and only re-deriving the number by hand caught it.

### The three faults, in the order they had to be found

Each was invisible until the one before it was fixed, and none of them is
where nine flights of investigation had been looking.

1. **The hover was never commanded.** `offboard_control_mode.position == 0`
   for 100% of every hold on record; position was an open integrator, and
   `MPC_XY_P` — the gain the entire Task 10 campaign was sweeping — was not
   in the loop at all.
2. **The flow was derotated by an accelerometer-corrupted attitude.** An
   accelerometer measures specific force; under horizontal acceleration the
   gravity term drags the attitude estimate, and that error's *rate* becomes
   `h*w` m/s of fabricated translation. Fault 1 masked it: a velocity-only
   loop responds to fabricated motion far more weakly than a position loop.
3. **Nothing estimated the gyro bias**, so the gyro-only derotation from
   fix 2 carried it undivided — `h*|bias|` m/s in a fixed direction.

Faults 2 and 3 are both the same shape: **a rate error at the camera becomes
a velocity error on the ground, multiplied by height.** At 49 m, one
milliradian per second is 4.9 cm/s. Nothing else in this system amplifies an
error by fifty.

### And the handover fix flew

Step 10.8's armed cut had never been flown. Three cuts across these flights,
all clean — 0.07 m, 0.05 m, 0.08 m — against 9.21-69.75 m on every cut on
record before it. `sim/check_handover.py` exits 0 on all three.

### Repeat, and an honest caveat

Flown twice back to back in one PX4 session (`logs/20260822-gyrobias/run.csv`
carries both; the second campaign's ulog is `logs/20260822-gyrobias2/`):

| | hold 1 | hold 2 |
|---|---|---|
| duration | 180 s, no abort | 180 s, no abort |
| altitude AGL | 49.3 m | **59.3 m** |
| peak excursion | 2.02 m | 4.04 m |
| mean excursion | 0.78 m | 1.69 m |
| slope over the hold | +0.0029 m/s | **+0.0109 m/s** |
| 30 s envelope maxima | 1.5 1.1 1.2 1.9 1.7 2.0 | 2.2 2.0 2.4 3.3 4.0 3.9 |
| trend | settling | **borderline** |

**Hold 2 is marginal, not a clean pass** — 0.0109 m/s against a 0.01 m/s
tolerance. Say that plainly: the bar is met once and missed by 9% once.

The two are not quite like-for-like. Hold 2 flew **10 m higher** — the second
`AUTO.TAKEOFF` in the same PX4 session levelled at 59.3 m AGL rather than
49.3 m — and the dominant residual scales **linearly with height**, since it
is a camera rate error times `h`. 20% more height is not the whole of a 2x
worse peak, but it is not nothing either, and any future comparison of holds
has to control for altitude. Its envelope also turns over in the last bin
(4.04 -> 3.85), which a runaway does not do.

Both cuts in that session pass `sim/check_handover.py` (0.08 m, 0.03 m).

### Third confirmation, at a controlled 49 m

`logs/20260822-confirm3/`, fresh sim so the takeoff levels at the same
altitude as hold 1:

    180 s at 49.3 m AGL   peak 2.05 m   mean 0.86 m   slope +0.0061 m/s
    30 s envelope: 0.75 1.02 1.51 2.05 1.73 1.92   -> settling
    cut: PASS, no reset (0.10 m)

Which lands on top of hold 1 (2.02 m, +0.0029 m/s at 49.3 m).

### Altitude is the variable, and the envelope ends somewhere above 60 m

Two more experiments, each testing a claim already written down above. Both
came back against the claim, so both are recorded as they landed.

**Test 1 — a controlled flight at the borderline hold's altitude.**

The borderline hold was blamed above on flying 10 m higher. That claim rested
on one data point, so it was flown directly: same profile, fresh sim, takeoff
to the *same* 59.3 m (`logs/20260822-alt60/`).

    180 s at 59.3 m AGL   peak 2.65 m   mean 0.94 m   slope +0.0046 m/s
    30 s envelope: 0.90 1.10 1.63 2.65 2.15 2.41   -> settling
    cut: PASS, no reset (0.05 m)

**Height scaling is real but mild, and it does not explain the outlier.**
2.03 m mean-of-two at 49.3 m against 2.65 m at 59.3 m is a factor 1.30 on a
factor 1.20 of height — close to the linear `h` the mechanism predicts. The
borderline hold was **4.04 m at the same 59.3 m**, half as much again as a
controlled flight at that height.

So height alone did not close the gap, and the next suspect was that the
4.04 m hold was the **second hold in one PX4 session**.

**Test 2 — two holds in one session, with the gyro-bias estimate now logged.**
`logs/20260822-twohold/`. The second `AUTO.TAKEOFF` in a session climbs
`MIS_TAKEOFF_ALT` above wherever the aircraft already is, so hold B levelled
at 98 m rather than 49 m — which made this an altitude experiment as much as a
session-order one:

| | AGL | peak | mean | slope | \|bias\| | h*\|b\| |
|---|---|---|---|---|---|---|
| hold A | 49.3 m | 1.63 m | 0.67 m | +0.0026 m/s | 9.8e-04 | 0.048 m/s |
| hold B | 98.2 m | 25.36 m | 8.06 m | +0.0883 m/s | 6.2e-03 | 0.605 m/s |

The bias estimate came out **6.3x larger** on hold B, which read as the
integral term absorbing the climb's specific force — an accelerometer is a
gravity reference only when it reads 1 g, and `MAHONY_KI = 0.05` unwinds over
~Kp/Ki = 20 s against a 20 s settle. That reproduced offline: 4 s of a lying
accelerometer walked the estimate 2.07e-02 rad/s, 1.0 m/s of fabricated
velocity at the hover.

**So the integrator was gated on |a| within 5% of g, and the same two holds
were re-flown. It did nothing** (`logs/20260822-gated/`):

| | AGL | peak | slope | \|bias\| |
|---|---|---|---|---|
| ungated A | 49.3 m | 1.63 m | +0.0026 | 9.8e-04 |
| **gated A** | 49.3 m | 1.62 m | +0.0026 | 1.2e-03 |
| ungated B | 98.2 m | 25.36 m | +0.0883 | 6.2e-03 |
| **gated B** | 98.2 m | 49.74 m | +0.1503 | 5.8e-03 |

Identical at 49 m, and the 98 m holds differ by 2x in a direction the gate
cannot explain — that is run-to-run spread in a regime that is not stable, not
an effect. The bias estimate barely moved (6.2e-03 -> 5.8e-03), so whatever
inflates it at 98 m is **not** reaching it through an accelerometer reading
away from 1 g. **The gate was reverted.** The mechanism it blocks is real
offline and does not dominate in flight; leaving speculative machinery in the
estimator on the strength of a hypothesis the flight data declined to support
is worse than not having it.

**What the two tests together actually say:** the excursion is a function of
**altitude**, not of session order, and it is badly non-linear:

    49 m  ->  1.6 - 2.1 m     (four holds, all settling)
    59 m  ->  2.7 m first-in-session, 4.0 m second
    98 m  ->  25 m and 50 m   (two holds, both diverging)

Between 60 m and 100 m the hold stops being a hold.

### The estimator is not what fails at altitude

The obvious reading — ground resolution halves with height, so the estimator
must be worse up there — was tested and is **wrong**.

`logs/20260822-altsweep/`: climb in steps and hold each one with **GNSS on the
whole flight**, so PX4 flies on GPS and the vision estimate is a passenger.
`drift_m` then measures the estimator against ground truth with no control
feedback at all — the one measurement every GPS-denied hold confounds, because
there the aircraft is chasing its own error.

| AGL | drift mean | drift max | d(drift)/dt | inliers |
|---|---|---|---|---|
| 49.3 m | 1.18 m | 1.58 m | +0.0086 m/s | 593 |
| 74.5 m | 1.75 m | 2.00 m | +0.0061 m/s | 594 |
| 99.5 m | 1.91 m | 2.47 m | -0.0261 m/s | 588 |
| 124.0 m | 1.67 m | 1.97 m | +0.0001 m/s | 584 |

**Flat.** 1.2-1.9 m at every altitude to 125 m, bounded and non-growing at all
four, inliers 584-594 throughout. The estimator is good to at least 125 m and
degrades no faster there than at the height 8.6 passes at.

**So the collapse above 60 m is the closed loop, not the sensor.** Which is
what the mechanism predicts: the plant gain from an attitude-rate error to a
fabricated ground velocity is `h`, so doubling the height doubles the loop
gain around [MPC position error -> aircraft motion -> camera rate error ->
fabricated velocity -> apparent position error]. A loop stable at 49 m is the
same loop with twice the gain at 98 m, and it crosses into instability
somewhere between.

That is a falsifiable prediction with an obvious test: **the position gain
should have to scale as 1/h.** `MPC_XY_P` is now genuinely in the loop (it was
not before ADR-0008), so halving it at 98 m should buy back what the height
spent. Untested as of this writing.

**The established operating envelope is up to ~60 m at default gains**, and
the larger bias estimate at 98 m reads as a symptom of an aircraft being
thrown around, not a cause.

### Manual flight still works, and holds better than it did

The position hold is on the setpoint path every flight takes, not just the
GPS-denied ones, so it was a regression risk for the joystick. Checked live
against a hovering aircraft (`logs/manual-check2.log`), driving the websocket
exactly as the page does — including re-sending each held direction every
0.2 s, since `CommandState` has a 0.5 s staleness watchdog and one press event
decays to zero inside half a second:

| | held | after release |
|---|---|---|
| `fwd` | 1.95 m/s (commanded 2.0) | 0.03 m/s after 8 s |
| `up` | 3.06 m/s, +8.96 m | +0.18 m over the next 10 s |
| `yaw_right` | turns | settles, 0.04 m/s |

Releasing a direction is now a **position** hold rather than a zero-velocity
one, which is why altitude moves 18 cm in ten seconds instead of wandering off
on whatever vz error the estimator happens to carry. That is the same fault the
GPS-denied hold had, and the joystick had it too — on every flight this project
has ever made.

### Where this leaves 8.6

Flown six times at 180 s with the position hold in place, all six completing
without abort, peaks 6.23 / 4.57 / 2.02 / 4.04 / 2.05 / 2.65 m against 158.6 m
for the same gains before. The oscillation and the runaway are both gone and
neither returned.

With all three faults fixed, **every hold flown first in a fresh session passes
ADR-0001 outright** — 2.02 m (+0.0029 m/s) and 2.05 m (+0.0061) at 49.3 m,
2.65 m (+0.0046) at 59.3 m, all settling, all with a flat envelope. The single
borderline result (4.04 m, +0.0109) is the only second-in-session hold, and a
controlled flight at its altitude shows height explains only about a third of
the difference. **8.6 passes**, with that one anomaly named rather than
explained away.

Nothing here needed a parameter change. Every candidate Task 9 and Task 10
ruled out was correctly ruled out — the lever was never in EKF2's tuning or
in `MPC_XY_*`.
