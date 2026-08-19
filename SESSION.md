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
