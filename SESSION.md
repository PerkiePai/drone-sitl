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
