# Session notes: VIO GPS-denied live bring-up (plan Task 8)

**Date:** 2026-08-11
**Plan:** `docs/superpowers/plans/2026-08-07-vio-gps-denied.md`, Task 8
**Result:** 8.1–8.5 pass. GPS-denied flight ACHIEVED and repeatable, but it
does not hold station for 60 s, so 8.6 does not pass. Read top to bottom:
four runs, each fix exposing the next fault. The root cause of the transport
faults was INT32 param encoding; what remains is estimator drift.

Run configuration:

```bash
DRONE_SETUP_DOWN_VIB_DAMP=False VIO=1 ./sim/launch-sitl.sh
python pipeline-streaming.py --scale 0.5 --print-every 15
python joystick-server.py --takeoff-alt 50 --speed-up 3.0 [--vision --site bangkok-survey-040]
```

**Superseded 2026-08-13 — two commands, not three:**

```bash
DRONE_SETUP_DOWN_VIB_DAMP=False VIO=1 ./sim/launch-sitl.sh
python joystick-server.py --takeoff-alt 50 --speed-up 3.0
```

`joystick-server.py` now spawns and supervises `pipeline-streaming.py` itself,
`--site` defaults to the launcher's own default, and `--vision` defaults ON.
Isaac stays a separate command and always will — `vio-streamer.py` runs inside
its physics callback, not as a process anything here could spawn.

`--no-vision` gives an ordinary GPS flight that touches no EKF2 params;
`--no-estimator` keeps vision but leaves the estimator to you. **The default
now reboots PX4 once at startup** (phase 0, `EKF2_HGT_REF`) on every run,
including ones where you only wanted GPS — that is the cost of the one-command
default and the startup banner says so.

Offline first: **168/168 tests pass** in the `drone` conda env. (They fail in
*base* conda purely for want of `websockets`/`msgpack` — wrong interpreter, not
a real failure. Use `~/miniconda3/envs/drone/bin/python`.)

## What passed

- **8.1** Stage built for `bangkok-survey-040`, drone spawned, Play pressed,
  MJPEG on 8080. `>>> override DOWN_VIB_DAMP = False` confirmed in the log.
- **8.2** Streamer up on `tcp://*:5556`, ~14.7 img/s. `meta` verified **over the
  wire** (not just from the log) by the estimator's own prime line:
  `site=bangkok-survey-040 vib_damp=False heading=0.0`. Zero `poll timeout`
  lines from PX4 all session.
- **8.3** Estimator drift vs GT while hovering at 49 m: **0.36–0.61 m**, ~595–600
  LK inliers, steady 8.8 fps. Bounded, not growing.
- **8.4** **GPS baseline flies.** Armed, `AUTO.TAKEOFF` to 49.2 m, held station
  (`vz≈0.01`, `gs≈0.01`), `home_valid=True`, `ready_for_offboard=True`.
  Nothing regressed.
- **8.5 (partial)** The vision wiring itself is correct: EKF2 param set applied,
  `GPS origin (13.66156872, 100.298235, 0.0)` sent, and `GLOBAL_POSITION_INT`
  kept producing a map position with GNSS fusion off — **the origin landed**.
  Task 7's telemetry row is live and correct end-to-end:
  `vio={'fresh': True, 'age_s': 0.02, 'n_inliers': 600, 'drift_m': 1.31,
  'fps': 8.81, 'sent': 749, 'dropped_stale': 0}`.

`sim_rate` held ~0.60 with the streamer running (Isaac + estimator + OpenSfM job
sharing the box). Nothing stalled at that rate.

## Blocker: `EKF2_HGT_REF` is reboot-required, and it is not optional

`vision_bridge.py:46`'s `HGT_REF_NEEDS_REBOOT` warning is not a nicety — it is
the thing that stops GPS-denied flight from working when `--vision` is applied
to an already-booted PX4. Observed live:

```
*** EKF2_HGT_REF is @reboot_required: PX4 now STORES the new value
    (QGC will show it) but keeps its old height reference until it restarts. ***
```

With `EKF2_GPS_CTRL=0` taking effect immediately and `EKF2_HGT_REF=0` **not**
taking effect, EKF2 is left with GNSS fusion off and its height reference still
pointing at GPS. The aircraft then descended without bound — reported altitude
ran 49 m → 12 m → 0 → **−22 m and still falling** at a constant −0.7 m/s, with
`AUTO.LAND` latched. Both `offboard` and `disarm` were refused throughout.

The estimator was healthy the whole way down (`n_inliers` 600, `drift_m` ~2.3,
`dropped_stale: 0`), so this is **not** a VIO quality problem. It is purely the
param-ordering one the code already predicted in a comment.

The param set is applied at `joystick-server.py`'s connect time, which is by
construction *after* PX4 has booted. So under the current design
`EKF2_HGT_REF=0` can never take effect on the run that needs it.

## New finding: the nadir camera is blind near the ground at this site

Not in the plan, and it constrains the whole GPS-denied flight profile.

| Drone state | `/down` frame | LK features |
|---|---|---|
| Parked, 0.5 m AGL | mean 1.0, **std 0.00** | **0** |
| Hovering, 49 m | mean 163.5, std 46.7 | **600** |

At the 0.5 m AGL spawn the down camera sits below the Cesium tile surface and
renders flat near-black, so **a vision-only takeoff from the ground is not
possible at `bangkok-survey-040`.** Any GPS-denied profile here has to reach
altitude first. Confirmed from the other side too: as the aircraft sank back
through the terrain, inliers collapsed 600 → 282 → 139 → 37.

### This corrects the "Isaac render needs a stream client" note

That note said the `down_cam` render product goes all-black until a livestream
client attaches to 8080. **Altitude, not a stream client, is the variable here.**
In the same session, with no browser attached anywhere:

- `/detect` rendered real content (mean 143.1, std 83.5) — so the renderer was
  live and no client was needed for it.
- `/down` stayed black (mean 1.0, std 0.00) *with* an MJPEG client actively
  pulling it, and came alive only on climbing.

The `/detect` frame at ground level shows the mechanism directly: terrain in the
far field, a black wedge in the near field where the nadir camera is looking.

The note's underlying advice still stands and is what caught this:
**check a frame's std is non-zero before trusting any VIO run.**

## Where this leaves Task 8

- [x] 8.1 spawn / Play / MJPEG
- [x] 8.2 streamer meta
- [x] 8.3 estimator drift bounded while hovering — **0.36–0.61 m**
- [x] 8.4 GPS baseline
- [~] 8.5 params + origin land and the VIO row works; `EKF2_HGT_REF` does not
- [ ] 8.6–8.9 blocked: need `EKF2_HGT_REF=0` in **before PX4 boots**

Candidate fixes, none yet chosen:

1. **Set the param, then reboot the FC** (`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`)
   while disarmed, before takeoff, and re-apply the rest. Stays inside this repo.
2. **Set it at PX4 boot** from the launcher (airframe / startup script), so the
   run begins with the right height reference. Touches the PX4 checkout.
3. **Two-phase profile**: take off on GPS above the tile surface, *then* hand
   over to vision. Needed at this site anyway because of the blind-nadir finding
   above — but it still requires fix 1 or 2, since `EKF2_HGT_REF` is what the
   handover depends on.

## Second run, after the phase split: the height fix works, and it uncovered the next one

Reflown 2026-08-11 with the three-phase sequence. **The `EKF2_HGT_REF` blocker
is fixed** — verified live, in order:

```
>>> vision: set EKF2_HGT_REF and rebooting PX4 so it takes effect
>>> vision: fusion params applied, GPS origin (13.66156872, 100.298235, 0.0)
>>> vision: GNSS still ON. Climb until the VIO row shows a healthy inlier count
```

PX4 rebooted, the params resumed on the fresh instance, and the aircraft armed
and climbed **on GPS** to a stable hover — `vz≈0.00`, `gs≈0.01`, 600 inliers,
drift 0.36–0.61 m. The blind-nadir effect was visible in passing exactly as
predicted: inliers ran 44 → 304 → 440 → 600 during the climb off the pad.

`gps_denied` was then accepted on its own terms:

```
>>> GPS-DENIED: GNSS fusion off, flying on vision (581 inliers, drift 0.42)
```

**And PX4 immediately fell out of OFFBOARD into ALTCTL and began descending at
0.70 m/s.** Re-commanding OFFBOARD was refused from then on. So EKF2 did *not*
accept the vision stream as a position source: with GNSS cut it had no
horizontal position at all, and dropped to the one mode that needs only baro.

### CORRECTION: the timestamp is not the cause

The diagnosis below was **wrong**, and is kept only so the reasoning can be
checked. Read the PX4 source before believing it:

```c
// src/lib/timesync/Timesync.cpp:127-136
uint64_t Timesync::sync_stamp(uint64_t usec)
{
	if (sync_converged()) {
		return usec + (int64_t)_time_offset;
	} else {
		return hrt_absolute_time();      // <-- our case
	}
}
```

`sync_converged()` is `_sequence >= CONVERGENCE_WINDOW`, and
`CONVERGENCE_WINDOW = 500` (`Timesync.hpp:76,120`). `_sequence` only advances
inside the TIMESYNC round-trip handler (`Timesync.cpp:93`). **Neither pymavlink
nor anything in this repo sends or answers TIMESYNC** — zero occurrences in
either — so `_sequence` never leaves 0, and
`mavlink_receiver.cpp:1213`'s `sync_stamp(vpe.usec)` returns PX4's own arrival
time for every single estimate.

So PX4 was already discarding the sim-time stamp and substituting its own.
Sending a corrected timestamp would change nothing. Whatever stopped EKF2
fusing vision, it is not this.

What made the wrong diagnosis look right: the mechanism is real in general
(the two clocks genuinely are on different epochs), and every other number in
the run was healthy, so the one unverified quantity attracted the blame. The
lesson is the ordinary one — the counter nothing checks is not thereby the
guilty one.

### The superseded diagnosis, kept for the record

Not yet confirmed against `ESTIMATOR_STATUS`, but the mechanism is concrete and
the evidence fits:

- `vio-streamer.py:216` takes `now = world.current_time` — **Isaac sim time,
  counted from Play** — and `vio-streamer.py:229` stamps every sample
  `ts_ns = int(now * 1e9)`. That timestamp is carried unchanged through the
  estimator (`pipeline-streaming.py:139`) and into
  `VISION_POSITION_ESTIMATE.usec` (`vision_bridge.py:126`).
- EKF2 interprets that field on **PX4's own `hrt_absolute_time()` clock**,
  counted from PX4 boot. The two epochs were never aligned, and the phase-0
  reboot now moves PX4's epoch *again*, mid-session.
- Samples that far outside the fusion window are discarded, which is consistent
  with everything observed: 5,597 VPE messages sent, a healthy 546–600 inliers
  throughout, and still no position once GNSS went away.

`dropped_stale: 0` is not evidence against this — `VisionPositionSender.send()`
judges staleness with wall-clock `received_at` (`vision_bridge.py:121`), by
deliberate design, so it never inspects the sim stamp it forwards. The one
number that would have caught this is the one nothing checks.

This was invisible before the height fix: the aircraft never survived long
enough on the previous run to test whether vision was actually being fused.

### Where Task 8 stands now

- [x] 8.1–8.4, unchanged
- [x] **The phase split and the PX4 reboot work** — verified live end to end
- [~] 8.5 params, origin, reboot ordering and the VIO row all good
- [ ] **8.6–8.9 still blocked**, now on timestamp alignment rather than on
      `EKF2_HGT_REF`

Next step is **not** the timestamp (see the correction above). PX4 already
stamps VPE on arrival. The open question is why EKF2 does not fuse the vision
source even in phase 1, while GNSS is still up and the estimate is healthy —
which has to be answered by reading `ESTIMATOR_STATUS` rather than by
reasoning about it from outside.

## Third run: the real root cause was INT32 param encoding

`set_param` sent INT32 params as `float(value)`. PX4 does not convert -- for an
INT32 param it reinterprets the PARAM_SET float field's **raw bytes**:

```c
param_set(param, &(set.param_value));   // mavlink_parameters.cpp:134
```

So `float(9)` stored **1091567616**, the bit pattern of `9.0f`, and PX4 accepted
it silently because that is a perfectly legal int32. Read back off a live PX4:

| Param | PX4 held | Intended |
|---|---|---|
| `COM_RCL_EXCEPT` | 1082130432 | 4 |
| `EKF2_EV_CTRL` | 1091567616 | 9 |
| `EKF2_EV_NOISE_MD` | 1065353216 | 1 |
| `EKF2_GPS_CTRL`, `EKF2_HGT_REF` | 0 | 0 ✓ |

**Only the zero-valued ones worked**, `0.0f` and int `0` sharing a bit pattern.
That asymmetry is what hid the bug for so long: every param that *disabled*
something worked, and every param that *enabled* something was dead. So GNSS
really did get cut, vision fusion never actually switched on, and the aircraft
lost its position exactly as if vision were broken.

It also means **`COM_RCL_EXCEPT` was never 4 on any flight this repo has ever
made** -- OFFBOARD was never actually exempted from the RC-loss failsafe.

Fixed in `offboard.py:set_param`, which now sends the bit pattern for INT32.
Verified live: `EKF2_EV_CTRL : 9`, `COM_RCL_EXCEPT : 4`, and for the first time
`cs_ev_pos: True` / `cs_ev_yaw: True` in `estimator_status_flags`.

### The timestamp was a red herring

The previous run blamed VPE timestamps. That was wrong, and the correction above
has the source. Worth keeping as a lesson: the unverified quantity attracted the
blame precisely because everything else looked healthy.

## What the fix then exposed, in order

Each of these was invisible until the one before it was fixed.

1. **Fusing a blind camera stops the aircraft arming.** With EV fusion genuinely
   on, the pad camera (0 inliers, see above) gave PX4
   `Preflight Fail: Yaw estimate error / heading estimate not stable`.
   Fixed: fusion is now deferred until the estimate clears
   `VISION_MIN_INLIERS`, which needs altitude.

2. **PX4 params persist across runs AND across reboots.** Deferring the fusion
   params in code changed nothing, because the previous run had already saved
   `EKF2_EV_CTRL=9`. Not setting a param is not the same as it being off.
   Fixed: startup now *asserts* GPS flight (`EKF2_EV_CTRL=0`,
   `EKF2_GPS_CTRL=7`). The dangerous direction is the other one -- a saved
   `EKF2_GPS_CTRL=0` would have the next run boot GPS-denied on the pad.

3. **The param re-send undid fusion mid-flight.** `_send_startup_params` is also
   the recovery path (`_check_px4_restart` re-runs it on a heartbeat gap), so
   asserting GPS flight there switched EKF2 back off vision at altitude.
   Fixed: the re-send now re-asserts whichever phase is actually current.

4. **`PX4_RESTART_GAP_S` was measured in wall time against a sim-time
   heartbeat.** PX4 SITL is lockstepped to Isaac, so its 1 Hz heartbeat arrives
   every `1/sim_rate` wall seconds -- ~1.8 s at the 0.55 this box runs. The old
   3 s threshold was barely 1.5 beats and tripped on ordinary jitter, which is
   what kept firing (3). Raised to 10 s.

## GPS-denied flight achieved, then lost

With all of the above fixed, the full sequence ran end to end:

```
>>> vision: FUSING alongside GNSS (573 inliers). `gps_denied` is now available.
>>> GPS-DENIED: GNSS fusion off, flying on vision (444 inliers, drift 24.4)
```

PX4 confirmed it: **`cs_gps: False`, `cs_ev_pos: True`, `EKF2_GPS_CTRL: 0`,
`reject_hor_pos: False`** — genuinely GPS-denied, holding OFFBOARD at ~24 m with
`vz≈0.00` and `gs` 0.07–0.48. That is the first time this repo has flown without
GNSS.

**It held for roughly 40 s and then diverged.** Altitude ran 24.9 → 23.3 → 19.1
→ 13.3 → −7.5 → −31.3 with `vz` reaching −20 m/s, ground speed to 12.7 m/s, and
estimator drift 28 → 230 → 352 m. So **8.6 does not pass**: the bar is holding
station for 60 s, and it did not.

### The next thing to fix, and it is the same bug class again

`dropped_stale` climbed all the way down: 15 → 40 → 54 → 63 → 110 → 122 → 179,
while `sim_rate` collapsed 0.56 → 0.36 → 0.26 → 0.11.

`VisionPositionSender.send()` judges staleness against **wall clock**
(`vision_bridge.py:121`, `max_age_s=0.5`) while everything producing those
estimates runs on **sim time**. When the sim slows, healthy estimates arrive
late in wall terms and get dropped as stale -- at precisely the moment they are
the aircraft's only position source. EKF2 starves, and the aircraft diverges.

That is the third wall-vs-sim-time confusion in this system, after
`PX4_RESTART_GAP_S` and the VPE timestamp theory. **It is worth auditing every
duration in the vision path for which clock it belongs to** rather than fixing
this one instance and waiting for the next.

Note the irony: `dropped_stale` is the counter the previous write-up dismissed
as "the one nothing checks". It was the smoking gun all along.

## Fourth run: the staleness fix works, and the remaining fault is the estimator

`VisionPositionSender` now ages estimates against PX4's `time_boot_ms` -- sim
time under lockstep -- instead of the wall clock. Flown 2026-08-12:

| | run 3 (wall clock) | run 4 (sim clock) |
|---|---|---|
| `dropped_stale` over the flight | 15 → 179 | **0 throughout** |
| `sim_rate` | 0.56 → 0.11 | 0.41–0.59, no collapse |

**That bug is fixed.** Not one estimate was dropped, at any point, including
while the sim was at 0.41.

**The aircraft still diverged, for a different reason.** `drift_m` -- the
estimator's own error against ground truth -- ran away:

```
23–37 m   at the moment GNSS was cut
60 → 77 → 318 → 704 → 1195 → 1801 → 2275 → 2544 m
```

with ground speed reaching 58 m/s and the estimator's own position at
`(1271, -1030)` when truth was near the origin. So **8.6 still does not pass**,
but the failure has moved out of the plumbing and into flow-odometry accuracy.

### Two things this exposes, neither of them a transport bug

1. **The estimate has already drifted tens of metres before the handover.**
   Hovering at 49 m in run 1 the drift was 0.36–0.61 m; after a 3 m/s climb to
   the same altitude it is 23–37 m. The drift is accumulated during the *climb*
   -- flow-odom solves translation against a ground plane at barometric height,
   and a fast climb is where that is worst conditioned.

2. **Nothing realigns the two frames at the cut.** EKF2 is handed a vision frame
   that already disagrees with where it believes it is by ~30 m, then follows
   it. The aircraft chases the drifting estimate, which moves the camera, which
   feeds more drift -- the runaway above. A handover that reset the vision
   origin to PX4's current position would start GPS-denied flight from zero
   error instead of from 30 m.

Worth being clear that (2) is a *design* gap in the handover, not a bug in
anything already written: the two-phase profile was specified before anyone had
flown far enough to see that the phases meet at a discontinuity.

### Where Task 8 stands

- [x] 8.1–8.5, and the phase split, the param encoding and the staleness clock
- [~] **8.6** GPS-denied flight is real and repeatable -- `cs_gps: False`,
      `cs_ev_pos: True`, holding OFFBOARD on vision alone -- but it does not
      hold station for 60 s, so the bar is not met
- [ ] 8.7–8.9 not attempted; they all sit downstream of a stable hover

Next, in order: realign the vision frame at handover (cheap, and (2) above is
the larger error by far), then look at climb-phase drift.

## The handover fix, and a second bug found while writing it

Both changes are in; **neither has been flown yet.** 2026-08-13.

### 1. The vision frame is now pinned onto PX4's at each phase transition

`vision_bridge.FrameAlignment` is a rigid transform from the estimator's frame
into PX4's local NED, re-solved by `align_to_px4` at fusion start and again at
the GNSS cut, then held fixed. Against the numbers above it closes the full
30.7 m and leaves the two frames agreeing to 0.00 m, while 40 m of subsequent
vision travel still arrives as 40 m — it is a rigid transform, not a snap.

Yaw is rotated as well as position. A translation-only fix would leave the
frames rotated against each other, and every metre flown after the handover
would then point a few degrees wrong — an error that *grows with distance*,
which is exactly the flight this has to survive.

Two deliberate choices:

- **Aligned at the transitions only, never every tick.** Re-solving
  continuously would feed EKF2 its own estimate back as an independent
  measurement: innovations would sit at zero by construction, EKF2 would gain
  confidence from a measurement carrying no information, and a completely
  broken estimator would look perfect right up until GNSS was cut. Phase 1b
  exists to prove the source in flight, which requires it to stay independent.
- **Both transitions refuse to proceed without a PX4 pose to align onto.**
  Fusing or cutting on an unaligned frame is the failure being removed, so it
  is not left as a fallback. Costs nothing live: `LOCAL_POSITION_NED` and
  `ATTITUDE` stream far faster than the camera clears the inlier gate.

`align_m` and `realigned` are in the `vio` telemetry block and on the flight
page, so the error being closed is visible at the moment of the handover
rather than inferred afterwards from a diverging track.

### 2. VISION_POSITION_ESTIMATE was carrying ENU attitude into an NED field

Found while writing the alignment, and it is not cosmetic. The estimator works
in ENU/FLU: `pipeline-streaming.py:132` takes yaw as
`arctan2(R_flu[1,0], R_flu[0,0])`, the heading of the body-forward axis
measured **from east, counter-clockwise**. `VisionPositionSender` converted the
*position* to NED and passed roll/pitch/yaw through untouched — into a message
PX4 reads as NED/FRD. So PX4 was told a number meaning "90° east-relative"
where it expected "degrees clockwise from north".

The error is `90 - 2·heading` degrees:

| heading | yaw sent | error |
|---|---|---|
| 0 (N) | 90 | **90** |
| 45 | 45 | 0 |
| 90 (E) | 0 | **90** |
| 135 | -45 | **180** |
| 180 (S) | -90 | **90** |

It vanishes at heading 45° and 225° — the reflection axis — and is a full 180°
at 135°/315°. `EKF2_EV_CTRL=9` sets bit3, so this yaw *was* being fused: EKF2
was told the aircraft faced somewhere it did not, and every horizontal
correction derived from vision was applied in the wrong direction. That is a
second, independent mechanism for the runaway, and it is heading-dependent,
which fits an aircraft that diverged rather than merely sat offset.

Pitch was mirrored too (FLU vs FRD). It matters even though EKF2 is not asked
to fuse EV attitude beyond yaw: PX4 rebuilds a quaternion from all three angles
and extracts yaw from that, so a mirrored pitch corrupts the one component that
is fused.

`enu_attitude_to_ned` now does the whole conversion — roll through, pitch
negated, yaw `pi/2 - yaw` — verified exact against `R_frd_ned = S · R_flu_enu · B`
over random attitudes.

**Note for the next flight:** the old test asserted the yaw passed through, and
passed, because it used `yaw = pi/2`. Under the old code that is heading 45° —
the one bearing where the raw ENU number happens to be right. The test now
sweeps headings.

### What to watch when this is flown

`align` on the VIO row at the moment `gps_denied` is pressed: it should read
tens of metres (the error that used to be inherited). Then drift from a frame
that starts at zero, rather than 60 → 318 → 1195 m from one that starts at 30.
If it still diverges, the remaining suspect is climb-phase drift itself, which
is untouched by either fix.

## Fifth run: the fixes fly. 8.6 still does not pass, but the failure changed shape

Flown 2026-08-13 with the frame alignment, the ENU->NED attitude fix, and
`EKF2_EVP_NOISE` raised 0.5 -> 3.0.

| | run 4 (2026-08-12) | run 5 (2026-08-13) |
|---|---|---|
| drift through the climb | 23–37 m | **0.29–0.60 m** |
| realign at fusion start | (did not exist) | closed 0.2 m, +0.1° |
| realign at the GNSS cut | (did not exist) | closed 0.4 m, −0.2° |
| drift at the cut | 23–37 m | **0.46 m** |
| after the cut | 60 → 318 → 1195 → 2544 m, 58 m/s | oscillation, ±5 → ±60 m |
| aircraft | ran 165 m off and diverged | stayed in OFFBOARD throughout |

**The handover is solved.** Both realignments had almost nothing left to close,
because the climb no longer accumulates tens of metres. For the first ~40 s
after GNSS was cut the aircraft held inside ~1 m with drift 0.42–1.04 m — the
first time vision-only station keeping has ever looked right.

**The climb drift collapsing from 23–37 m to 0.3 m was not the alignment.** The
alignment cannot affect `drift_m`, which is the raw estimator against GT and
never sees the transform. The likely cause is the attitude fix: EKF2 had been
fusing a vision yaw wrong by up to 180°, which made the aircraft fly badly
during the climb, which swept the camera and wrecked flow-odom's own input. The
estimator was being blamed for a fault upstream of it. Note run 5 also ran at
8.9 fps against run 4's 5.3, so some of the gain is a fresher sim, not the fix.

**What now fails: a slow, growing oscillation.** Roughly 40–60 s period, ±5 m
growing to ±60 m, with 550–600 inliers and no rejections throughout. It is a
control-loop instability, not a tracking failure — the monotonic runaway is
gone and nothing diverges at speed.

**Next suspect: `EKF2_EV_DELAY`.** It is still at its default while the real
path is camera → estimator → ZMQ → server → MAVLink. EKF2 applying vision at
the wrong timestamp is phase lag in a position loop, and phase lag is exactly
what produces a slow growing oscillation. Measure the true latency and set it
before touching `EKF2_EVP_NOISE` again.

## Two operational findings from the same session

**`alt` is negative on the pad and that is correct.** `sites.py` puts the
georeference origin at stage z = 0 and `ground_z` at −25.0, so a drone standing
on the ground is ~25 m below PX4's local origin. `relative_alt` reads 0.07 m,
correctly. Takeoff to 50 m displays as ~+25. (`sim/launch-sitl.sh:39` still
says −26.99; that comment is stale.)

**Arming can fail with `Preflight Fail: height estimate not stable`.** The
message is misleading — the height was steady. The trigger is
`pre_flt_fail_innov_height`, fed by the *baro* innovation against
`_hgt_innov_test_lim = 1.5f`, a compile-time constant in
`PreFlightChecker.hpp:199` that no parameter can relax. Cause: EKF2 initialised
while the drone was still settling onto the collision plane, because the
one-command launch now reboots PX4 the instant the server starts. Rebooting
again on a settled sim took `baro_vpos` from −1.76 to −0.91 and cleared it.
Ruled out by test, not assumption: the GPS origin altitude (0.0 against the
aircraft's −24.94) is innocent, and so is GPS-altitude fusion.

**Fix worth making:** gate the phase-0 reboot on the aircraft actually being at
rest, rather than firing it at startup.

## If this resurfaces

`alt_m` sinking steadily with `AUTO.LAND` latched and `offboard`/`disarm` both
refused, while the VIO row stays green, is the signature of the height reference
being wrong rather than of a vision failure. Check `EKF2_HGT_REF` took effect
*at boot*, not merely that QGC shows 0.
