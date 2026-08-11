# Session notes: VIO GPS-denied live bring-up (plan Task 8)

**Date:** 2026-08-11
**Plan:** `docs/superpowers/plans/2026-08-07-vio-gps-denied.md`, Task 8
**Result:** 8.1–8.5 pass. GPS-denied flight ACHIEVED and held ~40 s, then
diverged. 8.6 does not pass. Read top to bottom: three runs, and each fix
exposed the next fault. The root cause of all of it was INT32 param encoding.

Run configuration:

```bash
DRONE_SETUP_DOWN_VIB_DAMP=False VIO=1 ./sim/launch-sitl.sh
python pipeline-streaming.py --scale 0.5 --print-every 15
python joystick-server.py --takeoff-alt 50 --speed-up 3.0 [--vision --site bangkok-survey-040]
```

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

## If this resurfaces

`alt_m` sinking steadily with `AUTO.LAND` latched and `offboard`/`disarm` both
refused, while the VIO row stays green, is the signature of the height reference
being wrong rather than of a vision failure. Check `EKF2_HGT_REF` took effect
*at boot*, not merely that QGC shows 0.
