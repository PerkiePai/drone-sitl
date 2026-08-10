# Session notes: recorder wrote zero images -- the web-recorder plan's Task 6 blocker

**Date:** 2026-08-10
**Symptom:** `RECORD` (via `sim/recorder_control.py`'s HTTP layer) reached
`state: "recording"` and CSVs filled in normally, but `images` stayed at 0 the
whole run. A dataset that writes full CSVs and zero images fails
`load_dataset()` -- this was the acceptance bar the whole feature exists to
clear (web-recorder plan, Task 3 Step 3.3).

## Root cause

`vio-recorder-pai.py`'s per-frame image capture (around line 403) called
`rep.orchestrator.step(rt_subframes=1, pause_timeline=False)` before reading the
camera annotators, added 2026-08-07 (commit `d16255a`) to fix a *different*,
real problem: image content lagging its own timestamp by about one render
period (a frame stamped `ts=1.00s` held pixels from `~0.95s`).

That fix was never verified in-sim before being committed. Live-tested
2026-08-10 in **Isaac Sim 6 / omni.replicator.core 1.13.25**: the call raises

```
Synchronous call to `step` can only be performed in a standalone workflow
```

The exception escapes the physics callback *before any camera is read*, so
every frame's image-capture block aborts silently and the run writes full
CSVs with zero images.

## Fix

Removed the `rep.orchestrator.step()` call. The timestamp-lag problem it was
trying to fix is not solved -- it is accepted: image content still lags its
own timestamp by ~1 render period, documented in a comment at the call site.
A fix would have to come from the timestamp side (tag each frame with the
annotator's own timestamp) rather than by driving the renderer from inside a
physics callback, which this build of Isaac Sim does not support.

Applied to both copies that are supposed to stay byte-identical:
`~/pai/drone-sitl/vio-recorder-pai.py` and `~/pai/drone-vio/vio-recorder-pai.py`.

## Verification (live, 2026-08-10)

`./sim/launch-sitl.sh`, then drove `sim/recorder_control.py`'s HTTP layer
directly (`curl localhost:8091/record/...`) with the drone parked (not flown):

- Recorded 23 s: 200 images, 0 dropped, growing steadily -- no stall.
- `curl -X POST .../record/start` while already recording: `409`, second call
  did not create a second directory (`test_second_start_is_rejected...`,
  confirmed live not just in the unit test).
- Stopped; dataset at `~/vio_dataset/20260810_145056/` has 254 non-empty
  images in `images/cam0/`, full `imu.csv`/`poses.csv`/`frames.csv`/`geo.csv`.
- `flow_odometry.load_dataset()` loads all 254 records.
- `flow_odometry.run(d, scale=0.5, max_frames=0, min_track=30)` end-to-end:
  LK inlier counts ranged ~170-590/frame (healthy tracking on real image
  content, not blank frames). `impl_depth` was `nan` throughout -- expected,
  since the drone never moved during this recording and that diagnostic needs
  GT translation to imply a depth.

**Not yet verified live:** recording during actual flight (design R2 -- that
RECORD never enters `SetpointLoop.submit()` and can't drop PX4 out of
OFFBOARD -- is covered by `test_record_commands_never_reach_the_setpoint_queue`
in `streaming/tests/test_offboard_loop.py`, but not flown together with a live
recording session this pass). Also not tested: killing Isaac mid-recording,
and the low-disk gate under a live near-floor condition (both covered by unit
tests in `sim/tests/test_recorder_control.py`).

## Measured write rate

At 286 GB free (not the 180 GB the original plan assumed -- this box has more
headroom): ~321 KB/s while parked and recording ≈ **19 MB/min ≈ 1.1 GB/hour**.
This is a stationary-drone measurement (no flight motion); real flight
datasets will vary somewhat with scene detail but should be the same order of
magnitude. At this rate, 286 GB free is >100 hours of recording before
pruning matters.

## If this resurfaces

`grep -n "rep.orchestrator.step" vio-recorder-pai.py` should find nothing. If
it's back, it will reproduce the zero-image failure again on this Isaac Sim
build -- don't re-add it without solving the timestamp-lag problem from the
timestamp side first.
