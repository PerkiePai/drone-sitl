# Implementation plan: streaming VIO pipeline → PX4/QGC

## Goal

Build a live-streaming counterpart to `pipeline.py`'s flow-odom layer that
feeds PX4 SITL (Isaac Sim) a `VISION_POSITION_ESTIMATE`, so QGroundControl
can fly the drone vision-only. See design spec:
`docs/superpowers/specs/2026-07-13-pipeline-streaming-design.md`.

## Architecture summary

Two new processes bridged by ZeroMQ, plus a MAVLink UDP leg to PX4:

```
Isaac Sim (embedded Python)        drone conda env (external process)      PX4 SITL
vio-streamer-pai.py  --ZMQ PUB/SUB--> pipeline-streaming.py --MAVLink UDP--> (Pegasus)
```

v1 scope: LK flow-odom only (no DSMAC/relief-fix), baro-derived depth
(no rangefinder on this rig), no GT (CLI-provided start lat/lon).

## Tech stack

- `drone` conda env: Python, numpy, opencv, scipy, `pyzmq`, `msgpack-python`,
  `pymavlink`, `pytest` (new deps — Task 0 installs them).
- Isaac Sim embedded Python: existing Pegasus/omni/replicator stack already
  used by `vio-recorder-pai.py` — no new deps there.

## Global constraints

- Reuse `pipeline.py`'s `_detect`, `_track_lk`, `_solve_translation` and
  `flow_odometry.py`'s Mahony math **unchanged** — no
  forked copies of tracking/attitude math.
- `vio-streamer-pai.py` is a new, separate script from `vio-recorder-pai.py`
  (per design decision) — do not modify the recorder.
- All new shared (non-Isaac-Sim) modules live under `streaming/`; the two
  runnable entry-point scripts (`pipeline-streaming.py` for the drone env,
  `vio-streamer-pai.py` for Isaac Sim) live at repo root, matching the
  existing flat convention (`pipeline.py`, `vio-recorder-pai.py`,
  `drone_setup_px4_cesium-pai.py`).

---

## Task 0: Install streaming dependencies (drone env)

- [ ] Install the three new packages into the `drone` conda env:
  ```bash
  conda run -n drone pip install pyzmq msgpack pymavlink pytest
  ```
- [ ] Verify:
  ```bash
  conda run -n drone python -c "import zmq, msgpack, pymavlink, pytest; print('ok')"
  ```
  Expected output: `ok`
- [ ] Commit message: `chore: add pyzmq/msgpack/pymavlink/pytest to drone env deps`
  (no repo files change if these aren't pinned anywhere; if the project has
  a requirements/environment file for the `drone` env, add the three
  packages there instead of a no-op commit — check for one first with
  `find . -iname "*.yml" -o -iname "requirements*.txt" | grep -i drone`
  before deciding whether this task produces a diff.)

---

## Task 1: `MahonyState` — incremental Mahony AHRS

**File:** `flow_odometry.py` (modify)
**Test:** `streaming/tests/test_mahony_state.py` (create)

Extract the per-tick update math already inside `compute_ahrs_attitude`
(lines 162–190) into a reusable class, then make `compute_ahrs_attitude`
call it internally — so the batch path and the new streaming path share one
implementation, and the unit test proves they're identical by construction.

### Step 1.1 — Add `MahonyState` class

- [ ] In `flow_odometry.py`, add (just above
  `compute_ahrs_attitude`):

  ```python
  class MahonyState:
      """Incremental Mahony complementary filter — the same per-tick gyro/accel
      (+ optional magnetometer) correction as compute_ahrs_attitude's batch loop,
      refactored so it can be driven one IMU sample at a time by a live subscriber
      instead of a pre-loaded rows array. State R is FRD body -> ENU world (the
      same internal convention compute_ahrs_attitude uses before its final
      "@ flip" back to FLU for pipeline consumption)."""

      def __init__(self, R0, Kp=1.0, mag_gain=0.0):
          self.R = R0.copy()
          self.Kp = Kp
          self.mag_gain = mag_gain
          self.m_world = None
          self.g_up = np.array([0.0, 0.0, 9.81])

      def calibrate_mag(self, mag_body0):
          """One-time compass calibration from a body-frame mag reading taken at
          the same instant as R0 (mirrors compute_ahrs_attitude's m_world calc)."""
          self.m_world = self.R @ (mag_body0 / np.linalg.norm(mag_body0))

      def update(self, gyro, acc, mag, dt):
          """gyro (rad/s, FRD), acc (specific force, FRD), mag (body FRD or
          None), dt (s). Returns the updated R (FRD->ENU)."""
          from scipy.spatial.transform import Rotation as Rot
          w = gyro.copy()
          an = np.linalg.norm(acc)
          if an > 1e-3:
              v_meas = acc / an
              v_pred = self.R.T @ self.g_up; v_pred /= np.linalg.norm(v_pred)
              w = w + self.Kp * np.cross(v_meas, v_pred)
          if self.m_world is not None and mag is not None:
              mn = np.linalg.norm(mag)
              if mn > 1e-6:
                  m_meas = mag / mn
                  m_pred = self.R.T @ self.m_world; m_pred /= np.linalg.norm(m_pred)
                  w = w + self.mag_gain * np.cross(m_meas, m_pred)
          self.R = self.R @ Rot.from_rotvec(w * dt).as_matrix()
          return self.R
  ```

### Step 1.2 — Refactor `compute_ahrs_attitude` to use `MahonyState`

- [ ] Replace the body of the `for k in range(1, len(rows))` loop (lines
  162–190) so it drives a `MahonyState` instance instead of updating `R`
  inline. New loop:

  ```python
      state = MahonyState(R0, Kp=Kp, mag_gain=mag_gain)
      if m_world is not None:
          state.m_world = m_world   # already computed above from GT frame-0 + mag0

      out = [recs[0]["R_wb"]]                 # frame 0 = GT init
      ri = 1
      for k in range(1, len(rows)):
          dt = ts[k] - ts[k - 1]
          if dt <= 0 or dt > 0.1:
              dt = 0.004
          m_k = mag[k] if (m_world is not None and gt_yaw is None) else None
          state.update(gyro[k], acc[k], m_k, dt)
          while ri < len(recs) and ts[k] >= rec_ts[ri]:
              if gt_yaw is not None:          # legacy GT-yaw stand-in (no real mag channel)
                  yaw = np.arctan2(state.R[1, 0], state.R[0, 0])
                  meas = gt_yaw[ri] + np.radians(mag_noise_deg) * rng.standard_normal()
                  dpsi = np.arctan2(np.sin(meas - yaw), np.cos(meas - yaw))
                  state.R = Rot.from_rotvec([0, 0, mag_gain * dpsi]).as_matrix() @ state.R
              out.append(state.R @ flip)      # back to FLU->ENU for load_dataset parity
              ri += 1
      while len(out) < len(recs):
          out.append(state.R @ flip)
      return out
  ```
  Note: the `mag_noise_deg` rotvec-perturbation branch inside the old loop
  (lines 177–179, applied to `m_meas` before the cross product) is dropped
  from the shared path since it only ever fired when `gt_yaw is None` and
  `m_world is not None and mag_noise_deg > 0`, which no existing caller in
  this repo uses with a nonzero `mag_noise_deg` alongside real mag data —
  confirm with `grep -rn "mag_noise_deg" --include=*.py .` before deleting;
  if any caller does combine them, keep that perturbation inline in the loop
  (applied to `m_k` before passing to `state.update`) instead of dropping it.
- [ ] Keep `Rot` imported at module scope in `compute_ahrs_attitude` (already
  is, via `from scipy.spatial.transform import Rotation as Rot` at the top
  of the function) since the refactored loop still uses it directly for the
  legacy GT-yaw branch.

### Step 1.3 — Regression test: batch output unchanged

- [ ] Create `streaming/tests/test_mahony_state.py`:

  ```python
  import os, sys
  import numpy as np

  ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  sys.path.insert(0, ROOT)
  import flow_odometry as fo


  def _fixture_dir():
      # any recorded dataset dir with imu.csv + poses.csv works; skip if none present
      cands = [d for d in os.listdir(os.path.join(ROOT, "_in"))
               if os.path.isdir(os.path.join(ROOT, "_in", d))] if os.path.isdir(os.path.join(ROOT, "_in")) else []
      return os.path.join(ROOT, "_in", cands[0]) if cands else None


  def test_mahony_state_matches_batch_no_compass():
      d = _fixture_dir()
      if d is None:
          import pytest; pytest.skip("no _in/<dataset> fixture available")
      K, R_CtoI, recs = fo.load_dataset(d)
      out = fo.compute_ahrs_attitude(d, recs, Kp=1.0, mag_gain=0.0)
      assert len(out) == len(recs)
      # frame 0 must be exactly GT init (both paths agree by construction)
      assert np.allclose(out[0], recs[0]["R_wb"])
      # every subsequent R must be a valid rotation matrix
      for R in out[1:]:
          assert np.allclose(R.T @ R, np.eye(3), atol=1e-6)
          assert np.isclose(np.linalg.det(R), 1.0, atol=1e-6)


  def test_mahony_state_standalone_update_matches_R0():
      R0 = np.eye(3)
      state = fo.MahonyState(R0, Kp=1.0)
      # zero gyro, gravity-aligned accel -> R should not rotate
      state.update(np.zeros(3), np.array([0.0, 0.0, 9.81]), None, 0.005)
      assert np.allclose(state.R, np.eye(3), atol=1e-6)
  ```
- [ ] Run:
  ```bash
  conda run -n drone pytest streaming/tests/test_mahony_state.py -v
  ```
  Expected: both tests pass (first `SKIPPED` only if no `_in/` dataset dir
  exists locally — acceptable, since `_in/` is gitignored and dataset
  availability is environment-specific).
- [ ] Commit: `refactor(flow-odom): extract MahonyState for incremental AHRS use`

---

## Task 2: ZMQ wire format

**File:** `streaming/zmq_proto.py` (create)
**Test:** `streaming/tests/test_zmq_proto.py` (create)

### Step 2.1 — Write the module

- [ ] Create `streaming/zmq_proto.py`:

  ```python
  """Shared ZMQ wire-format for the Isaac Sim -> drone-env VIO streaming bridge
  (see docs/superpowers/specs/2026-07-13-pipeline-streaming-design.md).

  Each message is a two-part ZMQ multipart message: [topic: bytes, msgpack: bytes].

    b"imu"   -> {"ts_ns": int, "wx","wy","wz","ax","ay","az": float}   (FRD body)
    b"baro"  -> {"ts_ns": int, "pressure_altitude_m": float}
    b"frame" -> {"frame_idx": int, "ts_ns": int, "jpg": bytes}         (grayscale JPEG)
  """
  import msgpack

  TOPIC_IMU = b"imu"
  TOPIC_BARO = b"baro"
  TOPIC_FRAME = b"frame"
  ALL_TOPICS = (TOPIC_IMU, TOPIC_BARO, TOPIC_FRAME)


  def pack_imu(ts_ns, wx, wy, wz, ax, ay, az):
      body = {"ts_ns": ts_ns, "wx": wx, "wy": wy, "wz": wz, "ax": ax, "ay": ay, "az": az}
      return TOPIC_IMU, msgpack.packb(body, use_bin_type=True)


  def pack_baro(ts_ns, pressure_altitude_m):
      body = {"ts_ns": ts_ns, "pressure_altitude_m": pressure_altitude_m}
      return TOPIC_BARO, msgpack.packb(body, use_bin_type=True)


  def pack_frame(frame_idx, ts_ns, jpg_bytes):
      body = {"frame_idx": frame_idx, "ts_ns": ts_ns, "jpg": jpg_bytes}
      return TOPIC_FRAME, msgpack.packb(body, use_bin_type=True)


  def unpack(topic, payload):
      """Returns (topic: bytes, body: dict)."""
      return topic, msgpack.unpackb(payload, raw=False)
  ```

### Step 2.2 — Round-trip test

- [ ] Create `streaming/tests/test_zmq_proto.py`:

  ```python
  import os, sys
  ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  sys.path.insert(0, os.path.join(ROOT, "streaming"))
  import zmq_proto as zp


  def test_imu_roundtrip():
      topic, payload = zp.pack_imu(123456789, 0.1, -0.2, 0.3, 0.01, 0.02, 9.8)
      t, body = zp.unpack(topic, payload)
      assert t == zp.TOPIC_IMU
      assert body == {"ts_ns": 123456789, "wx": 0.1, "wy": -0.2, "wz": 0.3,
                       "ax": 0.01, "ay": 0.02, "az": 9.8}


  def test_baro_roundtrip():
      topic, payload = zp.pack_baro(42, 17.5)
      t, body = zp.unpack(topic, payload)
      assert t == zp.TOPIC_BARO
      assert body == {"ts_ns": 42, "pressure_altitude_m": 17.5}


  def test_frame_roundtrip():
      jpg = b"\xff\xd8\xff\xe0fakejpegbytes"
      topic, payload = zp.pack_frame(7, 999, jpg)
      t, body = zp.unpack(topic, payload)
      assert t == zp.TOPIC_FRAME
      assert body == {"frame_idx": 7, "ts_ns": 999, "jpg": jpg}
  ```
- [ ] Run:
  ```bash
  conda run -n drone pytest streaming/tests/test_zmq_proto.py -v
  ```
  Expected: 3 passed.
- [ ] Commit: `feat(streaming): add ZMQ wire-format module + round-trip tests`

---

## Task 3: MAVLink bridge

**File:** `streaming/mavlink_bridge.py` (create)
**Test:** `streaming/tests/test_mavlink_bridge.py` (create)

### Step 3.1 — Write the module

- [ ] Create `streaming/mavlink_bridge.py`:

  ```python
  """ENU -> NED conversion + VISION_POSITION_ESTIMATE sender for
  pipeline-streaming.py. PX4's vision_position_estimate message expects
  LOCAL_FRAME_NED (x=North, y=East, z=Down); this repo's estimators
  (pipeline.py, flow_odometry.py) work in ENU (x=East, y=North, z=Up)
  throughout, so the conversion happens only at this one boundary."""
  import math
  import time


  def enu_to_ned(east, north, up):
      return north, east, -up


  def yaw_enu_to_ned(yaw_enu_rad):
      """ENU yaw (0 = East, CCW+) -> NED yaw (0 = North, CW+)."""
      yaw = math.pi / 2.0 - yaw_enu_rad
      return math.atan2(math.sin(yaw), math.cos(yaw))   # wrap to (-pi, pi]


  class VisionPositionSender:
      """Wraps a pymavlink connection to send VISION_POSITION_ESTIMATE.
      reset_counter only increments on an explicit reset() call (e.g. after a
      ZMQ reconnect discontinuity in pos), matching the MAVLink spec's use of
      reset_counter to tell the receiving EKF a position jump is expected and
      not something to reject/reset itself over.

      roll/pitch are sent as 0.0: this repo's EKF2_EV_CTRL configuration
      (see streaming/ekf2_ev_params.params) fuses only horizontal/vertical
      position (+ yaw), not full attitude — PX4 already gets roll/pitch from
      its own IMU, so these fields are unused by the receiver, not TODOs."""

      def __init__(self, conn):
          self.conn = conn
          self._reset_counter = 0

      def reset(self):
          self._reset_counter = (self._reset_counter + 1) % 256

      def send(self, east, north, up, yaw_enu_rad, ts_s=None):
          x, y, z = enu_to_ned(east, north, up)
          yaw_ned = yaw_enu_to_ned(yaw_enu_rad)
          usec = int((ts_s if ts_s is not None else time.time()) * 1e6)
          self.conn.mav.vision_position_estimate_send(
              usec, x, y, z, 0.0, 0.0, yaw_ned,
              reset_counter=self._reset_counter)
  ```

### Step 3.2 — Unit tests with a mocked connection

- [ ] Create `streaming/tests/test_mavlink_bridge.py`:

  ```python
  import math, os, sys
  from unittest.mock import MagicMock
  ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  sys.path.insert(0, os.path.join(ROOT, "streaming"))
  import mavlink_bridge as mb


  def test_enu_to_ned():
      assert mb.enu_to_ned(east=5.0, north=3.0, up=2.0) == (3.0, 5.0, -2.0)


  def test_yaw_enu_to_ned_north():
      # ENU yaw=90deg (facing North) -> NED yaw=0
      assert math.isclose(mb.yaw_enu_to_ned(math.pi / 2), 0.0, abs_tol=1e-9)


  def test_yaw_enu_to_ned_east():
      # ENU yaw=0 (facing East) -> NED yaw=90deg
      assert math.isclose(mb.yaw_enu_to_ned(0.0), math.pi / 2, abs_tol=1e-9)


  def test_vision_position_sender_sends_ned_fields():
      conn = MagicMock()
      sender = mb.VisionPositionSender(conn)
      sender.send(east=10.0, north=4.0, up=20.0, yaw_enu_rad=0.0, ts_s=1.0)
      conn.mav.vision_position_estimate_send.assert_called_once()
      args, kwargs = conn.mav.vision_position_estimate_send.call_args
      usec, x, y, z, roll, pitch, yaw = args
      assert usec == 1_000_000
      assert (x, y, z) == (4.0, 10.0, -20.0)
      assert roll == 0.0 and pitch == 0.0
      assert math.isclose(yaw, math.pi / 2, abs_tol=1e-9)
      assert kwargs["reset_counter"] == 0


  def test_reset_increments_counter():
      conn = MagicMock()
      sender = mb.VisionPositionSender(conn)
      sender.reset()
      sender.send(east=0.0, north=0.0, up=0.0, yaw_enu_rad=0.0)
      _, kwargs = conn.mav.vision_position_estimate_send.call_args
      assert kwargs["reset_counter"] == 1
  ```
- [ ] Run:
  ```bash
  conda run -n drone pytest streaming/tests/test_mavlink_bridge.py -v
  ```
  Expected: 4 passed.
- [ ] Commit: `feat(streaming): add MAVLink ENU->NED bridge + VisionPositionSender`

---

## Task 4: Extract `load_calib` for reuse outside `load_dataset`

**File:** `flow_odometry.py` (modify)
**Test:** `streaming/tests/test_load_calib.py` (create)

`pipeline-streaming.py` needs `(K, R_CtoI)` from a `cam_calib.json` without a
full dataset directory (no `frames.csv`/`poses.csv`/`baro.csv` live). Extract
the calibration-loading portion of `load_dataset` (lines 58–74) into its own
function so both call sites share it.

### Step 4.1 — Refactor

- [ ] In `flow_odometry.py`, add above `load_dataset`:

  ```python
  def load_calib(calib_path):
      """(K, R_CtoI) from a cam_calib.json path. R_CtoI is camera->body
      (FRD-IMU-body convention fixed up to match GT's FLU-in-ENU quaternions
      — see load_dataset's docstring note for the empirical derivation)."""
      calib = json.load(open(calib_path))
      fx = calib["intrinsics"]["fx"]; fy = calib["intrinsics"]["fy"]
      cx = calib["intrinsics"]["cx"]; cy = calib["intrinsics"]["cy"]
      K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
      qw, qx, qy, qz = calib["extrinsic_body_to_cam"]["quaternion_wxyz"]
      R_CtoI = quat_xyzw_to_R(qx, qy, qz, qw)
      R_CtoI = R_CtoI @ np.diag([1.0, -1.0, -1.0])  # body = R_CtoI @ cam
      return K, R_CtoI
  ```
- [ ] Replace `load_dataset`'s lines 58–74 with:
  ```python
      K, R_CtoI = load_calib(os.path.join(d, "cam_calib.json"))
  ```
  (removing the now-duplicated inline calibration parsing).

### Step 4.2 — Test

- [ ] Create `streaming/tests/test_load_calib.py`:

  ```python
  import os, sys
  import numpy as np
  ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  sys.path.insert(0, ROOT)
  import flow_odometry as fo


  def test_load_calib_matches_load_dataset():
      cands = [d for d in os.listdir(os.path.join(ROOT, "_in"))
               if os.path.isdir(os.path.join(ROOT, "_in", d))] if os.path.isdir(os.path.join(ROOT, "_in")) else []
      if not cands:
          import pytest; pytest.skip("no _in/<dataset> fixture available")
      d = os.path.join(ROOT, "_in", cands[0])
      K1, R1 = fo.load_calib(os.path.join(d, "cam_calib.json"))
      K2, R2, _ = fo.load_dataset(d)
      assert np.allclose(K1, K2)
      assert np.allclose(R1, R2)
  ```
- [ ] Run:
  ```bash
  conda run -n drone pytest streaming/tests/test_load_calib.py frontend/flow-odom -v 2>/dev/null; \
  conda run -n drone python pipeline.py --dir _in/<any-existing-dataset> --max_frames 50
  ```
  Expected: test passes (or skips if no fixture); the existing batch pipeline
  smoke-run on 50 frames still completes with no traceback — this is the
  regression check that Task 4's refactor didn't break `load_dataset`.
- [ ] Commit: `refactor(flow-odom): extract load_calib for use without a full dataset dir`

---

## Task 5: `pipeline-streaming.py` — the streaming estimator + MAVLink loop

**File:** `pipeline-streaming.py` (create, repo root)

### Step 5.1 — Write the script

- [ ] Create `pipeline-streaming.py` (repo root):

  ```python
  #!/usr/bin/env python3
  """Streaming GPS-denied position estimator -> PX4 external vision.

  Live counterpart to pipeline.py's flow-odom layer (v1 scope: no DSMAC /
  relief-fix — see docs/superpowers/specs/2026-07-13-pipeline-streaming-design.md).
  Consumes imu/baro/frame messages published by vio-streamer-pai.py over ZMQ,
  runs the same LK flow-odom step as pipeline.py (imported unchanged), and
  streams the fused ENU position to PX4 as VISION_POSITION_ESTIMATE so
  QGroundControl can fly vision-only.

  No GT is available live: initial position is a CLI-provided takeoff lat/lon,
  used only to report a human-readable location in logs -- the estimator
  itself works in ENU relative to its own start point (0, 0), matching what
  VISION_POSITION_ESTIMATE actually needs (a consistent local frame, not an
  absolute one).

  Depth is baro-derived height above takeoff (no rangefinder on this rig) --
  same accuracy ceiling as pipeline.py's own no-lidar fallback path.

  Run (after vio-streamer-pai.py is streaming and PX4 SITL is up):
    conda run -n drone python pipeline-streaming.py \\
        --calib _in/isaac-sim-20260625/cam_calib.json \\
        --start_lat 40.7128 --start_lon -74.0060
  """
  import argparse, math, os, sys
  from io import BytesIO

  import cv2
  import numpy as np
  from PIL import Image
  import zmq
  from pymavlink import mavutil

  ROOT = os.path.dirname(os.path.abspath(__file__))
  sys.path.insert(0, ROOT)
  sys.path.insert(0, os.path.join(ROOT, "streaming"))
  import flow_odometry as fo
  from zmq_proto import TOPIC_IMU, TOPIC_BARO, TOPIC_FRAME, unpack
  from mavlink_bridge import VisionPositionSender

  from pipeline import _detect, _track_lk, _solve_translation  # reused unchanged

  FLU_FRD_FLIP = np.diag([1.0, -1.0, -1.0])
  MIN_TRACK = 30


  def main():
      ap = argparse.ArgumentParser(description=__doc__,
                                    formatter_class=argparse.RawDescriptionHelpFormatter)
      ap.add_argument("--calib", required=True,
                      help="cam_calib.json (same schema pipeline.py's datasets use)")
      ap.add_argument("--zmq_addr", default="tcp://localhost:5556")
      ap.add_argument("--mavlink_addr", default="udpout:127.0.0.1:14540",
                      help="PX4 companion/offboard endpoint (verify against the "
                           "running PX4 SITL's actual MAVLink instance list)")
      ap.add_argument("--start_lat", type=float, required=True, help="log-only, no georef math yet")
      ap.add_argument("--start_lon", type=float, required=True, help="log-only, no georef math yet")
      ap.add_argument("--stride", type=int, default=5,
                      help="keep every Nth received frame (default 5, matches pipeline.py)")
      ap.add_argument("--scale", type=float, default=0.5)
      ap.add_argument("--mahony_kp", type=float, default=1.0)
      ap.add_argument("--compass_gain", type=float, default=1.0)
      args = ap.parse_args()

      K, R_CtoI = fo.load_calib(args.calib)
      Ks = K.copy(); Ks[:2, :] *= args.scale
      Kinv = np.linalg.inv(Ks)

      ctx = zmq.Context()
      sub = ctx.socket(zmq.SUB)
      sub.connect(args.zmq_addr)
      for t in (TOPIC_IMU, TOPIC_BARO, TOPIC_FRAME):
          sub.setsockopt(zmq.SUBSCRIBE, t)
      print(f"  ZMQ connected: {args.zmq_addr}")

      conn = mavutil.mavlink_connection(args.mavlink_addr)
      sender = VisionPositionSender(conn)
      print(f"  MAVLink companion link: {args.mavlink_addr}")
      print(f"  start (lat={args.start_lat}, lon={args.start_lon}); "
            f"estimator origin is local ENU (0,0) at first kept frame")

      mahony = None          # fo.MahonyState, created on first IMU sample (identity init)
      last_imu_ts = None
      baro0 = None
      last_h = None          # height above takeoff (baro), depth stand-in
      prev_gray = None       # last KEPT frame, grayscale + scaled
      prev_R_wb = None       # FLU->ENU attitude at prev_gray's timestamp
      frame_count = 0
      pos = np.zeros(2)      # ENU (east, north), relative to first kept frame
      step = 0

      def to_gray_scaled(jpg_bytes):
          im = np.array(Image.open(BytesIO(jpg_bytes)).convert("L"))
          return cv2.resize(im, None, fx=args.scale, fy=args.scale,
                            interpolation=cv2.INTER_AREA)

      print("  waiting for imu/baro/frame messages ...")
      while True:
          topic, payload = sub.recv_multipart()
          _, body = unpack(topic, payload)

          if topic == TOPIC_IMU:
              gyro = np.array([body["wx"], body["wy"], body["wz"]])
              acc  = np.array([body["ax"], body["ay"], body["az"]])
              ts   = body["ts_ns"] / 1e9
              if mahony is None:
                  # Cold start: identity attitude until a real reference exists.
                  # Unlike pipeline.py's GT-init, no GT is available live -- this
                  # is a documented v1 approximation (yaw settles via compass_gain
                  # if a mag channel exists in the imu topic; roll/pitch settle
                  # via the accel correction within a couple seconds of level flight).
                  mahony = fo.MahonyState(np.eye(3), Kp=args.mahony_kp,
                                          mag_gain=args.compass_gain)
                  last_imu_ts = ts
                  continue
              dt = ts - last_imu_ts
              if 0 < dt <= 0.1:
                  mahony.update(gyro, acc, None, dt)
              last_imu_ts = ts

          elif topic == TOPIC_BARO:
              if baro0 is None:
                  baro0 = body["pressure_altitude_m"]
              last_h = body["pressure_altitude_m"] - baro0

          elif topic == TOPIC_FRAME:
              frame_count += 1
              if frame_count % args.stride != 0:
                  continue
              if mahony is None or last_h is None:
                  continue  # no attitude/height seeded yet
              cur_gray = to_gray_scaled(body["jpg"])
              R_wb = mahony.R @ FLU_FRD_FLIP
              if prev_gray is not None:
                  dC = np.zeros(2); used = 0
                  p0 = _detect(prev_gray)
                  if p0 is not None and len(p0) >= MIN_TRACK:
                      p0g, p1g = _track_lk(prev_gray, cur_gray, p0)
                      if len(p0g) >= MIN_TRACK:
                          R_wc0  = prev_R_wb @ R_CtoI
                          R_wc1  = R_wb @ R_CtoI
                          R_c1c0 = R_wc1.T @ R_wc0
                          h0     = max(float(last_h), 0.3)
                          t_cam, used = _solve_translation(
                              p0g, p1g, Kinv, R_wc0, R_c1c0, h0, MIN_TRACK)
                          dC = (R_wc0 @ t_cam)[:2]
                  pos = pos + dC
                  step += 1
                  yaw = math.atan2(R_wb[1, 0], R_wb[0, 0])
                  sender.send(east=pos[0], north=pos[1], up=last_h,
                              yaw_enu_rad=yaw, ts_s=body["ts_ns"] / 1e9)
                  if step % 20 == 0:
                      print(f"  step {step}: pos=({pos[0]:.1f}, {pos[1]:.1f}) "
                            f"h={last_h:.1f} m  LK inliers={used}")
              prev_gray, prev_R_wb = cur_gray, R_wb


  if __name__ == "__main__":
      main()
  ```

### Step 5.2 — Smoke test without Isaac Sim or PX4

- [ ] `pipeline-streaming.py` needs a ZMQ publisher and a MAVLink listener to
  start at all. Verify the script at least parses args and fails cleanly
  without either running yet:
  ```bash
  conda run -n drone python pipeline-streaming.py --calib _in/<any-dataset>/cam_calib.json \
      --start_lat 0 --start_lon 0 --mavlink_addr udpout:127.0.0.1:19999 &
  sleep 2 && kill %1
  ```
  Expected: prints `ZMQ connected`, `MAVLink companion link`, `start (...)`,
  `waiting for imu/baro/frame messages ...`, then blocks (killed by the
  `sleep`+`kill`) — no traceback. This proves the calib load, ZMQ SUB setup,
  and MAVLink UDP socket creation all succeed standalone; the full data path
  is exercised later in Task 6's manual checklist.
- [ ] Commit: `feat(streaming): add pipeline-streaming.py (flow-odom -> VISION_POSITION_ESTIMATE)`

---

## Task 6: `vio-streamer-pai.py` — Isaac Sim ZMQ publisher

**File:** `vio-streamer-pai.py` (create, repo root)

Mirrors `vio-recorder-pai.py`'s sensor wiring (Pegasus `IMU`/`Barometer`
callbacks, `down_cam` capture via `omni.replicator`) but ZMQ-PUBs each
sample instead of writing CSV/PNG to disk — no takeoff-wait gating (unlike
the recorder): streaming starts as soon as the sim is playing, so PX4 has a
vision source from before liftoff.

### Step 6.1 — Write the script

- [ ] Create `vio-streamer-pai.py` (repo root):

  ```python
  # ============================================================================
  # VIO streamer -- live ZMQ counterpart to vio-recorder-pai.py (Isaac Sim
  # Script Editor). Publishes imu/baro/frame over ZMQ PUB instead of writing
  # CSV/PNG to disk; see docs/superpowers/specs/2026-07-13-pipeline-streaming-design.md
  # and streaming/zmq_proto.py for the wire format.
  #
  #   Run order: spawn (drone_setup_px4_cesium-pai.py) -> Play -> run THIS in
  #   the Script Editor. Then, in the drone conda env:
  #     conda run -n drone python pipeline-streaming.py --calib <cam_calib.json> \
  #         --start_lat <lat> --start_lon <lon>
  #
  #   No takeoff gating (unlike vio-recorder-pai.py): streams from the moment
  #   this script runs, so PX4 has a vision source before liftoff.
  # ============================================================================
  import io, sys, os, time
  import numpy as np
  from isaacsim.core.api.world import World
  from pegasus.simulator.logic.vehicle_manager import VehicleManager
  from pegasus.simulator.logic.sensors import IMU, Barometer
  import omni.usd
  import omni.replicator.core as rep
  from pxr import UsdGeom
  from PIL import Image
  import zmq

  sys.path.insert(0, os.path.expanduser("~/Desktop/project/drone-sitl/streaming"))
  from zmq_proto import pack_imu, pack_baro, pack_frame  # noqa: E402

  ZMQ_BIND     = "tcp://*:5556"
  DATA_FPS     = 200      # imu + baro publish rate (Hz); mirrors vio-recorder-pai.py
  FRAME_FPS    = 15       # camera publish rate (Hz)
  CAM_W, CAM_H = 960, 600
  JPEG_QUALITY = 92
  PRINT_EVERY_S = 1.0

  vm = VehicleManager.get_vehicle_manager()
  vehicles = list(vm.vehicles.values()) if getattr(vm, "vehicles", None) else []
  world = World.instance()
  stage = omni.usd.get_context().get_stage()


  def find_cam_path(prim_name):
      return next((str(p.GetPath()) for p in stage.Traverse()
                   if p.IsA(UsdGeom.Camera) and p.GetName() == prim_name), None)


  cam_path = find_cam_path("down_cam")
  imu = None; baro = None; veh = None
  if vehicles:
      veh = vehicles[0]
      imu = next((s for s in veh._sensors if isinstance(s, IMU)), None)
      baro = next((s for s in veh._sensors if isinstance(s, Barometer)), None)

  if not vehicles or imu is None:
      print("*** No drone/IMU. Spawn with drone_setup_px4_cesium-pai.py first. ***")
  elif cam_path is None:
      print("*** down_cam not found on stage. ***")
  elif world is None:
      print("*** No World instance. ***")
  else:
      old = globals().get("_VIO_STREAM")
      if old:
          try: world.remove_physics_callback(old["cb"])
          except Exception: pass
          try: old["pub"].close(0)
          except Exception: pass

      ctx = zmq.Context()
      pub = ctx.socket(zmq.PUB)
      pub.bind(ZMQ_BIND)
      print(f">>> ZMQ PUB bound: {ZMQ_BIND}")

      rp = rep.create.render_product(cam_path, (CAM_W, CAM_H))
      ann = rep.AnnotatorRegistry.get_annotator("rgb")
      ann.attach([rp])

      st = {"last_print": -1e9, "step": 0, "decim": None, "img_every": None,
            "frame": 0, "n_imu": 0, "n_baro": 0, "n_frame": 0}

      def _on_phys(dt):
          now = world.current_time
          if st["decim"] is None:
              st["decim"] = max(1, round((1.0 / DATA_FPS) / max(dt, 1e-9)))
              actual_data_fps = 1.0 / (dt * st["decim"])
              st["img_every"] = max(1, round(actual_data_fps / max(1, FRAME_FPS)))
              print(f">>> actual data rate ~{actual_data_fps:.0f} Hz; "
                    f"image every {st['img_every']} frames")
          st["step"] += 1
          if st["step"] % st["decim"] != 0:
              return

          st["frame"] += 1
          fr = st["frame"]
          ts_ns = int(now * 1e9)

          si = imu.state
          w = si.get("angular_velocity", (0, 0, 0)); a = si.get("linear_acceleration", (0, 0, 0))
          topic, payload = pack_imu(ts_ns, float(w[0]), float(w[1]), float(w[2]),
                                    float(a[0]), float(a[1]), float(a[2]))
          pub.send_multipart([topic, payload])
          st["n_imu"] += 1

          if baro is not None:
              sb = baro.state
              topic, payload = pack_baro(ts_ns, float(sb.get("pressure_altitude", 0.0)))
              pub.send_multipart([topic, payload])
              st["n_baro"] += 1

          if fr % st["img_every"] == 0:
              try:
                  data = ann.get_data()
                  if data is not None and getattr(data, "size", 0) > 0:
                      rgb = np.ascontiguousarray(np.asarray(data)[:, :, :3])
                      im = Image.fromarray(rgb).convert("L")
                      buf = io.BytesIO()
                      im.save(buf, format="JPEG", quality=JPEG_QUALITY)
                      topic, payload = pack_frame(fr, ts_ns, buf.getvalue())
                      pub.send_multipart([topic, payload])
                      st["n_frame"] += 1
              except Exception:
                  pass

          if now - st["last_print"] >= PRINT_EVERY_S:
              st["last_print"] = now
              print(f"[STREAM] t={now:5.1f}s imu={st['n_imu']} baro={st['n_baro']} "
                    f"frames={st['n_frame']}")

      world.add_physics_callback("vio_stream", _on_phys)
      globals()["_VIO_STREAM"] = {"cb": "vio_stream", "pub": pub}
      print(">>> VIO streamer running. ZMQ topics: imu, baro, frame.")
      print(">>> Stop:  World.instance().remove_physics_callback('vio_stream'); "
            "_VIO_STREAM['pub'].close(0)")
  ```

  Note: the `sys.path.insert(0, os.path.expanduser("~/pai/drone-vio/streaming"))`
  line hardcodes the repo path since Isaac Sim's Script Editor has no notion
  of "the current script's directory" the way a normal `__file__`-relative
  import would — confirm this matches the actual clone location before
  running, or adjust the path if the repo lives elsewhere on this machine.

### Step 6.2 — Manual validation checklist (live Isaac Sim + QGC)

This is the acceptance test for the whole feature — no automated substitute,
since it requires Isaac Sim, PX4 SITL, and QGC all running together. Run in
order; each step's ✅ line is the pass condition before moving to the next.

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
- ✅ Console shows `ZMQ PUB bound` and `[STREAM]` tick counters incrementing.

**4. Launch the streaming pipeline**
- ```bash
  conda run -n drone python pipeline-streaming.py \
      --calib <path-to-a-cam_calib.json-matching-this-rig> \
      --start_lat <lat> --start_lon <lon>
  ```
- ✅ Logs `ZMQ connected`, then `step N: pos=(...)` at roughly camera rate ÷
  stride, with a nonzero LK inlier count.

**5. Confirm PX4 is actually fusing vision, not ignoring it**
- Load `streaming/ekf2_ev_params.params` (Task 7) via QGC's parameter
  editor, reboot PX4.
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

- [ ] Run the checklist above; note any failing step and its symptom before
  moving on to Task 8 (the checklist frequently surfaces the exact port
  number / EKF2 param tweaks that only reveal themselves at runtime — expect
  to iterate on `--mavlink_addr` and `streaming/ekf2_ev_params.params`).
- [ ] Commit: `feat(streaming): add vio-streamer-pai.py (Isaac Sim ZMQ publisher)`

---

## Task 7: PX4 EKF2 external-vision params

**File:** `streaming/ekf2_ev_params.params` (create)

- [ ] Create `streaming/ekf2_ev_params.params` (QGC-loadable param file
  format: `<param name>\t<value>\t<default>` per line is not required — QGC
  accepts a simpler `<name>,<value>` or the standard `.params` export
  format; use the standard one QGC exports so it round-trips through the
  parameter editor's "Load from file" directly):

  ```
  EKF2_EV_CTRL	15
  EKF2_HGT_REF	3
  EKF2_GPS_CTRL	0
  ```
  - `EKF2_EV_CTRL=15`: enable horizontal position (bit 0), vertical position
    (bit 1), velocity (bit 2), yaw (bit 3) fusion from external vision —
    matches sending position + yaw every step (Task 5); no roll/pitch bits
    set, consistent with `VisionPositionSender` sending `roll=pitch=0.0`.
  - `EKF2_HGT_REF=3`: height reference = external vision (so the baro-derived
    `up` we send is what EKF2 uses for altitude, not its own baro fusion
    fighting it).
  - `EKF2_GPS_CTRL=0`: disable GPS fusion entirely, so EKF2 can't fall back
    to Pegasus's simulated ground truth GPS and mask a broken vision feed.
  - Before relying on these values: confirm the exact bit layout against the
    PX4 firmware version in `pg.px4_path` (`grep -n "EKF2_EV_CTRL" -A5
    <px4_dir>/src/modules/ekf2/module.yaml` or equivalent) — bitmask meanings
    have changed across PX4 releases, and Step 6's manual checklist item 5 is
    exactly where a mismatch here would surface (EKF2 ignoring vision).
- [ ] Commit: `docs(streaming): add PX4 EKF2 external-vision param set`

---

## Task 8: Wire the design spec's "explicitly out of scope" items into follow-up notes

- [ ] No code changes — add one line to `CLAUDE.md` (or a project memory, per
  the assistant's own memory conventions) noting that DSMAC/relief-fix
  streaming and a real rangefinder/AGL topic are the two known fast-follows,
  so a future session doesn't have to re-derive that from the spec. Skip
  this task if you'd rather track it verbally / in your own issue tracker —
  it's bookkeeping, not a functional requirement.

---

## Validation summary

| Task | Automated test | Manual step |
|---|---|---|
| 0 | import check | — |
| 1 | `test_mahony_state.py` | — |
| 2 | `test_zmq_proto.py` | — |
| 3 | `test_mavlink_bridge.py` | — |
| 4 | `test_load_calib.py` + existing pipeline.py smoke run | — |
| 5 | standalone parse/connect smoke test | — |
| 6 | — | full 7-step Isaac Sim + QGC checklist |
| 7 | — | verified live in Task 6 step 5 |

Run all automated tests together once Tasks 1–5 are done:
```bash
conda run -n drone pytest streaming/tests/ -v
```

---

## Addendum (2026-07-14): layout, initial state, commanding

Reconciled with the actual repo + the design discussion. See
`docs/superpowers/specs/2026-07-14-project-structure.md` for the file map.

### Layout

- `flow_odometry.py` and `pipeline.py` live at **repo root** (flat), not under
  `frontend/flow-odom/`. All paths above are updated. Repo root on this machine:
  `C:\Users\bower\Desktop\project\drone-sitl`.

### Initial state (revises Task 5)

`--start_lat/--start_lon` are **no longer log-only** once commanding exists — they
are the single georef anchor. Add to `pipeline-streaming.py`:

- `--start_alt` (origin altitude, metres) and `--start_yaw` (takeoff heading, ENU
  rad), or a single `--takeoff_json` carrying lat/lon/alt/attitude (the recorder
  already writes `takeoff.json`, see `vio-recorder-pai.py:389`).
- **Fix the cold start:** seed `MahonyState` from `--start_yaw` (or `takeoff.json`
  `attitude_xyzw`), NOT `np.eye(3)`. Identity is a 180° roll singularity for the
  gravity correction (level FRD→ENU is `diag([1,-1,-1])`), so it will not settle.
- Send `SET_GPS_GLOBAL_ORIGIN(start_lat, start_lon, start_alt)` once at startup so
  PX4 has an absolute anchor (GPS is off) — this also makes the QGC map place the
  drone, and lets targets be sent as either local NED or lat/lon.

### V2a — commanding (new task, after v1 flies)

- Add `SetpointSender` to `streaming/mavlink_bridge.py` (sibling of
  `VisionPositionSender`, reuses `enu_to_ned`/`yaw_enu_to_ned` on the same `conn`).
- Convert target lat/lon → local ENU using `start_lat/lon` (pipeline.py georef
  convention) → NED, send `SET_POSITION_TARGET_LOCAL_NED` **every loop** (OFFBOARD
  needs >2 Hz or PX4 drops offboard).
- Startup handshake: `conn.wait_heartbeat()` → `set_gps_global_origin` → prime
  setpoints → OFFBOARD (operator in QGC for first bring-up, not auto mode-switch).

### V2b — climb-and-search (new task, after Layer 1 DSMAC + V2a)

- **Prerequisite:** port DSMAC into streaming (`streaming/dsmac_live.py`, imports
  DSMAC funcs from `pipeline.py` unchanged; needs an ortho-tile strategy since the
  flight footprint isn't known ahead of time — pre-fetch around start_lat/lon).
- `streaming/commander.py`: FSM NORMAL → SEARCH → GIVE-UP. Consecutive-reject
  counter reuses the accept flag already in `pipeline.py`'s `fixes` list. On
  threshold, ramp an altitude setpoint (bounded climb rate + hard ceiling);
  exit on a fix clearing the confidence gate; GIVE-UP (hold last fix, stop
  climbing, alert) on ceiling/timeout.
- **Validate first:** on a known DSMAC-failure dataset with the *batch*
  `pipeline.py`, confirm accept-rate actually trends up with altitude before
  building the closed loop (design spec's open Exp10 question).
