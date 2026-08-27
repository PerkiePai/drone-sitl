# Implementation plan: GPS-denied flight on pipeline VIO

Design: `docs/superpowers/specs/2026-08-07-vio-gps-denied-design.md`.
Supersedes `2026-07-13-pipeline-streaming.md` (retained for its Task 1–4
rationale, which Tasks 1–3 here reuse).

## Goal

Fly the SITL drone in Isaac Sim with GNSS fusion disabled, positioned by this
repo's own LK flow-odometry, commanded from the existing web UI — with live VIO
drift against ground truth visible on the page.

## Architecture summary

```
Isaac Sim (Kit)                drone conda env               drone conda env
vio-streamer.py  ──ZMQ:5556──▶ pipeline-streaming.py ──ZMQ:5557──▶ joystick-server.py ──14540──▶ PX4
 imu/baro/frame/gt/meta         MahonyState + LK flow-odom         VISION_POSITION_ESTIMATE
                                ENU integration + drift            EKF2 params, GPS origin
```

## Tech stack

`drone` conda env — already has everything: `zmq 27.1.0`, `msgpack 1.2.1`,
`cv2 4.13.0`, `numpy 2.4.4`, `scipy 1.17.1`, `pymavlink 2.4.49`, `pytest 9.1.1`,
`fastapi`, `uvicorn`. **No install task.** Isaac Sim's embedded Python needs no
new deps (`pyzmq`/`msgpack` ship with Kit — verified in Task 4 Step 4.1).

PX4 **v1.14.3** at `~/PX4-Autopilot`.

## Global constraints

- **Reuse, never fork.** `_detect`, `_track_lk`, `_solve_translation` are imported
  from `pipeline.py` unchanged; Mahony math is refactored in place in
  `flow_odometry.py` so exactly one implementation exists.
- **`joystick-server.py` stays the sole MAVLink owner.** No new code opens a
  connection to PX4. The ZMQ subscriber thread never touches `conn`; it writes a
  lock-guarded slot the setpoint thread reads, exactly as `CommandState` does.
- **Ground truth never enters the estimate** (design D4). The `gt` topic feeds
  scoring only.
- Shared importable code in `streaming/`; entry-point scripts at repo root.
- Every MAVLink/PX4 constant carries its source file and line, per
  `streaming/offboard.py:7-9`.

## Task list

| # | Task | Files |
|---|---|---|
| 0 | Branch | — |
| 1 | `MahonyState` incremental AHRS | `flow_odometry.py` |
| 2 | ZMQ wire format | `streaming/zmq_proto.py` |
| 3 | Vision bridge (VPE, EKF2 params, origin) | `streaming/vision_bridge.py` |
| 4 | Isaac publisher | `vio-streamer.py` |
| 5 | Streaming estimator | `pipeline-streaming.py` |
| 6 | `--vision` wiring | `joystick-server.py` |
| 7 | VIO row on the page | `web/`, `streaming/tests/test_web_ui.py` |
| 8 | Live bring-up | — |
| 9 | Diagnostic instrumentation | `sim/save-ulog.sh`, `streaming/vision_bridge.py`, `vio-streamer.py`, `pipeline-streaming.py`, `joystick-server.py` |
| 10 | `MPC_XY_*` gain sweep | `streaming/vision_bridge.py`, `joystick-server.py`, `sim/mpc_gain_sweep.py`, `sim/analyze_gain_sweep.py` |

Tasks 1–3 are pure unit work, no sim required. Task 4 needs Isaac; Task 5 can be
driven from a recorded dataset without it.

---

## Task 0 — Branch

- [ ] **0.1** Commit or stash outstanding work first — the tree currently carries
      the uncommitted web refactor (`joystick-server.py`, `web/index.html`,
      `web/css/`, `web/js/`) and a `+9` line change to `vio-recorder-pai.py`.

```bash
cd ~/pai/drone-sitl
git status --short
git checkout -b feat/vio-gps-denied
```

Also deal with `NvStreamer-20260807-111234.etli` (64 MB untracked trace) — delete
it or add to `.gitignore`; it must not land in a commit.

---

## Task 1 — `MahonyState`, incremental AHRS

**Consumes:** one IMU sample at a time (`gyro`, `acc`, `dt`, optional `mag`).
**Produces:** current FRD→ENU rotation; `.R_flu()` for FLU→ENU, the convention
`load_dataset` and `run()` use.

Today the math is trapped in a batch loop over `imu.csv`
(`flow_odometry.py:162-182`). Streaming needs it per-tick. This is a refactor of
validated math — Step 1.3 pins that batch output does not change.

### Step 1.1 — Failing test

Create `streaming/tests/test_mahony_state.py`:

```python
"""Unit tests for flow_odometry.MahonyState. No dataset, no sim."""
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from flow_odometry import MahonyState  # noqa: E402

GRAV_FRD_LEVEL = np.array([0.0, 0.0, -9.81])
"""Specific force read by a level FRD accelerometer at rest.

FRD's z axis points DOWN, and an accelerometer at rest measures specific force,
which points UP -- so the reading is negative on z.
"""


def test_identity_seed_holds_still_with_no_input():
    m = MahonyState(np.eye(3), Kp=0.0)
    R = m.update([0.0, 0.0, 0.0], GRAV_FRD_LEVEL, 0.005)
    assert np.allclose(R, np.eye(3), atol=1e-12)


def test_gyro_integrates_about_z():
    m = MahonyState(np.eye(3), Kp=0.0)
    for _ in range(100):
        m.update([0.0, 0.0, 1.0], GRAV_FRD_LEVEL, 0.01)   # 1 rad/s for 1 s
    ang = Rot.from_matrix(m.R).as_rotvec()
    assert ang[2] == pytest.approx(1.0, abs=1e-3)


def test_gravity_correction_levels_a_tilted_seed():
    """The whole point of the accelerometer term: a wrong initial roll/pitch is
    pulled back toward gravity, while yaw -- which gravity cannot observe --
    keeps whatever error it started with."""
    R0 = Rot.from_euler("x", 10.0, degrees=True).as_matrix()
    m = MahonyState(R0, Kp=1.0)
    for _ in range(2000):
        m.update([0.0, 0.0, 0.0], GRAV_FRD_LEVEL, 0.005)
    roll = Rot.from_matrix(m.R).as_euler("xyz", degrees=True)[0]
    assert abs(roll) < 1.0


def test_from_heading_seeds_yaw_and_is_level():
    """Compass 0=N/90=E, ENU yaw 0=E measured counter-clockwise, so the two
    differ by yaw_enu = 90 - heading."""
    m = MahonyState.from_heading(90.0)              # due East
    yaw = np.arctan2(m.R_flu()[1, 0], m.R_flu()[0, 0])
    assert yaw == pytest.approx(0.0, abs=1e-9)
    m = MahonyState.from_heading(0.0)               # due North
    yaw = np.arctan2(m.R_flu()[1, 0], m.R_flu()[0, 0])
    assert yaw == pytest.approx(np.pi / 2, abs=1e-9)


def test_from_heading_is_not_the_identity_singularity():
    """np.eye(3) as an FRD->ENU seed puts the body upside down: FRD z (down)
    lands on ENU z (up), so v_pred is ANTIPARALLEL to the measured gravity
    direction, their cross product is zero, and the correction that is supposed
    to level the filter is dead on arrival. from_heading must not do that."""
    m = MahonyState.from_heading(0.0)
    v_pred = m.R.T @ np.array([0.0, 0.0, 9.81])
    v_meas = GRAV_FRD_LEVEL / np.linalg.norm(GRAV_FRD_LEVEL)
    assert np.linalg.norm(np.cross(v_meas, v_pred / np.linalg.norm(v_pred))) < 1e-9
    assert v_pred[2] < 0            # agrees with the reading, not opposed to it


def test_dt_outlier_is_clamped_like_the_batch_loop():
    """flow_odometry.py:164 substitutes 0.004 s for a non-positive or >0.1 s
    gap. Streaming sees those on ZMQ reconnects, so the guard must live in
    update(), not in the batch caller."""
    a = MahonyState(np.eye(3), Kp=0.0)
    a.update([0.0, 0.0, 1.0], GRAV_FRD_LEVEL, 5.0)
    b = MahonyState(np.eye(3), Kp=0.0)
    b.update([0.0, 0.0, 1.0], GRAV_FRD_LEVEL, 0.004)
    assert np.allclose(a.R, b.R)
```

```bash
conda run -n drone pytest streaming/tests/test_mahony_state.py -q
```
Expect: `ImportError: cannot import name 'MahonyState'`.

### Step 1.2 — Implement

In `flow_odometry.py`, above `compute_ahrs_attitude`:

```python
class MahonyState:
    """Mahony complementary filter carried one IMU tick at a time.

    Holds the rotation that compute_ahrs_attitude's loop kept in a local. The
    internal frame is FRD->ENU, matching that loop; callers wanting the
    FLU->ENU convention used by load_dataset and run() call R_flu().

    Extracted so the streaming estimator and the batch pipeline run the SAME
    filter -- test_mahony_state.py pins the behaviour, and
    test_ahrs_regression.py pins that the batch output did not move.
    """

    FLIP = np.diag([1.0, -1.0, -1.0])     # FLU<->FRD, self-inverse
    G_UP = np.array([0.0, 0.0, 9.81])     # specific force at rest points UP in ENU

    def __init__(self, R0_frd, Kp=1.0, mag_gain=0.0, mag_noise_deg=0.0, rng=None):
        self.R = np.asarray(R0_frd, dtype=float).copy()
        self.Kp = Kp
        self.mag_gain = mag_gain
        self.mag_noise_deg = mag_noise_deg
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.m_world = None

    @classmethod
    def from_flu(cls, R0_flu, **kw):
        """Seed from an FLU->ENU rotation (what load_dataset stores as R_wb)."""
        return cls(np.asarray(R0_flu, dtype=float) @ cls.FLIP, **kw)

    @classmethod
    def from_heading(cls, heading_deg, **kw):
        """Seed level, at a compass heading. The LIVE cold start.

        The batch filter initialises from GT frame 0 (flow_odometry.py:144),
        which streaming does not have. np.eye(3) is not a usable substitute: as
        an FRD->ENU rotation it means "body upside down", which is a singularity
        for the gravity correction -- the predicted and measured gravity
        directions come out antiparallel, their cross product vanishes, and the
        filter can never level itself.

        Compass is 0=N and clockwise; ENU yaw is 0=E and counter-clockwise.
        """
        yaw = np.radians(90.0 - heading_deg)
        return cls.from_flu(Rotation.from_euler("z", yaw).as_matrix(), **kw)

    def calibrate_mag(self, m_body0):
        """One-time factory-compass calibration: fix the world magnetic vector
        from the current attitude and a body reading."""
        n = np.linalg.norm(m_body0)
        if n > 1e-9:
            self.m_world = self.R @ (np.asarray(m_body0, dtype=float) / n)

    def update(self, gyro, acc, dt, mag=None):
        """One IMU tick. Returns the updated FRD->ENU rotation."""
        if dt <= 0 or dt > 0.1:
            dt = 0.004                      # matches flow_odometry.py:164-165
        w = np.asarray(gyro, dtype=float).copy()
        a = np.asarray(acc, dtype=float)
        an = np.linalg.norm(a)
        v_pred = None
        if an > 1e-3:                       # tilt correction toward gravity
            v_meas = a / an
            v_pred = self.R.T @ self.G_UP
            v_pred /= np.linalg.norm(v_pred)
            w = w + self.Kp * np.cross(v_meas, v_pred)
        if self.m_world is not None and mag is not None and self.mag_gain > 0.0:
            mn = np.linalg.norm(mag)
            if mn > 1e-6:
                m_meas = np.asarray(mag, dtype=float) / mn
                if self.mag_noise_deg > 0.0 and v_pred is not None:
                    ang = np.radians(self.mag_noise_deg) * self.rng.standard_normal()
                    m_meas = Rotation.from_rotvec(v_pred * ang).apply(m_meas)
                m_pred = self.R.T @ self.m_world
                m_pred /= np.linalg.norm(m_pred)
                w = w + self.mag_gain * np.cross(m_meas, m_pred)
        self.R = self.R @ Rotation.from_rotvec(w * dt).as_matrix()
        return self.R

    def R_flu(self):
        """FLU->ENU, the convention load_dataset and run() expect."""
        return self.R @ self.FLIP
```

**`Rotation` is not available at module scope.** `flow_odometry.py`'s only import
of it is function-local, inside `compute_ahrs_attitude` (`flow_odometry.py:134`,
`from scipy.spatial.transform import Rotation as Rot`). `MahonyState` is a
module-level class, so add the import to the top of the file:

```python
from scipy.spatial.transform import Rotation
```

Keep the function-local `as Rot` alias where it is — `compute_ahrs_attitude` uses
`Rot` in several places and rewriting them is churn unrelated to this task.

```bash
conda run -n drone pytest streaming/tests/test_mahony_state.py -q
```
Expect: `7 passed`.

### Step 1.3 — Refactor the batch function onto it, prove output unchanged

First capture the current behaviour as a fixture, **before** editing:

```bash
conda run -n drone python -c "
import numpy as np, glob, sys
from flow_odometry import load_dataset, compute_ahrs_attitude
d = sorted(glob.glob('$HOME/vio_dataset/*'))[-1]
K, R_CtoI, recs = load_dataset(d)
out = compute_ahrs_attitude(d, recs[:400])
np.save('streaming/tests/fixtures/ahrs_baseline.npy', np.array(out))
print('saved', np.array(out).shape, 'from', d)
"
```

If `~/vio_dataset` is empty, record one first (Task 4 depends on the same rig) or
skip to Step 1.4 and treat Step 1.3 as blocked — do **not** refactor without a
baseline.

Now replace the per-tick body of `compute_ahrs_attitude`'s loop with the class,
keeping the per-rec `gt_yaw` stand-in and the sampling loop exactly as they are:

```python
    flip = MahonyState.FLIP
    state = MahonyState.from_flu(recs[0]["R_wb"], Kp=Kp, mag_gain=mag_gain,
                                 mag_noise_deg=mag_noise_deg, rng=rng)
    if has_mag and mag_gain > 0.0:
        i0 = int(np.clip(np.searchsorted(ts, rec_ts[0]), 0, len(ts) - 1))
        state.calibrate_mag(mag[max(0, i0 - 25):i0 + 25].mean(axis=0))

    out = [recs[0]["R_wb"]]                 # frame 0 = GT init
    ri = 1
    for k in range(1, len(rows)):
        state.update(gyro[k], acc[k], ts[k] - ts[k - 1],
                     mag=mag[k] if mag is not None else None)
        while ri < len(recs) and ts[k] >= rec_ts[ri]:
            if gt_yaw is not None:          # legacy GT-yaw stand-in
                yaw = np.arctan2(state.R[1, 0], state.R[0, 0])
                meas = gt_yaw[ri] + np.radians(mag_noise_deg) * rng.standard_normal()
                dpsi = np.arctan2(np.sin(meas - yaw), np.cos(meas - yaw))
                state.R = Rotation.from_rotvec(
                    [0, 0, mag_gain * dpsi]).as_matrix() @ state.R
            out.append(state.R_flu())
            ri += 1
    while len(out) < len(recs):
        out.append(state.R_flu())
    return out
```

Regression test — create `streaming/tests/test_ahrs_regression.py`:

```python
"""The MahonyState refactor must not move batch AHRS output at all."""
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "fixtures", "ahrs_baseline.npy")


@pytest.mark.skipif(not os.path.exists(FIXTURE),
                    reason="no baseline fixture; see plan Task 1 Step 1.3")
def test_batch_output_matches_pre_refactor_baseline():
    import glob
    from flow_odometry import load_dataset, compute_ahrs_attitude
    d = sorted(glob.glob(os.path.expanduser("~/vio_dataset/*")))[-1]
    _, _, recs = load_dataset(d)
    got = np.array(compute_ahrs_attitude(d, recs[:400]))
    assert np.allclose(got, np.load(FIXTURE), atol=1e-12)
```

```bash
mkdir -p streaming/tests/fixtures
conda run -n drone pytest streaming/tests/ -q
```
Expect all green, including the regression test (or a clear skip if no dataset).

**Commit:** `refactor(vio): extract MahonyState so streaming and batch share one filter`

---

## Task 2 — ZMQ wire format

**Consumes/produces:** msgpack multipart frames `[topic, payload]`. One module
both sides import so the format cannot drift.

### Step 2.1 — Failing test

`streaming/tests/test_zmq_proto.py`:

```python
"""Wire-format round-trip. No sockets: pack/unpack are pure functions."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import zmq_proto  # noqa: E402


def test_imu_round_trip():
    payload = {"ts_ns": 1234567890, "w": [0.1, -0.2, 0.3], "a": [0.0, 0.0, -9.81]}
    topic, got = zmq_proto.unpack(zmq_proto.pack(zmq_proto.TOPIC_IMU, payload))
    assert topic == zmq_proto.TOPIC_IMU
    assert got == payload


def test_frame_carries_raw_jpeg_bytes_untouched():
    """The frame topic is the only binary payload; msgpack must not coerce it
    to str, or cv2.imdecode gets garbage."""
    jpg = bytes([0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10]) + b"\x00\x01\x02\xfe"
    payload = {"frame_idx": 7, "ts_ns": 42, "jpg": jpg}
    _, got = zmq_proto.unpack(zmq_proto.pack(zmq_proto.TOPIC_FRAME, payload))
    assert got["jpg"] == jpg
    assert isinstance(got["jpg"], bytes)


def test_meta_round_trip_keeps_nested_matrices():
    payload = {"site": "bangkok-survey-040",
               "origin": {"lat": 13.66156872, "lon": 100.298235, "h": 0.0},
               "K": [[638.0, 0.0, 480.0], [0.0, 638.0, 300.0], [0.0, 0.0, 1.0]],
               "R_CtoI": [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
               "image_size": [960, 600], "vib_damp": False}
    _, got = zmq_proto.unpack(zmq_proto.pack(zmq_proto.TOPIC_META, payload))
    assert got == payload


def test_floats_survive_at_full_precision():
    """Position is metres and lat/lon is degrees; a float32 round-trip would
    put metre-scale error into the georeference."""
    payload = {"lat": 13.661568721234567, "x": 1234.5678901234567}
    _, got = zmq_proto.unpack(zmq_proto.pack(zmq_proto.TOPIC_VIO, payload))
    assert got["lat"] == payload["lat"]
    assert got["x"] == payload["x"]


def test_unknown_topic_is_rejected_not_guessed():
    with pytest.raises(ValueError):
        zmq_proto.pack(b"nonsense", {})


def test_pack_returns_two_parts_for_multipart_send():
    parts = zmq_proto.pack(zmq_proto.TOPIC_BARO, {"ts_ns": 1, "alt_m": 2.0})
    assert len(parts) == 2
    assert parts[0] == zmq_proto.TOPIC_BARO
```

```bash
conda run -n drone pytest streaming/tests/test_zmq_proto.py -q
```
Expect: `ModuleNotFoundError: No module named 'zmq_proto'`.

### Step 2.2 — Implement

`streaming/zmq_proto.py`:

```python
"""ZMQ wire format shared by vio-streamer.py, pipeline-streaming.py and
joystick-server.py.

Two buses:
  :5556  Isaac  -> estimator      meta, imu, baro, frame, gt
  :5557  estimator -> web server  vio

msgpack rather than JSON because the frame topic carries raw JPEG bytes, and
because float64 survives exactly -- position is in metres and the origin is in
degrees, so a float32 round-trip would inject metre-scale georeference error.
"""
import msgpack

TOPIC_META = b"meta"
TOPIC_IMU = b"imu"
TOPIC_BARO = b"baro"
TOPIC_FRAME = b"frame"
TOPIC_GT = b"gt"
TOPIC_VIO = b"vio"

TOPICS = (TOPIC_META, TOPIC_IMU, TOPIC_BARO, TOPIC_FRAME, TOPIC_GT, TOPIC_VIO)

SENSOR_PORT = 5556      # Isaac -> estimator
VIO_PORT = 5557         # estimator -> web server


def pack(topic, payload):
    """-> [topic, msgpack] for socket.send_multipart()."""
    if topic not in TOPICS:
        raise ValueError(f"unknown topic {topic!r}; expected one of {TOPICS}")
    return [topic, msgpack.packb(payload, use_bin_type=True)]


def unpack(parts):
    """[topic, msgpack] -> (topic, payload)."""
    topic, blob = parts[0], parts[1]
    if topic not in TOPICS:
        raise ValueError(f"unknown topic {topic!r}; expected one of {TOPICS}")
    return topic, msgpack.unpackb(blob, raw=False)
```

```bash
conda run -n drone pytest streaming/tests/test_zmq_proto.py -q
```
Expect: `6 passed`.

**Commit:** `feat(vio): add the ZMQ wire format shared by all three processes`

---

## Task 3 — Vision bridge: VPE, EKF2 params, GPS origin

**Consumes:** an ENU pose plus its timestamp. **Produces:** MAVLink writes on a
connection owned by someone else — every method is setpoint-thread-only, the same
contract as `OffboardLink` (`streaming/offboard.py:196-198`).

### Step 3.1 — Failing test

`streaming/tests/test_vision_bridge.py`:

```python
"""Unit tests for streaming/vision_bridge.py. No PX4, no Isaac Sim."""
import math
import os
import sys

import pytest
from unittest.mock import MagicMock

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import offboard  # noqa: E402
import vision_bridge  # noqa: E402


@pytest.fixture
def conn():
    c = MagicMock()
    c.mav = MagicMock()
    return c


# --- ENU -> NED ------------------------------------------------------------

def test_enu_to_ned_swaps_east_north_and_flips_up():
    assert vision_bridge.enu_to_ned(1.0, 2.0, 3.0) == (2.0, 1.0, -3.0)


def test_enu_to_ned_is_its_own_inverse():
    n, e, d = vision_bridge.enu_to_ned(4.0, -5.0, 6.0)
    assert vision_bridge.enu_to_ned(n, e, d) == (-5.0, 4.0, -6.0)


# --- VISION_POSITION_ESTIMATE ---------------------------------------------

def test_send_converts_enu_to_ned_and_yaw_to_radians(conn):
    s = vision_bridge.VisionPositionSender(conn)
    assert s.send(vision_bridge.VisionPose(
        ts_ns=1_000_000_000, x=1.0, y=2.0, z=3.0,
        roll=0.0, pitch=0.0, yaw=math.pi / 2), now=1.0)
    args = conn.mav.vision_position_estimate_send.call_args[0]
    assert args[1:4] == (2.0, 1.0, -3.0)          # x=N, y=E, z=Down
    assert args[6] == pytest.approx(math.pi / 2)   # yaw already radians


def test_send_stamps_microseconds_not_nanoseconds(conn):
    s = vision_bridge.VisionPositionSender(conn)
    s.send(vision_bridge.VisionPose(ts_ns=2_500_000_000, x=0, y=0, z=0,
                                    roll=0, pitch=0, yaw=0), now=1.0)
    assert conn.mav.vision_position_estimate_send.call_args[0][0] == 2_500_000


def test_stale_pose_is_dropped_not_repeated(conn):
    """A frozen-but-plausible position is worse than none: EKF2 has failsafes
    for a lost source and none for a lying one."""
    s = vision_bridge.VisionPositionSender(conn, max_age_s=0.5)
    pose = vision_bridge.VisionPose(ts_ns=0, x=0, y=0, z=0,
                                    roll=0, pitch=0, yaw=0)
    assert s.send(pose, now=100.0, received_at=99.9)
    conn.mav.vision_position_estimate_send.reset_mock()
    assert not s.send(pose, now=100.0, received_at=99.0)
    conn.mav.vision_position_estimate_send.assert_not_called()


def test_none_pose_sends_nothing(conn):
    s = vision_bridge.VisionPositionSender(conn)
    assert not s.send(None, now=1.0)
    conn.mav.vision_position_estimate_send.assert_not_called()


# --- EKF2 params -----------------------------------------------------------

def test_gps_fusion_is_disabled():
    assert dict((n, v) for n, v, _ in
                vision_bridge.EKF2_VISION_PARAMS)["EKF2_GPS_CTRL"] == 0


def test_vision_claims_horizontal_position_and_yaw_only():
    """EKF2_EV_CTRL bit0=horizontal position, bit3=yaw (ekf2_params.c:687).
    Bit1 (vertical position) stays CLEAR because flow-odom takes altitude
    straight from the barometer (flow_odometry.py:455) -- setting it would feed
    baro back as an independent vision observation while EKF2 already fuses
    baro directly, double-counting one sensor. Bit2 (velocity) stays clear
    because VISION_POSITION_ESTIMATE carries no velocity."""
    ctrl = dict((n, v) for n, v, _ in
                vision_bridge.EKF2_VISION_PARAMS)["EKF2_EV_CTRL"]
    assert ctrl & 0b0001            # horizontal position
    assert ctrl & 0b1000            # yaw
    assert not ctrl & 0b0010        # NOT vertical position
    assert not ctrl & 0b0100        # NOT velocity


def test_height_reference_stays_on_the_barometer():
    """EKF2_HGT_REF value 0 = Barometric pressure (ekf2_params.c:657)."""
    assert dict((n, v) for n, v, _ in
                vision_bridge.EKF2_VISION_PARAMS)["EKF2_HGT_REF"] == 0


def test_vision_noise_is_looser_than_px4_defaults():
    """Defaults are 0.1 m / 0.1 rad -- far too tight for drifting flow-odom;
    EKF2 would reject its own vision source as inconsistent."""
    p = dict((n, v) for n, v, _ in vision_bridge.EKF2_VISION_PARAMS)
    assert p["EKF2_EVP_NOISE"] > 0.1
    assert p["EKF2_EVA_NOISE"] > 0.1
    assert p["EKF2_EV_NOISE_MD"] == 1     # use the params, not a reported variance


def test_apply_params_sends_every_param_with_its_declared_type(conn):
    link = offboard.OffboardLink(conn)
    vision_bridge.apply_ekf2_vision_params(link)
    sent = {c[0][2].decode(): (c[0][3], c[0][4])
            for c in conn.mav.param_set_send.call_args_list}
    assert len(sent) == len(vision_bridge.EKF2_VISION_PARAMS)
    for name, value, ptype in vision_bridge.EKF2_VISION_PARAMS:
        assert sent[name] == (pytest.approx(float(value)), ptype)


# --- SET_GPS_GLOBAL_ORIGIN -------------------------------------------------

def test_origin_uses_degE7_and_millimetres(conn):
    """SET_GPS_GLOBAL_ORIGIN: latitude/longitude degE7, altitude MILLIMETRES."""
    link = offboard.OffboardLink(conn)
    vision_bridge.send_gps_global_origin(link, 13.66156872, 100.298235, 12.5)
    args = conn.mav.set_gps_global_origin_send.call_args[0]
    assert args[1] == 136615687
    assert args[2] == 1002982350
    assert args[3] == 12500
```

```bash
conda run -n drone pytest streaming/tests/test_vision_bridge.py -q
```
Expect: `ModuleNotFoundError: No module named 'vision_bridge'`.

### Step 3.2 — Implement

`streaming/vision_bridge.py`:

```python
"""VISION_POSITION_ESTIMATE, the EKF2 vision param set, and the GPS origin.

Imported by joystick-server.py, which owns the only MAVLink connection in this
system. Same contract as OffboardLink: SETPOINT-THREAD ONLY -- pymavlink
connections are not thread-safe.

PX4 v1.14.3. Every param value below was read out of ~/PX4-Autopilot, with the
source line on each one, per the convention in offboard.py:7-9.
"""
import math
from dataclasses import dataclass

from offboard import MAV_PARAM_TYPE_INT32, MAV_PARAM_TYPE_REAL32

# EKF2 vision fusion. Applied only under --vision: EKF2_GPS_CTRL=0 is a
# deliberate, safety-relevant act, never a default.
#
#   EKF2_GPS_CTRL     ekf2_params.c:706, default 7. 0 disables ALL GNSS fusion.
#   EKF2_EV_CTRL      ekf2_params.c:687, default 15. bit0 horizontal position,
#                     bit1 vertical position, bit2 3D velocity, bit3 yaw.
#                     9 = horizontal position + yaw, which is exactly what
#                     flow-odom measures. Vertical is deliberately NOT claimed:
#                     altitude comes straight from the barometer
#                     (flow_odometry.py:455), and EKF2 already fuses baro
#                     directly (EKF2_BARO_CTRL default 1), so setting bit1 would
#                     feed one sensor in twice and read as spurious agreement.
#   EKF2_HGT_REF      ekf2_params.c:657, default 1 (GPS). 0 = barometric, which
#                     is where the height genuinely comes from.
#   EKF2_EV_NOISE_MD  ekf2_params.c:814, default 0. 1 = use the noise params
#                     below rather than a reported variance we do not compute.
#   EKF2_EVP_NOISE    ekf2_params.c:837, default 0.1 m -- far too tight for a
#   EKF2_EVA_NOISE    ekf2_params.c:857, default 0.1 rad -- drifting estimator;
#                     EKF2 would reject its own vision source as inconsistent.
EKF2_VISION_PARAMS = (
    ("EKF2_GPS_CTRL", 0, MAV_PARAM_TYPE_INT32),
    ("EKF2_EV_CTRL", 9, MAV_PARAM_TYPE_INT32),
    ("EKF2_HGT_REF", 0, MAV_PARAM_TYPE_INT32),
    ("EKF2_EV_NOISE_MD", 1, MAV_PARAM_TYPE_INT32),
    ("EKF2_EVP_NOISE", 0.5, MAV_PARAM_TYPE_REAL32),
    ("EKF2_EVA_NOISE", 0.2, MAV_PARAM_TYPE_REAL32),
)

DEFAULT_MAX_AGE_S = 0.5


@dataclass(frozen=True)
class VisionPose:
    """One estimate, in the estimator's ENU frame. Angles in radians."""
    ts_ns: int
    x: float          # East
    y: float          # North
    z: float          # Up
    roll: float
    pitch: float
    yaw: float


def enu_to_ned(e, n, u):
    """(East, North, Up) -> (North, East, Down)."""
    return (n, e, -u)


def apply_ekf2_vision_params(link):
    """Push the vision param set. Setpoint thread only."""
    for name, value, ptype in EKF2_VISION_PARAMS:
        link.set_param(name, value, ptype)


def send_gps_global_origin(link, lat_deg, lon_deg, alt_m):
    """Anchor PX4's global frame. Setpoint thread only.

    Mandatory with GNSS off: VISION_POSITION_ESTIMATE carries local NED offsets
    and no lat/lon, so without this PX4 has no absolute reference. Three shipped
    features fail SILENTLY without it -- the map marker (GLOBAL_POSITION_INT),
    waypoint flight (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, offboard.py:222-238) and
    the FLY gate (HOME_POSITION, joystick-server.py:73-76).

    Altitude is MILLIMETRES here, unlike every other altitude in this codebase.
    """
    link.conn.mav.set_gps_global_origin_send(
        link.target_system,
        int(round(lat_deg * 1e7)),
        int(round(lon_deg * 1e7)),
        int(round(alt_m * 1000.0)))


class VisionPositionSender:
    """Streams VISION_POSITION_ESTIMATE. Setpoint thread only."""

    def __init__(self, conn, max_age_s=DEFAULT_MAX_AGE_S):
        self.conn = conn
        self.max_age_s = max_age_s
        self.sent = 0
        self.dropped_stale = 0

    def send(self, pose, now, received_at=None):
        """Send one estimate. Returns True if it went out.

        A pose older than max_age_s is DROPPED rather than repeated. Repeating a
        frozen position keeps EKF2 confident about a place the drone is not,
        and EKF2 has failsafes for a lost vision source but none for a lying
        one. `received_at` is wall time (time.monotonic) at which the estimate
        arrived over ZMQ -- not pose.ts_ns, which is SIM time and runs at its
        own rate under lockstep.
        """
        if pose is None:
            return False
        if received_at is not None and (now - received_at) > self.max_age_s:
            self.dropped_stale += 1
            return False
        x, y, z = enu_to_ned(pose.x, pose.y, pose.z)
        self.conn.mav.vision_position_estimate_send(
            int(pose.ts_ns // 1000),        # usec
            float(x), float(y), float(z),
            float(pose.roll), float(pose.pitch), float(pose.yaw))
        self.sent += 1
        return True
```

```bash
conda run -n drone pytest streaming/tests/ -q
```
Expect all green.

**Commit:** `feat(vio): add the vision bridge -- VPE, EKF2 params and the GPS origin`

---

## Task 4 — `vio-streamer.py`, the Isaac publisher

**Consumes:** Pegasus IMU/Barometer callbacks and the `down_cam` render product.
**Produces:** ZMQ PUB on `:5556` — `meta`, `imu`, `baro`, `frame`, `gt`.

Mirrors `vio-recorder-pai.py`'s wiring (461-line version) but publishes instead of
writing to disk. **Do not modify the recorder** — recording and streaming stay
independent, per the original design decision.

### Step 4.1 — Confirm Kit has pyzmq and msgpack

```bash
~/isaac-sim6/python.sh -c "import zmq, msgpack; print(zmq.__version__, msgpack.version)"
```

If either is missing, install into Kit's interpreter — **not** the conda env:

```bash
~/isaac-sim6/python.sh -m pip install pyzmq msgpack
```

Blocking: the rest of Task 4 cannot proceed without this.

### Step 4.2 — Write the streamer

Create `vio-streamer.py` at repo root. Structure, mirroring the recorder:

1. **Locate** vehicle, IMU, Barometer, `down_cam` — reuse the recorder's
   `find_cam_path("down_cam")` approach (`vio-recorder-pai.py:121-135`).
2. **Assert the mount is rigid** (design D3) — read `DOWN_VIB_DAMP` from the
   setup script's globals, publish it on `meta`, and print a loud warning if
   True.
3. **Compute `R_CtoI` analytically** — the constant `DOWN_IMG_ROLL_DEG` roll
   composed with the USD-camera −Z look convention (design D3). Cross-check
   against the live `down_mount` transform at startup and warn on disagreement
   beyond 1°.
4. **Bind** `zmq.PUB` on `tcp://*:5556`, `SNDHWM` small on `imu`/`gt`.
5. **Physics callback** at `DATA_FPS`: publish `imu`, `baro`, `gt`.
6. **Every `img_every` frames**: `rep.orchestrator.step(rt_subframes=1,
   pause_timeline=False)`, read the annotator, JPEG-encode grayscale at q92,
   publish `frame`.
7. **`meta` at 1 Hz**, so a restarted estimator re-primes without a sim restart.

The origin comes from `sim/sites.py` via `SITL_SITE` (design D2), not from
`read_cesium_georeference()` — the site config is authored, the stage is derived.

### Step 4.3 — Verify the render-sync call is safe under lockstep

**This step is not optional.** `rep.orchestrator.step()` inside a physics callback
carries the recorder's own warning: *"may be reentrant-unsafe on some Isaac Sim
builds (double physics step / hang)"* (`vio-recorder-pai.py:406-411`). The
recorder could absorb a stall; this cannot — PX4 SITL runs in **lockstep** with
Isaac (`SESSION.md:33-38`), so a hang there stalls the physics step PX4 is blocked
on, and the aircraft drops out of OFFBOARD mid-flight.

```bash
DRONE_SETUP_DOWN_VIB_DAMP=False ./sim/launch-sitl.sh
```

Then, with the streamer running and the drone hovering, for 60 s:

- [ ] `frame` counter advances at a steady ~15 Hz — no stalls, no bursts.
- [ ] `sim_rate` on the web page stays where it was **before** the streamer
      started (record the baseline first). A drop means `step()` is costing a
      physics step.
- [ ] PX4 stdout shows **no** `poll timeout` lines — the lockstep-failure
      signature from `SESSION.md:70-73`.
- [ ] The drone holds OFFBOARD without dropping out.

If any fail, drop the `step()` call and instead tag each frame with the annotator's
own timestamp, accepting ~1 render period of frame/pose skew. Record which branch
was taken in `SESSION.md` — this is exactly the class of failure that took an
entire session to diagnose last time.

### Step 4.4 — Smoke test

```bash
conda run -n drone python -c "
import zmq, sys, collections, time
sys.path.insert(0, 'streaming')
import zmq_proto
s = zmq.Context().socket(zmq.SUB)
s.connect('tcp://127.0.0.1:5556')
s.subscribe(b'')
n = collections.Counter(); t0 = time.time()
while time.time() - t0 < 10:
    topic, payload = zmq_proto.unpack(s.recv_multipart())
    n[topic] += 1
    if topic == zmq_proto.TOPIC_META and n[topic] == 1:
        print('meta:', payload)
for k, v in sorted(n.items()):
    print(f'{k.decode():6s} {v/10:7.1f} Hz')
"
```

Expect roughly: `imu ~200 Hz`, `baro ~200 Hz`, `gt ~200 Hz`, `frame ~15 Hz`,
`meta ~1 Hz`, and `meta` reporting `vib_damp: False`.

**Commit:** `feat(vio): publish Isaac sensor and camera streams over ZMQ`

---

## Task 5 — `pipeline-streaming.py`, the estimator

**Consumes:** ZMQ `:5556`. **Produces:** ZMQ PUB `:5557`, topic `vio` —
`{ts_ns, x, y, z, roll, pitch, yaw, n_inliers, fix_ok, frame_idx, drift_m, fps}`.

### Step 5.1 — Write it

- SUB all topics on `:5556`; block until the first `meta`.
- `MahonyState.from_heading(site.heading_deg)` — the live cold start (Task 1).
- Feed every `imu` into `state.update(...)`; keep the latest `baro` altitude.
- On each `frame` (respecting `--stride`): `cv2.imdecode` → `_detect` /
  `_track_lk` / `_solve_translation`, **imported unchanged from `pipeline.py`**.
- `h0 = max(baro_alt_above_takeoff, 0.3)`, matching `flow_odometry.py:408`.
- Integrate `dC` into ENU `pos`; `pos[2]` = baro altitude, per
  `flow_odometry.py:455`.
- Track drift against the latest `gt` sample — **for reporting only** (design D4).
- PUB `vio` after every processed frame.

### Step 5.2 — Offline smoke test, no Isaac required

Replay a recorded dataset through the same wire format. Create
`streaming/tests/replay_dataset.py` (a tool, not a test — it needs a dataset):

```bash
conda run -n drone python streaming/tests/replay_dataset.py ~/vio_dataset/<run> &
conda run -n drone python pipeline-streaming.py --no-mavlink --print-every 20
```

Expect per-frame inlier counts in the same range the batch pipeline reports on
that dataset, and final drift within ~20% of `flow_odometry.run()`'s on the same
data. A large gap means the streaming path diverged from batch — fix before
going near the sim.

- [ ] **GT isolation check** (design D4): rerun with the `gt` topic suppressed;
      the estimate must be **bit-identical**.

**Commit:** `feat(vio): add the streaming flow-odom estimator`

---

## Task 6 — `--vision` wiring in `joystick-server.py`

### Step 6.1 — Failing tests

Add to `streaming/tests/test_offboard_loop.py`. That file drives a **fake PX4 over
real UDP** on `FAKE_PX4_PORT = 14585` and loads the server with `_load_server()`
(`test_offboard_loop.py:17-26`); there is no `loop` fixture, so follow the existing
shape — construct `js.SetpointLoop(...)` inline and read messages with
`_collect(px4, seconds, kind=...)`, which already takes a message type:

```python
def test_vision_pose_is_forwarded_once_per_tick():
    """One VPE per setpoint tick -- the same steady-stream discipline the
    velocity setpoint already follows. Collect with
    _collect(px4, 1.0, kind="VISION_POSITION_ESTIMATE")."""


def test_setpoint_still_goes_out_when_vision_is_stale():
    """Vision going stale must NOT interrupt the setpoint stream: a gap there
    drops PX4 out of OFFBOARD (joystick-server.py:216-219), turning a degraded
    estimate into a loss of control. Assert SET_POSITION_TARGET_LOCAL_NED keeps
    arriving at ~20 Hz while VISION_POSITION_ESTIMATE stops."""


def test_ekf2_params_are_not_sent_without_the_vision_flag():
    """EKF2_GPS_CTRL=0 must never be applied by default. Collect PARAM_SET and
    assert no EKF2_* name appears."""


def test_gps_origin_is_sent_before_the_first_vision_estimate():
    """PX4 cannot place a local estimate without an anchor, so ordering is the
    assertion: SET_GPS_GLOBAL_ORIGIN must precede the first VPE."""


def test_gps_origin_is_resent_after_a_px4_restart():
    """A rebooted PX4 forgets the origin and the map silently stops updating.
    Simulate by pausing the fake PX4's heartbeats past the gap threshold, then
    resuming, and assert a second SET_GPS_GLOBAL_ORIGIN arrives."""


def test_zmq_thread_never_touches_the_mavlink_connection():
    """The sole-owner contract (offboard.py:196-198). Assert with a conn whose
    .mav raises if touched from any thread other than the setpoint thread."""
```

### Step 6.2 — Implement

In `joystick-server.py`:

- `--vision` flag; `--vio-endpoint` defaulting to `tcp://127.0.0.1:5557`.
- A `VisionSubscriber` thread storing `(VisionPose, received_at)` under a lock.
- In `_send_startup_params`, when `--vision`: `apply_ekf2_vision_params(link)`
  then `send_gps_global_origin(link, ...)` from the site config.
- In `run()`'s tick, after the existing setpoint send:
  `self.vision.send(pose, now, received_at)`.
- Telemetry gains a `vio` block: `{fresh, age_s, n_inliers, drift_m, fps, sent}`.

Re-send the origin when a `HEARTBEAT` gap indicates PX4 restarted.

```bash
conda run -n drone pytest streaming/tests/ -q
```

**Commit:** `feat(vio): fly GPS-denied -- forward vision estimates and set EKF2 up for it`

---

## Task 7 — VIO row on the page

`web/index.html` gains a row alongside the existing telemetry
(`web/index.html:24-32`); `web/js/telemetry.js` paints it with the existing
`good`/`wait`/`bad` vocabulary (`telemetry.js:84-105`).

Fields: `vio` (fresh/STALE), `drift` (m vs GT), `pts` (LK inliers), `fix` (Hz).

- [ ] Stale vision renders `bad` and is unmissable — it means the aircraft is
      flying on dead reckoning.
- [ ] `drift` shows `--` when no GT topic is present, never `0.0`.

Extend `streaming/tests/test_web_ui.py` in the existing style.

**Commit:** `feat(web): show VIO health and live drift on the flight page`

---

## Task 8 — Live bring-up

Order matters: each step isolates one failure class.

> **Status 2026-08-11:** 8.1-8.5 pass. **GPS-denied flight achieved** --
> `cs_gps: False`, `cs_ev_pos: True`, holding OFFBOARD on vision alone -- but it
> held only ~40 s before diverging, so 8.6 does not pass.
>
> The root cause behind the whole chain was `set_param` sending INT32 params as
> a float, so PX4 stored the bit pattern: `EKF2_EV_CTRL` was 1091567616, never
> 9, and vision fusion had never once switched on. Fixed. The next blocker is
> the estimator's own drift. The staleness clock is fixed (`dropped_stale` 0
> for a whole flight, was 179). What remains is that flow-odom accumulates
> 23-37 m of drift during the climb and nothing realigns the vision frame to
> PX4's at the handover, so EKF2 inherits that error and follows it away.
> Full write-up in `SESSION.md`.
>
> **Status 2026-08-13, run 5 flown:** the frame alignment and the ENU→NED
> attitude fix both flew. **The handover is solved** — drift through the climb
> fell from 23–37 m to 0.29–0.60 m, both realignments had almost nothing left to
> close, and the aircraft stayed in OFFBOARD throughout. What remains is a slow
> **growing oscillation**, ~40–60 s period, ±5 m growing to ±60 m, with 550–600
> inliers and no rejections. It is a control/estimation instability, not a
> tracking failure.
>
> **8.6 is redefined** (ADR-0001): 180 s vision-only, pass requires a non-growing
> excursion envelope, peak excursion recorded as a number. 60 s is barely one
> period of the observed mode and cannot distinguish damped from unstable.
>
> **Task 9 comes first** (ADR-0002): the next flight changes no parameter and
> classifies the mode instead. Reasons the plan did not anticipate — no flight has
> ever been recorded (ADR-0003), nothing computes aircraft excursion as opposed to
> estimator drift (ADR-0004), the estimator has no heading reference at all while
> EKF2 does (ADR-0005), and `EKF2_EV_DELAY` is measured on a clock whose rate
> wanders (ADR-0006).
>
> **Status 2026-08-19 (Task 10):** 8.6 retried with the best `MPC_XY_*`
> candidate the gain sweep found (`low_integral`), at the full 180 s bar.
> **Still fails** — an altitude-drop abort at t+~73 s (24.6 m → 4.5 m), not
> a horizontal runaway. Best result yet (78 m peak excursion, vs. 220-270 m
> in every pre-Task-10 attempt) but not a pass. See `SESSION.md`, Task 10
> Step 10.7 attempt, and Task 10's own status blockquote.
>
> **Status 2026-08-22 — 8.6 passes, up to ~60 m.** Six 180 s vision-only holds,
> none aborted; every hold flown first in a fresh session is inside ADR-0001's
> bar (2.02 m and 2.05 m at 49 m, 2.65 m at 59 m, all settling), against
> 158.6 m for the same gains before. **No parameter was changed** — Task 10's
> sweep is void as guidance, because it ranked candidates on `MPC_XY_P` while
> that gain was not in the loop at all. Three faults, each invisible until the
> one before it was fixed: the hover was commanded as zero *velocity* so the
> position loop was open (ADR-0008); the flow was derotated by an
> accelerometer-corrupted attitude; nothing estimated gyro bias.
>
> **What remains open is altitude.** The hold collapses above ~60 m — 25 m and
> 50 m peaks at 98 m against 1.6–2.1 m at 49 m. Flown and refuted: the
> estimator degrading with height (an open-loop sweep is flat to 125 m),
> position-gain scaling as 1/h, a climb-polluted gyro-bias estimate, and
> (2026-08-22, later) gating the accelerometer out of the derotation's bias.
> That last one identified a real mechanism — the Mahony integral absorbs
> `atan(a/g)` as a gyro bias and the derotation subtracts it, fabricating
> `1.26*h*|bias|` m/s — and the gate still came out net-negative in flight,
> halving the 100 m peak but breaking the 50 m hold (2.0 -> 13.7 m), because a
> frozen bias is a coherent ramp where a wandering one partly cancels.
> Reverted. The surviving hypothesis and the one experiment that would settle
> it — a 98 m phase-2 hold on velocity setpoints, position loop open — are at
> the end of `SESSION.md`, along with the new constraint that the frozen-bias
> walk was altitude-INDEPENDENT (0.094 m/s at 50 m, 0.096 at 100 m).
> **Established envelope: ~60 m at default gains.**
>
> **Status 2026-08-22, latest — the collapse is the POSITION LOOP, not the
> estimator.** The velocity-setpoint experiment named above was flown, with a
> 49 m companion. A 98 m phase-2 hold with the position loop OPEN holds flat
> at **3.34 m** (settling), and its 49 m companion at **2.97 m** — open-loop
> performance is altitude-INDEPENDENT, while the same aircraft, gains and
> estimator closed-loop goes 2 m -> 25-33 m over the same height change.
> Opening the loop at 98 m is worth a factor of 8-10.
>
> This is not a reason to revert ADR-0008: a velocity hold has no position
> reference and creeps on any velocity bias (+0.023 m/s on the 49 m run). It
> narrows the question to *what closing the loop around a vision position
> estimate does at 98 m that it does not do at 49 m* — measurement delay
> (`EKF2_EV_DELAY`, ADR-0006) and EKF2's own `estimator_ev_pos_bias` under a
> measurement now correlated with commanded motion are the two candidates, and
> the open/closed pair of 98 m ulogs to compare them is already on disk.
> `--open-loop-hold` is the (default-off, test-pinned) lever.

- [x] **8.1** `DRONE_SETUP_DOWN_VIB_DAMP=False ./sim/launch-sitl.sh` → drone
      spawned, Play pressed, MJPEG on 8080. **Done 2026-08-11.**
- [x] **8.2** Streamer up; `meta` reports `vib_damp: false` and the site origin.
      **Done** — verified over the wire by the estimator's prime line.
- [x] **8.3** Estimator up; drift vs GT printed and bounded while hovering.
      **Done** — 0.36-0.61 m at 49 m, ~600 inliers, 8.8 fps.
- [x] **8.4** **Baseline** — `joystick-server.py` **without** `--vision`. Fly
      normally on GPS. Confirms nothing regressed before vision is trusted.
      **Done** — took off to 49.2 m and held station.
- [~] **8.5** Restart with `--vision`. Confirm in QGC's parameter view that all
      six EKF2 params took, and that `GLOBAL_POSITION_INT` still produces a map
      position (proves the origin landed).
      **Partial** — origin landed and the VIO row works, but `EKF2_HGT_REF` is
      @reboot_required and cannot take effect when applied at connect time.
      See SESSION.md 2026-08-11.
- [x] **8.6** Arm, take off, hover **180 s** vision-only. Pass bar is a
      **non-growing aircraft-excursion envelope**, not zero drift (ADR-0001).
      **Done 2026-08-22** — peak aircraft excursion 2.02 m (+0.0029 m/s,
      settling) at 49.3 m, six holds flown, none aborted. Passes up to ~60 m;
      above that the hold collapses and the cause is open. See the status
      note above and `SESSION.md`.
- [ ] **8.7** Manual flight — forward/back/turn. Watch drift accumulate.
- [ ] **8.8** Fly a short map route vision-only. This exercises
      `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` against a vision-only estimator, the
      most likely place for an origin mismatch to surface.
- [ ] **8.9** **Failure mode** — kill the estimator mid-hover. VIO row goes red;
      PX4 failsafes on a lost source rather than lurching on a frozen one.
      Restart it; recovery without restarting the sim.
- [ ] **8.10** Record results in `SESSION.md`, including the Step 4.3 branch
      taken and measured drift over 60 s.

---

## Task 9 — Diagnostic instrumentation

Everything here is **measurement**. Nothing changes how the aircraft flies, which
is what lets the whole batch ship before a single flight (ADR-0002) — the
classification run stays a clean observation of the current system.

Ships as one batch, then one flight.

> **Status 2026-08-13:** 9.1–9.3 implemented and offline-verified (247/249
> tests pass in `streaming/tests/` + `sim/tests/`; the 2 failures are
> pre-existing and environmental — a live Isaac session's real recorder was up
> during this run, which is exactly what those two tests assume is not the
> case). `sim/save-ulog.sh` was run against the **live** PX4 process from an
> already-in-progress session and correctly copied a 394 MB `.ulg` out via
> `/proc/<pid>/cwd` — the mechanism is proven. **9.1d is not fully closed**:
> that live PX4 was never rebooted with the new `SDLOG_PROFILE=131`, so the
> copied ulog predates the computer-vision topics and `estimator_aid_src_ev_pos`
> was not confirmed present. 9.4 (the classification flight) has not been
> flown with this instrumentation — see `SESSION.md` for what's live right now
> and why a reboot wasn't forced mid-session.

### Step 9.1 — Get the flight recorder back

- [x] **9.1a** `sim/save-ulog.sh`: resolve PX4's working directory via
      `/proc/$(pgrep -f build/px4_sitl_default/bin/px4)/cwd`, copy `log/**/*.ulg`
      into `./logs/<timestamp>/`. Must run **while Isaac is still up** — the
      rootfs is a `tempfile.TemporaryDirectory` that dies with the process
      (`px4_launch_tool.py:44,63`). Refuse loudly if no PX4 process is found,
      rather than silently copying nothing. **Done and live-verified** — ran
      against the currently-running PX4 and copied a real `.ulg` out.
- [x] **9.1b** Add to `EKF2_BOOT_PARAMS` in `streaming/vision_bridge.py`:
      `SDLOG_MODE=2` (boot→shutdown) and `SDLOG_PROFILE=131` (bits 0 default,
      1 EKF2 replay, 7 computer vision). Both are `@reboot_required`
      (`logger/params.c:65,144`) and phase 0 already reboots, so they cost
      nothing extra. Both are INT32 — they go through the bit-pattern path in
      `offboard.set_param`, and the existing param-set test must cover them.
      **Done** — `test_boot_phase_enables_flight_logging_from_boot_to_shutdown`.
- [x] **9.1c** The docstring on `EKF2_BOOT_PARAMS` currently says these are EKF2
      vision params. Widen it: the tuple is now "params PX4 only re-reads at
      boot", which is what phase 0 has always actually meant. **Done.**
- [~] **9.1d** Verify on the ground: launch, run `save-ulog.sh`, confirm a
      non-empty `.ulg` and that `estimator_aid_src_ev_pos` is present in it.
      A broken capture must not be discovered after the 180 s flight.
      **Partial** — the copy mechanism is proven live, but the checked-out
      `.ulg` came from a PX4 that booted before `SDLOG_PROFILE=131` existed, so
      the CV topics were not confirmed. Needs a fresh phase-0 reboot to close.

### Step 9.2 — Give the estimator a magnetometer channel (gain 0)

- [x] **9.2a** `vio-streamer.py`: read Pegasus's `Magnetometer` sensor alongside
      `IMU`/`Barometer` and add `mx, my, mz` to the `imu` topic payload. If the
      vehicle has no magnetometer sensor, publish nothing and say so on `meta` —
      a silently absent field is the failure mode this project keeps paying for.
      **Done** — syntax-checked; this file only runs inside Kit, so it is
      unverified live until the streamer is restarted (see status note above).
- [x] **9.2b** `streaming/zmq_proto.py` round-trip test covers the new fields,
      including the absent case. **Done.**
- [x] **9.2c** `pipeline-streaming.py`: pass `mag=` through to
      `MahonyState.update()` and call `calibrate_mag()` once at prime.
      **`mag_gain` stays 0.0** for this flight (ADR-0005) — the channel is
      recorded, not acted on. Expose it as a flag so turning it on later is one
      argument. **Done** — `--mag-gain`, default 0.0; behaviour exercised
      directly (calibrates once, tolerates an absent `m` field, gain-0 update
      does not raise).

### Step 9.3 — Measure aircraft excursion, not just estimator drift

- [x] **9.3a** `pipeline-streaming.py`: include ground truth in the `vio` message
      alongside the estimate. The GT subscriber stays out of the update path —
      D4's isolation test must still pass unchanged. **Done** — `gt_x/gt_y/gt_z`,
      None when absent; exercised directly, confirmed GT never touches `self.pos`.
- [x] **9.3b** `joystick-server.py`: compute excursion from the hold point at
      the moment of the cut, add it to the `vio` telemetry block, render it on
      the VIO row next to `drift_m`. Two different numbers, two different labels
      — see `CONTEXT.md` on *estimator drift* versus *aircraft excursion*.
      **Done** — `_excursion_m()`, `t-vio-exc` on the page, resets to `--` on
      GNSS restore rather than showing a stale number.
- [x] **9.3c** Test that `VisionPositionSender` never reads the ground-truth
      field (ADR-0004). This is the fence that replaces D4's process separation.
      **Done** — `test_vision_pose_has_no_ground_truth_fields`.
- [x] **9.3d** 20 Hz run CSV from `joystick-server`: sim time, PX4 local pose,
      GT, estimator pose, excursion, `drift_m`, `align_m`, `realigned`,
      `n_inliers`, `fresh`, `dropped_stale`, `sim_rate`, phase. One row per tick,
      one file per run, written next to the ulog. **Done** — `RunCSV`,
      `./logs/<run-name>/run.csv`, flushed every row; `--run-name` lines it up
      with `sim/save-ulog.sh <name>`. Verified end-to-end (write/read-back) and
      cross-checked against `_vio_status()` in the same tick.

### Step 9.4 — The classification flight

- [x] **9.4a** Fly the standard profile to the cut, then hold **180 s**. Change
      no parameter. Save the ulog before shutting Isaac down. Flown as run 6
      (`20260813-231224`) — actual hold was 106 s, not 180 s (see 9.4d note).
- [x] **9.4b** Classify the mode from the CSV: an orbit or spiral indicts
      heading; a straight-line back-and-forth indicts lag or loop gain.
      **Straight line, one fixed bearing, damped overshoot to a false
      steady-state — not a spiral.** See SESSION.md, Task 9.4b.
- [x] **9.4c** From the ulog, plot applied EV delay (`estimator_aid_src_ev_pos`
      fusion timestamp vs sample timestamp) against `sim_rate`. If it tracks the
      rate, `EKF2_EV_DELAY` is a sim artifact and the answer is to stabilise the
      rate, not to tune the param (ADR-0006). **It does not track the rate** —
      see SESSION.md, Task 9.4c.
- [x] **9.4d** Plot recorded magnetometer heading against the Mahony yaw and
      against GT yaw. This is what says whether the compass would have held it.
      GT yaw was never recorded (only `gt_x`/`gt_y`); substituted an
      independent raw-magnetometer heading computed from the ulog's
      `sensor_mag` + `vehicle_attitude`. **Heading is ruled out** — see
      SESSION.md, Task 9.4d.
- [x] **9.4e** Offline replay sweep (`src/modules/replay`) over `EKF2_EV_DELAY`,
      `EKF2_EVP_NOISE` and `EKF2_EV_CTRL` with and without bit 3. **Open-loop**:
      it ranks candidates and kills bad ones; it cannot tell you the aircraft
      would have stopped oscillating. **All four candidates killed** — none
      changed the excursion shape by more than replay-to-replay noise. See
      SESSION.md, Task 9.4e.
- [x] **9.4f** Record in `SESSION.md`, in the same run-by-run style. Done
      incrementally as each of 9.4b-e completed, plus a closing summary.

Only then does 8.6 get attempted again, with whatever single change 9.4 justified.

> **Status 2026-08-14, Task 9 closed:** 9.4 justified no single parameter
> change (SESSION.md, Task 9.4b-e) — heading, the track's shape,
> `EKF2_EV_DELAY`, and every `EKF2_EV_*` vision-fusion param were each
> individually ruled out, including a replay sweep that killed all four. What
> is left is downstream of EKF2, in the position controller's own `MPC_XY_*`
> gains — Task 10.

---

## Task 10 — `MPC_XY_*` gain sweep for vision-only station-keeping

**Spec:** `docs/superpowers/specs/2026-08-14-mpc-xy-gain-sweep-design.md`
(decisions D1–D5 below refer to that document, not the D1–D7 in "Plan
validation", which are the original 2026-08-07 design's).

**Consumes:** the existing `/ws` command protocol (`{"type": "cmd", "name":
...}`, `joystick-server.py:1220`) and telemetry push (`joystick-server.py:1168`).
**Produces:** one new command, `set_param`; a `sim_s` telemetry field; a
campaign driver and an offline ranking script.

9.4e's own limitation is what points here: replay is open-loop, so it could
only rule EKF2 tuning out, not confirm what *would* fix the excursion. The
position controller flies today on PX4's stock, GPS-tuned `MPC_XY_*`
defaults, never adapted for a noisier vision source — the only untested lever
left, and the only one a live flight can screen quickly (design doc, "Why
this exists").

> **Status 2026-08-19:** Steps 10.1–10.5 done (offline, in a worktree at
> `.worktrees/task-10-mpc-gain-sweep`, branch `task-10-mpc-gain-sweep` off
> `feat/vio-gps-denied`): `MPC_XY_DEFAULTS`/`revert_mpc_gains`, `sim_s`
> telemetry, the `set_param` command, D4's revert wiring,
> `sim/mpc_gain_sweep.py`, `sim/analyze_gain_sweep.py`. Full offline suite
> (`streaming/tests/` + `sim/tests/`): 261 passed, 3 known failures (2
> environmental -- a live recorder was running against the box during this
> run; 1 a worktree-depth artifact in `test_sites.py`'s `../metashape/`
> relative path, present regardless of this task's changes and absent when
> run from the top-level checkout). **10.6–10.7 (the live screening and
> confirmation flights) not yet attempted.**
>
> **Status 2026-08-19, later — 10.6 done.** `sim/mpc_gain_sweep.py` had
> never flown before this session and took four live attempts to get real
> data, each exposing a genuine bug (missing `offboard` command; a frozen
> vision channel traced to run 1's runaway climb, needing an Isaac restart;
> an arm/takeoff race; too-tight a climb timeout) plus one analyzer bug
> (`rank_candidates` silently dropped every `aborted` candidate). All fixed
> and committed. Full write-up in `SESSION.md`, Task 10 Step 10.6. Ranked
> result: `gentler_p` is the clear standout (112.9 m peak vs. `baseline`'s
> 158.6 m; `more_damping`/`gentle_combo` both ran into the 400 m abort
> ceiling); `low_integral` still has no data (`failed_to_climb`, most
> likely stray GPU contention). **10.7 (confirmation flight) not yet
> attempted.**
>
> **Status 2026-08-19, later still.** `low_integral` re-run at confirm
> duration (180 s) to fill its gap: climbed and cut cleanly (the earlier
> failure was not a real bug), and is now the best candidate found —
> 78.1 m peak, 2.04 m/s slope, both better than `gentler_p`. But it did not
> hold the full 180 s either: D5's altitude-drop abort fired at t+~73 s
> (24.6 m → 4.5 m, a genuine descent, not a horizontal runaway — excursion
> was only 78 m at that point, nowhere near the 400 m ceiling). **8.6 is
> still open** — no candidate in this sweep has yet passed ADR-0001's bar.
> Full ranked table and detail in `SESSION.md`, Task 10 Step 10.7 attempt.
>
> **Status 2026-08-22 — closed, and void as guidance.** 8.6 passes, at
> `baseline`: the same stock gains that peaked at 158.6 m in Step 10.6 gave a
> 2.02 m peak once three faults outside the controller were fixed (Step 10.10,
> `SESSION.md`). **No gain was changed, and this sweep's ranking should not be
> used.** It scored candidates on `MPC_XY_P` while the position loop was open
> — station keeping went out as `send_velocity(0,0,0)`, so PX4 never computed
> a position error and that gain was not in the loop at all (ADR-0008). Its
> one apparent signal, `MPC_XY_VEL_I_ACC` low ranking best, is just the
> velocity integrator being the only term that could fight a velocity bias
> when nothing else was closed. `analyze_gain_sweep.py` is still the scoring
> tool, with `TREND_WINDOW_S` now 120 s — at 20 s it called the passing
> 2.02 m flight "growing". The open question is the >60 m collapse, tracked
> in Task 8's status above, and it is not a `MPC_XY_*` question.

### Step 10.1 — `MPC_XY_DEFAULTS` and `revert_mpc_gains`

Pure data plus one function, offline-testable exactly like
`EKF2_GNSS_RESTORE_PARAMS` / `apply_ekf2_gnss_restore_params` already are.

- [x] **10.1a Failing test.** Append to
      `streaming/tests/test_vision_bridge.py`, after the `# --- EKF2 params
      ---` section (before `# --- SET_GPS_GLOBAL_ORIGIN ---`):

```python
# --- MPC_XY_* gains (Task 10) -----------------------------------------------

def test_mpc_defaults_match_px4s_own_stock_values():
    """mc_pos_control_params.c:270,282,295,307 -- PX4's own shipped values,
    and D2's `baseline` candidate row."""
    defaults = dict((n, v) for n, v, _ in vision_bridge.MPC_XY_DEFAULTS)
    assert defaults == {
        "MPC_XY_P": 0.95,
        "MPC_XY_VEL_P_ACC": 1.8,
        "MPC_XY_VEL_I_ACC": 0.4,
        "MPC_XY_VEL_D_ACC": 0.2,
    }


def test_revert_mpc_gains_sends_exactly_the_four_defaults(conn):
    link = offboard.OffboardLink(conn)
    vision_bridge.revert_mpc_gains(link)
    sent = {c[0][2].decode(): (c[0][3], c[0][4])
            for c in conn.mav.param_set_send.call_args_list}
    assert len(sent) == len(vision_bridge.MPC_XY_DEFAULTS)
    for name, value, ptype in vision_bridge.MPC_XY_DEFAULTS:
        assert sent[name] == (pytest.approx(float(value)), ptype)
```

```bash
conda run -n drone pytest streaming/tests/test_vision_bridge.py -q
```
Expect: `AttributeError: module 'vision_bridge' has no attribute 'MPC_XY_DEFAULTS'`.

- [x] **10.1b Implement.** In `streaming/vision_bridge.py`, insert after
      `HGT_REF_NEEDS_REBOOT`'s docstring (line 165), before `DEFAULT_MAX_AGE_S`:

```python
MPC_XY_DEFAULTS = (
    ("MPC_XY_P", 0.95, MAV_PARAM_TYPE_REAL32),
    ("MPC_XY_VEL_P_ACC", 1.8, MAV_PARAM_TYPE_REAL32),
    ("MPC_XY_VEL_I_ACC", 0.4, MAV_PARAM_TYPE_REAL32),
    ("MPC_XY_VEL_D_ACC", 0.2, MAV_PARAM_TYPE_REAL32),
)
"""PX4's own stock position-controller gains (mc_pos_control_params.c:270,
282,295,307) -- none @reboot_required, unlike EKF2_HGT_REF, confirmed by their
absence from that file's docblocks. This is Task 10's `baseline` candidate
(docs/superpowers/specs/2026-08-14-mpc-xy-gain-sweep-design.md, D2) and also
what `revert_mpc_gains` restores to.
"""
```

      And after `apply_ekf2_gnss_restore_params` (line 398), before
      `reboot_for_boot_params`:

```python
def revert_mpc_gains(link):
    """D4: RESTORE GNSS reverts MPC_XY_* too, symmetric with EKF2_GPS_CTRL.

    Whatever gain set was active during a GPS-denied hold must not carry into
    a route or manual flight afterward. Called from `_go_gps_restore`
    alongside `apply_ekf2_gnss_restore_params` -- like that revert, this one
    can never make things worse, so it takes no gate and no precondition.
    """
    _apply(link, MPC_XY_DEFAULTS)
```

- [x] **10.1c Run tests, verify pass.**

```bash
conda run -n drone pytest streaming/tests/test_vision_bridge.py -q
```
Expect: PASS.

- [x] **10.1d Commit.**

```bash
git add streaming/vision_bridge.py streaming/tests/test_vision_bridge.py
git commit -m "feat(vio): add MPC_XY_DEFAULTS and revert_mpc_gains"
```

### Step 10.2 — `sim_s` telemetry, and the `set_param` command

The gain sweep is timed entirely in sim-seconds (70 s screen, 180 s confirm,
20 s settle) — this project has already hit the wall-vs-sim-clock bug three
times (`PX4_RESTART_GAP_S`, the VPE timestamp theory, the staleness budget;
SESSION.md, "the bug class to expect here"). `run.csv` has always had `sim_s`;
the pushed telemetry never has, so a websocket client has no sim clock to time
against. Fixing that here is what lets Step 10.4's driver avoid becoming bug
number four.

- [x] **10.2a Failing test.** Append to `streaming/tests/test_offboard_loop.py`
      (end of file, after `test_restore_without_vision_does_not_touch_ekf2`):

```python
# --- sim_s telemetry and set_param (Task 10) --------------------------------

def test_local_position_ned_updates_alt_and_sim_s_telemetry():
    """sim_s is PX4's own clock, which under lockstep IS sim time -- Task 10's
    gain-sweep driver waits on it rather than wall time (SESSION.md's
    wall-vs-sim-clock lesson, already hit three times in this project)."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 100}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)
    msgs = iter([_local_pos(time_boot_ms=42_000, z=-24.9)])
    loop.conn.recv_match = lambda **kw: next(msgs, None)

    loop._drain_mavlink()

    assert loop.telemetry()["alt_m"] == pytest.approx(24.9)
    assert loop.telemetry()["sim_s"] == pytest.approx(42.0)


def test_set_param_is_accepted_from_the_page():
    js = _load_server()
    _, loop, _ = _vision_loop(js, 101, _pose(), received_at=time.monotonic())
    loop.submit_set_param("MPC_XY_P", 0.5, offboard.MAV_PARAM_TYPE_REAL32)
    assert loop.commands.get_nowait() == (
        "set_param", "MPC_XY_P", 0.5, offboard.MAV_PARAM_TYPE_REAL32)


def test_run_command_applies_a_set_param_tuple():
    js = _load_server()
    _, loop, _ = _vision_loop(js, 102, _pose(), received_at=time.monotonic())
    sent = {}
    loop.link.set_param = (
        lambda name, value, ptype: sent.__setitem__(name, (value, ptype)))

    loop._run_command(("set_param", "MPC_XY_P", 0.5,
                       offboard.MAV_PARAM_TYPE_REAL32))

    assert sent["MPC_XY_P"] == (0.5, offboard.MAV_PARAM_TYPE_REAL32)
```

```bash
conda run -n drone pytest streaming/tests/test_offboard_loop.py -q
```
Expect: the first test fails on `sim_s` (KeyError or `!= pytest.approx(42.0)`,
since the key does not exist yet); the second fails with `AttributeError:
'SetpointLoop' object has no attribute 'submit_set_param'`; the third fails
because `_run_command` does not accept a tuple (`AttributeError` on
`getattr(self.link, name)` where `name` is a tuple, or similar).

- [x] **10.2b Implement `sim_s` telemetry.** In `joystick-server.py`, add to
      the `_telem` init dict (near `"streaming_s": 0.0,`):

```python
            # PX4's own clock (time_boot_ms / 1000), which under lockstep IS
            # sim time. None until the first LOCAL_POSITION_NED. A websocket
            # client (Task 10's gain-sweep driver) times its holds against
            # this, never against wall time -- see that file's _wait_for.
            "sim_s": None,
```

      And in `_drain_mavlink`'s `LOCAL_POSITION_NED` branch, right after
      `self._px4_sim_s = msg.time_boot_ms / 1000.0` (line 885):

```python
                self._px4_sim_s = msg.time_boot_ms / 1000.0
                with self._telem_lock:
                    self._telem["sim_s"] = self._px4_sim_s
```

- [x] **10.2c Implement `set_param` dispatch.** In `joystick-server.py`,
      add a new method next to `submit` (line 376):

```python
    def submit_set_param(self, name, value, param_type):
        """Called from the web thread. Queue only -- never touches `conn`.

        A separate method rather than routing through `submit(name)`: that
        one is a bare-string allowlist gate and every existing caller (the
        page, the mission verbs) relies on it staying that shape. set_param
        carries a payload, so it gets its own front door and its own tuple
        shape on the queue instead of overloading `submit`'s contract.
        """
        self.commands.put(("set_param", name, value, param_type))
```

      Change `_run_command` (line 383) to handle both shapes:

```python
    def _run_command(self, item):
        if isinstance(item, tuple):
            _, name, value, param_type = item
            self.link.set_param(name, value, param_type)
            print(f">>> set_param: {name}={value}")
            return
        name = item
        if name == "gps_denied":
            self._go_gps_denied()
            return
        if name == "gps_restore":
            self._restore_gnss()
            return
        method = self.MISSION_COMMANDS.get(name)
        if method is not None:
            getattr(self.mission, method)()
        else:
            getattr(self.link, name)()
        print(f">>> command: {name}")
```

      And route it from the websocket handler (`elif kind == "cmd":`,
      line 1220):

```python
                elif kind == "cmd":
                    if msg["name"] == "set_param":
                        loop_thread.submit_set_param(
                            msg["param"], msg["value"], msg["param_type"])
                    else:
                        loop_thread.submit(msg["name"])
```

- [x] **10.2d Run tests, verify pass.**

```bash
conda run -n drone pytest streaming/tests/test_offboard_loop.py -q
```
Expect: PASS.

- [x] **10.2e Websocket round-trip.** Extend
      `streaming/tests/test_web_ui.py`'s
      `test_websocket_accepts_control_messages_and_pushes_telemetry` — add
      after the existing `await ws.send(json.dumps({"type": "cmd", "name":
      "arm"}))` line:

```python
            await ws.send(json.dumps({
                "type": "cmd", "name": "set_param", "param": "MPC_XY_P",
                "value": 0.5, "param_type": 9}))   # MAV_PARAM_TYPE_REAL32
```

      No PX4 is attached in this test, so the assertion stays what it already
      is (`later["streaming_s"] > 0`) — this only confirms the message parses
      and does not crash the loop, which is what the original bug this file
      exists for (silent WebSocket-upgrade failure) would have hidden.

```bash
conda run -n drone pytest streaming/tests/test_web_ui.py -q
```
Expect: PASS.

- [x] **10.2f Commit.**

```bash
git add joystick-server.py streaming/tests/test_offboard_loop.py \
       streaming/tests/test_web_ui.py
git commit -m "feat(vio): add sim_s telemetry and the set_param command"
```

### Step 10.3 — D4: `RESTORE GNSS` reverts `MPC_XY_*` too

- [x] **10.3a Failing test.** Append to `streaming/tests/test_offboard_loop.py`,
      in the `# --- restoring GNSS ---` section, after
      `test_restoring_gnss_leaves_vision_fusing`:

```python
def test_restoring_gnss_reverts_mpc_gains_to_defaults():
    """D4: whatever gain set was active during a GPS-denied hold must not
    silently carry into a route or manual flight afterward -- symmetric with
    the EKF2_GPS_CTRL revert this same command already performs."""
    js = _load_server()
    loop = _denied_loop(js, 59)
    sent = {}
    loop.link.set_param = lambda name, value, ptype: sent.__setitem__(name, value)

    loop._run_command("gps_restore")

    for name, value, _ in vision_bridge.MPC_XY_DEFAULTS:
        assert sent[name] == value
```

```bash
conda run -n drone pytest streaming/tests/test_offboard_loop.py -q
```
Expect: FAIL — `KeyError: 'MPC_XY_P'` (not sent yet).

- [x] **10.3b Implement.** In `joystick-server.py`'s `_restore_gnss` (line
      601), add the call right after the existing EKF2 revert:

```python
        vision_bridge.apply_ekf2_gnss_restore_params(self.link)
        vision_bridge.revert_mpc_gains(self.link)
```

- [x] **10.3c Run tests, verify pass.**

```bash
conda run -n drone pytest streaming/tests/test_offboard_loop.py -q
```
Expect: PASS.

- [x] **10.3d Commit.**

```bash
git add joystick-server.py streaming/tests/test_offboard_loop.py
git commit -m "feat(vio): RESTORE GNSS reverts MPC_XY_* too (D4)"
```

### Step 10.4 — `sim/mpc_gain_sweep.py`, the campaign driver

Like Tasks 4, 5 and 7 (see "Plan validation"), the websocket-client parts of
this file are Isaac/PX4-live-only and are not TDD'd against a mock — they are
exercised live in Steps 10.6–10.7. `should_abort` (D5) is pure and gets a real
failing-test/implement pair; everything else is Implement-only, structured so
10.6/10.7 are "run this command" rather than "write this code live".

- [x] **10.4a Failing test.** Create `sim/tests/test_mpc_gain_sweep.py`:

```python
"""D5's screening-abort check. No PX4, no Isaac Sim, no socket."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import mpc_gain_sweep as sweep  # noqa: E402


def test_should_abort_is_false_with_no_data_yet():
    assert not sweep.should_abort(None, None, None)


def test_should_abort_on_excursion_past_the_worst_peak_seen():
    assert sweep.should_abort(401.0, alt_m=20.0, alt_at_cut_m=20.0)
    assert not sweep.should_abort(399.0, alt_m=20.0, alt_at_cut_m=20.0)


def test_should_abort_on_a_genuine_descent():
    assert sweep.should_abort(0.0, alt_m=-1.0, alt_at_cut_m=20.0)
    assert not sweep.should_abort(0.0, alt_m=5.0, alt_at_cut_m=20.0)


def test_should_abort_ignores_a_near_ground_z_reading_without_a_cut_baseline():
    """SESSION.md, run 6: a very negative alt_m near ground_z is not
    necessarily a strike -- without alt_at_cut_m there is nothing to compare
    it against, so it must not trip the abort."""
    assert not sweep.should_abort(0.0, alt_m=-24.89, alt_at_cut_m=None)
```

```bash
conda run -n drone pytest sim/tests/test_mpc_gain_sweep.py -q
```
Expect: `ModuleNotFoundError: No module named 'mpc_gain_sweep'`.

- [x] **10.4b Implement `should_abort` and the module scaffold.** Create
      `sim/mpc_gain_sweep.py`:

```python
#!/usr/bin/env python3
"""Drives a multi-candidate MPC_XY_* gain campaign over joystick-server.py's
existing /ws protocol -- arm/takeoff/gps_denied/gps_restore/land/disarm plus
one new command, set_param. One continuous session: Isaac Sim, PX4 and
joystick-server.py all stay up for the whole campaign; no PX4 reboot between
candidates, since none of the four gains are @reboot_required. See
docs/superpowers/specs/2026-08-14-mpc-xy-gain-sweep-design.md.

Run from the `drone` conda env against an ALREADY-RUNNING server (this script
does not start Isaac or joystick-server.py itself, matching how
sim/save-ulog.sh doesn't either):

    conda run -n drone python sim/mpc_gain_sweep.py \
        --candidates screen --campaign-name 20260815-screen
    conda run -n drone python sim/mpc_gain_sweep.py \
        --candidates confirm:more_damping --campaign-name 20260815-confirm
"""
import argparse
import asyncio
import json
import os
import sys
import time

import websockets

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
from offboard import MAV_PARAM_TYPE_REAL32  # noqa: E402

GAIN_PARAMS = ("MPC_XY_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC",
              "MPC_XY_VEL_D_ACC")

# D2's candidate set. All four gains are floats (mc_pos_control_params.c:270,
# 282,295,307), so every value here goes on the wire as MAV_PARAM_TYPE_REAL32
# -- never the INT32 bit-pattern path offboard.set_param also handles.
CANDIDATES = {
    "baseline":     (0.95, 1.8, 0.40, 0.2),
    "more_damping": (0.95, 1.8, 0.40, 0.5),
    "gentler_p":    (0.50, 1.2, 0.40, 0.2),
    "low_integral": (0.95, 1.8, 0.05, 0.2),
    "gentle_combo": (0.50, 1.2, 0.05, 0.5),
}
CANDIDATE_ORDER = ("baseline", "more_damping", "gentler_p", "low_integral",
                   "gentle_combo")
"""Flight order for a screening campaign. analyze_gain_sweep.py slices
run.csv's phase-2 segments in this same order -- the sidecar records it
explicitly rather than making the analyzer guess."""

SCREEN_HOLD_S = 70.0
CONFIRM_HOLD_S = 180.0
SETTLE_S = 20.0
OFFBOARD_TIMEOUT_S = 30.0

ABORT_EXCURSION_M = 400.0
"""D5: well past the worst peak seen so far, ~268 m (SESSION.md, run 6)."""
ABORT_ALT_DROP_M = 20.0
"""D5: a genuine descent, not the flat-ground_z-plane sign-convention trap
SESSION.md documents for readings near the ground."""


def should_abort(excursion_m, alt_m, alt_at_cut_m):
    """D5's screening abort check, pure so it needs no socket to test.

    `alt_m` is height above the launch point (telemetry's own convention --
    joystick-server.py computes it as `-msg.z`), not raw NED down, so a
    reading near ground_z is not mistaken for a strike (SESSION.md's "first
    read misread this as a crash"). Any missing value means the telemetry
    has not confirmed a real reading yet, so it never triggers an abort.
    """
    if excursion_m is not None and excursion_m > ABORT_EXCURSION_M:
        return True
    if (alt_m is not None and alt_at_cut_m is not None
            and (alt_at_cut_m - alt_m) > ABORT_ALT_DROP_M):
        return True
    return False
```

```bash
conda run -n drone pytest sim/tests/test_mpc_gain_sweep.py -q
```
Expect: PASS.

- [x] **10.4c Commit the tested piece.**

```bash
git add sim/mpc_gain_sweep.py sim/tests/test_mpc_gain_sweep.py
git commit -m "feat(vio): should_abort, the D5 gain-sweep screening check"
```

- [x] **10.4d Implement the driver.** Append to `sim/mpc_gain_sweep.py`:

```python
class Campaign:
    """Drives every candidate in `plan` (a list of (name, hold_s)) over one
    open websocket connection. `sidecar` records what actually flew, in
    order -- analyze_gain_sweep.py trusts that order rather than re-deriving
    it from timestamps."""

    def __init__(self, ws, campaign_name):
        self.ws = ws
        self.sidecar = {"campaign": campaign_name, "candidates": []}

    async def _recv_telem(self):
        return json.loads(await self.ws.recv())

    async def _cmd(self, name):
        await self.ws.send(json.dumps({"type": "cmd", "name": name}))

    async def _set_gain(self, param_name, value):
        await self.ws.send(json.dumps({
            "type": "cmd", "name": "set_param", "param": param_name,
            "value": value, "param_type": MAV_PARAM_TYPE_REAL32}))

    async def _wait_for(self, predicate, timeout_s, sim_time=True):
        """Poll telemetry (pushed at 5 Hz) until predicate(telem) is True or
        timeout_s elapses. sim_time=True (the default -- everything this
        driver waits on is a flight-time bar) measures elapsed time on PX4's
        own clock (telem["sim_s"]) rather than wall time: sim_rate wanders
        0.17-0.65 in this project (SESSION.md), and a wall-clock wait would
        hold for the wrong amount of simulated flight time. Returns the last
        telemetry frame seen either way -- callers check what actually
        happened rather than trusting the predicate held.
        """
        start_wall = time.monotonic()
        start_sim = None
        telem = None
        while True:
            telem = await self._recv_telem()
            if sim_time:
                if telem.get("sim_s") is None:
                    continue
                if start_sim is None:
                    start_sim = telem["sim_s"]
                elapsed = telem["sim_s"] - start_sim
            else:
                elapsed = time.monotonic() - start_wall
            if predicate(telem):
                return telem
            if elapsed >= timeout_s:
                return telem

    async def fly_candidate(self, name, hold_s):
        gains = CANDIDATES[name]
        record = {"name": name, "gains": dict(zip(GAIN_PARAMS, gains)),
                  "hold_s": hold_s, "status": "flying"}
        self.sidecar["candidates"].append(record)
        print(f">>> candidate {name}: {record['gains']}")

        for param_name, value in zip(GAIN_PARAMS, gains):
            await self._set_gain(param_name, value)

        await self._cmd("arm")
        await self._cmd("takeoff")
        telem = await self._wait_for(lambda t: t.get("mode") == "OFFBOARD",
                                     OFFBOARD_TIMEOUT_S)
        if telem.get("mode") != "OFFBOARD":
            record["status"] = "failed_to_offboard"
            return record

        await self._wait_for(lambda t: t.get("vision_fusing"),
                             OFFBOARD_TIMEOUT_S)
        await self._wait_for(lambda t: False, SETTLE_S)   # just settle

        await self._cmd("gps_denied")
        telem = await self._wait_for(lambda t: t.get("gps_denied"), 5.0,
                                     sim_time=False)
        if not telem.get("gps_denied"):
            await asyncio.sleep(5.0)
            await self._cmd("gps_denied")
            telem = await self._wait_for(lambda t: t.get("gps_denied"), 5.0,
                                         sim_time=False)
            if not telem.get("gps_denied"):
                record["status"] = "failed_to_cut"
                return record

        alt_at_cut = telem.get("alt_m")
        aborted = False

        def _hold_predicate(t):
            nonlocal aborted
            vio = t.get("vio") or {}
            if should_abort(vio.get("excursion_m"), t.get("alt_m"),
                            alt_at_cut):
                aborted = True
                return True
            return False

        await self._wait_for(_hold_predicate, hold_s)

        await self._cmd("gps_restore")
        await self._wait_for(lambda t: not t.get("gps_denied"), 10.0,
                             sim_time=False)
        await self._cmd("land")
        await self._wait_for(lambda t: t.get("mode") == "AUTO.LAND", 10.0,
                             sim_time=False)
        await self._cmd("disarm")

        record["status"] = "aborted" if aborted else "flown"
        print(f">>> candidate {name}: {record['status']}")
        return record

    def save_sidecar(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.sidecar, f, indent=2)


def _plan_for(candidates_arg):
    if candidates_arg == "screen":
        return [(name, SCREEN_HOLD_S) for name in CANDIDATE_ORDER]
    if candidates_arg.startswith("confirm:"):
        name = candidates_arg.split(":", 1)[1]
        if name not in CANDIDATES:
            raise SystemExit(f"unknown candidate {name!r}; choose from "
                             f"{sorted(CANDIDATES)}")
        return [(name, CONFIRM_HOLD_S)]
    raise SystemExit(f"--candidates must be 'screen' or 'confirm:<name>', "
                     f"got {candidates_arg!r}")


async def run_campaign(uri, candidates_arg, campaign_name):
    plan = _plan_for(candidates_arg)
    async with websockets.connect(uri) as ws:
        campaign = Campaign(ws, campaign_name)
        for name, hold_s in plan:
            await campaign.fly_candidate(name, hold_s)
        sidecar_path = os.path.join(
            ROOT, "logs", campaign_name, f"campaign_{campaign_name}.json")
        campaign.save_sidecar(sidecar_path)
        print(f">>> sidecar written to {sidecar_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uri", default="ws://127.0.0.1:8090/ws")
    ap.add_argument("--candidates", required=True,
                    help="'screen' (all 5 at 70 s) or 'confirm:<name>' "
                         "(one candidate at 180 s)")
    ap.add_argument("--campaign-name", required=True,
                    help="names the sidecar JSON; pass the SAME value as "
                         "joystick-server.py's --run-name at server-start "
                         "time so run.csv and the sidecar line up")
    args = ap.parse_args()
    asyncio.run(run_campaign(args.uri, args.candidates, args.campaign_name))


if __name__ == "__main__":
    main()
```

- [x] **10.4e Commit.**

```bash
git add sim/mpc_gain_sweep.py
git commit -m "feat(vio): mpc_gain_sweep.py campaign driver"
```

### Step 10.5 — `sim/analyze_gain_sweep.py`, the ranking script

Pure file-reading and arithmetic, no socket — full TDD, no live-only carve-out.

- [x] **10.5a Failing test.** Create `sim/tests/test_analyze_gain_sweep.py`:

```python
"""sim/analyze_gain_sweep.py's segment-slicing and trend-slope math, against
a synthetic run.csv + sidecar with known phase-2 segments and a known slope.
No PX4, no Isaac Sim."""
import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import analyze_gain_sweep as ags  # noqa: E402


def _row(sim_s, phase, excursion_m):
    return {"sim_s": str(sim_s), "phase": phase,
            "excursion_m": "" if excursion_m is None else str(excursion_m)}


def test_phase2_segments_splits_on_phase_boundaries():
    rows = ([_row(0, "1", None), _row(1, "1b", None)]
           + [_row(t, "2", 1.0) for t in range(2, 5)]
           + [_row(5, "1", None)]
           + [_row(t, "2", 2.0) for t in range(6, 9)])
    segments = ags.phase2_segments(rows)
    assert len(segments) == 2
    assert len(segments[0]) == 3
    assert len(segments[1]) == 3


def test_peak_excursion_ignores_blank_values():
    rows = [_row(0, "2", 1.0), _row(1, "2", None), _row(2, "2", 5.5),
            _row(3, "2", 3.0)]
    assert ags.peak_excursion(rows) == 5.5


def test_trend_slope_is_flat_for_a_settled_hold():
    rows = [_row(t, "2", 10.0) for t in range(0, 30)]
    slope = ags.trend_slope(rows, window_s=20.0)
    assert abs(slope) < ags.SLOPE_TOLERANCE
    assert ags.classify(slope) == "settling"


def test_trend_slope_is_positive_for_a_linear_growth():
    rows = [_row(t, "2", float(t)) for t in range(0, 30)]   # 1 m/s growth
    slope = ags.trend_slope(rows, window_s=20.0)
    assert slope == pytest.approx(1.0, abs=0.01)
    assert ags.classify(slope) == "growing"


def test_rank_candidates_orders_by_peak_excursion(tmp_path):
    csv_path = tmp_path / "run.csv"
    with open(csv_path, "w") as f:
        f.write("sim_s,phase,excursion_m\n")
        for t in range(0, 10):
            f.write(f"{t},2,20.0\n")             # candidate A: flat at 20 m
        for t in range(10, 20):
            f.write(f"{t},1,\n")                  # gap between candidates
        for t in range(20, 30):
            f.write(f"{t},2,5.0\n")              # candidate B: flat at 5 m

    sidecar_path = tmp_path / "campaign.json"
    sidecar = {"campaign": "test", "candidates": [
        {"name": "A", "gains": {}, "hold_s": 10, "status": "flown"},
        {"name": "B", "gains": {}, "hold_s": 10, "status": "flown"},
    ]}
    with open(sidecar_path, "w") as f:
        json.dump(sidecar, f)

    results = ags.rank_candidates(str(csv_path), str(sidecar_path))
    assert [r["name"] for r in results] == ["B", "A"]   # 5 m ranks before 20 m
```

```bash
conda run -n drone pytest sim/tests/test_analyze_gain_sweep.py -q
```
Expect: `ModuleNotFoundError: No module named 'analyze_gain_sweep'`.

- [x] **10.5b Implement.** Create `sim/analyze_gain_sweep.py`:

```python
#!/usr/bin/env python3
"""Reads a gain-sweep campaign's run.csv + campaign_<name>.json sidecar and
prints a ranked table: peak excursion_m and final-20s trend slope per
candidate, in flight order (D2, D3). See
docs/superpowers/specs/2026-08-14-mpc-xy-gain-sweep-design.md.
"""
import argparse
import csv
import json

SLOPE_TOLERANCE = 0.01
"""m/s. Below this, a fit slope is noise, not a real trend (D3)."""


def load_run_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def phase2_segments(rows):
    """Every contiguous run of phase=="2" rows, in flight order -- the k-th
    segment belongs to the k-th candidate flown (the campaign sidecar
    preserves that order)."""
    segments = []
    current = []
    for row in rows:
        if row["phase"] == "2":
            current.append(row)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def peak_excursion(segment):
    values = [float(r["excursion_m"]) for r in segment if r["excursion_m"]]
    return max(values) if values else None


def trend_slope(segment, window_s=20.0):
    """Least-squares slope of excursion_m over the final window_s sim
    seconds of the hold. Positive beyond SLOPE_TOLERANCE means growing (D3);
    this returns the number, classify() applies the label."""
    timed = [(float(r["sim_s"]), float(r["excursion_m"]))
            for r in segment if r["sim_s"] and r["excursion_m"]]
    if len(timed) < 2:
        return None
    end_t = timed[-1][0]
    window = [(t, x) for t, x in timed if t >= end_t - window_s]
    if len(window) < 2:
        window = timed
    n = len(window)
    mean_t = sum(t for t, _ in window) / n
    mean_x = sum(x for _, x in window) / n
    num = sum((t - mean_t) * (x - mean_x) for t, x in window)
    den = sum((t - mean_t) ** 2 for t, _ in window)
    return num / den if den else 0.0


def classify(slope):
    if slope is None:
        return "unknown"
    return "growing" if slope > SLOPE_TOLERANCE else "settling"


def rank_candidates(run_csv_path, sidecar_path):
    rows = load_run_csv(run_csv_path)
    with open(sidecar_path) as f:
        sidecar = json.load(f)
    segments = phase2_segments(rows)
    flown = [c for c in sidecar["candidates"] if c["status"] == "flown"]
    results = []
    for candidate, segment in zip(flown, segments):
        slope = trend_slope(segment)
        results.append({
            "name": candidate["name"],
            "gains": candidate["gains"],
            "peak_excursion_m": peak_excursion(segment),
            "trend_slope_mps": slope,
            "trend": classify(slope),
        })
    results.sort(key=lambda r: (r["peak_excursion_m"] is None,
                                r["peak_excursion_m"] or 0.0))
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_csv")
    ap.add_argument("sidecar_json")
    args = ap.parse_args()
    results = rank_candidates(args.run_csv, args.sidecar_json)
    print(f"{'candidate':<15} {'peak_m':>8} {'slope_m/s':>10} {'trend':>10}")
    for r in results:
        print(f"{r['name']:<15} {r['peak_excursion_m']:>8.1f} "
              f"{r['trend_slope_mps']:>10.3f} {r['trend']:>10}")


if __name__ == "__main__":
    main()
```

- [x] **10.5c Run tests, verify pass.**

```bash
conda run -n drone pytest sim/tests/test_analyze_gain_sweep.py -q
```
Expect: PASS.

- [x] **10.5d Full offline suite, verify nothing regressed.**

```bash
conda run -n drone pytest streaming/tests/ sim/tests/ -q
```
Expect: PASS (same 247/249 baseline as Task 9 — the 2 known environmental
failures only, per SESSION.md).

- [x] **10.5e Commit.**

```bash
git add sim/analyze_gain_sweep.py sim/tests/test_analyze_gain_sweep.py
git commit -m "feat(vio): analyze_gain_sweep.py ranking script"
```

### Step 10.6 — The screening campaign (manual, live)

- [x] **10.6a** Bring the stack up:

```bash
DRONE_SETUP_DOWN_VIB_DAMP=False VIO=1 ./sim/launch-sitl.sh
python joystick-server.py --takeoff-alt 50 --run-name 20260815-screen
```

- [x] **10.6b** Run the screen:

```bash
conda run -n drone python sim/mpc_gain_sweep.py \
    --candidates screen --campaign-name 20260815-screen
```

      Watch for `failed_to_offboard` / `failed_to_cut` / `aborted` in the
      console — any of those on `baseline` specifically means something is
      wrong with the harness, not the candidate, since `baseline` is what
      every prior run already flew successfully.

- [x] **10.6c** `sim/save-ulog.sh 20260815-screen` before shutting Isaac down
      (ADR-0003 — the ulog dies with the process otherwise).

- [x] **10.6d** Rank the results:

```bash
conda run -n drone python sim/analyze_gain_sweep.py \
    logs/20260815-screen/run.csv \
    logs/20260815-screen/campaign_20260815-screen.json
```

- [x] **10.6e** Record the ranked table in `SESSION.md`, in the same
      run-by-run style as Task 9.

### Step 10.7 — The confirmation flight (manual, live)

- [x] **10.7a** Whichever candidate ranked best on both peak excursion and
      trend (D3) gets one full 180 s flight, unless `baseline` won — a
      baseline win means the sweep found nothing, which is Task 10's own
      valid negative result (design doc, "Goal"), and 8.6 is not retried.

```bash
python joystick-server.py --takeoff-alt 50 --run-name 20260815-confirm
conda run -n drone python sim/mpc_gain_sweep.py \
    --candidates confirm:<winning-candidate> --campaign-name 20260815-confirm
sim/save-ulog.sh 20260815-confirm
```

- [x] **10.7b** Judge the confirmation flight against ADR-0001's actual bar —
      non-growing excursion envelope over three or more oscillation periods,
      not the 70 s screening bar. If it passes, **8.6 passes**; check it off
      in Task 8 and record the winning gain set in `SESSION.md`.

- [x] **10.7c** Record the outcome — pass or fail — in `SESSION.md` and in
      Task 8's status blockquote either way. A failing confirmation is a
      real result too: it means the position-controller gains were not the
      lever either, and per the design doc's "Why this exists", suspicion
      moves back upstream to the vision estimator's own closed-loop
      behavior, out of this plan's scope.

---

## Plan validation

Covers every design decision: D1 (Task 6), D2 (Tasks 4–5), D3 (Steps 4.2–4.3),
D4 (Steps 5.2, 7), D5 (Task 3), D6 (Task 3 + Step 6.2), D7 (scope throughout).

Task 10 implements the separate 2026-08-14 gain-sweep design's own D1–D5
(distinct numbering, cross-referenced at Task 10's header): D1 (Step 10.2,
`set_param` as a websocket command rather than a second MAVLink client), D2
(Step 10.4's `CANDIDATES`), D3 (Step 10.5's `SLOPE_TOLERANCE` / `classify`),
D4 (Step 10.3), D5 (Step 10.4's `should_abort`).

Names are consistent across tasks: `MahonyState.from_heading`,
`zmq_proto.pack/unpack`, `VisionPose`, `VisionPositionSender.send`,
`apply_ekf2_vision_params`, `send_gps_global_origin`, `enu_to_ned`,
`MPC_XY_DEFAULTS`, `revert_mpc_gains`, `should_abort`, `phase2_segments`.

**Known gaps, deliberate:** Tasks 4, 5 and 7 give structure and interfaces rather
than complete code — Task 4 depends on Step 4.1's Kit-interpreter check and Step
4.3's live safety result, Task 5's tuning depends on Step 5.2's measured
comparison against batch, and Task 7 depends on the telemetry shape Task 6 lands.
Tasks 1–3, which are pure and testable offline, carry complete code. Task 10
follows the same split: `should_abort` (Step 10.4a-c) and
`analyze_gain_sweep.py` (Step 10.5, fully pure) are TDD'd; `Campaign` and its
websocket-driven flight sequencing (Step 10.4d) are Isaac/PX4-live-only,
structure without a mock, exercised in Steps 10.6–10.7 exactly as Task 9's
9.4 exercised Task 9's instrumentation.

**Riskiest step:** 4.3. If `rep.orchestrator.step()` is unsafe under lockstep on
this build, VIO frames carry ~1 render period of pose skew and accuracy drops.
The fallback is specified and the decision is recorded rather than rediscovered.
