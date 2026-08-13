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
> **Status 2026-08-13, not yet flown:** the handover now pins the vision frame
> onto PX4's at both phase transitions (`vision_bridge.FrameAlignment`), so
> GPS-denied flight starts from zero error instead of ~30 m. Writing it turned
> up a second bug: `VISION_POSITION_ESTIMATE` was carrying the estimator's
> ENU/FLU attitude into NED/FRD fields, so the fused vision yaw was wrong by
> `90 - 2*heading` degrees — up to 180. Both fixed, both unflown. 8.6 is the
> next flight.

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
- [ ] **8.6** Arm, take off, hover 60 s vision-only. Pass bar is **holding
      station**, not zero drift.
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

## Plan validation

Covers every design decision: D1 (Task 6), D2 (Tasks 4–5), D3 (Steps 4.2–4.3),
D4 (Steps 5.2, 7), D5 (Task 3), D6 (Task 3 + Step 6.2), D7 (scope throughout).

Names are consistent across tasks: `MahonyState.from_heading`,
`zmq_proto.pack/unpack`, `VisionPose`, `VisionPositionSender.send`,
`apply_ekf2_vision_params`, `send_gps_global_origin`, `enu_to_ned`.

**Known gaps, deliberate:** Tasks 4, 5 and 7 give structure and interfaces rather
than complete code — Task 4 depends on Step 4.1's Kit-interpreter check and Step
4.3's live safety result, Task 5's tuning depends on Step 5.2's measured
comparison against batch, and Task 7 depends on the telemetry shape Task 6 lands.
Tasks 1–3, which are pure and testable offline, carry complete code.

**Riskiest step:** 4.3. If `rep.orchestrator.step()` is unsafe under lockstep on
this build, VIO frames carry ~1 render period of pose skew and accuracy drops.
The fallback is specified and the decision is recorded rather than rediscovered.
