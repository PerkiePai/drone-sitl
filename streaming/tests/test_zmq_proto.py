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


def test_imu_round_trip_with_magnetometer():
    """The `m` field: body-FRD magnetic field, added under Task 9 (ADR-0005) as
    a heading reference. Recorded at zero fusion gain -- see
    pipeline-streaming.py --mag-gain -- but the wire format has to carry it."""
    payload = {"ts_ns": 1, "w": [0.0, 0.0, 0.0], "a": [0.0, 0.0, -9.81],
               "m": [0.21, 0.05, 0.41]}
    _, got = zmq_proto.unpack(zmq_proto.pack(zmq_proto.TOPIC_IMU, payload))
    assert got == payload


def test_imu_round_trip_without_magnetometer():
    """A vehicle with no magnetometer sensor omits `m` entirely -- a silently
    absent field is the failure mode this project keeps paying for, so the
    absence has to be a missing key, not None, and this pins that shape."""
    payload = {"ts_ns": 1, "w": [0.0, 0.0, 0.0], "a": [0.0, 0.0, -9.81]}
    _, got = zmq_proto.unpack(zmq_proto.pack(zmq_proto.TOPIC_IMU, payload))
    assert "m" not in got


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
