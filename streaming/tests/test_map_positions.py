"""The three positions the map draws, and the projection that gets them there.

Ground truth and the vision estimate are metres in ENU; the map is lat/lon.
Everything here is about that conversion staying honest -- in particular that
the origin is latched once (a sliding origin would drag the traces along with
whatever it was measuring) and that the frames are projected RAW, so a constant
offset between GT's spawn anchor and PX4's EKF origin is visible on screen
rather than silently absorbed.
"""
import importlib.util
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "streaming"))

import offboard  # noqa: E402

FAKE_PX4_PORT = 14596       # not 14540, and not any other test's port


def _load_server():
    spec = importlib.util.spec_from_file_location(
        "joystick_server_map", os.path.join(ROOT, "joystick-server.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _load_pipeline():
    spec = importlib.util.spec_from_file_location(
        "pipeline_streaming_map", os.path.join(ROOT, "pipeline-streaming.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _FakeVision:
    """Stands in for the ZMQ vision source. Only `_last_msg` is ever read by
    the code under test, and only `vision is not None` gates the sender."""

    def __init__(self, last=None):
        self._last_msg = last or {}
        self.received = 0

    def latest(self):
        """(pose, received_at_wall) -- the shape _vision_clock unpacks. No pose
        has ever arrived as far as the clock is concerned, which leaves the
        estimate stale; none of these tests are about freshness."""
        return None, None


class _Fix:
    """A GLOBAL_POSITION_INT, in the units PX4 actually sends them in."""

    def __init__(self, lat, lon, hdg_cdeg=9000):
        self.lat = int(round(lat * 1e7))
        self.lon = int(round(lon * 1e7))
        self.hdg = hdg_cdeg


def _loop(js, vision=None):
    from pymavlink import mavutil
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT}")
    return js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), vision=vision)


# --- the projection origin ------------------------------------------------

def test_no_origin_until_both_a_fix_and_a_local_position_have_arrived():
    """Either one alone is not enough to know where local (0,0) is on Earth."""
    js = _load_server()
    loop = _loop(js)

    loop._handle_global_position(_Fix(13.7, 100.5))
    assert loop._map_origin is None, "a fix without LOCAL_POSITION_NED cannot site the origin"

    loop._px4_ned = (0.0, 0.0, -50.0)
    loop._handle_global_position(_Fix(13.7, 100.5))
    assert loop._map_origin is not None


def test_the_origin_is_the_local_frame_zero_not_the_current_position():
    """PX4 reports where it IS; the traces are drawn from where local (0,0) is.
    100 m north of the origin must put the origin 100 m SOUTH of the fix."""
    js = _load_server()
    loop = _loop(js)
    loop._px4_ned = (100.0, 0.0, -50.0)       # 100 m north of the local origin
    loop._handle_global_position(_Fix(13.7, 100.5))

    lat0, lon0 = loop._map_origin
    south_m = (13.7 - lat0) * js.M_PER_DEG_LAT
    assert south_m == pytest.approx(100.0, abs=0.5)
    assert lon0 == pytest.approx(100.5, abs=1e-6)


def test_the_origin_is_latched_once_and_never_moves():
    """A recomputed origin would slide under the traces, so a drift away from
    the hold point would draw as a stationary aircraft on a moving world."""
    js = _load_server()
    loop = _loop(js)
    loop._px4_ned = (0.0, 0.0, -50.0)
    loop._handle_global_position(_Fix(13.7, 100.5))
    first = loop._map_origin

    loop._px4_ned = (250.0, -80.0, -50.0)     # flown a long way since
    loop._handle_global_position(_Fix(13.702, 100.499))
    assert loop._map_origin == first


# --- the projection itself ------------------------------------------------

def test_enu_metres_project_back_to_the_metres_they_came_from():
    """Round-trip at this site's latitude: a point 100 m east and 60 m north of
    the origin must come back as 100 m east and 60 m north."""
    js = _load_server()
    loop = _loop(js)
    loop._px4_ned = (0.0, 0.0, -50.0)
    loop._handle_global_position(_Fix(13.7, 100.5))
    lat0, lon0 = loop._map_origin

    lat, lon = loop._enu_to_latlon(100.0, 60.0)
    north_m = (lat - lat0) * js.M_PER_DEG_LAT
    east_m = (lon - lon0) * js.M_PER_DEG_LAT * math.cos(math.radians(lat0))
    assert north_m == pytest.approx(60.0, abs=0.01)
    assert east_m == pytest.approx(100.0, abs=0.01)


def test_projection_returns_none_before_the_origin_is_latched():
    """None, never (0,0) -- an unsited point at null island is a lie the page
    would draw."""
    js = _load_server()
    assert _loop(js)._enu_to_latlon(10.0, 10.0) is None


# --- what reaches the page ------------------------------------------------

def test_ground_truth_and_vio_reach_the_page_as_latlon():
    js = _load_server()
    loop = _loop(js, vision=_FakeVision({
        "gt_x": 40.0, "gt_y": 30.0, "gt_yaw": 0.0,       # ENU: x east, y north
        "x": 44.0, "y": 33.0}))
    loop._px4_ned = (0.0, 0.0, -50.0)
    loop._handle_global_position(_Fix(13.7, 100.5))
    lat0, lon0 = loop._map_origin

    vio = loop._vio_status(0.0)
    gt_n = (vio["gt_lat"] - lat0) * js.M_PER_DEG_LAT
    v_n = (vio["vio_lat"] - lat0) * js.M_PER_DEG_LAT
    assert gt_n == pytest.approx(30.0, abs=0.01)
    assert v_n == pytest.approx(33.0, abs=0.01)
    assert vio["gt_lat"] != vio["vio_lat"], (
        "GT and VIO must be projected separately -- their difference IS drift")


def test_the_frames_are_drawn_raw_with_no_alignment():
    """GT's anchor is the spawn point and PX4's is the EKF origin. They are not
    the same point, and this display exists to SHOW that, not hide it."""
    js = _load_server()
    loop = _loop(js, vision=_FakeVision({"gt_x": 0.0, "gt_y": 0.0, "gt_yaw": 0.0}))
    loop._px4_ned = (7.0, 5.0, -50.0)          # PX4 is 7 m north of local zero
    loop._handle_global_position(_Fix(13.7, 100.5))

    vio = loop._vio_status(0.0)
    lat0, _ = loop._map_origin
    gt_n = (vio["gt_lat"] - lat0) * js.M_PER_DEG_LAT
    assert gt_n == pytest.approx(0.0, abs=0.01), (
        "GT at its own zero must draw at local zero, offset and all")


def test_map_positions_are_none_when_there_is_no_ground_truth():
    """Same convention as drift_m: None, never 0.0. No GT topic is "cannot
    tell", and a marker at the origin would read as "sitting on the pad"."""
    js = _load_server()
    loop = _loop(js, vision=_FakeVision({"x": 10.0, "y": 10.0}))
    loop._px4_ned = (0.0, 0.0, -50.0)
    loop._handle_global_position(_Fix(13.7, 100.5))

    vio = loop._vio_status(0.0)
    assert vio["gt_lat"] is None and vio["gt_lon"] is None
    assert vio["gt_heading_deg"] is None
    assert vio["vio_lat"] is not None, "the vision estimate does not need GT"


def test_map_positions_are_none_before_a_px4_fix():
    """The estimator can be running long before PX4 has a global position."""
    js = _load_server()
    loop = _loop(js, vision=_FakeVision({"gt_x": 1.0, "gt_y": 2.0, "x": 1.0, "y": 2.0}))
    vio = loop._vio_status(0.0)
    assert vio["gt_lat"] is None and vio["vio_lat"] is None


def test_the_gt_arrow_takes_a_compass_bearing_not_an_enu_yaw():
    """The page rotates the arrow by CSS degrees clockwise from north, and the
    estimator reports yaw counter-clockwise from east. Getting this wrong points
    a green arrow the wrong way with no other symptom."""
    js = _load_server()
    loop = _loop(js, vision=_FakeVision(
        {"gt_x": 0.0, "gt_y": 0.0, "gt_yaw": math.pi / 2}))   # ENU yaw 90 = north
    loop._px4_ned = (0.0, 0.0, -50.0)
    loop._handle_global_position(_Fix(13.7, 100.5))

    assert loop._vio_status(0.0)["gt_heading_deg"] == pytest.approx(0.0, abs=1e-6)

    loop.vision._last_msg["gt_yaw"] = 0.0                     # ENU yaw 0 = east
    assert loop._vio_status(0.0)["gt_heading_deg"] == pytest.approx(90.0, abs=1e-6)


# --- the estimator end ----------------------------------------------------

def test_the_estimator_forwards_the_ground_truth_heading():
    """The GT topic has carried the attitude quaternion all along; only the
    position was being kept. The arrow needs the heading too."""
    ps = _load_pipeline()
    est = ps.Estimator({"K": [[300.0, 0.0, 480.0], [0.0, 300.0, 300.0],
                              [0.0, 0.0, 1.0]],
                        "R_CtoI": np.eye(3).tolist(), "heading_deg": 90.0})
    # Quaternion xyzw, FLU in ENU: 90 deg about up, i.e. nose north.
    s = math.sin(math.pi / 4)
    est.on_gt({"p": [1.0, 2.0, 3.0], "q": [0.0, 0.0, s, s]})

    out = est.payload({"ts_ns": 0, "frame": 0})
    assert out["gt_yaw"] == pytest.approx(math.pi / 2, abs=1e-6)


def test_the_estimator_reports_no_heading_without_ground_truth():
    ps = _load_pipeline()
    est = ps.Estimator({"K": [[300.0, 0.0, 480.0], [0.0, 300.0, 300.0],
                              [0.0, 0.0, 1.0]],
                        "R_CtoI": np.eye(3).tolist(), "heading_deg": 90.0})
    assert est.payload({"ts_ns": 0, "frame": 0})["gt_yaw"] is None
