"""competition.state -- State/Arena value objects built from the server's
telemetry dict. No PX4, no server."""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition.state import Arena, State  # noqa: E402

TELEM = {
    "lat": 13.6, "lon": 100.3, "alt_m": 42.0,
    "vn": 1.5, "ve": -0.5, "vz": 0.2, "gs": 1.58,
    "heading_deg": 270.0, "roll_deg": 3.0, "pitch_deg": -12.0,
    "mission": {"state": "RUNNING", "index": 2, "count": 4, "dist_m": 37.0},
}


def test_state_maps_every_field():
    s = State.from_telemetry(TELEM, camera="nadir", time_elapsed=10.0,
                             time_limit=None)
    assert s.lat == 13.6 and s.lon == 100.3
    assert s.alt_agl == 42.0
    assert (s.vx, s.vy, s.vz) == (1.5, -0.5, 0.2)
    assert s.ground_speed == 1.58
    assert s.heading == 270.0
    assert (s.roll, s.pitch) == (3.0, -12.0)
    assert s.camera == "nadir"
    assert s.time_elapsed == 10.0
    assert s.time_remaining is None
    assert s.route.state == "RUNNING" and s.route.index == 2
    assert s.route.count == 4 and s.route.distance_m == 37.0


def test_time_remaining_is_clamped_and_computed_when_limited():
    s = State.from_telemetry(TELEM, camera="nadir", time_elapsed=250.0,
                             time_limit=300.0)
    assert s.time_remaining == 50.0
    s2 = State.from_telemetry(TELEM, camera="nadir", time_elapsed=999.0,
                              time_limit=300.0)
    assert s2.time_remaining == 0.0


def test_missing_keys_default_rather_than_raise():
    s = State.from_telemetry({}, camera="oblique", time_elapsed=0.0,
                             time_limit=None)
    assert s.lat is None and s.lon is None
    assert s.alt_agl == 0.0 and s.vz == 0.0
    assert s.route.state == "IDLE" and s.route.count == 0
    assert s.route.distance_m is None


def test_arena_around_is_a_box_centred_on_the_point():
    a = Arena.around(13.0, 100.0, radius_m=500.0, time_limit=None)
    lat_min, lon_min, lat_max, lon_max = a.bounds
    assert lat_min < 13.0 < lat_max
    assert lon_min < 100.0 < lon_max
    # ~500 m north/south is ~0.00449 deg latitude
    assert math.isclose((lat_max - lat_min) / 2, 500.0 / 111_320.0, rel_tol=1e-3)
    assert a.time_limit is None


def test_arena_longitude_span_widens_toward_the_equator():
    near_eq = Arena.around(1.0, 0.0, 500.0, None).bounds
    high_lat = Arena.around(60.0, 0.0, 500.0, None).bounds
    assert (near_eq[3] - near_eq[1]) < (high_lat[3] - high_lat[1])
