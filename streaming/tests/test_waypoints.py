"""Unit tests for waypoint sequencing. No PX4, no Isaac, no threads."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import waypoints  # noqa: E402


def test_one_degree_of_latitude_is_about_111_km():
    d = waypoints.haversine_m(0.0, 0.0, 1.0, 0.0)
    assert abs(d - 111194.9) < 1.0


def test_one_degree_of_longitude_at_the_equator_matches_latitude():
    assert abs(waypoints.haversine_m(0.0, 0.0, 0.0, 1.0)
               - waypoints.haversine_m(0.0, 0.0, 1.0, 0.0)) < 1.0


def test_a_short_local_leg_is_metres_not_degrees():
    """The scale that actually matters: a 0.001 deg step is ~111 m."""
    d = waypoints.haversine_m(40.0, -74.0, 40.001, -74.0)
    assert abs(d - 111.19) < 0.1


def test_distance_to_the_same_point_is_zero():
    assert waypoints.haversine_m(40.0, -74.0, 40.0, -74.0) == 0.0


def test_distance_is_symmetric():
    a = waypoints.haversine_m(40.0, -74.0, 40.01, -74.01)
    b = waypoints.haversine_m(40.01, -74.01, 40.0, -74.0)
    assert abs(a - b) < 1e-9


def test_bearing_of_the_four_cardinal_directions():
    assert waypoints.bearing_deg(0.0, 0.0, 1.0, 0.0) == 0.0      # north
    assert waypoints.bearing_deg(0.0, 0.0, 0.0, 1.0) == 90.0     # east
    assert waypoints.bearing_deg(0.0, 0.0, -1.0, 0.0) == 180.0   # south
    assert waypoints.bearing_deg(0.0, 0.0, 0.0, -1.0) == 270.0   # west


def test_bearing_northeast_is_about_45_but_not_exactly():
    """Great-circle, not flat: a NE leg starts at 44.996 deg, not 45."""
    assert abs(waypoints.bearing_deg(0.0, 0.0, 1.0, 1.0) - 45.0) < 0.01


def test_bearing_is_always_a_positive_compass_angle():
    for lat, lon in [(1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]:
        b = waypoints.bearing_deg(0.0, 0.0, lat, lon)
        assert 0.0 <= b < 360.0


ROUTE = [(40.0000, -74.0000), (40.0010, -74.0000), (40.0010, -74.0010)]


def test_a_fresh_mission_is_idle_and_empty():
    m = waypoints.Mission()
    s = m.status()
    assert s["state"] == waypoints.Mission.IDLE
    assert s["count"] == 0
    assert s["dist_m"] is None


def test_load_stores_the_route_but_does_not_start_it():
    """Planning must never move the aircraft -- FLY is a separate press."""
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    s = m.status()
    assert s["state"] == waypoints.Mission.IDLE
    assert s["count"] == 3
    assert s["index"] == 0


def test_fly_starts_a_loaded_route():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    assert m.status()["state"] == waypoints.Mission.RUNNING


def test_fly_with_no_waypoints_is_a_no_op():
    m = waypoints.Mission()
    m.fly()
    assert m.status()["state"] == waypoints.Mission.IDLE


def test_pause_only_applies_to_a_running_mission():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.pause()
    assert m.status()["state"] == waypoints.Mission.IDLE   # not PAUSED
    m.fly()
    m.pause()
    assert m.status()["state"] == waypoints.Mission.PAUSED


def test_pause_is_idempotent():
    """Every axis press submits a pause; holding a direction sends one, but
    tapping four buttons sends four. They must not stack into anything."""
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.pause(); m.pause(); m.pause()
    assert m.status()["state"] == waypoints.Mission.PAUSED


def test_fly_after_pause_resumes_without_losing_progress():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.advance(40.0010, -74.0000)      # arrive at wp 1 -> index steps to 1
    m.pause()
    m.fly()
    assert m.status()["state"] == waypoints.Mission.RUNNING
    assert m.status()["index"] == 1   # resumed, not restarted


def test_clear_drops_the_route_and_returns_to_idle():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.clear()
    s = m.status()
    assert s["state"] == waypoints.Mission.IDLE
    assert s["count"] == 0


def test_load_replaces_a_previous_route_and_resets_progress():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.advance(40.0010, -74.0000)
    m.load([(41.0, -75.0)], 20.0)
    s = m.status()
    assert s["count"] == 1
    assert s["index"] == 0
    assert s["state"] == waypoints.Mission.IDLE
