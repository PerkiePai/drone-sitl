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
