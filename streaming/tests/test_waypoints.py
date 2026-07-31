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

# ~111 m south of waypoint 1, well outside any arrival radius. Tests that mean
# "flying toward waypoint 1" must start here: advancing from ON a waypoint
# legitimately consumes it, which is the behaviour
# test_pressing_fly_while_sitting_on_waypoint_one_skips_it pins down.
START = (39.9990, -74.0000)


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
    m.advance(*ROUTE[0])              # arrive at wp 1 -> index steps to 1
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


def test_advance_returns_nothing_when_idle():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    assert m.advance(40.0, -74.0) is None


def test_advance_returns_nothing_when_paused():
    """The whole point of pause: the loop falls through to the manual path."""
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    m.pause()
    assert m.advance(40.0, -74.0) is None


def test_advance_returns_nothing_without_a_position_fix():
    """Steering toward a waypoint from an unknown position is worse than
    hovering. The mission stays RUNNING and picks up when position returns."""
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    assert m.advance(None, None) is None
    assert m.status()["state"] == waypoints.Mission.RUNNING


def test_advance_returns_nothing_when_no_route_is_loaded():
    m = waypoints.Mission()
    assert m.advance(40.0, -74.0) is None


def test_running_targets_the_first_waypoint_with_its_altitude():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    lat, lon, alt, yaw = m.advance(*START)
    assert (lat, lon) == ROUTE[0]
    assert alt == 12.0


def test_the_nose_points_along_the_leg():
    """wp 1 is due north of the start, so yaw should be ~0 deg."""
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    _, _, _, yaw = m.advance(40.0, -74.0)
    assert abs(yaw) < 0.5


def test_arriving_within_the_radius_steps_to_the_next_waypoint():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    lat, lon, _, _ = m.advance(*START)              # still en route to wp 1
    assert (lat, lon) == ROUTE[0]
    lat, lon, _, _ = m.advance(*ROUTE[0])           # arrived at wp 1
    assert (lat, lon) == ROUTE[1]


def test_a_waypoint_just_outside_the_radius_is_not_reached():
    """2.0 m radius; 0.00005 deg of latitude is ~5.6 m."""
    m = waypoints.Mission(arrival_radius_m=2.0)
    m.load(ROUTE, 12.0)
    m.fly()
    lat, lon, _, _ = m.advance(40.00005, -74.0000)
    assert (lat, lon) == ROUTE[0]
    assert m.status()["index"] == 0


def test_several_waypoints_inside_the_radius_are_consumed_at_once():
    """Two clicks a metre apart must not take two ticks to clear."""
    m = waypoints.Mission(arrival_radius_m=2.0)
    m.load([(40.0, -74.0), (40.000001, -74.0), (40.0010, -74.0)], 12.0)
    m.fly()
    lat, lon, _, _ = m.advance(40.0, -74.0)
    assert (lat, lon) == (40.0010, -74.0)


def test_passing_the_last_waypoint_finishes_the_mission():
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    m.advance(40.0, -74.0)
    m.advance(40.0010, -74.0)
    assert m.status()["state"] == waypoints.Mission.DONE


def test_done_keeps_holding_the_last_waypoint():
    """The regression this exists to catch: dispatching on state == RUNNING
    would drop a finished mission onto a zero-velocity hover, which drifts in
    wind. DONE must still yield a position target."""
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    m.advance(40.0, -74.0)
    m.advance(40.0010, -74.0)
    target = m.advance(40.0010, -74.0)
    assert target is not None
    lat, lon, alt, _ = target
    assert (lat, lon) == (40.0010, -74.0)
    assert alt == 12.0


def test_the_held_yaw_does_not_wander_once_holding():
    """Bearing to a point you are sitting on is numerically meaningless and
    would yaw the aircraft randomly on the spot. The final leg's bearing is
    stored on arrival, not recomputed."""
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    m.advance(40.0, -74.0)            # flying north, yaw ~0
    m.advance(40.0010, -74.0)         # arrive -> DONE
    yaws = [m.advance(40.0010 + 1e-7 * i, -74.0)[3] for i in range(5)]
    assert len(set(yaws)) == 1
    assert abs(yaws[0]) < 0.5         # still the northward course


def test_distance_to_the_active_waypoint_is_reported():
    m = waypoints.Mission()
    m.load([(40.0010, -74.0)], 12.0)
    m.fly()
    m.advance(40.0, -74.0)
    assert abs(m.status()["dist_m"] - 111.19) < 0.1


def test_flying_again_after_done_restarts_from_waypoint_one():
    m = waypoints.Mission()
    m.load(ROUTE, 12.0)
    m.fly()
    for lat, lon in ROUTE:
        m.advance(lat, lon)
    m.advance(*ROUTE[-1])
    assert m.status()["state"] == waypoints.Mission.DONE
    m.fly()
    assert m.status()["index"] == 0
    lat, lon, _, _ = m.advance(*START)
    assert (lat, lon) == ROUTE[0]


def test_pressing_fly_while_sitting_on_waypoint_one_skips_it():
    """Already there means already reached. Caught by four tests that flew
    'from' waypoint 1 and expected to be sent back to it -- the sequencer is
    right and those expectations were wrong. Pinned down here so the next
    reader does not re-derive it.
    """
    m = waypoints.Mission(arrival_radius_m=2.0)
    m.load(ROUTE, 12.0)
    m.fly()
    lat, lon, _, _ = m.advance(*ROUTE[0])
    assert (lat, lon) == ROUTE[1]
    assert m.status()["index"] == 1
