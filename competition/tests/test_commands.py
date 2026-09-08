"""competition.commands -- construction and validation.

A malformed command is a bug in the uploaded script; it must raise where the
script writes it, not fly. No PX4, no server.
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition import Agent, Command, Route, flight  # noqa: E402


def test_flight_defaults_to_all_zero_hover():
    f = flight(0, 0, 0, 0)
    assert (f.left_x, f.left_y, f.right_x, f.right_y) == (0.0, 0.0, 0.0, 0.0)


def test_flight_is_frozen():
    f = flight(0, 0, 0, 0)
    with pytest.raises(Exception):
        f.left_x = 1.0


def test_flight_rejects_non_numeric():
    with pytest.raises(TypeError):
        flight("fast", 0, 0, 0)


def test_flight_rejects_out_of_range():
    flight(1.0, -1.0, 1.0, -1.0)          # full deflection either way is fine
    with pytest.raises(ValueError):
        flight(1.1, 0, 0, 0)
    with pytest.raises(ValueError):
        flight(0, 0, 0, -1.1)


def test_route_needs_at_least_one_waypoint():
    with pytest.raises(ValueError):
        Route(waypoints=[], alt=30.0)


def test_route_normalizes_waypoints_to_tuple_of_float_pairs():
    r = Route(waypoints=[[1, 2], (3, 4)], alt=30.0)
    assert r.waypoints == ((1.0, 2.0), (3.0, 4.0))


def test_command_accepts_a_flight_command():
    c = Command(flight=flight(0, 0, 1.0, 0))
    assert isinstance(c.flight, flight)
    assert c.camera is None


def test_command_accepts_a_route_command():
    c = Command(flight=Route(waypoints=[(1, 2)], alt=30.0))
    assert isinstance(c.flight, Route)


def test_command_accepts_camera_only():
    c = Command(camera="oblique")
    assert c.flight is None and c.camera == "oblique"


def test_command_rejects_unknown_camera():
    with pytest.raises(ValueError):
        Command(camera="telephoto")


def test_command_rejects_a_non_command_flight():
    with pytest.raises(TypeError):
        Command(flight=(0, 0, 0))


def test_command_must_set_something():
    with pytest.raises(ValueError):
        Command()


def test_agent_callbacks_all_default_to_none():
    a = Agent()
    assert a.on_start(None) is None
    assert a.on_frame(None, None) is None
    assert a.on_tick(None) is None
    assert a.on_arrival(None) is None
    assert a.on_waypoint(0, None) is None
    assert a.on_route_complete(None) is None
