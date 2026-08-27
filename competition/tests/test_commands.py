"""competition.commands -- construction and validation.

A malformed command is a bug in the uploaded script; it must raise where the
script writes it, not fly. No PX4, no server.
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition import (  # noqa: E402
    Agent, Command, Velocity, VelocityWorld, Goto, Route, Hold,
)


def test_velocity_defaults_to_all_zero_hover():
    v = Velocity()
    assert (v.forward, v.right, v.up, v.yaw_rate) == (0.0, 0.0, 0.0, 0.0)


def test_velocity_is_frozen():
    v = Velocity(forward=1.0)
    with pytest.raises(Exception):
        v.forward = 2.0


def test_velocity_rejects_non_numeric():
    with pytest.raises(TypeError):
        Velocity(forward="fast")


def test_velocity_world_has_north_east_not_forward_right():
    v = VelocityWorld(north=2.0, east=-1.0)
    assert v.north == 2.0 and v.east == -1.0
    assert not hasattr(v, "forward")


def test_goto_speed_must_be_positive_when_given():
    Goto(1.0, 2.0, 30.0)              # None speed is fine
    Goto(1.0, 2.0, 30.0, speed=5.0)
    with pytest.raises(ValueError):
        Goto(1.0, 2.0, 30.0, speed=0.0)


def test_route_needs_at_least_one_waypoint():
    with pytest.raises(ValueError):
        Route(waypoints=[], alt=30.0)


def test_route_normalizes_waypoints_to_tuple_of_float_pairs():
    r = Route(waypoints=[[1, 2], (3, 4)], alt=30.0)
    assert r.waypoints == ((1.0, 2.0), (3.0, 4.0))


def test_hold_takes_no_fields():
    Hold()


def test_command_accepts_a_flight_command():
    c = Command(flight=Velocity(forward=3.0))
    assert isinstance(c.flight, Velocity)
    assert c.camera is None


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
