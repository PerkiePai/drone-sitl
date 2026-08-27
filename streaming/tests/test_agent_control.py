"""streaming.agent_control.AgentControl -- the agent's setpoint holder.

Mirrors offboard.CommandState: the /agent/control handler writes, the
setpoint thread reads once per tick, a silent child decays to hover. The
+up -> NED-down flip lives here and nowhere else.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))

from agent_control import AgentControl  # noqa: E402


def test_no_command_by_default():
    ac = AgentControl()
    assert ac.command() is None
    assert ac.active() is False


def test_body_velocity_flips_up_to_ned_down_and_converts_yaw_to_radians():
    ac = AgentControl(watchdog_s=10.0)
    ac.set_velocity_body(forward=3.0, right=1.0, up=2.0, yaw_rate=90.0)
    kind, vx, vy, vz, yaw = ac.command(now=0.0)
    assert kind == "body"
    assert (vx, vy) == (3.0, 1.0)
    assert vz == -2.0                       # +up -> NED down
    assert math.isclose(yaw, math.radians(90.0))
    assert ac.active() is True


def test_world_velocity_flips_up_and_keeps_north_east():
    ac = AgentControl(watchdog_s=10.0)
    ac.set_velocity_world(north=2.0, east=-1.0, up=1.5, yaw_rate=0.0)
    kind, vn, ve, vd, yaw = ac.command(now=0.0)
    assert kind == "world"
    assert (vn, ve) == (2.0, -1.0)
    assert vd == -1.5


def test_velocity_decays_to_zero_after_the_watchdog():
    ac = AgentControl(watchdog_s=0.5)
    ac.set_velocity_body(forward=3.0, right=0.0, up=0.0, yaw_rate=0.0, now=0.0)
    assert ac.command(now=0.4)[1] == 3.0
    stale = ac.command(now=1.0)
    assert stale == ("body", 0.0, 0.0, 0.0, 0.0)   # still "body", just zeroed
    assert ac.active() is True                       # still latched, just hover


def test_hold_is_zero_body_velocity_and_is_not_watchdogged():
    ac = AgentControl(watchdog_s=0.5)
    ac.hold(now=0.0)
    assert ac.command(now=100.0) == ("body", 0.0, 0.0, 0.0, 0.0)


def test_clear_drops_back_to_no_command():
    ac = AgentControl()
    ac.set_velocity_body(1.0, 0.0, 0.0, 0.0)
    ac.clear()
    assert ac.command() is None
    assert ac.active() is False


def test_switching_from_world_to_body_replaces_not_merges():
    ac = AgentControl(watchdog_s=10.0)
    ac.set_velocity_world(north=5.0, east=0.0, up=0.0, yaw_rate=0.0)
    ac.set_velocity_body(forward=1.0, right=0.0, up=0.0, yaw_rate=0.0)
    assert ac.command(now=0.0)[0] == "body"
