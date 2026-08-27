"""competition.harness -- the loop the harness owns so the script doesn't.

Driven by a virtual clock so cadence assertions are exact. Callbacks that
overrun are tested with real threads and a small real budget.
"""
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition import Agent, Command, Goto, Hold, Route, Velocity, VelocityWorld  # noqa: E402
from competition.harness import Harness  # noqa: E402
from competition.state import Arena  # noqa: E402


class FakeChannel:
    def __init__(self, telem=None):
        self.sent = []
        self._telem = telem or {"mission": {"state": "IDLE", "index": 0,
                                            "count": 0, "dist_m": None}}
        self.lock = threading.Lock()

    def send(self, msg):
        with self.lock:
            self.sent.append(msg)

    def telemetry(self):
        with self.lock:
            return dict(self._telem)

    def set_telemetry(self, telem):
        with self.lock:
            self._telem = telem

    def types(self):
        with self.lock:
            return [m["type"] for m in self.sent]


class FakeFrames:
    def __init__(self):
        self.selected = "nadir"
        self.image = "FRAME"

    def latest(self):
        return self.image

    def select(self, camera):
        self.selected = camera


class VirtualClock:
    """now()/sleep() that advance a shared counter -- no wall time passes.

    When bound to a harness with a deadline, sleep() stops the harness the
    instant virtual time crosses it, so callback-count assertions are exact
    rather than a race against the wall clock.
    """
    def __init__(self):
        self.t = 0.0
        self._harness = None
        self._deadline = None

    def bind(self, harness, deadline):
        self._harness = harness
        self._deadline = deadline

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 0.0)
        if self._deadline is not None and self.t >= self._deadline:
            self._harness.stop()


def _run_for(harness, clock, seconds):
    """Run the harness to `seconds` of virtual time, deterministically."""
    clock.bind(harness, seconds)
    th = threading.Thread(target=harness.run, daemon=True)
    th.start()
    th.join(timeout=2.0)
    assert not th.is_alive(), "harness.run did not return after stop()"


def test_on_start_result_is_sent_first():
    class A(Agent):
        def on_start(self, arena):
            return Command(flight=Velocity(forward=5.0))

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.01)
    assert ch.sent[0] == {"type": "velocity", "forward": 5.0, "right": 0.0,
                          "up": 0.0, "yaw_rate": 0.0}


def test_on_tick_runs_at_20hz_and_on_frame_at_5hz():
    counts = {"tick": 0, "frame": 0}

    class A(Agent):
        def on_tick(self, state):
            counts["tick"] += 1

        def on_frame(self, image, state):
            counts["frame"] += 1
            assert image == "FRAME"

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 1.0)
    assert 4 <= counts["frame"] <= 6, counts
    assert 13 <= counts["tick"] <= 17, counts
    assert counts["frame"] + counts["tick"] >= 19


def test_a_callback_that_overruns_its_budget_is_abandoned():
    """Real threads, small real budget. The slow on_tick must not block the
    loop, and its (late) return value must never reach the channel."""
    class A(Agent):
        def __init__(self):
            self.calls = 0

        def on_start(self, arena):
            return Command(flight=Velocity(forward=1.0))

        def on_tick(self, state):
            self.calls += 1
            time.sleep(0.5)                       # >> BUDGET_TICK_S
            return Command(flight=Velocity(forward=99.0))  # must be dropped

    ch = FakeChannel()
    agent = A()
    h = Harness(agent, ch, FakeFrames(), Arena.around(0, 0, 500, None),
                log=lambda *a: None)             # real clock here
    th = threading.Thread(target=h.run, daemon=True)
    th.start()
    time.sleep(0.4)
    h.stop()
    th.join(timeout=2.0)
    forwards = [m["forward"] for m in ch.sent if m["type"] == "velocity"]
    assert forwards, "nothing sent"
    assert all(f == 1.0 for f in forwards), f"a 99.0 leaked through: {forwards}"


def test_none_return_sends_nothing():
    class A(Agent):
        def on_tick(self, state):
            return None

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.5)
    assert ch.sent == []


def test_camera_switch_emits_a_camera_message_and_selects_the_source():
    class A(Agent):
        def on_start(self, arena):
            return Command(camera="oblique")

    ch, clock, frames = FakeChannel(), VirtualClock(), FakeFrames()
    h = Harness(A(), ch, frames, Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.01)
    assert {"type": "camera", "camera": "oblique"} in ch.sent
    assert frames.selected == "oblique"


def test_repeated_same_camera_is_not_re_emitted():
    class A(Agent):
        def on_tick(self, state):
            return Command(camera="nadir")       # already the default

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.5)
    assert "camera" not in ch.types()


def test_route_waypoint_and_completion_events_fire_from_mission_telemetry():
    seen = []

    class A(Agent):
        def on_start(self, arena):
            return Command(flight=Route(waypoints=[(0, 0), (1, 1), (2, 2)],
                                        alt=30.0))

        def on_waypoint(self, index, state):
            seen.append(("wp", index))

        def on_route_complete(self, state):
            seen.append(("done", None))

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    th = threading.Thread(target=h.run, daemon=True)
    th.start()
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "RUNNING", "index": 0, "count": 3}})
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "RUNNING", "index": 2, "count": 3}})
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "DONE", "index": 2, "count": 3}})
    time.sleep(0.05)
    h.stop()
    th.join(timeout=2.0)
    assert ("wp", 0) in seen and ("wp", 1) in seen
    assert ("done", None) in seen


def test_single_point_goto_completion_is_on_arrival_not_route_complete():
    seen = []

    class A(Agent):
        def on_start(self, arena):
            return Command(flight=Goto(1.0, 2.0, 30.0))

        def on_arrival(self, state):
            seen.append("arrived")

        def on_route_complete(self, state):
            seen.append("route_complete")

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    th = threading.Thread(target=h.run, daemon=True)
    th.start()
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "RUNNING", "index": 0, "count": 1}})
    time.sleep(0.05)
    ch.set_telemetry({"mission": {"state": "DONE", "index": 0, "count": 1}})
    time.sleep(0.05)
    h.stop()
    th.join(timeout=2.0)
    assert seen == ["arrived"]


def test_velocity_world_up_is_passed_through_positive():
    """The NED flip is server-side. The harness must send +up verbatim."""
    class A(Agent):
        def on_start(self, arena):
            return Command(flight=VelocityWorld(north=2.0, up=1.0))

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, None),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    _run_for(h, clock, 0.01)
    msg = next(m for m in ch.sent if m["type"] == "velocity_world")
    assert msg["north"] == 2.0 and msg["up"] == 1.0


def test_time_limit_ends_the_run_with_a_hold():
    class A(Agent):
        def on_tick(self, state):
            return Command(flight=Velocity(forward=1.0))

    ch, clock = FakeChannel(), VirtualClock()
    h = Harness(A(), ch, FakeFrames(), Arena.around(0, 0, 500, 0.3),
                now=clock.now, sleep=clock.sleep, log=lambda *a: None)
    th = threading.Thread(target=h.run, daemon=True)
    th.start()
    deadline = time.monotonic() + 5.0
    while th.is_alive() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not th.is_alive(), "run() did not return at the time limit"
    assert ch.sent[-1] == {"type": "hold"}
