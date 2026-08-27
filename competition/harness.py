"""The loop the harness owns so the competitor doesn't.

Given an Agent, a control channel and a frame source, this calls the Agent's
callbacks at the right rates, turns each returned Command into a channel
message, and synthesizes arrival events from mission telemetry.

A callback that overruns its budget is abandoned -- run on a worker thread,
joined with a timeout; the previous command stays in force. That is the
contract in docs/competition-api.md section 1. A genuinely wedged callback
leaves a daemon thread parked forever; Stop (killing the process) is the
backstop for that.
"""
import threading
import time

from competition.commands import (
    Command, Goto, Hold, Route, Velocity, VelocityWorld,
)
from competition.state import State

TICK_HZ = 20.0
FRAME_HZ = 5.0
BUDGET_FRAME_S = 0.200
BUDGET_TICK_S = 0.050
BUDGET_EVENT_S = 0.200
_OVERRUN_LOG_EVERY_S = 5.0


def _call_with_timeout(fn, budget_s, *args):
    """(status, payload): ("ok", result) | ("overrun", None) | ("error", exc)."""
    box = {}

    def target():
        try:
            box["result"] = fn(*args)
        except BaseException as exc:          # reported upward, not swallowed
            box["error"] = exc

    t = threading.Thread(target=target, daemon=True)
    t.start()
    t.join(budget_s)
    if t.is_alive():
        return "overrun", None
    if "error" in box:
        return "error", box["error"]
    return "ok", box.get("result")


class Harness:
    def __init__(self, agent, channel, frames, arena, *,
                 now=time.monotonic, sleep=time.sleep, log=print):
        self.agent = agent
        self.channel = channel          # .send(dict); .telemetry() -> dict
        self.frames = frames            # .latest() -> ndarray|None; .select(name)
        self.arena = arena
        self._now = now
        self._sleep = sleep
        self._log = log
        self._camera = "nadir"
        self._started_at = None
        self._last_frame_at = None
        self._prev_mission = {"state": "IDLE", "index": 0, "count": 0}
        self._last_overrun_log = -1e9
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    # -- command translation ------------------------------------------------

    def _emit(self, cmd):
        if cmd is None:
            return
        if not isinstance(cmd, Command):
            self._log(f"[harness] callback returned {type(cmd).__name__}, "
                      f"expected Command or None -- ignored")
            return
        if cmd.camera is not None and cmd.camera != self._camera:
            self._camera = cmd.camera
            self.frames.select(cmd.camera)
            self.channel.send({"type": "camera", "camera": cmd.camera})
        f = cmd.flight
        if isinstance(f, Velocity):
            self.channel.send({"type": "velocity", "forward": f.forward,
                               "right": f.right, "up": f.up,
                               "yaw_rate": f.yaw_rate})
        elif isinstance(f, VelocityWorld):
            self.channel.send({"type": "velocity_world", "north": f.north,
                               "east": f.east, "up": f.up,
                               "yaw_rate": f.yaw_rate})
        elif isinstance(f, Goto):
            self.channel.send({"type": "goto", "lat": f.lat, "lon": f.lon,
                               "alt": f.alt, "speed": f.speed})
        elif isinstance(f, Route):
            self.channel.send({"type": "route",
                               "points": [[a, b] for a, b in f.waypoints],
                               "alt": f.alt, "speed": f.speed})
        elif isinstance(f, Hold):
            self.channel.send({"type": "hold"})

    # -- dispatch ----------------------------------------------------------

    def _dispatch(self, fn, budget, *args):
        status, payload = _call_with_timeout(fn, budget, *args)
        if status == "overrun":
            now = self._now()
            if now - self._last_overrun_log > _OVERRUN_LOG_EVERY_S:
                self._log(f"[harness] {fn.__name__} overran "
                          f"{budget * 1000:.0f} ms -- call abandoned, "
                          f"previous command holds")
                self._last_overrun_log = now
            return None
        if status == "error":
            raise payload
        return payload

    def _state(self):
        return State.from_telemetry(
            self.channel.telemetry(), camera=self._camera,
            time_elapsed=self._now() - self._started_at,
            time_limit=self.arena.time_limit)

    # -- events ----------------------------------------------------------

    def _fire_events(self, state):
        m = self.channel.telemetry().get("mission") or {}
        prev = self._prev_mission
        cur = {"state": m.get("state", "IDLE"),
               "index": int(m.get("index", 0)),
               "count": int(m.get("count", 0))}
        if cur["count"] > 1 and cur["index"] > prev["index"]:
            for i in range(prev["index"], cur["index"]):
                self._emit(self._dispatch(self.agent.on_waypoint,
                                          BUDGET_EVENT_S, i, state))
        if prev["state"] == "RUNNING" and cur["state"] == "DONE":
            if cur["count"] <= 1:
                self._emit(self._dispatch(self.agent.on_arrival,
                                          BUDGET_EVENT_S, state))
            else:
                self._emit(self._dispatch(self.agent.on_route_complete,
                                          BUDGET_EVENT_S, state))
        self._prev_mission = cur

    # -- loop ----------------------------------------------------------

    def run(self):
        self._started_at = self._now()
        self._last_frame_at = self._started_at - 1.0 / FRAME_HZ
        self._emit(self._dispatch(self.agent.on_start, BUDGET_EVENT_S,
                                  self.arena))
        tick_dt = 1.0 / TICK_HZ
        frame_dt = 1.0 / FRAME_HZ
        next_tick = self._now()
        while not self._stop.is_set():
            state = self._state()
            self._fire_events(state)
            if (self.arena.time_limit is not None
                    and state.time_elapsed >= self.arena.time_limit):
                self._log("[harness] time limit reached -- holding")
                self.channel.send({"type": "hold"})
                return
            now = self._now()
            if now - self._last_frame_at >= frame_dt:
                self._last_frame_at = now
                self._emit(self._dispatch(self.agent.on_frame, BUDGET_FRAME_S,
                                          self.frames.latest(), state))
            else:
                self._emit(self._dispatch(self.agent.on_tick, BUDGET_TICK_S,
                                          state))
            next_tick += tick_dt
            nap = next_tick - self._now()
            if nap > 0:
                self._sleep(nap)
            else:
                next_tick = self._now()
