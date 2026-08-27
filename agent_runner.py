#!/usr/bin/env python3
"""Child process that flies an uploaded competition Agent.

Spawned by joystick-server.py once the aircraft is in OFFBOARD. Loads the
script, runs competition.harness.Harness, pulls frames from the Isaac camera
server, and streams Commands back to joystick-server.py over
/agent/control. Never imports pymavlink; its only outward effect is JSON on
that socket.

    agent_runner.py --file logs/agents/foo-20260827-120000.py

Runnable by hand for debugging; joystick-server.py fills in --file/--host/
--port so the operator never types a flag.
"""
import argparse
import asyncio
import importlib.util
import inspect
import json
import os
import sys
import threading
import traceback

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

from competition import Agent                       # noqa: E402
from competition.frames import MjpegFrames          # noqa: E402
from competition.harness import Harness             # noqa: E402
from competition.state import Arena                 # noqa: E402

import websockets                                   # noqa: E402


def load_agent(path):
    """Import `path` and return its single Agent subclass."""
    spec = importlib.util.spec_from_file_location("_uploaded_agent", path)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except BaseException as exc:                     # syntax / import error
        raise RuntimeError(f"could not load {path}: {exc!r}") from exc
    found = [obj for _, obj in inspect.getmembers(module, inspect.isclass)
             if issubclass(obj, Agent) and obj is not Agent
             and obj.__module__ == module.__name__]
    if not found:
        raise RuntimeError(f"{path}: no Agent subclass found -- your class must "
                           f"be `class MyAgent(Agent):`")
    if len(found) > 1:
        raise RuntimeError(f"{path}: more than one Agent subclass "
                           f"({', '.join(c.__name__ for c in found)}) -- "
                           f"upload exactly one")
    return found[0]


class WsChannel:
    """Adapts the /agent/control WebSocket to the Harness channel contract."""

    def __init__(self, ws, loop):
        self._ws = ws
        self._loop = loop
        self._telem = {}
        self._lock = threading.Lock()

    def telemetry(self):
        with self._lock:
            return dict(self._telem)

    def absorb(self, frame):
        with self._lock:
            self._telem = frame

    def send(self, msg):
        # Called from the harness worker threads; hop to the asyncio loop.
        asyncio.run_coroutine_threadsafe(
            self._ws.send(json.dumps(msg)), self._loop)


async def _run(args):
    uri = f"ws://{args.host}:{args.port}/agent/control"
    async with websockets.connect(uri) as ws:
        loop = asyncio.get_running_loop()
        channel = WsChannel(ws, loop)

        # First useful frame is the arena bootstrap; wait for a non-null origin.
        origin, radius, time_limit = None, 500.0, None
        while origin is None:
            frame = json.loads(await ws.recv())
            if frame.get("type") == "arena":
                radius = frame.get("radius_m", 500.0)
                time_limit = frame.get("time_limit")
                origin = frame.get("origin")
            # else: a telemetry frame before home was valid; keep waiting.
        arena = Arena.around(origin[0], origin[1], radius, time_limit)

        frames = MjpegFrames(args.host, args.video_port, camera="nadir")
        frames.start()

        agent_cls = load_agent(args.file)
        harness = Harness(agent_cls(), channel, frames, arena,
                          log=lambda m: print(m, flush=True))

        async def pump():
            try:
                async for raw in ws:
                    frame = json.loads(raw)
                    if frame.get("type") != "arena":
                        channel.absorb(frame)
            except websockets.ConnectionClosed:
                harness.stop()

        pump_task = asyncio.create_task(pump())
        try:
            await loop.run_in_executor(None, harness.run)
        finally:
            harness.stop()
            frames.stop()
            pump_task.cancel()


def main():
    ap = argparse.ArgumentParser(description="Fly an uploaded competition Agent")
    ap.add_argument("--file", required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--video-port", type=int, default=8080)
    args = ap.parse_args()

    try:
        load_agent(args.file)                        # fail fast, before flying
    except RuntimeError as exc:
        print(f"[agent_runner] {exc}", flush=True)
        sys.exit(2)

    try:
        asyncio.run(_run(args))
    except Exception:                                # noqa: BLE001
        traceback.print_exc()
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
