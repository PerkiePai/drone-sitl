"""Tests for the web layer of joystick-server.py.

These exist because of a bug that everything else missed: bare `uvicorn` ships
WITHOUT a WebSocket implementation, so `GET /` and `/config` kept returning 200
while `/ws` -- the entire control path -- failed to upgrade. The page loaded,
looked fine, and did nothing.

Note a Starlette TestClient would NOT catch that: it bypasses uvicorn's
protocol layer entirely, so the WebSocket test below deliberately runs a real
server in a subprocess.
"""
import asyncio
import importlib.util
import json
import os
import socket
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

WEB_PORT = 8097            # not 8090: must not collide with a running server
MAVLINK_PORT = 14599       # not 14540: must not collide with real PX4 SITL


def test_uvicorn_has_a_websocket_implementation():
    """Fast guard for the exact regression. `pip install uvicorn` alone is not
    enough -- it needs uvicorn[standard], websockets, or wsproto."""
    assert (importlib.util.find_spec("websockets")
            or importlib.util.find_spec("wsproto")), (
        "no WebSocket library installed; uvicorn will refuse the /ws upgrade "
        "while HTTP endpoints keep working. Fix: pip install websockets")


def _wait_for_port(port, timeout=20.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        with socket.socket() as s:
            s.settimeout(0.5)
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return True
        time.sleep(0.2)
    return False


@pytest.fixture
def server():
    proc = subprocess.Popen(
        [sys.executable, "-u", os.path.join(ROOT, "joystick-server.py"),
         "--port", str(WEB_PORT),
         "--mavlink", f"udpin:127.0.0.1:{MAVLINK_PORT}"],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        if not _wait_for_port(WEB_PORT):
            proc.kill()
            pytest.fail(f"server did not come up on port {WEB_PORT}")
        yield proc
    finally:
        proc.kill()
        proc.wait(timeout=10)


async def _telem_where(ws, predicate, tries=25):
    """Telemetry is pushed at 5 Hz and commands are applied asynchronously, so
    wait for a frame that satisfies the predicate rather than assuming the
    very next one does."""
    last = None
    for _ in range(tries):
        last = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
        if predicate(last):
            return last
    raise AssertionError(f"condition never held; last telemetry: {last}")


def test_websocket_accepts_control_messages_and_pushes_telemetry(server):
    """The real thing: upgrade a WebSocket against a live uvicorn, push every
    message type the UI sends, and confirm telemetry still flows afterwards."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            first = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            # No PX4 in this test, so `connected` is False -- that is correct,
            # not a failure. What matters is that telemetry arrives at all.
            assert "connected" in first and "mode" in first

            await ws.send(json.dumps({"type": "axis", "dir": "fwd",
                                      "pressed": True}))
            await ws.send(json.dumps({"type": "ping"}))
            await ws.send(json.dumps({"type": "cmd", "name": "arm"}))

            later = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert later["streaming_s"] > 0     # loop still running
            return later

    result = asyncio.run(exercise())
    assert result is not None


def test_server_logs_no_websocket_support_warning(server):
    """uvicorn degrades loudly but does not crash when the WebSocket library
    is missing -- it just refuses every upgrade. Catch that warning here so a
    silently broken control path fails the suite instead of the demo."""
    websockets = pytest.importorskip("websockets")

    async def connect_once():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)

    asyncio.run(connect_once())

    server.kill()
    out = server.stdout.read()
    assert "No supported WebSocket library" not in out, (
        f"uvicorn refused the WebSocket upgrade:\n{out}")
    assert "Unsupported upgrade request" not in out


def test_config_exposes_mission_speed_for_eta(server):
    """The map shows ETA to the next waypoint, which is a lie unless it uses
    the speed the server actually clamped PX4 to."""
    import urllib.request
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/config") as r:
        cfg = json.loads(r.read())
    assert cfg["video_port"] == 8080
    assert cfg["mission_speed"] == 3.0


def test_mission_can_be_planned_flown_paused_and_cleared_over_the_socket(server):
    """Full protocol round-trip against a live uvicorn, no PX4 needed."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)

            await ws.send(json.dumps({
                "type": "mission", "action": "fly",
                "points": [[40.0, -74.0], [40.001, -74.0]], "alt": 12.0}))
            t = await _telem_where(ws, lambda t: t["mission"]["count"] == 2)
            assert t["mission"]["state"] == "RUNNING"

            await ws.send(json.dumps({"type": "mission", "action": "pause"}))
            t = await _telem_where(ws, lambda t: t["mission"]["state"] == "PAUSED")

            # RESUME: same action, no points -- continues the loaded route
            await ws.send(json.dumps({"type": "mission", "action": "fly"}))
            t = await _telem_where(ws, lambda t: t["mission"]["state"] == "RUNNING")
            assert t["mission"]["count"] == 2      # route retained

            await ws.send(json.dumps({"type": "mission", "action": "clear"}))
            t = await _telem_where(ws, lambda t: t["mission"]["count"] == 0)
            assert t["mission"]["state"] == "IDLE"

    asyncio.run(exercise())


def test_pressing_a_direction_pauses_a_running_mission(server):
    """The takeover edge. Polling held() at 20 Hz would miss a press-release
    inside one tick and keep flying the route; a WS message cannot be missed."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({
                "type": "mission", "action": "fly",
                "points": [[40.0, -74.0]], "alt": 12.0}))
            await _telem_where(ws, lambda t: t["mission"]["state"] == "RUNNING")

            await ws.send(json.dumps({"type": "axis", "dir": "fwd",
                                      "pressed": True}))
            t = await _telem_where(ws, lambda t: t["mission"]["state"] == "PAUSED")
            assert t["mission"]["count"] == 1     # route retained, not cleared

    asyncio.run(exercise())


def test_releasing_a_direction_does_not_pause(server):
    """Only pressed=True pauses. If releases paused too, the mission would
    re-pause forever and RESUME could never take."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({
                "type": "mission", "action": "fly",
                "points": [[40.0, -74.0]], "alt": 12.0}))
            await _telem_where(ws, lambda t: t["mission"]["state"] == "RUNNING")

            await ws.send(json.dumps({"type": "axis", "dir": "fwd",
                                      "pressed": False}))
            for _ in range(4):
                t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert t["mission"]["state"] == "RUNNING"

    asyncio.run(exercise())
