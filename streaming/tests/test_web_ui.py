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


def test_telemetry_carries_a_recorder_block_with_isaac_down(server):
    """Isaac is not running in the test suite, and that is the common case on
    the page too -- before launch-sitl.sh finishes. It must read as `offline`
    with the button disabled, not as a missing key the UI then throws on."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert t["rec"]["state"] == "offline"
            assert t["rec"]["can_start"] is False

    asyncio.run(exercise())


def test_a_record_press_with_no_isaac_reports_a_reason_and_keeps_flying(server):
    """The command cannot succeed, so what matters is that it fails visibly and
    that the flight path is untouched: telemetry keeps arriving and the socket
    keeps taking commands."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({"type": "record", "action": "start"}))

            t = await _telem_where(ws, lambda t: t["rec"]["cmd_error"])
            assert "sim" in t["rec"]["cmd_error"].lower()
            assert t["rec"]["state"] == "offline"

            # The socket is still live and still driving the aircraft.
            await ws.send(json.dumps({"type": "axis", "dir": "fwd",
                                      "pressed": True}))
            t = await _telem_where(ws, lambda t: t["streaming_s"] > 0)

    asyncio.run(exercise())


def _fetch(path):
    import urllib.request
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}{path}") as r:
        return r.status, r.read().decode()


def test_the_record_button_and_its_module_are_actually_served(server):
    """ES modules fail as a chain: if /js/recorder.js 404s, main.js never
    finishes importing and NOTHING on the page updates -- telemetry, map and
    d-pad included. A missing file is silent in the browser and fatal here."""
    status, html = _fetch("/")
    assert status == 200
    assert 'id="c-record"' in html
    assert 'id="rec-stat"' in html

    status, recorder_js = _fetch("/js/recorder.js")
    assert status == 200
    assert "paintRecorder" in recorder_js

    _, main_js = _fetch("/js/main.js")
    assert "recorder.js" in main_js and "paintRecorder(t)" in main_js


def test_every_element_recorder_js_paints_exists_on_the_page(server):
    """A typo'd id makes el(...) return null, and the TypeError kills the whole
    telemetry paint on every frame -- the map and the flight readouts freeze
    because of a mistake in the recorder row."""
    import re

    _, recorder_js = _fetch("/js/recorder.js")
    _, html = _fetch("/")
    ids = set(re.findall(r"el\('([\w-]+)'\)", recorder_js))
    assert ids, "no el('...') lookups found; did the module change shape?"
    missing = [i for i in ids if f'id="{i}"' not in html]
    assert not missing, f"recorder.js paints ids that index.html lacks: {missing}"


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


# --- the VIO row (plan Task 7) ---------------------------------------------

def _read(*parts):
    with open(os.path.join(ROOT, *parts), encoding="utf-8") as fh:
        return fh.read()


def test_the_vio_row_is_hidden_until_the_server_offers_vision():
    """An ordinary GPS flight must not show a dead VIO row."""
    html = _read("web", "index.html")
    assert 'id="telem-vio"' in html
    row = html.split('id="telem-vio"')[1].split(">")[0]
    assert "hidden" in row, "the VIO row must start hidden"
    for field in ("t-vio", "t-vio-drift", "t-vio-pts", "t-vio-fps"):
        assert f'id="{field}"' in html


def test_stale_vision_renders_bad_and_says_so():
    """Stale means the aircraft is on dead reckoning. It has to be unmissable,
    not a quiet absence."""
    js = _read("web", "js", "telemetry.js")
    assert "STALE" in js
    assert "v.fresh ? 'good' : 'bad'" in js


def test_absent_drift_renders_as_dashes_never_zero():
    """No GT topic means "cannot tell", which is not the same as "no drift" --
    and 0.0 would be the single most reassuring wrong number on the page."""
    js = _read("web", "js", "telemetry.js")
    seg = js.split("t-vio-drift")[1].split(";")[0]
    assert "'--'" in seg
    assert "null" in seg


def test_telemetry_carries_no_vio_block_without_the_vision_flag():
    """The page keys the row off this being null."""
    sys.path.insert(0, os.path.join(ROOT, "streaming"))
    import importlib.util as iu
    import offboard
    spec = iu.spec_from_file_location(
        "joystick_server_vio", os.path.join(ROOT, "joystick-server.py"))
    mod = iu.module_from_spec(spec)
    spec.loader.exec_module(mod)

    from pymavlink import mavutil
    conn = mavutil.mavlink_connection("udpout:127.0.0.1:14599")
    loop = mod.SetpointLoop(conn, offboard.CommandState(2.0, 1.0))
    assert loop.telemetry()["vio"] is None
    assert loop._vio_status(0.0) is None
