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

            await ws.send(json.dumps({"type": "stick", "stick": "right",
                                      "x": 0, "y": -1}))
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


def test_deflecting_a_stick_pauses_a_running_mission(server):
    """The takeover edge. Polling command() at 20 Hz would miss a
    deflect-and-release inside one tick and keep flying the route; a WS
    message cannot be missed."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({
                "type": "mission", "action": "fly",
                "points": [[40.0, -74.0]], "alt": 12.0}))
            await _telem_where(ws, lambda t: t["mission"]["state"] == "RUNNING")

            await ws.send(json.dumps({"type": "stick", "stick": "right",
                                      "x": 0, "y": -1}))
            t = await _telem_where(ws, lambda t: t["mission"]["state"] == "PAUSED")
            assert t["mission"]["count"] == 1     # route retained, not cleared

    asyncio.run(exercise())


def test_centering_a_stick_does_not_pause(server):
    """Only the rest-to-active edge pauses. If returning to center paused
    too, the mission would re-pause forever and RESUME could never take."""
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({
                "type": "mission", "action": "fly",
                "points": [[40.0, -74.0]], "alt": 12.0}))
            await _telem_where(ws, lambda t: t["mission"]["state"] == "RUNNING")

            await ws.send(json.dumps({"type": "stick", "stick": "right",
                                      "x": 0, "y": 0}))
            for _ in range(4):
                t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert t["mission"]["state"] == "RUNNING"

    asyncio.run(exercise())


# --- agent upload / list -------------------------------------------------

import urllib.error  # noqa: E402


def test_agent_upload_accepts_a_py_file_and_lists_it(server):
    import urllib.request
    src = b"from competition import Agent\nclass A(Agent):\n    pass\n"
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=myagent.py",
        data=src, method="POST",
        headers={"Content-Type": "text/x-python"})
    with urllib.request.urlopen(req) as r:
        body = json.loads(r.read())
    assert body["stored"].startswith("myagent-") and body["stored"].endswith(".py")

    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/agent/list") as r:
        files = json.loads(r.read())["files"]
    assert body["stored"] in files


def test_agent_upload_rejects_a_non_py_name(server):
    import urllib.request
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=evil.sh",
        data=b"rm -rf /", method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_agent_upload_rejects_a_path_traversal_name(server):
    import urllib.request
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=../../etc/x.py",
        data=b"x = 1", method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_agent_upload_rejects_an_oversize_body(server):
    import urllib.request
    big = b"# " + b"x" * (256 * 1024 + 10)
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=big.py",
        data=big, method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


# --- docker upload / list -----------------------------------------------

def _make_tar_bytes(with_dockerfile=True):
    import io
    import tarfile
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tar:
        if with_dockerfile:
            data = b"FROM busybox\n"
            info = tarfile.TarInfo(name="Dockerfile")
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return buf.getvalue()


def test_docker_upload_rejects_non_tar_name(server):
    import urllib.request
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload-docker?name=bundle.zip",
        data=_make_tar_bytes(), method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_docker_upload_rejects_oversize_body(server):
    import urllib.request
    big = b"x" * (200 * 1024 * 1024 + 10)   # over AGENT_DOCKER_MAX_BYTES
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload-docker?name=bundle.tar",
        data=big, method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_docker_upload_without_dockerfile_in_bundle_fails(server):
    import urllib.request
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload-docker?name=bundle.tar",
        data=_make_tar_bytes(with_dockerfile=False), method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400
        body = json.loads(e.read())
        assert "Dockerfile" in body["detail"] or "log" in body


def test_agent_docker_list_returns_json_list(server):
    import urllib.request
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/agent/docker-list") as r:
        body = json.loads(r.read())
    assert "images" in body and isinstance(body["images"], list)


# --- AgentRun kind=docker (unit-level, no live server) -------------------

def _load_server():
    spec = importlib.util.spec_from_file_location(
        "joystick_server", os.path.join(ROOT, "joystick-server.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _FakeAgentControl:
    def clear(self):
        pass


class _FakeLoopThread:
    """Just enough of SetpointLoop's surface for AgentRun's preflight state
    machine: .telemetry()/.submit(), plus .agent_control/.agent_camera."""
    def __init__(self):
        self._telem = {"connected": True, "armed": False, "alt_m": 0.0,
                       "vz": 9.0, "ready_for_offboard": False, "mode": None}
        self.submitted = []
        self.agent_control = _FakeAgentControl()
        self.agent_camera = None

    def telemetry(self):
        return dict(self._telem)

    def submit(self, name):
        self.submitted.append(name)


def _fly_to_offboard(run):
    """Drive AgentRun's arm->takeoff->offboard preflight to completion by
    ticking it and advancing the fake telemetry, same sequence
    _advance_preflight checks for."""
    run.tick()
    run.loop_thread._telem["armed"] = True
    run.tick()
    run.loop_thread._telem.update(alt_m=5.0, vz=0.0, ready_for_offboard=True)
    run.tick()
    run.loop_thread._telem["mode"] = "OFFBOARD"
    run.tick()


def test_agent_run_docker_spawns_docker_run_with_network_host_and_name(monkeypatch):
    js = _load_server()
    calls = []

    class FakePopen:
        def __init__(self, argv, **kwargs):
            calls.append(argv)
            self.stdout = iter([])
        def poll(self):
            return None

    monkeypatch.setattr(js.subprocess, "Popen", FakePopen)
    # nvidia-container-runtime present -> --gpus all is safe to pass.
    monkeypatch.setattr(js.shutil, "which",
                        lambda name: "/usr/bin/nvidia-container-runtime")

    run = js.AgentRun(_FakeLoopThread(), "python", "127.0.0.1", 8090, 8080)
    run.run("docker", "submission-foo-20260908-120000")
    _fly_to_offboard(run)

    assert calls, "docker run was never spawned"
    argv = calls[0]
    assert argv[:6] == ["docker", "run", "--rm", "--network", "host", "--gpus"]
    assert "submission-foo-20260908-120000" in argv
    assert "--name" in argv
    assert run.state == "running"


def test_agent_run_docker_omits_gpus_flag_when_toolkit_unavailable(monkeypatch):
    """Caught live: docker info can list an "nvidia" runtime in
    /etc/docker/daemon.json with no nvidia-container-toolkit actually
    installed -- `--gpus all` then fails the whole run with
    "could not select device driver". Degrade to CPU instead of failing."""
    js = _load_server()
    calls = []

    class FakePopen:
        def __init__(self, argv, **kwargs):
            calls.append(argv)
            self.stdout = iter([])
        def poll(self):
            return None

    monkeypatch.setattr(js.subprocess, "Popen", FakePopen)
    monkeypatch.setattr(js.shutil, "which", lambda name: None)

    run = js.AgentRun(_FakeLoopThread(), "python", "127.0.0.1", 8090, 8080)
    run.run("docker", "submission-foo-20260908-120000")
    _fly_to_offboard(run)

    assert calls, "docker run was never spawned"
    argv = calls[0]
    assert "--gpus" not in argv
    assert argv[:5] == ["docker", "run", "--rm", "--network", "host"]
    assert run.state == "running"


def test_agent_run_stop_issues_docker_stop_for_docker_kind(monkeypatch):
    js = _load_server()
    stop_calls = []

    class FakePopen:
        def __init__(self, argv, **kwargs):
            self.stdout = iter([])
        def poll(self):
            return None
        def terminate(self):
            pass
        def wait(self, timeout=None):
            pass

    def fake_run(argv, **kwargs):
        stop_calls.append(argv)
        class R:
            returncode = 0
        return R()

    monkeypatch.setattr(js.subprocess, "Popen", FakePopen)
    monkeypatch.setattr(js.subprocess, "run", fake_run)

    run = js.AgentRun(_FakeLoopThread(), "python", "127.0.0.1", 8090, 8080)
    run.run("docker", "submission-foo-20260908-120000")
    _fly_to_offboard(run)

    run.stop("test")

    assert any(a[:2] == ["docker", "stop"] for a in stop_calls)
    assert run.state == "stopped"


def test_agent_run_stop_reaches_stopped_even_if_docker_stop_times_out(monkeypatch):
    """Caught live: docker stop's default 10s grace period can outlast a
    short subprocess.run timeout, raising TimeoutExpired -- that must not
    crash stop() before it reaches state = "stopped", or tick() later
    mistakes the eventual SIGKILL exit for a real error."""
    js = _load_server()

    class FakePopen:
        def __init__(self, argv, **kwargs):
            self.stdout = iter([])
        def poll(self):
            return None
        def terminate(self):
            pass
        def wait(self, timeout=None):
            pass

    def fake_run(argv, **kwargs):
        raise js.subprocess.TimeoutExpired(cmd=argv, timeout=kwargs.get("timeout"))

    monkeypatch.setattr(js.subprocess, "Popen", FakePopen)
    monkeypatch.setattr(js.subprocess, "run", fake_run)

    run = js.AgentRun(_FakeLoopThread(), "python", "127.0.0.1", 8090, 8080)
    run.run("docker", "submission-foo-20260908-120000")
    _fly_to_offboard(run)

    run.stop("test")

    assert run.state == "stopped"


# --- /agent/control socket ---------------------------------------------

def test_agent_control_socket_sends_arena_then_applies_a_flight(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(
                f"ws://127.0.0.1:{WEB_PORT}/agent/control") as ws:
            first = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert first["type"] == "arena"
            assert "radius_m" in first and "time_limit" in first

            await ws.send(json.dumps({"type": "flight", "left_x": 0.0,
                                      "left_y": 0.0, "right_x": 0.0,
                                      "right_y": 1.0}))
            for _ in range(10):
                t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
                if t.get("type") != "arena":
                    break
            assert "streaming_s" in t

    asyncio.run(exercise())


def test_agent_control_route_message_loads_and_flies_a_mission(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(
                f"ws://127.0.0.1:{WEB_PORT}/agent/control") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)   # arena
            await ws.send(json.dumps({
                "type": "route",
                "points": [[40.0, -74.0], [40.001, -74.0]],
                "alt": 20.0, "speed": None}))
            t = await _telem_where(ws, lambda t: (t.get("mission") or {}).get(
                "count") == 2)
            assert t["mission"]["state"] == "RUNNING"

    asyncio.run(exercise())


# --- /ws agent run / stop ----------------------------------------------

def test_ws_agent_run_is_refused_when_link_is_down(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            await asyncio.wait_for(ws.recv(), timeout=10)
            await ws.send(json.dumps({"type": "agent", "action": "run",
                                      "file": "whatever.py"}))
            t = await _telem_where(ws, lambda t: "agent" in t)
            for _ in range(6):
                t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
                assert t["agent"]["state"] in ("idle", "error")

    asyncio.run(exercise())


def test_ws_telemetry_carries_an_agent_block_from_the_start(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            t = await _telem_where(ws, lambda t: "agent" in t)
            assert t["agent"] == {"state": "idle", "kind": None, "file": None,
                                  "camera": None, "log": []}

    asyncio.run(exercise())


def test_agent_js_and_panel_are_served(server):
    import urllib.request
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/js/agent.js") as r:
        assert r.status == 200 and b"initAgent" in r.read()
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/") as r:
        assert b'id="agent"' in r.read()
