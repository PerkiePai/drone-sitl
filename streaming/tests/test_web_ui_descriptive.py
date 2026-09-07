"""Tests for joystick-server-descriptive.py -- the labelled-UI / ground-truth
fork of joystick-server.py.

Only the deltas from joystick-server.py are covered here (the shared control
path is already exercised by test_web_ui.py against the original):

  * serves web-v2/ (the legend markup) instead of web/
  * telemetry frame carries lat_gt / lon_gt / heading_gt
  * those track /tmp/drone_truth.json and revert to None once it goes stale
"""
import asyncio
import json
import os
import socket
import subprocess
import sys
import tempfile
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

WEB_PORT = 8098            # not 8097: must not collide with test_web_ui.py
MAVLINK_PORT = 14598
# A private truth file per run -- never /tmp/drone_truth.json, which a live
# Isaac sim on the same box would be rewriting at 10 Hz.
TRUTH_FILE = os.path.join(tempfile.gettempdir(),
                          f"drone_truth_test_{os.getpid()}.json")


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
    if os.path.exists(TRUTH_FILE):
        os.remove(TRUTH_FILE)
    proc = subprocess.Popen(
        [sys.executable, "-u", os.path.join(ROOT, "joystick-server-descriptive.py"),
         "--port", str(WEB_PORT),
         "--mavlink", f"udpin:127.0.0.1:{MAVLINK_PORT}"],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env={**os.environ, "DRONE_TRUTH_FILE": TRUTH_FILE})
    try:
        if not _wait_for_port(WEB_PORT):
            proc.kill()
            pytest.fail(f"server did not come up on port {WEB_PORT}")
        yield proc
    finally:
        proc.kill()
        proc.wait(timeout=10)
        if os.path.exists(TRUTH_FILE):
            os.remove(TRUTH_FILE)


def _write_truth(lat, lon, hdg):
    with open(TRUTH_FILE, "w") as f:
        json.dump({"t": time.time(), "lat": lat, "lon": lon,
                   "alt": 500.0, "heading_deg": hdg}, f)


def test_serves_web_v2_with_the_map_legend(server):
    import urllib.request
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/") as r:
        body = r.read()
    assert b'id="maplegend"' in body
    assert b"ground truth (sim)" in body
    with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/js/map.js") as r:
        assert b"gt-arrow" in r.read()


def test_agent_source_returns_script_text_and_rejects_traversal(server):
    import urllib.error
    import urllib.request

    # upload a script, then read it back through /agent/source
    src = b"from competition import Agent\nclass A(Agent):\n    pass  # marker-xyz\n"
    up = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload?name=srccheck.py",
        data=src, method="POST", headers={"Content-Type": "text/x-python"})
    stored = json.loads(urllib.request.urlopen(up).read())["stored"]

    with urllib.request.urlopen(
            f"http://127.0.0.1:{WEB_PORT}/agent/source?file={stored}") as r:
        assert r.headers["content-type"].startswith("text/plain")
        assert b"marker-xyz" in r.read()

    for bad, code in [("../../etc/passwd", 400), ("evil.sh", 400),
                      ("nope.py", 404)]:
        try:
            urllib.request.urlopen(
                f"http://127.0.0.1:{WEB_PORT}/agent/source?file={bad}")
            assert False, f"expected {code} for {bad}"
        except urllib.error.HTTPError as e:
            assert e.code == code


def test_telemetry_carries_ground_truth_fields(server):
    websockets = pytest.importorskip("websockets")

    async def exercise():
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            # present from the first frame, null until the sim publishes truth
            assert t["lat_gt"] is None and t["lon_gt"] is None
            assert "heading_gt" in t
            return t

    assert asyncio.run(exercise()) is not None


def test_ground_truth_tracks_the_file_and_expires_when_stale(server):
    websockets = pytest.importorskip("websockets")

    async def read_gt(ws):
        for _ in range(15):
            t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            if t["lat_gt"] is not None:
                return t
        return t

    async def exercise():
        # keep the file fresh across several server poll cycles
        for _ in range(12):
            _write_truth(47.1, 8.2, 123.4)
            await asyncio.sleep(0.2)
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            # one more refresh right before we start reading
            _write_truth(47.1, 8.2, 123.4)
            t = await read_gt(ws)
            assert t["lat_gt"] == pytest.approx(47.1)
            assert t["lon_gt"] == pytest.approx(8.2)
            assert t["heading_gt"] == pytest.approx(123.4)

        # stop refreshing -> file ages past TRUTH_MAX_AGE_S -> fields clear
        await asyncio.sleep(2.5)
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            for _ in range(15):
                t = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            assert t["lat_gt"] is None

    asyncio.run(exercise())
