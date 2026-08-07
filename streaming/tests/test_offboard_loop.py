"""Integration test for the setpoint thread against a fake PX4 over real UDP.

No PX4 and no Isaac Sim required. Ports are deliberately NOT 14540/14580 so
the test cannot collide with a real SITL instance running on the same box.
"""
import asyncio
import importlib.util
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pymavlink import mavutil

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import offboard  # noqa: E402

FAKE_PX4_PORT = 14585


def _load_server():
    spec = importlib.util.spec_from_file_location(
        "joystick_server", os.path.join(ROOT, "joystick-server.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _collect(px4, seconds, kind="SET_POSITION_TARGET_LOCAL_NED"):
    seen = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = px4.recv_match(type=kind, blocking=True, timeout=0.5)
        if msg is not None:
            seen.append(msg)
    return seen


def test_loop_streams_setpoints_fast_enough_for_offboard():
    """PX4 rejects OFFBOARD unless setpoints arrive above 2 Hz. At 20 Hz we
    should comfortably clear 10 messages in one second."""
    js = _load_server()
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{FAKE_PX4_PORT}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT}")
        state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
        js.SetpointLoop(conn, state, rate_hz=20.0).start()
        state.set("fwd", True)

        seen = _collect(px4, 1.0)
        assert len(seen) >= 10, f"expected >=10 setpoints in 1 s, got {len(seen)}"

        last = seen[-1]
        assert last.coordinate_frame == offboard.MAV_FRAME_BODY_NED
        assert last.type_mask == offboard.VEL_YAWRATE_TYPE_MASK
        assert abs(last.vx - 2.0) < 1e-6
        assert last.vy == 0.0
        assert last.yaw_rate == 0.0
    finally:
        px4.close()


def test_loop_keeps_streaming_zeros_when_nothing_is_held():
    """Idle must not mean silent: a gap in the stream drops OFFBOARD, so
    hovering has to be expressed as an explicit zero setpoint."""
    js = _load_server()
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{FAKE_PX4_PORT + 1}")
    try:
        conn = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{FAKE_PX4_PORT + 1}")
        state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
        js.SetpointLoop(conn, state, rate_hz=20.0).start()

        seen = _collect(px4, 1.0)
        assert len(seen) >= 10, f"idle loop went quiet: only {len(seen)} sent"
        assert all(m.vx == 0.0 and m.vy == 0.0 and m.vz == 0.0 for m in seen)
    finally:
        px4.close()


def test_heading_ignores_the_unknown_sentinel():
    """GLOBAL_POSITION_INT.hdg is centidegrees, but 65535 means UNKNOWN.
    Dividing that by 100 points the map arrow at 655 deg on every frame
    before a heading estimate exists."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 6}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)

    class Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

        def get_type(self):
            return "GLOBAL_POSITION_INT"

    loop._handle_global_position(
        Msg(lat=407128000, lon=-740060000, hdg=9000))
    assert loop.telemetry()["heading_deg"] == 90.0

    loop._handle_global_position(
        Msg(lat=407128000, lon=-740060000, hdg=65535))
    assert loop.telemetry()["heading_deg"] == 90.0      # unchanged, not 655.35


def test_position_telemetry_starts_null_and_fills_in():
    """The map must show 'waiting for position' rather than centring on
    lat/lon 0,0 in the Gulf of Guinea."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 7}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)

    t = loop.telemetry()
    assert t["lat"] is None and t["lon"] is None
    assert t["home_valid"] is False
    assert loop._position() == (None, None)

    class Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

        def get_type(self):
            return "GLOBAL_POSITION_INT"

    loop._handle_global_position(
        Msg(lat=407128000, lon=-740060000, hdg=0))
    assert abs(loop.telemetry()["lat"] - 40.7128) < 1e-7
    assert abs(loop.telemetry()["lon"] + 74.0060) < 1e-7
    assert loop._position() == (loop.telemetry()["lat"],
                                loop.telemetry()["lon"])


def test_startup_params_clamp_the_mission_speed():
    """The pad does 2 m/s; PX4's default position-setpoint ceiling is 12.
    Pressing FLY must not be a step change in how the aircraft behaves."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 8}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0),
                           rate_hz=20.0, mission_speed=3.0)
    sent = {}
    loop.link.set_param = lambda name, value, ptype: sent.__setitem__(name, value)

    loop._send_startup_params()

    assert sent["MPC_XY_VEL_MAX"] == 3.0
    assert sent["COM_RCL_EXCEPT"] == offboard.COM_RCL_EXCEPT_OFFBOARD
    assert "MIS_TAKEOFF_ALT" in sent


def _fake_gpi(lat_int, lon_int, hdg=0):
    return type("M", (), {"lat": lat_int, "lon": lon_int, "hdg": hdg,
                          "get_type": lambda s: "GLOBAL_POSITION_INT"})()


def test_a_running_mission_sends_global_position_setpoints():
    """The dispatch: a target from advance() means a position setpoint, not
    the velocity one the manual path sends."""
    js = _load_server()
    port = FAKE_PX4_PORT + 9
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
        loop = js.SetpointLoop(conn, state, rate_hz=20.0)
        loop._handle_global_position(_fake_gpi(400000000, -740000000))
        loop.load_mission([[40.0010, -74.0]], 12.0)
        loop.mission.fly()
        loop.start()

        seen = _collect(px4, 1.0, kind="SET_POSITION_TARGET_GLOBAL_INT")
        assert len(seen) >= 10, f"expected >=10 position setpoints, got {len(seen)}"
        last = seen[-1]
        assert last.coordinate_frame == offboard.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        assert last.type_mask == offboard.POS_YAW_TYPE_MASK
        assert last.lat_int == 400010000
        assert abs(last.alt - 12.0) < 1e-4
    finally:
        px4.close()


def test_pausing_a_mission_hands_control_back_to_the_joystick():
    """Takeover: after a pause the very next setpoints are velocity ones
    carrying the held direction, and the stream never stops."""
    js = _load_server()
    port = FAKE_PX4_PORT + 10
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
        loop = js.SetpointLoop(conn, state, rate_hz=20.0)
        loop._handle_global_position(_fake_gpi(400000000, -740000000))
        loop.load_mission([[40.0010, -74.0]], 12.0)
        loop.mission.fly()
        loop.start()
        _collect(px4, 0.5, kind="SET_POSITION_TARGET_GLOBAL_INT")

        state.set("fwd", True)
        loop.submit("mission_pause")

        seen = _collect(px4, 1.0, kind="SET_POSITION_TARGET_LOCAL_NED")
        assert len(seen) >= 10, f"joystick did not take over: {len(seen)} sent"
        assert abs(seen[-1].vx - 2.0) < 1e-6
        assert loop.mission.status()["state"] == "PAUSED"
    finally:
        px4.close()


def test_leaving_offboard_auto_pauses_a_running_mission():
    """PX4 accepts and discards setpoints outside OFFBOARD
    (mavlink_receiver.cpp:1163). A mission left RUNNING there would look fine
    and do nothing."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 11}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)
    loop.load_mission([[40.0010, -74.0]], 12.0)
    loop.mission.fly()

    loop._note_mode("AUTO.LAND")
    assert loop.mission.status()["state"] == "PAUSED"


# --- the recorder proxy ----------------------------------------------------
#
# Recording is the one command from the page that does NOT touch MAVLink, and
# therefore the one that must not go anywhere near the setpoint thread.

class _FakeControlServer:
    """Stand-in for sim/recorder_control.py inside Kit.

    `delay` makes it hang, which is the interesting case: Kit's main thread can
    be mid-frame when a request lands.
    """

    def __init__(self, delay=0.0, status_code=200, body=None):
        self.delay = delay
        self.status_code = status_code
        self.body = body if body is not None else {"state": "idle",
                                                   "can_start": True,
                                                   "images": 0}
        self.hits = []
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def _reply(self):
                outer.hits.append((self.command, self.path))
                if outer.delay:
                    time.sleep(outer.delay)
                code = 200 if self.path.endswith("status") else outer.status_code
                payload = (outer.body if self.path.endswith("status")
                           else {"queued": "ok", "error": "refused for a reason"})
                blob = json.dumps(payload).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(blob)))
                self.end_headers()
                self.wfile.write(blob)

            do_GET = _reply
            do_POST = _reply

            def log_message(self, *a):
                pass

        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.port = self.srv.server_address[1]
        self.url = f"http://127.0.0.1:{self.port}"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def close(self):
        self.srv.shutdown()
        self.srv.server_close()


def test_record_commands_never_reach_the_setpoint_queue():
    """Design R2. Every other command routes through SetpointLoop.submit()
    because it touches MAVLink; recording does not. An HTTP call to Kit can
    block for hundreds of milliseconds, and a gap in the setpoint stream drops
    PX4 out of OFFBOARD (joystick-server.py:216-219) -- so routing RECORD
    through that thread would mean pressing it could drop the aircraft."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 12}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)

    for name in ("record", "record_start", "record_stop", "start", "stop"):
        loop.submit(name)
    assert loop.commands.empty(), (
        "a recording verb reached the setpoint queue; it must be handled on "
        "the web thread")

    # And the proxy that does handle it owns no MAVLink connection at all.
    proxy = js.RecorderProxy("http://127.0.0.1:1")
    assert not hasattr(proxy, "conn") and not hasattr(proxy, "link")


def test_a_hanging_recorder_call_does_not_gap_the_setpoint_stream():
    """R2 made concrete: Kit stalls for 3 s while the drone is flying forward.
    The setpoint stream must be entirely unaffected -- 20 Hz throughout, with
    the held velocity intact."""
    js = _load_server()
    port = FAKE_PX4_PORT + 13
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    server = _FakeControlServer(delay=3.0)
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
        state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
        js.SetpointLoop(conn, state, rate_hz=20.0).start()
        state.set("fwd", True)

        async def press_record_while_flying():
            proxy = js.RecorderProxy(server.url, timeout=5.0)
            task = proxy.submit("start")
            # The event loop must stay responsive while the call is in flight.
            await asyncio.sleep(0.2)
            assert not task.done()
            return proxy

        threading.Thread(
            target=lambda: asyncio.run(press_record_while_flying()),
            daemon=True).start()

        seen = _collect(px4, 1.5)
        assert len(seen) >= 25, (
            f"setpoint stream gapped while the recorder call was in flight: "
            f"{len(seen)} in 1.5 s")
        assert abs(seen[-1].vx - 2.0) < 1e-6, "the held velocity was lost"
    finally:
        px4.close()
        server.close()


def test_unreachable_control_server_reports_offline_not_an_error():
    """Normal before Isaac is up. The page must show a disabled button with a
    reason, not a stack trace."""
    js = _load_server()
    # Port 1 is privileged and unbound: connection refused, immediately.
    proxy = js.RecorderProxy("http://127.0.0.1:1", timeout=0.5)

    assert proxy.status()["state"] == "offline"
    assert proxy.status()["can_start"] is False

    asyncio.run(proxy.refresh())
    assert proxy.status()["state"] == "offline"

    asyncio.run(proxy.command("start"))
    assert proxy.status()["state"] == "offline"
    assert "sim" in proxy.status()["cmd_error"].lower()


def test_a_refused_command_surfaces_the_control_servers_reason():
    """507 means the disk is too full to start. The number is the whole point:
    "recording failed" tells the operator nothing they can act on."""
    js = _load_server()
    server = _FakeControlServer(status_code=507)
    try:
        proxy = js.RecorderProxy(server.url)
        asyncio.run(proxy.command("start"))
        assert proxy.status()["cmd_error"] == "refused for a reason"
    finally:
        server.close()


def test_status_poll_timeout_does_not_stall_telemetry():
    """The 5 Hz telemetry push must keep going while a status poll hangs."""
    js = _load_server()
    server = _FakeControlServer(delay=5.0)
    try:
        async def exercise():
            proxy = js.RecorderProxy(server.url, timeout=0.5)
            poller = asyncio.create_task(proxy.poll_forever())

            class FakeSock:
                def __init__(self):
                    self.frames = []

                async def send_text(self, text):
                    self.frames.append(json.loads(text))

            class FakeLoop:
                def telemetry(self):
                    return {"connected": True, "mode": "OFFBOARD"}

            sock = FakeSock()
            pusher = asyncio.create_task(
                js._push_telemetry(sock, FakeLoop(), proxy, hz=5.0))
            await asyncio.sleep(1.0)
            pusher.cancel()
            poller.cancel()
            return sock.frames

        frames = asyncio.run(exercise())
        assert len(frames) >= 4, (
            f"telemetry stalled behind the hanging status poll: "
            f"{len(frames)} frames in 1 s")
        # And every frame still carries a recorder block, offline being the
        # honest answer while the poll is timing out.
        assert all("rec" in f for f in frames)
        assert frames[-1]["rec"]["state"] == "offline"
    finally:
        server.close()
