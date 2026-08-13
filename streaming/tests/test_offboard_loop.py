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

import pytest
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


# --- GPS-denied flight on vision (plan Task 6) ------------------------------

import vision_bridge  # noqa: E402


class _FakeVision:
    """Stands in for VisionSubscriber: the setpoint thread calls latest() and
    reads _last_msg, so a test needs nothing more than those two.

    `inliers` defaults to a healthy count because most tests are about
    something else; the blind-camera case passes 0 explicitly."""

    def __init__(self, pose=None, received_at=None, inliers=600):
        self.pose = pose
        self.received_at = received_at
        self.calls = 0
        self._last_msg = {"n_inliers": inliers, "drift_m": 0.4, "fps": 8.8}

    def latest(self):
        self.calls += 1
        return self.pose, self.received_at


def _pose(x=1.0, y=2.0, z=3.0):
    return vision_bridge.VisionPose(ts_ns=1_000_000_000, x=x, y=y, z=z,
                                    roll=0.0, pitch=0.0, yaw=0.0)


ORIGIN = (13.66156872, 100.298235, 0.0)


def _vision_loop(js, port_offset, pose=None, received_at=None, inliers=600,
                 px4_ned=(0.0, 0.0, -49.0), px4_yaw=0.0):
    """A loop with vision, and with PX4's own pose already seen.

    `px4_ned`/`px4_yaw` stand in for LOCAL_POSITION_NED and ATTITUDE, which PX4
    streams continuously and which are therefore present long before any phase
    transition. They are what the vision frame is aligned onto; pass
    `px4_ned=None` to model the pathological case where they are missing.
    """
    conn = mavutil.mavlink_connection(
        f"udpout:127.0.0.1:{FAKE_PX4_PORT + port_offset}")
    vision = _FakeVision(pose, received_at, inliers)
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0,
                           vision=vision, vision_origin=ORIGIN)
    loop._px4_ned = px4_ned
    loop._px4_yaw = px4_yaw
    return conn, loop, vision


def test_vision_pose_is_forwarded_once_per_tick():
    """One VISION_POSITION_ESTIMATE per setpoint tick -- the same steady-stream
    discipline the velocity setpoint already follows."""
    js = _load_server()
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{FAKE_PX4_PORT + 20}")
    try:
        _, loop, _ = _vision_loop(js, 20, _pose(), received_at=time.monotonic())
        loop.start()
        seen = _collect(px4, 1.0, kind="VISION_POSITION_ESTIMATE")
        assert len(seen) >= 10, f"expected >=10 VPE in 1 s, got {len(seen)}"
        assert abs(seen[-1].x - 2.0) < 1e-6      # ENU y=2 -> NED north
        assert abs(seen[-1].y - 1.0) < 1e-6      # ENU x=1 -> NED east
        assert abs(seen[-1].z + 3.0) < 1e-6      # ENU up=3 -> NED down=-3
    finally:
        px4.close()


def test_setpoint_still_goes_out_when_vision_is_stale():
    """Vision going stale must NOT interrupt the setpoint stream: a gap there
    drops PX4 out of OFFBOARD, turning a degraded estimate into a loss of
    control."""
    js = _load_server()
    px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{FAKE_PX4_PORT + 21}")
    try:
        # received 60 s ago -- far past DEFAULT_MAX_AGE_S
        _, loop, _ = _vision_loop(js, 21, _pose(),
                                  received_at=time.monotonic() - 60.0)
        loop.start()
        setpoints = _collect(px4, 1.0)
        assert len(setpoints) >= 10, "stale vision must not gap the setpoints"
        vpe = _collect(px4, 0.5, kind="VISION_POSITION_ESTIMATE")
        assert not vpe, "a stale pose must be dropped, not repeated"
    finally:
        px4.close()


def test_ekf2_params_are_not_sent_without_the_vision_flag():
    """EKF2_GPS_CTRL=0 disables all GNSS fusion. It must never be a default."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 22}")
    loop = js.SetpointLoop(conn, offboard.CommandState(2.0, 1.0), rate_hz=20.0)
    sent = {}
    loop.link.set_param = lambda name, value, ptype: sent.__setitem__(name, value)

    loop._send_startup_params()

    assert not [n for n in sent if n.startswith("EKF2_")], sent


def test_startup_reboots_px4_before_sending_anything_else():
    """EKF2_HGT_REF is @reboot_required, so the first startup pass sets it and
    restarts PX4 -- and sends nothing else, because PX4 is about to drop off
    the link and params posted into that gap are simply lost.

    Skipping this is not a cosmetic bug: with GNSS cut and the height reference
    still on GPS, the aircraft descends without bound (SESSION.md 2026-08-11)."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 23, _pose(), received_at=time.monotonic())
    order = []
    loop.link.set_param = lambda name, value, ptype: order.append(("param", name))
    loop.link.reboot_autopilot = lambda: order.append(("reboot", None))
    loop.link.conn.mav.set_gps_global_origin_send = (
        lambda *a, **k: order.append(("origin", a)))

    loop._send_startup_params()

    kinds = [k for k, _ in order]
    assert ("param", "EKF2_HGT_REF") in order, order
    assert "reboot" in kinds, order
    assert kinds.index("reboot") == len(kinds) - 1, (
        "the reboot must be the LAST thing sent", order)
    assert "origin" not in kinds, ("nothing may follow the reboot", order)


def test_the_reboot_happens_once_not_on_every_restart():
    """_check_px4_restart re-runs the startup params on any heartbeat gap, and
    a reboot IS a heartbeat gap. Without a latch that is an infinite loop of
    reboots."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 26, _pose(), received_at=time.monotonic())
    reboots = []
    loop.link.set_param = lambda name, value, ptype: None
    loop.link.reboot_autopilot = lambda: reboots.append(1)

    loop._send_startup_params()          # phase 0: reboots
    loop._send_startup_params()          # PX4 back: must NOT reboot again
    loop._send_startup_params()

    assert len(reboots) == 1, reboots


def test_gps_origin_is_sent_before_the_first_vision_estimate():
    """PX4 cannot place a local estimate without an anchor, so ordering is the
    assertion."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 23, _pose(), received_at=time.monotonic())
    loop.link.set_param = lambda name, value, ptype: None
    loop.link.reboot_autopilot = lambda: None
    loop._send_startup_params()          # phase 0: the reboot pass

    order = []
    loop.link.set_param = lambda name, value, ptype: order.append(("param", name))
    loop.link.conn.mav.set_gps_global_origin_send = (
        lambda *a, **k: order.append(("origin", a)))
    loop.vision_sender.conn.mav.vision_position_estimate_send = (
        lambda *a, **k: order.append(("vpe", a)))

    loop._send_startup_params()          # phase 1: the origin
    loop._send_vision(time.monotonic())

    kinds = [k for k, _ in order]
    assert "origin" in kinds, kinds
    assert "vpe" in kinds, kinds
    assert kinds.index("origin") < kinds.index("vpe")
    # Fusion is not switched ON here -- startup asserts it OFF and waits for a
    # camera that can see.
    assert ("param", "EKF2_EV_CTRL") in order


def test_startup_asserts_gps_flight_rather_than_leaving_params_alone():
    """PX4 params PERSIST across runs and reboots, so a previous GPS-denied
    session leaves EKF2_GPS_CTRL=0 and EKF2_EV_CTRL=9 saved. Merely not setting
    them would start this run GPS-denied on the pad with vision fused over a
    blind camera. Startup has to assert the state it wants."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 27, _pose(), received_at=time.monotonic())
    sent = {}
    loop.link.set_param = lambda name, value, ptype: sent.__setitem__(name, value)
    loop.link.reboot_autopilot = lambda: None

    loop._send_startup_params()
    loop._send_startup_params()

    assert sent["EKF2_GPS_CTRL"] == vision_bridge.EKF2_GPS_CTRL_DEFAULT
    assert sent["EKF2_GPS_CTRL"] != 0, "startup must never cut GNSS"
    assert sent["EKF2_EV_CTRL"] == 0, "fusion must start OFF, not merely unset"
    assert loop.telemetry()["gps_denied"] is False
    assert loop.telemetry()["vision_fusing"] is False


def test_gps_denied_is_refused_without_a_fresh_estimate():
    """Cutting GNSS with a dead estimator leaves EKF2 with no position source
    at all -- the exact descent the phase split exists to prevent."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 28, _pose(), received_at=None)
    sent = []
    loop.link.set_param = lambda name, value, ptype: sent.append(name)

    loop._run_command("gps_denied")

    assert sent == [], sent
    assert loop.telemetry()["gps_denied"] is False


def test_a_blind_camera_is_fresh_but_never_fused():
    """The pad case: the estimator publishes promptly and confidently with
    ZERO tracked features. Fusing that costs an arming refusal --
    `Preflight Fail: Yaw estimate error` -- so freshness alone must not be
    enough to switch fusion on."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 30, _pose(), received_at=time.monotonic(),
                              inliers=0)
    sent = []
    loop.link.set_param = lambda name, value, ptype: sent.append(name)
    loop._params_sent = True

    loop._maybe_start_fusing_vision(time.monotonic())

    assert sent == [], sent
    assert loop.telemetry()["vision_fusing"] is False
    assert loop._vio_status(time.monotonic())["fresh"] is True, (
        "the point of this test is that it IS fresh")


def test_vision_starts_fusing_once_the_camera_can_see():
    js = _load_server()
    _, loop, _ = _vision_loop(js, 31, _pose(), received_at=time.monotonic())
    sent = []
    loop.link.set_param = lambda name, value, ptype: sent.append(name)
    loop._params_sent = True

    loop._maybe_start_fusing_vision(time.monotonic())
    loop._maybe_start_fusing_vision(time.monotonic())   # latched, not repeated

    assert sent.count("EKF2_EV_CTRL") == 1, sent
    assert "EKF2_GPS_CTRL" not in sent, "phase 1 must not cut GNSS"
    assert loop.telemetry()["vision_fusing"] is True


def test_gps_denied_is_refused_before_vision_is_fusing():
    """Ordering: EKF2 has to be fusing vision BEFORE GNSS is taken away, or
    the cut lands on a source it is not using."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 32, _pose(), received_at=time.monotonic())
    sent = []
    loop.link.set_param = lambda name, value, ptype: sent.append(name)

    loop._run_command("gps_denied")          # never started fusing

    assert sent == [], sent
    assert loop.telemetry()["gps_denied"] is False


def _local_pos(time_boot_ms, z=-10.0, x=0.0, y=0.0):
    # x/y are the aircraft's north/east: the loop reads them for the frame
    # alignment, so a fake without them is not a LOCAL_POSITION_NED.
    return type("M", (), {
        "time_boot_ms": time_boot_ms, "x": x, "y": y, "z": z,
        "vz": 0.0, "vx": 0.0, "vy": 0.0,
        "get_type": lambda s: "LOCAL_POSITION_NED"})()


def test_a_slow_sim_does_not_make_good_estimates_look_stale():
    """The staleness budget is in SIM seconds, so a sim running at 0.11 must not
    turn a perfectly good estimate into a dropped one.

    This is the crash of 2026-08-11: `dropped_stale` ran 15 -> 179 as `sim_rate`
    fell 0.56 -> 0.11, starving EKF2 of the only position source it had left."""
    js = _load_server()
    _, loop, vision = _vision_loop(js, 35, _pose(), received_at=time.monotonic())
    loop.link.set_param = lambda name, value, ptype: None

    # PX4 sim clock advances 0.1 s while NINE wall seconds pass -- sim_rate 0.011.
    loop._px4_sim_s = 100.0
    assert loop._send_vision(time.monotonic())          # first pose, age 0
    loop._px4_sim_s = 100.1
    assert loop._send_vision(time.monotonic() + 9.0), (
        "a 0.1 s sim-time gap must not be judged stale because 9 wall seconds "
        "elapsed")
    assert loop.vision_sender.dropped_stale == 0


def test_a_genuinely_frozen_estimator_is_still_caught():
    """The guard must survive being put on the right clock: an estimator that
    stops producing while SIM time advances is still dropped, because repeating
    a frozen position is worse than having none."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 36, _pose(), received_at=time.monotonic())
    loop.link.set_param = lambda name, value, ptype: None

    loop._px4_sim_s = 100.0
    assert loop._send_vision(time.monotonic())
    # Same pose (same ts_ns), sim time marches on well past the budget.
    loop._px4_sim_s = 100.0 + loop.vision_sender.max_age_s + 0.5
    assert not loop._send_vision(time.monotonic())
    assert loop.vision_sender.dropped_stale == 1


def test_the_page_and_the_sender_agree_on_freshness():
    """`fresh` gates the GNSS cut and the drop decision gates the aircraft's
    position source. If they were computed from different clocks the page could
    offer a cut that the sender was already refusing to feed."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 37, _pose(), received_at=time.monotonic())
    loop.link.set_param = lambda name, value, ptype: None

    loop._px4_sim_s = 50.0
    loop._send_vision(time.monotonic())
    assert loop._vio_status(time.monotonic())["fresh"] is True

    loop._px4_sim_s = 50.0 + loop.vision_sender.max_age_s + 0.5
    sent = loop._send_vision(time.monotonic())
    assert sent is False
    assert loop._vio_status(time.monotonic())["fresh"] is False


def test_a_heartbeat_gap_does_not_switch_vision_back_off():
    """_send_startup_params is the RECOVERY path as well as the startup one --
    _check_px4_restart re-runs it on any heartbeat gap. Asserting GPS flight
    unconditionally there turns fusion off mid-flight, which is what happened
    live 2026-08-11: fusion engaged at altitude and a gap quietly undid it."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 33, _pose(), received_at=time.monotonic())
    loop._params_sent = True
    loop._vision_rebooted = True         # phase 0 already done
    loop.link.set_param = lambda name, value, ptype: None
    loop.link.reboot_autopilot = lambda: None
    loop._maybe_start_fusing_vision(time.monotonic())
    assert loop._vision_fusing

    sent = {}
    loop.link.set_param = lambda name, value, ptype: sent.__setitem__(name, value)
    loop._send_startup_params()          # as a heartbeat gap would

    assert sent["EKF2_EV_CTRL"] == 9, "fusion must be re-asserted, not undone"


def test_a_heartbeat_gap_keeps_gnss_cut_once_gps_denied():
    """Same hazard in the other direction: re-asserting GPS flight after the
    operator cut GNSS would silently hand the aircraft back to GPS."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 34, _pose(), received_at=time.monotonic())
    loop._params_sent = True
    loop._vision_rebooted = True         # phase 0 already done
    loop.link.set_param = lambda name, value, ptype: None
    loop.link.reboot_autopilot = lambda: None
    loop._maybe_start_fusing_vision(time.monotonic())
    loop._run_command("gps_denied")
    assert loop.telemetry()["gps_denied"] is True

    sent = {}
    loop.link.set_param = lambda name, value, ptype: sent.__setitem__(name, value)
    loop._send_startup_params()

    assert sent["EKF2_GPS_CTRL"] == 0, "GNSS must stay cut across a gap"
    assert sent["EKF2_EV_CTRL"] == 9


def test_the_restart_gap_allows_for_sim_time_heartbeats():
    """PX4 heartbeats in SIM time while this threshold is WALL time. At the
    ~0.55 sim rate this box runs, a 1 Hz heartbeat is ~1.8 s apart, so a 3 s
    threshold is barely one and a half beats and trips on ordinary jitter."""
    js = _load_server()
    assert js.PX4_RESTART_GAP_S >= 5 * 1.8


def test_gps_denied_cuts_gnss_once_vision_is_fusing():
    js = _load_server()
    _, loop, _ = _vision_loop(js, 29, _pose(), received_at=time.monotonic())
    loop._params_sent = True
    loop.link.set_param = lambda name, value, ptype: None
    loop._maybe_start_fusing_vision(time.monotonic())

    sent = []
    loop.link.set_param = lambda name, value, ptype: sent.append(name)
    loop._run_command("gps_denied")

    assert "EKF2_GPS_CTRL" in sent, sent
    assert loop.telemetry()["gps_denied"] is True


def test_the_handover_realigns_the_vision_frame_onto_px4():
    """The 2026-08-12 divergence. The estimator had drifted 23-37 m by the time
    GNSS was cut, EKF2 was handed that frame unchanged, and the aircraft flew
    at the discrepancy -- 60 -> 318 -> 1195 -> 2544 m. After the cut the vision
    stream must report where PX4 believes it is, not where the estimator does."""
    js = _load_server()
    # Vision says 30 m east / 20 m north of an origin PX4 puts itself at (5, -3).
    _, loop, _ = _vision_loop(js, 38, _pose(x=30.0, y=20.0, z=49.0),
                              received_at=time.monotonic(),
                              px4_ned=(5.0, -3.0, -49.0), px4_yaw=0.2)
    loop._params_sent = True
    loop.link.set_param = lambda name, value, ptype: None

    loop._maybe_start_fusing_vision(time.monotonic())
    loop._run_command("gps_denied")
    assert loop.telemetry()["gps_denied"] is True

    n, e, _, _, _, yaw = loop.vision_sender.alignment.to_px4_ned(
        loop.vision.pose)
    assert (n, e) == pytest.approx((5.0, -3.0))
    assert yaw == pytest.approx(0.2)


def test_the_handover_reports_the_error_it_closed():
    """The gap is the whole point, so it has to be visible in flight rather
    than inferred afterwards from a diverging track."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 39, _pose(x=0.0, y=0.0, z=49.0),
                              received_at=time.monotonic(),
                              px4_ned=(30.0, 40.0, -49.0))
    loop._params_sent = True
    loop.link.set_param = lambda name, value, ptype: None

    loop._maybe_start_fusing_vision(time.monotonic())

    vio = loop._vio_status(time.monotonic())
    assert vio["realigned"] == 1
    assert vio["align_m"] == pytest.approx(50.0)


def test_the_frame_is_realigned_at_both_phase_transitions():
    """Fusion start and the GNSS cut are separated by the climb, and the climb
    is where flow-odom drifts worst. Aligning only once would let the whole
    climb's drift back into the handover."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 40, _pose(), received_at=time.monotonic())
    loop._params_sent = True
    loop.link.set_param = lambda name, value, ptype: None

    loop._maybe_start_fusing_vision(time.monotonic())
    assert loop.vision_sender.realigned == 1
    loop._run_command("gps_denied")
    assert loop.vision_sender.realigned == 2


def test_the_frame_is_not_realigned_every_tick():
    """Re-solving the transform continuously would feed EKF2 its own estimate
    back as an independent measurement: innovations would sit at zero by
    construction and a broken estimator would look perfect right up until GNSS
    was cut. Phase 1b exists to prove the source, which requires it to stay
    independent between the transitions."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 41, _pose(), received_at=time.monotonic())
    loop._params_sent = True
    loop.link.set_param = lambda name, value, ptype: None
    loop._maybe_start_fusing_vision(time.monotonic())

    for _ in range(20):
        loop._send_vision(time.monotonic())

    assert loop.vision_sender.realigned == 1


def test_fusion_waits_for_a_px4_pose_to_align_onto():
    """EKF2 must never see one unaligned EV sample. Costs nothing in practice:
    LOCAL_POSITION_NED and ATTITUDE stream far faster than the camera clears
    the inlier gate."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 42, _pose(), received_at=time.monotonic(),
                              px4_ned=None)
    loop._params_sent = True
    sent = []
    loop.link.set_param = lambda name, value, ptype: sent.append(name)

    loop._maybe_start_fusing_vision(time.monotonic())

    assert sent == [], sent
    assert loop.telemetry()["vision_fusing"] is False


def test_gps_denied_is_refused_without_a_pose_to_align_onto():
    """Cutting GNSS unaligned is the failure this all exists to prevent, so a
    missing PX4 pose refuses the cut rather than proceeding on the raw frame."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 43, _pose(), received_at=time.monotonic())
    loop._params_sent = True
    loop.link.set_param = lambda name, value, ptype: None
    loop._maybe_start_fusing_vision(time.monotonic())

    loop._px4_yaw = None                 # ATTITUDE stops arriving
    sent = []
    loop.link.set_param = lambda name, value, ptype: sent.append(name)
    loop._run_command("gps_denied")

    assert "EKF2_GPS_CTRL" not in sent, sent
    assert loop.telemetry()["gps_denied"] is False


def test_gps_origin_is_resent_after_a_px4_restart():
    """A rebooted PX4 forgets the origin and the map silently stops updating."""
    js = _load_server()
    _, loop, _ = _vision_loop(js, 24, _pose(), received_at=time.monotonic())
    origins = []
    loop.link.set_param = lambda name, value, ptype: None
    loop.link.reboot_autopilot = lambda: None
    loop.link.conn.mav.set_gps_global_origin_send = (
        lambda *a, **k: origins.append(a))

    loop._send_startup_params()          # phase 0: the reboot pass, no origin
    loop._send_startup_params()
    assert len(origins) == 1

    # PX4 goes quiet for longer than the restart threshold, then comes back.
    loop._last_heartbeat = time.monotonic() - (js.PX4_RESTART_GAP_S + 1.0)
    loop._check_px4_restart(time.monotonic())
    assert not loop._params_sent, "a heartbeat gap must re-arm the param send"

    loop._send_startup_params()
    assert len(origins) == 2, "the origin must be re-sent after a restart"


def test_zmq_thread_never_touches_the_mavlink_connection():
    """The sole-owner contract (offboard.py:196-198): the subscriber holds a
    pose slot and nothing else. It is never handed the connection, so it
    cannot touch it from its own thread even by mistake."""
    js = _load_server()
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT + 25}")
    sub = js.VisionSubscriber("tcp://127.0.0.1:15599")
    try:
        assert conn not in vars(sub).values()
        assert not any(hasattr(v, "mav") for v in vars(sub).values())

        pose = _pose()
        done = threading.Event()

        def writer():                       # stands in for the ZMQ thread
            sub._store(pose, 123.0)
            done.set()

        threading.Thread(target=writer, daemon=True).start()
        assert done.wait(2.0)
        got, at = sub.latest()
        assert got is pose and at == 123.0
    finally:
        sub.close()
