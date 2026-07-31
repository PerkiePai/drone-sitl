"""Integration test for the setpoint thread against a fake PX4 over real UDP.

No PX4 and no Isaac Sim required. Ports are deliberately NOT 14540/14580 so
the test cannot collide with a real SITL instance running on the same box.
"""
import importlib.util
import os
import sys
import time

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
