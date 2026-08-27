#!/usr/bin/env python3
"""Web joystick -> PX4 OFFBOARD velocity control (proof of concept).

Serves web/index.html plus a WebSocket at /ws, and streams body-frame velocity
setpoints to PX4 SITL at 20 Hz. Four commands only: climb, descend, forward,
backward.

Run with Isaac Sim already playing drone_setup_px4_cesium.py:

    conda run -n drone python joystick-server.py

then open http://<box-ip>:8090/ from any device on the LAN.

Design: docs/superpowers/specs/2026-07-30-joystick-offboard-design.md
"""
import argparse
import asyncio
import json
import math
import os
import queue
import sys
import threading
import time

from pymavlink import mavutil

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import offboard  # noqa: E402
import waypoints  # noqa: E402
import agent_control as agentctl  # noqa: E402

# Uploaded control scripts land here; agent_runner.py is spawned against them.
# A module constant, not a flag -- the server still runs with a bare
# `python joystick-server.py`.
AGENT_UPLOAD_DIR = os.path.join(ROOT, "logs", "agents")
AGENT_MAX_BYTES = 256 * 1024
ARENA_RADIUS_M = 500.0
AGENT_TIME_LIMIT_S = None
os.makedirs(AGENT_UPLOAD_DIR, exist_ok=True)


def _safe_agent_name(name):
    """A base filename ending .py with no path parts, or None."""
    if not name or not name.endswith(".py"):
        return None
    if name != os.path.basename(name) or "/" in name or "\\" in name \
            or ".." in name:
        return None
    return name


class SetpointLoop(threading.Thread):
    """Sole owner of the MAVLink connection.

    Sends a velocity setpoint every tick forever -- zeros are a valid hover
    setpoint, and a gap in the stream drops PX4 out of OFFBOARD. One-shot
    commands arrive on a queue and execute on this thread so that nothing
    else ever touches `conn`.
    """

    # Mission verbs go through the same queue as arm/takeoff so that nothing
    # but the setpoint thread ever mutates flight state mid-tick.
    MISSION_COMMANDS = {"mission_fly": "fly",
                        "mission_pause": "pause",
                        "mission_clear": "clear"}

    def __init__(self, conn, state, rate_hz=20.0, takeoff_alt=5.0, warmup_s=1.0,
                 mission_speed=3.0, arrival_radius=2.0, agent_control=None):
        super().__init__(daemon=True)
        self.conn = conn
        self.state = state
        self.link = offboard.OffboardLink(conn)
        self.mission = waypoints.Mission(arrival_radius)
        # Third setpoint source, alongside manual (state) and mission. Owned
        # here so existing callers/tests that don't pass one still work.
        self.agent_control = (agent_control if agent_control is not None
                              else agentctl.AgentControl())
        self.agent_camera = "nadir"
        self.dt = 1.0 / rate_hz
        self.takeoff_alt = takeoff_alt
        self.mission_speed = mission_speed
        self.warmup_s = warmup_s
        self.commands = queue.Queue()
        self._telem_lock = threading.Lock()
        self._telem = {
            "connected": False,
            "armed": False,
            "mode": "--",
            "alt_m": 0.0,
            "vz": 0.0,
            "heading_deg": 0.0,
            # Attitude, degrees. The agent API's state.roll/pitch -- "read this
            # before trusting a frame" -- and unused by the manual UI.
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            # World-frame horizontal velocity, m/s (+N, +E). Feeds the agent
            # state.vx/vy; the manual UI shows ground speed (gs) instead.
            "vn": 0.0,
            "ve": 0.0,
            # Map feed. None until the first GLOBAL_POSITION_INT, so the page
            # can say "waiting for position" instead of centring on 0,0.
            "lat": None,
            "lon": None,
            # PX4 needs a valid home altitude to accept GLOBAL_RELATIVE_ALT
            # setpoints and returns SILENTLY without one
            # (mavlink_receiver.cpp:1107-1110). FLY is gated on this.
            "home_valid": False,
            "mission": {"state": "IDLE", "index": 0, "count": 0,
                        "dist_m": None},
            # Commanded vs measured, so "it tilts but does not move" is
            # observable rather than a guess: cmd high + actual ~0 means PX4
            # is receiving the setpoint but not achieving it.
            "cmd_vx": 0.0,
            "cmd_yaw_rate": 0.0,
            "gs": 0.0,
            # PX4 SITL runs in lockstep with Isaac, so PX4's clock IS sim time.
            # Ratio < 1 means the sim is running slower than wall clock and the
            # drone only LOOKS sluggish -- it is accelerating correctly in sim
            # seconds. Without this, slow rendering is indistinguishable from a
            # control bug.
            "sim_rate": 0.0,
            "ready_for_offboard": False,
            "streaming_s": 0.0,
        }
        self._stream_start = None
        self._params_sent = False
        self._sim_ref = None          # (px4_boot_ms, wall_monotonic) baseline

    def telemetry(self):
        with self._telem_lock:
            return dict(self._telem)

    def _position(self):
        """Current (lat, lon), either may be None. For the setpoint thread."""
        with self._telem_lock:
            return self._telem["lat"], self._telem["lon"]

    def _handle_global_position(self, msg):
        """GLOBAL_POSITION_INT -> map position and heading.

        Heading comes from here rather than ATTITUDE so the map arrow and the
        telemetry row are the same number and cannot disagree.
        """
        with self._telem_lock:
            self._telem["lat"] = msg.lat / 1e7
            self._telem["lon"] = msg.lon / 1e7
            # hdg is centidegrees 0-35999, with 65535 meaning UNKNOWN. Keep
            # the last good heading rather than reporting 655 degrees.
            if msg.hdg != 65535:
                self._telem["heading_deg"] = msg.hdg / 100.0

    def _handle_attitude(self, msg):
        """ATTITUDE carries radians; the agent API and any UI want degrees."""
        with self._telem_lock:
            self._telem["roll_deg"] = math.degrees(msg.roll)
            self._telem["pitch_deg"] = math.degrees(msg.pitch)

    def load_mission(self, points, alt_m):
        """Called from the web thread. Mission carries its own lock and
        touches no MAVLink, so this needs neither the queue nor _telem_lock."""
        self.mission.load(points, alt_m)

    def _note_mode(self, mode):
        """Record PX4's actual mode, auto-pausing a mission that has lost its
        only means of flying.

        PX4 accepts setpoints outside OFFBOARD and then discards them
        (mavlink_receiver.cpp:1163), so a mission left RUNNING after a mode
        change would report progress it is not making.
        """
        with self._telem_lock:
            self._telem["mode"] = mode
        if mode != "OFFBOARD":
            self.mission.pause()

    def submit(self, name):
        """Called from the web thread. Queue only -- never touches `conn`."""
        if (name in ("arm", "disarm", "takeoff", "land", "offboard")
                or name in self.MISSION_COMMANDS):
            self.commands.put(name)

    def _run_command(self, name):
        method = self.MISSION_COMMANDS.get(name)
        if method is not None:
            getattr(self.mission, method)()
        else:
            getattr(self.link, name)()
        print(f">>> command: {name}")

    def _send_startup_params(self):
        self.link.set_param("COM_RCL_EXCEPT", offboard.COM_RCL_EXCEPT_OFFBOARD,
                            offboard.MAV_PARAM_TYPE_INT32)
        self.link.set_param("MIS_TAKEOFF_ALT", self.takeoff_alt,
                            offboard.MAV_PARAM_TYPE_REAL32)
        # PX4's default MPC_XY_VEL_MAX is 12 m/s (mc_pos_control_params.c:412)
        # against the pad's 2 m/s. Unclamped, FLY would send the aircraft off
        # six times faster than anything the operator has seen it do.
        self.link.set_param("MPC_XY_VEL_MAX", self.mission_speed,
                            offboard.MAV_PARAM_TYPE_REAL32)
        self._params_sent = True
        print(f">>> params: COM_RCL_EXCEPT=4 (offboard exempt from RC-loss "
              f"failsafe), MIS_TAKEOFF_ALT={self.takeoff_alt}, "
              f"MPC_XY_VEL_MAX={self.mission_speed}")

    def _drain_mavlink(self):
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return
            kind = msg.get_type()
            if kind == "HEARTBEAT":
                self.link.bind_target(msg)
                armed = bool(msg.base_mode & offboard.MAV_MODE_FLAG_SAFETY_ARMED)
                mode = offboard.decode_px4_mode(msg.custom_mode)
                with self._telem_lock:
                    self._telem["connected"] = True
                    self._telem["armed"] = armed
                self._note_mode(mode)
                if not self._params_sent:
                    self._send_startup_params()
            elif kind == "LOCAL_POSITION_NED":
                with self._telem_lock:
                    self._telem["alt_m"] = -msg.z    # NED down -> altitude up
                    self._telem["vz"] = -msg.vz
                    self._telem["gs"] = math.hypot(msg.vx, msg.vy)
                # Sim clock vs wall clock, measured over a rolling 3 s window.
                wall = time.monotonic()
                if self._sim_ref is None:
                    self._sim_ref = (msg.time_boot_ms, wall)
                else:
                    boot0, wall0 = self._sim_ref
                    d_wall = wall - wall0
                    if d_wall >= 3.0:
                        d_sim = (msg.time_boot_ms - boot0) / 1000.0
                        with self._telem_lock:
                            self._telem["sim_rate"] = d_sim / d_wall
                        self._sim_ref = (msg.time_boot_ms, wall)
            elif kind == "GLOBAL_POSITION_INT":
                self._handle_global_position(msg)
                with self._telem_lock:
                    self._telem["vn"] = msg.vx / 100.0     # cm/s -> m/s
                    self._telem["ve"] = msg.vy / 100.0
            elif kind == "ATTITUDE":
                self._handle_attitude(msg)
            elif kind == "HOME_POSITION":
                with self._telem_lock:
                    self._telem["home_valid"] = True

    def run(self):
        next_tick = time.monotonic()
        while True:
            self._drain_mavlink()
            while True:
                try:
                    self._run_command(self.commands.get_nowait())
                except queue.Empty:
                    break
            # The whole autonomous/manual split. A target from advance() means
            # fly the route; None means the operator has it. Either way exactly
            # one setpoint goes out this tick -- a gap drops PX4 out of
            # OFFBOARD.
            lat, lon = self._position()
            target = self.mission.advance(lat, lon)
            if target is not None:
                wp_lat, wp_lon, wp_alt, wp_yaw = target
                self.link.send_position_global(wp_lat, wp_lon, wp_alt, wp_yaw)
                vx, yaw_rate = 0.0, 0.0
            elif (agent_cmd := self.agent_control.command()) is not None:
                # An uploaded script has the aircraft: its Velocity /
                # VelocityWorld, or a zeroed hover once its watchdog expires.
                # Wins over the manual pad -- a manual press kills the agent
                # up in the /ws handler, so this branch is the backstop.
                akind, a0, a1, a2, yaw_rate = agent_cmd
                if akind == "world":
                    self.link.send_velocity_world(a0, a1, a2, yaw_rate)
                else:
                    self.link.send_velocity(a0, a1, a2, yaw_rate)
                vx = math.hypot(a0, a1)
            else:
                vx, vy, vz, yaw_rate = self.state.command()
                self.link.send_velocity(vx, vy, vz, yaw_rate)

            if self._stream_start is None:
                self._stream_start = time.monotonic()
            streaming_s = time.monotonic() - self._stream_start
            # Taken before _telem_lock, never inside it: Mission has its own
            # lock and nesting the two would introduce a cycle.
            mission_status = self.mission.status()
            with self._telem_lock:
                self._telem["cmd_vx"] = vx
                self._telem["cmd_yaw_rate"] = yaw_rate
                self._telem["streaming_s"] = streaming_s
                self._telem["mission"] = mission_status
                self._telem["ready_for_offboard"] = (
                    streaming_s >= self.warmup_s and self._telem["connected"])

            next_tick += self.dt
            nap = next_tick - time.monotonic()
            if nap > 0:
                time.sleep(nap)
            else:
                next_tick = time.monotonic()   # fell behind; resync


async def _push_telemetry(sock, loop_thread, agent_run=None, hz=5.0):
    try:
        while True:
            t = loop_thread.telemetry()
            if agent_run is not None:
                # The agent run state machine is stepped on exactly one
                # cadence -- this one, the /ws pusher's. The /agent/control
                # pusher passes agent_run=None.
                agent_run.tick()
                t["agent"] = agent_run.snapshot()
            await sock.send_text(json.dumps(t))
            await asyncio.sleep(1.0 / hz)
    except Exception:
        pass          # socket closed; the /ws handler cleans up


def build_app(loop_thread, state, video_port, mission_speed):
    from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
    from fastapi.responses import JSONResponse
    from fastapi.staticfiles import StaticFiles

    app = FastAPI()

    @app.get("/config")
    def config():
        return JSONResponse({"video_port": video_port,
                             "mission_speed": mission_speed})

    @app.post("/agent/upload")
    async def agent_upload(request: Request):
        # Raw body, not multipart -- keeps python-multipart out of the deps.
        # The browser reads the .py with FileReader and POSTs the text.
        name = request.query_params.get("name", "")
        safe = _safe_agent_name(name)
        if safe is None:
            return JSONResponse({"detail": "name must be a bare *.py filename"},
                                status_code=400)
        body = await request.body()
        if len(body) > AGENT_MAX_BYTES:
            return JSONResponse({"detail": f"file over {AGENT_MAX_BYTES} bytes"},
                                status_code=400)
        try:
            text = body.decode("utf-8")
        except UnicodeDecodeError:
            return JSONResponse({"detail": "file is not valid UTF-8 text"},
                                status_code=400)
        stored = f"{safe[:-3]}-{time.strftime('%Y%m%d-%H%M%S')}.py"
        with open(os.path.join(AGENT_UPLOAD_DIR, stored), "w") as fh:
            fh.write(text)
        return JSONResponse({"stored": stored})

    @app.get("/agent/list")
    def agent_list():
        try:
            entries = [e for e in os.scandir(AGENT_UPLOAD_DIR)
                       if e.is_file() and e.name.endswith(".py")]
        except FileNotFoundError:
            entries = []
        entries.sort(key=lambda e: e.stat().st_mtime, reverse=True)
        return JSONResponse({"files": [e.name for e in entries]})

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        state.clear()
        pusher = asyncio.create_task(_push_telemetry(sock, loop_thread))
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                kind = msg.get("type")
                if kind == "axis":
                    state.set(msg["dir"], bool(msg.get("pressed")))
                    # An EDGE, not a level. Polling held() at 20 Hz would miss
                    # a press-release inside one tick -- the drone would twitch
                    # and the mission would keep flying. Only presses pause: if
                    # releases did too, the mission would re-pause forever and
                    # RESUME could never take.
                    if msg.get("pressed"):
                        loop_thread.submit("mission_pause")
                elif kind == "cmd":
                    loop_thread.submit(msg["name"])
                elif kind == "mission":
                    action = msg.get("action")
                    if action == "fly":
                        # Overloaded on purpose: points present means FLY (new
                        # route from waypoint 1), points absent means RESUME
                        # (continue the loaded one). Start and continue are the
                        # same transition, so they are the same action.
                        if "points" in msg:
                            loop_thread.load_mission(msg["points"],
                                                     msg.get("alt", 0.0))
                        loop_thread.submit("mission_fly")
                    elif action == "pause":
                        loop_thread.submit("mission_pause")
                    elif action == "clear":
                        loop_thread.submit("mission_clear")
                elif kind == "ping":
                    state.touch()
        except (WebSocketDisconnect, json.JSONDecodeError, KeyError, ValueError):
            pass
        finally:
            pusher.cancel()
            # A dropped socket must not latch the last commanded velocity.
            state.clear()

    @app.websocket("/agent/control")
    async def agent_control_ws(sock: WebSocket):
        """agent_runner.py connects here. One socket carries both directions:
        the server pushes telemetry, the child sends translated Commands."""
        await sock.accept()
        t0 = loop_thread.telemetry()
        origin = ([t0["lat"], t0["lon"]] if t0.get("home_valid")
                  and t0.get("lat") is not None else None)
        await sock.send_text(json.dumps({
            "type": "arena", "origin": origin,
            "radius_m": ARENA_RADIUS_M, "time_limit": AGENT_TIME_LIMIT_S}))
        pusher = asyncio.create_task(_push_telemetry(sock, loop_thread))
        ac = loop_thread.agent_control
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                kind = msg.get("type")
                if kind == "velocity":
                    ac.set_velocity_body(msg["forward"], msg["right"],
                                         msg["up"], msg["yaw_rate"])
                elif kind == "velocity_world":
                    ac.set_velocity_world(msg["north"], msg["east"],
                                          msg["up"], msg["yaw_rate"])
                elif kind == "goto":
                    # `speed` is accepted but the server clamps MPC_XY_VEL_MAX
                    # to --mission-speed at startup; per-leg speed is not wired.
                    loop_thread.load_mission([[msg["lat"], msg["lon"]]],
                                             msg["alt"])
                    loop_thread.submit("mission_fly")
                elif kind == "route":
                    loop_thread.load_mission(msg["points"], msg["alt"])
                    loop_thread.submit("mission_fly")
                elif kind == "hold":
                    loop_thread.submit("mission_clear")
                    ac.hold()
                elif kind == "camera":
                    loop_thread.agent_camera = msg.get("camera", "nadir")
        except (WebSocketDisconnect, json.JSONDecodeError, KeyError, ValueError):
            pass
        finally:
            pusher.cancel()
            # Deliberately NOT clearing agent_control here: a control-socket
            # blip must not drop a latched Hold. The /ws stop path and the
            # velocity watchdog own cleanup.

    # Mounted last so /config and /ws above take priority for those paths;
    # this serves index.html at "/" plus css/js/vendor as plain static files.
    app.mount("/", StaticFiles(directory=os.path.join(ROOT, "web"), html=True),
              name="web")

    return app


def main():
    ap = argparse.ArgumentParser(
        description="Web joystick -> PX4 OFFBOARD velocity control")
    ap.add_argument("--mavlink", default="udpin:0.0.0.0:14540",
                    help="PX4 offboard link. MUST be udpin: PX4 binds 14580 "
                         "and sends TO 14540, so udpout never receives.")
    ap.add_argument("--port", type=int, default=8090, help="web UI port")
    ap.add_argument("--video-port", type=int, default=8080,
                    help="Isaac MJPEG port from drone_setup_px4_cesium.py")
    ap.add_argument("--speed-fwd", type=float, default=2.0, help="m/s")
    ap.add_argument("--speed-up", type=float, default=1.0, help="m/s")
    ap.add_argument("--yaw-rate", type=float, default=offboard.DEFAULT_YAW_RATE_DPS,
                    help="turn rate for the left/right buttons, deg/s")
    ap.add_argument("--takeoff-alt", type=float, default=5.0, help="m")
    ap.add_argument("--mission-speed", type=float, default=3.0,
                    help="waypoint cruise, m/s. Clamps PX4's MPC_XY_VEL_MAX, "
                         "whose 12 m/s default dwarfs the pad's 2 m/s")
    ap.add_argument("--arrival-radius", type=float, default=2.0,
                    help="metres; a waypoint counts as reached inside this")
    ap.add_argument("--watchdog", type=float, default=0.5,
                    help="seconds of silence before velocity is forced to zero")
    ap.add_argument("--rate", type=float, default=20.0, help="setpoint Hz")
    ap.add_argument("--offboard-warmup", type=float, default=1.0,
                    help="seconds of streaming before OFFBOARD is offered")
    args = ap.parse_args()

    import uvicorn

    conn = mavutil.mavlink_connection(args.mavlink)
    state = offboard.CommandState(args.speed_fwd, args.speed_up, args.watchdog,
                                  args.yaw_rate)
    loop_thread = SetpointLoop(conn, state, args.rate, args.takeoff_alt,
                               args.offboard_warmup, args.mission_speed,
                               args.arrival_radius)
    loop_thread.start()

    print(f">>> MAVLink offboard link: {args.mavlink}")
    print(f">>> setpoint loop at {args.rate:.0f} Hz "
          f"({args.speed_fwd} m/s fwd, {args.speed_up} m/s climb, "
          f"{args.yaw_rate:.0f} deg/s turn)")
    print(f">>> open http://<box-ip>:{args.port}/")
    uvicorn.run(build_app(loop_thread, state, args.video_port,
                          args.mission_speed),
                host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
