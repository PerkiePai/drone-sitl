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


class SetpointLoop(threading.Thread):
    """Sole owner of the MAVLink connection.

    Sends a velocity setpoint every tick forever -- zeros are a valid hover
    setpoint, and a gap in the stream drops PX4 out of OFFBOARD. One-shot
    commands arrive on a queue and execute on this thread so that nothing
    else ever touches `conn`.
    """

    def __init__(self, conn, state, rate_hz=20.0, takeoff_alt=5.0, warmup_s=1.0):
        super().__init__(daemon=True)
        self.conn = conn
        self.state = state
        self.link = offboard.OffboardLink(conn)
        self.dt = 1.0 / rate_hz
        self.takeoff_alt = takeoff_alt
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
            "ready_for_offboard": False,
            "streaming_s": 0.0,
        }
        self._stream_start = None
        self._params_sent = False

    def telemetry(self):
        with self._telem_lock:
            return dict(self._telem)

    def submit(self, name):
        """Called from the web thread. Queue only -- never touches `conn`."""
        if name in ("arm", "disarm", "takeoff", "land", "offboard"):
            self.commands.put(name)

    def _run_command(self, name):
        getattr(self.link, name)()
        print(f">>> command: {name}")

    def _send_startup_params(self):
        self.link.set_param("COM_RCL_EXCEPT", offboard.COM_RCL_EXCEPT_OFFBOARD,
                            offboard.MAV_PARAM_TYPE_INT32)
        self.link.set_param("MIS_TAKEOFF_ALT", self.takeoff_alt,
                            offboard.MAV_PARAM_TYPE_REAL32)
        self._params_sent = True
        print(f">>> params: COM_RCL_EXCEPT=4 (offboard exempt from RC-loss "
              f"failsafe), MIS_TAKEOFF_ALT={self.takeoff_alt}")

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
                    self._telem["mode"] = mode
                if not self._params_sent:
                    self._send_startup_params()
            elif kind == "LOCAL_POSITION_NED":
                with self._telem_lock:
                    self._telem["alt_m"] = -msg.z    # NED down -> altitude up
                    self._telem["vz"] = -msg.vz
            elif kind == "ATTITUDE":
                with self._telem_lock:
                    self._telem["heading_deg"] = (
                        math.degrees(msg.yaw) + 360.0) % 360.0

    def run(self):
        next_tick = time.monotonic()
        while True:
            self._drain_mavlink()
            while True:
                try:
                    self._run_command(self.commands.get_nowait())
                except queue.Empty:
                    break
            vx, vy, vz = self.state.velocity()
            self.link.send_velocity(vx, vy, vz)

            if self._stream_start is None:
                self._stream_start = time.monotonic()
            streaming_s = time.monotonic() - self._stream_start
            with self._telem_lock:
                self._telem["streaming_s"] = streaming_s
                self._telem["ready_for_offboard"] = (
                    streaming_s >= self.warmup_s and self._telem["connected"])

            next_tick += self.dt
            nap = next_tick - time.monotonic()
            if nap > 0:
                time.sleep(nap)
            else:
                next_tick = time.monotonic()   # fell behind; resync
