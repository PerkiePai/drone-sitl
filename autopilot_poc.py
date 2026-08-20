#!/usr/bin/env python3
"""Scripted autonomous flight over joystick-server.py's existing /ws protocol.

Sends exactly the messages web/js/controls.js sends -- axis press/release,
cmd, ping -- so this is a proof that an external script can fly the drone
through the same unmodified interface a human's browser uses. No server or
protocol changes.

Run joystick-server.py and Isaac Sim first (see RUN-WEBSITE.md
section 6 for the ARM -> TAKEOFF -> OFFBOARD -> fly -> LAND sequence this
script replays), then:

    conda run -n drone python autopilot_poc.py
"""
import argparse
import asyncio
import json
import time

import websockets

PING_INTERVAL_S = 0.15
"""Matches web/js/controls.js's keepalive. The server's watchdog
(offboard.py CommandState) zeroes velocity after 0.5 s of silence."""

CONFIRM_SIM_S = 3.0
"""Default confirmation deadline, in SIM seconds -- matches the wait the web
UI itself uses for ARM/TAKEOFF/OFFBOARD (RUN-WEBSITE.md section 6).
Converted to a wall-clock budget via the measured sim_rate in _wait_for,
never used as a wall-clock number directly: PX4 SITL runs in lockstep with
Isaac, so a fixed wall deadline would misjudge a slow-rendering sim as a
refused command."""


class Autopilot:
    """One flight, one WebSocket. Owns the telemetry-driven wait/hold logic
    that a human operator does by eye against the page's telemetry row."""

    def __init__(self, ws):
        self.ws = ws
        self.telemetry = {}
        self.sim_rate = 1.0     # unmeasured yet; server also starts at 0.0

    async def _recv_loop(self):
        async for raw in self.ws:
            self.telemetry = json.loads(raw)
            rate = self.telemetry.get("sim_rate")
            if rate:
                self.sim_rate = rate

    async def _wait_for(self, predicate, description, timeout_sim_s=CONFIRM_SIM_S):
        """Poll telemetry until predicate(telemetry) is true or time out.

        See CONFIRM_SIM_S for why the deadline is stated in sim seconds and
        scaled by sim_rate rather than being a fixed wall-clock number.
        """
        deadline = time.monotonic() + timeout_sim_s / max(self.sim_rate, 0.05)
        while time.monotonic() < deadline:
            if predicate(self.telemetry):
                return
            await asyncio.sleep(0.1)
        raise TimeoutError(f"{description}: no confirmation from PX4 within "
                           f"{timeout_sim_s:.0f}s sim time (command refused?)")

    async def _send(self, msg):
        await self.ws.send(json.dumps(msg))

    async def _cmd(self, name):
        await self._send({"type": "cmd", "name": name})
        print(f">>> sent: {name}")

    async def _hold(self, direction, seconds):
        """Press `direction`, keep it alive for `seconds` of WALL time, release.

        Wall time on purpose: a human holding a button is timing against a
        real clock, and the server's silence watchdog is wall-clock too
        (offboard.py CommandState.watchdog_s) -- there is no sim-time
        equivalent to convert against here, unlike _wait_for's PX4
        confirmations.
        """
        await self._send({"type": "axis", "dir": direction, "pressed": True})
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            await asyncio.sleep(PING_INTERVAL_S)
            await self._send({"type": "ping"})
        await self._send({"type": "axis", "dir": direction, "pressed": False})

    async def fly_sequence(self):
        await self._wait_for(lambda t: t.get("connected"), "startup")
        print(">>> connected to joystick-server.py")

        await self._cmd("arm")
        await self._wait_for(lambda t: t.get("armed"), "ARM")
        print(">>> ARM confirmed")

        await self._cmd("takeoff")
        await self._wait_for(lambda t: t.get("mode") == "AUTO.TAKEOFF",
                             "TAKEOFF mode")
        # "wait for it to stop climbing" (RUN-WEBSITE.md section 6.2):
        # vz settles near zero once the target altitude is reached.
        await self._wait_for(
            lambda t: t.get("alt_m", 0.0) > 1.0 and abs(t.get("vz", 99.0)) < 0.1,
            "TAKEOFF settle", timeout_sim_s=10.0)
        print(f">>> TAKEOFF complete, alt={self.telemetry.get('alt_m'):.1f} m")

        await self._wait_for(lambda t: t.get("ready_for_offboard"),
                             "OFFBOARD readiness")
        await self._cmd("offboard")
        await self._wait_for(lambda t: t.get("mode") == "OFFBOARD", "OFFBOARD")
        print(">>> OFFBOARD -- the joystick is live")

        print(">>> forward, 5.0 s")
        await self._hold("fwd", 5.0)
        print(">>> turn right, 2.0 s")
        await self._hold("yaw_right", 2.0)
        print(">>> climb, 2.0 s")
        await self._hold("up", 2.0)

        await self._cmd("land")
        await self._wait_for(lambda t: not t.get("armed"), "LAND/disarm",
                             timeout_sim_s=20.0)
        print(">>> landed and disarmed")


async def _main_async(host, port):
    uri = f"ws://{host}:{port}/ws"
    print(f">>> connecting to {uri}")
    async with websockets.connect(uri) as ws:
        pilot = Autopilot(ws)
        recv_task = asyncio.create_task(pilot._recv_loop())
        try:
            await pilot.fly_sequence()
        finally:
            recv_task.cancel()


def main():
    ap = argparse.ArgumentParser(
        description="Fly a scripted demo through joystick-server.py's "
                    "existing /ws protocol, exactly as a browser would.")
    ap.add_argument("--host", default="127.0.0.1",
                    help="joystick-server.py host")
    ap.add_argument("--port", type=int, default=8090,
                    help="joystick-server.py port")
    args = ap.parse_args()
    asyncio.run(_main_async(args.host, args.port))


if __name__ == "__main__":
    main()
