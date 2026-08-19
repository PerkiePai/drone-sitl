#!/usr/bin/env python3
"""Drives a multi-candidate MPC_XY_* gain campaign over joystick-server.py's
existing /ws protocol -- arm/takeoff/gps_denied/gps_restore/land/disarm plus
one new command, set_param. One continuous session: Isaac Sim, PX4 and
joystick-server.py all stay up for the whole campaign; no PX4 reboot between
candidates, since none of the four gains are @reboot_required. See
docs/superpowers/specs/2026-08-14-mpc-xy-gain-sweep-design.md.

Run from the `drone` conda env against an ALREADY-RUNNING server (this script
does not start Isaac or joystick-server.py itself, matching how
sim/save-ulog.sh doesn't either):

    conda run -n drone python sim/mpc_gain_sweep.py \\
        --candidates screen --campaign-name 20260815-screen
    conda run -n drone python sim/mpc_gain_sweep.py \\
        --candidates confirm:more_damping --campaign-name 20260815-confirm
"""
import argparse
import asyncio
import json
import os
import sys
import time

import websockets

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
from offboard import MAV_PARAM_TYPE_REAL32  # noqa: E402

GAIN_PARAMS = ("MPC_XY_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC",
              "MPC_XY_VEL_D_ACC")

# D2's candidate set. All four gains are floats (mc_pos_control_params.c:270,
# 282,295,307), so every value here goes on the wire as MAV_PARAM_TYPE_REAL32
# -- never the INT32 bit-pattern path offboard.set_param also handles.
CANDIDATES = {
    "baseline":     (0.95, 1.8, 0.40, 0.2),
    "more_damping": (0.95, 1.8, 0.40, 0.5),
    "gentler_p":    (0.50, 1.2, 0.40, 0.2),
    "low_integral": (0.95, 1.8, 0.05, 0.2),
    "gentle_combo": (0.50, 1.2, 0.05, 0.5),
}
CANDIDATE_ORDER = ("baseline", "more_damping", "gentler_p", "low_integral",
                   "gentle_combo")
"""Flight order for a screening campaign. analyze_gain_sweep.py slices
run.csv's phase-2 segments in this same order -- the sidecar records it
explicitly rather than making the analyzer guess."""

SCREEN_HOLD_S = 70.0
CONFIRM_HOLD_S = 180.0
SETTLE_S = 20.0
OFFBOARD_TIMEOUT_S = 30.0
CLIMB_TIMEOUT_S = 90.0
"""AUTO.TAKEOFF -> AUTO.LOITER, separate from OFFBOARD_TIMEOUT_S: confirmed
live that a real climb to MIS_TAKEOFF_ALT sits close enough to 30 sim-s that
whether it lands just under or just over is closer to a coin flip than a
real failure -- 2 of 5 candidates climbed fine inside 30s, 3 of 5 didn't, in
the same campaign. Generous margin here costs nothing but wall-clock time on
a candidate that would have climbed fine anyway."""

ABORT_EXCURSION_M = 400.0
"""D5: well past the worst peak seen so far, ~268 m (SESSION.md, run 6)."""
ABORT_ALT_DROP_M = 20.0
"""D5: a genuine descent, not the flat-ground_z-plane sign-convention trap
SESSION.md documents for readings near the ground."""


def should_abort(excursion_m, alt_m, alt_at_cut_m):
    """D5's screening abort check, pure so it needs no socket to test.

    `alt_m` is height above the launch point (telemetry's own convention --
    joystick-server.py computes it as `-msg.z`), not raw NED down, so a
    reading near ground_z is not mistaken for a strike (SESSION.md's "first
    read misread this as a crash"). Any missing value means the telemetry
    has not confirmed a real reading yet, so it never triggers an abort.
    """
    if excursion_m is not None and excursion_m > ABORT_EXCURSION_M:
        return True
    if (alt_m is not None and alt_at_cut_m is not None
            and (alt_at_cut_m - alt_m) > ABORT_ALT_DROP_M):
        return True
    return False


class Campaign:
    """Drives every candidate in `plan` (a list of (name, hold_s)) over one
    open websocket connection. `sidecar` records what actually flew, in
    order -- analyze_gain_sweep.py trusts that order rather than re-deriving
    it from timestamps."""

    def __init__(self, ws, campaign_name):
        self.ws = ws
        self.sidecar = {"campaign": campaign_name, "candidates": []}

    async def _recv_telem(self):
        return json.loads(await self.ws.recv())

    async def _cmd(self, name):
        await self.ws.send(json.dumps({"type": "cmd", "name": name}))

    async def _set_gain(self, param_name, value):
        await self.ws.send(json.dumps({
            "type": "cmd", "name": "set_param", "param": param_name,
            "value": value, "param_type": MAV_PARAM_TYPE_REAL32}))

    async def _wait_for(self, predicate, timeout_s, sim_time=True):
        """Poll telemetry (pushed at 5 Hz) until predicate(telem) is True or
        timeout_s elapses. sim_time=True (the default -- everything this
        driver waits on is a flight-time bar) measures elapsed time on PX4's
        own clock (telem["sim_s"]) rather than wall time: sim_rate wanders
        0.17-0.65 in this project (SESSION.md), and a wall-clock wait would
        hold for the wrong amount of simulated flight time. Returns the last
        telemetry frame seen either way -- callers check what actually
        happened rather than trusting the predicate held.
        """
        start_wall = time.monotonic()
        start_sim = None
        telem = None
        while True:
            telem = await self._recv_telem()
            if sim_time:
                if telem.get("sim_s") is None:
                    continue
                if start_sim is None:
                    start_sim = telem["sim_s"]
                elapsed = telem["sim_s"] - start_sim
            else:
                elapsed = time.monotonic() - start_wall
            if predicate(telem):
                return telem
            if elapsed >= timeout_s:
                return telem

    async def _recover(self):
        """Best-effort return to a clean disarmed state after a candidate
        fails early. Without this, the next candidate's arm/takeoff/offboard
        stacks onto an aircraft still airborne from the failed one -- exactly
        what turned five failed OFFBOARD attempts into an unbounded climb the
        first time this driver ran live. gps_restore is deliberately
        ungated/idempotent (harmless even if GNSS was never cut), matching
        the same land-then-disarm shape as a normal candidate's own ending.
        """
        print(">>> recovering: land and disarm before the next candidate")
        await self._cmd("gps_restore")
        await self._cmd("land")
        await self._wait_for(lambda t: t.get("mode") == "AUTO.LAND", 10.0,
                             sim_time=False)
        await self._cmd("disarm")

    async def fly_candidate(self, name, hold_s):
        gains = CANDIDATES[name]
        record = {"name": name, "gains": dict(zip(GAIN_PARAMS, gains)),
                  "hold_s": hold_s, "status": "flying"}
        self.sidecar["candidates"].append(record)
        print(f">>> candidate {name}: {record['gains']}")

        for param_name, value in zip(GAIN_PARAMS, gains):
            await self._set_gain(param_name, value)

        await self._cmd("arm")
        telem = await self._wait_for(lambda t: t.get("armed") is True, 10.0,
                                     sim_time=False)
        if not telem.get("armed"):
            # Never armed -- nothing airborne to land, so no _recover() call.
            record["status"] = "failed_to_arm"
            print(f">>> candidate {name}: {record['status']}")
            return record

        await self._cmd("takeoff")
        # Wait for the climb to actually finish (PX4 auto-transitions
        # AUTO.TAKEOFF -> AUTO.LOITER on reaching MIS_TAKEOFF_ALT) before
        # switching to OFFBOARD. Sending "offboard" immediately after
        # "takeoff" races AUTO.TAKEOFF's own climb: OFFBOARD's zero-velocity
        # setpoint (nothing commands forward/up motion) wins the race and
        # the aircraft never leaves the ground -- confirmed live: mode
        # reached OFFBOARD while px4_d stayed pinned at ground level for the
        # entire remaining hold.
        telem = await self._wait_for(lambda t: t.get("mode") == "AUTO.LOITER",
                                     CLIMB_TIMEOUT_S)
        if telem.get("mode") != "AUTO.LOITER":
            record["status"] = "failed_to_climb"
            print(f">>> candidate {name}: {record['status']}")
            await self._recover()
            return record

        await self._cmd("offboard")
        telem = await self._wait_for(lambda t: t.get("mode") == "OFFBOARD",
                                     OFFBOARD_TIMEOUT_S)
        if telem.get("mode") != "OFFBOARD":
            record["status"] = "failed_to_offboard"
            print(f">>> candidate {name}: {record['status']}")
            await self._recover()
            return record

        telem = await self._wait_for(lambda t: t.get("vision_fusing"),
                                     OFFBOARD_TIMEOUT_S)
        if not telem.get("vision_fusing"):
            record["status"] = "failed_to_fuse"
            print(f">>> candidate {name}: {record['status']}")
            await self._recover()
            return record
        await self._wait_for(lambda t: False, SETTLE_S)   # just settle

        await self._cmd("gps_denied")
        telem = await self._wait_for(lambda t: t.get("gps_denied"), 5.0,
                                     sim_time=False)
        if not telem.get("gps_denied"):
            await asyncio.sleep(5.0)
            await self._cmd("gps_denied")
            telem = await self._wait_for(lambda t: t.get("gps_denied"), 5.0,
                                         sim_time=False)
            if not telem.get("gps_denied"):
                record["status"] = "failed_to_cut"
                print(f">>> candidate {name}: {record['status']}")
                await self._recover()
                return record

        alt_at_cut = telem.get("alt_m")
        aborted = False

        def _hold_predicate(t):
            nonlocal aborted
            vio = t.get("vio") or {}
            if should_abort(vio.get("excursion_m"), t.get("alt_m"),
                            alt_at_cut):
                aborted = True
                return True
            return False

        await self._wait_for(_hold_predicate, hold_s)

        await self._cmd("gps_restore")
        await self._wait_for(lambda t: not t.get("gps_denied"), 10.0,
                             sim_time=False)
        await self._cmd("land")
        await self._wait_for(lambda t: t.get("mode") == "AUTO.LAND", 10.0,
                             sim_time=False)
        await self._cmd("disarm")

        record["status"] = "aborted" if aborted else "flown"
        print(f">>> candidate {name}: {record['status']}")
        return record

    def save_sidecar(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.sidecar, f, indent=2)


def _plan_for(candidates_arg):
    if candidates_arg == "screen":
        return [(name, SCREEN_HOLD_S) for name in CANDIDATE_ORDER]
    if candidates_arg.startswith("confirm:"):
        name = candidates_arg.split(":", 1)[1]
        if name not in CANDIDATES:
            raise SystemExit(f"unknown candidate {name!r}; choose from "
                             f"{sorted(CANDIDATES)}")
        return [(name, CONFIRM_HOLD_S)]
    raise SystemExit(f"--candidates must be 'screen' or 'confirm:<name>', "
                     f"got {candidates_arg!r}")


async def run_campaign(uri, candidates_arg, campaign_name):
    plan = _plan_for(candidates_arg)
    async with websockets.connect(uri) as ws:
        campaign = Campaign(ws, campaign_name)
        for name, hold_s in plan:
            await campaign.fly_candidate(name, hold_s)
        sidecar_path = os.path.join(
            ROOT, "logs", campaign_name, f"campaign_{campaign_name}.json")
        campaign.save_sidecar(sidecar_path)
        print(f">>> sidecar written to {sidecar_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uri", default="ws://127.0.0.1:8090/ws")
    ap.add_argument("--candidates", required=True,
                    help="'screen' (all 5 at 70 s) or 'confirm:<name>' "
                         "(one candidate at 180 s)")
    ap.add_argument("--campaign-name", required=True,
                    help="names the sidecar JSON; pass the SAME value as "
                         "joystick-server.py's --run-name at server-start "
                         "time so run.csv and the sidecar line up")
    args = ap.parse_args()
    asyncio.run(run_campaign(args.uri, args.candidates, args.campaign_name))


if __name__ == "__main__":
    main()
