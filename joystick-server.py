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
import csv
import json
import math
import os
import queue
import sys
import threading
import time
import urllib.error
import urllib.request

from pymavlink import mavutil

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "streaming"))
import offboard  # noqa: E402
import vision_bridge  # noqa: E402
import waypoints  # noqa: E402

ESTIMATOR_SCRIPT = "pipeline-streaming.py"
"""The estimator this server brings up itself, so the stack is one command.

Isaac Sim stays separate and always will -- `vio-streamer.py` runs inside its
physics callback, not as a process anyone here could spawn.
"""

DEFAULT_SITE = "bangkok-survey-040"
"""Falls back to the launcher's own default (sim/launch-sitl.sh:69).

`SITL_SITE` is exported by that script into ISAAC's environment, not into the
shell this server is usually started from, so leaning on it alone means typing
--site every run for a value that has exactly one possible answer today
(sim/sites.py:120).
"""

VISION_MIN_INLIERS = 100
"""Tracked features below which the vision estimate is not worth fusing.

Not a tuning knob so much as a blind-camera detector. At the pad the nadir
camera renders flat black (below the Cesium tile surface) and the estimator
still publishes at full rate with 0 inliers -- fresh, confident and empty.
Fusing that costs an arming refusal: `Preflight Fail: Yaw estimate error`.

Healthy flight sits at ~600 (the detector's cap) and the climb passes through
44 -> 304 -> 440 -> 600, so anything in the low hundreds separates the two
cases with room to spare. The estimator's own floor for tracking at all is 30
(pipeline-streaming.py --min-track).
"""

SETTLE_SPEED_MS = 0.05
"""Speed below which the airframe counts as at rest, m/s.

Measured on the pad with the sim idle: `gs` sits at 0.00-0.01 and `vz` at
0.00, so 0.05 clears physics noise with room to spare while still catching an
aircraft that is genuinely still dropping onto the collision plane.
"""

SETTLE_S = 3.0
"""How long the airframe must stay at rest before phase 0 reboots PX4, in SIM
SECONDS.

Sim, not wall, for the reason this codebase has now learned four times
(DEFAULT_MAX_AGE_S, PX4_RESTART_GAP_S, the VPE ageing): the aircraft settles in
simulated seconds, and at sim_rate 0.4 a wall-clock budget would be two and a
half times longer than intended.

The drone spawns at `spawn_z` and falls ~0.44 m onto `ground_z`
(sim/sites.py) -- about 0.3 s of free fall plus damping. 3 s is generous
against that.
"""

PX4_RESTART_GAP_S = 10.0
"""Heartbeat silence that means PX4 restarted rather than merely lagged.

A restarted PX4 has forgotten the GPS origin, and with GNSS off nothing else
will ever set it again -- the map marker and every GLOBAL_RELATIVE_ALT setpoint
just silently stop working. Re-arming the param send on a gap is what makes
recovery automatic instead of a puzzling half-dead session.

**Measured in WALL time; PX4 heartbeats in SIM time.** That is why this is 10 s
and not the 3 s it used to be. PX4 SITL is lockstepped to Isaac, so its 1 Hz
heartbeat arrives every 1/sim_rate seconds of wall clock -- at the ~0.55 this
box runs with the VIO streamer up, that is ~1.8 s per beat, and 3 s is barely
one and a half beats. Observed 2026-08-11: routine jitter tripped it repeatedly
and re-ran the startup params mid-flight, which under the phased vision profile
silently switched EKF2 back off vision. 10 s is >5 beats even at 0.55.
"""


class VisionSubscriber(threading.Thread):
    """Receives `vio` estimates over ZMQ into a lock-guarded slot.

    Deliberately NOT given the MAVLink connection. offboard.py:196-198 makes
    the setpoint thread the sole owner of `conn`, and the cheapest way to keep
    that true is for this thread to have no way to reach it -- it holds a pose
    and a timestamp, and the setpoint thread comes and takes them.
    """

    def __init__(self, endpoint, topic=None):
        super().__init__(daemon=True)
        self.endpoint = endpoint
        self.topic = topic
        self._lock = threading.Lock()
        self._pose = None
        self._received_at = None
        self._last_msg = None
        self._stop = threading.Event()
        self.received = 0

    def _store(self, pose, received_at, msg=None):
        with self._lock:
            self._pose = pose
            self._received_at = received_at
            self._last_msg = msg
            self.received += 1

    def latest(self):
        """(VisionPose|None, monotonic time it arrived|None). Any thread."""
        with self._lock:
            return self._pose, self._received_at

    def close(self):
        self._stop.set()

    def run(self):
        import zmq
        import zmq_proto

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        # Small receive queue: a backlog here would hand the setpoint thread a
        # pose from seconds ago, and VisionPositionSender would then drop it as
        # stale anyway. Better never to queue it.
        sock.setsockopt(zmq.RCVHWM, 10)
        sock.setsockopt(zmq.RCVTIMEO, 250)
        sock.connect(self.endpoint)
        sock.subscribe(self.topic if self.topic is not None
                       else zmq_proto.TOPIC_VIO)
        print(f">>> vision SUB {self.endpoint}")
        try:
            while not self._stop.is_set():
                try:
                    parts = sock.recv_multipart()
                except zmq.Again:
                    continue
                except Exception:
                    break
                try:
                    _, msg = zmq_proto.unpack(parts)
                    pose = vision_bridge.VisionPose(
                        ts_ns=int(msg["ts_ns"]),
                        x=float(msg["x"]), y=float(msg["y"]), z=float(msg["z"]),
                        roll=float(msg["roll"]), pitch=float(msg["pitch"]),
                        yaw=float(msg["yaw"]))
                except (KeyError, TypeError, ValueError):
                    continue        # a malformed estimate is not worth flying on
                self._store(pose, time.monotonic(), msg)
        finally:
            sock.close(linger=0)


class RunCSV:
    """One row per setpoint tick under --vision: sim time, PX4's own pose,
    ground truth, the raw vision estimate, aircraft excursion, estimator
    drift, frame alignment and vision health -- Task 9's classification
    flight (ADR-0002) reads this back after the fact, since nothing computed
    any of it before now. `phase` follows CONTEXT.md's numbering.

    Flushed every row, not buffered: a run ending in a crash is exactly the
    case this exists to survive (ADR-0003, which also covers why the ulog
    saved by sim/save-ulog.sh needs a matching --run-name to land beside this).
    """

    FIELDS = ("sim_s", "phase",
              "px4_n", "px4_e", "px4_d", "px4_yaw",
              "gt_x", "gt_y", "gt_z",
              "vio_x", "vio_y", "vio_z", "vio_yaw",
              "excursion_m", "drift_m", "align_m", "realigned",
              "n_inliers", "fresh", "dropped_stale", "sim_rate")

    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._fh = open(path, "w", newline="")
        self._w = csv.writer(self._fh)
        self._w.writerow(self.FIELDS)
        self._fh.flush()

    def write(self, row):
        self._w.writerow("" if row.get(f) is None else row.get(f)
                         for f in self.FIELDS)
        self._fh.flush()

    def close(self):
        self._fh.close()


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
                 mission_speed=3.0, arrival_radius=2.0,
                 vision=None, vision_origin=None, csv_path=None):
        super().__init__(daemon=True)
        self.conn = conn
        self.state = state
        self.link = offboard.OffboardLink(conn)
        # Task 9 instrumentation: only meaningful under --vision, since every
        # field but sim_s/phase comes off the vision telemetry.
        self._csv = RunCSV(csv_path) if csv_path is not None else None
        # Vision is opt-in end to end: no source means no EKF2 param changes,
        # no origin, and no VPE stream. Disabling GNSS fusion is never a
        # side effect of starting the server.
        self.vision = vision
        self.vision_origin = vision_origin
        self.vision_sender = (vision_bridge.VisionPositionSender(conn)
                              if vision is not None else None)
        self._last_heartbeat = None
        self.mission = waypoints.Mission(arrival_radius)
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
            # PX4's own clock (time_boot_ms / 1000), which under lockstep IS
            # sim time. None until the first LOCAL_POSITION_NED. A websocket
            # client (Task 10's gain-sweep driver) times its holds against
            # this, never against wall time -- see that file's _wait_for.
            "sim_s": None,
            # None when the server was started without --vision, so the page
            # can hide the row entirely rather than render a dead one.
            "vio": None,
            # False until `gps_denied` is commanded and accepted. Under
            # --vision the aircraft still flies on GNSS until then, and the
            # difference matters enough to be visible rather than inferred.
            "gps_denied": False,
            # True once EKF2 is fusing vision alongside GNSS. The page gates
            # the GPS-denied control on this, so the button cannot be offered
            # before the step it depends on has happened.
            "vision_fusing": False,
        }
        self._stream_start = None
        self._params_sent = False
        # Latched, not per-connection: the reboot must happen once. A PX4
        # restart re-runs _send_startup_params (see _check_px4_restart), and
        # without this latch that would reboot it again, forever.
        self._vision_rebooted = False
        # Phase 1 latch. Vision is fused only once the camera can actually see
        # (_maybe_start_fusing_vision), never at startup over a blind one.
        self._vision_fusing = False
        # Mirrors telemetry["gps_denied"], but owned by the setpoint thread so
        # the re-send path can read the phase without taking the telemetry lock.
        self._gps_denied = False
        self._sim_ref = None          # (px4_boot_ms, wall_monotonic) baseline
        # PX4's own clock, which under lockstep IS sim time. Used to age vision
        # estimates in the same seconds the aircraft actually flies in.
        self._px4_sim_s = None
        self._vision_seen_ts = None    # pose.ts_ns of the newest pose seen
        self._vision_seen_at = None    # clock reading when it first appeared
        self._vision_age_s = None      # computed once per tick by _send_vision
        # PX4's own local-NED pose, which is what the vision frame is aligned
        # ONTO at each phase transition (_realign_vision_frame). Position and
        # yaw arrive in different messages and neither is useful alone, so both
        # stay None until their message has been seen at least once.
        self._px4_ned = None           # (north, east, down), LOCAL_POSITION_NED
        self._px4_yaw = None           # radians, ATTITUDE
        self._px4_vel = None           # (vx, vy, vz) m/s, LOCAL_POSITION_NED
        # Sim-clock reading when the airframe first looked at rest, or None if
        # it is moving. Phase 0 waits on this -- see _px4_at_rest.
        self._at_rest_since = None
        self._settle_logged = False
        # Ground truth at the moment of the GNSS cut -- "where it was
        # commanded to hold" (CONTEXT.md, aircraft excursion). None until a cut
        # has actually been taken, or if no gt topic reached the estimator at
        # that instant; either way excursion_m stays None rather than being
        # computed against a stale or fabricated anchor.
        self._hold_point_gt = None

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
                or name in ("gps_denied", "gps_restore")
                or name in self.MISSION_COMMANDS):
            self.commands.put(name)

    def submit_set_param(self, name, value, param_type):
        """Called from the web thread. Queue only -- never touches `conn`.

        A separate method rather than routing through `submit(name)`: that
        one is a bare-string allowlist gate and every existing caller (the
        page, the mission verbs) relies on it staying that shape. set_param
        carries a payload, so it gets its own front door and its own tuple
        shape on the queue instead of overloading `submit`'s contract.
        """
        self.commands.put(("set_param", name, value, param_type))

    def _run_command(self, item):
        if isinstance(item, tuple):
            _, name, value, param_type = item
            self.link.set_param(name, value, param_type)
            print(f">>> set_param: {name}={value}")
            return
        name = item
        if name == "gps_denied":
            self._go_gps_denied()
            return
        if name == "gps_restore":
            self._restore_gnss()
            return
        method = self.MISSION_COMMANDS.get(name)
        if method is not None:
            getattr(self.mission, method)()
        else:
            getattr(self.link, name)()
        print(f">>> command: {name}")

    def _healthy_vision(self, now):
        """The VIO block if the estimate is worth fusing, else None.

        `fresh` alone is not enough. On the pad the nadir camera sits below the
        Cesium tile surface and renders flat black, so the estimator publishes
        promptly and confidently with ZERO tracked features -- fresh, and
        meaningless. Inlier count is what tells those apart.
        """
        health = self._vio_status(now)
        if not health or not health.get("fresh"):
            return None
        if (health.get("n_inliers") or 0) < VISION_MIN_INLIERS:
            return None
        return health

    def _px4_pose(self):
        """PX4's own local-NED pose, or None if either half is missing.

        Setpoint thread only -- both halves are written by _drain_mavlink on
        that same thread, so this needs no lock.
        """
        if self._px4_ned is None or self._px4_yaw is None:
            return None
        north, east, down = self._px4_ned
        return vision_bridge.Px4Pose(north=north, east=east, down=down,
                                     yaw=self._px4_yaw)

    def _realign_vision_frame(self, why):
        """Pin the vision frame onto PX4's at a phase transition. True if taken.

        The estimator drifts, PX4's estimate does not follow it, and nothing
        closed the gap between them: at the 2026-08-12 handover EKF2 was simply
        handed a frame 23-37 m from where it believed it was, and it flew at
        the discrepancy. Re-solving the transform here means each phase starts
        from zero error rather than from whatever the climb accumulated.

        Deliberately NOT done every tick -- see vision_bridge.align_to_px4 for
        why that would destroy the one property phase 1b exists to provide.

        A PX4 restart resets its local frame and would invalidate the
        alignment, but the only restart in this system is phase 0's, which
        happens before any alignment is taken (_send_startup_params).
        """
        pose, _ = self.vision.latest()
        px4 = self._px4_pose()
        if pose is None or px4 is None:
            return False
        align = self.vision_sender.realign(pose, px4)
        print(f">>> vision: frame realigned at {why} -- "
              f"closed {align.offset_m():.1f} m and "
              f"{math.degrees(align.yaw):+.1f} deg against PX4")
        return True

    def _maybe_start_fusing_vision(self, now):
        """Phase 1: begin fusing vision alongside GNSS, once it can see.

        Deferred rather than applied at startup because turning EV fusion on
        over a blind camera is actively harmful: PX4 refuses to arm with
        `Preflight Fail: Yaw estimate error / heading estimate not stable`,
        the vision yaw being a constant derived from no features at all.
        Observed live 2026-08-11, and only visible once INT32 params started
        arriving intact -- before that EKF2_EV_CTRL never took and nothing was
        ever fused.

        So the aircraft takes off on GPS with vision merely streaming, and the
        moment the camera has a real view this fires exactly once. By the time
        the operator can sensibly ask for GNSS to be cut, vision has been
        fusing alongside it for the whole climb.
        """
        if self.vision is None or self._vision_fusing or not self._params_sent:
            return
        health = self._healthy_vision(now)
        if health is None:
            return
        # Align BEFORE the params, never after: EKF2 must never see a single
        # unaligned EV sample. Waiting for PX4's own pose costs nothing --
        # LOCAL_POSITION_NED and ATTITUDE both stream far faster than the
        # camera clears the inlier gate -- and fusing a frame we could not
        # align is the failure this whole change exists to remove.
        if not self._realign_vision_frame("fusion start"):
            return
        self._vision_fusing = True
        vision_bridge.apply_ekf2_fusion_params(self.link)
        with self._telem_lock:
            self._telem["vision_fusing"] = True
        print(f">>> vision: FUSING alongside GNSS "
              f"({health.get('n_inliers')} inliers). `gps_denied` is now "
              f"available.")

    def _go_gps_denied(self):
        """Phase 2: cut GNSS and fly on vision alone. Setpoint thread only.

        Refused unless vision is actually being fused and still healthy. The
        whole point of phasing the param set is that this step is taken with
        the vision source already proven in flight; letting it through on a
        dead estimator would just reproduce the descent it was split up to
        prevent.
        """
        if self.vision is None:
            print("*** gps_denied: server was not started with --vision. ***")
            return
        if not self._vision_fusing:
            print("*** gps_denied: REFUSED -- EKF2 is not fusing vision yet. "
                  "Climb until the camera can see ground. ***")
            return
        health = self._healthy_vision(time.monotonic())
        if health is None:
            print("*** gps_denied: REFUSED -- no fresh vision estimate with "
                  "enough inliers. Cutting GNSS now would leave EKF2 with no "
                  "position source at all. ***")
            return
        # The handover itself. Between fusion start and now the aircraft has
        # climbed, and the climb is where flow-odom drifts worst -- it solves
        # translation against a ground plane at barometric height, which is
        # worst conditioned exactly when altitude is changing fast (0.36-0.61 m
        # hovering, 23-37 m after a 3 m/s climb to the same altitude). So the
        # frame is pinned again here, and GNSS goes away with the two estimates
        # agreeing to the metre.
        if not self._realign_vision_frame("the GNSS cut"):
            print("*** gps_denied: REFUSED -- no PX4 local pose to align the "
                  "vision frame onto. Cutting GNSS now would hand EKF2 a frame "
                  "tens of metres from where it believes it is. ***")
            return
        # The excursion anchor (ADR-0004): "where it was commanded to hold" is
        # ground truth AT THIS INSTANT, not PX4's estimate -- scoring PX4
        # against its own position would show a rock-steady hover while the
        # aircraft flew away, which is exactly the failure 8.6 needs to catch.
        last = getattr(self.vision, "_last_msg", None) or {}
        gx, gy = last.get("gt_x"), last.get("gt_y")
        self._hold_point_gt = (gx, gy) if gx is not None and gy is not None else None
        if self._hold_point_gt is None:
            print("*** gps_denied: no ground truth in the latest estimate -- "
                  "aircraft excursion will not be computable this flight. ***")
        vision_bridge.apply_ekf2_gps_denied_params(self.link)
        self._gps_denied = True
        with self._telem_lock:
            self._telem["gps_denied"] = True
        print(f">>> GPS-DENIED: GNSS fusion off, flying on vision "
              f"({health.get('n_inliers')} inliers, "
              f"drift {health.get('drift_m')}).")

    def _px4_at_rest(self):
        """True once the airframe has held still for SETTLE_S sim seconds.

        Setpoint thread only -- reads state written by _drain_mavlink on that
        same thread.

        Phase 0 reboots PX4, and EKF2 picks its height reference as it comes
        back up. Rebooting into an aircraft that is still dropping onto the
        collision plane bakes that transient into the reference: the barometer
        innovation then sits ~1.76 m from truth for the rest of the session,
        against the 1.5 m limit at PreFlightChecker.hpp:199 -- a COMPILE-TIME
        constant no parameter can relax -- and PX4 refuses to arm with
        `Preflight Fail: height estimate not stable`. Observed 2026-08-13; the
        message is misleading, because the height is perfectly steady, it is
        just steadily wrong. Rebooting again on a settled sim took the
        innovation to 0.91 m and cleared it.

        This became reachable when the launcher collapsed to one command: the
        server now starts the instant PX4 does, where before it was typed by
        hand a minute later, so the race was there all along and nobody could
        lose it.

        Being at rest is a PROXY for the barometer having settled, not a proof
        of it. It targets the transient actually observed. If this resurfaces
        on an aircraft that is provably still, the thing to gate on is the baro
        innovation itself rather than a longer wait here.
        """
        if self._px4_vel is None or self._px4_sim_s is None:
            return False        # nothing has told us where it is yet
        now = self._px4_sim_s
        if math.sqrt(sum(v * v for v in self._px4_vel)) > SETTLE_SPEED_MS:
            self._at_rest_since = None
            return False
        # `now < since` means PX4's clock restarted under us. Treat that as the
        # settle never having happened rather than as a huge elapsed time.
        if self._at_rest_since is None or now < self._at_rest_since:
            self._at_rest_since = now
        return (now - self._at_rest_since) >= SETTLE_S

    def _wait_for_settle(self):
        """True if phase 0 should hold off this heartbeat. Setpoint thread only.

        Logged once each way rather than per heartbeat: this runs at PX4's 1 Hz
        and the wait is normally a few seconds.
        """
        if self._px4_at_rest():
            if self._settle_logged:
                print(">>> vision: airframe settled -- running phase 0 now")
                self._settle_logged = False
            return False
        if not self._settle_logged:
            self._settle_logged = True
            speed = (None if self._px4_vel is None
                     else math.sqrt(sum(v * v for v in self._px4_vel)))
            print(f">>> vision: HOLDING phase 0 until the airframe is at rest "
                  f"for {SETTLE_S:.0f} sim s "
                  f"(speed {'unknown' if speed is None else f'{speed:.2f} m/s'}"
                  f"). Rebooting PX4 mid-motion is what makes EKF2 refuse to "
                  f"arm with `height estimate not stable`. If the aircraft is "
                  f"FLYING, phase 0 will not run at all -- land, or restart "
                  f"with --no-vision.")
        return True

    def _restore_gnss(self):
        """Undo the cut: GNSS back on, vision still fused. Setpoint thread only.

        The abort, and the only way back from `gps_denied` -- until 2026-08-13
        the cut was one-way and recovery meant `px4-param set EKF2_GPS_CTRL 7`
        in a shell on the box. That was tolerable while GPS-denied flight was
        the thing being proven and someone was always sitting at the terminal.
        It stopped being tolerable once the estimator was diverging on every
        flight: the recovery action was a shell command, on a machine the
        operator might not be at, while the aircraft accelerated away. Flown
        2026-08-13, 180 m off and making 3.8 m/s, and the fix was a command
        line.

        Takes NO preconditions and can never refuse. Every refusal in
        `_go_gps_denied` is there because cutting GNSS onto a bad source can
        put the aircraft in the ground; restoring it has no such failure mode.
        A recovery control that can say no is not a recovery control.

        Clearing `_gps_denied` also stops `_send_startup_params` from
        re-asserting the cut on the next heartbeat gap -- without that, the
        restore would silently undo itself.
        """
        if self.vision is None:
            print("*** gps_restore: server was not started with --vision, so "
                  "GNSS was never cut. ***")
            return
        vision_bridge.apply_ekf2_gnss_restore_params(self.link)
        was_denied = self._gps_denied
        self._gps_denied = False
        with self._telem_lock:
            self._telem["gps_denied"] = False
        print(f">>> GNSS RESTORED: EKF2_GPS_CTRL="
              f"{vision_bridge.EKF2_GPS_CTRL_DEFAULT}, vision still fusing "
              f"alongside it"
              + ("." if was_denied else " (it had not been cut).")
              + " `gps_denied` is available again once you are happy with the "
                "estimate.")

    def _send_startup_params(self):
        # Phase 0 is a reboot, so nothing goes out at all until the airframe is
        # at rest -- not even the ordinary params below, which would only be
        # lost in the reboot gap. Returning without setting _params_sent is
        # what makes the next heartbeat try again.
        if (self.vision is not None and not self._vision_rebooted
                and self._wait_for_settle()):
            return
        self.link.set_param("COM_RCL_EXCEPT", offboard.COM_RCL_EXCEPT_OFFBOARD,
                            offboard.MAV_PARAM_TYPE_INT32)
        self.link.set_param("MIS_TAKEOFF_ALT", self.takeoff_alt,
                            offboard.MAV_PARAM_TYPE_REAL32)
        # PX4's default MPC_XY_VEL_MAX is 12 m/s (mc_pos_control_params.c:412)
        # against the pad's 2 m/s. Unclamped, FLY would send the aircraft off
        # six times faster than anything the operator has seen it do.
        self.link.set_param("MPC_XY_VEL_MAX", self.mission_speed,
                            offboard.MAV_PARAM_TYPE_REAL32)
        if self.vision is not None:
            if not self._vision_rebooted:
                # Phase 0. PX4 is already booted by the time we can talk to it,
                # so the @reboot_required height reference can only be made to
                # take by restarting it. Everything else waits: PX4 is about to
                # drop off the link, and params sent into that gap are lost.
                # The reboot shows up as a heartbeat gap, _check_px4_restart
                # clears _params_sent, and this method runs again with
                # _vision_rebooted set -- so phase 1 lands on the fresh PX4.
                self._vision_rebooted = True
                vision_bridge.reboot_for_boot_params(self.link)
                print(f">>> vision: set {', '.join(vision_bridge.HGT_REF_NEEDS_REBOOT)}"
                      f" and rebooting PX4 so it takes effect -- "
                      f"params resume when it comes back")
                return
            # Re-assert whatever phase we are ACTUALLY in, not phase 1.
            #
            # This method re-runs on any heartbeat gap (_check_px4_restart), so
            # it is not only a startup path -- it is the recovery path too.
            # Blindly asserting GPS flight here turns vision fusion back off
            # mid-flight, which is exactly what happened on 2026-08-11: fusion
            # engaged at altitude, a heartbeat gap re-ran this, and EKF2 quietly
            # stopped using vision again.
            if self._gps_denied:
                vision_bridge.apply_ekf2_fusion_params(self.link)
                vision_bridge.apply_ekf2_gps_denied_params(self.link)
            elif self._vision_fusing:
                vision_bridge.apply_ekf2_fusion_params(self.link)
            else:
                # PX4 params persist, so a previous GPS-denied session can
                # otherwise leave this run booting with GNSS off and vision
                # fused over a blind camera.
                vision_bridge.apply_ekf2_gps_flight_params(self.link)
            # The origin goes in now; fusion waits for a camera that can see.
            #
            # Order matters and is asserted by
            # test_gps_origin_is_sent_before_the_first_vision_estimate: PX4
            # cannot place a local estimate without an anchor, and VPE carries
            # no lat/lon of its own.
            if self.vision_origin is not None:
                vision_bridge.send_gps_global_origin(self.link,
                                                     *self.vision_origin)
                print(f">>> vision: GPS origin {self.vision_origin}")
            else:
                print("*** vision: NO ORIGIN configured. GLOBAL_POSITION_INT, "
                      "waypoint flight and the FLY gate will all fail "
                      "silently. ***")
            print(f">>> vision: GNSS still ON and EKF2 is NOT yet fusing "
                  f"vision -- it starts once the estimate clears "
                  f"{VISION_MIN_INLIERS} inliers, which needs altitude. "
                  f"Then send `gps_denied` to cut GNSS.")
        self._params_sent = True
        print(f">>> params: COM_RCL_EXCEPT=4 (offboard exempt from RC-loss "
              f"failsafe), MIS_TAKEOFF_ALT={self.takeoff_alt}, "
              f"MPC_XY_VEL_MAX={self.mission_speed}")

    def _check_px4_restart(self, now):
        """Re-arm the param send if PX4 has gone quiet long enough to have
        rebooted. Setpoint thread only."""
        if not self._params_sent or self._last_heartbeat is None:
            return
        if now - self._last_heartbeat > PX4_RESTART_GAP_S:
            self._params_sent = False
            self._last_heartbeat = None
            print(">>> PX4 heartbeat gap -- assuming a restart; params and "
                  "GPS origin will be re-sent on the next heartbeat.")

    def _send_vision(self, now):
        """Forward the latest estimate. Setpoint thread only.

        Never raises into run(): a vision problem must not be able to gap the
        setpoint stream, because that gap is what drops PX4 out of OFFBOARD.
        """
        if self.vision_sender is None:
            return False
        pose, clock_now, received_at = self._vision_clock(now)
        return self.vision_sender.send(pose, clock_now, received_at)

    def _vision_clock(self, now_wall):
        """(pose, now, received_at) with both times on ONE clock, sim if known.

        Idempotent within a tick, and safe to call in any order, so the sender's
        drop decision and the page's `fresh` cannot drift apart -- the aircraft's
        position source is gated on the first and the GNSS cut on the second.

        The clock is PX4's `time_boot_ms`, which under lockstep IS sim time.
        Wall time is used only until PX4 has streamed its first position, at
        which point nothing is flying anyway. "Age" means time since a NEW
        estimate arrived, so it is measured from when a pose first appeared
        rather than from each time this is called.
        """
        pose, received_at_wall = self.vision.latest()
        if self._px4_sim_s is None:
            self._vision_age_s = (None if received_at_wall is None
                                  else now_wall - received_at_wall)
            return pose, now_wall, received_at_wall

        clock_now = self._px4_sim_s
        if pose is not None and pose.ts_ns != self._vision_seen_ts:
            self._vision_seen_ts = pose.ts_ns
            self._vision_seen_at = clock_now
        received_at = self._vision_seen_at
        self._vision_age_s = (None if received_at is None
                              else clock_now - received_at)
        return pose, clock_now, received_at

    def _phase(self):
        """Current flight phase, CONTEXT.md's numbering. Setpoint thread only."""
        if self.vision is None:
            return "1"          # vision never configured: plain GPS flight
        if not self._vision_rebooted:
            return "0"
        if self._gps_denied:
            return "2"
        if self._vision_fusing:
            return "1b"
        return "1"

    def _excursion_m(self):
        """Distance from the hold point taken at the cut, in ground truth.

        None whenever it is not meaningful: no cut has been taken, the cut had
        no ground truth to anchor on, or GNSS has since been restored -- a
        restored flight is not being held to phase 2's hold point at all, and
        showing a number against a stale anchor would misreport the aircraft
        as excursing when it is simply flying under GNSS again.
        """
        if not self._gps_denied or self._hold_point_gt is None:
            return None
        last = getattr(self.vision, "_last_msg", None) or {}
        gx, gy = last.get("gt_x"), last.get("gt_y")
        if gx is None or gy is None:
            return None
        hx, hy = self._hold_point_gt
        return math.hypot(gx - hx, gy - hy)

    def _vio_status(self, now):
        """The `vio` telemetry block, or None when vision is not configured.

        `fresh` is the one the operator has to be able to trust: false means
        the aircraft is flying on dead reckoning, which the page renders as
        `bad` rather than as a quiet absence.
        """
        if self.vision_sender is None:
            return None
        # The same age the sender judged on, so the page's `fresh` and the
        # sender's drop decision can never disagree -- the GPS-denied control is
        # gated on one of them and the aircraft's position source on the other.
        self._vision_clock(now)
        age = self._vision_age_s
        last = getattr(self.vision, "_last_msg", None) or {}
        return {
            "fresh": bool(age is not None
                          and age <= self.vision_sender.max_age_s),
            "age_s": age,
            "n_inliers": last.get("n_inliers"),
            # None, never 0.0 -- no GT topic is not zero drift.
            "drift_m": last.get("drift_m"),
            # Aircraft excursion (CONTEXT.md): where the airframe truly is vs
            # where it was told to hold, from ground truth -- NOT drift_m,
            # which is the estimator's own error and never sees a hold point.
            # None until a cut has actually been taken (ADR-0004, ADR-0001).
            "excursion_m": self._excursion_m(),
            "fps": last.get("fps"),
            "sent": self.vision_sender.sent,
            "dropped_stale": self.vision_sender.dropped_stale,
            "received": getattr(self.vision, "received", 0),
            # How many times the vision frame has been pinned onto PX4's, and
            # how far the last pin moved it. `align_m` is the error EKF2 would
            # have inherited had nothing been done -- the number this whole
            # change is about, so it is visible in flight rather than only in
            # the console.
            "realigned": self.vision_sender.realigned,
            "align_m": self.vision_sender.alignment.offset_m(),
        }

    def _write_csv_row(self, vio_status):
        """Append one RunCSV row. Setpoint thread only -- reads state written
        by _drain_mavlink on that same thread, plus vio_status just computed
        this tick by _vio_status(), so nothing here re-derives the age/fresh
        judgement or can disagree with the telemetry the operator is reading.
        """
        last = getattr(self.vision, "_last_msg", None) or {}
        n, e, d = self._px4_ned if self._px4_ned is not None else (None, None, None)
        self._csv.write({
            "sim_s": self._px4_sim_s,
            "phase": self._phase(),
            "px4_n": n, "px4_e": e, "px4_d": d, "px4_yaw": self._px4_yaw,
            "gt_x": last.get("gt_x"), "gt_y": last.get("gt_y"),
            "gt_z": last.get("gt_z"),
            "vio_x": last.get("x"), "vio_y": last.get("y"), "vio_z": last.get("z"),
            "vio_yaw": last.get("yaw"),
            "excursion_m": (vio_status or {}).get("excursion_m"),
            "drift_m": last.get("drift_m"),
            "align_m": self.vision_sender.alignment.offset_m()
                if self.vision_sender is not None else None,
            "realigned": self.vision_sender.realigned
                if self.vision_sender is not None else None,
            "n_inliers": last.get("n_inliers"),
            "fresh": (vio_status or {}).get("fresh"),
            "dropped_stale": self.vision_sender.dropped_stale
                if self.vision_sender is not None else None,
            "sim_rate": self._telem.get("sim_rate"),
        })

    def _drain_mavlink(self):
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return
            kind = msg.get_type()
            if kind == "HEARTBEAT":
                self.link.bind_target(msg)
                self._last_heartbeat = time.monotonic()
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
                self._px4_ned = (msg.x, msg.y, msg.z)
                self._px4_vel = (msg.vx, msg.vy, msg.vz)
                self._px4_sim_s = msg.time_boot_ms / 1000.0
                with self._telem_lock:
                    self._telem["sim_s"] = self._px4_sim_s
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
            elif kind == "ATTITUDE":
                # Radians, NED, already in the convention VPE wants -- unlike
                # telemetry["heading_deg"], which comes from
                # GLOBAL_POSITION_INT.hdg in centidegrees for the map arrow.
                # The alignment takes this one because it is the raw quantity
                # and needs no unwinding of a display format.
                self._px4_yaw = msg.yaw
            elif kind == "GLOBAL_POSITION_INT":
                self._handle_global_position(msg)
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
            else:
                vx, vy, vz, yaw_rate = self.state.command()
                self.link.send_velocity(vx, vy, vz, yaw_rate)

            # AFTER the setpoint, never before and never instead: the setpoint
            # stream is what holds OFFBOARD, so vision rides along with it and
            # can never displace it.
            now_mono = time.monotonic()
            try:
                self._send_vision(now_mono)
                self._maybe_start_fusing_vision(now_mono)
            except Exception as exc:
                print(f"*** vision send failed (setpoints continue): {exc!r} ***")
            self._check_px4_restart(now_mono)

            if self._stream_start is None:
                self._stream_start = time.monotonic()
            streaming_s = time.monotonic() - self._stream_start
            # Taken before _telem_lock, never inside it: Mission has its own
            # lock and nesting the two would introduce a cycle.
            mission_status = self.mission.status()
            vio_status = self._vio_status(now_mono)
            if self._csv is not None:
                self._write_csv_row(vio_status)
            with self._telem_lock:
                self._telem["cmd_vx"] = vx
                self._telem["cmd_yaw_rate"] = yaw_rate
                self._telem["streaming_s"] = streaming_s
                self._telem["vio"] = vio_status
                self._telem["mission"] = mission_status
                self._telem["ready_for_offboard"] = (
                    streaming_s >= self.warmup_s and self._telem["connected"])

            next_tick += self.dt
            nap = next_tick - time.monotonic()
            if nap > 0:
                time.sleep(nap)
            else:
                next_tick = time.monotonic()   # fell behind; resync


class EstimatorProcess:
    """Runs pipeline-streaming.py as a child so one command brings the stack up.

    A SUBPROCESS, never a thread, and that is a hard design point rather than a
    convenience. The estimator's LK tracking is the heaviest work in this
    system, and the setpoint thread must tick at 20 Hz or PX4 drops OFFBOARD.
    This server already refuses to put a few-hundred-millisecond HTTP call on
    that thread (RecorderProxy, design R2); continuous frame processing in the
    same interpreter is a larger version of the same hazard. A separate process
    also means an OpenCV fault takes down the estimate and not the aircraft's
    control link.

    Output is INHERITED, not piped: `--print-every` lines are the instrument the
    operator actually reads during a climb, and piping them through here would
    only add a thread and a chance to lose them.

    Restarts are the point of supervising at all -- a dead estimator is plan
    Task 8.9's failure mode, and recovery is meant to need no sim restart. But
    a child that dies IMMEDIATELY is not a crash to recover from, it is a
    misconfiguration (no cv2, port already bound because an estimator is
    already running), and retrying that forever would bury the reason in a
    scroll of identical errors. So rapid failures give up and say why.
    """

    RESTART_DELAY_S = 2.0
    HEALTHY_S = 20.0
    """A child that lived this long counts as having run, so its exit is a
    crash worth restarting rather than a startup failure worth reporting."""
    MAX_RAPID_FAILURES = 3

    def __init__(self, script, args, python=None):
        self.script = script
        self.args = list(args)
        self.python = python or sys.executable
        self.proc = None
        self._stop = threading.Event()
        self._rapid = 0

    def command(self):
        return [self.python, self.script, *self.args]

    def start(self):
        """Spawn the child and supervise it on a daemon thread."""
        threading.Thread(target=self._supervise, daemon=True).start()

    def _spawn(self):
        import subprocess
        return subprocess.Popen(self.command(), cwd=os.path.dirname(self.script)
                                or None)

    def _supervise(self):
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self.proc = self._spawn()
            except Exception as exc:
                print(f"*** estimator: cannot start {self.script}: {exc!r} ***")
                return
            code = self.proc.wait()
            if self._stop.is_set():
                return
            lived = time.monotonic() - started
            if lived >= self.HEALTHY_S:
                self._rapid = 0
            else:
                self._rapid += 1
            if self._rapid >= self.MAX_RAPID_FAILURES:
                print(f"*** estimator: exited {self._rapid} times in under "
                      f"{self.HEALTHY_S:.0f}s (last code {code}). NOT "
                      f"restarting again. If one is already running, start "
                      f"this server with --no-estimator. ***")
                return
            print(f"*** estimator exited (code {code}) after {lived:.0f}s -- "
                  f"restarting in {self.RESTART_DELAY_S:.0f}s. Vision goes "
                  f"STALE until it is back. ***")
            if self._stop.wait(self.RESTART_DELAY_S):
                return

    def stop(self):
        """Kill the child. Without this it outlives the server and keeps 5557
        bound, so the next run cannot start its own."""
        self._stop.set()
        proc, self.proc = self.proc, None
        if proc is None or proc.poll() is not None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except Exception:
            proc.kill()


class RecorderProxy:
    """The page's view of the VIO recorder, which lives inside Isaac Sim.

    Deliberately NOT routed through SetpointLoop.submit(). Every other command
    from the page goes on that queue because it touches the MAVLink connection,
    which only the setpoint thread may own. Recording touches no MAVLink at all,
    and an HTTP call into Kit can block for hundreds of milliseconds while its
    main thread is mid-frame. A gap that long in the setpoint stream drops PX4
    out of OFFBOARD -- so routing RECORD through that thread would mean pressing
    it could drop the aircraft (design R2).

    The `drone` env has no async HTTP client, so urllib runs in a worker thread
    (design R3). Nothing here ever blocks the event loop for longer than the
    timeout.
    """

    # What the page sees when Isaac is not up. A state, not an error: this is
    # the normal condition before launch-sitl.sh has finished.
    OFFLINE = {"state": "offline", "can_start": False, "run_dir": None,
               "elapsed_s": 0.0, "frames": 0, "images": 0, "dropped": 0,
               "queue": 0, "bytes": 0, "free_bytes": None,
               "min_free_bytes": None, "warn_free_bytes": None, "error": None}

    def __init__(self, base_url, timeout=2.0, poll_hz=1.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.poll_hz = poll_hz
        self._status = dict(self.OFFLINE)
        self._cmd_error = None      # sticky until the next command is issued
        self._pending = set()

    def status(self):
        return dict(self._status, cmd_error=self._cmd_error)

    def submit(self, action):
        """Fire-and-forget, called from the WebSocket handler.

        Awaiting the command there would stop that handler reading messages for
        as long as the HTTP call takes -- up to the 2 s timeout -- and 0.5 s of
        silence is all it takes for the watchdog to zero a held direction. So
        pressing RECORD while flying forward would stop the drone. The result
        reaches the page through the telemetry frames instead.
        """
        task = asyncio.create_task(self.command(action))
        self._pending.add(task)                 # or it can be GC'd mid-flight
        task.add_done_callback(self._pending.discard)
        return task

    def _request(self, path, method="GET"):
        """Blocking. Only ever called inside asyncio.to_thread.

        -> (http_code, body). A refused command (409/507/503) is a normal
        answer carrying a reason, not an exception, so HTTPError is unwrapped
        rather than raised.
        """
        req = urllib.request.Request(self.base_url + path, method=method,
                                     data=b"" if method == "POST" else None)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.status, json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as exc:
            try:
                return exc.code, json.loads(exc.read() or b"{}")
            except Exception:
                return exc.code, {}

    async def command(self, action):
        """start | stop. Runs on the web thread's event loop (design R2)."""
        if action not in ("start", "stop"):
            return
        self._cmd_error = None
        try:
            code, body = await asyncio.to_thread(self._request,
                                                 f"/record/{action}", "POST")
        except Exception as exc:
            self._cmd_error = (f"cannot reach the recorder in Isaac Sim "
                               f"({exc.__class__.__name__}) -- is the sim running?")
            self._status = dict(self.OFFLINE)
            return
        if code >= 400:
            # 409/503/507 all carry the reason the control server refused, and
            # that reason is the whole point -- "disk too full" must reach the
            # operator, not just "failed".
            self._cmd_error = body.get("error") or f"recorder refused ({code})"
        else:
            # Don't wait up to a second for the next poll to show the change.
            await self.refresh()

    async def refresh(self):
        try:
            code, body = await asyncio.to_thread(self._request, "/record/status")
        except Exception:
            self._status = dict(self.OFFLINE)
            return
        self._status = body if code == 200 else dict(self.OFFLINE)

    async def poll_forever(self):
        """1 Hz, not the 5 Hz telemetry rate: the counters do not change faster
        than that and Kit should not be polled for free disk space 5 times a
        second."""
        while True:
            await self.refresh()
            await asyncio.sleep(1.0 / self.poll_hz)


async def _push_telemetry(sock, loop_thread, recorder, hz=5.0):
    try:
        while True:
            telem = loop_thread.telemetry()
            telem["rec"] = recorder.status()
            await sock.send_text(json.dumps(telem))
            await asyncio.sleep(1.0 / hz)
    except Exception:
        pass          # socket closed; the /ws handler cleans up


def build_app(loop_thread, state, video_port, mission_speed, recorder):
    from contextlib import asynccontextmanager

    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import JSONResponse
    from fastapi.staticfiles import StaticFiles

    @asynccontextmanager
    async def lifespan(_app):
        # One poller for the server, not one per socket: two phones on the page
        # must not double the load on Kit.
        poller = asyncio.create_task(recorder.poll_forever())
        yield
        poller.cancel()

    app = FastAPI(lifespan=lifespan)

    @app.get("/config")
    def config():
        return JSONResponse({"video_port": video_port,
                             "mission_speed": mission_speed})

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        state.clear()
        pusher = asyncio.create_task(_push_telemetry(sock, loop_thread,
                                                     recorder))
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
                    if msg["name"] == "set_param":
                        loop_thread.submit_set_param(
                            msg["param"], msg["value"], msg["param_type"])
                    else:
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
                elif kind == "record":
                    # Inline on this loop, never loop_thread.submit(). See
                    # RecorderProxy for why the setpoint thread must not carry
                    # this, and .submit() for why it is not awaited here.
                    recorder.submit(msg.get("action"))
                elif kind == "ping":
                    state.touch()
        except (WebSocketDisconnect, json.JSONDecodeError, KeyError, ValueError):
            pass
        finally:
            pusher.cancel()
            # A dropped socket must not latch the last commanded velocity.
            state.clear()

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
    ap.add_argument("--recorder-url", default="http://127.0.0.1:8091",
                    help="VIO recorder control server inside Isaac Sim "
                         "(sim/recorder_control.py). Unreachable is fine -- "
                         "the page shows the recorder as offline")
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
    ap.add_argument("--vision", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="set up for GPS-DENIED flight: fuse "
                         "pipeline-streaming.py's estimate and REBOOT PX4 at "
                         "startup so EKF2_HGT_REF takes. On by default so the "
                         "whole stack is one command; --no-vision gives an "
                         "ordinary GPS flight that touches no EKF2 params. "
                         "GNSS itself is cut only by the page's GO GPS-DENIED "
                         "button, never by starting this server.")
    ap.add_argument("--estimator", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="run pipeline-streaming.py as a child process under "
                         "--vision. --no-estimator if you are already running "
                         "one by hand.")
    ap.add_argument("--estimator-scale", type=float, default=0.5,
                    help="image downscale before tracking, passed through")
    ap.add_argument("--estimator-print-every", type=int, default=15,
                    help="estimator prints pos/drift every Nth frame -- the "
                         "instrument for watching drift accumulate")
    ap.add_argument("--sensor-endpoint", default="tcp://127.0.0.1:5556",
                    help="where vio-streamer.py publishes, for the estimator")
    ap.add_argument("--vio-endpoint", default="tcp://127.0.0.1:5557",
                    help="where pipeline-streaming.py publishes `vio`")
    ap.add_argument("--estimator-bind", default="tcp://*:5557",
                    help="the BIND side of --vio-endpoint, handed to the "
                         "child. Separate because a ZMQ bind address is not a "
                         "connect address -- deriving one from the other by "
                         "string surgery breaks the moment either is "
                         "overridden.")
    ap.add_argument("--site", default=os.environ.get("SITL_SITE",
                                                     DEFAULT_SITE),
                    help="sim/sites.py key; supplies the GPS origin under "
                         "--vision")
    ap.add_argument("--run-name", default=None,
                    help="names ./logs/<run-name>/run.csv under --vision. "
                         "Pass the same name to sim/save-ulog.sh so the ulog "
                         "lands beside it. Defaults to a timestamp.")
    args = ap.parse_args()

    import uvicorn

    vision = None
    vision_origin = None
    estimator = None
    csv_path = None
    if args.vision:
        if not args.site:
            ap.error("--vision needs --site (or SITL_SITE) for the GPS origin: "
                     "with GNSS off, PX4 has no other way to anchor its global "
                     "frame, and the map, waypoints and the FLY gate all fail "
                     "silently without it.")
        sys.path.insert(0, os.path.join(ROOT, "sim"))
        import sites
        site = sites.get_site(args.site)
        vision_origin = (site.latitude, site.longitude, site.height)
        # Task 9 instrumentation (ADR-0002/0003/0004): a run CSV alongside the
        # ulog sim/save-ulog.sh copies out, so a classification flight can be
        # read back afterwards instead of only watched live.
        run_name = args.run_name or time.strftime("%Y%m%d-%H%M%S")
        csv_path = os.path.join(ROOT, "logs", run_name, "run.csv")
        print(f">>> run CSV: logs/{run_name}/run.csv -- pass "
              f"\"{run_name}\" to sim/save-ulog.sh to keep the ulog beside it")
        if args.estimator:
            estimator = EstimatorProcess(
                os.path.join(ROOT, ESTIMATOR_SCRIPT),
                ["--sensor-endpoint", args.sensor_endpoint,
                 "--vio-endpoint", args.estimator_bind,
                 "--scale", str(args.estimator_scale),
                 "--print-every", str(args.estimator_print_every)])
            estimator.start()
            print(f">>> estimator: {' '.join(estimator.command())}")
        vision = VisionSubscriber(args.vio_endpoint)
        vision.start()
        # Loud on purpose. This used to be opt-in, and the thing that changed
        # is a DEFAULT, not the behaviour: PX4 is about to be rebooted so
        # EKF2_HGT_REF takes, and the EKF2 param set is about to be rewritten.
        # An operator who wanted a plain GPS flight has to be told, not left to
        # notice the autopilot restarting.
        print(f">>> GPS-DENIED READY (--no-vision for a plain GPS flight): "
              f"vision from {args.vio_endpoint}, origin from site {site.name}."
              f" PX4 will be REBOOTED once at startup for EKF2_HGT_REF. GNSS "
              f"stays ON until you press GO GPS-DENIED.")

    conn = mavutil.mavlink_connection(args.mavlink)
    state = offboard.CommandState(args.speed_fwd, args.speed_up, args.watchdog,
                                  args.yaw_rate)
    loop_thread = SetpointLoop(conn, state, args.rate, args.takeoff_alt,
                               args.offboard_warmup, args.mission_speed,
                               args.arrival_radius,
                               vision=vision, vision_origin=vision_origin,
                               csv_path=csv_path)
    loop_thread.start()

    print(f">>> MAVLink offboard link: {args.mavlink}")
    print(f">>> setpoint loop at {args.rate:.0f} Hz "
          f"({args.speed_fwd} m/s fwd, {args.speed_up} m/s climb, "
          f"{args.yaw_rate:.0f} deg/s turn)")
    print(f">>> open http://<box-ip>:{args.port}/")
    print(f">>> recorder control: {args.recorder_url}")
    recorder = RecorderProxy(args.recorder_url)
    try:
        uvicorn.run(build_app(loop_thread, state, args.video_port,
                              args.mission_speed, recorder),
                    host="0.0.0.0", port=args.port, log_level="warning")
    finally:
        # Ctrl-C included. An orphaned estimator keeps 5557 bound, so the next
        # run's child cannot start and the failure looks like a broken server.
        if estimator is not None:
            estimator.stop()
        if loop_thread._csv is not None:
            loop_thread._csv.close()


if __name__ == "__main__":
    main()
