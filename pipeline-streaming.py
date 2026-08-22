#!/usr/bin/env python
"""Streaming flow-odometry estimator: ZMQ sensors in, ENU position out.

    SUB tcp://127.0.0.1:5556   meta | imu | baro | frame | gt   (vio-streamer.py)
    PUB tcp://*:5557           vio                              (joystick-server.py)

The estimating half of GPS-denied flight. Runs OUTSIDE Kit, in the `drone`
conda env, because the LK tracking below is far too heavy for the physics
callback that PX4 SITL is lockstepped to -- a stall there drops the aircraft
out of OFFBOARD.

Every piece of vision maths is imported from pipeline.py unchanged and the
attitude filter from flow_odometry.py, so there is exactly one implementation
of each and this file is only the plumbing between them.

GROUND TRUTH NEVER ENTERS THE ESTIMATE (design D4). The `gt` topic feeds the
drift number and nothing else; --no-gt proves it by refusing to subscribe at
all, and the estimate must come out bit-identical.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import zmq

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "streaming"))

import zmq_proto                                            # noqa: E402
from flow_odometry import MahonyState                       # noqa: E402
from pipeline import _detect, _track_lk, _solve_translation  # noqa: E402

MIN_H0_M = 0.3
"""Floor on the depth scale, matching flow_odometry.py:473.

Below this the ground-plane depth blows up and one bad frame can throw the
position by hundreds of metres.
"""


MAHONY_KI = 0.05
"""Mahony integral gain on the live filter, in 1/s^2.

Sets how fast the gyro-bias estimate converges -- roughly Kp/Ki, so ~20 s here
-- against how much of an accelerating airframe's lie it absorbs on the way.
The proportional term is excluded from the derotation entirely, so this is the
only path by which the accelerometer still reaches the flow solve, and 0.05
keeps that path slow enough that a manoeuvre's contribution unwinds inside a
hold rather than accumulating across it. The batch pipeline keeps Ki=0.
"""


class Estimator:
    """Flow-odometry carried one message at a time.

    Mirrors flow_odometry.run()'s loop: detect on the previous frame, LK-track
    into the current one, solve a camera translation against a ground plane at
    the barometric height, and integrate that into ENU. Altitude is never
    integrated -- it is read straight off the barometer, as in
    flow_odometry.py:520.
    """

    def __init__(self, meta, scale=1.0, min_track=30, use_gt=True, mag_gain=0.0):
        self.K = np.asarray(meta["K"], dtype=float)
        self.R_CtoI = np.asarray(meta["R_CtoI"], dtype=float)
        self.scale = scale
        self.min_track = min_track
        self.use_gt = use_gt
        if scale != 1.0:            # intrinsics must follow the resize
            self.K = self.K.copy()
            self.K[:2, :] *= scale
        self.Kinv = np.linalg.inv(self.K)

        # mag_gain stays 0.0 by default (ADR-0005): the magnetometer channel is
        # recorded so offline analysis can show what it would have corrected,
        # not acted on live. MahonyState.update() ignores `mag` entirely while
        # its own mag_gain is 0, so calibrating and feeding it every tick below
        # costs nothing when the flag is left at the default.
        self.state = MahonyState.from_heading(float(meta.get("heading_deg", 0.0)),
                                              Ki=MAHONY_KI, mag_gain=mag_gain)
        # A SECOND attitude integration, gyro only (Kp=0 makes update() ignore
        # the accelerometer term entirely). It exists for one job: the
        # inter-frame rotation the flow solve derotates by.
        #
        # An accelerometer measures specific force, so while the aircraft
        # accelerates horizontally its reading is tilted atan(a/g) from true
        # down and `self.state`'s gravity term pulls the attitude toward it. In
        # the ABSOLUTE attitude that error only mis-scales real motion. In the
        # RELATIVE rotation it is far worse: an attitude error moving at w
        # rad/s subtracts a rotation that never happened, and at height h the
        # leftover flow reads as h*w m/s of translation -- fabricated from a
        # still hover, then flown for real by a controller cancelling it.
        # Measured on run 20260822-poshold: h*d(atan(a/g))/dt predicted 5.75
        # m/s of false velocity against 6.69 m/s observed.
        #
        # Only CONSECUTIVE differences of this one are ever read, so its own
        # unbounded yaw drift cancels and never reaches the estimate.
        self.gyro_state = MahonyState.from_heading(
            float(meta.get("heading_deg", 0.0)), Kp=0.0, mag_gain=0.0)
        self.mag_calibrated = False
        self.pos = np.zeros(3)      # ENU, anchored at the first frame
        self.baro_alt = None
        self.baro0 = None
        self.prev_gray = None
        self.prev_R_wc = None
        self.prev_R_wc_gyro = None
        self.n_inliers = 0
        self.n_frames = 0
        self.n_solved = 0
        self.last_gt = None         # SCORING ONLY -- never read by the estimate
        # The GT heading, same scoring-only status. The map's green arrow
        # points where the airframe TRULY points, which nothing else in
        # this payload can supply -- self.state's yaw is the estimate.
        self.last_gt_yaw = None

    # --- sensor inputs -----------------------------------------------------

    def on_imu(self, msg, dt):
        mag = msg.get("m")
        if mag is not None and not self.mag_calibrated:
            # One-time factory-compass calibration (flow_odometry.py's batch
            # loop averages ~50 samples; streaming just takes the first tick
            # that has one -- close enough for a slowly-varying field, and the
            # gain stays 0 either way).
            self.state.calibrate_mag(mag)
            self.mag_calibrated = True
        self.state.update(msg["w"], msg["a"], dt, mag=mag)
        # The gyro MINUS the bias the filter above just estimated, and nothing
        # else: no proportional term, so an accelerating airframe cannot reach
        # the derotation, and no standing bias either.
        self.gyro_state.update(
            np.asarray(msg["w"], dtype=float) - self.state.gyro_bias,
            msg["a"], dt)

    def on_baro(self, msg):
        self.baro_alt = float(msg["alt_m"])
        if self.baro0 is None:
            self.baro0 = self.baro_alt

    def on_gt(self, msg):
        """Drift reporting only. Nothing here may reach self.pos."""
        if self.use_gt:
            self.last_gt = np.asarray(msg["p"], dtype=float)
            q = msg.get("q")
            if q is not None:
                # xyzw, FLU in ENU (vio-streamer.py sends Isaac's own order).
                # Yaw about up, counter-clockwise from EAST -- the server turns
                # it into a compass bearing for the arrow.
                x, y, z, w = (float(v) for v in q)
                self.last_gt_yaw = float(np.arctan2(
                    2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def on_frame(self, msg):
        """One camera frame. Returns the vio payload, or None if not processed."""
        buf = np.frombuffer(msg["jpg"], dtype=np.uint8)
        gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return None
        if self.scale != 1.0:
            gray = cv2.resize(gray, None, fx=self.scale, fy=self.scale,
                              interpolation=cv2.INTER_AREA)

        R_wc = self.state.R_flu() @ self.R_CtoI     # camera -> ENU, this frame
        # The same transform off the gyro-only attitude. Used ONLY for the
        # inter-frame rotation below; the absolute R_wc still carries the
        # gravity-corrected attitude, which is what the ground-plane depth and
        # the ENU rotation of the solved translation need.
        R_wc_gyro = self.gyro_state.R_flu() @ self.R_CtoI
        h_above = 0.0 if self.baro0 is None else (self.baro_alt - self.baro0)

        if self.prev_gray is not None:
            self.n_frames += 1
            p0 = _detect(self.prev_gray)
            if p0 is not None and len(p0) >= self.min_track:
                p0g, p1g = _track_lk(self.prev_gray, gray, p0)
                if len(p0g) >= self.min_track:
                    R_c1c0 = R_wc_gyro.T @ self.prev_R_wc_gyro
                    h0 = max(h_above, MIN_H0_M)
                    t_cam, used = _solve_translation(
                        p0g, p1g, self.Kinv, self.prev_R_wc, R_c1c0,
                        h0, self.min_track)
                    if t_cam is not None:
                        dC = self.prev_R_wc @ t_cam
                        self.pos[0] += dC[0]
                        self.pos[1] += dC[1]
                        self.n_inliers = used
                        self.n_solved += 1

        self.pos[2] = h_above       # altitude from baro, never integrated
        self.prev_gray = gray
        self.prev_R_wc = R_wc
        self.prev_R_wc_gyro = R_wc_gyro
        return self.payload(msg)

    # --- output ------------------------------------------------------------

    def payload(self, msg):
        R = self.state.R_flu()
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        pitch = float(-np.arcsin(np.clip(R[2, 0], -1.0, 1.0)))
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        drift = None
        if self.last_gt is not None:
            drift = float(np.linalg.norm(self.pos[:2] - self.last_gt[:2]))
        gt = self.last_gt
        return {
            "ts_ns": int(msg["ts_ns"]),
            "frame": int(msg.get("frame", 0)),
            "x": float(self.pos[0]), "y": float(self.pos[1]), "z": float(self.pos[2]),
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "n_inliers": int(self.n_inliers),
            "fix_ok": bool(self.n_inliers >= self.min_track),
            "drift_m": drift,          # None, never 0.0, when GT is absent
            "n_frames": int(self.n_frames),
            # The Mahony integral term's gyro-bias estimate, rad/s. Exposed
            # because it is now IN the derotation path -- a bias estimate
            # polluted by a manoeuvre shows up as a constant-direction walk in
            # the hold, and there was no way to see it from outside.
            "gyro_bias": [float(b) for b in self.state.gyro_bias],
            "n_solved": int(self.n_solved),
            # SCORING ONLY (design D4, ADR-0004) -- forwarded so
            # joystick-server.py can compute aircraft excursion against PX4's
            # OWN estimate, which self.pos cannot be scored against without
            # this. None, never 0.0, when GT is absent -- same convention as
            # drift_m.
            "gt_x": None if gt is None else float(gt[0]),
            "gt_y": None if gt is None else float(gt[1]),
            "gt_z": None if gt is None else float(gt[2]),
            # Radians, ENU, counter-clockwise from east. None when the GT topic
            # is absent or predates the field -- same convention as gt_x.
            "gt_yaw": self.last_gt_yaw,
        }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sensor-endpoint", default="tcp://127.0.0.1:5556")
    ap.add_argument("--vio-endpoint", default=f"tcp://*:{zmq_proto.VIO_PORT}")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="image downscale before tracking (0.5 is ~4x faster)")
    ap.add_argument("--stride", type=int, default=1,
                    help="process every Nth frame; a longer baseline tracks better")
    ap.add_argument("--min-track", type=int, default=30)
    ap.add_argument("--mag-gain", type=float, default=0.0,
                    help="magnetometer fusion gain into the Mahony filter's yaw. "
                         "0.0 (default, ADR-0005) records the channel without "
                         "acting on it -- turning it on is this one flag")
    ap.add_argument("--print-every", type=int, default=0,
                    help="print every Nth processed frame (0 = never)")
    ap.add_argument("--no-gt", action="store_true",
                    help="do not even subscribe to gt -- the design D4 isolation check")
    ap.add_argument("--no-publish", action="store_true",
                    help="estimate but do not bind the vio PUB socket")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop after N processed frames (0 = run forever)")
    ap.add_argument("--rcvhwm", type=int, default=2000,
                    help="SUB receive high-water mark; 0 = unbounded (replays)")
    args = ap.parse_args(argv)

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    # Bound the receive queue explicitly rather than inheriting ZMQ's default
    # of 1000. A publisher faster than this estimator -- which is any replay
    # run flat out -- silently drops the overflow, and a dropped FRAME is an
    # unexplained gap in the trajectory. Live, a bound is what we want (old
    # samples are worthless); replaying, pass --rcvhwm 0 so nothing is lost.
    sub.setsockopt(zmq.RCVHWM, args.rcvhwm)
    sub.connect(args.sensor_endpoint)
    topics = [zmq_proto.TOPIC_META, zmq_proto.TOPIC_IMU,
              zmq_proto.TOPIC_BARO, zmq_proto.TOPIC_FRAME]
    if not args.no_gt:
        topics.append(zmq_proto.TOPIC_GT)
    for t in topics:
        sub.subscribe(t)
    print(f">>> SUB {args.sensor_endpoint} topics="
          f"{[t.decode() for t in topics]}", flush=True)

    pub = None
    if not args.no_publish:
        pub = ctx.socket(zmq.PUB)
        pub.setsockopt(zmq.SNDHWM, 50)
        pub.bind(args.vio_endpoint)
        print(f">>> PUB {args.vio_endpoint} topic=vio", flush=True)

    est = None
    last_imu_ts = None
    frame_i = 0
    t_start = None
    try:
        while True:
            topic, msg = zmq_proto.unpack(sub.recv_multipart())

            if topic == zmq_proto.TOPIC_META:
                if est is None:
                    est = Estimator(msg, scale=args.scale,
                                    min_track=args.min_track,
                                    use_gt=not args.no_gt,
                                    mag_gain=args.mag_gain)
                    print(f">>> primed from meta: site={msg.get('site')} "
                          f"vib_damp={msg.get('vib_damp')} "
                          f"heading={msg.get('heading_deg')}", flush=True)
                    if msg.get("vib_damp"):
                        print("*** WARNING: vib_damp is ON -- the camera<->IMU "
                              "extrinsic is time-varying and this estimate will "
                              "drift faster than it should. ***", flush=True)
                continue

            if est is None:
                continue            # nothing is meaningful before meta arrives

            if topic == zmq_proto.TOPIC_IMU:
                ts = msg["ts_ns"]
                dt = 0.0 if last_imu_ts is None else (ts - last_imu_ts) / 1e9
                last_imu_ts = ts
                est.on_imu(msg, dt)     # update() clamps a bad dt itself
            elif topic == zmq_proto.TOPIC_BARO:
                est.on_baro(msg)
            elif topic == zmq_proto.TOPIC_GT:
                est.on_gt(msg)
            elif topic == zmq_proto.TOPIC_FRAME:
                frame_i += 1
                if args.stride > 1 and frame_i % args.stride:
                    continue
                out = est.on_frame(msg)
                if out is None:
                    continue
                if t_start is None:
                    t_start = time.monotonic()
                out["fps"] = (est.n_frames / max(time.monotonic() - t_start, 1e-9))
                if pub is not None:
                    try:
                        pub.send_multipart(
                            zmq_proto.pack(zmq_proto.TOPIC_VIO, out), zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                if args.print_every and est.n_frames % args.print_every == 0:
                    d = out["drift_m"]
                    print(f"  frame {est.n_frames:5d}  pos=({out['x']:8.2f}, "
                          f"{out['y']:8.2f}, {out['z']:6.2f})  pts={out['n_inliers']:4d}"
                          f"  drift={'--' if d is None else f'{d:6.2f}m'}"
                          f"  {out['fps']:5.1f} fps", flush=True)
                if args.max_frames and est.n_frames >= args.max_frames:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if est is not None:
            d = None if est.last_gt is None else float(
                np.linalg.norm(est.pos[:2] - est.last_gt[:2]))
            print(f">>> {est.n_frames} frames, {est.n_solved} solved, "
                  f"final pos=({est.pos[0]:.2f}, {est.pos[1]:.2f}, {est.pos[2]:.2f})"
                  f" drift={'--' if d is None else f'{d:.2f} m'}", flush=True)
        sub.close(linger=0)
        if pub is not None:
            pub.close(linger=0)
    return est


if __name__ == "__main__":
    main()
