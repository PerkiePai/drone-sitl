#!/usr/bin/env python
"""Replay a recorded dataset over the ZMQ wire format vio-streamer.py uses.

A TOOL, not a test -- it needs a dataset on disk, so it is run by hand:

    conda run -n drone python streaming/tests/replay_dataset.py ~/vio_dataset/<run> &
    conda run -n drone python pipeline-streaming.py --print-every 20

This is what lets Task 5 be verified without Isaac running: the estimator
cannot tell a replay from a live sim, so a drift number measured here is
directly comparable to flow_odometry.run() on the same recording.

Publishes at the recording's own timestamps by default; --speed 0 goes as fast
as the subscriber can drain.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import zmq

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "streaming"))

import zmq_proto                                    # noqa: E402
from flow_odometry import load_dataset, quat_xyzw_to_R  # noqa: E402


def heading_from_takeoff(d):
    """Compass heading at frame 0, so MahonyState seeds the way batch does.

    flow_odometry.compute_ahrs_attitude initialises from GT frame 0. A live
    streamer gets this from the site config instead; a replay has the real
    thing, and using it keeps the comparison against run() honest rather than
    handicapping the streaming path with a wrong initial yaw.
    """
    path = os.path.join(d, "takeoff.json")
    if not os.path.isfile(path):
        return 0.0
    with open(path) as fh:
        qx, qy, qz, qw = json.load(fh)["attitude_xyzw"]
    R = quat_xyzw_to_R(qx, qy, qz, qw)          # FLU -> ENU
    yaw_enu = float(np.arctan2(R[1, 0], R[0, 0]))
    return float(90.0 - np.degrees(yaw_enu))    # ENU yaw -> compass


def read_csv(path):
    import csv
    if not os.path.isfile(path):
        return []
    with open(path) as fh:
        return list(csv.DictReader(fh))


def build_events(d, recs):
    """Every sample as (ts_ns, topic, payload), in recording order."""
    events = []
    for r in read_csv(os.path.join(d, "imu.csv")):
        events.append((int(r["ts_ns"]), zmq_proto.TOPIC_IMU, {
            "frame": int(r["frame"]), "ts_ns": int(r["ts_ns"]),
            "w": [float(r["wx"]), float(r["wy"]), float(r["wz"])],
            "a": [float(r["ax"]), float(r["ay"]), float(r["az"])]}))
    for r in read_csv(os.path.join(d, "baro.csv")):
        events.append((int(r["ts_ns"]), zmq_proto.TOPIC_BARO, {
            "frame": int(r["frame"]), "ts_ns": int(r["ts_ns"]),
            "pressure_hpa": float(r["pressure_hpa"]),
            "alt_m": float(r["pressure_altitude_m"]),
            "temp_c": float(r["temperature_c"])}))
    # GT comes from the loaded records so it is anchored exactly the way
    # load_dataset anchors it -- otherwise the drift number would be measured
    # against a different origin than run() uses.
    for rec in recs:
        events.append((rec["ts"], zmq_proto.TOPIC_GT, {
            "frame": rec["frame"], "ts_ns": rec["ts"],
            "p": [float(rec["gt"][0]), float(rec["gt"][1]), float(rec["gt"][2])],
            "q": [0.0, 0.0, 0.0, 1.0]}))
    for i, rec in enumerate(recs):
        with open(rec["img"], "rb") as fh:
            jpg = fh.read()
        events.append((rec["ts"], zmq_proto.TOPIC_FRAME, {
            "frame": rec["frame"], "frame_idx": i, "ts_ns": rec["ts"], "jpg": jpg}))
    events.sort(key=lambda e: (e[0], e[1] == zmq_proto.TOPIC_FRAME))
    return events


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dataset")
    ap.add_argument("--endpoint", default=f"tcp://*:{zmq_proto.SENSOR_PORT}")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="playback rate; 0 = as fast as possible")
    ap.add_argument("--settle-s", type=float, default=1.0,
                    help="wait before publishing so SUB sockets can connect")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args(argv)

    d = os.path.expanduser(args.dataset)
    K, R_CtoI, recs = load_dataset(d)
    if args.max_frames:
        recs = recs[:args.max_frames]
    print(f">>> {os.path.basename(d)}: {len(recs)} records", flush=True)

    meta = {
        "site": f"replay:{os.path.basename(d)}",
        "origin": {"lat": 0.0, "lon": 0.0, "h": 0.0},
        "heading_deg": heading_from_takeoff(d),
        "K": [[float(v) for v in row] for row in K],
        "R_CtoI": [[float(v) for v in row] for row in R_CtoI],
        "image_size": [int(K[0, 2] * 2), int(K[1, 2] * 2)],
        "vib_damp": False,
        "data_fps": 200, "frame_fps": 15,
    }

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 0)       # a replay must not drop; it is not live
    pub.bind(args.endpoint)
    print(f">>> PUB {args.endpoint}; settling {args.settle_s}s", flush=True)
    time.sleep(args.settle_s)

    events = build_events(d, recs)
    pub.send_multipart(zmq_proto.pack(zmq_proto.TOPIC_META, meta))

    t0_ns = events[0][0]
    wall0 = time.monotonic()
    n_frames = 0
    last_meta_ns = t0_ns
    for ts_ns, topic, payload in events:
        if args.speed > 0:
            due = wall0 + (ts_ns - t0_ns) / 1e9 / args.speed
            gap = due - time.monotonic()
            if gap > 0:
                time.sleep(gap)
        # Re-announce meta the way vio-streamer.py does. ZMQ PUB drops anything
        # sent before a subscriber has finished connecting, so a single meta at
        # the top is a race: lose it and the estimator ignores every subsequent
        # message forever, looking exactly like a hang.
        if ts_ns - last_meta_ns >= 1_000_000_000:
            last_meta_ns = ts_ns
            pub.send_multipart(zmq_proto.pack(zmq_proto.TOPIC_META, meta))
        pub.send_multipart(zmq_proto.pack(topic, payload))
        if topic == zmq_proto.TOPIC_FRAME:
            n_frames += 1
    # PUB has no delivery guarantee; give the subscriber a moment to drain
    # before the socket closes, or the last frames vanish.
    time.sleep(1.0)
    pub.close(linger=1000)
    print(f">>> replayed {len(events)} messages ({n_frames} frames)", flush=True)


if __name__ == "__main__":
    main()
