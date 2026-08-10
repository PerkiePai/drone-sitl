# ============================================================================
# VIO streamer -- publishes Isaac's sensor and camera streams over ZMQ so the
# flow-odometry estimator can run OUTSIDE Kit, in the `drone` conda env.
#
#   PUB tcp://*:5556    meta | imu | baro | frame | gt
#
# Mirrors vio-recorder-pai.py's wiring (same vehicle/IMU/Barometer lookup, same
# DATA_FPS decimation, same down_cam render product) but sends instead of
# writing to disk. The recorder is deliberately NOT modified: recording and
# streaming stay independent, so a dataset capture and a live flight cannot
# break each other.
#
#   Run order: launch-sitl.sh (spawn + Play) -> run THIS in the Script Editor.
#   Consumer: pipeline-streaming.py in the `drone` conda env.
# ============================================================================
import os, sys
import numpy as np
import cv2
import zmq
from isaacsim.core.api.world import World
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.sensors import IMU, Barometer
import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom

REPO_ROOT = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() \
    else os.path.expanduser("~/pai/drone-sitl")
for _p in (os.path.join(REPO_ROOT, "sim"), os.path.join(REPO_ROOT, "streaming")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import sites            # noqa: E402
import zmq_proto        # noqa: E402
from cam_extrinsics import (analytic_R_body_cam, to_pipeline_R_CtoI,  # noqa: E402
                            rot_angle_deg)

DATA_FPS   = 200       # imu + baro + gt rate (Hz), matching the recorder
FRAME_FPS  = 15        # image rate (Hz); flow-odom's best stride was ~10 fps
CAM_W, CAM_H = 960, 600
JPEG_QUALITY = 92      # visually lossless for feature tracking, ~5x smaller
META_EVERY_S = 1.0     # re-announce so a restarted estimator re-primes
PRINT_EVERY_S = 5.0

# ZMQ send high-water mark, deliberately SMALL. One PUB socket carries every
# topic, so this is one bound for all of them. If the estimator stalls we want
# ZMQ to drop old samples rather than build a backlog that would eventually
# feed PX4 a position from seconds ago -- a late pose is worse than no pose,
# which is the same reasoning as vision_bridge.VisionPositionSender.send.
SNDHWM = 200


# The extrinsic derivation lives in streaming/cam_extrinsics.py so it can be
# pinned against real recorded calibrations offline -- see
# test_cam_extrinsics.py, which runs this same 1 deg check without Isaac.

# --- locate vehicle + IMU + barometer + down_cam ---------------------------
vm = VehicleManager.get_vehicle_manager()
vehicles = list(vm.vehicles.values()) if getattr(vm, "vehicles", None) else []
world = World.instance()
stage = omni.usd.get_context().get_stage()


def find_cam_path(prim_name):
    return next((str(p.GetPath()) for p in stage.Traverse()
                 if p.IsA(UsdGeom.Camera) and p.GetName() == prim_name), None)


def find_body_prim():
    bodies = [p for p in stage.Traverse() if p.GetName() == "body"]
    for kw in ("px4_drone", "quadrotor", "iris", "drone", "multirotor"):
        b = next((b for b in bodies if kw in str(b.GetPath()).lower()), None)
        if b:
            return b
    return bodies[0] if bodies else None


cam_path = find_cam_path("down_cam")
imu = None
baro = None
veh = None
if vehicles:
    veh = vehicles[0]
    imu = next((s for s in veh._sensors if isinstance(s, IMU)), None)
    baro = next((s for s in veh._sensors if isinstance(s, Barometer)), None)

if not vehicles or imu is None:
    print("*** No drone/IMU. Launch through sim/launch-sitl.sh first. ***")
elif cam_path is None:
    print("*** down_cam not found on stage. ***")
elif world is None:
    print("*** No World instance. ***")
else:
    # clean up a previous streamer run (Script Editor re-runs are the norm)
    old = globals().get("_VIO_STREAM")
    if old:
        try: world.remove_physics_callback(old["cb"])
        except Exception: pass
        try: old["sock"].close(linger=0)
        except Exception: pass
        try: old["ctx"].term()
        except Exception: pass
        print(">>> replaced the previous vio-streamer run.")

    # --- site: the authored origin, NOT a read-back off the stage -----------
    # design D2. The site config is the source of truth and the stage is
    # derived from it, so reading the georeference back would only re-derive
    # what sites.py already states -- and would disagree with it if the stage
    # build had drifted.
    site_name = os.environ.get("SITL_SITE", "")
    site = sites.get_site(site_name) if site_name else None
    if site is None:
        print("*** SITL_SITE is not set -- cannot publish an origin. Launch "
              "through sim/launch-sitl.sh. ***")
        raise SystemExit

    # --- vibration mount: must be RIGID for the extrinsic to be constant ----
    setup_ns = globals().get("_DRONE_SETUP_NS") or {}
    vib_damp = setup_ns.get("DOWN_VIB_DAMP")
    if vib_damp is None:
        raw = os.environ.get("DRONE_SETUP_DOWN_VIB_DAMP")
        vib_damp = (raw.strip().lower() not in ("false", "0", "no")) if raw else True
    img_roll_deg = setup_ns.get("DOWN_IMG_ROLL_DEG")
    if img_roll_deg is None:
        raw = os.environ.get("DRONE_SETUP_DOWN_IMG_ROLL_DEG")
        img_roll_deg = float(raw) if raw else -90.0
    vib_damp = bool(vib_damp)
    if vib_damp:
        print("*** WARNING: DOWN_VIB_DAMP is ON. /World/down_mount low-passes the "
              "body attitude, so the camera<->IMU extrinsic is TIME-VARYING and "
              "the published R_CtoI is only an approximation. Relaunch with "
              "DRONE_SETUP_DOWN_VIB_DAMP=False for VIO. ***")

    # --- extrinsic: analytic, then cross-checked against the live stage -----
    R_body_cam = analytic_R_body_cam(img_roll_deg)
    R_CtoI = to_pipeline_R_CtoI(R_body_cam)
    xc = UsdGeom.XformCache()
    body_prim = find_body_prim()
    cam_prim = stage.GetPrimAtPath(cam_path)
    if body_prim is not None and cam_prim.IsValid():
        try:
            xc.Clear()
            Rb = np.array(xc.GetLocalToWorldTransform(body_prim).ExtractRotationMatrix()).T
            Rc = np.array(xc.GetLocalToWorldTransform(cam_prim).ExtractRotationMatrix()).T
            live = Rb.T @ Rc                       # body -> cam, as the stage has it
            err = rot_angle_deg(R_body_cam, live)
            if err > 1.0:
                print(f"*** WARNING: analytic camera extrinsic disagrees with the live "
                      f"down_mount transform by {err:.2f} deg. Every flow vector will be "
                      f"mis-rotated by that much. Expected <1 deg with a rigid mount"
                      f"{' -- but DOWN_VIB_DAMP is ON, which explains it' if vib_damp else ''}. ***")
            else:
                print(f">>> extrinsic cross-check OK ({err:.3f} deg vs the live stage)")
        except Exception as exc:
            print(f"*** extrinsic cross-check could not run: {exc!r} ***")

    # --- camera intrinsics, read live off the prim -------------------------
    uc = UsdGeom.Camera(cam_prim)
    focal = uc.GetFocalLengthAttr().Get()
    h_ap = uc.GetHorizontalApertureAttr().Get()
    v_ap = uc.GetVerticalApertureAttr().Get()
    fx = focal / h_ap * CAM_W
    fy = focal / v_ap * CAM_H
    K = [[fx, 0.0, CAM_W / 2.0], [0.0, fy, CAM_H / 2.0], [0.0, 0.0, 1.0]]
    print(f">>> down_cam optics: focal={focal:.3f}mm -> {CAM_W}x{CAM_H} "
          f"fx={fx:.2f} fy={fy:.2f}px")

    rp = rep.create.render_product(cam_path, (CAM_W, CAM_H))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])

    # --- ZMQ ---------------------------------------------------------------
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, SNDHWM)
    sock.bind(f"tcp://*:{zmq_proto.SENSOR_PORT}")
    print(f">>> vio-streamer PUB on tcp://*:{zmq_proto.SENSOR_PORT}")

    META = {
        "site": site.name,
        "origin": {"lat": site.latitude, "lon": site.longitude, "h": site.height},
        "heading_deg": site.heading_deg,
        "ground_z": site.ground_z,
        "K": K,
        "R_CtoI": [list(map(float, r)) for r in R_CtoI],
        "image_size": [CAM_W, CAM_H],
        "vib_damp": vib_damp,
        "img_roll_deg": float(img_roll_deg),
        "data_fps": DATA_FPS,
        "frame_fps": FRAME_FPS,
    }

    IMG_EVERY = max(1, round(DATA_FPS / max(1, FRAME_FPS)))
    st = {"step": 0, "decim": None, "img_every": IMG_EVERY, "frame": 0,
          "n_img": 0, "n_img_empty": 0, "last_meta": -1e9, "last_print": -1e9,
          "anchor": None}

    def _send(topic, payload):
        """Never let a publish failure escape into the physics callback.

        An exception here would abort the rest of the tick -- which is exactly
        how the recorder came to write full CSVs and zero images (SESSION.md).
        """
        try:
            sock.send_multipart(zmq_proto.pack(topic, payload), zmq.NOBLOCK)
            return True
        except zmq.Again:
            return False            # HWM reached: dropping is the intended behaviour
        except Exception as exc:
            print(f"*** vio-streamer publish failed on {topic!r}: {exc!r} ***")
            return False

    def _on_phys(dt):
        now = world.current_time
        if st["decim"] is None:
            st["decim"] = max(1, round((1.0 / DATA_FPS) / max(dt, 1e-9)))
            actual = 1.0 / (dt * st["decim"])
            st["img_every"] = max(1, round(actual / max(1, FRAME_FPS)))
            print(f">>> actual data rate ~{actual:.0f} Hz; frame every "
                  f"{st['img_every']} samples -> ~{actual/st['img_every']:.1f} img/s")
        st["step"] += 1
        if st["step"] % st["decim"] != 0:
            return

        st["frame"] += 1
        fr = st["frame"]
        ts_ns = int(now * 1e9)

        if now - st["last_meta"] >= META_EVERY_S:
            st["last_meta"] = now
            _send(zmq_proto.TOPIC_META, META)

        si = imu.state
        w = si.get("angular_velocity", (0.0, 0.0, 0.0))
        a = si.get("linear_acceleration", (0.0, 0.0, 0.0))
        _send(zmq_proto.TOPIC_IMU, {
            "frame": fr, "ts_ns": ts_ns,
            "w": [float(w[0]), float(w[1]), float(w[2])],
            "a": [float(a[0]), float(a[1]), float(a[2])]})

        if baro is not None:
            sb = baro.state
            _send(zmq_proto.TOPIC_BARO, {
                "frame": fr, "ts_ns": ts_ns,
                "pressure_hpa": float(sb.get("absolute_pressure", 0.0)),
                "alt_m": float(sb.get("pressure_altitude", 0.0)),
                "temp_c": float(sb.get("temperature", 0.0))})

        # Ground truth. SCORING ONLY -- design D4. The estimator must never let
        # this reach the estimate, and Step 5.2 pins that by rerunning with the
        # topic suppressed and requiring a bit-identical result.
        ss = veh.state
        p = np.asarray(ss.position, dtype=float)
        q = ss.attitude                             # xyzw, FLU in ENU
        if st["anchor"] is None:
            st["anchor"] = p.copy()
        pr = p - st["anchor"]
        _send(zmq_proto.TOPIC_GT, {
            "frame": fr, "ts_ns": ts_ns,
            "p": [float(pr[0]), float(pr[1]), float(pr[2])],
            "p_world": [float(p[0]), float(p[1]), float(p[2])],
            "q": [float(q[0]), float(q[1]), float(q[2]), float(q[3])]})

        if fr % st["img_every"] == 0:
            # NO rep.orchestrator.step() here. It raises "Synchronous call to
            # `step` can only be performed in a standalone workflow" on Isaac
            # Sim 6 / omni.replicator.core 1.13.25, and because the exception
            # escapes before any camera is read, EVERY frame is lost -- that is
            # exactly how the recorder came to write zero images (SESSION.md,
            # commit 7b03c00). The cost of leaving it out is that image content
            # lags its own timestamp by about one render period; that skew is
            # accepted and belongs to the timestamp side to fix, not to driving
            # the renderer from inside a physics callback (plan Step 4.3).
            try:
                data = annot.get_data()
            except Exception as exc:
                data = None
                print(f"*** annotator read failed: {exc!r} ***")
            if data is not None and getattr(data, "size", 0) > 0:
                rgb = np.asarray(data)[:, :, :3]
                gray = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2GRAY)
                ok, buf = cv2.imencode(".jpg", gray,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if ok and _send(zmq_proto.TOPIC_FRAME, {
                        "frame": fr, "frame_idx": st["n_img"], "ts_ns": ts_ns,
                        "jpg": buf.tobytes()}):
                    st["n_img"] += 1
            else:
                st["n_img_empty"] += 1

        if now - st["last_print"] >= PRINT_EVERY_S:
            st["last_print"] = now
            empty = f" EMPTY_READS={st['n_img_empty']}" if st["n_img_empty"] else ""
            print(f"[VIO-STREAM] t={now:7.1f}s frame={fr} images={st['n_img']}{empty}")

    world.add_physics_callback("vio_streamer", _on_phys)
    globals()["_VIO_STREAM"] = {"cb": "vio_streamer", "sock": sock, "ctx": ctx,
                                "st": st, "meta": META}
    _send(zmq_proto.TOPIC_META, META)
    print(f">>> vio-streamer running. site={site.name} "
          f"origin=({site.latitude}, {site.longitude}, {site.height}) "
          f"vib_damp={vib_damp}")
