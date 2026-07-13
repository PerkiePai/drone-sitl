# ============================================================================
# PX4 + Cesium all-in-one:  spawn the drone matched to the Cesium map
# (lat/lon/alt + heading) AND attach the two onboard ZED cameras in one run.
#
#   Window > Script Editor > paste > Run (Ctrl+Enter)  -- with your Cesium (NY)
#   stage loaded and the sim STOPPED. Then press Play.
#
#   1. Reads the CesiumGeoreference origin off the stage -> sets the PX4 GPS
#      origin so QGC shows the drone at the exact same place as the tiles.
#   2. Spawns the PX4 drone with a chosen compass HEADING.
#   3. Adds two ZED X One Wide cameras (down_cam fixed + detect_cam movable with
#      keyboard pan/tilt) to the freshly spawned drone.
#
#   PX4 only. Does NOT touch ArduPilot or configs.yaml.
# ============================================================================
import asyncio
from scipy.spatial.transform import Rotation
from isaacsim.core.api.world import World
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, PX4MavlinkBackendConfig)

import omni.usd

# ----------------------------------------------------------------------------
# Tunables
HEADING_DEG        = 0.0     # desired compass heading: 0=North, 90=East, 180=S, 270=W
HEADING_OFFSET_DEG = 0.0     # if QGC heading is off by a constant, correct it here
SPAWN_XYZ          = [0.0, 0.0, 0.5]   # local meters from origin (x=E, y=N, z=Up)
VEHICLE_ID         = 0
ROBOT_MODEL        = "Iris"            # visual asset key in ROBOTS
PX4_AUTOLAUNCH     = True              # let Pegasus start PX4 SITL (False = launch it yourself)
ADD_CAMERAS        = True              # attach the two ZED cameras after spawn
CAM_ROLL_DEG       = -90.0             # de-rotate image. -90 corrects the "forward goes right" roll (was +90 = upside down/180).
DOWN_STABILIZE     = False             # VIO: body-rigid (tilts with drone) — constant extrinsic, required by OpenVINS.
DOWN_FOLLOW_YAW    = True              # True = heading follows the drone yaw (turn 0->90 rotates the image 90). False = locked North-up.
DOWN_YAW_SIGN      = 1.0               # flip to -1.0 if the image rotates opposite to the turn
DOWN_IMG_ROLL_DEG  = -90.0             # constant roll offset of the down image (deg); -90 fixes the CCW-90 mount
DOWN_Z_OFFSET      = -0.05             # down_cam height offset below the drone body (m)
ADD_FPV            = True              # add a body-rigid forward FPV camera (cam1) for VIO alongside down_cam (cam0)
FPV_TILT_DEG       = -75.0            # FPV pitch: 0=straight down, -90=straight forward; -75 = forward + 15 deg down
FPV_FWD_OFFSET     = 0.12             # FPV mount distance forward of body centre (m)
STREAM_CAMERAS     = False             # KEEP FALSE WHILE RECORDING: live MJPEG streams create extra render products that starve the recorder's cam1 (FPV) capture (~76% dropped at 30fps). Set True only for live viewing when NOT recording a VIO dataset.
STREAM_PORT        = 8080
STREAM_W, STREAM_H = 640, 400          # streamed resolution (per camera)
STREAM_FPS         = 20                # max encode/stream rate
RECORD_KEY         = "R"               # press R (focus a viewport) to start/stop MP4 recording of BOTH cams
REC_W, REC_H       = 1280, 800         # recording resolution (per camera) — higher than the stream
REC_FPS            = 30                # recording frame rate
REC_DIR            = "~/flight_recordings"   # MP4s saved here, timestamped per camera

# Manual fallback if the Cesium georeference can't be read off the stage:
FALLBACK_LAT, FALLBACK_LON, FALLBACK_ALT = 40.7128, -74.0060, 10.0
# ----------------------------------------------------------------------------


def read_cesium_georeference():
    """Return (lat, lon, height) from the CesiumGeoreference prim, or None."""
    stage = omni.usd.get_context().get_stage()
    lat_names = ["cesium:georeferenceOrigin:latitude",  "georeferenceOrigin:latitude"]
    lon_names = ["cesium:georeferenceOrigin:longitude", "georeferenceOrigin:longitude"]
    alt_names = ["cesium:georeferenceOrigin:height",    "georeferenceOrigin:height"]

    def first_attr(prim, names):
        for n in names:
            a = prim.GetAttribute(n)
            if a and a.IsValid() and a.Get() is not None:
                return a.Get()
        return None

    for prim in stage.Traverse():
        tname = prim.GetTypeName()
        if "CesiumGeoreference" in str(tname) or "Georeference" in prim.GetName():
            lat = first_attr(prim, lat_names)
            lon = first_attr(prim, lon_names)
            alt = first_attr(prim, alt_names)
            if lat is not None and lon is not None:
                print(f">>> Cesium georeference found at {prim.GetPath()}: "
                      f"lat={lat}, lon={lon}, height={alt}")
                return float(lat), float(lon), float(alt if alt is not None else FALLBACK_ALT)
    return None


def setup_cameras():
    """Attach down_cam (fixed) + detect_cam (movable, keyboard pan/tilt) to the
    spawned drone. Must run while the sim is STOPPED."""
    import omni.timeline, omni.appwindow, omni.kit.app
    import carb, carb.input
    from pxr import UsdGeom, Gf, Sdf
    import math

    tl = omni.timeline.get_timeline_interface()
    if tl.is_playing():
        print("# STOP the sim first, then re-run (cameras can't be added while playing).")
        return

    stage = omni.usd.get_context().get_stage()

    # find the drone body prim
    bodies = [str(p.GetPath()) for p in stage.Traverse() if p.GetName() == "body"]
    BODY_PATH = None
    for kw in ("px4_drone", "quadrotor", "iris", "drone", "multirotor"):
        for b in bodies:
            if kw in b.lower():
                BODY_PATH = b
                break
        if BODY_PATH:
            break
    if BODY_PATH is None and bodies:
        BODY_PATH = bodies[0]
    if BODY_PATH is None:
        print("*** Could not find a drone 'body' prim to attach cameras. ***")
        return
    print(f">>> Attaching cameras to drone body: {BODY_PATH}")

    def _cam(path, focal=15.0, h_ap=20.955, v_ap=None):
        c = UsdGeom.Camera.Define(stage, path)
        c.GetFocalLengthAttr().Set(focal)
        c.GetHorizontalApertureAttr().Set(h_ap)
        if v_ap is not None:
            c.GetVerticalApertureAttr().Set(v_ap)
        c.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100000.0))
        return c

    def _ops(prim):
        xf = UsdGeom.Xformable(prim); xf.ClearXformOpOrder()
        return xf.AddTranslateOp(), xf.AddRotateXYZOp()

    # ZED X One GS Wide optics (applied to both cameras). Native sensor 1920x1200 (16:10).
    ZED_MODEL    = "GS"
    ZED_FOCAL_MM = 2.2
    ZED_HFOV     = 110.0                          # horizontal FOV (deg) — authoritative
    ZED_SENSOR_W, ZED_SENSOR_H = 1920, 1200       # native resolution -> defines pixel aspect
    zed_h_ap = 2 * ZED_FOCAL_MM * math.tan(math.radians(ZED_HFOV / 2.0))
    # derive vertical aperture from the sensor aspect so pixels are SQUARE (not anamorphic);
    # at 16:10 this gives VFOV ~= 83.5 deg, consistent across stream/record/VIO capture.
    zed_v_ap = zed_h_ap * (ZED_SENSOR_H / ZED_SENSOR_W)

    def _zedcam(path):
        return _cam(path, focal=ZED_FOCAL_MM, h_ap=zed_h_ap, v_ap=zed_v_ap)

    # NADIR-STABILIZED downward camera.
    # Mounted on a TOP-LEVEL gimbal (/World/down_gimbal), NOT under the drone body,
    # so it never inherits the drone's yaw/pitch/roll. A per-frame callback copies
    # only the body's world POSITION; the gimbal orientation stays locked looking
    # straight down (world -Z). Result: always points down, no matter the heading.
    body_prim = stage.GetPrimAtPath(BODY_PATH)
    if DOWN_STABILIZE:
        dgim = UsdGeom.Xform.Define(stage, "/World/down_gimbal"); dgt, dgr = _ops(dgim)
        dgr.Set(Gf.Vec3f(0.0, 0.0, DOWN_IMG_ROLL_DEG))   # fixed nadir orientation (+image roll)
        dwn_path = "/World/down_gimbal/down_cam"
        dwn = _zedcam(dwn_path); ddt, ddr = _ops(dwn)
        ddt.Set(Gf.Vec3d(0.0, 0.0, 0.0)); ddr.Set(Gf.Vec3f(0.0, 0.0, 0.0))  # camera looks local -Z = world down
    else:
        # body-rigid downward camera: constant extrinsic, required for VIO
        dwn_path = f"{BODY_PATH}/down_cam"
        dwn = _zedcam(dwn_path); dt, dr = _ops(dwn)
        dt.Set(Gf.Vec3d(0.0, 0.0, -0.05)); dr.Set(Gf.Vec3f(0.0, 0.0, CAM_ROLL_DEG))
        dgt = None

    # body-rigid FORWARD FPV camera (cam1): the second independent monocular VIO
    # sensor (CLAUDE.md north star). FIXED mount = constant extrinsic = VIO-valid
    # (unlike the movable detect_cam). Geometry mirrors detect_cam — a fixed tilt
    # node (rotate Y) sets the forward look, and the roll sits on the camera so the
    # image stays upright about the optical axis. A camera looks down its local -Z,
    # so tilt 0=down, -90=forward; FPV_TILT_DEG=-75 -> forward + 15 deg down.
    fpv_path = None
    if ADD_FPV:
        fpv_mnt = UsdGeom.Xform.Define(stage, f"{BODY_PATH}/fpv_mount"); fmt, fmr = _ops(fpv_mnt)
        fmt.Set(Gf.Vec3d(FPV_FWD_OFFSET, 0.0, 0.0))
        fmr.Set(Gf.Vec3f(0.0, FPV_TILT_DEG, 0.0))
        fpv_path = f"{BODY_PATH}/fpv_mount/fpv_cam"
        fpv = _zedcam(fpv_path); fct, fcr = _ops(fpv)
        fct.Set(Gf.Vec3d(0.0, 0.0, 0.0)); fcr.Set(Gf.Vec3f(0.0, 0.0, CAM_ROLL_DEG))

    # movable detect camera: pan gimbal (Z) -> tilt node (Y) -> camera (fixed roll Z).
    # Putting tilt on a parent node and the roll on the camera keeps the roll about
    # the OPTICAL axis at every pan/tilt, so the image stays level.
    gim = UsdGeom.Xform.Define(stage, f"{BODY_PATH}/detect_gimbal"); gt, gr = _ops(gim)
    gt.Set(Gf.Vec3d(0.12, 0.0, -0.03))
    tilt_node = UsdGeom.Xform.Define(stage, f"{BODY_PATH}/detect_gimbal/detect_tilt"); tt, tr = _ops(tilt_node)
    tt.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    det_path = f"{BODY_PATH}/detect_gimbal/detect_tilt/detect_cam"
    dc = _zedcam(det_path); ct, cr = _ops(dc)
    cr.Set(Gf.Vec3f(0.0, 0.0, CAM_ROLL_DEG))   # fixed roll about optical axis (de-rotate image)

    st = {"pan": 0.0, "tilt": -60.0}   # tilt 0=down, -90=forward; -60 = forward+30 down
    def _apply():
        gr.Set(Gf.Vec3f(0.0, 0.0, st["pan"]))
        tr.Set(Gf.Vec3f(0.0, max(-95.0, min(5.0, st["tilt"])), 0.0))
    _apply()

    ok_d = stage.GetPrimAtPath(dwn_path).IsValid()
    ok_t = stage.GetPrimAtPath(det_path).IsValid()
    ok_f = bool(fpv_path) and stage.GetPrimAtPath(fpv_path).IsValid()
    print(f"down_cam created: {ok_d}   detect_cam created: {ok_t}   "
          f"fpv_cam created: {ok_f if ADD_FPV else 'skipped'}")
    if not (ok_d and ok_t):
        print("*** Camera creation FAILED. ***")
        return
    if ADD_FPV and not ok_f:
        print("*** WARN: fpv_cam creation failed — recorder will fall back to cam0 only. ***")

    # keyboard pan/tilt for detect_cam
    def _on_key(e):
        if e.type not in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT):
            return True
        K = carb.input.KeyboardInput; k = e.input; step = 5.0
        if   k == K.I: st["tilt"] -= step
        elif k == K.K: st["tilt"] += step
        elif k == K.J: st["pan"]  += step
        elif k == K.L: st["pan"]  -= step
        elif k == K.U: st["pan"] = 0.0; st["tilt"] = 0.0
        elif k == K.O: st["pan"] = 0.0; st["tilt"] = -60.0
        else: return True
        _apply()
        print(f"[detect_cam] pan={st['pan']:.0f}  tilt={st['tilt']:.0f}")
        return True

    iface = carb.input.acquire_input_interface()
    kb = omni.appwindow.get_default_app_window().get_keyboard()
    prev = globals().get("_CAM_KEY_SUB")
    if prev is not None:
        try: iface.unsubscribe_to_keyboard_events(kb, prev)
        except Exception: pass
    globals()["_CAM_KEY_SUB"] = iface.subscribe_to_keyboard_events(kb, _on_key)
    globals()["_CAM_ON_KEY"]  = _on_key

    from omni.kit.viewport.utility import create_viewport_window
    create_viewport_window("DownCam (cam0, ZED Wide)", camera_path=Sdf.Path(dwn_path),
                           width=512, height=320, position_x=40, position_y=40)
    create_viewport_window("DetectCam (movable, ZED Wide)", camera_path=Sdf.Path(det_path),
                           width=512, height=320, position_x=560, position_y=40)
    if ok_f:
        create_viewport_window("FpvCam (cam1, forward, ZED Wide)", camera_path=Sdf.Path(fpv_path),
                               width=512, height=320, position_x=40, position_y=380)
    print("Cameras ready. Click inside DetectCam, then: I/K=tilt, J/L=pan, U=down, O=default.")

    # nadir stabilization: each app update, move the top-level down_gimbal to the
    # drone body's LIVE world position (orientation stays locked looking straight
    # down — top-level gimbal never inherits the drone's attitude, so we only need
    # the body POSITION, not its rotation).
    if DOWN_STABILIZE and dgt is not None and body_prim and body_prim.IsValid():
        def _build_pose_getter():
            # returns a fn -> (x, y, z, yaw_deg) of the drone body.
            # prefer the physics/fabric-aware core API (live during play); fall back
            # to a USD XformCache read (can be stale under the Fabric delegate).
            import numpy as _np
            def _yaw_from_fwd(fx, fy):
                return math.degrees(math.atan2(fy, fx))
            for _mod in ("isaacsim.core.prims", "omni.isaac.core.prims"):
                try:
                    XFormPrim = __import__(_mod, fromlist=["XFormPrim"]).XFormPrim
                    vp = XFormPrim(BODY_PATH)
                    def _g():
                        p, q = vp.get_world_poses()
                        p = _np.asarray(p).reshape(-1)
                        q = _np.asarray(q).reshape(-1)              # quaternion w,x,y,z
                        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
                        f = r.apply([1.0, 0.0, 0.0])               # body forward (+X) in world
                        return float(p[0]), float(p[1]), float(p[2]), _yaw_from_fwd(f[0], f[1])
                    _g()  # smoke test
                    print(f">>> down_gimbal pose source: {_mod}.XFormPrim (live)")
                    return _g
                except Exception:
                    continue
            xc = UsdGeom.XformCache()
            def _g():
                xc.Clear()
                m = xc.GetLocalToWorldTransform(body_prim)
                t = m.ExtractTranslation()
                f = m.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))        # body forward (+X) in world
                return float(t[0]), float(t[1]), float(t[2]), _yaw_from_fwd(f[0], f[1])
            print(">>> down_gimbal pose source: USD XformCache (fallback)")
            return _g

        _get_pose = _build_pose_getter()
        def _stabilize(e):
            try:
                x, y, z, yaw = _get_pose()
                dgt.Set(Gf.Vec3d(x, y, z + DOWN_Z_OFFSET))
                if DOWN_FOLLOW_YAW:
                    dgr.Set(Gf.Vec3f(0.0, 0.0, DOWN_YAW_SIGN * yaw + DOWN_IMG_ROLL_DEG))
            except Exception:
                pass
        prev_stab = globals().get("_DOWN_GIMBAL_SUB")
        if prev_stab is not None:
            try: prev_stab.unsubscribe()
            except Exception: pass
        globals()["_DOWN_GIMBAL_SUB"] = omni.kit.app.get_app().get_update_event_stream(
            ).create_subscription_to_pop(_stabilize, name="down_gimbal_stab")
        print(">>> down_cam NADIR gimbal: stays straight down (pitch/roll stabilized); "
              f"heading {'follows drone yaw' if DOWN_FOLLOW_YAW else 'locked North-up'}.")

    if STREAM_CAMERAS:
        stream_cams = {"down": dwn_path, "detect": det_path}
        if ok_f:
            stream_cams["fpv"] = fpv_path
        start_camera_streams(stream_cams)


def start_camera_streams(cam_paths):
    """Grab RGB from each camera's render product and serve both as MJPEG over
    HTTP. View in any LAN browser at http://<box-ip>:STREAM_PORT/.
    Frame grab + JPEG encode happen on Isaac's main thread (app-update callback);
    HTTP worker threads only read the latest encoded bytes."""
    import threading, time, io, os, subprocess
    import numpy as np
    import omni.kit.app, omni.appwindow
    import carb, carb.input
    import omni.replicator.core as rep
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    try:
        from PIL import Image
    except Exception:
        print("*** Pillow (PIL) not available in Isaac's python — can't JPEG-encode. "
              "Run in Isaac's python:  <isaac>/python.sh -m pip install pillow  ***")
        return

    # shut down a previous run's server/callback/recording if the script is re-run
    old = globals().get("_CAM_STREAM")
    if old:
        try: old.get("stop_rec", lambda: None)()
        except Exception: pass
        try: old["httpd"].shutdown()
        except Exception: pass
        try: old["sub"].unsubscribe()
        except Exception: pass
        try: old["iface"].unsubscribe_to_keyboard_events(old["kb"], old["keysub"])
        except Exception: pass

    latest = {k: b"" for k in cam_paths}          # latest JPEG bytes per camera
    annots = {}
    for key, path in cam_paths.items():
        rp = rep.create.render_product(path, (STREAM_W, STREAM_H))
        a = rep.AnnotatorRegistry.get_annotator("rgb")
        a.attach([rp])
        annots[key] = a

    # --- MP4 recording (separate, higher-res render products; lazy-created) ----
    rec_dir = os.path.expanduser(REC_DIR)
    rec = {"on": False, "procs": {}, "annots": {}, "paths": {}}

    def _ensure_rec_annots():
        if rec["annots"]:
            return
        for key, path in cam_paths.items():
            rp = rep.create.render_product(path, (REC_W, REC_H))
            a = rep.AnnotatorRegistry.get_annotator("rgb")
            a.attach([rp])
            rec["annots"][key] = a

    def _start_recording():
        os.makedirs(rec_dir, exist_ok=True)
        _ensure_rec_annots()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        for key in cam_paths:
            out = os.path.join(rec_dir, f"{key}_{stamp}.mp4")
            cmd = ["/usr/bin/ffmpeg", "-y", "-loglevel", "error",
                   "-f", "rawvideo", "-pix_fmt", "rgb24",
                   "-s", f"{REC_W}x{REC_H}", "-r", str(REC_FPS), "-i", "-",
                   "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                   "-preset", "veryfast", out]
            rec["procs"][key] = subprocess.Popen(cmd, stdin=subprocess.PIPE)
            rec["paths"][key] = out
        state["last_rec"] = 0.0
        rec["on"] = True
        print(f">>> RECORDING started -> {', '.join(rec['paths'].values())}")

    def _stop_recording():
        if not rec["on"] and not rec["procs"]:
            return
        rec["on"] = False
        for key, p in list(rec["procs"].items()):
            try:
                p.stdin.close(); p.wait(timeout=30)
            except Exception:
                try: p.kill()
                except Exception: pass
        saved = list(rec["paths"].values())
        rec["procs"].clear(); rec["paths"].clear()
        print(f">>> RECORDING stopped. Saved MP4s: {saved}")

    state = {"last_stream": 0.0, "last_rec": 0.0}
    stream_dt = 1.0 / max(1, STREAM_FPS)
    rec_dt = 1.0 / max(1, REC_FPS)

    def _on_update(e):
        now = time.time()
        # streaming (throttled to STREAM_FPS)
        if now - state["last_stream"] >= stream_dt:
            state["last_stream"] = now
            for key, a in annots.items():
                try:
                    data = a.get_data()
                    if data is None or getattr(data, "size", 0) == 0:
                        continue
                    rgb = np.asarray(data)[:, :, :3]          # drop alpha
                    buf = io.BytesIO()
                    Image.fromarray(rgb).save(buf, format="JPEG", quality=70)
                    latest[key] = buf.getvalue()
                except Exception:
                    pass
        # recording (throttled to REC_FPS, raw frames piped to ffmpeg)
        if rec["on"] and now - state["last_rec"] >= rec_dt:
            state["last_rec"] = now
            for key, a in rec["annots"].items():
                proc = rec["procs"].get(key)
                if proc is None:
                    continue
                try:
                    data = a.get_data()
                    if data is None or getattr(data, "size", 0) == 0:
                        continue
                    rgb = np.ascontiguousarray(np.asarray(data)[:, :, :3], dtype=np.uint8)
                    if rgb.shape[:2] != (REC_H, REC_W):
                        continue
                    proc.stdin.write(rgb.tobytes())
                except Exception:
                    pass

    sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        _on_update, name="cam_mjpeg_stream")

    # keyboard: toggle recording with RECORD_KEY
    rec_key_enum = getattr(carb.input.KeyboardInput, RECORD_KEY, carb.input.KeyboardInput.R)
    def _rec_key(ev):
        if ev.type == carb.input.KeyboardEventType.KEY_PRESS and ev.input == rec_key_enum:
            _stop_recording() if rec["on"] else _start_recording()
        return True
    _iface = carb.input.acquire_input_interface()
    _kb = omni.appwindow.get_default_app_window().get_keyboard()
    keysub = _iface.subscribe_to_keyboard_events(_kb, _rec_key)

    cams = list(cam_paths.keys())

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):   # silence per-request logging
            pass

        def do_GET(self):
            p = self.path.split("?")[0].strip("/")
            if p in ("", "index.html"):
                imgs = "".join(
                    f"<div style='text-align:center'><div style='color:#ccc;font:14px sans-serif'>{c}</div>"
                    f"<img src='/{c}' style='width:48vw;max-width:720px'></div>" for c in cams)
                html = ("<html><body style='margin:0;background:#111;display:flex;"
                        "gap:8px;justify-content:center;align-items:center;height:100vh'>"
                        + imgs + "</body></html>").encode()
                self.send_response(200); self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(html))); self.end_headers()
                self.wfile.write(html); return
            if p in latest:
                self.send_response(200)
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        jpeg = latest[p]
                        if jpeg:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                        time.sleep(stream_dt)
                except (BrokenPipeError, ConnectionResetError, ValueError):
                    return
                return
            self.send_response(404); self.end_headers()

    httpd = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), H)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    globals()["_CAM_STREAM"] = {"httpd": httpd, "sub": sub, "keysub": keysub,
                                "kb": _kb, "iface": _iface, "stop_rec": _stop_recording}
    print(f">>> Camera MJPEG server on http://0.0.0.0:{STREAM_PORT}/  "
          f"(open from the Mac at http://<box-ip>:{STREAM_PORT}/ ; "
          f"single feeds: /{cams[0]} , /{cams[1]})")
    print(f">>> RECORD: focus a viewport and press '{RECORD_KEY}' to start/stop MP4 "
          f"recording of both cams -> {rec_dir}/ ({REC_W}x{REC_H} @ {REC_FPS}fps)")


pg = PegasusInterface()

# 1) Match the GPS origin to the Cesium map -----------------------------------
geo = read_cesium_georeference()
if geo is None:
    print("*** CesiumGeoreference not found on stage — using FALLBACK coords. "
          "Load your Cesium tiles first, or set FALLBACK_* above. ***")
    lat, lon, alt = FALLBACK_LAT, FALLBACK_LON, FALLBACK_ALT
else:
    lat, lon, alt = geo
pg.set_global_coordinates(latitude=lat, longitude=lon, altitude=alt)
print(f">>> Pegasus GPS origin set to: {lat}, {lon}, {alt}")
# Stash for the VIO recorder so geo.csv/georef.json use the EXACT same ENU origin
# as the PX4 GPS (the local world origin (0,0,0) maps to this lat/lon/height).
globals()["_CESIUM_GEOREF"] = {"lat": lat, "lon": lon, "alt": alt,
                               "is_fallback": geo is None}

# 2) Compass heading -> Isaac ENU yaw (yaw=0 -> East; yaw = 90 - heading)
isaac_yaw_deg = 90.0 - HEADING_DEG + HEADING_OFFSET_DEG


async def _spawn_px4_keep_stage():
    pg._world = World(**pg._world_settings)
    await pg._world.initialize_simulation_context_async()
    pg._world = World.instance()

    cfg = PX4MavlinkBackendConfig({
        "vehicle_id": VEHICLE_ID,
        "px4_autolaunch": PX4_AUTOLAUNCH,
        "px4_dir": pg.px4_path,
    })
    vcfg = MultirotorConfig()
    vcfg.backends = [PX4MavlinkBackend(cfg)]

    Multirotor(
        f"/World/px4_drone{VEHICLE_ID}",
        ROBOTS[ROBOT_MODEL],
        VEHICLE_ID,
        SPAWN_XYZ,
        Rotation.from_euler("XYZ", [0.0, 0.0, isaac_yaw_deg], degrees=True).as_quat(),
        config=vcfg,
    )

    await pg._world.reset_async()
    await pg._world.stop_async()
    print(f">>> PX4 drone spawned. Heading={HEADING_DEG} deg (Isaac yaw={isaac_yaw_deg:.1f}).")

    # 3) attach cameras now (sim is stopped) ---------------------------------
    if ADD_CAMERAS:
        setup_cameras()

    print(">>> Done. Press Play, then connect QGC.")
    print("    If QGC heading is rotated vs the map, nudge HEADING_OFFSET_DEG "
          "(usually +/-90 or 180) and re-run.")

asyncio.ensure_future(_spawn_px4_keep_stage())
