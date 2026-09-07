# ============================================================================
# PX4 + Cesium all-in-one:  spawn the drone matched to the Cesium map
# (lat/lon/alt + heading), un-flip the ground plane, add realistic gusty wind,
# AND attach the onboard ZED cameras in one run.
#
#   Window > Script Editor > paste > Run (Ctrl+Enter)  -- with your Cesium (NY)
#   stage loaded and the sim STOPPED. Then press Play.
#
#   1. Reads the CesiumGeoreference origin off the stage -> sets the PX4 GPS
#      origin so QGC shows the drone at the exact same place as the tiles.
#   2. Spawns the PX4 drone with a chosen compass HEADING.
#   3. Adds the DOWN camera as a real ZED X One GS (global-shutter, 2.2mm Wide),
#      HARD-MOUNTED with no gimbal + a soft anti-vibration mount, plus a movable
#      detect_cam (keyboard pan/tilt), and a free-standing CHASE camera that
#      trails/frames the drone from behind, to the freshly spawned drone.
#   4. Un-flips the Cesium-tipped ground plane and adds lockstep-safe gusty wind.
#
#   PX4 only. Does NOT touch ArduPilot or configs.yaml.
# ============================================================================
import asyncio
import time
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
SPAWN_XYZ          = [0.0, 0.0, 0.5]   # local meters from origin (x=E, y=N, z=Up).
                                       # Set z yourself to sit just above YOUR ground plane.
# --- ground-plane un-flip ---------------------------------------------------
FIX_GROUND_FLIP    = False             # the stage builder authors the plane flat in a Z-up
                                       # stage, so there is nothing to un-flip and running
                                       # fix_ground_plane() would BREAK a correct plane by
                                       # forcing rotateX=90. Set True only for the
                                       # paste-into-Script-Editor path on a hand-built
                                       # Y-up stage, which is what it was always for: a
                                       # Z-up-authored plane only LOOKS tipped in a Y-up
                                       # stage — the Cesium georeference never tipped it.
GROUND_FLIP_DEG    = 90.0             # corrective rotation about X (deg). Try -90.0 if it tips the wrong way.
# --- wind (applied in Isaac — PX4 cannot inject wind into Pegasus) -----------
ADD_WIND           = False      # add wind via Pegasus's own drag path (lockstep-safe; won't break QGC).
                                # OFF for joystick-offboard bring-up: a random-bearing 5 m/s gusting wind
                                # makes the drone crab sideways while you hold "forward", which muddies the
                                # one thing that PoC is meant to show. Re-enable it afterwards as the
                                # robustness demo (holds heading + velocity through gusts).
WIND_SPEED_MS      = 5.0        # MEAN wind speed (m/s) — gusts vary around this
WIND_FROM_DEG      = None       # MEAN direction wind blows FROM (deg; 270 = from West). None = random each run.
WIND_DIR_WANDER_DEG= 45.0       # how far the mean direction slowly drifts over the flight (± deg; 0 = fixed heading)
WIND_GUST_FRAC     = 0.5        # gust strength as a fraction of mean speed (0.5 = gusts ±~50%)
WIND_GUST_TAU_S    = 3.0        # gust correlation time (s): smaller = choppier, larger = longer swells
WIND_COEFF         = 0.8        # drag coefficient — how hard the wind pushes
WIND_MIN_AGL_M     = 1.0        # wind engages only after the drone climbs this far (grounded drone untouched)
WIND_SEED          = None       # int → reproducible wind for a dataset; None → different every run
VEHICLE_ID         = 0
ROBOT_MODEL        = "Iris"            # visual asset key in ROBOTS
PX4_AUTOLAUNCH     = True              # let Pegasus start PX4 SITL (False = launch it yourself)
ADD_CAMERAS        = True              # attach the ZED cameras after spawn
CAM_ROLL_DEG       = -90.0             # de-rotate image. -90 corrects the "forward goes right" roll (was +90 = upside down/180).
# --- DOWN camera: ZED X One GS, HARD-MOUNTED (no gimbal) + anti-vibration mount
DOWN_IMG_ROLL_DEG  = -90.0             # constant in-image roll offset (deg); -90 aligns the GS sensor mount
DOWN_Z_OFFSET      = -0.05             # down_cam height offset below the drone body (m)
DOWN_VIB_DAMP      = True              # simulate the silicone anti-vibration mount the gimbal-less GS cam needs
DOWN_VIB_TAU_S     = 0.05              # mount time constant (s): passes slow attitude (<~3 Hz),
                                       # soaks up high-freq airframe/gust vibration. 0 / False = rigid hard mount.
# --- CHASE camera: free-standing third-person cam that trails + frames the drone
CHASE_CAM          = True              # add a chase camera (not body-mounted) that trails the drone
CHASE_TRAIL_DIST   = 1.5               # meters behind the drone, along its horizontal heading
CHASE_TRAIL_HEIGHT = 1               # meters above the drone
CHASE_DAMP_TAU_S   = 0.15              # smoothing time constant (s) for the trailing motion —
                                       # eases toward the target pose instead of snapping to it.
                                       # Only the camera's POSITION is smoothed; the aim point
                                       # always tracks the drone live so fast moves don't push
                                       # it toward the frame edge.
CHASE_MAX_DIST     = 2.0               # hard leash (m): the damped lag can let the camera fall
                                       # further behind than CHASE_TRAIL_DIST while the drone
                                       # accelerates -- clamped back to this distance every
                                       # frame so it never drifts further than this from the drone.
STREAM_CAMERAS     = True              # serve all feeds over HTTP (MJPEG) to any browser on the LAN
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

# --- environment overrides ---------------------------------------------------
# Any tunable above can be overridden with DRONE_SETUP_<NAME>, holding a Python
# literal. sim/launch-sitl.sh uses this to fly one baked stage from different
# takeoff points without editing this file:
#     DRONE_SETUP_SPAWN_XYZ='[12.0, -4.0, 0.7]' DRONE_SETUP_ADD_WIND=True
# Pasting this script into the Script Editor with no such vars set is unchanged.
def _env_overrides(known):
    """Return {tunable: value} parsed from the DRONE_SETUP_* environment."""
    import ast, os
    out = {}
    for key, raw in sorted(os.environ.items()):
        if not key.startswith("DRONE_SETUP_"):
            continue
        name = key[len("DRONE_SETUP_"):]
        if name not in known:
            print(f"*** {key}: no tunable named {name} in this script — ignored ***")
            continue
        try:
            out[name] = ast.literal_eval(raw)
        except (ValueError, SyntaxError):
            out[name] = raw              # bare strings (REC_DIR, RECORD_KEY, ...)
        print(f">>> override {name} = {out[name]!r}")
    return out


globals().update(_env_overrides(set(globals())))
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
    """Attach the DOWN camera (real ZED X One GS, hard-mounted + anti-vibration
    soft mount) and a movable detect_cam (keyboard pan/tilt) to the spawned drone.
    Must run while the sim is STOPPED."""
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

    # ---- ZED X One GS (Wide) — physically-accurate optics / calibration -------
    # Datasheet: Sony IMX392, 1/2.3" GLOBAL-SHUTTER (Pregius), 1920x1200 (2.3 MP),
    # 3.45 µm square pixels, fixed-focus Wide 2.2 mm lens, ~110° HFOV, up to 60 fps.
    # The pinhole intrinsics fall straight out of the physical sensor + focal length
    # (no FOV back-solving), so they match a real ZED X One GS calibration:
    #   horizontal aperture = 1920 * 3.45µm = 6.624 mm   (USD aperture is in mm)
    #   vertical   aperture = 1200 * 3.45µm = 4.140 mm
    #   fx = fy = focal/pixel = 2.2 / 0.00345 ≈ 637.7 px ; cx=960, cy=600
    #   → geometric FOV ≈ 112.7° H × 86.6° V (consistent with the rated 110° Wide).
    ZED_MODEL    = "GS"                            # global shutter (vibration-tolerant: no rolling-shutter jello)
    ZED_PIXEL_UM = 3.45                            # IMX392 pixel pitch (µm), square pixels
    ZED_FOCAL_MM = 2.2                             # Wide fixed-focus lens
    ZED_SENSOR_W, ZED_SENSOR_H = 1920, 1200        # native resolution (2.3 MP)
    zed_h_ap = ZED_SENSOR_W * ZED_PIXEL_UM / 1000.0   # 6.624 mm
    zed_v_ap = ZED_SENSOR_H * ZED_PIXEL_UM / 1000.0   # 4.140 mm

    def _zedcam(path):
        return _cam(path, focal=ZED_FOCAL_MM, h_ap=zed_h_ap, v_ap=zed_v_ap)

    # ---- DOWN camera: ZED X One GS, HARD-MOUNTED (no gimbal) ------------------
    # The real ZED X One GS has NO gimbal, so it is bolted to the airframe and
    # tilts with the drone (true nadir only when level) — exactly the real rig.
    # To stand in for the silicone anti-vibration mount you'd use on a hard-mounted
    # GS camera, the mount is a TOP-LEVEL frame (/World/down_mount) driven per
    # frame: its POSITION tracks the body exactly and its ORIENTATION follows the
    # body's FULL attitude through a first-order low-pass (DOWN_VIB_TAU_S). That
    # passes slow attitude (<~3 Hz) but soaks up high-frequency airframe/gust
    # vibration; together with the global shutter (no rolling-shutter jello) it
    # yields clean, distortion-free nadir frames. A quaternion ORIENT op is used
    # (not RotateXYZ) so the full 3-axis attitude is set without Euler ambiguity.
    body_prim = stage.GetPrimAtPath(BODY_PATH)
    dgim = UsdGeom.Xform.Define(stage, "/World/down_mount")
    _dxf = UsdGeom.Xformable(dgim); _dxf.ClearXformOpOrder()
    dgt = _dxf.AddTranslateOp()
    dgo = _dxf.AddOrientOp()                                   # quaternion (Gf.Quatf)
    dgt.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    _seed_q = Rotation.from_euler("XYZ", [0.0, 0.0, DOWN_IMG_ROLL_DEG], degrees=True).as_quat()
    dgo.Set(Gf.Quatf(float(_seed_q[3]), float(_seed_q[0]), float(_seed_q[1]), float(_seed_q[2])))
    dwn_path = "/World/down_mount/down_cam"
    dwn = _zedcam(dwn_path); ddt, ddr = _ops(dwn)
    ddt.Set(Gf.Vec3d(0.0, 0.0, 0.0)); ddr.Set(Gf.Vec3f(0.0, 0.0, 0.0))  # camera looks local -Z = mount down

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
    print(f"down_cam created: {ok_d}   detect_cam created: {ok_t}")
    if not (ok_d and ok_t):
        print("*** Camera creation FAILED. ***")
        return

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
    create_viewport_window("DownCam (ZED X One GS, hard-mount)", camera_path=Sdf.Path(dwn_path),
                           width=512, height=320, position_x=40, position_y=40)
    create_viewport_window("DetectCam (movable, ZED Wide)", camera_path=Sdf.Path(det_path),
                           width=512, height=320, position_x=560, position_y=40)
    print("Cameras ready. Click inside DetectCam, then: I/K=tilt, J/L=pan, U=down, O=default.")

    # anti-vibration body-follow: each app update, copy the drone body's LIVE world
    # POSE to the top-level down_mount — POSITION exactly, ORIENTATION through a
    # first-order low-pass (the soft mount). DOWN_VIB_DAMP off → a perfectly rigid
    # hard mount (mount attitude == body attitude, with the constant image roll).
    if dgt is not None and body_prim and body_prim.IsValid():
        import numpy as _np
        # constant body->image correction (the -90° sensor-mount roll about optical Z)
        _img_roll = Rotation.from_euler("XYZ", [0.0, 0.0, DOWN_IMG_ROLL_DEG], degrees=True)

        def _build_pose_getter():
            # returns a fn -> (pos[3], quat_xyzw[4]) of the drone body in world.
            # prefer the physics/fabric-aware core API (live during play); fall back
            # to a USD XformCache read (can be stale under the Fabric delegate).
            for _mod in ("isaacsim.core.prims", "omni.isaac.core.prims"):
                try:
                    XFormPrim = __import__(_mod, fromlist=["XFormPrim"]).XFormPrim
                    vp = XFormPrim(BODY_PATH)
                    def _g():
                        p, q = vp.get_world_poses()
                        p = _np.asarray(p).reshape(-1)
                        q = _np.asarray(q).reshape(-1)              # quaternion w,x,y,z
                        return (p[:3].astype(float),
                                _np.array([q[1], q[2], q[3], q[0]], dtype=float))
                    _g()  # smoke test
                    print(f">>> down_mount pose source: {_mod}.XFormPrim (live)")
                    return _g
                except Exception:
                    continue
            xc = UsdGeom.XformCache()
            def _g():
                xc.Clear()
                m = xc.GetLocalToWorldTransform(body_prim)
                t = m.ExtractTranslation()
                q = m.ExtractRotationQuat()                        # Gf.Quatd
                im = q.GetImaginary()
                return (_np.array([t[0], t[1], t[2]], dtype=float),
                        _np.array([im[0], im[1], im[2], q.GetReal()], dtype=float))
            print(">>> down_mount pose source: USD XformCache (fallback)")
            return _g

        _get_pose = _build_pose_getter()
        _filt = {"q": None}
        def _track(e):
            try:
                pos, qb = _get_pose()
                # rigid target = full body attitude * constant in-image roll
                q_target = (Rotation.from_quat(qb) * _img_roll).as_quat()   # x,y,z,w
                q_target = q_target / (_np.linalg.norm(q_target) or 1.0)
                if DOWN_VIB_DAMP and _filt["q"] is not None:
                    try: dt = float(e.payload["dt"])
                    except Exception: dt = 1.0 / 60.0
                    a = 1.0 - math.exp(-dt / max(DOWN_VIB_TAU_S, 1e-3))      # low-pass gain
                    q_prev = _filt["q"]
                    if float(_np.dot(q_prev, q_target)) < 0.0:              # shortest-arc nlerp
                        q_target = -q_target
                    q_new = (1.0 - a) * q_prev + a * q_target
                    q_new = q_new / (_np.linalg.norm(q_new) or 1.0)
                else:
                    q_new = q_target
                _filt["q"] = q_new
                dgt.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2] + DOWN_Z_OFFSET)))
                dgo.Set(Gf.Quatf(float(q_new[3]), float(q_new[0]), float(q_new[1]), float(q_new[2])))
            except Exception:
                pass
        prev_stab = globals().get("_DOWN_GIMBAL_SUB")
        if prev_stab is not None:
            try: prev_stab.unsubscribe()
            except Exception: pass
        globals()["_DOWN_GIMBAL_SUB"] = omni.kit.app.get_app().get_update_event_stream(
            ).create_subscription_to_pop(_track, name="down_mount_vib")
        _mode = (f"soft anti-vibration mount (tau={DOWN_VIB_TAU_S:.3f}s, isolates >~"
                 f"{1.0/(2*math.pi*max(DOWN_VIB_TAU_S,1e-3)):.1f} Hz)"
                 if DOWN_VIB_DAMP else "rigid hard mount")
        print(f">>> down_cam ZED X One GS: hard-mounted (NO gimbal), {_mode}; "
              "tilts with the drone (true nadir when level).")

    # ---- CHASE camera: free-standing third-person cam that trails the drone ---
    # Unlike down_cam/detect_cam this is NOT parented to the body — it's its own
    # /World prim, driven each frame from the drone's live world pose (reusing
    # down_mount's pose getter). Target POSITION = offset behind (along the
    # body's flattened horizontal forward axis) and above the drone; a
    # first-order low-pass (CHASE_DAMP_TAU_S) eases the camera's position
    # toward that target so quick maneuvers don't whip the view around. The
    # AIM point is NOT lagged — it tracks the drone's live position every
    # frame, so a fast drone stays centered even while the camera's own
    # position is still catching up (lagging the aim too let the drone run
    # off-frame during quick moves, since the camera kept pointing at a
    # stale, aged position instead of where the drone actually is).
    chase_path = "/World/chase_cam"
    if CHASE_CAM and body_prim and body_prim.IsValid():
        chs = _zedcam(chase_path)
        cxf = UsdGeom.Xformable(chs); cxf.ClearXformOpOrder()
        cht = cxf.AddTranslateOp()
        cho = cxf.AddOrientOp()                                    # quaternion (Gf.Quatf)

        _cfilt = {"pos": None}

        def _chase_track(e):
            try:
                pos, qb = _get_pose()
                fwd = Rotation.from_quat(qb).apply([1.0, 0.0, 0.0])
                fwd[2] = 0.0
                n = _np.linalg.norm(fwd)
                fwd = fwd / n if n > 1e-6 else _np.array([1.0, 0.0, 0.0])
                target_pos = pos - fwd * CHASE_TRAIL_DIST + _np.array([0.0, 0.0, CHASE_TRAIL_HEIGHT])

                try: dt = float(e.payload["dt"])
                except Exception: dt = 1.0 / 60.0
                a = 1.0 - math.exp(-dt / max(CHASE_DAMP_TAU_S, 1e-3))
                if _cfilt["pos"] is None:
                    _cfilt["pos"] = target_pos
                else:
                    _cfilt["pos"] = (1.0 - a) * _cfilt["pos"] + a * target_pos

                # hard leash: never let the lag put the camera further than
                # CHASE_MAX_DIST from the drone, however fast it accelerates
                offset = _cfilt["pos"] - pos
                dist = _np.linalg.norm(offset)
                if dist > CHASE_MAX_DIST:
                    _cfilt["pos"] = pos + offset * (CHASE_MAX_DIST / dist)

                cam_pos, look_at = _cfilt["pos"], pos                # aim: live drone position

                # look-at basis (OpenGL/USD convention: camera looks down local -Z)
                z_axis = cam_pos - look_at
                zn = _np.linalg.norm(z_axis)
                z_axis = z_axis / zn if zn > 1e-6 else _np.array([0.0, 1.0, 0.0])
                world_up = _np.array([0.0, 0.0, 1.0])
                x_axis = _np.cross(world_up, z_axis)
                xn = _np.linalg.norm(x_axis)
                x_axis = x_axis / xn if xn > 1e-6 else _np.array([1.0, 0.0, 0.0])
                y_axis = _np.cross(z_axis, x_axis)
                q = Rotation.from_matrix(_np.column_stack([x_axis, y_axis, z_axis])).as_quat()  # x,y,z,w

                cht.Set(Gf.Vec3d(float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])))
                cho.Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))
            except Exception:
                pass

        prev_chase = globals().get("_CHASE_CAM_SUB")
        if prev_chase is not None:
            try: prev_chase.unsubscribe()
            except Exception: pass
        globals()["_CHASE_CAM_SUB"] = omni.kit.app.get_app().get_update_event_stream(
            ).create_subscription_to_pop(_chase_track, name="chase_cam_follow")
        print(f">>> chase_cam ready: trails {CHASE_TRAIL_DIST:.1f}m behind / "
              f"{CHASE_TRAIL_HEIGHT:.1f}m above, damped (tau={CHASE_DAMP_TAU_S:.2f}s).")

        create_viewport_window("ChaseCam (third-person, trailing)", camera_path=Sdf.Path(chase_path),
                               width=512, height=320, position_x=1080, position_y=40)

    cam_paths = {"down": dwn_path, "detect": det_path}
    if CHASE_CAM and stage.GetPrimAtPath(chase_path).IsValid():
        cam_paths["chase"] = chase_path

    if STREAM_CAMERAS:
        start_camera_streams(cam_paths)


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
          f"single feeds: {', '.join('/' + c for c in cams)})")
    print(f">>> RECORD: focus a viewport and press '{RECORD_KEY}' to start/stop MP4 "
          f"recording of all cams -> {rec_dir}/ ({REC_W}x{REC_H} @ {REC_FPS}fps)")


def fix_ground_plane():
    """Find the ground plane on the stage and rotate it back to flat. A plane
    created via Create > Physics > Ground Plane under a Cesium georeference gets
    tipped up into a 'wall'; this applies a corrective rotation about X.
    Idempotent: it SETS an absolute rotation (preserving the plane's translation),
    so re-running the script never stacks extra 90s."""
    from pxr import UsdGeom, Gf
    stage = omni.usd.get_context().get_stage()

    target = None
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if ("groundplane" in name or "ground_plane" in name
                or name == "defaultgroundplane"):
            target = prim
            break
    if target is None:
        print("*** fix_ground_plane: no ground plane found — skipping. "
              "(Add one via Create > Physics > Ground Plane.) ***")
        return

    xf = UsdGeom.Xformable(target)
    # preserve any existing translation, then set a single clean rotateXYZ
    trans = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            trans = op.Get()
    xf.ClearXformOpOrder()
    t = xf.AddTranslateOp()
    t.Set(trans if trans is not None else Gf.Vec3d(0.0, 0.0, 0.0))
    r = xf.AddRotateXYZOp()
    r.Set(Gf.Vec3f(GROUND_FLIP_DEG, 0.0, 0.0))
    print(f">>> Ground plane {target.GetPath()} set to rotateX={GROUND_FLIP_DEG:.0f} "
          f"(flat). If it's still a wall, set GROUND_FLIP_DEG = {-GROUND_FLIP_DEG:.0f} and re-run.")


def setup_wind():
    """Add REALISTIC gusty wind the lockstep-safe way: swap the vehicle's drag
    model for a wind-aware one. Pegasus calls veh._drag.update(state, dt) and
    applies the result inside Multirotor.update() in the SAME physics step (right
    before the PX4 backend runs) — so this never disrupts lockstep or the QGC link.

    Wind model (horizontal only; z force stays 0 so the drone never sinks):
        wind_enu = mean + gust
      - mean : a prevailing wind whose DIRECTION slowly wanders (±WIND_DIR_WANDER_DEG)
               and whose SPEED gently swells, both via slow Ornstein–Uhlenbeck (OU)
               drift. Base direction is WIND_FROM_DEG, or RANDOM each run if None.
      - gust : a fast OU (first-order Gauss–Markov) turbulence vector, per-axis std
               WIND_GUST_FRAC*WIND_SPEED_MS and correlation time WIND_GUST_TAU_S.
    OU keeps the noise smooth, bounded and time-correlated (real gusts), not white
    jitter. Set WIND_SEED for a reproducible wind track across dataset runs.

    Force: F_body = -D*(v_body - v_wind_body), D = diag(k, k, 0). Engages only above
    WIND_MIN_AGL_M so a grounded drone is left alone.

    NOTE: PX4 cannot create wind in Pegasus (physics lives in Isaac). Wind is applied
    here; the matching PX4 wind ESTIMATOR params are printed so PX4's EKF can be made
    aware if you want (those only feed the estimator, not the dynamics)."""
    import math, time as _time
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pegasus.simulator.logic.vehicle_manager import VehicleManager
    from pegasus.simulator.logic.dynamics.linear_drag import LinearDrag

    vm = VehicleManager.get_vehicle_manager()
    vehicles = list(vm.vehicles.values()) if getattr(vm, "vehicles", None) else []
    veh = vehicles[0] if vehicles else None
    if veh is None:
        print("*** setup_wind: no Pegasus vehicle found — wind disabled. ***")
        return

    rng = np.random.RandomState(WIND_SEED if WIND_SEED is not None else None)
    base_dir = float(WIND_FROM_DEG) if WIND_FROM_DEG is not None else float(rng.uniform(0.0, 360.0))

    def _enu_from(speed, from_deg):
        # direction wind blows FROM -> ENU vector it pushes the drone TOWARD
        return np.array([-math.sin(math.radians(from_deg)) * speed,
                         -math.cos(math.radians(from_deg)) * speed, 0.0])

    _COMPASS = ["N","NE","E","SE","S","SW","W","NW"]

    class WindDrag(LinearDrag):
        def __init__(self):
            super().__init__([WIND_COEFF, WIND_COEFF, 0.0])   # z=0 → never vertical
            self._ground_z = None
            self._last_print = 0.0
            self._dir_off = 0.0          # deg, slow OU around 0 → wanders base_dir
            self._spd_off = 0.0          # m/s, slow OU around 0 → swells base speed
            self._gust = np.zeros(3)     # m/s ENU, fast OU turbulence
            self._wind_enu = _enu_from(WIND_SPEED_MS, base_dir)

        @staticmethod
        def _ou(x, tau, sigma, dt, n):
            # first-order Gauss–Markov / OU step; mean-reverts to 0, stationary std = sigma
            tau = max(tau, 1e-3)
            return x - (x / tau) * dt + sigma * math.sqrt(2.0 / tau) * math.sqrt(max(dt, 0.0)) * n

        def update(self, state, dt):
            body_vel = state.linear_body_velocity            # FLU body-frame velocity
            z = float(state.position[2])
            if self._ground_z is None:
                self._ground_z = z
            agl = z - self._ground_z

            # --- evolve the wind field EVERY step (so it's already natural at lift-off) ---
            dt = float(dt) if dt and dt > 0 else 1.0 / 250.0
            if WIND_DIR_WANDER_DEG > 0:
                self._dir_off = self._ou(self._dir_off, 20.0, WIND_DIR_WANDER_DEG, dt, rng.randn())
            self._spd_off = self._ou(self._spd_off, 15.0, 0.25 * WIND_SPEED_MS, dt, rng.randn())
            mean_spd = max(0.0, WIND_SPEED_MS + self._spd_off)
            mean_enu = _enu_from(mean_spd, base_dir + self._dir_off)
            sig = WIND_GUST_FRAC * WIND_SPEED_MS
            self._gust[0] = self._ou(self._gust[0], WIND_GUST_TAU_S, sig, dt, rng.randn())
            self._gust[1] = self._ou(self._gust[1], WIND_GUST_TAU_S, sig, dt, rng.randn())
            self._wind_enu = mean_enu + self._gust

            wind_body = np.zeros(3)
            if agl >= WIND_MIN_AGL_M:
                wind_body = Rotation.from_quat(state.attitude).inv().apply(self._wind_enu)
            self._drag_force = -np.dot(self._drag_coefficients, body_vel - wind_body)

            now = _time.time()
            if now - self._last_print >= 1.0:
                self._last_print = now
                w = self._wind_enu
                spd = float(np.hypot(w[0], w[1]))
                frm = math.degrees(math.atan2(-w[0], -w[1])) % 360.0   # current wind's FROM direction
                comp = _COMPASS[round(frm / 45) % 8]
                f = self._drag_force
                tag = "ACTIVE" if agl >= WIND_MIN_AGL_M else f"below {WIND_MIN_AGL_M:.1f} m — inactive"
                print(f"[WIND] {spd:4.1f} m/s FROM {frm:5.0f}° ({comp:>2})  "
                      f"force=({f[0]:+.1f},{f[1]:+.1f},{f[2]:+.1f}) N  agl={agl:.1f} m  ({tag})")
            return self._drag_force

    veh._drag = WindDrag()
    comp0 = _COMPASS[round(base_dir / 45) % 8]
    seed_s = WIND_SEED if WIND_SEED is not None else "random"
    print(f">>> Wind (REALISTIC/gusty): mean {WIND_SPEED_MS:.1f} m/s FROM {base_dir:.0f}° ({comp0}), "
          f"gusts ±{WIND_GUST_FRAC*100:.0f}% (tau={WIND_GUST_TAU_S:.1f}s), dir wander ±{WIND_DIR_WANDER_DEG:.0f}°, "
          f"coeff={WIND_COEFF}, seed={seed_s} (Isaac-side, lockstep-safe; engages at +{WIND_MIN_AGL_M:.1f} m).")
    print(f"    Optional PX4 estimator awareness (paste in QGC MAVLink console): "
          f"param set SIM_WIND_SPD {WIND_SPEED_MS:.1f} ; param set SIM_WIND_DIR {base_dir:.0f}")
    print(f"    Disable wind: veh._drag = LinearDrag([0.5,0.3,0.0])")


def setup_truth_publisher():
    """Write the drone's TRUE world pose to /tmp/drone_truth.json at ~10 Hz.

    joystick-server-descriptive.py reads this file and forwards it to the web
    map, which draws a second marker for where the drone REALLY is next to
    PX4's own (drifting) estimate. Same file-drop IPC as /tmp/vio_gps.json.

    Pure read side-effect on the sim: it only samples the body prim's world
    transform, never writes to the stage. Safe to leave running always.
    """
    import json as _json, math as _math, os as _os, tempfile as _tempfile
    import numpy as _np
    import omni.usd, omni.kit.app
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    bodies = [str(p.GetPath()) for p in stage.Traverse() if p.GetName() == "body"]
    body_path = next((b for kw in ("px4_drone", "quadrotor", "iris", "drone",
                                   "multirotor")
                      for b in bodies if kw in b.lower()), bodies[0] if bodies
                     else None)
    if body_path is None:
        print("*** setup_truth_publisher: no drone 'body' prim — truth feed off. ***")
        return

    # Same live-first / XformCache-fallback pose getter as the camera mount.
    _get_pose = None
    for _mod in ("isaacsim.core.prims", "omni.isaac.core.prims"):
        try:
            XFormPrim = __import__(_mod, fromlist=["XFormPrim"]).XFormPrim
            _vp = XFormPrim(body_path)

            def _get_pose(_vp=_vp):
                p, q = _vp.get_world_poses()
                p = _np.asarray(p).reshape(-1)
                q = _np.asarray(q).reshape(-1)                 # w, x, y, z
                return (p[:3].astype(float),
                        _np.array([q[1], q[2], q[3], q[0]], float))  # x,y,z,w
            _get_pose()
            break
        except Exception:
            _get_pose = None
    if _get_pose is None:
        body_prim = stage.GetPrimAtPath(body_path)
        _xc = UsdGeom.XformCache()

        def _get_pose(body_prim=body_prim, _xc=_xc):
            _xc.Clear()
            m = _xc.GetLocalToWorldTransform(body_prim)
            t = m.ExtractTranslation()
            qq = m.ExtractRotationQuat()
            im = qq.GetImaginary()
            return (_np.array([t[0], t[1], t[2]], float),
                    _np.array([im[0], im[1], im[2], qq.GetReal()], float))

    # Local ENU metres -> lat/lon, using the SAME spherical reprojection and
    # Earth radius Pegasus's own GPS uses (geo_mag_utils.reprojection), so the
    # truth marker and the PX4 estimate share a projection and any gap between
    # them is real estimator error, not a datum mismatch.
    EARTH_R = 6353000.0
    lat0, lon0, alt0 = _math.radians(lat), _math.radians(lon), alt
    out_path = _os.environ.get("DRONE_TRUTH_FILE", "/tmp/drone_truth.json")
    acc = {"t": 0.0}

    def _publish(e):
        try:
            acc["t"] += float(e.payload["dt"])
        except Exception:
            acc["t"] += 1.0 / 60.0
        if acc["t"] < 0.1:
            return
        acc["t"] = 0.0
        try:
            pos, q = _get_pose()
            x_rad = pos[1] / EARTH_R          # north
            y_rad = pos[0] / EARTH_R          # east
            c = _math.hypot(x_rad, y_rad)
            if c > 1e-12:
                sc, cc = _math.sin(c), _math.cos(c)
                lat_r = _math.asin(cc * _math.sin(lat0)
                                   + x_rad * sc * _math.cos(lat0) / c)
                lon_r = lon0 + _math.atan2(
                    y_rad * sc,
                    c * _math.cos(lat0) * cc - x_rad * _math.sin(lat0) * sc)
            else:
                lat_r, lon_r = lat0, lon0
            # yaw about world +Z from the xyzw quaternion, ENU -> compass.
            qx, qy, qz, qw = q
            yaw_enu = _math.atan2(2.0 * (qw * qz + qx * qy),
                                  1.0 - 2.0 * (qy * qy + qz * qz))
            heading = (90.0 - _math.degrees(yaw_enu)) % 360.0
            payload = _json.dumps({
                "t": time.time(),
                "lat": _math.degrees(lat_r),
                "lon": _math.degrees(lon_r),
                "alt": float(pos[2]) + alt0,
                "heading_deg": heading,
            })
            fd, tmp = _tempfile.mkstemp(dir=_os.path.dirname(out_path) or ".",
                                        prefix=".drone_truth.")
            with _os.fdopen(fd, "w") as f:
                f.write(payload)
            _os.replace(tmp, out_path)        # atomic: reader never sees a partial write
        except Exception:
            pass

    prev = globals().get("_TRUTH_PUB_SUB")
    if prev is not None:
        try: prev.unsubscribe()
        except Exception: pass
    globals()["_TRUTH_PUB_SUB"] = omni.kit.app.get_app().get_update_event_stream(
        ).create_subscription_to_pop(_publish, name="drone_truth_publish")
    print(f">>> ground-truth feed: {out_path} @ ~10 Hz (body {body_path})")


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
        # Explicit on purpose. px4_mavlink_backend.py's own defaults were
        # changed 2026-08-05 for a HITL setup (real FC over VPN): tcpin->udpin,
        # localhost->0.0.0.0, lockstep True->False. Local SITL needs the
        # opposite of all three -- PX4's own px4-rc.simulator runs
        # `simulator_mavlink start -c <port>`, which connects OUT over TCP, so
        # Isaac has to be the TCP listener (tcpin) or the two sides never even
        # share a transport. Without an explicit lockstep=True, PX4 SITL's
        # simulator_mavlink module spins on "ERROR poll timeout" forever,
        # because Isaac never blocks the physics step to wait for it. Relying
        # on the backend's defaults means this file's behavior silently
        # depends on which config PegasusSimulator happens to ship with --
        # stating the SITL values here keeps this script correct regardless.
        "connection_type": "tcpin",
        "connection_ip": "localhost",
        "enable_lockstep": True,
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

    # ground-truth feed for the descriptive web UI (read-only on the sim)
    setup_truth_publisher()

    # 4) un-flip the ground plane, then add wind -----------------------------
    if FIX_GROUND_FLIP:
        fix_ground_plane()

    if ADD_WIND:
        setup_wind()

    print(">>> Done. Press Play, then connect QGC.")
    print("    If QGC heading is rotated vs the map, nudge HEADING_OFFSET_DEG "
          "(usually +/-90 or 180) and re-run.")

asyncio.ensure_future(_spawn_px4_keep_stage())
