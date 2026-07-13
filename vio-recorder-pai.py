# ============================================================================
# VIO recorder (Phase A1+A2+A3) — records THREE parallel streams, joined by `frame`:
#   imu.csv    frame, ts_ns, wx,wy,wz, ax,ay,az     (Pegasus IMU, FRD body, noisy, @DATA_FPS)
#   baro.csv   frame, ts_ns, pressure_hpa, pressure_altitude_m, temperature_c
#                                                   (Pegasus Barometer — a SEPARATE sensor, not an IMU field; @DATA_FPS)
#   poses.csv  frame, x,y,z, qx,qy,qz,qw           (GROUND TRUTH, takeoff-anchored ENU pos + per-frame attitude @DATA_FPS)
#   geo.csv    frame, ts_ns, lat_deg, lon_deg, alt_m (ABSOLUTE geodetic from raw ENU; enables AGL = baro_AMSL - DEM(lat,lon))
#   frames.csv  frame, ts_ns, image_path            (cam0 = down_cam frames, every IMG_EVERY frames)
#   frames_cam1.csv (same, cam1 = fpv_cam — only if fpv_cam is on the stage)
#   + images/cam0/fNNNNNN.png  (+ images/cam1/... for fpv), takeoff.json,
#     cam_calib.json (cam0) + cam_calib_cam1.json (cam1), georef.json (ENU origin lat/lon/height)
#
# TWO independent monocular VIO cameras (see CLAUDE.md north star): cam0 nadir +
# cam1 forward FPV. NOT a stereo rig — each is its own sensor. cam1 is optional.
#
# One counter @DATA_FPS drives all three (shared `frame` + world.current_time clock).
# Camera = the EXISTING down_cam. Works with EITHER:
#   DOWN_STABILIZE=False -> body-rigid cam, constant cam<->imu extrinsic -> VIO-valid (preferred).
#   DOWN_STABILIZE=True  -> gimbal-stabilized (nadir) cam; cam<->body extrinsic is TIME-VARYING,
#                           cam_calib.json then holds only the takeoff snapshot (see its note).
#
#   Run order: spawn (~/drone_setup_px4_cesium.py) -> Play
#   -> run THIS in the Script Editor just before takeoff.
#   Output: ~/vio_dataset/<run_ts>/
# ============================================================================
import os, time, json, threading, queue
import numpy as np
from scipy.spatial.transform import Rotation
from isaacsim.core.api.world import World
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.sensors import IMU, Barometer
import omni.usd, omni.kit.app
import omni.replicator.core as rep
from pxr import UsdGeom, Gf
from PIL import Image

OUT_DIR    = os.path.expanduser("~/vio_dataset")
FRAME_FPS  = 15        # image rate (Hz) per camera. Physics renders @60Hz, so <=60 is real. Exact rate is pinned to the actual data rate at runtime. (~15 is plenty for VIO; flow-odom's best stride was ~10 fps.)
DATA_FPS   = 200       # imu + baro + poses rate (Hz). Physics runs @250Hz; decim=1 gives ~250Hz actual (closest integer divisor).
CAM_W, CAM_H = 960, 600   # VIO capture size, 16:10 to match ZED X One GS (native 1920x1200). Optics read live from prim.
# Image storage. PNG is lossless but ~5x larger and slow to encode (which also
# starved the writer queue -> dropped frames). JPEG q92 is visually lossless for
# feature tracking, ~5x smaller, and fast to encode. GRAYSCALE halves/thirds size
# again for pure nadir VIO; keep RGB (default) if cam1/DSMAC/real2sim need colour.
IMG_FORMAT   = "jpg"   # "jpg" (~5x smaller, fast) or "png" (lossless, large)
JPEG_QUALITY = 92      # 90-95 ~ visually lossless for VIO; lower = smaller
GRAYSCALE    = True    # True -> 1-channel (smaller); RGB only needed for COLOUR map-matching / real2sim
PRINT_EVERY_S = 1.0
TAKEOFF_ALT_M = 0.5   # start recording once the drone climbs this far above its resting altitude

# Cesium georeference fallback (matches drone_setup_px4_cesium-pai.py). The local
# ENU world origin (0,0,0) sits at this lat/lon/height; needed downstream to turn
# VIO ENU into lat/lon for terrain-referenced AGL (AGL = baro_AMSL - DEM(lat,lon)).
FALLBACK_LAT, FALLBACK_LON, FALLBACK_ALT = 40.7128, -74.0060, 10.0


def read_cesium_georeference():
    """(lat, lon, height) of the CesiumGeoreference origin from the stage, or None.
    Mirrors drone_setup_px4_cesium-pai.py so the recorder logs the SAME origin the
    PX4 GPS was set to."""
    stg = omni.usd.get_context().get_stage()
    lat_names = ["cesium:georeferenceOrigin:latitude",  "georeferenceOrigin:latitude"]
    lon_names = ["cesium:georeferenceOrigin:longitude", "georeferenceOrigin:longitude"]
    alt_names = ["cesium:georeferenceOrigin:height",    "georeferenceOrigin:height"]

    def first_attr(prim, names):
        for n in names:
            a = prim.GetAttribute(n)
            if a and a.IsValid() and a.Get() is not None:
                return a.Get()
        return None

    for prim in stg.Traverse():
        tname = prim.GetTypeName()
        if "CesiumGeoreference" in str(tname) or "Georeference" in prim.GetName():
            lat = first_attr(prim, lat_names)
            lon = first_attr(prim, lon_names)
            alt = first_attr(prim, alt_names)
            if lat is not None and lon is not None:
                return float(lat), float(lon), float(alt if alt is not None else FALLBACK_ALT)
    return None


def enu_to_geodetic(e, n, u, lat0_deg, lon0_deg, h0):
    """WGS84 local-ENU (origin lat0/lon0/h0) -> (lat_deg, lon_deg, alt_m).
    Self-contained (no pymap3d): ENU -> ECEF -> geodetic."""
    a = 6378137.0; f = 1.0 / 298.257223563; e2 = f * (2.0 - f)
    lat0 = np.radians(lat0_deg); lon0 = np.radians(lon0_deg)
    sl, cl = np.sin(lat0), np.cos(lat0)
    so, co = np.sin(lon0), np.cos(lon0)
    N0 = a / np.sqrt(1.0 - e2 * sl * sl)
    x0 = (N0 + h0) * cl * co
    y0 = (N0 + h0) * cl * so
    z0 = (N0 * (1.0 - e2) + h0) * sl
    # ENU -> ECEF rotation (R = [[-so, -sl*co, cl*co],[co, -sl*so, cl*so],[0, cl, sl]])
    dx = -so * e - sl * co * n + cl * co * u
    dy = co * e - sl * so * n + cl * so * u
    dz = cl * n + sl * u
    x, y, z = x0 + dx, y0 + dy, z0 + dz
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1.0 - e2))
    for _ in range(5):
        N = a / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1.0 - e2 * N / (N + h)))
    N = a / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
    h = p / np.cos(lat) - N
    return np.degrees(lat), np.degrees(lon), h

# --- locate vehicle + IMU + cameras ---------------------------------------
# Two independent monocular VIO cameras (see CLAUDE.md north star):
#   cam0 = down_cam (nadir)   cam1 = fpv_cam (forward).  Each is its own sensor;
#   not a stereo rig.  cam1 is optional — if fpv_cam isn't on the stage we record
#   cam0 only, so old single-camera stages still work.
vm = VehicleManager.get_vehicle_manager()
vehicles = list(vm.vehicles.values()) if getattr(vm, "vehicles", None) else []
world = World.instance()
stage = omni.usd.get_context().get_stage()

def find_cam_path(prim_name):
    return next((str(p.GetPath()) for p in stage.Traverse()
                 if p.IsA(UsdGeom.Camera) and p.GetName() == prim_name), None)

def find_body_prim():
    """The drone body prim (extrinsics are computed relative to it, so cameras
    nested under mount/gimbal nodes resolve correctly)."""
    bodies = [p for p in stage.Traverse() if p.GetName() == "body"]
    for kw in ("px4_drone", "quadrotor", "iris", "drone", "multirotor"):
        b = next((b for b in bodies if kw in str(b.GetPath()).lower()), None)
        if b:
            return b
    return bodies[0] if bodies else None

cam_path = find_cam_path("down_cam")      # cam0 (required)
fpv_path = find_cam_path("fpv_cam")       # cam1 (optional)
imu = None
baro = None
veh = None
if vehicles:
    veh = vehicles[0]
    imu = next((s for s in veh._sensors if isinstance(s, IMU)), None)
    baro = next((s for s in veh._sensors if isinstance(s, Barometer)), None)  # separate sensor, not part of IMU

cam_stabilized = bool(cam_path) and "down_gimbal" in cam_path

if not vehicles or imu is None:
    print("*** No drone/IMU. Spawn with ~/drone_setup_px4_cesium.py first. ***")
elif cam_path is None:
    print("*** down_cam not found on stage. Run drone_setup_px4_cesium.py (cameras). ***")
elif world is None:
    print("*** No World instance. ***")
else:
    if cam_stabilized:
        print("*** NOTE: using the STABILIZED (gimbal) down_cam (DOWN_STABILIZE=True). It stays "
              "nadir while the body rotates, so the body<->cam extrinsic is TIME-VARYING; "
              "cam_calib.json stores the TAKEOFF SNAPSHOT only (cam orientation is world-nadir). ***")
    # clean up a previous recorder run
    old = globals().get("_VIO_REC")
    if old:
        try: world.remove_physics_callback(old["cb"])
        except Exception: pass
        try: old["sub"].unsubscribe()
        except Exception: pass
        try: old["q"].put_nowait(None)
        except Exception: pass
        for fh in old.get("files", []):
            try: fh.close()
            except Exception: pass

    run_dir = os.path.join(OUT_DIR, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    # logical->stage cameras. cam0 = down (required), cam1 = fpv (optional). Each is
    # its own monocular VIO sensor (no stereo). cam0 keeps the legacy filenames
    # (frames.csv, cam_calib.json, images/cam0) so the proven downward pipeline is
    # unchanged; cam1 is purely additive (frames_cam1.csv, cam_calib_cam1.json).
    cam_specs = [("cam0", "down_cam", cam_path)]
    if fpv_path:
        cam_specs.append(("cam1", "fpv_cam", fpv_path))
        print(f">>> fpv_cam found -> recording cam1 ({fpv_path})")
    else:
        print("*** fpv_cam not on stage -> recording cam0 (down) only. Add it via "
              "drone_setup_px4_cesium-pai.py (ADD_CAMERAS=True). ***")
    f_imu   = open(os.path.join(run_dir, "imu.csv"), "w");   f_imu.write("frame,ts_ns,wx,wy,wz,ax,ay,az\n")
    # poses.csv includes per-frame attitude quaternion (xyzw, ENU world frame) so
    # isaac_to_bag.py uses real orientation per frame instead of the static takeoff snapshot.
    f_pose  = open(os.path.join(run_dir, "poses.csv"), "w"); f_pose.write("frame,x,y,z,qx,qy,qz,qw\n")
    # barometer is a SEPARATE Pegasus sensor (not an IMU field) -> its own stream, joined by `frame`
    f_baro  = open(os.path.join(run_dir, "baro.csv"), "w") if baro is not None else None
    if f_baro is not None:
        f_baro.write("frame,ts_ns,pressure_hpa,pressure_altitude_m,temperature_c\n")
    else:
        print("*** No Barometer on the vehicle — skipping baro.csv. ***")

    # --- georeference: lat/lon/height of the local ENU origin (Cesium origin) ---
    # Enables terrain-referenced AGL downstream (AGL = baro_AMSL - DEM(lat,lon)).
    # Prefer the origin the setup script resolved (_CESIUM_GEOREF, set by
    # drone_setup_px4_cesium-pai.py) so it's identical to the PX4 GPS origin; else
    # read it off the stage; else fall back (and warn).
    g = globals().get("_CESIUM_GEOREF")
    geo_origin = (g["lat"], g["lon"], g["alt"]) if g else read_cesium_georeference()
    geo_is_fallback = geo_origin is None
    if geo_is_fallback:
        geo_origin = (FALLBACK_LAT, FALLBACK_LON, FALLBACK_ALT)
        print("*** WARN: no Cesium georeference (stage or _CESIUM_GEOREF) — geo.csv "
              f"lat/lon use FALLBACK {geo_origin}; terrain/DEM lookups will be wrong. ***")
    geo_lat, geo_lon, geo_alt = geo_origin
    with open(os.path.join(run_dir, "georef.json"), "w") as gf:
        json.dump({
            "origin": {"latitude": geo_lat, "longitude": geo_lon, "height_m": geo_alt},
            "is_fallback": geo_is_fallback,
            "frame": "local ENU (x=East, y=North, z=Up) world origin = this lat/lon/height; "
                     "WGS84 ellipsoidal height. poses.csv is takeoff-anchored (origin - takeoff); "
                     "geo.csv lat/lon are ABSOLUTE (from the raw ENU world position).",
            "source": "_CESIUM_GEOREF (setup script)" if g else
                      ("CesiumGeoreference prim" if not geo_is_fallback else "FALLBACK constants"),
        }, gf, indent=2)
    print(f">>> georeference origin: lat={geo_lat}, lon={geo_lon}, h={geo_alt} "
          f"({'fallback' if geo_is_fallback else 'real'}) -> georef.json")
    # per-frame absolute lat/lon (GT) for DEM-based AGL validation + deploy mapping
    f_geo   = open(os.path.join(run_dir, "geo.csv"), "w")
    f_geo.write("frame,ts_ns,lat_deg,lon_deg,alt_m\n")

    # --- capture intrinsics + body->cam extrinsic for EACH camera -------------
    # Extrinsic is computed relative to the drone body prim, so cameras nested
    # under mount/gimbal nodes (fpv_mount/fpv_cam, down_gimbal/down_cam) resolve
    # correctly. body-rigid -> the relative transform is constant (VIO-valid);
    # a stabilized gimbal cam yields only a takeoff snapshot (time-varying).
    xc = UsdGeom.XformCache()

    def body_to_cam_local(cprim):
        """body->cam extrinsic by composing the LOCAL (authored, static) transforms
        from the camera up to its 'body' ancestor. Local transforms don't change
        with physics/Fabric, so this is exact at any point during play — unlike the
        old M_body^-1 * M_cam from world transforms, which was unreliable under the
        Fabric delegate (gave 11 m / wrong-rotation extrinsics, worse for nested
        cams like fpv_mount/fpv_cam). Row-vector USD convention:
            LtW(cam) = Lcam * Lmount * ... * Lbody * ...
            T_body_cam = LtW(cam) * LtW(body)^-1 = product of locals between them.
        Returns (Gf.Matrix4d, found_body: bool)."""
        M = Gf.Matrix4d(1.0)
        p = cprim
        while p and p.IsValid() and p.GetName() != "body":
            M = M * UsdGeom.Xformable(p).GetLocalTransformation()
            p = p.GetParent()
        return M, bool(p and p.IsValid() and p.GetName() == "body")

    def capture_calib(cpath, primname, stabilized):
        cprim = stage.GetPrimAtPath(cpath)
        Tbc, found = body_to_cam_local(cprim)
        if not found:
            # stabilized top-level gimbal cam has no 'body' ancestor: fall back to
            # the world-transform snapshot (this extrinsic is time-varying anyway).
            bp = find_body_prim() or cprim.GetParent()
            Tbc = xc.GetLocalToWorldTransform(cprim) * xc.GetLocalToWorldTransform(bp).GetInverse()
        t_bc = Tbc.ExtractTranslation(); q_bc = Tbc.ExtractRotationQuat()  # GfQuat: w + imaginary
        uc = UsdGeom.Camera(cprim)
        focal = uc.GetFocalLengthAttr().Get()
        h_ap  = uc.GetHorizontalApertureAttr().Get()
        v_ap  = uc.GetVerticalApertureAttr().Get()
        fx = focal / h_ap * CAM_W; fy = focal / v_ap * CAM_H
        square = bool(np.isclose(fx, fy, rtol=1e-3))
        hfov = float(np.degrees(2.0 * np.arctan(h_ap / (2.0 * focal))))
        vfov = float(np.degrees(2.0 * np.arctan(v_ap / (2.0 * focal))))
        print(f">>> {primname} optics: focal={focal:.3f}mm -> {CAM_W}x{CAM_H} fx={fx:.2f} fy={fy:.2f}px "
              f"({'square' if square else 'ANAMORPHIC'} px), HFOV={hfov:.1f} VFOV={vfov:.1f}")
        if not square:
            print(f"*** WARN: {primname} fx != fy — set CAM_W/CAM_H to the sensor aspect (e.g. 960x600). ***")
        return {
            "resolution": [CAM_W, CAM_H],
            "intrinsics": {"fx": fx, "fy": fy, "cx": CAM_W/2.0, "cy": CAM_H/2.0, "distortion": [0,0,0,0,0]},
            "fov_deg": {"horizontal": hfov, "vertical": vfov},
            "optics": {"focal_mm": focal, "horizontal_aperture": h_ap, "vertical_aperture": v_ap,
                       "square_pixels": square, "source": "ZED X One GS Wide via drone_setup_px4_cesium"},
            "extrinsic_body_to_cam": {"translation": [t_bc[0], t_bc[1], t_bc[2]],
                                      "quaternion_wxyz": [q_bc.GetReal(), *q_bc.GetImaginary()]},
            "frames": "GT pose ENU world (takeoff-anchored); IMU FRD body",
            "extrinsic_is_constant": (not stabilized),
            "note": (f"{primname} STABILIZED gimbal: extrinsic_body_to_cam is the takeoff snapshot only "
                     "(TIME-VARYING, not a rigid VIO extrinsic)." if stabilized else
                     f"{primname} body-rigid (DOWN_STABILIZE=False / fixed mount) — constant extrinsic, VIO-valid."),
        }

    # per-camera I/O: image dir, frames csv, render product + rgb annotator, calib
    cams = []
    for logical, primname, cpath in cam_specs:
        cdir = os.path.join(run_dir, "images", logical); os.makedirs(cdir, exist_ok=True)
        frames_name = "frames.csv" if logical == "cam0" else f"frames_{logical}.csv"
        ff = open(os.path.join(run_dir, frames_name), "w"); ff.write("frame,ts_ns,image_path\n")
        rp = rep.create.render_product(cpath, (CAM_W, CAM_H))
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach([rp])
        stab = (logical == "cam0" and cam_stabilized)
        calib = capture_calib(cpath, primname, stab)
        calib_name = "cam_calib.json" if logical == "cam0" else f"cam_calib_{logical}.json"
        with open(os.path.join(run_dir, calib_name), "w") as cf:
            json.dump(calib, cf, indent=2)
        cams.append({"logical": logical, "dir": cdir, "frames": ff, "annot": ann})

    # --- background image writers (keep disk I/O off the sim loop) ---
    # Why cam1 was dropping ~47%: a SINGLE writer doing slow default PNG compression
    # could not drain 2 cams x 30fps, so the queue filled and put_nowait silently
    # dropped frames — and since cam1 is enqueued AFTER cam0 each frame, cam1 lost
    # the race. Fix: several writer threads + fast (still lossless) PNG + a per-file
    # lock for the frames CSVs + an explicit drop counter so future drops are visible.
    NUM_WRITERS = 6
    cam_io = {c["logical"]: (c["dir"], c["frames"], threading.Lock()) for c in cams}
    imgq = queue.Queue(maxsize=256 * len(cams))
    def _writer():
        while True:
            item = imgq.get()
            if item is None:
                imgq.put_nowait(None)        # propagate sentinel to the other writers
                break
            logical, frame, ts_ns, rgb = item
            cdir, ff, lock = cam_io[logical]
            path = os.path.join(cdir, f"f{frame:06d}.{IMG_FORMAT}")
            try:
                im = Image.fromarray(rgb)
                if GRAYSCALE:
                    im = im.convert("L")
                if IMG_FORMAT == "jpg":
                    im.save(path, quality=JPEG_QUALITY)   # lossy but visually lossless @q92; ~5x smaller + fast
                else:
                    im.save(path, compress_level=1)        # lossless PNG, larger/slower
                with lock:
                    ff.write(f"{frame},{ts_ns},{path}\n")
            except Exception:
                pass
    writers = [threading.Thread(target=_writer, daemon=True) for _ in range(NUM_WRITERS)]
    for wt in writers:
        wt.start()

    # An image (from every camera) is taken every IMG_EVERY-th data frame. IMG_EVERY
    # is recomputed at runtime from the ACTUAL data rate (physics may run faster than
    # DATA_FPS), so the image rate truly matches FRAME_FPS. This nominal value is for
    # the startup message only.
    IMG_EVERY = max(1, round(DATA_FPS / max(1, FRAME_FPS)))

    anchor = {"done": False, "p0": np.zeros(3)}
    st = {"last_print": -1e9, "step": 0, "decim": None, "img_every": IMG_EVERY,
          "frame": 0, "n_frame": 0, "armed": False, "ground_z": None, "dropped": 0}

    def _on_phys(dt):
        now = world.current_time
        # decimate physics steps to DATA_FPS exactly (physics runs faster, ~250Hz).
        # decim = physics_steps_per_data_sample, derived from the live step dt.
        if st["decim"] is None:
            st["decim"] = max(1, round((1.0 / DATA_FPS) / max(dt, 1e-9)))
            actual_data_fps = 1.0 / (dt * st["decim"])
            st["img_every"] = max(1, round(actual_data_fps / max(1, FRAME_FPS)))
            print(f">>> actual data rate ~{actual_data_fps:.0f} Hz; image every "
                  f"{st['img_every']} frames -> ~{actual_data_fps/st['img_every']:.1f} img/s/cam "
                  f"(target {FRAME_FPS}).")
        st["step"] += 1
        if st["step"] % st["decim"] != 0:            # one counter at DATA_FPS drives all 3 CSV streams
            return

        # GT pose (ENU world / FLU); q is xyzw (Pegasus convention)
        ss = veh.state; p = np.array(ss.position); q = ss.attitude; v = ss.linear_velocity

        # --- wait for TAKEOFF: record nothing while the drone sits on the ground ---
        if not st["armed"]:
            if st["ground_z"] is None:
                st["ground_z"] = float(p[2])             # resting altitude reference
            climbed = float(p[2]) - st["ground_z"]
            if climbed < TAKEOFF_ALT_M:
                if now - st["last_print"] >= PRINT_EVERY_S:
                    st["last_print"] = now
                    print(f"[REC] waiting for takeoff... climbed {climbed:+.2f} m / {TAKEOFF_ALT_M:.2f} m")
                return
            st["armed"] = True
            print(f">>> TAKEOFF detected (+{climbed:.2f} m) — recording started.")

        st["frame"] += 1
        fr = st["frame"]
        ts_ns = int(now * 1e9)
        # IMU (FRD body, noisy)
        si = imu.state
        w = si.get("angular_velocity", (0,0,0)); a = si.get("linear_acceleration", (0,0,0))
        f_imu.write(f"{fr},{ts_ns},{w[0]:.9f},{w[1]:.9f},{w[2]:.9f},{a[0]:.9f},{a[1]:.9f},{a[2]:.9f}\n")
        # Barometer (separate sensor): absolute pressure [hPa], pressure altitude [m AMSL], temperature [C]
        if f_baro is not None:
            sb = baro.state
            f_baro.write(f"{fr},{ts_ns},{sb.get('absolute_pressure',0.0):.6f},"
                         f"{sb.get('pressure_altitude',0.0):.6f},{sb.get('temperature',0.0):.6f}\n")
        # anchored so takeoff = (0,0,0); attitude written per-frame (xyzw, ENU world)
        if not anchor["done"]:
            anchor["done"] = True; anchor["p0"] = p.copy()
            with open(os.path.join(run_dir, "takeoff.json"), "w") as jf:
                json.dump({"frame": fr, "ts_ns": ts_ns, "position_enu": p.tolist(),
                           "attitude_xyzw": [float(q[0]),float(q[1]),float(q[2]),float(q[3])],
                           "note": "poses.csv position already has this subtracted (starts at 0,0,0)"}, jf, indent=2)
        pr = p - anchor["p0"]                       # ENU, relative to takeoff: x=East, y=North, z=Up
        f_pose.write(f"{fr},{pr[0]:.6f},{pr[1]:.6f},{pr[2]:.6f},"
                     f"{float(q[0]):.9f},{float(q[1]):.9f},{float(q[2]):.9f},{float(q[3]):.9f}\n")
        # absolute geodetic position from the RAW ENU world position p (not takeoff-
        # anchored) -> enables AGL = baro_AMSL - DEM(lat,lon) downstream.
        lat_f, lon_f, alt_f = enu_to_geodetic(float(p[0]), float(p[1]), float(p[2]),
                                              geo_lat, geo_lon, geo_alt)
        f_geo.write(f"{fr},{ts_ns},{lat_f:.9f},{lon_f:.9f},{alt_f:.4f}\n")
        # grab a frame from EVERY camera on the same data frame (shared ts_ns) so
        # cam0/cam1 are time-synced by construction.
        if fr % st["img_every"] == 0:
            # grab ALL cameras first, then enqueue atomically: either every camera's
            # frame for this data frame is kept, or none is. Prevents the per-camera
            # bias (cam0 enqueued first, so under queue pressure cam1 lost the race ->
            # ~36% cam1 drops). All-or-nothing keeps cam0/cam1 frame-synced.
            batch = []
            for c in cams:
                try:
                    data = c["annot"].get_data()
                    if data is not None and getattr(data, "size", 0) > 0:
                        rgb = np.ascontiguousarray(np.asarray(data)[:, :, :3])
                        batch.append((c["logical"], fr, ts_ns, rgb))
                except Exception:
                    pass
            if len(batch) == len(cams) and imgq.qsize() + len(batch) <= imgq.maxsize:
                for item in batch:
                    imgq.put_nowait(item)
                st["n_frame"] += 1
            elif batch:
                st["dropped"] += 1     # not enough room for the whole synced set (was silent)
        if now - st["last_print"] >= PRINT_EVERY_S:
            st["last_print"] = now
            f_imu.flush(); f_pose.flush(); f_geo.flush()
            for c in cams: c["frames"].flush()
            if f_baro is not None: f_baro.flush()
            drp = f" DROPPED={st['dropped']}" if st["dropped"] else ""
            print(f"[REC] t={now:5.1f}s frame={fr} imgs={st['n_frame']} q={imgq.qsize()}{drp}  "
                  f"pos=({pr[0]:+.2f},{pr[1]:+.2f},{pr[2]:+.2f}) m  |a|={np.linalg.norm(a):.2f}")

    world.add_physics_callback("vio_rec", _on_phys)
    globals()["_VIO_REC"] = {"cb": "vio_rec", "q": imgq,
                             "files": [f for f in ([f_imu, f_pose, f_baro, f_geo] +
                                                   [c["frames"] for c in cams]) if f is not None],
                             "dir": run_dir}
    cam_list = ", ".join(f"{c['logical']}={prim}" for (c, (_, prim, _2)) in zip(cams, cam_specs))
    print(f">>> VIO recorder writing: {run_dir}")
    print(f">>> cameras: {cam_list}  (cam0 -> frames.csv/cam_calib.json; cam1 -> frames_cam1.csv/cam_calib_cam1.json)")
    print(f">>> data(imu+poses) @{DATA_FPS}Hz; images @~{FRAME_FPS}/s/cam (exact rate set at runtime); ONE clock.")
    print(f">>> image format: {IMG_FORMAT.upper()}"
          + (f" q{JPEG_QUALITY}" if IMG_FORMAT == "jpg" else " (lossless)")
          + (", GRAYSCALE" if GRAYSCALE else ", RGB")
          + (f"  (~5x smaller than PNG; lower FRAME_FPS to shrink further)" if IMG_FORMAT == "jpg" else ""))
    print(f">>> Armed. Recording auto-starts on TAKEOFF (drone climbs >{TAKEOFF_ALT_M:.2f} m above rest). "
          "Sitting on the ground writes nothing.")
    print(">>> Press Play / take off. down_cam = " +
          ("STABILIZED gimbal (stays nadir; extrinsic is takeoff snapshot — see cam_calib.json note)."
           if cam_stabilized else
           "body-rigid (looks down, tilts with the drone) — VIO-valid."))
    print(">>> Stop:  _VIO_REC['q'].put(None); World.instance().remove_physics_callback('vio_rec'); "
          "[f.close() for f in _VIO_REC['files']]")
