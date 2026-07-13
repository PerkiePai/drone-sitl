#!/usr/bin/env python3
"""Altitude-scaled optical-flow odometry for the nadir drone camera.

Why this exists
---------------
Monocular MSCKF VIO (OpenVINS) diverges to km-scale on this dataset because the
drone flies near-constant-velocity cruise (accel std ~1.1 m/s^2): metric scale
is unobservable from the accelerometer.  A downward camera over (roughly flat)
ground does not need the accelerometer for scale -- the *height* gives it:

    v_ground = (translational_flow_normalized / dt) * Z,   Z ~= height_AGL

This is the PX4Flow / optical-flow + rangefinder approach used on real
GPS-denied drones.  Scale comes from height (observable via baro), so the
estimate cannot blow up the way the filter scale does.

Pipeline per consecutive camera-frame pair (t0 -> t1):
  1. Track features with pyramidal Lucas-Kanade.
  2. Normalize pixels with the pinhole intrinsics (distortion is zero here).
  3. De-rotate: predict where each ray goes under camera rotation alone
     (relative attitude), subtract it -> pure translational flow.
  4. Per-point ground depth Z from height + attitude (ray-ground intersection).
  5. Least-squares solve camera-frame translation t_cam from the standard
     translational-flow equations:  Z*du = -tx + x*tz ,  Z*dv = -ty + y*tz.
  6. Rotate t_cam into the world (ENU) frame and accumulate XY.  Z uses baro.

Attitude source: GT quaternions as a stand-in for a good AHRS (a real drone has
a solid IMU attitude estimate; the hard/unobservable part is metric XY scale,
which is exactly what flow+height fixes).  Height source: baro (subtracting the
takeoff sample).  Both are sensor-realistic; nothing here uses GT *position*.
"""
import argparse
import csv
import json
import os

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_DIR = "/home/innovation/pai/drone-vio/_in/isaac-sim-20260623"


def quat_xyzw_to_R(qx, qy, qz, qw):
    """Body->world rotation from an (x,y,z,w) quaternion."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def load_dataset(d):
    calib = json.load(open(os.path.join(d, "cam_calib.json")))
    fx = calib["intrinsics"]["fx"]
    fy = calib["intrinsics"]["fy"]
    cx = calib["intrinsics"]["cx"]
    cy = calib["intrinsics"]["cy"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    # camera->body rotation from extrinsic quaternion (w,x,y,z).
    qw, qx, qy, qz = calib["extrinsic_body_to_cam"]["quaternion_wxyz"]
    R_CtoI = quat_xyzw_to_R(qx, qy, qz, qw)
    # Convention fix (verified empirically against GT motion): the extrinsic is
    # defined w.r.t. the FRD IMU body, but the GT attitude quaternions in
    # poses.csv are for an FLU body in ENU world.  FLU and FRD differ by 180 deg
    # about x.  R_body_cam = R_CtoI @ Rx(180).  This makes the camera optical
    # axis point DOWN in world (as it must for a nadir cam) and the recovered
    # per-frame motion match GT direction & magnitude.
    R_CtoI = R_CtoI @ np.diag([1.0, -1.0, -1.0])  # body = R_CtoI @ cam

    frames = list(csv.DictReader(open(os.path.join(d, "frames.csv"))))
    poses = {int(r["frame"]): r for r in csv.DictReader(open(os.path.join(d, "poses.csv")))}
    baro = {int(r["frame"]): float(r["pressure_altitude_m"])
            for r in csv.DictReader(open(os.path.join(d, "baro.csv")))}
    baro0 = baro[min(baro)]  # takeoff pressure altitude

    # Newer recorder runs (real magnetometer in imu.csv) drop per-frame GT attitude
    # from poses.csv -- only the takeoff attitude survives, in takeoff.json. R_wb is
    # dead weight past frame 0 anyway (compute_ahrs_attitude overwrites every rec),
    # so only frame 0 needs a real value; interior frames get None.
    has_gt_quat = "qx" in next(iter(poses.values()))
    R0 = None
    if not has_gt_quat:
        takeoff = json.load(open(os.path.join(d, "takeoff.json")))
        qx0, qy0, qz0, qw0 = takeoff["attitude_xyzw"]
        R0 = quat_xyzw_to_R(qx0, qy0, qz0, qw0)

    img_dir = os.path.join(d, "images", "cam0")
    recs = []
    for fr in frames:
        fi = int(fr["frame"])
        if fi not in poses or fi not in baro:
            continue
        p = poses[fi]
        R_wb = (quat_xyzw_to_R(float(p["qx"]), float(p["qy"]), float(p["qz"]), float(p["qw"]))
                 if has_gt_quat else None)
        recs.append({
            "frame": fi,
            "ts": int(fr["ts_ns"]),
            "img": os.path.join(img_dir, os.path.basename(fr["image_path"])),
            "R_wb": R_wb,
            "h": baro[fi] - baro0,                 # height above takeoff (baro)
            "gt": np.array([float(p["x"]), float(p["y"]), float(p["z"])]),
        })
    if not has_gt_quat:
        recs[0]["R_wb"] = R0                        # AHRS init needs frame-0 GT only
    return K, R_CtoI, recs


def compute_ahrs_attitude(d, recs, Kp=1.0, mag_gain=0.0, mag_noise_deg=0.0):
    """Per-frame body attitude from the IMU ALONE (deployment-realistic; no GT).
    Mahony complementary filter integrated at the IMU rate in the FRD body frame:
    the gyro integrates orientation, the accelerometer corrects roll/pitch toward
    gravity. Initialised from GT frame-0; yaw drifts with no compass, exactly as on
    hardware.

    If mag_gain>0 and imu.csv has real mx,my,mz columns, a genuine magnetometer
    correction (same Mahony cross-product form as the gravity term, applied every
    IMU tick) pulls yaw toward the magnetic-north direction — real sensor fusion,
    not a GT stand-in. The world magnetic vector is calibrated once, from the GT
    frame-0 attitude + the mag reading at that instant (a one-time factory-compass
    calibration; legitimate since it only uses the single already-required GT
    frame-0 attitude, not per-frame GT).

    Older datasets with no mag channel fall back to the original GT-yaw stand-in
    (nudges yaw toward GT heading + optional gaussian noise mag_noise_deg) at each
    camera frame — kept for backward-compat / ablation comparisons where per-frame
    GT is still logged. Returns R_wb (FLU->ENU) per rec ts."""
    from scipy.spatial.transform import Rotation as Rot
    rows = list(csv.DictReader(open(os.path.join(d, "imu.csv"))))
    ts = np.array([int(r["ts_ns"]) for r in rows]) / 1e9
    gyro = np.array([[float(r["wx"]), float(r["wy"]), float(r["wz"])] for r in rows])  # FRD rad/s
    acc = np.array([[float(r["ax"]), float(r["ay"]), float(r["az"])] for r in rows])   # FRD specific force
    has_mag = "mx" in rows[0]
    mag = (np.array([[float(r["mx"]), float(r["my"]), float(r["mz"])] for r in rows])
           if has_mag else None)

    flip = np.diag([1.0, -1.0, -1.0])      # FLU<->FRD (self-inverse)
    R0 = recs[0]["R_wb"] @ flip             # init FRD->ENU from GT frame 0
    R = R0.copy()
    g_up = np.array([0.0, 0.0, 9.81])      # specific-force-at-rest points UP in ENU
    rec_ts = [r["ts"] / 1e9 for r in recs]
    rng = np.random.default_rng(0)

    m_world = None
    if has_mag and mag_gain > 0.0:
        i0 = int(np.clip(np.searchsorted(ts, rec_ts[0]), 0, len(ts) - 1))
        m_body0 = mag[max(0, i0 - 25):i0 + 25].mean(axis=0)
        m_world = R0 @ (m_body0 / np.linalg.norm(m_body0))   # one-time calibration from GT

    gt_yaw = None
    if mag_gain > 0.0 and not has_mag:      # legacy stand-in only when there's no real mag channel
        gt_yaw = [np.arctan2(r["R_wb"][1, 0], r["R_wb"][0, 0]) for r in recs]

    out = [recs[0]["R_wb"]]                 # frame 0 = GT init
    ri = 1
    for k in range(1, len(rows)):
        dt = ts[k] - ts[k - 1]
        if dt <= 0 or dt > 0.1:
            dt = 0.004
        w = gyro[k].copy()
        a = acc[k]; an = np.linalg.norm(a)
        v_pred = None
        if an > 1e-3:                       # tilt correction from gravity direction
            v_meas = a / an
            v_pred = R.T @ g_up; v_pred /= np.linalg.norm(v_pred)
            w = w + Kp * np.cross(v_meas, v_pred)
        if m_world is not None:             # continuous magnetometer yaw correction
            mn = np.linalg.norm(mag[k])
            if mn > 1e-6:
                m_meas = mag[k] / mn
                if mag_noise_deg > 0.0 and v_pred is not None:
                    ang = np.radians(mag_noise_deg) * rng.standard_normal()
                    m_meas = Rot.from_rotvec(v_pred * ang).apply(m_meas)
                m_pred = R.T @ m_world; m_pred /= np.linalg.norm(m_pred)
                w = w + mag_gain * np.cross(m_meas, m_pred)
        R = R @ Rot.from_rotvec(w * dt).as_matrix()
        while ri < len(recs) and ts[k] >= rec_ts[ri]:
            if gt_yaw is not None:          # legacy GT-yaw stand-in (no real mag channel)
                yaw = np.arctan2(R[1, 0], R[0, 0])
                meas = gt_yaw[ri] + np.radians(mag_noise_deg) * rng.standard_normal()
                dpsi = np.arctan2(np.sin(meas - yaw), np.cos(meas - yaw))
                R = Rot.from_rotvec([0, 0, mag_gain * dpsi]).as_matrix() @ R   # about world up
            out.append(R @ flip)           # back to FLU->ENU for load_dataset parity
            ri += 1
    while len(out) < len(recs):
        out.append(R @ flip)
    return out


def _triangulate_midpoint(C0, d0, C1, d1):
    """Closest 3D point to two world rays (origin C, unit dir d). None if degenerate."""
    r = C0 - C1
    a, bb, c = d0 @ d0, d0 @ d1, d1 @ d1
    M = np.array([[a, -bb], [bb, -c]])
    rhs = np.array([-(d0 @ r), -(d1 @ r)])
    if abs(np.linalg.det(M)) < 1e-9 or abs(bb) > 0.99995:   # near-parallel rays
        return None
    l0, l1 = np.linalg.solve(M, rhs)
    if l0 <= 0 or l1 <= 0:                                   # behind a camera
        return None
    return 0.5 * ((C0 + l0 * d0) + (C1 + l1 * d1))


def compute_true_agl(recs, K, R_CtoI, scale, W=10, step=4, min_pts=20):
    """Per-frame true AGL = camera_altitude - terrain_elevation, the honest sim
    stand-in for `baro - DEM(lat,lon)`.  Terrain elevation is reconstructed by
    triangulating tracked features with the GT camera poses (structure from known
    motion) over a W-frame baseline, taking the median ground z, then linearly
    interpolating across all frames (terrain varies smoothly).  This uses GT poses
    only to synthesise the 'DEM'; on the real drone AGL = baro_altitude - DEM, both
    real sensors.  Returns an array aligned to recs (one AGL per index)."""
    Ks = K.copy(); Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=400, qualityLevel=0.01, minDistance=10, blockSize=7)

    def load(p):
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        return cv2.resize(im, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1 else im

    idxs, agls = [], []
    for i in range(0, len(recs) - W, step):
        r0, r1 = recs[i], recs[i + W]
        im0, im1 = load(r0["img"]), load(r1["img"])
        p0 = cv2.goodFeaturesToTrack(im0, mask=None, **feat)
        if p0 is None:
            continue
        p1, stt, _ = cv2.calcOpticalFlowPyrLK(im0, im1, p0, None, **lk)
        stt = stt.reshape(-1).astype(bool)
        p0g, p1g = p0.reshape(-1, 2)[stt], p1.reshape(-1, 2)[stt]
        C0, C1 = r0["gt"], r1["gt"]
        R0, R1 = r0["R_wb"] @ R_CtoI, r1["R_wb"] @ R_CtoI
        terr = []
        for (u0, v0), (u1, v1) in zip(p0g, p1g):
            d0 = R0 @ (Kinv @ np.array([u0, v0, 1.0])); d0 /= np.linalg.norm(d0)
            d1 = R1 @ (Kinv @ np.array([u1, v1, 1.0])); d1 /= np.linalg.norm(d1)
            X = _triangulate_midpoint(C0, d0, C1, d1)
            if X is not None and X[2] < C0[2] - 1.0:        # ground below camera
                terr.append(X[2])
        if len(terr) >= min_pts:
            idxs.append(i)
            agls.append(C0[2] - np.median(terr))
    if len(idxs) < 2:
        raise RuntimeError("AGL triangulation failed — too few valid frames")
    return np.interp(np.arange(len(recs)), idxs, agls)


def build_dem_raster(recs, K, R_CtoI, scale, cell_m=5.0, W=10, step=4, min_pts=200):
    """Offline, whole-flight DEM stand-in for terrain-relief correlation (Exp10).
    Same triangulation primitive as compute_true_agl (GT-pose triangulation — the
    idealized-truth role that function already plays for AGL) but keeps each
    point's full (X, Y, Z) instead of collapsing to a median Z per window, then
    grids the accumulated point cloud into a 2D elevation raster. This DEM is
    built once, offline, from the whole flight -- it plays the role of a
    pre-loaded reference map (like DSMAC's Esri ortho tiles), NOT a live
    per-step estimate, so correlating a live/online relief signal against it is
    not the self-triangulation circularity Exp06 already ruled out.

    Camera centers (recs[i]["gt"]) are real GT position (poses.csv x/y/z,
    always logged). Camera attitude (recs[i]["R_wb"]) is whatever pipeline.py
    has already populated it with by the time this is called -- for the
    Exp10 target datasets (real magnetometer, no per-frame GT quaternion in
    poses.csv) that is the AHRS+compass estimate from compute_ahrs_attitude,
    not literal GT attitude. This is a corrected premise vs an earlier draft
    of this experiment's design: ray direction carries ~2 deg bounded AHRS
    tilt error (this project's own measured figure), not zero. It is still
    not the Exp06 self-triangulation circularity, because AHRS attitude comes
    from independent physical sensors (gyro+accel+compass), not from the live
    flow-odom position trace that this DEM is later correlated against.

    Returns a dict: raster (ny,nx) float32 grid of elevation (NaN outside the
    triangulated points' convex hull -- deliberately NOT filled, so a
    correlation search step must fail closed rather than match against
    extrapolated terrain it never actually observed), e0/n0 (grid origin,
    metres), cell_m, nx, ny, n_points (diagnostic)."""
    Ks = K.copy(); Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=400, qualityLevel=0.01, minDistance=10, blockSize=7)

    def load(p):
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        return cv2.resize(im, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA) if scale != 1 else im

    pts_e, pts_n, pts_z = [], [], []
    for i in range(0, len(recs) - W, step):
        r0, r1 = recs[i], recs[i + W]
        im0, im1 = load(r0["img"]), load(r1["img"])
        p0 = cv2.goodFeaturesToTrack(im0, mask=None, **feat)
        if p0 is None:
            continue
        p1, stt, _ = cv2.calcOpticalFlowPyrLK(im0, im1, p0, None, **lk)
        stt = stt.reshape(-1).astype(bool)
        p0g, p1g = p0.reshape(-1, 2)[stt], p1.reshape(-1, 2)[stt]
        C0, C1 = r0["gt"], r1["gt"]
        R0, R1 = r0["R_wb"] @ R_CtoI, r1["R_wb"] @ R_CtoI
        for (u0, v0), (u1, v1) in zip(p0g, p1g):
            d0 = R0 @ (Kinv @ np.array([u0, v0, 1.0])); d0 /= np.linalg.norm(d0)
            d1 = R1 @ (Kinv @ np.array([u1, v1, 1.0])); d1 /= np.linalg.norm(d1)
            X = _triangulate_midpoint(C0, d0, C1, d1)
            if X is not None and X[2] < C0[2] - 1.0:      # ground below camera
                pts_e.append(X[0]); pts_n.append(X[1]); pts_z.append(X[2])

    pts_e, pts_n, pts_z = np.array(pts_e), np.array(pts_n), np.array(pts_z)
    if len(pts_e) < min_pts:
        raise RuntimeError(
            f"build_dem_raster: only {len(pts_e)} triangulated ground points "
            f"(need >= {min_pts}) -- too sparse to build a usable DEM raster")

    e_min, e_max = pts_e.min(), pts_e.max()
    n_min, n_max = pts_n.min(), pts_n.max()
    nx = max(2, int((e_max - e_min) / cell_m) + 2)
    ny = max(2, int((n_max - n_min) / cell_m) + 2)
    grid_e = e_min + cell_m * np.arange(nx)
    grid_n = n_min + cell_m * np.arange(ny)
    GE, GN = np.meshgrid(grid_e, grid_n)

    from scipy.interpolate import griddata
    raster = griddata((pts_e, pts_n), pts_z, (GE, GN), method="linear")

    return dict(raster=raster.astype(np.float32), e0=float(e_min), n0=float(n_min),
                cell_m=float(cell_m), nx=nx, ny=ny, n_points=int(len(pts_e)))


def run(d, scale, max_frames, min_track, depth_source="baro", stride=1, fb_check=True,
        agl_arr=None, attitude_R=None, skip_frames=0):
    K, R_CtoI, recs = load_dataset(d)
    if skip_frames:
        recs = recs[skip_frames:]
    if max_frames:
        recs = recs[:max_frames]
    if attitude_R is not None:              # use IMU-AHRS attitude instead of GT
        for i in range(len(recs)):
            recs[i]["R_wb"] = attitude_R[i]
    Ks = K.copy()
    Ks[:2, :] *= scale
    Kinv = np.linalg.inv(Ks)
    fxs = Ks[0, 0]

    agl = None
    if depth_source == "agl":
        if agl_arr is not None:               # reuse a precomputed AGL (fast sweeps)
            agl = agl_arr
        else:
            cache_path = os.path.join(d, "agl_cache.npz")
            if os.path.exists(cache_path):
                agl = np.load(cache_path)["agl"]
                print(f"  loaded cached AGL from {cache_path}")
            else:
                print("  computing true AGL (GT-pose triangulation; stand-in for baro-DEM)...")
                agl = compute_true_agl(recs, K, R_CtoI, scale)
                np.savez(cache_path, agl=agl)
                print(f"  saved AGL cache → {cache_path}")
        print(f"  AGL: median {np.median(agl):.0f} m, range [{agl.min():.0f}, {agl.max():.0f}] m")

    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    feat = dict(maxCorners=600, qualityLevel=0.01, minDistance=8, blockSize=7)

    def load(path):
        im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if scale != 1.0:
            im = cv2.resize(im, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        return im

    pos = recs[0]["gt"].copy()             # start at GT origin (0,0,0)
    pos[2] = recs[0]["h"]
    est = [pos.copy()]
    proc = [0]                              # recs indices that have an est entry
    n_used = []
    impl_depth = [np.nan]                   # depth implied by flow + GT motion (validation)

    prev = load(recs[0]["img"])
    # process every `stride`-th frame: at 30 fps the 1-frame flow is ~0.3 px (poor
    # SNR); a larger baseline gives a bigger, cleaner displacement -> less drift.
    for i in range(stride, len(recs), stride):
        cur = load(recs[i]["img"])
        r0, r1 = recs[i - stride], recs[i]
        dt = (r1["ts"] - r0["ts"]) / 1e9

        p0 = cv2.goodFeaturesToTrack(prev, mask=None, **feat)
        used = 0
        idep = np.nan
        if p0 is not None and len(p0) >= min_track:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **lk)
            st = st.reshape(-1).astype(bool)
            if fb_check:                     # forward-backward consistency: re-track
                p0b, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **lk)
                fbe = np.linalg.norm(p0.reshape(-1, 2) - p0b.reshape(-1, 2), axis=1)
                st = st & st2.reshape(-1).astype(bool) & (fbe < 1.0)
            p0g, p1g = p0.reshape(-1, 2)[st], p1.reshape(-1, 2)[st]

            # camera frames cam0->world via attitude and the rigid extrinsic
            R_wc0 = r0["R_wb"] @ R_CtoI
            R_wc1 = r1["R_wb"] @ R_CtoI
            R_c1c0 = R_wc1.T @ R_wc0          # rotates a cam0 direction into cam1
            # depth scale source: baro height-above-takeoff (wrong over terrain) or
            # true AGL = camera_alt - terrain_elev (the metric fix we are testing).
            h0 = max(agl[i - 1] if agl is not None else r0["h"], 0.3)

            A, b, tfl = [], [], []
            for (u0, v0), (u1, v1) in zip(p0g, p1g):
                n0 = Kinv @ np.array([u0, v0, 1.0])     # normalized ray cam0
                n1 = Kinv @ np.array([u1, v1, 1.0])
                x0, y0 = n0[0], n0[1]
                # predicted location under rotation only (point at infinity)
                pr = R_c1c0 @ n0
                if pr[2] <= 1e-6:
                    continue
                pr = pr / pr[2]
                du, dv = n1[0] - pr[0], n1[1] - pr[1]   # translational flow (norm)
                # ground depth along the ray: C_z + Z*(R_wc0 @ n0)_z = 0, C_z = h0 (up)
                dz = (R_wc0 @ np.array([x0, y0, 1.0]))[2]
                if abs(dz) < 1e-3:
                    continue
                Z = -h0 / dz
                if Z <= 0 or Z > 50 * h0:
                    continue
                # Z*du = -tx + x0*tz ;  Z*dv = -ty + y0*tz
                A.append([-1, 0, x0]); b.append(Z * du)
                A.append([0, -1, y0]); b.append(Z * dv)
                tfl.append(np.hypot(du, dv))

            if len(A) >= 2 * min_track:
                A = np.asarray(A); b = np.asarray(b)
                t_cam, *_ = np.linalg.lstsq(A, b, rcond=None)
                # one IRLS reweight to suppress moving objects / mismatches
                resid = (A @ t_cam - b).reshape(-1, 2)
                rn = np.linalg.norm(resid, axis=1)
                med = np.median(rn) + 1e-9
                w = np.repeat((rn < 3 * med).astype(float), 2)
                if w.sum() >= 2 * min_track:
                    Aw, bw = A * w[:, None], b * w
                    t_cam, *_ = np.linalg.lstsq(Aw, bw, rcond=None)
                dC = R_wc0 @ t_cam            # camera = body origin offset ignored (4cm)
                pos[0] += dC[0]
                pos[1] += dC[1]
                used = int(w.sum() // 2) if 'w' in dir() else len(b) // 2
                # validation only: depth implied by flow + GT horizontal step.
                # (compares the TRUE camera-to-ground depth against baro height.)
                gstep = np.linalg.norm((r1["gt"] - r0["gt"])[:2])
                mtf = np.median(tfl) if tfl else 0.0
                if gstep > 0.4 and mtf > 1e-4:
                    idep = gstep / mtf

        pos[2] = r1["h"]                      # altitude straight from baro
        est.append(pos.copy())
        proc.append(i)
        n_used.append(used)
        impl_depth.append(idep)
        prev = cur
        if i % 600 == 0:
            print(f"  frame {i}/{len(recs)}  used~{used} pts")

    est = np.array(est)
    recs = [recs[k] for k in proc]            # align recs/GT to processed frames
    gt = np.array([r["gt"] for r in recs])
    return est, gt, np.array(n_used), np.array(impl_depth), recs


def plot(est, gt, n_used, impl_depth, recs, out, depth_source="baro"):
    t = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])
    err = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
    path_len = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))
    est_len = np.sum(np.linalg.norm(np.diff(est[:, :2], axis=0), axis=1))

    fig, ax = plt.subplots(2, 2, figsize=(13, 10))
    a = ax[0, 0]
    a.plot(gt[:, 0], gt[:, 1], "b-", lw=1.5, label="GT")
    a.plot(est[:, 0], est[:, 1], "r-", lw=1.2, label="flow-odom")
    a.plot(gt[0, 0], gt[0, 1], "go", ms=8); a.plot(gt[-1, 0], gt[-1, 1], "ks", ms=7)
    a.plot(est[-1, 0], est[-1, 1], "r*", ms=12)
    a.set_title("Top-down trajectory (ENU)"); a.set_xlabel("East (m)"); a.set_ylabel("North (m)")
    a.axis("equal"); a.legend(); a.grid(alpha=0.3)

    a = ax[0, 1]
    a.plot(t, err, "r-")
    a.set_title(f"Horizontal error vs GT  (final {err[-1]:.1f} m = "
                f"{100*err[-1]/path_len:.1f}% of {path_len:.0f} m path)")
    a.set_xlabel("time (s)"); a.set_ylabel("error (m)"); a.grid(alpha=0.3)

    a = ax[1, 0]
    a.axis("off")
    duration_s = t[-1] - t[0]
    max_err = err.max()
    max_err_t = t[np.argmax(err)]
    baro_h = np.array([r["h"] for r in recs])
    m_d = np.isfinite(impl_depth)
    ratio = np.nanmedian(impl_depth[m_d] / baro_h[m_d]) if m_d.any() else float("nan")
    stats = (
        f"{'FLIGHT STATISTICS':^38}\n"
        f"{'─'*38}\n"
        f"{'Depth source':<22} {depth_source}\n"
        f"{'Duration':<22} {duration_s:.0f} s  ({duration_s/60:.1f} min)\n"
        f"{'Frames processed':<22} {len(recs)}\n"
        f"{'─'*38}\n"
        f"{'PATH LENGTH':<22}\n"
        f"  GT                   {path_len:.1f} m\n"
        f"  Estimated            {est_len:.1f} m\n"
        f"  Scale (est/GT)       {est_len/path_len:.3f}\n"
        f"{'─'*38}\n"
        f"{'HORIZONTAL ERROR':<22}\n"
        f"  Final                {err[-1]:.1f} m  ({100*err[-1]/path_len:.2f}%)\n"
        f"  Mean                 {err.mean():.1f} m  ({100*err.mean()/path_len:.2f}%)\n"
        f"  Max                  {max_err:.1f} m  ({100*max_err/path_len:.2f}%)  @{max_err_t:.0f}s\n"
        f"  RMSE                 {np.sqrt((err**2).mean()):.1f} m\n"
        f"{'─'*38}\n"
        f"{'AGL (depth source)':<22}\n"
        f"  Median               {np.nanmedian(impl_depth[m_d]):.0f} m\n"
        f"  Range                {np.nanmin(impl_depth[m_d]):.0f}–{np.nanmax(impl_depth[m_d]):.0f} m\n"
        + (f"  Baro depth ratio     {ratio:.1f}x\n" if np.isfinite(ratio) else "")
    )
    a.text(0.05, 0.97, stats, transform=a.transAxes, va="top", ha="left",
           fontsize=9.5, fontfamily="monospace",
           bbox=dict(boxstyle="round,pad=0.5", facecolor="#f7f7f7", edgecolor="#aaaaaa"))

    # The key diagnosis: TRUE camera-to-ground depth (from flow + GT) vs the baro
    # height-above-takeoff that the scaling assumes.  On Cesium real terrain the
    # ground falls away into a valley, so depth >> baro height -> scale too small.
    a = ax[1, 1]
    a.plot(t[m_d], impl_depth[m_d], "m.", ms=3, label="true depth (flow+GT)")
    a.plot(t, baro_h, "g-", lw=1.5, label="baro height a.g.t.")
    a.set_title(f"Why scale is off: ground is ~{ratio:.0f}x deeper than baro height\n"
                "(terrain falls away — baro a.g.t. != camera-ground depth)")
    a.set_xlabel("time (s)"); a.set_ylabel("metres"); a.legend(); a.grid(alpha=0.3)

    fig.suptitle(f"Altitude-scaled optical-flow odometry vs GT — Isaac Sim nadir "
                 f"(depth source: {depth_source})", fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"\nsaved {out}")
    print(f"depth source      : {depth_source}")
    print(f"final horiz error : {err[-1]:.1f} m  ({100*err[-1]/path_len:.1f}% of path)")
    print(f"mean  horiz error : {err.mean():.1f} m")
    print(f"path length  GT   : {path_len:.0f} m")
    print(f"path length  est  : {est_len:.0f} m   (scale est/GT = {est_len/path_len:.2f})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--max_frames", type=int, default=0)
    ap.add_argument("--min_track", type=int, default=30)
    ap.add_argument("--depth", choices=["baro", "agl"], default="baro",
                    help="depth/scale source: 'baro' height-above-takeoff (wrong over "
                         "terrain) or 'agl' true camera-alt - terrain-elev (stand-in for baro-DEM)")
    ap.add_argument("--stride", type=int, default=1,
                    help="process every Nth frame (bigger baseline -> better flow SNR at high fps)")
    ap.add_argument("--no_fb", action="store_true", help="disable forward-backward LK consistency check")
    ap.add_argument("--skip_frames", type=int, default=0, help="skip first N frames (to run a later segment)")
    ap.add_argument("--attitude", choices=["gt", "ahrs", "ahrs_compass"], default="gt",
                    help="gt = GT quaternions; ahrs = Mahony gyro+accel only; "
                         "ahrs_compass = Mahony + perfect-compass yaw correction")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    if args.out:
        out = args.out
    else:
        ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        out = os.path.join(ROOT, "experiment", os.path.basename(args.dir),
                           f"flow_{args.depth}_{args.attitude}_s{args.stride}.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    att_R = None
    if args.attitude == "ahrs":
        _, _, recs0 = load_dataset(args.dir)
        att_R = compute_ahrs_attitude(args.dir, recs0)
        print("  using Mahony AHRS attitude (gyro+accel only, no compass)")
    elif args.attitude == "ahrs_compass":
        _, _, recs0 = load_dataset(args.dir)
        att_R = compute_ahrs_attitude(args.dir, recs0, mag_gain=1.0)
        print("  using Mahony AHRS + compass (yaw corrected from GT heading proxy)")
    est, gt, n_used, impl_depth, recs = run(args.dir, args.scale, args.max_frames,
                                            args.min_track, args.depth, args.stride,
                                            not args.no_fb, skip_frames=args.skip_frames,
                                            attitude_R=att_R)
    tvec = np.array([(r["ts"] - recs[0]["ts"]) / 1e9 for r in recs])
    if args.attitude == "gt":
        traj_cache = os.path.join(args.dir, "tracker_trajs.npz")
        if not os.path.exists(traj_cache):
            np.savez(traj_cache, LK=est[:, :2], GT=gt[:, :2], t_vec=tvec)
            print(f"saved trajectory cache → {traj_cache}")
    else:
        traj_cache = os.path.join(args.dir, f"tracker_trajs_{args.attitude}_{args.depth}.npz")
        np.savez(traj_cache, LK=est[:, :2], GT=gt[:, :2], t_vec=tvec)
        print(f"saved trajectory cache → {traj_cache}")
    plot(est, gt, n_used, impl_depth, recs, out, args.depth)
