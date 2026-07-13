#!/usr/bin/env python3
"""Full GPS-denied navigation pipeline — best settings from Exp01–05.

Two-layer fused odometry, no GPS / no lidar / no rangefinder:
  Layer 1 — LK optical-flow odometry (local, metric, drifts)
  Layer 2 — SIFT+LightGlue DSMAC geo-localization (global, drift-free)

Best configuration (long 22 km dataset, Exp05):
  Flow-odom : LK, stride=5, scale=0.5, AGL depth, AHRS+compass attitude
  DSMAC     : SIFT+LightGlue, fix_every=30 steps, blend=0.8, reject=150 m, skip_below=13 m
  Result    : RMSE ~15 m / final ~1 m over 22 km

Inputs (per dataset dir):
  cam_calib.json, frames.csv, poses.csv, baro.csv, imu.csv,
  geo.csv, georef.json, agl_cache.npz, ortho_tiles/ (cached Esri tiles)

GT is used ONLY for final scoring. All navigation is GT-free.

Run:
  conda run -n drone python pipeline.py --dir _in/isaac-sim-20260625
"""
import argparse, csv, json, math, os, sys, time, urllib.request

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import torch

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
import flow_odometry as fo

# ─── constants ───────────────────────────────────────────────────────────────

DEVICE    = "cuda" if torch.cuda.is_available() else "cpu"
MAX_KPTS  = 2048
TILE_URL  = ("https://services.arcgisonline.com/ArcGIS/rest/services/"
             "World_Imagery/MapServer/tile/{z}/{y}/{x}")
LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FEAT_PARAMS = dict(maxCorners=600, qualityLevel=0.01, minDistance=8, blockSize=7)
FB_THRESH   = 1.0  # forward-backward pixel consistency threshold


# ─── SIFT + LightGlue ────────────────────────────────────────────────────────

def make_sift_lg():
    """Return a SIFT+LightGlue matcher with .extract(gray) and .match(f0, f1)."""
    from lightglue import SIFT, LightGlue
    from lightglue.utils import rbd
    ext = SIFT(max_num_keypoints=MAX_KPTS).eval().to(DEVICE)
    mat = LightGlue(features="sift").eval().to(DEVICE)
    print(f"  SIFT+LightGlue loaded on {DEVICE}")

    def _t(gray):
        return torch.from_numpy(gray)[None, None].float().to(DEVICE) / 255.0

    class _Matcher:
        @torch.no_grad()
        def extract(self, gray):
            return ext.extract(_t(gray), resize=None)

        @torch.no_grad()
        def match(self, f0, f1):
            m   = mat({"image0": f0, "image1": f1})
            f0r, f1r, mr = rbd(f0), rbd(f1), rbd(m)
            idx = mr["matches"]
            k0  = f0r["keypoints"][idx[:, 0]].cpu().numpy()
            k1  = f1r["keypoints"][idx[:, 1]].cpu().numpy()
            return k0, k1

    return _Matcher()


# ─── satellite ortho helpers ──────────────────────────────────────────────────

def _deg2tile(lat, lon, z):
    n = 2 ** z
    return ((lon + 180) / 360 * n,
            (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)


def _probe_zoom(lat, lon, z_max, z_min=15):
    """Esri World Imagery doesn't cover every location at every zoom -- deep
    zooms over lower-priority (e.g. rural) areas fall back to a flat grey
    'Map data not yet available' placeholder tile, identical across all such
    locations and much smaller than a real detailed tile (~2.5 KB vs 6.5+ KB
    here). Probe the centre tile top-down from z_max and return the first zoom
    with real imagery, so DSMAC never matches against a placeholder."""
    for z in range(z_max, z_min - 1, -1):
        x, y = _deg2tile(lat, lon, z)
        url = TILE_URL.format(z=z, y=int(y), x=int(x))
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "research-prototype"})
            data = urllib.request.urlopen(req, timeout=20).read()
        except Exception:
            continue
        if len(data) > 4000:      # real tiles are markedly larger than the placeholder
            return z
    print(f"  WARNING: no Esri coverage found down to z{z_min} — using z{z_min} anyway")
    return z_min


def build_ortho(geo, z, margin, tiles_dir):
    """Stitch Esri tiles covering the geo.csv footprint; cached per tile."""
    os.makedirs(tiles_dir, exist_ok=True)
    lat = np.array([float(r["lat_deg"]) for r in geo])
    lon = np.array([float(r["lon_deg"]) for r in geo])
    la0, la1 = lat.min() - margin, lat.max() + margin
    lo0, lo1 = lon.min() - margin, lon.max() + margin
    x0, _  = _deg2tile(la0, lo0, z);  x1, _  = _deg2tile(la1, lo1, z)
    _,  y0 = _deg2tile(la1, lo0, z);  _,  y1 = _deg2tile(la0, lo1, z)
    xa, xb = int(math.floor(min(x0, x1))), int(math.floor(max(x0, x1)))
    ya, yb = int(math.floor(min(y0, y1))), int(math.floor(max(y0, y1)))
    W, H   = (xb - xa + 1) * 256, (yb - ya + 1) * 256
    mem_mb = W * H * 3 / 1e6
    print(f"  estimated ortho RAM: {mem_mb:.0f} MB  ({W}×{H} px)")
    if mem_mb > 500:
        print(f"  WARNING: ortho > 500 MB — consider reducing zoom (current z={z})")
    ortho  = np.zeros((H, W, 3), np.uint8)
    placed = 0
    for ty in range(ya, yb + 1):
        for tx in range(xa, xb + 1):
            fp = os.path.join(tiles_dir, f"{z}_{ty}_{tx}.jpg")
            if not os.path.exists(fp):
                url = TILE_URL.format(z=z, y=ty, x=tx)
                for attempt in range(3):
                    try:
                        req = urllib.request.Request(
                            url, headers={"User-Agent": "research-prototype"})
                        with open(fp, "wb") as fh:
                            fh.write(urllib.request.urlopen(req, timeout=20).read())
                        break
                    except Exception:
                        if attempt == 2:
                            print(f"  tile fail {z}/{ty}/{tx}")
                        time.sleep(0.5)
            im = cv2.imread(fp) if os.path.exists(fp) else None
            if im is not None and im.shape[:2] == (256, 256):
                ortho[(ty-ya)*256:(ty-ya+1)*256,
                      (tx-xa)*256:(tx-xa+1)*256] = im
                placed += 1
    meta = dict(z=z, xa=xa, ya=ya, W=W, H=H)
    print(f"  ortho {xb-xa+1}×{yb-ya+1} tiles = {W}×{H}px "
          f"({placed}/{(xb-xa+1)*(yb-ya+1)} placed)")
    return ortho, meta


def warp_north_up(im, yaw_deg, f):
    """De-rotate by -yaw and scale by f; returns (warped_img, frame_centre_pt)."""
    h, w = im.shape
    M = cv2.getRotationMatrix2D((w / 2, h / 2), -yaw_deg, f)
    cos, sin = abs(M[0, 0]), abs(M[0, 1])
    nw = int(w * cos + h * sin)
    nh = int(w * sin + h * cos)
    M[0, 2] += nw / 2 - w / 2
    M[1, 2] += nh / 2 - h / 2
    return cv2.warpAffine(im, M, (nw, nh)), M @ np.array([w / 2, h / 2, 1.0])


# ─── canopy / repetitive-terrain gate ────────────────────────────────────────
# Exp08 found forest canopy gives DSMAC ~0% clear rate regardless of AGL or zoom
# (absence of correspondable structure between sim-rendered canopy and real Esri
# imagery, not a weak-matcher problem -- SIFT already finds 1000+ keypoints on
# both sides). This gate does NOT fix that; it only skips the SIFT+LightGlue
# call on windows already known to be hopeless, saving GPU time. Thresholds
# calibrated in experiment/09/validate_gate.py against Exp08's 164 known-outcome
# samples -- see experiment/09/result.md for the calibration and its caveats.
GREEN_DOMINANCE_THRESH = 22.09  # Exp09 validate_gate.py: lowest 95th-pctile sweep
                                # step with precision=1.0 and zero false positives
                                # on dataset 1 (the only dataset with real fixes to
                                # lose); recall=0.38 of all failed samples.
REPETITIVENESS_THRESH  = 0.56   # same calibration; precision=1.0, recall=0.13 alone,
                                # lifts color_texture's combined recall to 0.47.


def green_dominance(window_bgr):
    """Mean(G-R) over a color ortho crop. Canopy/forest renders strongly
    green-dominant in real Esri imagery vs. farmland/urban."""
    b, g, r = cv2.split(window_bgr.astype(np.float32))
    return float((g - r).mean())


def repetitiveness(window_gray, patch_frac=0.3, exclusion_mult=1.0):
    """Height of the strongest secondary autocorrelation peak: a central
    sub-patch matched against the full window via normalized cross-
    correlation, masking a patch-sized box around the trivial true peak
    (the patch always matches itself there with score ~1.0). High = the
    window contains another region nearly identical to its own centre
    (periodic/repetitive terrain); low = locally unique."""
    h, w = window_gray.shape
    ph, pw = int(h * patch_frac), int(w * patch_frac)
    cy0, cx0 = (h - ph) // 2, (w - pw) // 2
    patch = window_gray[cy0:cy0 + ph, cx0:cx0 + pw]
    corr = cv2.matchTemplate(window_gray, patch, cv2.TM_CCOEFF_NORMED)
    ey, ex = int(ph * exclusion_mult), int(pw * exclusion_mult)
    y0e, y1e = max(0, cy0 - ey), min(corr.shape[0], cy0 + ey)
    x0e, x1e = max(0, cx0 - ex), min(corr.shape[1], cx0 + ex)
    masked = corr.copy()
    masked[y0e:y1e, x0e:x1e] = -1.0
    return float(masked.max())


def is_canopy_nonviable(window_bgr, mode):
    """True if this DSMAC search window should be skipped before SIFT+LightGlue.
    mode: 'color' (green-dominance only) or 'color_texture' (also gated on
    repetitiveness). Thresholds calibrated in experiment/09/validate_gate.py."""
    if green_dominance(window_bgr) >= GREEN_DOMINANCE_THRESH:
        return True
    if mode == "color_texture":
        window_gray = cv2.cvtColor(window_bgr, cv2.COLOR_BGR2GRAY)
        if repetitiveness(window_gray) >= REPETITIVENESS_THRESH:
            return True
    return False


# ─── LK optical-flow odometry ────────────────────────────────────────────────

def _detect(gray):
    return cv2.goodFeaturesToTrack(gray, mask=None, **FEAT_PARAMS)


def _track_lk(prev, cur, p0):
    """Pyramidal LK + bidirectional FB check; returns (p0g, p1g) inlier pairs."""
    p1,  st,  _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **LK_PARAMS)
    p0b, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **LK_PARAMS)
    fbe  = np.linalg.norm(p0.reshape(-1, 2) - p0b.reshape(-1, 2), axis=1)
    good = (st.reshape(-1).astype(bool) & st2.reshape(-1).astype(bool)
            & (fbe < FB_THRESH))
    return p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]


def _solve_translation(p0g, p1g, Kinv, R_wc0, R_c1c0, h0, min_track):
    """Least-squares + IRLS camera translation from flow correspondences.
    Returns (t_cam_3d, n_inliers)."""
    A, b = [], []
    for (u0, v0), (u1, v1) in zip(p0g, p1g):
        n0 = Kinv @ np.array([u0, v0, 1.0])
        n1 = Kinv @ np.array([u1, v1, 1.0])
        pr = R_c1c0 @ n0
        if pr[2] <= 1e-6:
            continue
        pr  = pr / pr[2]
        du  = n1[0] - pr[0]
        dv  = n1[1] - pr[1]
        dz  = (R_wc0 @ np.array([n0[0], n0[1], 1.0]))[2]
        if abs(dz) < 1e-3:
            continue
        Z = -h0 / dz
        if Z <= 0 or Z > 50 * h0:
            continue
        A.append([-1, 0, n0[0]]); b.append(Z * du)
        A.append([0, -1, n0[1]]); b.append(Z * dv)
    if len(A) < 2 * min_track:
        return np.zeros(3), 0
    A = np.asarray(A, float); b = np.asarray(b, float)
    t, *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = (A @ t - b).reshape(-1, 2)
    rn    = np.linalg.norm(resid, axis=1)
    w     = np.repeat((rn < 3 * (np.median(rn) + 1e-9)).astype(float), 2)
    if w.sum() >= 2 * min_track:
        t, *_ = np.linalg.lstsq(A * w[:, None], b * w, rcond=None)
    return t, int(w.sum() // 2)


# ─── main pipeline ────────────────────────────────────────────────────────────

def _ensure_geo_georef(D):
    """Newer recorder runs don't write geo.csv/georef.json directly (no more
    _CESIUM_GEOREF setup-script pass). Reconstruct them from takeoff.json's
    georef_origin + poses.csv's takeoff-anchored ENU — same absolute lat/lon/alt
    geo.csv always held, just derived instead of pre-logged. No-op if both exist."""
    geo_path, georef_path = os.path.join(D, "geo.csv"), os.path.join(D, "georef.json")
    if os.path.exists(geo_path) and os.path.exists(georef_path):
        return
    takeoff = json.load(open(os.path.join(D, "takeoff.json")))
    lat0 = takeoff["georef_origin"]["latitude_deg"]
    lon0 = takeoff["georef_origin"]["longitude_deg"]
    h0   = takeoff["georef_origin"]["height_m"]
    tx, ty, tz = takeoff["position_enu"]     # takeoff point, relative to the ENU world origin
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    if not os.path.exists(georef_path):
        json.dump({"origin": {"latitude": lat0, "longitude": lon0, "height_m": h0},
                   "is_fallback": True,
                   "frame": "local ENU (x=East, y=North, z=Up); reconstructed from "
                            "takeoff.json (georef_origin + position_enu), not pre-logged",
                   "source": "pipeline.py:_ensure_geo_georef"},
                  open(georef_path, "w"), indent=2)
        print(f"  reconstructed georef.json from takeoff.json")
    if not os.path.exists(geo_path):
        frames = list(csv.DictReader(open(os.path.join(D, "frames.csv"))))
        poses  = {int(r["frame"]): r for r in csv.DictReader(open(os.path.join(D, "poses.csv")))}
        with open(geo_path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["frame", "ts_ns", "lat_deg", "lon_deg", "alt_m"])
            for fr in frames:
                fi = int(fr["frame"])
                if fi not in poses:
                    continue
                x, y, z = float(poses[fi]["x"]), float(poses[fi]["y"]), float(poses[fi]["z"])
                lat = lat0 + (ty + y) / mlat
                lon = lon0 + (tx + x) / mlon
                alt = h0 + (tz + z)
                w.writerow([fi, fr["ts_ns"], f"{lat:.9f}", f"{lon:.9f}", f"{alt:.4f}"])
        print(f"  reconstructed geo.csv from takeoff.json + poses.csv ({len(frames)} rows)")


def run_pipeline(args):
    D  = args.dir
    ds = os.path.basename(D)
    print(f"\n=== GPS-denied pipeline: {ds} ===")

    # dataset
    K, R_CtoI, recs = fo.load_dataset(D)
    N = len(recs)
    if args.max_frames:
        recs = recs[:args.max_frames]
        N = len(recs)
    print(f"  {N} frames loaded")

    # AGL (rangefinder): real lidar.csv → triangulated cache → baro fallback
    # lidar.csv is sampled at a different tick stride than frames.csv (e.g. every
    # 5th sim tick vs every 10th) so frame numbers never exactly coincide —
    # interpolate onto the camera frame numbers instead of requiring an exact hit.
    lidar_path = os.path.join(D, "lidar.csv")
    agl = None
    if os.path.exists(lidar_path):
        lidar_rows = list(csv.DictReader(open(lidar_path)))
        lidar_fi   = np.array([int(r["frame"]) for r in lidar_rows])
        lidar_agl  = np.array([float(r["agl_m"]) for r in lidar_rows])
        order      = np.argsort(lidar_fi)
        lidar_fi, lidar_agl = lidar_fi[order], lidar_agl[order]
        frame_idx  = np.array([r["frame"] for r in recs])
        if frame_idx.min() >= lidar_fi.min() and frame_idx.max() <= lidar_fi.max():
            agl = np.interp(frame_idx, lidar_fi, lidar_agl)
            print(f"  AGL: median {np.median(agl):.0f} m  "
                  f"[{agl.min():.0f}, {agl.max():.0f}] m  (real lidar rangefinder)")
    if agl is None:
        agl_path = os.path.join(D, "agl_cache.npz")
        if os.path.exists(agl_path):
            agl = np.load(agl_path)["agl"]
            if len(agl) < N:
                agl = np.pad(agl, (0, N - len(agl)), mode="edge")
            print(f"  AGL: median {np.median(agl):.0f} m  "
                  f"[{agl.min():.0f}, {agl.max():.0f}] m  (triangulated cache)")
        else:
            agl = np.array([r["h"] for r in recs])
            print("  WARNING: no lidar.csv / agl_cache.npz found — using baro "
                  "(scale error ~5× at altitude)")

    # attitude: Mahony AHRS + magnetometer compass (no GT)
    print("  computing AHRS+compass attitude ...")
    att_R = fo.compute_ahrs_attitude(D, recs, Kp=1.0, mag_gain=args.compass_gain)
    for i in range(N):
        recs[i]["R_wb"] = att_R[i]

    # DEM raster + relief signal (Exp10) -- only built if actually needed.
    # Placed after attitude is populated: build_dem_raster needs recs[i]["R_wb"]
    # for ray direction, which is None past frame 0 until this point (these
    # datasets have no per-frame GT quaternion in poses.csv, only AHRS+compass).
    dem = None
    terrain_elev = None
    if args.relief_gate == "on":
        dem_path = os.path.join(D, "dem_cache.npz")
        if os.path.exists(dem_path):
            z = np.load(dem_path)
            dem = dict(raster=z["raster"], e0=float(z["e0"]), n0=float(z["n0"]),
                       cell_m=float(z["cell_m"]), nx=int(z["nx"]), ny=int(z["ny"]))
            print(f"  DEM raster: {dem['nx']}x{dem['ny']} cells @ {dem['cell_m']} m "
                  f"(cached)")
        else:
            print("  building offline DEM raster (whole-flight GT-position + "
                  "AHRS-attitude triangulation) ...")
            dem = fo.build_dem_raster(recs, K, R_CtoI, args.scale)
            np.savez(dem_path, raster=dem["raster"], e0=dem["e0"], n0=dem["n0"],
                     cell_m=dem["cell_m"], nx=dem["nx"], ny=dem["ny"])
            print(f"  DEM raster: {dem['nx']}x{dem['ny']} cells @ {dem['cell_m']} m, "
                  f"{dem['n_points']} triangulated points")
        baro_h = np.array([r["h"] for r in recs])
        terrain_elev = baro_h - agl

    # satellite ortho
    _ensure_geo_georef(D)
    with open(os.path.join(D, "geo.csv")) as fh:
        geo = list(csv.DictReader(fh))
    lat_c = (min(float(r["lat_deg"]) for r in geo) + max(float(r["lat_deg"]) for r in geo)) / 2
    lon_c = (min(float(r["lon_deg"]) for r in geo) + max(float(r["lon_deg"]) for r in geo)) / 2
    zoom  = _probe_zoom(lat_c, lon_c, 19)
    if zoom != 19:
        print(f"  Esri z19 not covered here — using z{zoom}")
    ortho, meta = build_ortho(geo, zoom, 0.0016, os.path.join(D, "ortho_tiles"))
    orthog     = cv2.cvtColor(ortho, cv2.COLOR_BGR2GRAY)
    nN         = 2 ** meta["z"]
    with open(os.path.join(D, "georef.json")) as fh:
        g = json.load(fh)
    lat0, lon0 = g["origin"]["latitude"], g["origin"]["longitude"]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    GSD  = 156543.03392 * math.cos(math.radians(lat0)) / nN
    fx   = K[0, 0]

    def enu_to_px(E, Nn):
        lat = lat0 + Nn / mlat;  lon = lon0 + E / mlon
        gx  = (lon + 180) / 360 * nN
        gy  = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * nN
        return (gx - meta["xa"]) * 256, (gy - meta["ya"]) * 256

    def px_to_enu(px, py):
        gx  = meta["xa"] + px / 256;  gy  = meta["ya"] + py / 256
        lon = gx / nN * 360 - 180
        lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * gy / nN))))
        return (lon - lon0) * mlon, (lat - lat0) * mlat

    # SIFT+LightGlue matcher (Exp05 winner: 95% match rate on long dataset)
    lg = make_sift_lg()

    # scaled intrinsics
    Ks    = K.copy(); Ks[:2, :] *= args.scale
    Kinv  = np.linalg.inv(Ks)
    min_track = 30

    def _load(path):
        im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        return cv2.resize(im, None, fx=args.scale, fy=args.scale,
                          interpolation=cv2.INTER_AREA)

    def _dsmac_fix(rec_i, prior_xy):
        """One SIFT+LG DSMAC fix attempt; returns (eE, eN, inliers) or None."""
        r   = recs[rec_i]
        im  = cv2.imread(r["img"], cv2.IMREAD_GRAYSCALE)
        yaw = math.degrees(math.atan2(r["R_wb"][1, 0], r["R_wb"][0, 0]))
        f   = (agl[rec_i] / fx) / GSD
        cx, cy = enu_to_px(prior_xy[0], prior_xy[1])
        x0 = int(np.clip(cx - args.win, 0, meta["W"] - 2 * args.win))
        y0 = int(np.clip(cy - args.win, 0, meta["H"] - 2 * args.win))
        win = orthog[y0:y0 + 2 * args.win, x0:x0 + 2 * args.win]
        if win.shape[0] < 50 or win.shape[1] < 50:
            return None
        if args.canopy_gate != "off":
            win_bgr = ortho[y0:y0 + 2 * args.win, x0:x0 + 2 * args.win]
            if is_canopy_nonviable(win_bgr, args.canopy_gate):
                skipped_canopy[0] += 1
                return None
        # Hm maps q-keypoints → win-keypoints.  cpt is the drone image centre in
        # q-space (output of warp_north_up).  Hm @ cpt → fix in win-space → add
        # (x0, y0) for ortho-pixel coords → px_to_enu.
        q, cpt = warp_north_up(im, yaw, f)
        try:
            k0, k1 = lg.match(lg.extract(q), lg.extract(win))
            if len(k0) < 8:
                return None
            Hm, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 5.0)
            if Hm is None or mask is None or int(mask.sum()) < args.min_inliers:
                return None
            p = Hm @ np.array([cpt[0], cpt[1], 1.0]); p /= p[2]
            eE, eN = px_to_enu(x0 + p[0], y0 + p[1])
            return eE, eN, int(mask.sum())
        except Exception:
            return None

    def _relief_fix(fused_list, frame_idx_list):
        """TERCOM-style relief-correlation fix: slide the last `--relief_window`
        steps' dead-reckoned position trace over a 2D grid of candidate offsets
        in the DEM raster; score each candidate by Pearson correlation between
        the sensed terrain_elev shape and the DEM values there (correlation is
        already scale/offset-invariant, so no separate affine fit is needed --
        only shape is trusted, not the sensed signal's absolute scale/bias);
        accept the best-scoring candidate if both signals show real spread and
        the correlation clears the threshold. Returns (eE, eN, corr) or None."""
        W = args.relief_window
        if len(fused_list) < W:
            return None
        path   = np.array(fused_list[-W:])                       # (W,2) ENU
        idxs   = frame_idx_list[-W:]
        sensed = np.array([terrain_elev[fi] for fi in idxs])
        if np.std(sensed) < args.relief_min_sensed_std:
            return None    # sensed profile itself is flat -- no shape to match

        raster = dem["raster"]; e0, n0, cell_m = dem["e0"], dem["n0"], dem["cell_m"]
        ny, nx = raster.shape

        def sample(cand_path):
            gx = (cand_path[:, 0] - e0) / cell_m
            gy = (cand_path[:, 1] - n0) / cell_m
            if gx.min() < 0 or gy.min() < 0 or gx.max() >= nx - 1 or gy.max() >= ny - 1:
                return None
            x0 = np.floor(gx).astype(int); y0 = np.floor(gy).astype(int)
            fx = gx - x0; fy = gy - y0
            v00 = raster[y0,     x0    ]; v10 = raster[y0,     x0 + 1]
            v01 = raster[y0 + 1, x0    ]; v11 = raster[y0 + 1, x0 + 1]
            vals = (v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy)
                    + v01 * (1 - fx) * fy + v11 * fx * fy)
            return None if np.isnan(vals).any() else vals

        best = None
        offsets = np.arange(-args.relief_win, args.relief_win + 1e-6, dem["cell_m"])
        for dE in offsets:
            for dN in offsets:
                dem_vals = sample(path + np.array([dE, dN]))
                if dem_vals is None or np.std(dem_vals) < args.relief_min_relief_std:
                    continue
                corr = np.corrcoef(sensed, dem_vals)[0, 1]
                if np.isfinite(corr) and (best is None or corr > best[2]):
                    best = (dE, dN, corr)

        if best is None or best[2] < args.relief_min_corr:
            return None
        dE, dN, corr = best
        eE, eN = path[-1, 0] + dE, path[-1, 1] + dN
        return eE, eN, float(corr)

    # ── integrated flow-odom + DSMAC loop ────────────────────────────────────
    # Initial position: GT nadir point at takeoff.
    # In deployment, replace with first GPS fix or known takeoff coordinates.
    pos         = recs[0]["gt"][:2].copy()
    if args.init_offset_m > 0:
        _rng = np.random.default_rng(args.init_seed)
        pos  = pos + _rng.normal(0, args.init_offset_m, size=2)
        print(f"  init offset applied: {np.linalg.norm(pos - recs[0]['gt'][:2]):.1f} m "
              f"(seed {args.init_seed})")
    fused       = [pos.copy()]
    gt_list     = [recs[0]["gt"][:2]]
    ts_list     = [recs[0]["ts"]]
    n_used      = []
    fixes       = []   # (step, eE, eN, accepted, dist_m, score, source)
    frame_idx_list = []
    drift_since = 0.0
    step        = 0
    warmup_jumps = []   # |fix - pos| residuals collected at blend=1.0
    dsmac_std    = None # frozen after warmup; None means still warming up
    prev        = _load(recs[0]["img"])
    skipped_canopy = [0]   # mutable counter, closed over by _dsmac_fix
    relief_attempts_total = [0]   # times the canopy gate routed to _relief_fix,
                                   # regardless of whether it returned None

    total_steps = (N - 1) // args.stride
    print(f"  {total_steps} flow-odom steps — "
          f"DSMAC every {args.fix_every} steps  "
          f"(skip_below={args.skip_below} m, reject={args.reject} m) ...")

    for i in range(args.stride, N, args.stride):
        r0, r1 = recs[i - args.stride], recs[i]
        cur    = _load(r1["img"])

        # LK: detect on prev, track to cur
        p0 = _detect(prev)
        p0g = p1g = np.empty((0, 2), np.float32)
        if p0 is not None and len(p0) >= min_track:
            p0g, p1g = _track_lk(prev, cur, p0)

        # odometry: de-rotate → per-point AGL depth → LS solve
        dC = np.zeros(2); used = 0
        if len(p0g) >= min_track:
            R_wc0  = r0["R_wb"] @ R_CtoI
            R_wc1  = r1["R_wb"] @ R_CtoI
            R_c1c0 = R_wc1.T @ R_wc0
            h0     = max(float(agl[i - args.stride]), 0.3)
            t_cam, used = _solve_translation(
                p0g, p1g, Kinv, R_wc0, R_c1c0, h0, min_track)
            dC = (R_wc0 @ t_cam)[:2]

        pos         = pos + dC
        drift_since += float(np.linalg.norm(dC))
        step        += 1

        # DSMAC / relief: fire every fix_every steps, once drift exceeds skip_below
        skip = dsmac_std if (args.autotune and dsmac_std is not None) else args.skip_below
        if step % args.fix_every == 0 and drift_since >= skip:
            fix, source = None, None
            nonviable = False
            if args.canopy_gate != "off":
                cx, cy = enu_to_px(pos[0], pos[1])
                x0 = int(np.clip(cx - args.win, 0, meta["W"] - 2 * args.win))
                y0 = int(np.clip(cy - args.win, 0, meta["H"] - 2 * args.win))
                probe = ortho[y0:y0 + 2 * args.win, x0:x0 + 2 * args.win]
                nonviable = probe.shape[0] >= 50 and probe.shape[1] >= 50 \
                    and is_canopy_nonviable(probe, args.canopy_gate)
            if not nonviable:
                fix = _dsmac_fix(i, pos)
                source = "dsmac"
            elif args.relief_gate == "on":
                relief_attempts_total[0] += 1
                fix = _relief_fix(fused, frame_idx_list)
                source = "relief"
            if fix is not None:
                eE, eN, score = fix
                d   = math.hypot(eE - pos[0], eN - pos[1])
                if args.autotune and dsmac_std is not None:
                    reject = drift_since + 3 * dsmac_std
                else:
                    reject = args.reject
                acc = d <= reject
                if acc:
                    if args.autotune:
                        if len(warmup_jumps) < args.warmup_fixes:
                            gt_err = float(np.linalg.norm(pos - recs[i]["gt"][:2]))
                            print(f"  warmup {len(warmup_jumps)+1}/{args.warmup_fixes}: "
                                  f"d(fix-prior)={d:.1f} m  d(flow-GT)={gt_err:.1f} m  "
                                  f"ratio={d/max(gt_err, 1e-3):.2f}")
                            warmup_jumps.append(d)
                            blend = 1.0
                        else:
                            if dsmac_std is None:
                                dsmac_std = np.std(warmup_jumps) if len(warmup_jumps) > 1 else d
                            inlier_conf = (min(1.0, score / 50) if source == "dsmac"
                                           else float(np.clip(score, 0.0, 1.0)))
                            flow_std    = drift_since * args.flow_std_coeff
                            blend       = (flow_std ** 2) / (flow_std ** 2 + dsmac_std ** 2)
                            blend       = float(np.clip(blend * inlier_conf, args.blend_floor, 1.0))
                    else:
                        blend = args.blend
                    pos = np.array([pos[0] + blend * (eE - pos[0]),
                                    pos[1] + blend * (eN - pos[1])])
                    drift_since = 0.0
                fixes.append((step, eE, eN, acc, d, score, source))

        fused.append(pos.copy())
        gt_list.append(r1["gt"][:2])
        ts_list.append(r1["ts"])
        n_used.append(used)
        frame_idx_list.append(i)
        prev = cur

        if i % 3000 == 0:
            cur_err = float(np.linalg.norm(pos - r1["gt"][:2]))
            nacc    = sum(1 for f in fixes if f[3])
            print(f"    frame {i:5d}/{N}  err {cur_err:6.0f} m  "
                  f"drift {drift_since:5.0f} m  fixes {nacc}/{len(fixes)}")

    return (np.array(fused),
            np.array(gt_list),
            (np.array(ts_list) - ts_list[0]) / 1e9,
            fixes,
            n_used,
            skipped_canopy[0],
            relief_attempts_total[0])


# ─── output ───────────────────────────────────────────────────────────────────

def report_and_plot(fused, GT, tvec, fixes, n_used, skipped_canopy,
                     relief_attempts_total, args):
    err      = np.linalg.norm(fused - GT, axis=1)
    path_len = float(np.sum(np.linalg.norm(np.diff(GT, axis=0), axis=1)))
    rmse     = float(np.sqrt(np.mean(err ** 2)))
    nacc     = sum(1 for f in fixes if f[3])

    print(f"\n{'─'*58}")
    print(f"  PIPELINE RESULTS — {os.path.basename(args.dir)}")
    print(f"{'─'*58}")
    print(f"  Path length         : {path_len:.0f} m")
    print(f"  Duration            : {tvec[-1] / 60:.1f} min")
    print(f"  Fix attempts att/acc: {len(fixes)} / {nacc}  "
          f"({100 * nacc / max(len(fixes), 1):.0f}% accepted)")
    for src in ("dsmac", "relief"):
        att = [f for f in fixes if f[6] == src]
        acc = [f for f in att if f[3]]
        print(f"    {src:<8}: {len(acc)}/{len(att)} accepted/attempted")
    if args.relief_gate == "on":
        print(f"  Relief-gate routed  : {relief_attempts_total}  "
              f"(times canopy gate flagged nonviable and _relief_fix was called, "
              f"regardless of whether it returned None)")
    print(f"  Canopy-gate skips   : {skipped_canopy}  (mode={args.canopy_gate})")
    print(f"  Fused RMSE          : {rmse:.1f} m  "
          f"({100 * rmse / path_len:.2f}% of path)")
    print(f"  Final error         : {err[-1]:.1f} m  "
          f"({100 * err[-1] / path_len:.2f}%)")
    print(f"  Mean LK inliers     : {np.mean(n_used):.0f} / step")
    print(f"{'─'*58}\n")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    a = axes[0, 0]
    a.plot(GT[:, 0], GT[:, 1], "b-", lw=1.2, label="GT track")
    a.plot(fused[:, 0], fused[:, 1], "r-", lw=1.0, alpha=0.85, label="fused")
    a.plot(GT[0, 0], GT[0, 1], "go", ms=8, label="start")
    a.plot(GT[-1, 0], GT[-1, 1], "ks", ms=7, label="end GT")
    a.plot(fused[-1, 0], fused[-1, 1], "r*", ms=10, label="end est")
    for f in fixes:
        if f[3]:
            a.plot(f[1], f[2], "r.", ms=3, alpha=0.4)
    a.plot([], [], "r.", label="DSMAC fix")
    a.set_title("Top-down trajectory (ENU)")
    a.set_xlabel("East (m)"); a.set_ylabel("North (m)")
    a.axis("equal"); a.legend(fontsize=7); a.grid(alpha=0.3)

    a = axes[0, 1]
    a.plot(tvec, err, "r-", lw=1.2)
    for f in fixes:
        idx = min(f[0], len(tvec) - 1)
        if f[3]:
            a.axvline(tvec[idx], color="r", lw=0.3, alpha=0.15)
        else:
            a.plot(tvec[idx], args.reject, "rx", ms=4, alpha=0.4)
    a.set_title(f"Position error   RMSE={rmse:.1f} m   final={err[-1]:.1f} m")
    a.set_xlabel("time (s)"); a.set_ylabel("error (m)"); a.grid(alpha=0.3)

    a = axes[1, 0]; a.axis("off")
    stats = (
        f"{'CONFIGURATION':^44}\n{'─'*44}\n"
        f"{'Tracker':<22} LK (Shi-Tomasi + pyrLK + FB)\n"
        f"{'Stride':<22} {args.stride}\n"
        f"{'Scale':<22} {args.scale}\n"
        f"{'Depth':<22} AGL (cache / rangefinder)\n"
        f"{'Attitude':<22} AHRS + compass (Mahony)\n"
        f"{'DSMAC extractor':<22} SIFT+LightGlue\n"
        f"{'fix_every':<22} {args.fix_every} steps\n"
        f"{'skip_below':<22} {'autotune (=dsmac_std)' if args.autotune else str(args.skip_below) + ' m'}\n"
        f"{'reject':<22} {'autotune (drift+3σ)' if args.autotune else str(args.reject) + ' m'}\n"
        f"{'blend':<22} {'autotune (warmup=' + str(args.warmup_fixes) + ')' if args.autotune else args.blend}\n"
        f"{'─'*44}\n"
        f"{'Path length':<22} {path_len:.0f} m\n"
        f"{'Duration':<22} {tvec[-1]/60:.1f} min\n"
        f"{'DSMAC fixes':<22} {nacc}/{len(fixes)} accepted\n"
        f"{'Canopy-gate skips':<22} {skipped_canopy} ({args.canopy_gate})\n"
        f"{'─'*44}\n"
        f"{'Fused RMSE':<22} {rmse:.1f} m ({100*rmse/path_len:.2f}%)\n"
        f"{'Final error':<22} {err[-1]:.1f} m ({100*err[-1]/path_len:.2f}%)\n"
    )
    a.text(0.04, 0.97, stats, transform=a.transAxes, va="top", ha="left",
           fontsize=8.5, fontfamily="monospace",
           bbox=dict(boxstyle="round,pad=0.5",
                     facecolor="#f0f0f0", edgecolor="#aaa"))

    a = axes[1, 1]
    if n_used:
        a.plot(tvec[1:len(n_used) + 1], n_used, "m.", ms=2)
    for f in fixes:
        if not f[3]:
            a.axvline(tvec[min(f[0], len(tvec) - 1)],
                      color="orange", lw=0.4, alpha=0.4)
    a.set_title("LK inlier count / step  (orange = rejected DSMAC fix)")
    a.set_xlabel("time (s)"); a.set_ylabel("inliers"); a.grid(alpha=0.3)

    fig.suptitle(
        f"GPS-denied pipeline  "
        f"(LK + AHRS+compass + AGL  +  SIFT+LightGlue DSMAC) — "
        f"{os.path.basename(args.dir)}",
        fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"saved {args.out}")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="GPS-denied pipeline: LK flow-odom + SIFT+LG DSMAC fusion")
    ap.add_argument("--dir", default=os.path.join(ROOT, "_in/isaac-sim-20260625"),
                    help="dataset directory")
    ap.add_argument("--stride",      type=int,   default=5,
                    help="process every Nth frame (default 5)")
    ap.add_argument("--scale",       type=float, default=0.5,
                    help="image resize factor (default 0.5)")
    ap.add_argument("--win",         type=int,   default=420,
                    help="DSMAC ortho search half-window px (default 420)")
    ap.add_argument("--fix_every",   type=int,   default=30,
                    help="DSMAC attempt cadence in flow-odom steps (default 30)")
    ap.add_argument("--reject",      type=float, default=150.0,
                    help="reject DSMAC fix if farther than this from prior (m)")
    ap.add_argument("--blend",       type=float, default=0.8,
                    help="fix blend: pos += blend*(fix - pos) (default 0.8)")
    ap.add_argument("--skip_below",  type=float, default=13.0,
                    help="skip DSMAC until estimated drift exceeds this (m)")
    ap.add_argument("--min_inliers", type=int,   default=30,
                    help="min RANSAC inliers to accept a DSMAC fix "
                         "(default 30 — Exp06: inl=15 lets low-confidence fixes "
                         "corrupt the trajectory; 30 is the validated floor)")
    ap.add_argument("--canopy_gate", choices=["off", "color", "color_texture"],
                    default="off",
                    help="skip SIFT+LightGlue on windows flagged non-viable before "
                         "matching (Exp09) -- latency optimization only, does NOT "
                         "improve fix rate/accuracy on genuinely hopeless terrain "
                         "(default off)")
    ap.add_argument("--relief_gate", choices=["off", "on"], default="off",
                    help="try a terrain-relief/DEM correlation fix (Exp10) when the "
                         "canopy gate flags DSMAC hopeless -- additive, DSMAC-only "
                         "behaviour is unchanged when this is off (default off)")
    ap.add_argument("--relief_window", type=int, default=30,
                    help="number of recent flow-odom steps used as the sensed "
                         "relief profile (default 30, matches --fix_every)")
    ap.add_argument("--relief_win", type=float, default=150.0,
                    help="relief-fix search half-window in metres (default 150, "
                         "matches the default --reject distance)")
    ap.add_argument("--relief_min_corr", type=float, default=0.6,
                    help="min Pearson correlation to accept a relief fix (default 0.6)")
    ap.add_argument("--relief_min_relief_std", type=float, default=3.0,
                    help="min DEM elevation std (m) within a candidate window to "
                         "bother scoring it -- skips flat terrain (default 3.0)")
    ap.add_argument("--relief_min_sensed_std", type=float, default=1.0,
                    help="min sensed terrain_elev std (m) to attempt a relief fix "
                         "at all -- skips a flat sensed profile (default 1.0)")
    ap.add_argument("--max_frames", type=int, default=0,
                    help="truncate to the first N loaded frames (0=all); for fast "
                         "smoke tests, mirrors flow_odometry.py's existing flag")
    ap.add_argument("--autotune",      action="store_true",
                    help="enable Kalman-style blend autotune (default: off)")
    ap.add_argument("--warmup_fixes",  type=int,   default=6,
                    help="warmup fixes before autotune activates (default: 6)")
    ap.add_argument("--flow_std_coeff", type=float, default=0.05,
                    help="drift-to-uncertainty ratio for autotune blend (default 0.05)")
    ap.add_argument("--blend_floor",   type=float, default=0.3,
                    help="min blend in autotune mode (default 0.3; 0=no floor)")
    ap.add_argument("--init_offset_m", type=float, default=0.0,
                    help="σ of Gaussian init position noise in m (0=GT init)")
    ap.add_argument("--init_seed",     type=int, default=42,
                    help="RNG seed for init offset direction (vary to test bearing-sensitivity)")
    ap.add_argument("--compass_gain",  type=float, default=1.0,
                    help="AHRS compass correction strength (0=pure gyro/accel, 1=default)")
    ap.add_argument("--out",         default=None,
                    help="output plot path (default: _out/pipeline_<dataset>.png)")
    args = ap.parse_args()

    ds = os.path.basename(args.dir)
    if args.out is None:
        out_dir = os.path.join(ROOT, "_out")
        os.makedirs(out_dir, exist_ok=True)
        args.out = os.path.join(out_dir, f"pipeline_{ds}.png")

    fused, GT, tvec, fixes, n_used, skipped_canopy, relief_attempts_total = run_pipeline(args)
    report_and_plot(fused, GT, tvec, fixes, n_used, skipped_canopy,
                     relief_attempts_total, args)


if __name__ == "__main__":
    main()
