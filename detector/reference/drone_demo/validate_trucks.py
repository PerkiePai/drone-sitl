#!/usr/bin/env python3
"""ตรวจตำแหน่งรถใน trucks_all.json กับภาพจากรอบบินที่เคยบินแล้ว — ไม่ต้องบินใหม่ ไม่ต้องเปิด Isaac (DEPLOY_GUIDE C1b)

    python3 validate_trucks.py --run-dir <โฟลเดอร์รอบบินเก่า>                  # ตรวจ trucks_all.json ปัจจุบัน
    python3 validate_trucks.py --run-dir <โฟลเดอร์รอบบินเก่า> --old-targets    # ตรวจเป้าที่รอบนั้นใช้บินจริง

ทำอะไร
  1) ฉายศูนย์กลางรถแต่ละคันลงทุกเฟรมที่บันทึกไว้ (คิดท่าทางโดรน roll/pitch/yaw ครบ — กล้องติดตาย)
     แล้วดูว่า detector ตีกรอบตรงนั้นไหม -> ถ้าตำแหน่งถูก เฟรมที่รถ "ควรอยู่ในภาพ" ต้องมีกรอบเกือบทุกเฟรม
  2) หาตำแหน่งรถที่ detector เห็นจริงจากวิดีโอ (กรอบเลื่อนตามการบิน -> คำนวณ x, y, z ย้อนกลับ)
     แล้วบอกว่ารถที่เห็นใกล้ศูนย์กลางใน json ที่สุดอยู่ห่างเท่าไร

ผลรอบบินจริง 2026-09-13 (runs/drone demo/real_flight) ที่ใช้พิสูจน์ปัญหา: เป้าเดิมจากกลาง bbox ผิดทุกคันที่ตรวจได้
"""

import argparse
import bisect
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import config as C                     # noqa: E402

FX = (C.IMG_W / 2.0) / math.tan(C.HFOV_RAD / 2.0)     # ≈ 1466 px
CX, CY = C.IMG_W / 2.0, C.IMG_H / 2.0
MAX_POSE_GAP_S = 0.1
HIT_MARGIN_PX = 30
IN_VIEW_MARGIN_PX = 20
MIN_IN_VIEW = 20
OK_RATIO, BAD_RATIO = 0.5, 0.15
MATCH_XY, MATCH_Z = 10.0, 10.0


# --------------------------------------------------------------------------- geometry (กล้องชี้ลง ติดตายกับตัวโดรน)
def r_nb(roll_deg, pitch_deg, yaw_deg):
    """ตัวโดรน FRD -> NED (PX4: Rz(yaw) Ry(pitch) Rx(roll)) · คืน 3×3 เป็น list"""
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def project(point_enu, pose, axis_map=None):
    """จุดโลก ENU -> พิกเซล (u, v) หรือ None ถ้าอยู่หลังกล้อง"""
    m = axis_map or C.IMAGE_AXIS_MAP
    x, y, z = pose["pos_enu"]
    ned = (point_enu[1] - y, point_enu[0] - x, z - point_enu[2])
    R = r_nb(pose.get("roll_deg", 0.0), pose.get("pitch_deg", 0.0), pose.get("yaw_deg", 0.0))
    f, r, d = (sum(R[i][k] * ned[i] for i in range(3)) for k in range(3))        # Rᵀ · ned
    if d <= 1e-6:
        return None
    return (CX + FX * (m[0][0] * f + m[0][1] * r) / d, CY + FX * (m[1][0] * f + m[1][1] * r) / d)


def backproject(u, v, pose, z_plane, axis_map=None):
    """พิกเซล -> จุดบนระนาบ z = z_plane (x, y) หรือ None"""
    m = axis_map or C.IMAGE_AXIS_MAP
    a, b = (u - CX) / FX, (v - CY) / FX
    det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
    f = (m[1][1] * a - m[0][1] * b) / det
    r = (-m[1][0] * a + m[0][0] * b) / det
    R = r_nb(pose.get("roll_deg", 0.0), pose.get("pitch_deg", 0.0), pose.get("yaw_deg", 0.0))
    ned = [R[i][0] * f + R[i][1] * r + R[i][2] for i in range(3)]
    if ned[2] <= 1e-6:
        return None
    t = (pose["pos_enu"][2] - z_plane) / ned[2]
    if t <= 0:
        return None
    return (pose["pos_enu"][0] + t * ned[1], pose["pos_enu"][1] + t * ned[0])


# --------------------------------------------------------------------------- data
def read_jsonl(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


class Poses:
    def __init__(self, flight):
        self.f = sorted(flight, key=lambda r: r["t"])
        self.t = [r["t"] for r in self.f]

    def at(self, t):
        i = bisect.bisect_left(self.t, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.f)]
        if not cands:
            return None
        j = min(cands, key=lambda k: abs(self.t[k] - t))
        return self.f[j] if abs(self.t[j] - t) <= MAX_POSE_GAP_S else None


def targets_from_run_meta(meta, alt=None):
    """เป้าที่รอบบินนั้นใช้จริง: ขา hover -> ศูนย์กลางรถ = เป้า − ความสูงเหนือรถ"""
    alt = float(meta.get("alt", C.ALT_ABOVE_TRUCK) if alt is None else alt)
    out, seen = [], set()
    for leg in meta.get("legs", []):
        if leg.get("phase") == "hover" and leg.get("truck") and leg["truck"] not in seen:
            seen.add(leg["truck"])
            x, y, z = leg["target"]
            out.append(dict(name=leg["truck"], center=[x, y, z - alt], center_source="run_meta (เป้าเดิม)", test=True))
    return out


# --------------------------------------------------------------------------- checks
def check_truck(truck, poses, dets, axis_map=None):
    """เฟรมที่ศูนย์กลางรถควรอยู่ในภาพ / เฟรมที่มีกรอบครอบจุดนั้น / อัตราขนาดกรอบเทียบที่ควรเป็น"""
    c = truck["center"]
    in_view = hits = 0
    size_ratio = []
    for d in dets:
        pose = poses.at(d["t"])
        if pose is None or pose.get("phase") in ("prestream",):
            continue
        px = project(c, pose, axis_map)
        if px is None:
            continue
        u, v = px
        if not (IN_VIEW_MARGIN_PX <= u <= C.IMG_W - IN_VIEW_MARGIN_PX and IN_VIEW_MARGIN_PX <= v <= C.IMG_H - IN_VIEW_MARGIN_PX):
            continue
        in_view += 1
        for b in d.get("boxes") or []:
            if b[0] - HIT_MARGIN_PX <= u <= b[2] + HIT_MARGIN_PX and b[1] - HIT_MARGIN_PX <= v <= b[3] + HIT_MARGIN_PX:
                hits += 1
                depth = pose["pos_enu"][2] - c[2]
                if depth > 1:
                    size_ratio.append(max(b[2] - b[0], b[3] - b[1]) / (FX * C.TRUCK_LEN_UNITS / depth))
                break
    size_ratio.sort()
    return dict(in_view=in_view, hits=hits, ratio=(hits / in_view) if in_view else None,
                size_ratio=size_ratio[len(size_ratio) // 2] if size_ratio else None)


def locate_detected(poses, dets, axis_map=None, z_range=(-200.0, 300.0), z_step=2.0, min_len=20):
    """ตำแหน่งรถที่ detector เห็นจริง จากกรอบที่ติดตามได้ต่อเนื่องขณะโดรนบินผ่านที่ความสูงคงที่

    สำหรับแต่ละเส้นทางกรอบ: ลองความสูงรถหลายค่า -> ค่าที่ทำให้ทุกเฟรมชี้จุดเดียวกันบนพื้นคือคำตอบ
    """
    tracks = []
    for d in dets:
        pose = poses.at(d["t"])
        if pose is None:
            continue
        for b in d.get("boxes") or []:
            u, v = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
            best = None
            for tr in tracks:
                lt, lu, lv = tr[-1][0], tr[-1][1], tr[-1][2]
                if 0 < d["t"] - lt <= 0.4:
                    dist = math.hypot(u - lu, v - lv)
                    if dist < 80 and (best is None or dist < best[0]):
                        best = (dist, tr)
            rec = (d["t"], u, v, pose, b[4] if len(b) > 4 else None)
            if best:
                best[1].append(rec)
            else:
                tracks.append([rec])
    found = []
    for tr in tracks:
        if len(tr) < min_len:
            continue
        zs = [r[3]["pos_enu"][2] for r in tr]
        xs = [r[3]["pos_enu"][0] for r in tr]
        ys = [r[3]["pos_enu"][1] for r in tr]
        if max(zs) - min(zs) > 3 or math.hypot(max(xs) - min(xs), max(ys) - min(ys)) < 20:
            continue
        best = None
        z = z_range[0]
        while z <= z_range[1]:
            pts = [backproject(r[1], r[2], r[3], z, axis_map) for r in tr]
            pts = [p for p in pts if p]
            if len(pts) >= min_len:
                mx = sum(p[0] for p in pts) / len(pts)
                my = sum(p[1] for p in pts) / len(pts)
                spread = math.sqrt(sum((p[0] - mx) ** 2 + (p[1] - my) ** 2 for p in pts) / len(pts))
                if best is None or spread < best[0]:
                    best = (spread, z, mx, my)
            z += z_step
        if best and best[0] <= 3.0:
            found.append(dict(xyz=[round(best[2], 1), round(best[3], 1), round(best[1], 1)],
                              spread=round(best[0], 2), frames=len(tr)))
    merged = []
    for f in sorted(found, key=lambda f: -f["frames"]):
        m = next((g for g in merged if math.hypot(g["xyz"][0] - f["xyz"][0], g["xyz"][1] - f["xyz"][1]) <= 8), None)
        if m:
            m["frames"] += f["frames"]
            m["seen"] += 1
        else:
            merged.append(dict(f, seen=1))
    return merged


def verdict(chk, near):
    """✅ ตรง / ❌ ผิด / ❔ ตรวจไม่ได้"""
    matched = near is not None and near["dxy"] <= MATCH_XY and abs(near["dz"]) <= MATCH_Z
    if chk["in_view"] >= MIN_IN_VIEW and chk["ratio"] >= OK_RATIO:
        return "ok"
    if matched:
        return "ok"
    if chk["in_view"] >= MIN_IN_VIEW and chk["ratio"] < BAD_RATIO:
        return "bad"
    if near is not None and near["dxy"] <= 100 and (near["dxy"] > MATCH_XY or abs(near["dz"]) > 3 * MATCH_Z):
        return "suspect"
    return "unknown"


def evaluate(trucks, flight, dets, names=None, axis_map=None):
    poses = Poses(flight)
    dets = sorted(dets, key=lambda d: d["t"])
    seen = locate_detected(poses, dets, axis_map)
    rows = []
    for t in trucks:
        if names and t["name"] not in names:
            continue
        chk = check_truck(t, poses, dets, axis_map)
        near = None
        for s in seen:
            dxy = math.hypot(s["xyz"][0] - t["center"][0], s["xyz"][1] - t["center"][1])
            if near is None or dxy < near["dxy"]:
                near = dict(dxy=round(dxy, 1), dz=round(t["center"][2] - s["xyz"][2], 1), xyz=s["xyz"])
        rows.append(dict(name=t["name"], center=t["center"], source=t.get("center_source"), check=chk, nearest_seen=near,
                         verdict=verdict(chk, near)))
    return dict(rows=rows, seen=seen)


def input_problem(trucks, flight, dets, names=None):
    """ข้อมูลที่ตรวจต่อไม่ได้ -> ข้อความบอกวิธีแก้ (None = ตรวจได้) — กันตารางว่างที่ดูเหมือนผ่าน (2026-09-27)"""
    have = [t for t in trucks if not names or t.get("name") in names]
    if not trucks:
        return ("trucks_all.json ไม่มีรถเลย — น่าจะมาจากรัน export_trucks.py ตอนเปิดฉาก 'No Truck' "
                "-> เปิด 'Map With Truck dataset.usd' แล้วทำ C1 ใหม่")
    if not have:
        return f"trucks_all.json ไม่มีรถเป้าหมาย {sorted(names)} -> ทำ C1 ใหม่"
    if not flight or not dets:
        return "รอบบินนี้ไม่มี flight_log / detections -> เลือกรอบ real_flight (13 ก.ย. 01:39)"
    boxed = sum(1 for d in dets if d.get("boxes"))
    if boxed < 50:
        return (f"รอบบินนี้มีเฟรมที่ detector ตีกรอบแค่ {boxed} เฟรม — ใช้ตรวจไม่ได้ "
                "-> เลือกรอบ real_flight (13 ก.ย. 01:39 · บินครบ 5 คัน)")
    return None


ICON = dict(ok="✅ ตรง", bad="❌ ผิด", suspect="⚠️ น่าสงสัย", unknown="❔ ตรวจไม่ได้")


def print_report(rep, label):
    print(f"\nตรวจ {label}")
    print(f"{'รถ':<11}{'ศูนย์กลางใน json':<28}{'ควรเห็น':>8}{'เจอกรอบ':>8}{'%':>6}{'ขนาด':>6}  {'รถที่เห็นในวิดีโอใกล้สุด':<34}ผล")
    for r in rep["rows"]:
        c, n = r["check"], r["nearest_seen"]
        pct = "-" if c["ratio"] is None else f"{c['ratio']:.0%}"
        sz = "-" if c["size_ratio"] is None else f"{c['size_ratio']:.2f}"
        near = "-" if n is None else f"{n['xyz']} ห่าง {n['dxy']:.0f} · z ต่าง {n['dz']:+.0f}"
        print(f"{r['name']:<11}{str([round(v, 1) for v in r['center']]):<28}{c['in_view']:>8}{c['hits']:>8}{pct:>6}{sz:>6}  {near:<34}{ICON[r['verdict']]}")
    print(f"\nรถที่ detector เห็นจริงในวิดีโอ (คำนวณย้อนจากการบิน): {len(rep['seen'])} คัน")
    for s in rep["seen"]:
        print(f"  {s['xyz']}  (เห็น {s['frames']} เฟรม · ความคลาดเคลื่อน {s['spread']:.1f})")
    bad = [r["name"] for r in rep["rows"] if r["verdict"] in ("bad", "suspect")]
    print("\nความหมาย: 'ควรเห็น' = เฟรมที่ศูนย์กลางรถควรอยู่ในภาพ · '%' ควรสูง (≥ 50%) ถ้าตำแหน่งถูก"
          " · 'ขนาด' = กรอบจริง ÷ กรอบที่ควรเป็น ควรใกล้ 1 (ห่างมาก = ความสูง z ผิด)")
    if bad:
        print(f"❌ {bad}: ตำแหน่งไม่ตรงกับภาพจริง — อย่าเพิ่งบินไปคันเหล่านี้ (รัน export_trucks.py ใหม่ / ส่งผลนี้มาให้ดู)")
    return 1 if bad else 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--run-dir", required=True, help="โฟลเดอร์รอบบินเก่า (มี flight_log.jsonl + detections.jsonl) · LATEST ได้")
    p.add_argument("--trucks", default=C.TRUCKS_JSON, help="trucks_all.json ที่จะตรวจ")
    p.add_argument("--old-targets", action="store_true", help="ตรวจเป้าที่รอบนั้นใช้บินจริง (จาก run_meta.json) แทน --trucks")
    p.add_argument("--all", action="store_true", help="ตรวจรถทั้ง 50 คัน (ค่าเริ่มต้น: 5 คันเป้าหมาย)")
    p.add_argument("--json", default=None, help="บันทึกผลเป็น json")
    args = p.parse_args(argv)
    try:
        run = C.resolve_run_dir(args.run_dir)
        flight = read_jsonl(os.path.join(run, "flight_log.jsonl"))
        dets = read_jsonl(os.path.join(run, "detections.jsonl"))
        if args.old_targets:
            with open(os.path.join(run, "run_meta.json")) as f:
                trucks, label = targets_from_run_meta(json.load(f)), f"เป้าเดิมของรอบ {run}"
        else:
            with open(os.path.expanduser(args.trucks)) as f:
                data = json.load(f)
            trucks = data["trucks"] if isinstance(data, dict) else data
            label = f"{args.trucks} กับภาพรอบ {run}"
    except (OSError, ValueError, KeyError) as e:
        print(f"[validate] อ่านข้อมูลไม่ได้: {e}")
        return 2
    problem = input_problem(trucks, flight, dets, None if args.all else set(C.TEST_TRUCKS))
    if problem:
        print(f"[validate] ❌ {problem}")
        return 2
    rep = evaluate(trucks, flight, dets, None if args.all else set(C.TEST_TRUCKS))
    code = print_report(rep, label)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(rep, f, ensure_ascii=False, indent=1)
    return code


if __name__ == "__main__":
    sys.exit(main())
