#!/usr/bin/env python3
"""วัดผลหลังบิน — จับคู่ flight_log.jsonl กับ detections.jsonl (DRONE_FLIGHT_PLAN.md B3)

    python3 match_runs.py --run-dir LATEST        # รอบล่าสุดจาก new_run.sh
    python3 match_runs.py --run-dir ~/Tiger/runs/drone_demo/<เวลา> --trucks ~/Tiger/drone_demo/trucks_all.json

ออกผล 3 ส่วน -> report.json + ตารางบนจอ
  1) เจอรถระหว่างลอยค้าง  ← เกณฑ์ผ่าน (≥ 4/5)
  2) กล่องหลอนบนเฟรมที่ภาพไม่ทับรถคันไหนเลย  ← รายงานอย่างเดียว
  3) FPS / เวลาประมวลผล  ← รายงานอย่างเดียว

exit code: 0 = PASS · 1 = FAIL · 2 = input ใช้ไม่ได้

ต้องรู้ก่อนเชื่อตัวเลข
---------------------
* ทั้งสองไฟล์ต้องบันทึก `t = time.time()` ของเครื่องเดียวกัน (plan §0 ข้อ 2)
* การฉายตำแหน่งรถลงภาพพึ่ง IMAGE_AXIS_MAP ใน config ซึ่ง **ต้องวัดจริงใน C4**
  ถ้าในวิดีโอเห็นกล่องบนรถ แต่ตรงนี้บอกไม่เจอ -> แก้ IMAGE_AXIS_MAP ไม่ใช่แก้โมเดล
"""

import argparse
import bisect
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import config as C   # noqa: E402

MAX_POSE_GAP_S = 0.25      # เฟรมที่ไม่มี log ตำแหน่งภายในช่วงนี้ = ข้าม


class InputError(Exception):
    pass


# --------------------------------------------------------------------------- io
def read_jsonl(path):
    if not os.path.exists(path):
        raise InputError(f"ไม่พบไฟล์ {path}")
    out = []
    with open(path) as f:
        for i, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                # บรรทัดสุดท้ายอาจขาดตอนถ้า process ถูก kill — ข้ามได้
                print(f"[match] ข้ามบรรทัด {i} ของ {os.path.basename(path)} (JSON เสีย)")
    return out


# --------------------------------------------------------------------------- geometry
def world_to_pixel(truck_xy, drone_xyz, truck_z, yaw_deg, axis_map=None):
    """ฉายจุดศูนย์กลางรถ (โลก ENU) ลงภาพของกล้องที่ชี้ลงตรง

    offset ENU -> NED -> แกนตัวโดรน FRD (หมุนด้วย yaw ของ PX4: 0 = หันเหนือ ตามเข็ม)
    -> (u ขวา, v ลง) ด้วย IMAGE_AXIS_MAP
    คืน None ถ้าโดรนไม่ได้อยู่เหนือรถ
    """
    m = axis_map or C.IMAGE_AXIS_MAP
    h = drone_xyz[2] - truck_z
    if h <= 1e-6:
        return None
    de = truck_xy[0] - drone_xyz[0]
    dn = truck_xy[1] - drone_xyz[1]
    yaw = math.radians(yaw_deg)
    fwd = dn * math.cos(yaw) + de * math.sin(yaw)
    right = -dn * math.sin(yaw) + de * math.cos(yaw)
    k = C.px_per_unit(h)
    u = C.IMG_W / 2.0 + (m[0][0] * fwd + m[0][1] * right) * k
    v = C.IMG_H / 2.0 + (m[1][0] * fwd + m[1][1] * right) * k
    return u, v


def box_contains(box, u, v, margin):
    x1, y1, x2, y2 = box[:4]
    return x1 - margin <= u <= x2 + margin and y1 - margin <= v <= y2 + margin


def truck_in_view(truck, drone_xyz, ground_z, yaw_deg, tilt_deg, axis_map=None):
    """ภาพ (ขยายขอบเผื่อครึ่งคัน + การเอียงตัว) ทับรถคันนี้ไหม"""
    m = axis_map or C.IMAGE_AXIS_MAP
    h = drone_xyz[2] - ground_z
    if h <= 1e-6:
        return True                                          # ต่ำผิดปกติ ถือว่าไม่ว่าง (ไม่นับหลอน)
    hw, hh = C.footprint_half(h)
    pad = C.TRUCK_HALF_EXTENT + h * math.tan(math.radians(tilt_deg))
    de = truck["center"][0] - drone_xyz[0]
    dn = truck["center"][1] - drone_xyz[1]
    yaw = math.radians(yaw_deg)
    fwd = dn * math.cos(yaw) + de * math.sin(yaw)
    right = -dn * math.sin(yaw) + de * math.cos(yaw)
    iu = m[0][0] * fwd + m[0][1] * right                     # หน่วยฉาก ตามแกนกว้างของภาพ
    iv = m[1][0] * fwd + m[1][1] * right                     # หน่วยฉาก ตามแกนสูงของภาพ
    return abs(iu) <= hw + pad and abs(iv) <= hh + pad


# --------------------------------------------------------------------------- sim speed
def realtime_factor(flight, window_s=1.0, min_speed=1.0, min_samples=10):
    """โลกจำลองเดินเร็วแค่ไหนเทียบเวลาจริง

    = (ระยะที่โดรนขยับต่อ 1 วินาทีจริง) / (ความเร็วที่ PX4 รายงาน ซึ่งเป็นหน่วยต่อวินาทีในโลกจำลอง)
    1.0 = real-time · 0.2 = ช้ากว่าเวลาจริง 5 เท่า (PX4 lockstep + Isaac FPS ต่ำ)
    ใช้ช่วงห่าง ~window_s ไม่ใช่ tick ติดกัน เพราะ log 20 Hz แต่ตำแหน่งอัปเดตไม่ทุก tick
    คืน None ถ้าข้อมูลขณะเคลื่อนที่ไม่พอ
    """
    fl = sorted((r for r in flight if r.get("vel_enu") and r.get("pos_enu")), key=lambda r: r["t"])
    if len(fl) < 2:
        return None
    ts = [r["t"] for r in fl]
    ratios, i = [], 0
    while i < len(fl):
        j = bisect.bisect_left(ts, ts[i] + window_s)
        if j >= len(fl):
            break
        a, b = fl[i], fl[j]
        dt = b["t"] - a["t"]
        v = 0.5 * (math.sqrt(sum(x * x for x in a["vel_enu"])) + math.sqrt(sum(x * x for x in b["vel_enu"])))
        if dt > 0 and v >= min_speed:
            ratios.append((math.dist(a["pos_enu"], b["pos_enu"]) / dt) / v)
        i = j
    return statistics.median(ratios) if len(ratios) >= min_samples else None


# --------------------------------------------------------------------------- core
def required_for(test_names):
    """เกณฑ์ 4/5 ใช้กับรอบเต็ม 5 คัน · รอบ --only (C5) ต้องเจอครบทุกคันที่บิน"""
    return C.PASS_MIN_TRUCKS if len(test_names) == len(C.TEST_TRUCKS) else len(test_names)


def evaluate(flight, dets, trucks, test_names, axis_map=None, alt=None):
    alt = C.ALT_ABOVE_TRUCK if alt is None else alt
    if not flight:
        raise InputError("flight_log ว่าง")
    if not dets:
        raise InputError("detections ว่าง")
    flight = sorted(flight, key=lambda r: r["t"])
    dets = sorted(dets, key=lambda r: r["t"])
    f0, f1 = flight[0]["t"], flight[-1]["t"]
    d0, d1 = dets[0]["t"], dets[-1]["t"]
    if d1 < f0 or d0 > f1:
        raise InputError(
            f"เวลา 2 log ไม่ทับกันเลย (flight {f0:.0f}–{f1:.0f} · detections {d0:.0f}–{d1:.0f}) "
            f"— ส่ง --run-dir ผิดโฟลเดอร์ หรือไฟล์มาจากคนละรอบ")

    ft = [r["t"] for r in flight]
    by_name = {t["name"]: t for t in trucks}
    mean_truck_z = statistics.fmean(t["center"][2] for t in trucks)

    def pose_at(t):
        i = bisect.bisect_left(ft, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(flight)]
        j = min(cands, key=lambda k: abs(ft[k] - t))
        return flight[j] if abs(ft[j] - t) <= MAX_POSE_GAP_S else None

    per_truck = {n: dict(frames=0, hits=0, no_pose=0) for n in test_names}
    empty = dict(frames=0, frames_with_box=0, boxes=0)
    no_pose_total = 0

    for d in dets:
        pose = pose_at(d["t"])
        if pose is None:
            no_pose_total += 1
            continue
        boxes = d.get("boxes") or []
        drone = pose["pos_enu"]
        yaw = pose.get("yaw_deg", 0.0)
        tilt = pose.get("tilt_deg", 0.0)

        # 1) ลอยค้างเหนือรถเป้าหมาย (นับเฉพาะหลังถึงที่แล้ว)
        if pose["phase"] == "hover" and pose.get("settled") and pose.get("truck") in per_truck:
            name = pose["truck"]
            tr = by_name[name]
            st = per_truck[name]
            st["frames"] += 1
            px = world_to_pixel(tr["center"][:2], drone, tr["center"][2], yaw, axis_map)
            if px and any(box_contains(b, px[0], px[1], C.HIT_MARGIN_PX) for b in boxes):
                st["hits"] += 1
            continue

        # 2) บินเร็วที่ความสูงเดินทาง ภาพไม่ทับรถคันไหน -> กล่องที่ขึ้นคือหลอน
        if pose["phase"] != "transit":
            continue
        tgt = pose.get("target_enu")
        ground_z = (tgt[2] - alt) if tgt else mean_truck_z
        if abs((drone[2] - ground_z) - alt) > C.EMPTY_ALT_TOL or tilt > C.EMPTY_MAX_TILT_DEG:
            continue
        if any(truck_in_view(t, drone, ground_z, yaw, tilt, axis_map) for t in trucks):
            continue
        empty["frames"] += 1
        empty["boxes"] += len(boxes)
        empty["frames_with_box"] += 1 if boxes else 0

    detected = []
    for name, st in per_truck.items():
        st["ratio"] = (st["hits"] / st["frames"]) if st["frames"] else 0.0
        st["detected"] = st["frames"] > 0 and st["ratio"] >= C.HIT_FRAME_RATIO
        if st["detected"]:
            detected.append(name)

    dt = [b["t"] - a["t"] for a, b in zip(dets, dets[1:]) if b["t"] > a["t"]]
    infer = sorted(x["infer_ms"] for x in dets if x.get("infer_ms") is not None)

    def pct(xs, q):
        return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))] if xs else None

    return dict(
        passed=bool(test_names) and len(detected) >= required_for(test_names),
        detected=len(detected), required=required_for(test_names), of=len(test_names),
        per_truck=per_truck,
        hallucination=dict(
            empty_frames=empty["frames"], boxes=empty["boxes"],
            boxes_per_empty_frame=(empty["boxes"] / empty["frames"]) if empty["frames"] else None,
            pct_empty_frames_with_box=(100.0 * empty["frames_with_box"] / empty["frames"])
            if empty["frames"] else None),
        speed=dict(
            frames=len(dets),
            fps=(1.0 / statistics.median(dt)) if dt else None,
            infer_ms_p50=pct(infer, 0.5), infer_ms_p95=pct(infer, 0.95)),
        skipped_no_pose=no_pose_total,
        sim_speed=dict(realtime_factor=realtime_factor(flight)),
        axis_map=[list(r) for r in (axis_map or C.IMAGE_AXIS_MAP)],
    )


def print_report(rep):
    print(f"\n{'รถ':<11}{'เฟรมลอยค้าง':>12}{'hit':>6}{'สัดส่วน':>9}   ผล")
    for name, st in rep["per_truck"].items():
        mark = "✅ เจอ" if st["detected"] else ("⚠️ ไม่มีเฟรม" if st["frames"] == 0 else "❌ ไม่เจอ")
        print(f"{name:<11}{st['frames']:>12}{st['hits']:>6}{st['ratio']:>9.0%}   {mark}")
    verdict = "PASS" if rep["passed"] else "FAIL"
    print(f"\nเจอ {rep['detected']}/{rep['of']} (ต้อง ≥ {rep['required']}) -> {verdict}")
    h = rep["hallucination"]
    if h["empty_frames"]:
        print(f"กล่องหลอน: {h['boxes']} กล่อง บน {h['empty_frames']} เฟรมว่าง "
              f"({h['boxes_per_empty_frame']:.3f} กล่อง/เฟรม · "
              f"{h['pct_empty_frames_with_box']:.1f}% ของเฟรมว่างมีกล่อง)")
    else:
        print("กล่องหลอน: ไม่มีเฟรมว่างให้วัด (บินเร็วไม่พอ / เอียงเกิน / ภาพทับรถตลอด)")
    s = rep["speed"]
    fps = f"{s['fps']:.1f}" if s["fps"] else "-"
    print(f"ความเร็ว: {s['frames']} เฟรม · FPS {fps} · infer p50 {s['infer_ms_p50']} ms · p95 {s['infer_ms_p95']} ms")
    rtf = (rep.get("sim_speed") or {}).get("realtime_factor")
    if rtf is not None:
        note = "" if rtf >= 0.8 else (" — โลกจำลองช้ากว่าเวลาจริง (Isaac FPS ต่ำ) · "
                                      "demo.mp4 จัดจังหวะตามเวลาในโลกจำลองแล้ว เล่นด้วยความเร็วปกติ")
        print(f"ความเร็วการจำลอง ≈ {rtf:.2f}× เวลาจริง{note}")
    if rep["skipped_no_pose"]:
        print(f"ข้าม {rep['skipped_no_pose']} เฟรมที่ไม่มีตำแหน่งโดรนใกล้ ๆ (> {MAX_POSE_GAP_S}s)")


def main(argv=None):
    p = argparse.ArgumentParser(description="วัดผลหลังบินโดรน")
    p.add_argument("--run-dir", required=True, help="path หรือ LATEST")
    p.add_argument("--trucks", default=C.TRUCKS_JSON)
    args = p.parse_args(argv)
    try:
        run = C.resolve_run_dir(args.run_dir)
    except FileNotFoundError as e:
        print(f"[match] {e}")
        return 2
    print(f"[match] run dir: {run}")
    try:
        with open(os.path.expanduser(args.trucks)) as f:
            data = json.load(f)
        trucks = data["trucks"] if isinstance(data, dict) else data
        meta_path = os.path.join(run, "run_meta.json")
        meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
        test_names = [n for n in C.TEST_TRUCKS if n in set(meta.get("order") or C.TEST_TRUCKS)]
        rep = evaluate(read_jsonl(os.path.join(run, "flight_log.jsonl")),
                       read_jsonl(os.path.join(run, "detections.jsonl")),
                       trucks, test_names,
                       axis_map=meta.get("image_axis_map"), alt=meta.get("alt"))
    except InputError as e:
        print(f"[match] input ใช้ไม่ได้: {e}")
        return 2
    with open(os.path.join(run, "report.json"), "w") as f:
        json.dump(rep, f, ensure_ascii=False, indent=2)
    print_report(rep)
    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
