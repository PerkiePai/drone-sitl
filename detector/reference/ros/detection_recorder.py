# WHAT : บันทึกผลตรวจจับทุกเฟรมเป็น detections.jsonl + วิดีโอ demo.mp4 (ใช้กับ ros_detector.py --record-dir)
# RUN  : ถูก import โดย ros_detector.py เท่านั้น · ไม่มี ROS ในไฟล์นี้ -> ทดสอบบน Mac ได้
# WHY  : match_runs.py (drone_demo) ต้องจับคู่เวลาเฟรมกับตำแหน่งโดรน -> "t" ใน jsonl ใช้ time.time() ของเครื่องเดียวกัน
#        ไม่ใช้ header.stamp จับคู่ เพราะภาพจาก Isaac ประทับเวลาในโลกจำลอง (DRONE_FLIGHT_PLAN.md §0 ข้อ 2)
#
#        แต่ "วิดีโอ" จัดจังหวะตาม header.stamp (2026-09-13): ไฟล์ mp4 มี fps คงที่ ถ้าเขียนทุกเฟรมที่เข้ามา
#        คลิปจะเร็ว/ช้าตามความเร็วที่ภาพเข้ามา (ภาพเข้า ~10 fps แต่ไฟล์ 15 fps = เล่นเร็วกว่าจริง 1.5 เท่า)
#        -> เติมเฟรมซ้ำ/ข้ามเฟรมให้ตรงเวลา และใช้เวลาในโลกจำลอง เพื่อให้คลิปเล่นความเร็วปกติแม้ Isaac FPS ต่ำ
#
#        ปิดคลิปเองเมื่อบินจบ (2026-09-13): fly_route.py เขียน <run_dir>/flight_done.json ตอนจบเที่ยวบิน
#        -> อีก tail_s วินาทีต่อมา recorder ปิดไฟล์ให้เอง แม้ detector จะยังรันค้าง คลิปก็ใช้ได้ทันที
import json
import os
import time

FLIGHT_DONE_NAME = "flight_done.json"   # ต้องตรงกับ fly_route.py


def to_record(t_wall, stamp_sec, stamp_nanosec, infer_ms, triples):
    """หนึ่งบรรทัดของ detections.jsonl

    triples = [(class_name, conf, (x1, y1, x2, y2)), ...] ตามรูปแบบใน ros_detector.py
    boxes   = [[x1, y1, x2, y2, conf, class_name], ...]
    """
    boxes = []
    for name, conf, xyxy in triples:
        x1, y1, x2, y2 = (round(float(v), 1) for v in xyxy)
        boxes.append([x1, y1, x2, y2, round(float(conf), 4), str(name)])
    return {
        "t": float(t_wall),
        "stamp": [int(stamp_sec), int(stamp_nanosec)],
        "infer_ms": None if infer_ms is None else round(float(infer_ms), 2),
        "n": len(boxes),
        "boxes": boxes,
    }


def _cv2_writer(path, fps, size):
    import cv2
    return cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)


class DetectionRecorder:
    """เปิดไฟล์ครั้งเดียว เขียนทีละเฟรม ปิดตอนจบ

    pacing = "auto"  : ใช้ header.stamp ถ้าเฟรมแรกมี stamp > 0 ไม่งั้นใช้เวลาเครื่อง (ล็อกตั้งแต่เฟรมแรก)
             "stamp" : เวลาในโลกจำลองเสมอ · "wall" : เวลาเครื่องเสมอ
    max_gap_s        : ช่องว่างยาวกว่านี้ (เช่นกด Pause) ไม่เติมภาพค้างเกินนี้
    """

    def __init__(self, run_dir, fps=15.0, video=True, pacing="auto", max_gap_s=2.0, writer_factory=None,
                 finish_marker=FLIGHT_DONE_NAME, tail_s=3.0):
        self.run_dir = os.path.expanduser(run_dir)
        os.makedirs(self.run_dir, exist_ok=True)
        self.jsonl_path = os.path.join(self.run_dir, "detections.jsonl")
        self.video_path = os.path.join(self.run_dir, "demo.mp4")
        self._f = open(self.jsonl_path, "a", buffering=1)
        self._fps = float(fps)
        self._video = video
        self._pacing = pacing
        self._max_dup = max(1, int(round(max_gap_s * self._fps)))
        self._writer_factory = writer_factory or _cv2_writer
        self._writer = None
        self._clock = None            # "stamp" | "wall" — ตัดสินที่เฟรมแรกของวิดีโอ
        self._next_t = None
        self._last_t = None
        self.frames = 0               # จำนวนเฟรมที่ได้รับ (= บรรทัดใน jsonl)
        self.video_frames = 0         # จำนวนเฟรมในไฟล์ mp4 (หลังเติม/ข้าม)
        self.finished = False         # ปิดไฟล์แล้วเพราะบินจบ -> write() ต่อจากนี้ไม่ทำอะไร
        self._marker = os.path.join(self.run_dir, finish_marker) if finish_marker else None
        self._tail_s = float(tail_s)
        self._started = time.time()   # ไม่สนใจ flight_done.json ที่เก่ากว่าตอนเริ่มอัด (ของรอบก่อนในโฟลเดอร์เดิม)
        self._next_check = 0.0

    @property
    def clock(self):
        return self._clock

    def _video_time(self, t_wall, stamp_sec, stamp_nanosec):
        stamp = float(stamp_sec) + float(stamp_nanosec) * 1e-9
        if self._clock is None:
            if self._pacing == "wall":
                self._clock = "wall"
            elif self._pacing == "stamp":
                self._clock = "stamp"
            else:
                self._clock = "stamp" if stamp > 0.0 else "wall"
        if self._clock == "stamp":
            return stamp if stamp > 0.0 else None     # stamp หายกลางทาง -> ไม่เปิดช่องเวลาใหม่
        return float(t_wall)

    def _write_video(self, t, frame):
        dt = 1.0 / self._fps
        if self._writer is None:
            h, w = frame.shape[:2]
            self._writer = self._writer_factory(self.video_path, self._fps, (w, h))
            self._next_t = t
        elif self._last_t is not None and t < self._last_t - dt:
            self._next_t = t                          # นาฬิกาถอยหลัง (Stop แล้ว Play ใหม่) -> นับต่อจากตรงนี้
        n = 0
        while self._next_t <= t + 1e-9 and n < self._max_dup:
            self._writer.write(frame)                 # เฟรมเข้าช้ากว่า fps -> เขียนซ้ำให้เต็มช่องเวลา
            self._next_t += dt
            n += 1
        if n == self._max_dup and self._next_t <= t:
            self._next_t = t + dt                     # ช่องว่างยาว -> ไม่เติมภาพค้างยาวเกิน max_gap_s
        self._last_t = t
        self.video_frames += n                        # n = 0 = เฟรมเข้าเร็วกว่า fps -> ข้าม

    def _flight_finished(self, t_wall):
        """มี flight_done.json ของรอบนี้ และผ่านมาแล้วอย่างน้อย tail_s วินาที (ตรวจไม่เกินวินาทีละครั้ง)"""
        if not self._marker or t_wall < self._next_check:
            return False
        self._next_check = t_wall + 1.0
        try:
            with open(self._marker) as f:
                done_t = float(json.load(f).get("t", 0.0))
        except (OSError, ValueError, TypeError, AttributeError):
            return False
        return done_t >= self._started - 1.0 and t_wall - done_t >= self._tail_s

    def write(self, t_wall, stamp_sec, stamp_nanosec, infer_ms, triples, annotated=None):
        if self.finished:
            return
        self._f.write(json.dumps(
            to_record(t_wall, stamp_sec, stamp_nanosec, infer_ms, triples), ensure_ascii=False) + "\n")
        if self._video and annotated is not None:
            t = self._video_time(t_wall, stamp_sec, stamp_nanosec)
            if t is not None:
                self._write_video(t, annotated)
        self.frames += 1
        if self._flight_finished(t_wall):
            self.close()
            self.finished = True
            print(f"[recorder] บินจบแล้ว -> ปิดคลิปเรียบร้อย: {self.video_path} "
                  f"({self.video_frames} เฟรม) · detector ยังรันอยู่ได้ กด Ctrl+C หยุดเมื่อไรก็ได้", flush=True)

    def close(self):
        # ต้องปิดให้ครบ ไม่งั้น mp4 เปิดไม่ได้
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        if not self._f.closed:
            self._f.close()
