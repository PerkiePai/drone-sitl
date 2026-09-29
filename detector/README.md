# Truck detector — package สำหรับ integrate เข้าระบบเว็บ

โมเดล **RF-DETR Medium** (1 คลาส: `Truck`) ที่ใช้ในระบบโดรน + โค้ด inference ที่ไม่พึ่ง ROS / Isaac Sim

## สิ่งที่อยู่ในนี้

| พาธ | คืออะไร |
|---|---|
| `model/rfdetr_truck_medium_ema.pth` | **ไฟล์โมเดล** (128 MB) — `checkpoint_best_ema.pth` จากรอบเทรน `truck_group_quality_aug` (แยกชุดเทสตามคัน รถ 5 คันที่โมเดลไม่เคยเห็น) · sha256 ใน `model/SHA256SUMS` |
| `model/model_card.json` | สเปก, คลาส, threshold แนะนำ, metric, **ข้อจำกัด** |
| `model/training_config.json`, `train_kwargs.json`, `metrics.csv` | ค่าที่ใช้เทรน + log metric (เผื่อเทรนซ้ำ/ตรวจย้อน) |
| `truck_detector.py` | **ไลบรารีหลัก** `TruckDetector` — เรียกจาก Python ตรง ๆ ได้ |
| `server/app.py` | ตัวอย่าง REST API (FastAPI): `/health`, `/detect`, `/detect/annotated` |
| `infer_video.py` | ตรวจจับทั้งไฟล์วิดีโอ → mp4 + `jsonl` (ไม่ใช้ ROS) |
| `samples/` | ภาพตัวอย่าง 2 ภาพจาก test split + `ground_truth.json` + ภาพผลลัพธ์ที่ควรได้ |
| `requirements.txt`, `requirements-web.txt` | dependency (ทดสอบกับ rfdetr 1.9.4) |
| `reference/ros/` | สคริปต์เดิมของระบบโดรน: `ros_detector.py`, `detection_recorder.py`, `run_detector.sh`, `infer_video_rfdetr.py` — **อ้างอิงเท่านั้น** (hardcode path `/home/innovation/...` และต้องมี ROS 2) |
| `reference/drone_demo/` | `match_runs.py`, `validate_trucks.py` — ตัวจับคู่ detection กับตำแหน่งรถจริง (ใช้เข้าใจรูปแบบ `detections.jsonl`) |
| `reference/eval_training/` | สคริปต์เทรน/ประเมินผล/แบ่ง split (`train_rfdetr_group.py`, `eval_common.py`, `run_eval.py`, `build_group_split.py`, `convert_coco_to_rfdetr.py`) + `requirements-venv_rfdetr-snapshot.txt` (pip freeze เต็มของ env เทรน) |

## ติดตั้ง + ลองรัน

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install torch torchvision          # เลือกตาม CUDA ของเครื่อง: https://pytorch.org/get-started/locally/
pip install -r requirements.txt
sha256sum -c model/SHA256SUMS          # เช็คไฟล์โมเดลไม่เสียหาย

python truck_detector.py samples/truck_0005.png
# -> 1 detection: Truck conf ~0.90, box ~ (552,419)-(818,632)   (เทียบ samples/ground_truth.json ได้)
```

ไม่มี GPU ก็รันได้ (`device="cpu"` อัตโนมัติ) แต่ช้ากว่ามาก

## ใช้จาก Python

```python
import cv2
from truck_detector import TruckDetector

det = TruckDetector()                       # โหลดโมเดลครั้งเดียวตอนเริ่มระบบ
img = cv2.imread("samples/truck_0005.png")  # BGR (ถ้าเป็น RGB ใส่ input_format="rgb")
dets = det.predict(img, conf=0.25)
# [{'x1':552.4,'y1':418.6,'x2':818.5,'y2':631.7,'confidence':0.9007,'class_name':'Truck'}]
cv2.imwrite("out.jpg", det.annotate(img, dets))
```

- พิกัดเป็น **พิกเซลของภาพต้นฉบับ** (ไม่ใช่ภาพที่ย่อ 640) เรียงจาก confidence มาก→น้อย
- ภาพขนาดใดก็ได้ (โมเดลย่อเองภายใน) ภาพที่ใช้เทรนคือ 1280×720
- instance เดียวห้ามเรียกพร้อมกันหลาย thread → ใช้ `with det.lock:` หรือ 1 process ต่อ 1 instance

## REST API

```bash
pip install -r requirements-web.txt
uvicorn server.app:app --host 0.0.0.0 --port 8000     # ใช้ 1 worker ต่อ GPU (แต่ละ worker โหลดโมเดลของตัวเอง)

curl -s localhost:8000/health
curl -s -F file=@samples/truck_0005.png "localhost:8000/detect?conf=0.25"
curl -s -F file=@samples/truck_0005.png localhost:8000/detect/annotated -o out.jpg
```

ตอบ `/detect`:
```json
{"image":{"width":1280,"height":720},"conf":0.25,"count":1,"infer_ms":117.6,
 "detections":[{"x1":552.4,"y1":418.6,"x2":818.5,"y2":631.7,"confidence":0.9007,"class_name":"Truck"}]}
```
ไฟล์ไม่ใช่รูป → 400 · ใหญ่เกิน `MAX_UPLOAD_MB` (default 20) → 413 · `conf` นอก 0–1 → 422
env var: `TRUCK_WEIGHTS`, `TRUCK_CONF`, `TRUCK_DEVICE`, `MAX_UPLOAD_MB`

**ก่อนขึ้น production:** ตัวอย่างนี้ไม่มี auth / rate-limit / HTTPS / คิวงาน — ต้องใส่เองตามระบบเว็บ (หรือให้ backend หลักเรียกภายในเครือข่ายเท่านั้น)

## ⚠️ ข้อจำกัดที่ต้องรู้ก่อนเอาไปใช้จริง

1. **เทรนจากภาพจำลองเท่านั้น** — ภาพ render จาก Isaac Sim ของรถบรรทุกชุดเดียว (gaussian splat) มุมมองแบบโดรน ยัง **ไม่เคยวัดกับภาพถ่ายจริง/รถชนิดอื่น** ผล mAP ด้านล่างใช้ตัดสินความแม่นบนภาพจริงไม่ได้
2. **False positive บนภาพที่ไม่มีรถ** — การศึกษา Phase-0 ในกลุ่มโมเดลนี้วัดได้ ~0.8–0.86 กล่อง/ภาพว่าง (ทุกเงื่อนไข) ตัวเลข AP ไม่เห็นเพราะทุกภาพเทสมีรถอยู่แล้ว → ควรปรับ `conf` ขึ้น (ทดลองที่ 0.5–0.7), กรองกล่องเล็กเกินไป, และทดสอบกับภาพว่างของระบบคุณเอง
3. **ตัวเลขบนชุดเทสที่โมเดลไม่เคยเห็น (รถ 5 คันที่กันไว้):** mAP50 = 0.9999, mAP50-95 = 0.765, mAP75 = 0.978 (ภาพสังเคราะห์)
4. **ความเร็ว:** งาน GPU จริง ~4 ms/ภาพ แต่ทดสอบวันนี้ end-to-end ได้ ~65–105 ms/ภาพ (RTX 5090) เพราะ CPU เครื่องกำลังถูกโปรเซสอื่นใช้หนัก (load ~8) — ตอนบินจริงเคยวัดได้ ~14.5 ms (p50) ตอนเครื่องว่าง · ควรวัดซ้ำบนเครื่องปลายทาง · การเรียกครั้งแรกหลังโหลดโมเดลช้ามาก (วัดได้ ~6 วินาที) จึงควร warm-up ตอนเริ่มระบบ (`server/app.py` ทำให้แล้ว)
5. `server/app.py` ใช้ `@app.on_event("startup")` ซึ่งใน FastAPI รุ่นใหม่ขึ้นเตือน deprecated (ยังทำงานปกติ) — เปลี่ยนเป็น lifespan ได้ถ้าต้องการ
6. โมเดลเป็น PyTorch (`.pth`) — **ยังไม่ได้แปลงเป็น ONNX/TensorRT** (env นี้ไม่มี `onnx`; ไม่ได้ติดตั้งเพิ่มเพื่อไม่กระทบเครื่องที่ใช้ร่วมกัน) ถ้าระบบเว็บต้องการ runtime แบบเบา แจ้งได้ครับ
7. ตัวโมเดลใช้ RF-DETR (Apache-2.0) ส่วน YOLO `Env2aug.pt` (ultralytics) **ไม่ได้รวมมา** — ultralytics เป็น AGPL-3.0 ซึ่งมีผลกับบริการเว็บ ถ้าต้องการใช้ต้องตรวจสอบสัญญาอนุญาตก่อน

## ทดสอบแล้ว (2026-09-29)

- `TruckDetector` บน `samples/*.png`: พบ 1 คัน/ภาพ, IoU กับ ground truth 0.82 และ 0.76, ภาพดำ → 0 กล่อง, ทางเข้า BGR/RGB ให้ผลเหมือนกัน
- API ด้วย FastAPI TestClient + โมเดลจริง: `/health` 200, `/detect` 200 (ผลตรงกับไลบรารี), `conf=0.99` → 0 กล่อง, `/detect/annotated` ได้ JPEG, ไฟล์เสีย → 400, conf=5 → 422
- `infer_video.py` กับคลิป 20 เฟรมที่ประกอบจากภาพตัวอย่าง: ได้ mp4 + jsonl ครบทุกเฟรม พบรถทุกเฟรม
- ยัง **ไม่ได้ทดสอบ**: ติดตั้งใน venv ใหม่ตาม `requirements.txt` (ทดสอบใน env เดิมของโปรเจกต์), การรัน uvicorn จริงข้ามเครื่อง, โหมด CPU-only, วิดีโอจริงจากโดรน
