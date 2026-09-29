# WHAT : run the fine-tuned RF-DETR over a video FILE offline, write an annotated mp4 + per-frame bbox log
# RUN  : terminal  (source /opt/ros/jazzy/setup.bash + .venv, or any env with rfdetr + cv2 + torch)
# CMD  : python3 infer_video_rfdetr.py <input_video> [output_video]
# MODEL: RF-DETR medium - DroneDetectionProject/runs_rfdetr/truck_medium_quality_aug/checkpoint_best_ema.pth
import sys
import time
from pathlib import Path

import cv2
import torch
import supervision as sv
from rfdetr import from_checkpoint

WEIGHTS = '/home/innovation/Tiger/DroneDetectionProject/runs_rfdetr/truck_medium_quality_aug/checkpoint_best_ema.pth'
CONF_THRESHOLD = 0.25


def main():
    if len(sys.argv) < 2:
        print('usage: infer_video_rfdetr.py <input_video> [output_video]')
        sys.exit(1)

    in_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else in_path.with_name(f'{in_path.stem}_rfdetr{in_path.suffix}')

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f'device = {device}')

    model = from_checkpoint(WEIGHTS, device=device)
    try:
        model.optimize_for_inference()
    except Exception as e:
        print(f'optimize_for_inference skipped: {e}')

    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    cap = cv2.VideoCapture(str(in_path))
    if not cap.isOpened():
        print(f'failed to open {in_path}')
        sys.exit(1)

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))

    n = 0
    frames_with_dets = 0
    total_dets = 0
    t0 = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        detections = model.predict(rgb, threshold=CONF_THRESHOLD)

        if len(detections) > 0:
            frames_with_dets += 1
            total_dets += len(detections)
            ts = n / fps
            for name, conf, (x1, y1, x2, y2) in zip(
                    detections.data['class_name'], detections.confidence, detections.xyxy):
                print(f'[t={ts:6.2f}s frame={n}] bbox: {name} conf={conf:.2f} '
                      f'xyxy=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})')

        labels = [f'{name} {conf:.2f}' for name, conf in
                  zip(detections.data['class_name'], detections.confidence)]
        annotated = box_annotator.annotate(scene=frame.copy(), detections=detections)
        annotated = label_annotator.annotate(scene=annotated, detections=detections, labels=labels)
        writer.write(annotated)

        if n % 30 == 0 or n == total_frames:
            print(f'progress: {n}/{total_frames} frames')

    cap.release()
    writer.release()
    dt = time.time() - t0
    print(f'done in {dt:.1f}s ({n / dt:.1f} fps) -> {out_path}')
    print(f'frames with detections: {frames_with_dets}/{n}, total detections: {total_dets}')


if __name__ == '__main__':
    main()
