# WHAT : ROS 2 node - truck detector on /detection_camera/image_raw, model-selectable (RF-DETR / YOLO), optional OCR
# RUN  : terminal  (source /opt/ros/jazzy/setup.bash + .venv)  - normally launched via run_detector.sh
# CMD  : bash run_detector.sh --model rfdetr|yolo [--weights PATH] [--conf 0.25] [--ocr] [--list-models] [--help]
#        drone demo: ... --in-topic /drone/down_cam/image_raw --qos-depth 1 --quiet-boxes --record-dir RUN_DIR
#        (--qos-depth/--quiet-boxes/--record-dir added 2026-09-11 for DRONE_FLIGHT_PLAN.md B2;
#         defaults keep the old behaviour byte-for-byte: depth 10, per-bbox log, no files written)
# MODEL: default RF-DETR = DroneDetectionProject/runs_rfdetr/truck_medium_quality_aug/checkpoint_best_ema.pth
#        default YOLO   = /home/innovation/Tiger/Env2aug.pt   (ultralytics, 1-class Truck)
#
# The RF-DETR path is a verbatim copy of the old ros_yolo.py logic (kept identical on purpose);
# ros_yolo.py / ros_yolo_ocr.py are archived under _archive/2026-09-11_rename-detectors-and-dataset-gen/.
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import supervision as sv
import torch, cv2

# ---------------------------------------------------------------------------
# MODEL REGISTRY - the single place that maps a model name to how it loads
# and to its default weights. Add a model here, nowhere else.
# ---------------------------------------------------------------------------
RFDETR_DEFAULT_WEIGHTS = '/home/innovation/Tiger/DroneDetectionProject/runs_rfdetr/truck_medium_quality_aug/checkpoint_best_ema.pth'
YOLO_DEFAULT_WEIGHTS = '/home/innovation/Tiger/Env2aug.pt'


def _load_rfdetr(weights, device, logger):
    from rfdetr import from_checkpoint                       # was: top-level import in ros_yolo.py
    model = from_checkpoint(weights, device=device)          # ros_yolo.py:20
    try:                                                     # ros_yolo.py:21
        model.optimize_for_inference()                       # ros_yolo.py:22
    except Exception as e:                                   # ros_yolo.py:23
        logger.warn(f'optimize_for_inference skipped: {e}')  # ros_yolo.py:24
    return model


def _load_yolo(weights, device, logger):
    from ultralytics import YOLO
    model = YOLO(weights)
    model.to(device)
    return model


MODELS = {
    'rfdetr': dict(label='RF-DETR medium (truck_medium_quality_aug)',
                   default_weights=RFDETR_DEFAULT_WEIGHTS,
                   loader=_load_rfdetr),
    'yolo': dict(label='YOLO ultralytics (1-class Truck)',
                 default_weights=YOLO_DEFAULT_WEIGHTS,
                 loader=_load_yolo),
}


def list_models():
    print(f'{"model":<8}  default_weights')
    for name, spec in MODELS.items():
        print(f'{name:<8}  {spec["default_weights"]}')


class DetectorNode(Node):
    def __init__(self, args):
        super().__init__('detector_node')
        self.bridge = CvBridge()
        self.device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')

        spec = MODELS[args.model]
        self.kind = args.model
        self.label = spec['label']
        self.weights = args.weights or spec['default_weights']
        self.conf = args.conf
        self.win_title = f'ros_detector | {self.label} | {os.path.basename(self.weights)} | {self.device}'

        # first log line + window title carry model + weights file + device on purpose
        self.get_logger().info(
            f'[ros_detector] model={self.label} | weights={self.weights} | device={self.device}')

        self.model = spec['loader'](self.weights, self.device, self.get_logger())

        self.box_annotator = sv.BoxAnnotator()               # ros_yolo.py:26
        self.label_annotator = sv.LabelAnnotator()           # ros_yolo.py:27

        # optional OCR (verbatim from ros_yolo_ocr.py) - built only with --ocr
        self.ocr = None
        if args.ocr:
            import easyocr                                   # ros_yolo_ocr.py:9 (lazy here)
            self.ocr = easyocr.Reader(['en'], gpu=(self.device == 'cuda'))   # ros_yolo_ocr.py:34
            self.MIN_CONF = 0.5                              # ros_yolo_ocr.py:35
            self.OCR_EVERY = 1                               # ros_yolo_ocr.py:36
            self.txt_pub = self.create_publisher(String, args.ocr_topic, 10)  # ros_yolo_ocr.py:44
            self.win_title = self.win_title.replace('ros_detector', 'ros_detector +OCR')

        # QoS identical to ros_yolo.py:30-32 (best-effort, so it accepts both
        # a reliable and a best-effort publisher on /detection_camera/image_raw)
        qos = QoSProfile(depth=args.qos_depth,              # was depth=10; drone demo uses 1 (latest frame only)
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            Image, args.in_topic, self.cb, qos)              # ros_yolo.py:33-34
        self.pub = self.create_publisher(Image, args.out_topic, 10)   # ros_yolo.py:35
        self.n = 0                                           # ros_yolo.py:36

        # optional per-frame record for drone_demo/match_runs.py - built only with --record-dir
        self.quiet_boxes = args.quiet_boxes
        self.recorder = None
        if args.record_dir:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from detection_recorder import DetectionRecorder
            self.recorder = DetectionRecorder(args.record_dir, fps=args.record_fps)
            self.get_logger().info(
                f'[ros_detector] recording -> {self.recorder.jsonl_path} + {self.recorder.video_path}')

    # -- RF-DETR inference: byte-for-byte ros_yolo.py:40-46 --------------------
    def _infer_rfdetr(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)                              # ros_yolo.py:40
        detections = self.model.predict(rgb, threshold=self.conf)                # ros_yolo.py:41 (CONF_THRESHOLD -> self.conf)
        labels = [f'{name} {conf:.2f}' for name, conf in                         # ros_yolo.py:43
                  zip(detections.data['class_name'], detections.confidence)]     # ros_yolo.py:44
        annotated = self.box_annotator.annotate(scene=frame.copy(), detections=detections)          # ros_yolo.py:45
        annotated = self.label_annotator.annotate(scene=annotated, detections=detections, labels=labels)  # ros_yolo.py:46
        triples = list(zip(detections.data['class_name'], detections.confidence, detections.xyxy))   # ros_yolo.py:49-50
        return annotated, triples, len(detections)

    # -- YOLO inference: ultralytics, drawn with results.plot() (matches detection.py / old yolo_isaac.py)
    def _infer_yolo(self, frame):
        r = self.model(frame, conf=self.conf, verbose=False)[0]
        annotated = r.plot()
        triples = []
        for b in r.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            triples.append((self.model.names[int(b.cls[0])], float(b.conf[0]), (x1, y1, x2, y2)))
        return annotated, triples, len(r.boxes)

    def cb(self, msg):
        t_wall = time.time()      # same wall clock as fly_route.py -> match_runs pairs frames with drone pose
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')          # ros_yolo.py:39

        t_inf = time.perf_counter()
        if self.kind == 'rfdetr':
            annotated, triples, n_det = self._infer_rfdetr(frame)
        else:
            annotated, triples, n_det = self._infer_yolo(frame)
        infer_ms = (time.perf_counter() - t_inf) * 1000.0

        self.n += 1                                                             # ros_yolo.py:48
        if not self.quiet_boxes:
            for name, conf, (x1, y1, x2, y2) in triples:                       # ros_yolo.py:49-50
                self.get_logger().info(                                        # ros_yolo.py:51
                    f'bbox: {name} conf={conf:.2f} xyxy=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})')  # ros_yolo.py:52
        if self.n % 30 == 0:                                                   # ros_yolo.py:53
            self.get_logger().info(f'frame {self.n}: {n_det} detections')      # ros_yolo.py:54

        # OCR block: verbatim from ros_yolo_ocr.py:64-82, runs only with --ocr
        if self.ocr is not None:
            import numpy as np
            found = []
            if self.n % self.OCR_EVERY == 0:
                for poly, t, s in self.ocr.readtext(frame):
                    if s < self.MIN_CONF or not t.strip():
                        continue
                    found.append(t.strip())
                    pts = np.array(poly, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(annotated, [pts], True, (0, 255, 0), 2)
                    x, y = pts[0][0]
                    cv2.putText(annotated, t.strip(), (int(x), int(y) - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if found:
                self.txt_pub.publish(String(data=' | '.join(found)))
            if self.n % 30 == 0:
                self.get_logger().info(f'frame {self.n}: ocr={found}')

        if self.recorder is not None:
            self.recorder.write(t_wall, msg.header.stamp.sec, msg.header.stamp.nanosec,
                                infer_ms, triples, annotated)

        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')             # ros_yolo.py:56
        out.header = msg.header                                                # ros_yolo.py:57
        self.pub.publish(out)                                                  # ros_yolo.py:58
        cv2.imshow(self.win_title, annotated)                                  # ros_yolo.py:59 ('RF-DETR live' -> dynamic title)
        cv2.waitKey(1)                                                         # ros_yolo.py:60


def build_arg_parser():
    p = argparse.ArgumentParser(
        prog='ros_detector.py',
        description='ROS 2 truck detector on /detection_camera/image_raw. '
                    'Default = RF-DETR medium (identical to the old run_ros_yolo.sh).')
    p.add_argument('--model', choices=sorted(MODELS), default='rfdetr',
                   help='detection model (default: rfdetr)')
    p.add_argument('--weights', default=None,
                   help='override weights path (default: per-model default, see --list-models)')
    p.add_argument('--conf', type=float, default=0.25, help='confidence threshold (default: 0.25)')
    p.add_argument('--device', default=None, help='cuda | cpu (default: cuda if available)')
    p.add_argument('--ocr', action='store_true', help='also run EasyOCR and publish text on --ocr-topic')
    p.add_argument('--in-topic', default='/detection_camera/image_raw', help='input Image topic')
    p.add_argument('--out-topic', default='/yolo/annotated', help='annotated Image topic (default kept for back-compat)')
    p.add_argument('--ocr-topic', default='/yolo/ocr_text', help='OCR text topic (with --ocr)')
    p.add_argument('--qos-depth', type=int, default=10,
                   help='subscriber queue depth (default 10 = old behaviour; 1 = always newest frame, no lag build-up)')
    p.add_argument('--quiet-boxes', action='store_true', help='do not log every bbox every frame (keep the 30-frame summary)')
    p.add_argument('--record-dir', default=None,
                   help='write detections.jsonl (wall-clock t per frame) + demo.mp4 into this folder')
    p.add_argument('--record-fps', type=float, default=15.0, help='nominal fps of demo.mp4 (default 15)')
    p.add_argument('--list-models', action='store_true', help='print the model registry and exit')
    return p


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.list_models:
        list_models()
        return
    rclpy.init()                                             # ros_yolo.py:63
    node = DetectorNode(args)
    try:
        rclpy.spin(node)                                     # ros_yolo.py:65
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass                                                 # clean Ctrl-C / SIGTERM teardown
    if node.recorder is not None:
        node.recorder.close()                                # finalize mp4, otherwise the file is unplayable
        node.get_logger().info(f'[ros_detector] recorded {node.recorder.frames} frames')
    node.destroy_node()                                      # ros_yolo.py:66
    cv2.destroyAllWindows()                                  # ros_yolo.py:67
    try:
        rclpy.shutdown()                                     # ros_yolo.py:68
    except Exception:
        pass


if __name__ == '__main__':                                   # ros_yolo.py:70
    main()                                                   # ros_yolo.py:71
