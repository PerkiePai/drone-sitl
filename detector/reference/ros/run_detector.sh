#!/usr/bin/env bash
# WHAT : launcher for ros_detector.py - selects the detection model (RF-DETR / YOLO)
# RUN  : terminal
# CMD  : bash run_detector.sh --model rfdetr|yolo [--weights PATH] [--conf 0.25] [--ocr] [--list-models] [--help]
# MODEL: default RF-DETR checkpoint_best_ema.pth  |  --model yolo -> Env2aug.pt
#
# No args == the old run_ros_yolo.sh behaviour (RF-DETR medium, /detection_camera/image_raw -> /yolo/annotated).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/.venv/bin/activate"

exec python3 "$SCRIPT_DIR/ros_detector.py" "$@"
