#!/usr/bin/env bash
set -euo pipefail

WS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
  echo "尚未编译，请先运行 ./build_airborne.sh" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1090
source "$WS_DIR/devel/setup.bash"
set -u

MODEL_PATH=$(rospack find detection_msgs)/rknnModel/yolov8-NewDrone.rknn
test -s "$MODEL_PATH"
rosmsg show detection_msgs/Detection | grep -q '^std_msgs/Header header'
rospack find png_planner >/dev/null
test -x "$WS_DIR/devel/lib/png_planner/fixed_camera_ibvs"
test -x "$WS_DIR/devel/lib/png_planner/radar_approach_handover"

echo "机载工作区静态检查通过。"
echo "RKNN 模型: $MODEL_PATH"
