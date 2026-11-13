#!/usr/bin/env bash
set -euo pipefail

WS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "未找到 ROS Noetic: /opt/ros/noetic/setup.bash" >&2
  exit 2
fi

# ROS setup hooks are not compatible with nounset.
set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
set -u

cd "$WS_DIR"
ROS_PYTHON_BIN=${ROS_PYTHON_EXECUTABLE:-/usr/bin/python3}
if [[ ! -x "$ROS_PYTHON_BIN" ]]; then
  echo "未找到可执行的 ROS Python: $ROS_PYTHON_BIN" >&2
  exit 2
fi
BUILD_JOBS=${CATKIN_BUILD_JOBS:-4}
BUILD_LOAD=${CATKIN_BUILD_LOAD:-4}
catkin_make -j"$BUILD_JOBS" -l"$BUILD_LOAD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE="$ROS_PYTHON_BIN"

echo "编译完成。下一步运行: ./verify_airborne.sh"
