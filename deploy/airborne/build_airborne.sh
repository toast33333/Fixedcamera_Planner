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
catkin_make -DCMAKE_BUILD_TYPE=Release

echo "编译完成。下一步运行: ./verify_airborne.sh"
