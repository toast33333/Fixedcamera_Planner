#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
DEFAULT_WS_DIR=$(cd "$REPO_DIR/../.." && pwd)
WS_DIR=${WS_DIR:-$DEFAULT_WS_DIR}
PREFLIGHT_TIMEOUT_S=${PREFLIGHT_TIMEOUT_S:-10}
SKIP_PREFLIGHT_CHECKS=${SKIP_PREFLIGHT_CHECKS:-0}

if [[ ! -f /opt/ros/noetic/setup.bash || ! -f "$WS_DIR/devel/setup.bash" ]]; then
  echo "未找到ROS或工作空间环境。" >&2
  exit 3
fi
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
set -u

# A circle test owns the MAVROS velocity-command topic by itself. Refuse to
# coexist with any visual/radar interception controller even when an operator
# explicitly skips the remaining preflight checks.
for node in /fixed_camera_ibvs /radar_approach_handover /circle_trajectory_test; do
  if rosnode list 2>/dev/null | grep -Fxq "$node"; then
    echo "检测到冲突或重复控制节点: $node" >&2
    exit 4
  fi
done

if [[ "$SKIP_PREFLIGHT_CHECKS" != "1" ]]; then
  state_message=$(timeout "${PREFLIGHT_TIMEOUT_S}s" \
    rostopic echo -n 1 /mavros/state 2>/dev/null || true)
  if ! grep -Eq 'connected: *(True|true|1)' <<<"$state_message"; then
    echo "MAVROS未连接。" >&2
    exit 4
  fi
  if ! grep -Eq 'armed: *(True|true|1)' <<<"$state_message"; then
    echo "飞机尚未解锁。" >&2
    exit 4
  fi
  if ! grep -Eq 'mode: *"?GUIDED"?' <<<"$state_message"; then
    echo "圆圈测试要求飞手先确认安全并切入GUIDED。" >&2
    exit 4
  fi
  extended_message=$(timeout "${PREFLIGHT_TIMEOUT_S}s" \
    rostopic echo -n 1 /mavros/extended_state 2>/dev/null || true)
  if ! grep -Eq 'landed_state: *2' <<<"$extended_message"; then
    echo "MAVROS未确认飞机处于空中，拒绝启动圆圈测试。" >&2
    exit 4
  fi
  timeout "${PREFLIGHT_TIMEOUT_S}s" rostopic echo -n 1 \
    /mavros/local_position/pose >/dev/null 2>&1 || {
      echo "没有本地位置数据。" >&2
      exit 4
    }
fi

exec roslaunch png_planner circle_trajectory_test.launch \
  minimum_speed:="${MINIMUM_SPEED:-5.0}" \
  maximum_speed:="${MAXIMUM_SPEED:-15.0}" \
  speed_step:="${SPEED_STEP:-2.0}" \
  start_radius:="${START_RADIUS:-20.0}" \
  radial_rate:="${RADIAL_RATE:-0.5}" \
  trajectory_error_limit:="${TRAJECTORY_ERROR_LIMIT:-3.0}" \
  geofence_radius:="${GEOFENCE_RADIUS:-50.0}" \
  geofence_margin:="${GEOFENCE_MARGIN:-5.0}" \
  csv_path:="${CSV_PATH:-/root/.ros/circle_trajectory_test.csv}"
