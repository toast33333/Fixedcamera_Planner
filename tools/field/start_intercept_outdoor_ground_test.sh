#!/usr/bin/env bash
# Outdoor, props-off, disarmed full-chain test. This uses the real GPS/EKF,
# IMU, camera, servo and final MAVROS command topic; the flight controller must
# remain disarmed throughout the test.
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
DEFAULT_WS_DIR=$(cd "$REPO_DIR/../.." && pwd)
WS_DIR=${WS_DIR:-$DEFAULT_WS_DIR}
MAVROS_STATE_TOPIC=${MAVROS_STATE_TOPIC:-/mavros/state}
SERVO_COMMAND_TOPIC=${SERVO_COMMAND_TOPIC:-/camera_pitch_servo/command_deg}
FINAL_COMMAND_TOPIC=${FINAL_COMMAND_TOPIC:-/mavros/setpoint_velocity/cmd_vel}
DETECTION_TOPIC=${DETECTION_TOPIC:-/object_detections}
PREFLIGHT_TIMEOUT_S=${PREFLIGHT_TIMEOUT_S:-10}

if [[ ! -f /opt/ros/noetic/setup.bash || ! -f "$WS_DIR/devel/setup.bash" ]]; then
  echo "未找到 ROS 或工作空间环境。" >&2
  exit 3
fi
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
set -u

if ! rosparam get /rosdistro >/dev/null 2>&1; then
  echo "ROS master 未运行。" >&2
  exit 4
fi

state_message=$(timeout "${PREFLIGHT_TIMEOUT_S}s" \
  rostopic echo -n 1 "$MAVROS_STATE_TOPIC" 2>/dev/null || true)
if ! grep -Eq 'connected: *(True|true|1)' <<<"$state_message"; then
  echo "MAVROS 未连接，无法进行真实链路地面测试。" >&2
  exit 4
fi
if grep -Eq 'armed: *(True|true|1)' <<<"$state_message"; then
  echo "拒绝启动：地面测试要求飞控保持上锁。" >&2
  exit 4
fi
if grep -Eq 'mode: *"?GUIDED"?' <<<"$state_message"; then
  echo "拒绝启动：请先退出 GUIDED，再进行上锁地面测试。" >&2
  exit 4
fi

CONTROL_PID=""
MONITOR_PIDS=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${MONITOR_PIDS[@]:-}"; do
    [[ -n "$pid" ]] || continue
    pkill -TERM -P "$pid" >/dev/null 2>&1 || true
    kill -TERM "$pid" >/dev/null 2>&1 || true
    wait "$pid" >/dev/null 2>&1 || true
  done
  if [[ -n "$CONTROL_PID" ]] && kill -0 "$CONTROL_PID" >/dev/null 2>&1; then
    kill -INT "$CONTROL_PID" >/dev/null 2>&1 || true
    wait "$CONTROL_PID" >/dev/null 2>&1 || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

stream_topic() {
  local label=$1
  local topic=$2
  rostopic echo "$topic" 2>&1 | while IFS= read -r line; do
    printf '[%s] %s\n' "$label" "$line"
  done
}

echo "============================================================"
echo "室外无桨上锁地面测试：真实完整视觉拦截链路"
echo "真实输入: GPS/EKF + IMU + 相机检测"
echo "真实输出: $FINAL_COMMAND_TOPIC（飞控上锁，不会驱动电机）"
echo "舵机: 完整模式，执行两级安全解锁并实际输出 PWM"
echo "窗口将持续打印舵机期望角、最终飞行指令和视觉检测。"
echo "飞控必须始终保持上锁、非 GUIDED；按 Ctrl-C 结束。"
echo "============================================================"

ENABLE_CAMERA_PITCH_SERVO=true \
SERVO_HARDWARE_AUTO_ARM=1 \
DETECTION_TOPIC="$DETECTION_TOPIC" \
"$SCRIPT_DIR/start_visual_intercept.sh" full &
CONTROL_PID=$!

stream_topic "SERVO_DEG" "$SERVO_COMMAND_TOPIC" &
MONITOR_PIDS+=("$!")
stream_topic "FINAL_CMD" "$FINAL_COMMAND_TOPIC" &
MONITOR_PIDS+=("$!")
stream_topic "DETECTION" "$DETECTION_TOPIC" &
MONITOR_PIDS+=("$!")

set +e
wait "$CONTROL_PID"
control_status=$?
set -e
CONTROL_PID=""
exit "$control_status"
