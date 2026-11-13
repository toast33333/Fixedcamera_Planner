#!/usr/bin/env bash
# Pilot-flown field test: real visual detection and camera-pitch servo only.
# Aircraft interception guidance is disabled and no MAVROS velocity command is
# advertised by the visual-to-servo tracker.
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
DEFAULT_WS_DIR=$(cd "$REPO_DIR/../.." && pwd)
WS_DIR=${WS_DIR:-$DEFAULT_WS_DIR}
DETECTION_TOPIC=${DETECTION_TOPIC:-/object_detections}
SERVO_COMMAND_TOPIC=${SERVO_COMMAND_TOPIC:-/camera_pitch_servo/command_deg}
MAVROS_STATE_TOPIC=${MAVROS_STATE_TOPIC:-/mavros/state}
IMU_TOPIC=${IMU_TOPIC:-/mavros/imu/data}
POSE_TOPIC=${POSE_TOPIC:-/mavros/local_position/pose}
VISUAL_NODE=${VISUAL_NODE:-/field_yolo_detector}
VISUAL_LAUNCH_PACKAGE=${VISUAL_LAUNCH_PACKAGE:-detection_msgs}
VISUAL_LAUNCH_FILE=${VISUAL_LAUNCH_FILE:-field_detector.launch}
PREFLIGHT_TIMEOUT_S=${PREFLIGHT_TIMEOUT_S:-10}
VISUAL_START_TIMEOUT_S=${VISUAL_START_TIMEOUT_S:-45}
SERVO_START_TIMEOUT_S=${SERVO_START_TIMEOUT_S:-10}
SERVO_MAPPER_ENABLE_SERVICE=${SERVO_MAPPER_ENABLE_SERVICE:-/camera_pitch_servo/set_output_enabled}
SERVO_HARDWARE_ENABLE_SERVICE=${SERVO_HARDWARE_ENABLE_SERVICE:-/camera_pitch_servo/set_hardware_output_enabled}
SERVO_HARDWARE_ACTIVE_TOPIC=${SERVO_HARDWARE_ACTIVE_TOPIC:-/camera_pitch_servo/hardware_active}

if [[ ! -f /opt/ros/noetic/setup.bash || ! -f "$WS_DIR/devel/setup.bash" ]]; then
  echo "未找到 ROS 或工作空间环境。" >&2
  exit 3
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "$WS_DIR/devel/setup.bash"
set -u

if ! rosparam get /rosdistro >/dev/null 2>&1; then
  echo "ROS master 未运行。" >&2
  exit 4
fi

RUN_STAMP=$(date +%Y%m%d_%H%M%S)
ROS_LOG_DIR=${ROS_LOG_DIR:-$WS_DIR/log/visual_servo_manual_flight/${RUN_STAMP}}
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

CONTROL_PID=""
VISUAL_PID=""
MONITOR_PIDS=()

read_one_topic() {
  local topic=$1
  timeout "${PREFLIGHT_TIMEOUT_S}s" rostopic echo -n 1 "$topic" 2>/dev/null || true
}

call_servo_gate() {
  local service_name=$1
  local requested_value=$2
  local response
  response=$(rosservice call "$service_name" "data: ${requested_value}" 2>&1) || {
    echo "调用舵机安全服务失败: $service_name" >&2
    echo "$response" >&2
    return 1
  }
  if ! grep -Eq 'success: *(True|true|1)' <<<"$response"; then
    echo "舵机安全服务拒绝请求: $service_name" >&2
    echo "$response" >&2
    return 1
  fi
}

wait_for_node() {
  local node=$1
  local timeout_s=$2
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if rosnode list 2>/dev/null | grep -Fxq "$node"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

wait_for_servo_services() {
  local deadline=$((SECONDS + SERVO_START_TIMEOUT_S))
  local services
  while (( SECONDS < deadline )); do
    services=$(rosservice list 2>/dev/null || true)
    if grep -Fxq "$SERVO_MAPPER_ENABLE_SERVICE" <<<"$services" &&
       grep -Fxq "$SERVO_HARDWARE_ENABLE_SERVICE" <<<"$services"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

wait_for_servo_hardware_active() {
  local deadline=$((SECONDS + SERVO_START_TIMEOUT_S))
  local state
  while (( SECONDS < deadline )); do
    state=$(timeout 1 rostopic echo -n 1 "$SERVO_HARDWARE_ACTIVE_TOPIC" 2>/dev/null || true)
    if grep -Eq 'data: *(True|true|1)' <<<"$state"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

stop_process_tree() {
  local pid=${1:-}
  [[ -n "$pid" ]] || return 0
  pkill -INT -P "$pid" >/dev/null 2>&1 || true
  kill -INT "$pid" >/dev/null 2>&1 || true
  wait "$pid" >/dev/null 2>&1 || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  # Release physical PWM before stopping the ROS nodes.
  call_servo_gate "$SERVO_HARDWARE_ENABLE_SERVICE" false >/dev/null 2>&1 || true
  call_servo_gate "$SERVO_MAPPER_ENABLE_SERVICE" false >/dev/null 2>&1 || true

  local pid
  for pid in "${MONITOR_PIDS[@]:-}"; do
    stop_process_tree "$pid"
  done
  stop_process_tree "$CONTROL_PID"
  stop_process_tree "$VISUAL_PID"
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

existing_nodes=$(rosnode list 2>/dev/null || true)
if grep -Eq '^/(fixed_camera_ibvs|camera_pitch_visual_tracker)$' <<<"$existing_nodes"; then
  echo "检测到已有视觉控制/舵机跟踪节点，请先停止旧测试。" >&2
  exit 4
fi
if grep -Eq '^/camera_pitch_servo_(open_loop|sysfs_pwm)$' <<<"$existing_nodes"; then
  echo "检测到已有舵机节点，请先停止旧舵机测试，避免重复占用 PWM。" >&2
  exit 4
fi

state_message=$(read_one_topic "$MAVROS_STATE_TOPIC")
if ! grep -Eq 'connected: *(True|true|1)' <<<"$state_message"; then
  echo "MAVROS 未连接，无法取得飞行中的真实姿态。" >&2
  exit 4
fi
if [[ -z "$(read_one_topic "$IMU_TOPIC")" ]]; then
  echo "没有收到 IMU：$IMU_TOPIC。" >&2
  exit 4
fi
if [[ -z "$(read_one_topic "$POSE_TOPIC")" ]]; then
  echo "没有收到本地位姿：$POSE_TOPIC；请先等待 GPS/EKF 建立位置解。" >&2
  exit 4
fi

if rosnode list 2>/dev/null | grep -Fxq "$VISUAL_NODE"; then
  echo "视觉节点已运行，直接复用: $VISUAL_NODE"
else
  echo "正在启动视觉节点: ${VISUAL_LAUNCH_PACKAGE}/${VISUAL_LAUNCH_FILE}"
  roslaunch "$VISUAL_LAUNCH_PACKAGE" "$VISUAL_LAUNCH_FILE" \
    >"$ROS_LOG_DIR/visual_detector.log" 2>&1 &
  VISUAL_PID=$!
  if ! wait_for_node "$VISUAL_NODE" "$VISUAL_START_TIMEOUT_S"; then
    echo "视觉节点启动失败，日志: $ROS_LOG_DIR/visual_detector.log" >&2
    tail -n 40 "$ROS_LOG_DIR/visual_detector.log" >&2 || true
    exit 4
  fi
fi

if [[ -z "$(read_one_topic "$DETECTION_TOPIC")" ]]; then
  echo "视觉节点没有发布 $DETECTION_TOPIC。" >&2
  exit 4
fi

echo "============================================================"
echo "手飞视觉 + 舵机测试"
echo "飞行方式: 完全由飞手控制"
echo "视觉: 启动/复用 $VISUAL_NODE，并实时打印 $DETECTION_TOPIC"
echo "舵机: 启动视觉跟踪、姿态补偿、角度映射和 PWM 输出"
echo "拦截: 关闭；不计算、不发布 MAVROS 飞行控制指令"
echo "低置信度保护: score <= 0.5 时保持上一舵机角度，不跟随该检测框"
echo "按 Ctrl-C 结束；脚本会先关闭舵机 PWM。"
echo "日志目录: $ROS_LOG_DIR"
echo "============================================================"

roslaunch png_planner camera_pitch_visual_servo_test.launch \
  detection_topic:="$DETECTION_TOPIC" \
  pose_topic:="$POSE_TOPIC" \
  attitude_topic:="$IMU_TOPIC" \
  >"$ROS_LOG_DIR/visual_servo_tracker.log" 2>&1 &
CONTROL_PID=$!

if ! wait_for_node /camera_pitch_visual_tracker "$SERVO_START_TIMEOUT_S"; then
  echo "视觉舵机跟踪节点未启动，日志: $ROS_LOG_DIR/visual_servo_tracker.log" >&2
  tail -n 60 "$ROS_LOG_DIR/visual_servo_tracker.log" >&2 || true
  exit 4
fi
if ! wait_for_servo_services; then
  echo "舵机安全服务没有就绪。" >&2
  exit 4
fi

# The dedicated tracker must never publish the aircraft velocity setpoint.
tracker_info=$(rosnode info /camera_pitch_visual_tracker 2>/dev/null || true)
if grep -Fq '/mavros/setpoint_velocity/cmd_vel' <<<"$tracker_info"; then
  echo "安全检查失败：仅视觉测试节点仍发布飞行控制话题。" >&2
  exit 4
fi

echo "正在执行舵机两级安全解锁……"
call_servo_gate "$SERVO_HARDWARE_ENABLE_SERVICE" true
call_servo_gate "$SERVO_MAPPER_ENABLE_SERVICE" false
call_servo_gate "$SERVO_MAPPER_ENABLE_SERVICE" true
if ! wait_for_servo_hardware_active; then
  echo "舵机 PWM 没有进入 active 状态。" >&2
  exit 4
fi
echo "舵机 PWM 已激活；开始实时打印视觉和舵机期望角。"

stream_topic "DETECTION" "$DETECTION_TOPIC" &
MONITOR_PIDS+=("$!")
stream_topic "SERVO_DEG" "$SERVO_COMMAND_TOPIC" &
MONITOR_PIDS+=("$!")

set +e
wait "$CONTROL_PID"
control_status=$?
set -e
CONTROL_PID=""
exit "$control_status"
