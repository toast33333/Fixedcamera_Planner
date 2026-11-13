#!/usr/bin/env bash
# ROS environment hooks are not compatible with nounset; enable it only after
# both setup files have been sourced.
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
DEFAULT_WS_DIR=$(cd "$REPO_DIR/../.." && pwd)

MODE=${1:-}
WS_DIR=${WS_DIR:-$DEFAULT_WS_DIR}
STATE_SOURCE=${STATE_SOURCE:-mavros}
DETECTION_MODE=${DETECTION_MODE:-1}
DETECTION_TOPIC=${DETECTION_TOPIC:-/object_detections}
MAVROS_STATE_TOPIC=${MAVROS_STATE_TOPIC:-/mavros/state}
MAVROS_DIAGNOSTICS_TOPIC=${MAVROS_DIAGNOSTICS_TOPIC:-/diagnostics}
MAVROS_FRAME_PARAM=${MAVROS_FRAME_PARAM:-/mavros/setpoint_velocity/mav_frame}
IMU_TOPIC=${IMU_TOPIC:-/mavros/imu/data}
POSE_TOPIC=${POSE_TOPIC:-/mavros/local_position/pose}
VELOCITY_TOPIC=${VELOCITY_TOPIC:-/mavros/local_position/velocity_local}
FORWARD_SPEED_MPS=${FORWARD_SPEED_MPS:-5.0}
LOST_FORWARD_SPEED_MPS=${LOST_FORWARD_SPEED_MPS:-0.0}
PREFLIGHT_TIMEOUT_S=${PREFLIGHT_TIMEOUT_S:-10}
SKIP_PREFLIGHT_CHECKS=${SKIP_PREFLIGHT_CHECKS:-0}
VISUAL_NODE=${VISUAL_NODE:-/field_yolo_detector}
VISUAL_LAUNCH_PACKAGE=${VISUAL_LAUNCH_PACKAGE:-detection_msgs}
VISUAL_LAUNCH_FILE=${VISUAL_LAUNCH_FILE:-field_detector.launch}
VISUAL_START_TIMEOUT_S=${VISUAL_START_TIMEOUT_S:-45}
SERVO_HARDWARE_AUTO_ARM=${SERVO_HARDWARE_AUTO_ARM:-0}
SERVO_START_TIMEOUT_S=${SERVO_START_TIMEOUT_S:-10}
SERVO_MAPPER_ENABLE_SERVICE=${SERVO_MAPPER_ENABLE_SERVICE:-/camera_pitch_servo/set_output_enabled}
SERVO_HARDWARE_ENABLE_SERVICE=${SERVO_HARDWARE_ENABLE_SERVICE:-/camera_pitch_servo/set_hardware_output_enabled}
SERVO_HARDWARE_ACTIVE_TOPIC=${SERVO_HARDWARE_ACTIVE_TOPIC:-/camera_pitch_servo/hardware_active}
# The RK3576 container entrypoint requests these streams once at startup.
# Enable this only when running the field script without that entrypoint.
REQUEST_APM_STREAMS=${REQUEST_APM_STREAMS:-0}
REQUIRE_ARDUPILOT=${REQUIRE_ARDUPILOT:-1}
ALLOW_ALREADY_GUIDED=${ALLOW_ALREADY_GUIDED:-0}

APM_RAW_SENSORS_HZ=${APM_RAW_SENSORS_HZ:-20}
APM_EXTENDED_STATUS_HZ=${APM_EXTENDED_STATUS_HZ:-5}
APM_POSITION_HZ=${APM_POSITION_HZ:-20}
APM_EXTRA1_HZ=${APM_EXTRA1_HZ:-30}
APM_EXTRA2_HZ=${APM_EXTRA2_HZ:-10}
APM_EXTRA3_HZ=${APM_EXTRA3_HZ:-5}

case "$MODE" in
  full)
    ENABLE_HORIZONTAL_CONTROL=true
    ENABLE_HEIGHT_CONTROL=true
    SERVO_CONTROL_SUPPORTED=true
    MODE_TEXT="完整控制（水平 + 竖直）"
    ;;
  horizontal_only)
    ENABLE_HORIZONTAL_CONTROL=true
    ENABLE_HEIGHT_CONTROL=false
    SERVO_CONTROL_SUPPORTED=false
    MODE_TEXT="仅水平控制（竖直指令为零）"
    ;;
  vertical_only)
    ENABLE_HORIZONTAL_CONTROL=false
    ENABLE_HEIGHT_CONTROL=true
    SERVO_CONTROL_SUPPORTED=true
    MODE_TEXT="仅竖直控制（横向和偏航指令为零）"
    ;;
  *)
    echo "用法: $0 {full|horizontal_only|vertical_only}" >&2
    exit 2
    ;;
esac

if [[ -z "${ENABLE_CAMERA_PITCH_SERVO+x}" ]]; then
  # With no physical output, the controller must not assume that the camera
  # followed its angle command.  Therefore formal servo control is enabled by
  # default only together with the explicit hardware auto-arm request.
  if [[ "$SERVO_HARDWARE_AUTO_ARM" == "1" &&
        "$SERVO_CONTROL_SUPPORTED" == "true" ]]; then
    ENABLE_CAMERA_PITCH_SERVO=true
  else
    ENABLE_CAMERA_PITCH_SERVO=false
  fi
fi
if [[ "$ENABLE_CAMERA_PITCH_SERVO" != "true" &&
      "$ENABLE_CAMERA_PITCH_SERVO" != "false" ]]; then
  echo "ENABLE_CAMERA_PITCH_SERVO 只能是 true 或 false。" >&2
  exit 2
fi
if [[ "$SERVO_HARDWARE_AUTO_ARM" != "0" &&
      "$SERVO_HARDWARE_AUTO_ARM" != "1" ]]; then
  echo "SERVO_HARDWARE_AUTO_ARM 只能是 0 或 1。" >&2
  exit 2
fi
if [[ "$SERVO_HARDWARE_AUTO_ARM" == "1" &&
      "$ENABLE_CAMERA_PITCH_SERVO" != "true" ]]; then
  echo "舵机控制算法关闭时不能启用舵机硬件输出。" >&2
  exit 2
fi
if [[ "$SERVO_CONTROL_SUPPORTED" != "true" &&
      "$ENABLE_CAMERA_PITCH_SERVO" == "true" ]]; then
  echo "仅水平模式禁止启用竖直舵机控制。" >&2
  exit 2
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "未找到 /opt/ros/noetic/setup.bash。" >&2
  exit 3
fi
if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
  echo "未找到工作空间环境: $WS_DIR/devel/setup.bash" >&2
  echo "请先编译工作空间，或通过 WS_DIR 指定正确路径。" >&2
  exit 3
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "$WS_DIR/devel/setup.bash"
set -u

case "$STATE_SOURCE" in
  mavros|apm)
    LAUNCH_FILE=fixed_camera_ibvs.launch
    ;;
  px4)
    echo "警告：STATE_SOURCE=px4 是旧名称；实机现为 ArduCopter，请改用 mavros。" >&2
    LAUNCH_FILE=fixed_camera_ibvs.launch
    ;;
  inslib)
    LAUNCH_FILE=fixed_camera_ibvs_inslib.launch
    ;;
  *)
    echo "STATE_SOURCE 只能是 mavros、apm、inslib（px4 仅保留旧脚本兼容）。" >&2
    exit 2
    ;;
esac

read_one_topic() {
  local topic=$1
  timeout "${PREFLIGHT_TIMEOUT_S}s" rostopic echo -n 1 "$topic" 2>/dev/null || true
}

request_apm_stream() {
  local stream_id=$1
  local rate_hz=$2
  # This MAVROS version may apply the request while keeping the service call
  # blocked. Bound the call; the fresh-topic checks below are authoritative.
  timeout 2 rosservice call /mavros/set_stream_rate \
    "{stream_id: ${stream_id}, message_rate: ${rate_hz}, on_off: true}" \
    >/dev/null 2>&1 || true
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
  echo "$response"
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
  echo "舵机节点在 ${SERVO_START_TIMEOUT_S}s 内未提供安全开关服务。" >&2
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
  echo "舵机 PWM 在 ${SERVO_START_TIMEOUT_S}s 内没有进入 active 状态。" >&2
  return 1
}

RUN_STAMP=$(date +%Y%m%d_%H%M%S)
ROS_LOG_DIR=${ROS_LOG_DIR:-$WS_DIR/log/field_visual_intercept/${RUN_STAMP}_${MODE}}
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

visual_node_running() {
  rosnode list 2>/dev/null | grep -Fxq "$VISUAL_NODE"
}

ensure_visual_detector() {
  local visual_log="$ROS_LOG_DIR/visual_detector.log"
  local waited=0

  if visual_node_running; then
    echo "视觉节点已运行，直接复用: $VISUAL_NODE"
    return 0
  fi

  if pgrep -f "roslaunch ${VISUAL_LAUNCH_PACKAGE} ${VISUAL_LAUNCH_FILE}" >/dev/null 2>&1; then
    echo "视觉启动进程已经存在，等待节点注册: $VISUAL_NODE"
  else
    echo "视觉节点未运行，正在启动 ${VISUAL_LAUNCH_PACKAGE}/${VISUAL_LAUNCH_FILE}"
    setsid roslaunch "$VISUAL_LAUNCH_PACKAGE" "$VISUAL_LAUNCH_FILE" \
      >"$visual_log" 2>&1 </dev/null &
    echo $! >"$ROS_LOG_DIR/visual_detector_roslaunch.pid"
  fi

  while (( waited < VISUAL_START_TIMEOUT_S )); do
    if visual_node_running; then
      echo "视觉节点启动成功: $VISUAL_NODE"
      return 0
    fi
    sleep 1
    ((waited += 1))
  done

  echo "视觉节点在 ${VISUAL_START_TIMEOUT_S}s 内未启动成功。" >&2
  echo "视觉日志: $visual_log" >&2
  if [[ -f "$visual_log" ]]; then
    tail -n 40 "$visual_log" >&2 || true
  fi
  return 1
}

if ! rosparam get /rosdistro >/dev/null 2>&1; then
  echo "ROS master 未运行，无法启动视觉和拦截控制节点。" >&2
  exit 4
fi

if rosnode list 2>/dev/null | grep -Fxq /fixed_camera_ibvs; then
  echo "/fixed_camera_ibvs 已经在运行；请勿重复启动控制器。" >&2
  exit 4
fi

if rosnode list 2>/dev/null | grep -Eq \
    '^/camera_pitch_servo_(open_loop|sysfs_pwm)$'; then
  echo "检测到已有舵机节点；请先停止旧的舵机测试或控制进程，避免节点重名。" >&2
  exit 4
fi

if ! ensure_visual_detector; then
  exit 4
fi

if [[ "$SKIP_PREFLIGHT_CHECKS" != "1" ]]; then
  state_message=$(read_one_topic "$MAVROS_STATE_TOPIC")
  if ! grep -Eq 'connected: *(True|true|1)' <<<"$state_message"; then
    echo "MAVROS 未连接，或 $MAVROS_STATE_TOPIC 在 ${PREFLIGHT_TIMEOUT_S}s 内无有效消息。" >&2
    exit 4
  fi

  if [[ "$ALLOW_ALREADY_GUIDED" != "1" ]] && \
     grep -Eq 'mode: *"?GUIDED"?' <<<"$state_message"; then
    echo "飞控已经处于 GUIDED；为避免控制器启动瞬间接管，本脚本拒绝继续。" >&2
    echo "请先由飞手切回悬停模式，启动控制器后再手动切入 GUIDED。" >&2
    echo "仅在明确需要故障恢复时使用 ALLOW_ALREADY_GUIDED=1。" >&2
    exit 4
  fi

  mav_frame=$(rosparam get "$MAVROS_FRAME_PARAM" 2>/dev/null || true)
  if [[ "$mav_frame" != "LOCAL_NED" ]]; then
    echo "MAVROS 速度坐标系必须为 LOCAL_NED，当前为: ${mav_frame:-未设置}" >&2
    echo "当前控制器已经把机体系前向速度旋转到世界 ENU，不能再使用 BODY_NED。" >&2
    exit 4
  fi

  if [[ "$REQUIRE_ARDUPILOT" == "1" ]]; then
    diagnostics_message=$(read_one_topic "$MAVROS_DIAGNOSTICS_TOPIC")
    if ! awk '
      /key: "Autopilot type"/ { checking = 1; next }
      checking && /value: "ArduPilot"/ { found = 1 }
      END { exit(found ? 0 : 1) }
    ' <<<"$diagnostics_message"; then
      echo "未从 $MAVROS_DIAGNOSTICS_TOPIC 确认飞控类型为 ArduPilot。" >&2
      exit 4
    fi
  fi

  if [[ "$REQUEST_APM_STREAMS" == "1" ]]; then
    if ! rosservice list 2>/dev/null | grep -Fxq /mavros/set_stream_rate; then
      echo "缺少 /mavros/set_stream_rate，无法配置 ArduCopter 的 MAVLink 数据流。" >&2
      exit 4
    fi
    request_apm_stream 1 "$APM_RAW_SENSORS_HZ"
    request_apm_stream 2 "$APM_EXTENDED_STATUS_HZ"
    request_apm_stream 6 "$APM_POSITION_HZ"
    request_apm_stream 10 "$APM_EXTRA1_HZ"
    request_apm_stream 11 "$APM_EXTRA2_HZ"
    request_apm_stream 12 "$APM_EXTRA3_HZ"
  fi

  imu_message=$(read_one_topic "$IMU_TOPIC")
  if [[ -z "$imu_message" ]]; then
    echo "没有收到 IMU：$IMU_TOPIC。" >&2
    exit 4
  fi
  pose_message=$(read_one_topic "$POSE_TOPIC")
  if [[ -z "$pose_message" ]]; then
    echo "没有收到本地位姿：$POSE_TOPIC。请检查 GPS/EKF 是否已经建立位置解。" >&2
    exit 4
  fi
  velocity_message=$(read_one_topic "$VELOCITY_TOPIC")
  if [[ -z "$velocity_message" ]]; then
    echo "没有收到本地速度：$VELOCITY_TOPIC。请检查 GPS/EKF 和 MAVLink 数据流。" >&2
    exit 4
  fi

  detection_message=$(read_one_topic "$DETECTION_TOPIC")
  if ! grep -Eq 'is_tracking: *(True|true|1)' <<<"$detection_message"; then
    echo "视觉尚未稳定跟踪：$DETECTION_TOPIC 的 is_tracking 不是 true。" >&2
    echo "请先让靶机进入测试机画面，再重新运行本脚本。" >&2
    exit 4
  fi
fi

echo "============================================================"
echo "固定相机外场视觉控制器"
echo "模式: $MODE_TEXT"
echo "状态源: $STATE_SOURCE"
echo "飞控接口: ArduCopter/MAVROS，速度坐标系 LOCAL_NED"
echo "视觉节点: $VISUAL_NODE（本脚本负责启动或复用）"
echo "视觉话题: $DETECTION_TOPIC"
echo "视野保持: 开启；仅 score > 0.5 的检测框可触发附加修正"
echo "时间对齐: 使用图像采集 header.stamp 对齐 IMU，不使用固定延迟估计"
echo "前向速度: $FORWARD_SPEED_MPS m/s"
echo "丢失目标后的前向速度: $LOST_FORWARD_SPEED_MPS m/s"
echo "舵机节点: 随本控制器启动并退出"
echo "舵机角度控制: $ENABLE_CAMERA_PITCH_SERVO"
if [[ "$SERVO_HARDWARE_AUTO_ARM" == "1" ]]; then
  echo "舵机 PWM: 将在节点就绪后执行两级安全解锁"
else
  echo "舵机 PWM: 安全关闭（需要时设置 SERVO_HARDWARE_AUTO_ARM=1）"
fi
echo "日志目录: $ROS_LOG_DIR"
echo "本脚本不会解锁、起飞或切换模式。"
echo "控制器启动后，由飞手确认安全，再手动将测试机切入 GUIDED。"
echo "============================================================"

LAUNCH_ARGUMENTS=(
  detection_mode:="$DETECTION_MODE"
  detection_topic:="$DETECTION_TOPIC"
  enable_horizontal_control:="$ENABLE_HORIZONTAL_CONTROL"
  enable_height_control:="$ENABLE_HEIGHT_CONTROL"
  enable_camera_pitch_servo:="$ENABLE_CAMERA_PITCH_SERVO"
  forward_speed_mps:="$FORWARD_SPEED_MPS"
  lost_forward_speed_mps:="$LOST_FORWARD_SPEED_MPS"
)

if [[ "$SERVO_HARDWARE_AUTO_ARM" != "1" ]]; then
  exec roslaunch png_planner "$LAUNCH_FILE" "${LAUNCH_ARGUMENTS[@]}"
fi

CONTROL_LAUNCH_PID=""
cleanup_control_and_servo() {
  local status=$?
  trap - EXIT INT TERM

  # Disable physical output first.  Both calls are best-effort so cleanup also
  # works if one node has already stopped.
  call_servo_gate "$SERVO_HARDWARE_ENABLE_SERVICE" false >/dev/null 2>&1 || true
  call_servo_gate "$SERVO_MAPPER_ENABLE_SERVICE" false >/dev/null 2>&1 || true

  if [[ -n "$CONTROL_LAUNCH_PID" ]] &&
     kill -0 "$CONTROL_LAUNCH_PID" >/dev/null 2>&1; then
    kill -INT "$CONTROL_LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$CONTROL_LAUNCH_PID" >/dev/null 2>&1 || true
  fi
  exit "$status"
}
trap cleanup_control_and_servo EXIT INT TERM

roslaunch png_planner "$LAUNCH_FILE" "${LAUNCH_ARGUMENTS[@]}" &
CONTROL_LAUNCH_PID=$!

wait_for_servo_services
echo "正在执行舵机两级安全解锁……"
call_servo_gate "$SERVO_HARDWARE_ENABLE_SERVICE" true
call_servo_gate "$SERVO_MAPPER_ENABLE_SERVICE" false
call_servo_gate "$SERVO_MAPPER_ENABLE_SERVICE" true
wait_for_servo_hardware_active
echo "舵机 PWM 已激活；退出本脚本时会自动关闭。"

set +e
wait "$CONTROL_LAUNCH_PID"
control_status=$?
set -e
CONTROL_LAUNCH_PID=""
exit "$control_status"
