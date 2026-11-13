#!/usr/bin/env bash
# Ground-only full-chain test for the visual interception controller.
# It never publishes commands to MAVROS and refuses to run while armed/GUIDED.
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
DEFAULT_WS_DIR=$(cd "$REPO_DIR/../.." && pwd)

WS_DIR=${WS_DIR:-$DEFAULT_WS_DIR}
SIMULATED_SPEED_MPS=${SIMULATED_SPEED_MPS:-5.0}
VISUAL_NODE=${VISUAL_NODE:-/field_yolo_detector}
VISUAL_LAUNCH_PACKAGE=${VISUAL_LAUNCH_PACKAGE:-detection_msgs}
VISUAL_LAUNCH_FILE=${VISUAL_LAUNCH_FILE:-field_detector.launch}
VISUAL_START_TIMEOUT_S=${VISUAL_START_TIMEOUT_S:-45}
DETECTION_TOPIC=${DETECTION_TOPIC:-/object_detections}
IMU_TOPIC=${IMU_TOPIC:-/mavros/imu/data}
MAVROS_STATE_TOPIC=${MAVROS_STATE_TOPIC:-/mavros/state}
TEST_POSE_TOPIC=${TEST_POSE_TOPIC:-/fixed_camera_ibvs/ground_test_pose}
TEST_VELOCITY_TOPIC=${TEST_VELOCITY_TOPIC:-/fixed_camera_ibvs/ground_test_velocity}
TEST_COMMAND_TOPIC=${TEST_COMMAND_TOPIC:-/fixed_camera_ibvs/ground_test_cmd}

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "未找到 /opt/ros/noetic/setup.bash。" >&2
  exit 3
fi
if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
  echo "未找到工作空间环境: $WS_DIR/devel/setup.bash" >&2
  exit 3
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "$WS_DIR/devel/setup.bash"
set -u

RUN_STAMP=$(date +%Y%m%d_%H%M%S)
ROS_LOG_DIR=${ROS_LOG_DIR:-$WS_DIR/log/ground_visual_intercept/$RUN_STAMP}
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

POSE_PUBLISHER_PID=""
VELOCITY_PUBLISHER_PID=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "$POSE_PUBLISHER_PID" "$VELOCITY_PUBLISHER_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
      wait "$pid" >/dev/null 2>&1 || true
    fi
  done
  exit "$status"
}
trap cleanup EXIT INT TERM

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
  [[ ! -f "$visual_log" ]] || tail -n 40 "$visual_log" >&2 || true
  return 1
}

read_one_topic() {
  local topic=$1
  local timeout_s=${2:-5}
  timeout "${timeout_s}s" rostopic echo -n 1 "$topic" 2>/dev/null || true
}

if ! rosparam get /rosdistro >/dev/null 2>&1; then
  echo "ROS master 未运行。" >&2
  exit 4
fi

if rosnode list 2>/dev/null | grep -Fxq /fixed_camera_ibvs; then
  echo "/fixed_camera_ibvs 已经运行，请先停止现有控制器。" >&2
  exit 4
fi

if rosnode list 2>/dev/null | grep -Eq \
    '^/camera_pitch_servo_(open_loop|sysfs_pwm)$'; then
  echo "检测到已有舵机节点；请先停止旧的舵机测试或控制进程，避免节点重名。" >&2
  exit 4
fi

state_message=$(read_one_topic "$MAVROS_STATE_TOPIC" 5)
if [[ -z "$state_message" ]]; then
  echo "没有收到飞控状态: $MAVROS_STATE_TOPIC" >&2
  exit 4
fi
if grep -Eq 'armed: *(True|true|1)' <<<"$state_message"; then
  echo "地面测试拒绝运行：飞控当前已解锁。" >&2
  exit 4
fi
if grep -Eq 'guided: *(True|true|1)|mode: *"?GUIDED' <<<"$state_message"; then
  echo "地面测试拒绝运行：飞控当前处于 GUIDED。" >&2
  exit 4
fi

if ! ensure_visual_detector; then
  exit 4
fi

if [[ -z "$(read_one_topic "$IMU_TOPIC" 5)" ]]; then
  echo "没有收到 IMU: $IMU_TOPIC" >&2
  exit 4
fi
if [[ -z "$(read_one_topic "$DETECTION_TOPIC" 10)" ]]; then
  echo "没有收到视觉检测消息: $DETECTION_TOPIC" >&2
  exit 4
fi

rostopic pub -r 20 "$TEST_POSE_TOPIC" geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
  >"$ROS_LOG_DIR/fake_pose.log" 2>&1 &
POSE_PUBLISHER_PID=$!

rostopic pub -r 20 "$TEST_VELOCITY_TOPIC" geometry_msgs/TwistStamped \
  "{header: {frame_id: 'map'}, twist: {linear: {x: ${SIMULATED_SPEED_MPS}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  >"$ROS_LOG_DIR/fake_velocity.log" 2>&1 &
VELOCITY_PUBLISHER_PID=$!

echo "============================================================"
echo "固定相机拦截算法：室内地面全链路测试"
echo "真实输入: 相机检测 + 飞控 IMU"
echo "模拟输入: 零位置 + ${SIMULATED_SPEED_MPS} m/s 水平速度"
echo "隔离输出: $TEST_COMMAND_TOPIC"
echo "不会向 /mavros/setpoint_velocity/cmd_vel 发布控制指令"
echo "舵机角度算法与两个舵机节点已启动，可查看 /camera_pitch_servo/command_deg"
echo "舵机两级硬件开关强制保持关闭，不会输出 PWM"
echo "飞控必须保持未解锁、非 GUIDED"
echo "按 Ctrl-C 结束测试"
echo "日志目录: $ROS_LOG_DIR"
echo "============================================================"

roslaunch png_planner fixed_camera_ibvs_ground_test.launch \
  detection_topic:="$DETECTION_TOPIC" \
  attitude_topic:="$IMU_TOPIC" \
  pose_topic:="$TEST_POSE_TOPIC" \
  velocity_topic:="$TEST_VELOCITY_TOPIC" \
  command_topic:="$TEST_COMMAND_TOPIC" \
  forward_speed_mps:="$SIMULATED_SPEED_MPS"
