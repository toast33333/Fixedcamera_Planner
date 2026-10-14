#!/usr/bin/env bash
# ROS environment hooks are not compatible with nounset; enable it only after
# both setup files have been sourced.
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
DEFAULT_WS_DIR=$(cd "$REPO_DIR/../.." && pwd)

MODE=${1:-}
WS_DIR=${WS_DIR:-$DEFAULT_WS_DIR}
STATE_SOURCE=${STATE_SOURCE:-px4}
DETECTION_MODE=${DETECTION_MODE:-1}
DETECTION_TOPIC=${DETECTION_TOPIC:-/object_detections}
MAVROS_STATE_TOPIC=${MAVROS_STATE_TOPIC:-/mavros/state}
FORWARD_SPEED_MPS=${FORWARD_SPEED_MPS:-5.0}
LOST_FORWARD_SPEED_MPS=${LOST_FORWARD_SPEED_MPS:-0.0}
PREFLIGHT_TIMEOUT_S=${PREFLIGHT_TIMEOUT_S:-10}
SKIP_PREFLIGHT_CHECKS=${SKIP_PREFLIGHT_CHECKS:-0}

case "$MODE" in
  full)
    ENABLE_HORIZONTAL_CONTROL=true
    ENABLE_HEIGHT_CONTROL=true
    MODE_TEXT="完整控制（水平 + 竖直）"
    ;;
  horizontal_only)
    ENABLE_HORIZONTAL_CONTROL=true
    ENABLE_HEIGHT_CONTROL=false
    MODE_TEXT="仅水平控制（竖直指令为零）"
    ;;
  vertical_only)
    ENABLE_HORIZONTAL_CONTROL=false
    ENABLE_HEIGHT_CONTROL=true
    MODE_TEXT="仅竖直控制（横向和偏航指令为零）"
    ;;
  *)
    echo "用法: $0 {full|horizontal_only|vertical_only}" >&2
    exit 2
    ;;
esac

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
  px4)
    LAUNCH_FILE=fixed_camera_ibvs.launch
    ;;
  inslib)
    LAUNCH_FILE=fixed_camera_ibvs_inslib.launch
    ;;
  *)
    echo "STATE_SOURCE 只能是 px4 或 inslib。" >&2
    exit 2
    ;;
esac

if [[ "$SKIP_PREFLIGHT_CHECKS" != "1" ]]; then
  if ! rosparam get /rosdistro >/dev/null 2>&1; then
    echo "ROS master 未运行。请先启动飞控、MAVROS 和视觉节点。" >&2
    exit 4
  fi

  if rosnode list 2>/dev/null | grep -Fxq /fixed_camera_ibvs; then
    echo "/fixed_camera_ibvs 已经在运行；请勿重复启动控制器。" >&2
    exit 4
  fi

  state_message=$(timeout "${PREFLIGHT_TIMEOUT_S}s" \
    rostopic echo -n 1 "$MAVROS_STATE_TOPIC" 2>/dev/null || true)
  if ! grep -Eq 'connected: *(True|true|1)' <<<"$state_message"; then
    echo "MAVROS 未连接，或 $MAVROS_STATE_TOPIC 在 ${PREFLIGHT_TIMEOUT_S}s 内无有效消息。" >&2
    exit 4
  fi

  detection_message=$(timeout "${PREFLIGHT_TIMEOUT_S}s" \
    rostopic echo -n 1 "$DETECTION_TOPIC" 2>/dev/null || true)
  if ! grep -Eq 'is_tracking: *(True|true|1)' <<<"$detection_message"; then
    echo "视觉尚未稳定跟踪：$DETECTION_TOPIC 的 is_tracking 不是 true。" >&2
    echo "请先让靶机进入测试机画面，再重新运行本脚本。" >&2
    exit 4
  fi
fi

RUN_STAMP=$(date +%Y%m%d_%H%M%S)
ROS_LOG_DIR=${ROS_LOG_DIR:-$WS_DIR/log/field_visual_intercept/${RUN_STAMP}_${MODE}}
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

echo "============================================================"
echo "固定相机外场视觉控制器"
echo "模式: $MODE_TEXT"
echo "状态源: $STATE_SOURCE"
echo "视觉话题: $DETECTION_TOPIC"
echo "时间对齐: 使用图像采集 header.stamp 对齐 IMU，不使用固定延迟估计"
echo "前向速度: $FORWARD_SPEED_MPS m/s"
echo "丢失目标后的前向速度: $LOST_FORWARD_SPEED_MPS m/s"
echo "日志目录: $ROS_LOG_DIR"
echo "本脚本不会解锁、起飞或切换模式。"
echo "控制器启动后，由飞手确认安全，再手动将测试机切入 GUIDED。"
echo "============================================================"

exec roslaunch png_planner "$LAUNCH_FILE" \
  detection_mode:="$DETECTION_MODE" \
  detection_topic:="$DETECTION_TOPIC" \
  enable_horizontal_control:="$ENABLE_HORIZONTAL_CONTROL" \
  enable_height_control:="$ENABLE_HEIGHT_CONTROL" \
  forward_speed_mps:="$FORWARD_SPEED_MPS" \
  lost_forward_speed_mps:="$LOST_FORWARD_SPEED_MPS"
