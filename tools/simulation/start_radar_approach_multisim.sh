#!/usr/bin/env bash
set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

ROS_SETUP=/opt/ros/noetic/setup.bash
PX4_DIR=/home/toast/PX4-Autopilot
WS_DIR=/home/toast/fixedcamera_radar_multisim_ws
RUN_ROOT=/home/toast/fixedcamera_radar_multisim_results
DISPLAY_VALUE=${DISPLAY_VALUE:-:1}
XAUTHORITY_FILE=${XAUTHORITY_FILE:-/run/user/1000/gdm/Xauthority}

CASE_NAME=${CASE_NAME:-nominal_tail}
SCENARIO_NAME=${SCENARIO_NAME:-radar_approach}
TEST_GROUP=${TEST_GROUP:-unspecified}
ERROR_PROFILE=${ERROR_PROFILE:-clean}
BATCH_ID=${BATCH_ID:-}
TARGET_SPAWN_X=${TARGET_SPAWN_X:-70.0}
TARGET_SPAWN_Y=${TARGET_SPAWN_Y:-0.0}
INTERCEPTOR_ALTITUDE_M=${INTERCEPTOR_ALTITUDE_M:-4.0}
TARGET_ALTITUDE_M=${TARGET_ALTITUDE_M:-9.0}
TARGET_SPEED_MPS=${TARGET_SPEED_MPS:-2.0}
TARGET_HEADING_DEG=${TARGET_HEADING_DEG:-0.0}
TARGET_TURN_RATE_DEGPS=${TARGET_TURN_RATE_DEGPS:-0.0}
TARGET_CLIMB_RATE_MPS=${TARGET_CLIMB_RATE_MPS:-0.0}
RADAR_PERIOD_SEC=${RADAR_PERIOD_SEC:-3.0}
RADAR_TIMEOUT_SEC=${RADAR_TIMEOUT_SEC:-9.5}
RADAR_BOOTSTRAP_SAMPLES=${RADAR_BOOTSTRAP_SAMPLES:-4}
RADAR_MEASUREMENT_STD_M=${RADAR_MEASUREMENT_STD_M:-5.773502692}
RADAR_PROCESS_ACCEL_STD_MPS2=${RADAR_PROCESS_ACCEL_STD_MPS2:-0.20}
RADAR_BOOTSTRAP_HUBER_DELTA_M=${RADAR_BOOTSTRAP_HUBER_DELTA_M:-10.0}
RADAR_NIS_SOFT_THRESHOLD=${RADAR_NIS_SOFT_THRESHOLD:-9.21}
RADAR_NIS_REJECT_THRESHOLD=${RADAR_NIS_REJECT_THRESHOLD:-25.0}
FILTER_ALPHA=${FILTER_ALPHA:-0.70}
FILTER_BETA=${FILTER_BETA:-0.20}
MAX_RADAR_INNOVATION_M=${MAX_RADAR_INNOVATION_M:-30.0}
MAX_TARGET_SPEED_MPS=${MAX_TARGET_SPEED_MPS:-7.0}
MAX_TARGET_HORIZONTAL_SPEED_MPS=${MAX_TARGET_HORIZONTAL_SPEED_MPS:-7.0}
MAX_TARGET_VERTICAL_SPEED_MPS=${MAX_TARGET_VERTICAL_SPEED_MPS:-0.0}
TERMINAL_CAMERA_RANGE_M=${TERMINAL_CAMERA_RANGE_M:-25.0}
POSITION_GAIN_XY=${POSITION_GAIN_XY:-0.40}
POSITION_GAIN_Z=${POSITION_GAIN_Z:-0.35}
USE_BRAKING_SPEED_PROFILE=${USE_BRAKING_SPEED_PROFILE:-true}
APPROACH_BRAKING_DECELERATION_MPS2=${APPROACH_BRAKING_DECELERATION_MPS2:-2.0}
APPROACH_BRAKING_POSITION_BUFFER_M=${APPROACH_BRAKING_POSITION_BUFFER_M:-2.5}
TARGET_VELOCITY_FEEDFORWARD_GAIN_XY=${TARGET_VELOCITY_FEEDFORWARD_GAIN_XY:-1.0}
TARGET_VELOCITY_FEEDFORWARD_GAIN_Z=${TARGET_VELOCITY_FEEDFORWARD_GAIN_Z:-0.0}
ENFORCE_MINIMUM_HANDOVER_ALTITUDE=${ENFORCE_MINIMUM_HANDOVER_ALTITUDE:-true}
MINIMUM_HANDOVER_ALTITUDE_M=${MINIMUM_HANDOVER_ALTITUDE_M:-3.0}
CENTER_TOLERANCE_U_PX=${CENTER_TOLERANCE_U_PX:-80.0}
CENTER_TOLERANCE_V_PX=${CENTER_TOLERANCE_V_PX:-80.0}
REQUIRED_CENTERED_DETECTIONS=${REQUIRED_CENTERED_DETECTIONS:-10}
HANDOVER_POSITION_TOLERANCE_M=${HANDOVER_POSITION_TOLERANCE_M:-5.0}
HANDOVER_YAW_TOLERANCE_DEG=${HANDOVER_YAW_TOLERANCE_DEG:-20.0}
USE_LEGACY_POSE_HANDOVER_GATE=${USE_LEGACY_POSE_HANDOVER_GATE:-true}
HANDOVER_MIN_ESTIMATED_RANGE_M=${HANDOVER_MIN_ESTIMATED_RANGE_M:-18.0}
HANDOVER_MAX_ESTIMATED_RANGE_M=${HANDOVER_MAX_ESTIMATED_RANGE_M:-38.0}
MAX_HANDOVER_VELOCITY_STD_MPS=${MAX_HANDOVER_VELOCITY_STD_MPS:-1.0}
MAX_HANDOVER_RADAR_AGE_SEC=${MAX_HANDOVER_RADAR_AGE_SEC:-5.0}
MAX_HANDOVER_COMMAND_YAW_RATE_RADPS=${MAX_HANDOVER_COMMAND_YAW_RATE_RADPS:-0.30}
MAX_HANDOVER_CLOSING_SPEED_MPS=${MAX_HANDOVER_CLOSING_SPEED_MPS:-2.0}
MAX_HANDOVER_OPENING_SPEED_MPS=${MAX_HANDOVER_OPENING_SPEED_MPS:-0.5}
RADAR_NOISE_STD_M=${RADAR_NOISE_STD_M:-0.0}
RADAR_PERIOD_JITTER_STD_SEC=${RADAR_PERIOD_JITTER_STD_SEC:-0.0}
RADAR_DELAY_MEAN_SEC=${RADAR_DELAY_MEAN_SEC:-0.0}
RADAR_DELAY_JITTER_STD_SEC=${RADAR_DELAY_JITTER_STD_SEC:-0.0}
RADAR_DROPOUT_PROBABILITY=${RADAR_DROPOUT_PROBABILITY:-0.0}
RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS=${RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS:-0.0}
RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS=${RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS:-0.0}
RADAR_BIAS_LIMIT_XY_M=${RADAR_BIAS_LIMIT_XY_M:-0.0}
RADAR_BIAS_LIMIT_Z_M=${RADAR_BIAS_LIMIT_Z_M:-0.0}
RADAR_DROPOUT_START_SEC=${RADAR_DROPOUT_START_SEC:--1.0}
RADAR_DROPOUT_DURATION_SEC=${RADAR_DROPOUT_DURATION_SEC:-0.0}
RADAR_ACQUIRE_DURATION_SEC=${RADAR_ACQUIRE_DURATION_SEC:-9.7}
RADAR_ACQUIRE_TIMEOUT_SEC=${RADAR_ACQUIRE_TIMEOUT_SEC:-24.0}
RADAR_SEED=${RADAR_SEED:-1}
CAPTURE_DISTANCE_M=${CAPTURE_DISTANCE_M:-5.0}
CHASE_TIMEOUT_S=${CHASE_TIMEOUT_S:-45.0}
RESULT_HOLD_DURATION_SEC=${RESULT_HOLD_DURATION_SEC:-2.0}
HANDOVER_BLEND_DURATION_SEC=${HANDOVER_BLEND_DURATION_SEC:-0.0}
DETECTION_RANGE_M=${DETECTION_RANGE_M:-30.0}
RUN_WALL_TIMEOUT_S=${RUN_WALL_TIMEOUT_S:-120}
EXPECTED_RESULT=${EXPECTED_RESULT:-unspecified}
SHOW_GUI=${SHOW_GUI:-0}
SHOW_RQT=${SHOW_RQT:-$SHOW_GUI}
DEMO_GUI=${DEMO_GUI:-0}
DEMO_VISUAL_MARKERS=${DEMO_VISUAL_MARKERS:-0}
RECORD_VIDEO=${RECORD_VIDEO:-0}
VIDEO_FRAMERATE=${VIDEO_FRAMERATE:-24}
VIDEO_BITRATE_KBPS=${VIDEO_BITRATE_KBPS:-6000}
FOV_BARRIER_GAIN=${FOV_BARRIER_GAIN:-0.20}
FOV_BARRIER_MAX=${FOV_BARRIER_MAX:-4.0}
FOV_SLOWDOWN_START_RATIO=${FOV_SLOWDOWN_START_RATIO:-0.99}
FOV_MIN_FORWARD_SPEED_RATIO=${FOV_MIN_FORWARD_SPEED_RATIO:-1.0}
FOV_EMERGENCY_YAW_RATIO=${FOV_EMERGENCY_YAW_RATIO:-0.85}
K_Z_FF=${K_Z_FF:-1.0}
N_NAV_Z=${N_NAV_Z:-2.0}
MAX_VZ=${MAX_VZ:-2.0}
K_YAW=${K_YAW:-1.5}
K_YAW_D=${K_YAW_D:-0.3}
MAX_YAW_RATE=${MAX_YAW_RATE:-1.0}
YAW_ENABLE_SPEED_RATIO=${YAW_ENABLE_SPEED_RATIO:-0.4}
CONTROLLER_FOV_HORIZONTAL_LIMIT_DEG=${CONTROLLER_FOV_HORIZONTAL_LIMIT_DEG:-23.94}
CONTROLLER_FOV_VERTICAL_LIMIT_DEG=${CONTROLLER_FOV_VERTICAL_LIMIT_DEG:-18.54}
MULTISIM_LAUNCH_FILE=${MULTISIM_LAUNCH_FILE:-radar_approach_multisim.launch}
VISUAL_FORWARD_SPEED_MPS=${VISUAL_FORWARD_SPEED_MPS:-4.5}
FORWARD_SPEED_ACCEL_LIMIT_MPS2=${FORWARD_SPEED_ACCEL_LIMIT_MPS2:-0.0}
GATE_DETECTION_UNTIL_CHASE=${GATE_DETECTION_UNTIL_CHASE:-false}
START_IBVS=${START_IBVS:-true}
STOP_ON_HANDOVER=${STOP_ON_HANDOVER:-false}
DISTURBANCE_MODEL=${DISTURBANCE_MODEL:-none}
DISTURBANCE_SEED=${DISTURBANCE_SEED:-20260926}
DISTURBANCE_TIME_CONSTANT_S=${DISTURBANCE_TIME_CONSTANT_S:-2.0}
DISTURBANCE_CLIP_SIGMA=${DISTURBANCE_CLIP_SIGMA:-3.0}
INTERCEPTOR_DISTURBANCE_STD_XY_MPS=${INTERCEPTOR_DISTURBANCE_STD_XY_MPS:-0.0}
INTERCEPTOR_DISTURBANCE_STD_Z_MPS=${INTERCEPTOR_DISTURBANCE_STD_Z_MPS:-0.0}
TARGET_DISTURBANCE_STD_XY_MPS=${TARGET_DISTURBANCE_STD_XY_MPS:-0.0}
TARGET_DISTURBANCE_STD_Z_MPS=${TARGET_DISTURBANCE_STD_Z_MPS:-0.0}
VISUAL_DISTURBANCE_STD_PX=${VISUAL_DISTURBANCE_STD_PX:-0.0}
VISUAL_BIAS_RANDOM_WALK_PX_SQRTS=${VISUAL_BIAS_RANDOM_WALK_PX_SQRTS:-0.0}
VISUAL_BIAS_LIMIT_PX=${VISUAL_BIAS_LIMIT_PX:-0.0}
VISUAL_DELAY_MEAN_SEC=${VISUAL_DELAY_MEAN_SEC:-0.0}
VISUAL_DELAY_JITTER_STD_SEC=${VISUAL_DELAY_JITTER_STD_SEC:-0.0}
VISUAL_DROPOUT_PROBABILITY=${VISUAL_DROPOUT_PROBABILITY:-0.0}
GUIDANCE_DELAY_MEAN_SEC=${GUIDANCE_DELAY_MEAN_SEC:-0.0}
GUIDANCE_DELAY_JITTER_STD_SEC=${GUIDANCE_DELAY_JITTER_STD_SEC:-0.0}
GUIDANCE_DROPOUT_PROBABILITY=${GUIDANCE_DROPOUT_PROBABILITY:-0.0}
CONTROLLER_IMAGE_DELAY_SEC=${CONTROLLER_IMAGE_DELAY_SEC:-0.0}
NAV_PROXY_SEED=${NAV_PROXY_SEED:-$DISTURBANCE_SEED}
NAV_POSE_POSITION_WHITE_NOISE_STD_M=${NAV_POSE_POSITION_WHITE_NOISE_STD_M:-0.0}
NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS=${NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS:-0.0}
NAV_POSE_POSITION_BIAS_LIMIT_M=${NAV_POSE_POSITION_BIAS_LIMIT_M:-0.0}
NAV_ATTITUDE_WHITE_NOISE_STD_DEG=${NAV_ATTITUDE_WHITE_NOISE_STD_DEG:-0.0}
NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS=${NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS:-0.0}
NAV_ATTITUDE_BIAS_LIMIT_DEG=${NAV_ATTITUDE_BIAS_LIMIT_DEG:-0.0}
NAV_VELOCITY_WHITE_NOISE_STD_MPS=${NAV_VELOCITY_WHITE_NOISE_STD_MPS:-0.0}
NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS=${NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS:-0.0}
NAV_VELOCITY_BIAS_LIMIT_MPS=${NAV_VELOCITY_BIAS_LIMIT_MPS:-0.0}
NAV_GYRO_WHITE_NOISE_STD_RADPS=${NAV_GYRO_WHITE_NOISE_STD_RADPS:-0.0}
NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS=${NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS:-0.0}
NAV_GYRO_BIAS_LIMIT_RADPS=${NAV_GYRO_BIAS_LIMIT_RADPS:-0.0}
NAV_ACCEL_WHITE_NOISE_STD_MPS2=${NAV_ACCEL_WHITE_NOISE_STD_MPS2:-0.0}
NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS=${NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS:-0.0}
NAV_ACCEL_BIAS_LIMIT_MPS2=${NAV_ACCEL_BIAS_LIMIT_MPS2:-0.0}
NAV_POSE_DELAY_MEAN_SEC=${NAV_POSE_DELAY_MEAN_SEC:-0.0}
NAV_POSE_DELAY_JITTER_STD_SEC=${NAV_POSE_DELAY_JITTER_STD_SEC:-0.0}
NAV_POSE_DROPOUT_PROBABILITY=${NAV_POSE_DROPOUT_PROBABILITY:-0.0}
NAV_VELOCITY_DELAY_MEAN_SEC=${NAV_VELOCITY_DELAY_MEAN_SEC:-0.0}
NAV_VELOCITY_DELAY_JITTER_STD_SEC=${NAV_VELOCITY_DELAY_JITTER_STD_SEC:-0.0}
NAV_VELOCITY_DROPOUT_PROBABILITY=${NAV_VELOCITY_DROPOUT_PROBABILITY:-0.0}
NAV_IMU_DELAY_MEAN_SEC=${NAV_IMU_DELAY_MEAN_SEC:-0.0}
NAV_IMU_DELAY_JITTER_STD_SEC=${NAV_IMU_DELAY_JITTER_STD_SEC:-0.0}
NAV_IMU_DROPOUT_PROBABILITY=${NAV_IMU_DROPOUT_PROBABILITY:-0.0}
WIND_ENABLED=${WIND_ENABLED:-0}
WIND_VELOCITY_MEAN_MPS=${WIND_VELOCITY_MEAN_MPS:-0.0}
WIND_VELOCITY_MAX_MPS=${WIND_VELOCITY_MAX_MPS:-5.0}
WIND_VELOCITY_VARIANCE=${WIND_VELOCITY_VARIANCE:-0.0}
WIND_DIRECTION_X=${WIND_DIRECTION_X:-0.0}
WIND_DIRECTION_Y=${WIND_DIRECTION_Y:-1.0}
WIND_DIRECTION_Z=${WIND_DIRECTION_Z:-0.0}
WIND_PUBLISH_RATE_HZ=${WIND_PUBLISH_RATE_HZ:-20.0}

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source /home/toast/png_planner_ws/devel/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/devel/setup.bash"

if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null; then
  echo "PX4 or Gazebo is already running. Stop that run before starting this experiment."
  exit 2
fi

run_stamp=$(date +%Y%m%d_%H%M%S)
if [[ -n "$BATCH_ID" ]]; then
  run_dir="$RUN_ROOT/$BATCH_ID/$CASE_NAME"
else
  run_dir="$RUN_ROOT/${run_stamp}_${CASE_NAME}"
fi
mkdir -p "$run_dir" "$RUN_ROOT"
printf '%s\n' "$run_dir" > "$RUN_ROOT/latest_run.txt"
printf '%s\n' "$$" > "$RUN_ROOT/active.pid"
cp "$WS_DIR/src/Fixedcamera_Planner/src/png_planner/config/radar_approach_handover.yaml" \
  "$run_dir/radar_approach_handover.yaml"
sha256sum \
  "$WS_DIR/src/Fixedcamera_Planner/src/png_planner/src/radar_approach_handover.cpp" \
  "$WS_DIR/src/Fixedcamera_Planner/src/png_planner/src/fixed_camera_ibvs.cpp" \
  > "$run_dir/controller_source_sha256.txt"

export CASE_NAME SCENARIO_NAME TEST_GROUP ERROR_PROFILE TARGET_SPAWN_X TARGET_SPAWN_Y
export INTERCEPTOR_ALTITUDE_M TARGET_ALTITUDE_M TARGET_SPEED_MPS
export TARGET_HEADING_DEG TARGET_TURN_RATE_DEGPS TARGET_CLIMB_RATE_MPS
export RADAR_PERIOD_SEC RADAR_TIMEOUT_SEC RADAR_NOISE_STD_M
export RADAR_BOOTSTRAP_SAMPLES RADAR_MEASUREMENT_STD_M
export RADAR_PROCESS_ACCEL_STD_MPS2 RADAR_BOOTSTRAP_HUBER_DELTA_M
export RADAR_NIS_SOFT_THRESHOLD RADAR_NIS_REJECT_THRESHOLD
export FILTER_ALPHA FILTER_BETA MAX_RADAR_INNOVATION_M
export MAX_TARGET_SPEED_MPS MAX_TARGET_HORIZONTAL_SPEED_MPS
export MAX_TARGET_VERTICAL_SPEED_MPS TERMINAL_CAMERA_RANGE_M
export POSITION_GAIN_XY POSITION_GAIN_Z
export USE_BRAKING_SPEED_PROFILE APPROACH_BRAKING_DECELERATION_MPS2
export APPROACH_BRAKING_POSITION_BUFFER_M
export TARGET_VELOCITY_FEEDFORWARD_GAIN_XY TARGET_VELOCITY_FEEDFORWARD_GAIN_Z
export ENFORCE_MINIMUM_HANDOVER_ALTITUDE MINIMUM_HANDOVER_ALTITUDE_M
export CENTER_TOLERANCE_U_PX CENTER_TOLERANCE_V_PX REQUIRED_CENTERED_DETECTIONS
export HANDOVER_POSITION_TOLERANCE_M HANDOVER_YAW_TOLERANCE_DEG
export USE_LEGACY_POSE_HANDOVER_GATE HANDOVER_MIN_ESTIMATED_RANGE_M
export HANDOVER_MAX_ESTIMATED_RANGE_M MAX_HANDOVER_VELOCITY_STD_MPS
export MAX_HANDOVER_RADAR_AGE_SEC MAX_HANDOVER_COMMAND_YAW_RATE_RADPS
export MAX_HANDOVER_CLOSING_SPEED_MPS MAX_HANDOVER_OPENING_SPEED_MPS
export RADAR_PERIOD_JITTER_STD_SEC RADAR_DELAY_MEAN_SEC
export RADAR_DELAY_JITTER_STD_SEC RADAR_DROPOUT_PROBABILITY
export RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS
export RADAR_BIAS_LIMIT_XY_M RADAR_BIAS_LIMIT_Z_M
export RADAR_DROPOUT_START_SEC RADAR_DROPOUT_DURATION_SEC
export RADAR_ACQUIRE_DURATION_SEC RADAR_ACQUIRE_TIMEOUT_SEC
export RADAR_SEED CAPTURE_DISTANCE_M
export CHASE_TIMEOUT_S DETECTION_RANGE_M EXPECTED_RESULT
export RESULT_HOLD_DURATION_SEC HANDOVER_BLEND_DURATION_SEC
export FOV_BARRIER_GAIN FOV_BARRIER_MAX FOV_SLOWDOWN_START_RATIO
export FOV_MIN_FORWARD_SPEED_RATIO FOV_EMERGENCY_YAW_RATIO
export K_Z_FF N_NAV_Z MAX_VZ K_YAW K_YAW_D MAX_YAW_RATE YAW_ENABLE_SPEED_RATIO
export CONTROLLER_FOV_HORIZONTAL_LIMIT_DEG CONTROLLER_FOV_VERTICAL_LIMIT_DEG
export MULTISIM_LAUNCH_FILE
export VISUAL_FORWARD_SPEED_MPS
export FORWARD_SPEED_ACCEL_LIMIT_MPS2
export GATE_DETECTION_UNTIL_CHASE
export START_IBVS STOP_ON_HANDOVER
export DISTURBANCE_MODEL DISTURBANCE_SEED DISTURBANCE_TIME_CONSTANT_S
export DISTURBANCE_CLIP_SIGMA INTERCEPTOR_DISTURBANCE_STD_XY_MPS
export INTERCEPTOR_DISTURBANCE_STD_Z_MPS TARGET_DISTURBANCE_STD_XY_MPS
export TARGET_DISTURBANCE_STD_Z_MPS VISUAL_DISTURBANCE_STD_PX
export VISUAL_BIAS_RANDOM_WALK_PX_SQRTS VISUAL_BIAS_LIMIT_PX
export VISUAL_DELAY_MEAN_SEC VISUAL_DELAY_JITTER_STD_SEC
export VISUAL_DROPOUT_PROBABILITY GUIDANCE_DELAY_MEAN_SEC
export GUIDANCE_DELAY_JITTER_STD_SEC GUIDANCE_DROPOUT_PROBABILITY
export CONTROLLER_IMAGE_DELAY_SEC
export NAV_PROXY_SEED NAV_POSE_POSITION_WHITE_NOISE_STD_M
export NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS NAV_POSE_POSITION_BIAS_LIMIT_M
export NAV_ATTITUDE_WHITE_NOISE_STD_DEG NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS
export NAV_ATTITUDE_BIAS_LIMIT_DEG NAV_VELOCITY_WHITE_NOISE_STD_MPS
export NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS NAV_VELOCITY_BIAS_LIMIT_MPS
export NAV_GYRO_WHITE_NOISE_STD_RADPS NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS
export NAV_GYRO_BIAS_LIMIT_RADPS NAV_ACCEL_WHITE_NOISE_STD_MPS2
export NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS NAV_ACCEL_BIAS_LIMIT_MPS2
export NAV_POSE_DELAY_MEAN_SEC NAV_POSE_DELAY_JITTER_STD_SEC
export NAV_POSE_DROPOUT_PROBABILITY NAV_VELOCITY_DELAY_MEAN_SEC
export NAV_VELOCITY_DELAY_JITTER_STD_SEC NAV_VELOCITY_DROPOUT_PROBABILITY
export NAV_IMU_DELAY_MEAN_SEC NAV_IMU_DELAY_JITTER_STD_SEC
export NAV_IMU_DROPOUT_PROBABILITY
export WIND_ENABLED WIND_VELOCITY_MEAN_MPS WIND_VELOCITY_MAX_MPS
export WIND_VELOCITY_VARIANCE WIND_DIRECTION_X WIND_DIRECTION_Y
export WIND_DIRECTION_Z WIND_PUBLISH_RATE_HZ
python3 - "$run_dir" <<'PY'
import json
import os
import sys

keys = [
    "CASE_NAME", "SCENARIO_NAME", "TEST_GROUP", "ERROR_PROFILE",
    "TARGET_SPAWN_X", "TARGET_SPAWN_Y",
    "INTERCEPTOR_ALTITUDE_M", "TARGET_ALTITUDE_M", "TARGET_SPEED_MPS",
    "TARGET_HEADING_DEG", "TARGET_TURN_RATE_DEGPS", "TARGET_CLIMB_RATE_MPS",
    "RADAR_PERIOD_SEC", "RADAR_TIMEOUT_SEC", "RADAR_NOISE_STD_M",
    "RADAR_BOOTSTRAP_SAMPLES", "RADAR_MEASUREMENT_STD_M",
    "RADAR_PROCESS_ACCEL_STD_MPS2", "RADAR_BOOTSTRAP_HUBER_DELTA_M",
    "RADAR_NIS_SOFT_THRESHOLD", "RADAR_NIS_REJECT_THRESHOLD",
    "FILTER_ALPHA", "FILTER_BETA",
    "MAX_RADAR_INNOVATION_M", "MAX_TARGET_SPEED_MPS",
    "MAX_TARGET_HORIZONTAL_SPEED_MPS", "MAX_TARGET_VERTICAL_SPEED_MPS",
    "TERMINAL_CAMERA_RANGE_M",
    "POSITION_GAIN_XY", "POSITION_GAIN_Z",
    "USE_BRAKING_SPEED_PROFILE", "APPROACH_BRAKING_DECELERATION_MPS2",
    "APPROACH_BRAKING_POSITION_BUFFER_M",
    "TARGET_VELOCITY_FEEDFORWARD_GAIN_XY",
    "TARGET_VELOCITY_FEEDFORWARD_GAIN_Z",
    "ENFORCE_MINIMUM_HANDOVER_ALTITUDE", "MINIMUM_HANDOVER_ALTITUDE_M",
    "CENTER_TOLERANCE_U_PX", "CENTER_TOLERANCE_V_PX",
    "REQUIRED_CENTERED_DETECTIONS", "HANDOVER_POSITION_TOLERANCE_M",
    "HANDOVER_YAW_TOLERANCE_DEG", "USE_LEGACY_POSE_HANDOVER_GATE",
    "HANDOVER_MIN_ESTIMATED_RANGE_M", "HANDOVER_MAX_ESTIMATED_RANGE_M",
    "MAX_HANDOVER_VELOCITY_STD_MPS", "MAX_HANDOVER_RADAR_AGE_SEC",
    "MAX_HANDOVER_COMMAND_YAW_RATE_RADPS", "MAX_HANDOVER_CLOSING_SPEED_MPS",
    "MAX_HANDOVER_OPENING_SPEED_MPS",
    "RADAR_PERIOD_JITTER_STD_SEC", "RADAR_DELAY_MEAN_SEC",
    "RADAR_DELAY_JITTER_STD_SEC", "RADAR_DROPOUT_PROBABILITY",
    "RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS",
    "RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS", "RADAR_BIAS_LIMIT_XY_M",
    "RADAR_BIAS_LIMIT_Z_M",
    "RADAR_DROPOUT_START_SEC", "RADAR_DROPOUT_DURATION_SEC",
    "RADAR_ACQUIRE_DURATION_SEC", "RADAR_ACQUIRE_TIMEOUT_SEC",
    "RADAR_SEED", "CAPTURE_DISTANCE_M",
    "CHASE_TIMEOUT_S", "DETECTION_RANGE_M", "EXPECTED_RESULT",
    "RESULT_HOLD_DURATION_SEC", "HANDOVER_BLEND_DURATION_SEC",
    "FOV_BARRIER_GAIN", "FOV_BARRIER_MAX", "FOV_SLOWDOWN_START_RATIO",
    "FOV_MIN_FORWARD_SPEED_RATIO", "FOV_EMERGENCY_YAW_RATIO",
    "K_Z_FF", "N_NAV_Z", "MAX_VZ", "K_YAW", "K_YAW_D", "MAX_YAW_RATE",
    "YAW_ENABLE_SPEED_RATIO",
    "CONTROLLER_FOV_HORIZONTAL_LIMIT_DEG", "CONTROLLER_FOV_VERTICAL_LIMIT_DEG",
    "MULTISIM_LAUNCH_FILE",
    "VISUAL_FORWARD_SPEED_MPS",
    "FORWARD_SPEED_ACCEL_LIMIT_MPS2",
    "GATE_DETECTION_UNTIL_CHASE",
    "START_IBVS", "STOP_ON_HANDOVER",
    "DISTURBANCE_MODEL", "DISTURBANCE_SEED", "DISTURBANCE_TIME_CONSTANT_S",
    "DISTURBANCE_CLIP_SIGMA", "INTERCEPTOR_DISTURBANCE_STD_XY_MPS",
    "INTERCEPTOR_DISTURBANCE_STD_Z_MPS", "TARGET_DISTURBANCE_STD_XY_MPS",
    "TARGET_DISTURBANCE_STD_Z_MPS", "VISUAL_DISTURBANCE_STD_PX",
    "VISUAL_BIAS_RANDOM_WALK_PX_SQRTS", "VISUAL_BIAS_LIMIT_PX",
    "VISUAL_DELAY_MEAN_SEC", "VISUAL_DELAY_JITTER_STD_SEC",
    "VISUAL_DROPOUT_PROBABILITY", "GUIDANCE_DELAY_MEAN_SEC",
    "GUIDANCE_DELAY_JITTER_STD_SEC", "GUIDANCE_DROPOUT_PROBABILITY",
    "CONTROLLER_IMAGE_DELAY_SEC",
    "NAV_PROXY_SEED", "NAV_POSE_POSITION_WHITE_NOISE_STD_M",
    "NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS",
    "NAV_POSE_POSITION_BIAS_LIMIT_M", "NAV_ATTITUDE_WHITE_NOISE_STD_DEG",
    "NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS", "NAV_ATTITUDE_BIAS_LIMIT_DEG",
    "NAV_VELOCITY_WHITE_NOISE_STD_MPS",
    "NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS", "NAV_VELOCITY_BIAS_LIMIT_MPS",
    "NAV_GYRO_WHITE_NOISE_STD_RADPS", "NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS",
    "NAV_GYRO_BIAS_LIMIT_RADPS", "NAV_ACCEL_WHITE_NOISE_STD_MPS2",
    "NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS", "NAV_ACCEL_BIAS_LIMIT_MPS2",
    "NAV_POSE_DELAY_MEAN_SEC", "NAV_POSE_DELAY_JITTER_STD_SEC",
    "NAV_POSE_DROPOUT_PROBABILITY", "NAV_VELOCITY_DELAY_MEAN_SEC",
    "NAV_VELOCITY_DELAY_JITTER_STD_SEC", "NAV_VELOCITY_DROPOUT_PROBABILITY",
    "NAV_IMU_DELAY_MEAN_SEC", "NAV_IMU_DELAY_JITTER_STD_SEC",
    "NAV_IMU_DROPOUT_PROBABILITY",
    "WIND_ENABLED", "WIND_VELOCITY_MEAN_MPS", "WIND_VELOCITY_MAX_MPS",
    "WIND_VELOCITY_VARIANCE", "WIND_DIRECTION_X", "WIND_DIRECTION_Y",
    "WIND_DIRECTION_Z", "WIND_PUBLISH_RATE_HZ",
]
params = {}
for key in keys:
    value = os.environ[key]
    output_key = key.lower()
    if key == "CASE_NAME":
        output_key = "case_name"
    elif key == "SCENARIO_NAME":
        output_key = "scenario_name"
    elif key == "EXPECTED_RESULT":
        output_key = "expected_result"
    else:
        try:
            value = float(value)
        except ValueError:
            pass
    params[output_key] = value
with open(os.path.join(sys.argv[1], "case_params.json"), "w", encoding="utf-8") as stream:
    json.dump(params, stream, indent=2, ensure_ascii=False)
PY

export DISPLAY="$DISPLAY_VALUE"
export XAUTHORITY="$XAUTHORITY_FILE"
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export ROS_MASTER_URI=http://127.0.0.1:11311
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export PX4_SIM_MODEL=gazebo-classic_iris

child_pids=()
px4_pids=()
rosbag_pid=""
video_pid=""

stop_video_recording() {
  if [[ -n "$video_pid" ]] && kill -0 "$video_pid" 2>/dev/null; then
    kill -INT "$video_pid" 2>/dev/null || true
    wait "$video_pid" 2>/dev/null || true
  fi
  video_pid=""
}

cleanup() {
  set +e
  stop_video_recording
  if [[ -n "$rosbag_pid" ]] && kill -0 "$rosbag_pid" 2>/dev/null; then
    kill -INT "$rosbag_pid" 2>/dev/null || true
    wait "$rosbag_pid" 2>/dev/null || true
  fi
  for pid in "${child_pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in "${px4_pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${px4_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  rm -f "$RUN_ROOT/active.pid"
}
trap cleanup INT TERM EXIT

roscore > "$run_dir/roscore.log" 2>&1 &
child_pids+=("$!")
for _ in $(seq 1 40); do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

# shellcheck disable=SC1090
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"

base_world="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world"
simulation_world="$run_dir/intercept_sim.world"
wind_enable_args=()
if [[ "$WIND_ENABLED" == "1" ]]; then
  wind_enable_args+=(--enabled)
fi
demo_gui_args=()
if [[ "$DEMO_GUI" == "1" ]]; then
  demo_gui_args+=(--demo-gui --demo-camera-model iris_1 \
    --demo-camera-xyz -80 -50 60)
fi
if ! python3 "$SCRIPT_DIR/generate_wind_world.py" \
  "$base_world" "$simulation_world" "${wind_enable_args[@]}" \
  "${demo_gui_args[@]}" \
  --mean-mps "$WIND_VELOCITY_MEAN_MPS" \
  --max-mps "$WIND_VELOCITY_MAX_MPS" \
  --variance "$WIND_VELOCITY_VARIANCE" \
  --direction "$WIND_DIRECTION_X" "$WIND_DIRECTION_Y" "$WIND_DIRECTION_Z" \
  --publish-rate-hz "$WIND_PUBLISH_RATE_HZ"; then
  echo "Failed to generate the per-run Gazebo wind world."
  exit 3
fi

gzserver "$simulation_world" \
  --verbose -s libgazebo_ros_api_plugin.so > "$run_dir/gzserver.log" 2>&1 &
child_pids+=("$!")
sleep 5

spawn_vehicle() {
  local instance=$1
  local spawn_x=$2
  local spawn_y=$3
  local working_dir="$PX4_DIR/build/px4_sitl_default/rootfs/$instance"
  local generated_sdf="$run_dir/iris_$instance.sdf"
  mkdir -p "$working_dir"

  (
    cd "$working_dir" || exit 1
    exec "$PX4_DIR/build/px4_sitl_default/bin/px4" -i "$instance" \
      -d "$PX4_DIR/build/px4_sitl_default/etc" \
      > "$run_dir/px4_$instance.log" 2> "$run_dir/px4_${instance}_error.log"
  ) &
  px4_pids+=("$!")

  python3 "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py" \
    "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja" \
    "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
    --mavlink_tcp_port "$((4560 + instance))" \
    --mavlink_udp_port "$((14560 + instance))" \
    --mavlink_id "$((1 + instance))" \
    --gst_udp_port "$((5600 + instance))" \
    --video_uri "$((5600 + instance))" \
    --mavlink_cam_udp_port "$((14530 + instance))" \
    --output-file "$generated_sdf"

  if [[ "$DEMO_VISUAL_MARKERS" == "1" ]]; then
    if [[ "$instance" == "1" ]]; then
      python3 "$SCRIPT_DIR/decorate_demo_model.py" "$generated_sdf" \
        --rgba 0.05 0.30 1.0 1.0
    else
      python3 "$SCRIPT_DIR/decorate_demo_model.py" "$generated_sdf" \
        --rgba 1.0 0.08 0.03 1.0
    fi
  fi

  gz model --spawn-file="$generated_sdf" --model-name="iris_$instance" \
    -x "$spawn_x" -y "$spawn_y" -z 0.83
}

spawn_vehicle 1 0.0 0.0
spawn_vehicle 2 "$TARGET_SPAWN_X" "$TARGET_SPAWN_Y"
sleep 4

if [[ "$SHOW_GUI" == "1" ]]; then
  gzclient > "$run_dir/gzclient.log" 2>&1 &
  child_pids+=("$!")
  sleep 4
  # Gazebo Classic keeps this helper alive after applying the follow target on
  # some builds.  Bound it so display setup cannot block the actual run.
  timeout 3s gz camera -c user_camera -f iris_1 \
    >> "$run_dir/gzclient.log" 2>&1 || true
fi

if [[ "$RECORD_VIDEO" == "1" ]]; then
  if [[ "$SHOW_GUI" != "1" ]]; then
    echo "RECORD_VIDEO=1 requires SHOW_GUI=1."
    exit 4
  fi
  python3 "$SCRIPT_DIR/record_video_clock.py" "$run_dir" \
    > "$run_dir/video_clock.log" 2>&1 &
  child_pids+=("$!")
  for _ in $(seq 1 40); do
    [[ -s "$run_dir/video_clock.csv" ]] && break
    sleep 0.25
  done
  if [[ ! -s "$run_dir/video_clock.csv" ]]; then
    echo "Could not record /clock for video synchronization." >&2
    exit 4
  fi
  date +%s.%N > "$run_dir/video_start_epoch_s.txt"
  gst-launch-1.0 -e \
    ximagesrc display-name="$DISPLAY" use-damage=0 show-pointer=false \
    ! video/x-raw,framerate="${VIDEO_FRAMERATE}/1" \
    ! videoscale \
    ! video/x-raw,width=1920,height=1080 \
    ! videoconvert \
    ! x264enc speed-preset=ultrafast tune=zerolatency \
        bitrate="$VIDEO_BITRATE_KBPS" key-int-max="$VIDEO_FRAMERATE" \
    ! h264parse \
    ! mp4mux \
    ! filesink location="$run_dir/gazebo_raw.mp4" \
    > "$run_dir/video_record.log" 2>&1 &
  video_pid=$!
  sleep 1
fi

rosbag record -O "$run_dir/experiment.bag" \
  /interceptor/mavros/state \
  /interceptor/mavros/local_position/pose \
  /interceptor/mavros/local_position/velocity_local \
  /interceptor/mavros/imu/data \
  /fixed_camera_sim/nav/pose \
  /fixed_camera_sim/nav/velocity \
  /fixed_camera_sim/nav/imu \
  /target/mavros/state \
  /target/mavros/local_position/pose \
  /target/mavros/local_position/velocity_local \
  /interceptor/mavros/setpoint_velocity/cmd_vel \
  /target/mavros/setpoint_velocity/cmd_vel \
  /radar/target/position \
  /radar_approach/state \
  /radar_approach/handover_complete \
  /radar_approach/predicted_target \
  /radar_approach/handover_pose \
  /radar_approach/estimated_target_velocity \
  /fixed_camera_sim/ibvs_candidate \
  /fixed_camera_sim/guidance_cmd \
  /object_kcf \
  /fixed_camera_ibvs/los_world \
  /fixed_camera_ibvs/fov_state \
  /fixed_camera_ibvs/nominal_pn_cmd \
  /fixed_camera_ibvs/fov_correction \
  /fixed_camera_sim/interceptor_velocity_disturbance \
  /fixed_camera_sim/target_velocity_disturbance \
  /fixed_camera_sim/visual_disturbance \
  /fixed_camera_sim/radar_bias \
  /fixed_camera_sim/visual_bias \
  /fixed_camera_sim/delay_state \
  /fixed_camera_sim/status \
  /fixed_camera_sim/distance \
  /fixed_camera_sim/fov_ratio \
  > "$run_dir/rosbag.log" 2>&1 &
rosbag_pid=$!

if [[ "$SHOW_RQT" == "1" ]]; then
  rqt_image_view /fixed_camera_sim/image > "$run_dir/rqt_image_view.log" 2>&1 &
  child_pids+=("$!")
  rqt_plot /fixed_camera_sim/distance/data /fixed_camera_sim/fov_ratio/data \
    > "$run_dir/rqt_plot.log" 2>&1 &
  child_pids+=("$!")
fi

set +e
timeout --signal=TERM --kill-after=10s "${RUN_WALL_TIMEOUT_S}s" \
  roslaunch png_planner "$MULTISIM_LAUNCH_FILE" \
    results_dir:="$run_dir" \
    target_speed_mps:="$TARGET_SPEED_MPS" \
    target_heading_deg:="$TARGET_HEADING_DEG" \
    target_turn_rate_degps:="$TARGET_TURN_RATE_DEGPS" \
    target_climb_rate_mps:="$TARGET_CLIMB_RATE_MPS" \
    interceptor_altitude_m:="$INTERCEPTOR_ALTITUDE_M" \
    target_altitude_m:="$TARGET_ALTITUDE_M" \
    capture_distance_m:="$CAPTURE_DISTANCE_M" \
    chase_timeout_s:="$CHASE_TIMEOUT_S" \
    result_hold_duration_sec:="$RESULT_HOLD_DURATION_SEC" \
    handover_blend_duration_sec:="$HANDOVER_BLEND_DURATION_SEC" \
    radar_period_sec:="$RADAR_PERIOD_SEC" \
    radar_bootstrap_samples:="$RADAR_BOOTSTRAP_SAMPLES" \
    radar_measurement_std_m:="$RADAR_MEASUREMENT_STD_M" \
    radar_process_accel_std_mps2:="$RADAR_PROCESS_ACCEL_STD_MPS2" \
    radar_bootstrap_huber_delta_m:="$RADAR_BOOTSTRAP_HUBER_DELTA_M" \
    radar_nis_soft_threshold:="$RADAR_NIS_SOFT_THRESHOLD" \
    radar_nis_reject_threshold:="$RADAR_NIS_REJECT_THRESHOLD" \
    filter_alpha:="$FILTER_ALPHA" \
    filter_beta:="$FILTER_BETA" \
    max_radar_innovation_m:="$MAX_RADAR_INNOVATION_M" \
    max_target_speed_mps:="$MAX_TARGET_SPEED_MPS" \
    max_target_horizontal_speed_mps:="$MAX_TARGET_HORIZONTAL_SPEED_MPS" \
    max_target_vertical_speed_mps:="$MAX_TARGET_VERTICAL_SPEED_MPS" \
    terminal_camera_range_m:="$TERMINAL_CAMERA_RANGE_M" \
    position_gain_xy:="$POSITION_GAIN_XY" \
    position_gain_z:="$POSITION_GAIN_Z" \
    use_braking_speed_profile:="$USE_BRAKING_SPEED_PROFILE" \
    approach_braking_deceleration_mps2:="$APPROACH_BRAKING_DECELERATION_MPS2" \
    approach_braking_position_buffer_m:="$APPROACH_BRAKING_POSITION_BUFFER_M" \
    target_velocity_feedforward_gain_xy:="$TARGET_VELOCITY_FEEDFORWARD_GAIN_XY" \
    target_velocity_feedforward_gain_z:="$TARGET_VELOCITY_FEEDFORWARD_GAIN_Z" \
    enforce_minimum_handover_altitude:="$ENFORCE_MINIMUM_HANDOVER_ALTITUDE" \
    minimum_handover_altitude_m:="$MINIMUM_HANDOVER_ALTITUDE_M" \
    center_tolerance_u_px:="$CENTER_TOLERANCE_U_PX" \
    center_tolerance_v_px:="$CENTER_TOLERANCE_V_PX" \
    required_centered_detections:="$REQUIRED_CENTERED_DETECTIONS" \
    handover_position_tolerance_m:="$HANDOVER_POSITION_TOLERANCE_M" \
    handover_yaw_tolerance_deg:="$HANDOVER_YAW_TOLERANCE_DEG" \
    use_legacy_pose_handover_gate:="$USE_LEGACY_POSE_HANDOVER_GATE" \
    handover_min_estimated_range_m:="$HANDOVER_MIN_ESTIMATED_RANGE_M" \
    handover_max_estimated_range_m:="$HANDOVER_MAX_ESTIMATED_RANGE_M" \
    max_handover_velocity_std_mps:="$MAX_HANDOVER_VELOCITY_STD_MPS" \
    max_handover_radar_age_sec:="$MAX_HANDOVER_RADAR_AGE_SEC" \
    max_handover_command_yaw_rate_radps:="$MAX_HANDOVER_COMMAND_YAW_RATE_RADPS" \
    max_handover_closing_speed_mps:="$MAX_HANDOVER_CLOSING_SPEED_MPS" \
    max_handover_opening_speed_mps:="$MAX_HANDOVER_OPENING_SPEED_MPS" \
    radar_period_jitter_std_sec:="$RADAR_PERIOD_JITTER_STD_SEC" \
    radar_timeout_sec:="$RADAR_TIMEOUT_SEC" \
    radar_noise_std_m:="$RADAR_NOISE_STD_M" \
    radar_delay_mean_sec:="$RADAR_DELAY_MEAN_SEC" \
    radar_delay_jitter_std_sec:="$RADAR_DELAY_JITTER_STD_SEC" \
    radar_dropout_probability:="$RADAR_DROPOUT_PROBABILITY" \
    radar_bias_random_walk_xy_m_sqrts:="$RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS" \
    radar_bias_random_walk_z_m_sqrts:="$RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS" \
    radar_bias_limit_xy_m:="$RADAR_BIAS_LIMIT_XY_M" \
    radar_bias_limit_z_m:="$RADAR_BIAS_LIMIT_Z_M" \
    radar_dropout_start_sec:="$RADAR_DROPOUT_START_SEC" \
    radar_dropout_duration_sec:="$RADAR_DROPOUT_DURATION_SEC" \
    radar_acquire_duration_sec:="$RADAR_ACQUIRE_DURATION_SEC" \
    radar_acquire_timeout_sec:="$RADAR_ACQUIRE_TIMEOUT_SEC" \
    radar_seed:="$RADAR_SEED" \
    detection_range_m:="$DETECTION_RANGE_M" \
    fov_barrier_gain:="$FOV_BARRIER_GAIN" \
    fov_barrier_max:="$FOV_BARRIER_MAX" \
    fov_slowdown_start_ratio:="$FOV_SLOWDOWN_START_RATIO" \
    fov_min_forward_speed_ratio:="$FOV_MIN_FORWARD_SPEED_RATIO" \
    fov_emergency_yaw_ratio:="$FOV_EMERGENCY_YAW_RATIO" \
    k_z_ff:="$K_Z_FF" \
    N_nav_z:="$N_NAV_Z" \
    max_vz:="$MAX_VZ" \
    k_yaw:="$K_YAW" \
    k_yaw_d:="$K_YAW_D" \
    max_yaw_rate:="$MAX_YAW_RATE" \
    yaw_enable_speed_ratio:="$YAW_ENABLE_SPEED_RATIO" \
    controller_fov_horizontal_limit_deg:="$CONTROLLER_FOV_HORIZONTAL_LIMIT_DEG" \
    controller_fov_vertical_limit_deg:="$CONTROLLER_FOV_VERTICAL_LIMIT_DEG" \
    visual_forward_speed_mps:="$VISUAL_FORWARD_SPEED_MPS" \
    forward_speed_accel_limit_mps2:="$FORWARD_SPEED_ACCEL_LIMIT_MPS2" \
    gate_detection_until_chase:="$GATE_DETECTION_UNTIL_CHASE" \
    start_ibvs:="$START_IBVS" \
    stop_on_handover:="$STOP_ON_HANDOVER" \
    disturbance_model:="$DISTURBANCE_MODEL" \
    disturbance_seed:="$DISTURBANCE_SEED" \
    disturbance_time_constant_s:="$DISTURBANCE_TIME_CONSTANT_S" \
    disturbance_clip_sigma:="$DISTURBANCE_CLIP_SIGMA" \
    interceptor_disturbance_std_xy_mps:="$INTERCEPTOR_DISTURBANCE_STD_XY_MPS" \
    interceptor_disturbance_std_z_mps:="$INTERCEPTOR_DISTURBANCE_STD_Z_MPS" \
    target_disturbance_std_xy_mps:="$TARGET_DISTURBANCE_STD_XY_MPS" \
    target_disturbance_std_z_mps:="$TARGET_DISTURBANCE_STD_Z_MPS" \
    visual_disturbance_std_px:="$VISUAL_DISTURBANCE_STD_PX" \
    visual_bias_random_walk_px_sqrts:="$VISUAL_BIAS_RANDOM_WALK_PX_SQRTS" \
    visual_bias_limit_px:="$VISUAL_BIAS_LIMIT_PX" \
    visual_delay_mean_sec:="$VISUAL_DELAY_MEAN_SEC" \
    visual_delay_jitter_std_sec:="$VISUAL_DELAY_JITTER_STD_SEC" \
    visual_dropout_probability:="$VISUAL_DROPOUT_PROBABILITY" \
    guidance_delay_mean_sec:="$GUIDANCE_DELAY_MEAN_SEC" \
    guidance_delay_jitter_std_sec:="$GUIDANCE_DELAY_JITTER_STD_SEC" \
    guidance_dropout_probability:="$GUIDANCE_DROPOUT_PROBABILITY" \
    controller_image_delay_sec:="$CONTROLLER_IMAGE_DELAY_SEC" \
    nav_proxy_seed:="$NAV_PROXY_SEED" \
    nav_pose_position_white_noise_std_m:="$NAV_POSE_POSITION_WHITE_NOISE_STD_M" \
    nav_pose_position_bias_random_walk_m_sqrts:="$NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS" \
    nav_pose_position_bias_limit_m:="$NAV_POSE_POSITION_BIAS_LIMIT_M" \
    nav_attitude_white_noise_std_deg:="$NAV_ATTITUDE_WHITE_NOISE_STD_DEG" \
    nav_attitude_bias_random_walk_deg_sqrts:="$NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS" \
    nav_attitude_bias_limit_deg:="$NAV_ATTITUDE_BIAS_LIMIT_DEG" \
    nav_velocity_white_noise_std_mps:="$NAV_VELOCITY_WHITE_NOISE_STD_MPS" \
    nav_velocity_bias_random_walk_mps_sqrts:="$NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS" \
    nav_velocity_bias_limit_mps:="$NAV_VELOCITY_BIAS_LIMIT_MPS" \
    nav_gyro_white_noise_std_radps:="$NAV_GYRO_WHITE_NOISE_STD_RADPS" \
    nav_gyro_bias_random_walk_radps_sqrts:="$NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS" \
    nav_gyro_bias_limit_radps:="$NAV_GYRO_BIAS_LIMIT_RADPS" \
    nav_accel_white_noise_std_mps2:="$NAV_ACCEL_WHITE_NOISE_STD_MPS2" \
    nav_accel_bias_random_walk_mps2_sqrts:="$NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS" \
    nav_accel_bias_limit_mps2:="$NAV_ACCEL_BIAS_LIMIT_MPS2" \
    nav_pose_delay_mean_sec:="$NAV_POSE_DELAY_MEAN_SEC" \
    nav_pose_delay_jitter_std_sec:="$NAV_POSE_DELAY_JITTER_STD_SEC" \
    nav_pose_dropout_probability:="$NAV_POSE_DROPOUT_PROBABILITY" \
    nav_velocity_delay_mean_sec:="$NAV_VELOCITY_DELAY_MEAN_SEC" \
    nav_velocity_delay_jitter_std_sec:="$NAV_VELOCITY_DELAY_JITTER_STD_SEC" \
    nav_velocity_dropout_probability:="$NAV_VELOCITY_DROPOUT_PROBABILITY" \
    nav_imu_delay_mean_sec:="$NAV_IMU_DELAY_MEAN_SEC" \
    nav_imu_delay_jitter_std_sec:="$NAV_IMU_DELAY_JITTER_STD_SEC" \
    nav_imu_dropout_probability:="$NAV_IMU_DROPOUT_PROBABILITY" \
    > "$run_dir/roslaunch.log" 2>&1
launch_status=$?
set -e
printf '%s\n' "$launch_status" > "$run_dir/roslaunch_exit_code.txt"

stop_video_recording

if [[ -n "$rosbag_pid" ]] && kill -0 "$rosbag_pid" 2>/dev/null; then
  kill -INT "$rosbag_pid" 2>/dev/null || true
  wait "$rosbag_pid" 2>/dev/null || true
fi
rosbag_pid=""

if [[ -s "$run_dir/experiment.csv" ]]; then
  python3 "$WS_DIR/src/Fixedcamera_Planner/tools/simulation/analyze_radar_multisim_result.py" \
    "$run_dir" | tee "$run_dir/analyzer.log"
else
  echo "experiment.csv is missing; see roslaunch.log" | tee "$run_dir/analyzer.log"
fi

echo "Radar approach multisim run directory: $run_dir"
exit 0
