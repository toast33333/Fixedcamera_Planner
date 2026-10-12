#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODE=${1:-}

case "$MODE" in
  full)
    export ENABLE_HORIZONTAL_CONTROL=true
    export ENABLE_HEIGHT_CONTROL=true
    export CASE_NAME=${CASE_NAME:-intercept_9mps_full}
    ;;
  horizontal_only)
    export ENABLE_HORIZONTAL_CONTROL=true
    export ENABLE_HEIGHT_CONTROL=false
    export CASE_NAME=${CASE_NAME:-intercept_9mps_horizontal_only}
    ;;
  vertical_only)
    export ENABLE_HORIZONTAL_CONTROL=false
    export ENABLE_HEIGHT_CONTROL=true
    export CASE_NAME=${CASE_NAME:-intercept_9mps_vertical_only}
    ;;
  *)
    echo "Usage: $0 {full|horizontal_only|vertical_only}" >&2
    exit 2
    ;;
esac

export SCENARIO_NAME=${SCENARIO_NAME:-direct_visual_axis_comparison}
export TEST_GROUP=${TEST_GROUP:-axis_ablation_9mps}
export ERROR_PROFILE=${ERROR_PROFILE:-clean}
export MULTISIM_LAUNCH_FILE=fov_tuning_visual_multisim.launch
export GATE_DETECTION_UNTIL_CHASE=true

# All three modes use exactly the same initial geometry.  The target begins in
# the physical fixed-camera FOV and flies horizontally at a constant 9 m/s.
export TARGET_SPAWN_X=${TARGET_SPAWN_X:-40.0}
export TARGET_SPAWN_Y=${TARGET_SPAWN_Y:-1.5}
export INTERCEPTOR_ALTITUDE_M=${INTERCEPTOR_ALTITUDE_M:-4.0}
export TARGET_ALTITUDE_M=${TARGET_ALTITUDE_M:-7.0}
export TARGET_SPEED_MPS=${TARGET_SPEED_MPS:-9.0}
export TARGET_HEADING_DEG=${TARGET_HEADING_DEG:-0.0}
export TARGET_TURN_RATE_DEGPS=${TARGET_TURN_RATE_DEGPS:-0.0}
export TARGET_CLIMB_RATE_MPS=${TARGET_CLIMB_RATE_MPS:-0.0}

export DETECTION_RANGE_M=${DETECTION_RANGE_M:-80.0}
export CAPTURE_DISTANCE_M=${CAPTURE_DISTANCE_M:-5.0}
export CHASE_TIMEOUT_S=${CHASE_TIMEOUT_S:-45.0}
export RESULT_HOLD_DURATION_SEC=${RESULT_HOLD_DURATION_SEC:-4.0}
export VISUAL_FORWARD_SPEED_MPS=${VISUAL_FORWARD_SPEED_MPS:-11.5}
export FORWARD_SPEED_ACCEL_LIMIT_MPS2=${FORWARD_SPEED_ACCEL_LIMIT_MPS2:-2.0}
export YAW_ENABLE_SPEED_RATIO=${YAW_ENABLE_SPEED_RATIO:-0.1}
export MAX_VZ=${MAX_VZ:-1.6}
export EXPECTED_RESULT=${EXPECTED_RESULT:-success}
export RUN_WALL_TIMEOUT_S=${RUN_WALL_TIMEOUT_S:-105}

exec "$SCRIPT_DIR/start_radar_approach_multisim.sh"
