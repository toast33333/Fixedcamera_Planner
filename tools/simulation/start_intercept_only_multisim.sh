#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Situation name requested for the direct visual-interception experiment.
export SCENARIO_NAME=${SCENARIO_NAME:-情况_只拦截}
export CASE_NAME=${CASE_NAME:-intercept_only_nominal}
export MULTISIM_LAUNCH_FILE=fov_tuning_visual_multisim.launch
export GATE_DETECTION_UNTIL_CHASE=true

# The target is already inside the physical fixed-camera FOV when CHASE starts.
export TARGET_SPAWN_X=${TARGET_SPAWN_X:-40.0}
export TARGET_SPAWN_Y=${TARGET_SPAWN_Y:-6.0}
export INTERCEPTOR_ALTITUDE_M=${INTERCEPTOR_ALTITUDE_M:-4.0}
export TARGET_ALTITUDE_M=${TARGET_ALTITUDE_M:-9.0}
export TARGET_SPEED_MPS=${TARGET_SPEED_MPS:-4.0}
export TARGET_HEADING_DEG=${TARGET_HEADING_DEG:-15.0}
export TARGET_TURN_RATE_DEGPS=${TARGET_TURN_RATE_DEGPS:-0.25}
export TARGET_CLIMB_RATE_MPS=${TARGET_CLIMB_RATE_MPS:-0.3}
export DETECTION_RANGE_M=${DETECTION_RANGE_M:-60.0}
export CAPTURE_DISTANCE_M=${CAPTURE_DISTANCE_M:-5.0}
export CHASE_TIMEOUT_S=${CHASE_TIMEOUT_S:-35.0}
export VISUAL_FORWARD_SPEED_MPS=${VISUAL_FORWARD_SPEED_MPS:-8.0}
export FORWARD_SPEED_ACCEL_LIMIT_MPS2=${FORWARD_SPEED_ACCEL_LIMIT_MPS2:-1.5}
export YAW_ENABLE_SPEED_RATIO=${YAW_ENABLE_SPEED_RATIO:-0.1}
export MAX_VZ=${MAX_VZ:-1.6}
export EXPECTED_RESULT=${EXPECTED_RESULT:-success}
export SHOW_GUI=${SHOW_GUI:-0}
export RUN_WALL_TIMEOUT_S=${RUN_WALL_TIMEOUT_S:-100}

exec "$SCRIPT_DIR/start_radar_approach_multisim.sh"
