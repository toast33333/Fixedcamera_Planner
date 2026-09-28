#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CASES_FILE=${1:-"$SCRIPT_DIR/fov_tuning_coarse_cases.csv"}
RUN_ROOT=/home/toast/fixedcamera_radar_multisim_results
BATCH_ID=${BATCH_ID:-$(date +%Y%m%d_%H%M%S)_fov_tuning}
export BATCH_ID

batch_dir="$RUN_ROOT/$BATCH_ID"
mkdir -p "$batch_dir"
cp "$CASES_FILE" "$batch_dir/cases.csv"

while IFS=, read -r case_name target_spawn_x target_spawn_y target_altitude_m \
    target_speed_mps target_heading_deg target_turn_rate_degps target_climb_rate_mps \
    radar_period_sec radar_timeout_sec radar_noise_std_m radar_dropout_start_sec \
    radar_dropout_duration_sec radar_acquire_duration_sec chase_timeout_s expected_result \
    fov_barrier_gain fov_barrier_max fov_slowdown_start_ratio \
    fov_min_forward_speed_ratio fov_emergency_yaw_ratio k_z_ff n_nav_z max_vz \
    k_yaw k_yaw_d max_yaw_rate controller_fov_horizontal_limit_deg \
    controller_fov_vertical_limit_deg; do
  [[ -z "$case_name" ]] && continue
  export CASE_NAME="$case_name"
  export TARGET_SPAWN_X="$target_spawn_x"
  export TARGET_SPAWN_Y="$target_spawn_y"
  export TARGET_ALTITUDE_M="$target_altitude_m"
  export TARGET_SPEED_MPS="$target_speed_mps"
  export TARGET_HEADING_DEG="$target_heading_deg"
  export TARGET_TURN_RATE_DEGPS="$target_turn_rate_degps"
  export TARGET_CLIMB_RATE_MPS="$target_climb_rate_mps"
  export RADAR_PERIOD_SEC="$radar_period_sec"
  export RADAR_TIMEOUT_SEC="$radar_timeout_sec"
  export RADAR_NOISE_STD_M="$radar_noise_std_m"
  export RADAR_DROPOUT_START_SEC="$radar_dropout_start_sec"
  export RADAR_DROPOUT_DURATION_SEC="$radar_dropout_duration_sec"
  export RADAR_ACQUIRE_DURATION_SEC="$radar_acquire_duration_sec"
  export CHASE_TIMEOUT_S="$chase_timeout_s"
  export EXPECTED_RESULT="$expected_result"
  export FOV_BARRIER_GAIN="$fov_barrier_gain"
  export FOV_BARRIER_MAX="$fov_barrier_max"
  export FOV_SLOWDOWN_START_RATIO="$fov_slowdown_start_ratio"
  export FOV_MIN_FORWARD_SPEED_RATIO="$fov_min_forward_speed_ratio"
  export FOV_EMERGENCY_YAW_RATIO="$fov_emergency_yaw_ratio"
  export K_Z_FF="$k_z_ff"
  export N_NAV_Z="$n_nav_z"
  export MAX_VZ="$max_vz"
  export K_YAW="$k_yaw"
  export K_YAW_D="$k_yaw_d"
  export MAX_YAW_RATE="$max_yaw_rate"
  export CONTROLLER_FOV_HORIZONTAL_LIMIT_DEG="${controller_fov_horizontal_limit_deg:-26.6}"
  export CONTROLLER_FOV_VERTICAL_LIMIT_DEG="${controller_fov_vertical_limit_deg:-20.6}"
  export RADAR_SEED=20260924
  export SHOW_GUI=0
  export RUN_WALL_TIMEOUT_S=150

  echo "=== Running $case_name ==="
  "$SCRIPT_DIR/start_radar_approach_multisim.sh" \
    > "$batch_dir/${case_name}_launcher.log" 2>&1
  tail -n 9 "$batch_dir/${case_name}_launcher.log"
done < <(tail -n +2 "$CASES_FILE")

python3 "$SCRIPT_DIR/aggregate_fov_tuning.py" "$batch_dir"
printf '%s\n' "$batch_dir" > "$RUN_ROOT/latest_fov_tuning_batch.txt"
echo "FOV tuning batch complete: $batch_dir"
