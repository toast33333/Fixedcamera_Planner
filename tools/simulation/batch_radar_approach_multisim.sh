#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CASES_FILE=${1:-"$SCRIPT_DIR/radar_multisim_cases.csv"}
RUN_ROOT=/home/toast/fixedcamera_radar_multisim_results
BATCH_ID=${BATCH_ID:-$(date +%Y%m%d_%H%M%S)_radar_matrix}
export BATCH_ID

batch_dir="$RUN_ROOT/$BATCH_ID"
mkdir -p "$batch_dir"
cp "$CASES_FILE" "$batch_dir/cases.csv"

while IFS=, read -r case_name target_spawn_x target_spawn_y target_altitude_m \
    target_speed_mps target_heading_deg target_turn_rate_degps target_climb_rate_mps \
    radar_period_sec radar_timeout_sec radar_noise_std_m radar_dropout_start_sec \
    radar_dropout_duration_sec radar_acquire_duration_sec chase_timeout_s expected_result; do
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
  export RADAR_SEED=20260924
  export SHOW_GUI=0
  export RUN_WALL_TIMEOUT_S=140

  echo "=== Running $case_name (expected $expected_result) ==="
  "$SCRIPT_DIR/start_radar_approach_multisim.sh" \
    > "$batch_dir/${case_name}_launcher.log" 2>&1
  tail -n 14 "$batch_dir/${case_name}_launcher.log"
done < <(tail -n +2 "$CASES_FILE")

python3 "$SCRIPT_DIR/aggregate_radar_multisim.py" "$batch_dir"
printf '%s\n' "$batch_dir" > "$RUN_ROOT/latest_batch.txt"
echo "Batch complete: $batch_dir"
