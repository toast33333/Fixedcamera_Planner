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
RADAR_TIMEOUT_SEC=${RADAR_TIMEOUT_SEC:-7.0}
RADAR_NOISE_STD_M=${RADAR_NOISE_STD_M:-0.0}
RADAR_DROPOUT_START_SEC=${RADAR_DROPOUT_START_SEC:--1.0}
RADAR_DROPOUT_DURATION_SEC=${RADAR_DROPOUT_DURATION_SEC:-0.0}
RADAR_ACQUIRE_DURATION_SEC=${RADAR_ACQUIRE_DURATION_SEC:-6.5}
RADAR_SEED=${RADAR_SEED:-1}
CAPTURE_DISTANCE_M=${CAPTURE_DISTANCE_M:-5.0}
CHASE_TIMEOUT_S=${CHASE_TIMEOUT_S:-45.0}
DETECTION_RANGE_M=${DETECTION_RANGE_M:-30.0}
RUN_WALL_TIMEOUT_S=${RUN_WALL_TIMEOUT_S:-120}
EXPECTED_RESULT=${EXPECTED_RESULT:-unspecified}
SHOW_GUI=${SHOW_GUI:-0}
FOV_BARRIER_GAIN=${FOV_BARRIER_GAIN:-0.20}
FOV_BARRIER_MAX=${FOV_BARRIER_MAX:-3.0}
FOV_SLOWDOWN_START_RATIO=${FOV_SLOWDOWN_START_RATIO:-0.99}
FOV_MIN_FORWARD_SPEED_RATIO=${FOV_MIN_FORWARD_SPEED_RATIO:-1.0}
FOV_EMERGENCY_YAW_RATIO=${FOV_EMERGENCY_YAW_RATIO:-0.85}
K_Z_FF=${K_Z_FF:-1.0}
N_NAV_Z=${N_NAV_Z:-2.0}
MAX_VZ=${MAX_VZ:-1.4}
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

export CASE_NAME SCENARIO_NAME TARGET_SPAWN_X TARGET_SPAWN_Y
export INTERCEPTOR_ALTITUDE_M TARGET_ALTITUDE_M TARGET_SPEED_MPS
export TARGET_HEADING_DEG TARGET_TURN_RATE_DEGPS TARGET_CLIMB_RATE_MPS
export RADAR_PERIOD_SEC RADAR_TIMEOUT_SEC RADAR_NOISE_STD_M
export RADAR_DROPOUT_START_SEC RADAR_DROPOUT_DURATION_SEC
export RADAR_ACQUIRE_DURATION_SEC RADAR_SEED CAPTURE_DISTANCE_M
export CHASE_TIMEOUT_S DETECTION_RANGE_M EXPECTED_RESULT
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
export WIND_ENABLED WIND_VELOCITY_MEAN_MPS WIND_VELOCITY_MAX_MPS
export WIND_VELOCITY_VARIANCE WIND_DIRECTION_X WIND_DIRECTION_Y
export WIND_DIRECTION_Z WIND_PUBLISH_RATE_HZ
python3 - "$run_dir" <<'PY'
import json
import os
import sys

keys = [
    "CASE_NAME", "SCENARIO_NAME", "TARGET_SPAWN_X", "TARGET_SPAWN_Y",
    "INTERCEPTOR_ALTITUDE_M", "TARGET_ALTITUDE_M", "TARGET_SPEED_MPS",
    "TARGET_HEADING_DEG", "TARGET_TURN_RATE_DEGPS", "TARGET_CLIMB_RATE_MPS",
    "RADAR_PERIOD_SEC", "RADAR_TIMEOUT_SEC", "RADAR_NOISE_STD_M",
    "RADAR_DROPOUT_START_SEC", "RADAR_DROPOUT_DURATION_SEC",
    "RADAR_ACQUIRE_DURATION_SEC", "RADAR_SEED", "CAPTURE_DISTANCE_M",
    "CHASE_TIMEOUT_S", "DETECTION_RANGE_M", "EXPECTED_RESULT",
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

cleanup() {
  set +e
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
if ! python3 "$SCRIPT_DIR/generate_wind_world.py" \
  "$base_world" "$simulation_world" "${wind_enable_args[@]}" \
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

  gz model --spawn-file="$generated_sdf" --model-name="iris_$instance" \
    -x "$spawn_x" -y "$spawn_y" -z 0.83
}

spawn_vehicle 1 0.0 0.0
spawn_vehicle 2 "$TARGET_SPAWN_X" "$TARGET_SPAWN_Y"
sleep 4

if [[ "$SHOW_GUI" == "1" ]]; then
  gzclient > "$run_dir/gzclient.log" 2>&1 &
  child_pids+=("$!")
fi

rosbag record -O "$run_dir/experiment.bag" \
  /interceptor/mavros/state \
  /interceptor/mavros/local_position/pose \
  /interceptor/mavros/local_position/velocity_local \
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
  /fixed_camera_sim/status \
  /fixed_camera_sim/distance \
  /fixed_camera_sim/fov_ratio \
  > "$run_dir/rosbag.log" 2>&1 &
rosbag_pid=$!

if [[ "$SHOW_GUI" == "1" ]]; then
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
    radar_period_sec:="$RADAR_PERIOD_SEC" \
    radar_timeout_sec:="$RADAR_TIMEOUT_SEC" \
    radar_noise_std_m:="$RADAR_NOISE_STD_M" \
    radar_dropout_start_sec:="$RADAR_DROPOUT_START_SEC" \
    radar_dropout_duration_sec:="$RADAR_DROPOUT_DURATION_SEC" \
    radar_acquire_duration_sec:="$RADAR_ACQUIRE_DURATION_SEC" \
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
    > "$run_dir/roslaunch.log" 2>&1
launch_status=$?
set -e
printf '%s\n' "$launch_status" > "$run_dir/roslaunch_exit_code.txt"

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
