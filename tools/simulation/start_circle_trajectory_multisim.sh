#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
ROS_SETUP=${ROS_SETUP:-/opt/ros/noetic/setup.bash}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
WS_DIR=${WS_DIR:-$(cd "$REPO_DIR/../.." && pwd)}
RUN_ROOT=${RUN_ROOT:-$WS_DIR/../circle_trajectory_results}
DISPLAY_VALUE=${DISPLAY_VALUE:-${DISPLAY:-:0}}
XAUTHORITY_FILE=${XAUTHORITY_FILE:-${XAUTHORITY:-}}
SHOW_GUI=${SHOW_GUI:-1}
RUN_WALL_TIMEOUT_S=${RUN_WALL_TIMEOUT_S:-480}

MINIMUM_SPEED=${MINIMUM_SPEED:-5.0}
MAXIMUM_SPEED=${MAXIMUM_SPEED:-15.0}
SPEED_STEP=${SPEED_STEP:-2.0}
START_RADIUS=${START_RADIUS:-20.0}
RADIAL_RATE=${RADIAL_RATE:-0.5}
TRAJECTORY_ERROR_LIMIT=${TRAJECTORY_ERROR_LIMIT:-3.0}
CLOCKWISE=${CLOCKWISE:-false}
TARGET_ALTITUDE_M=${TARGET_ALTITUDE_M:-5.0}
XY_VELOCITY_LIMIT_MPS=${XY_VELOCITY_LIMIT_MPS:-20.0}

[[ -f "$ROS_SETUP" ]] || { echo "ROS setup not found: $ROS_SETUP" >&2; exit 2; }
[[ -f "$WS_DIR/devel/setup.bash" ]] || {
  echo "Workspace is not built: $WS_DIR/devel/setup.bash" >&2
  exit 2
}
[[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]] || {
  echo "PX4 SITL is not built under: $PX4_DIR" >&2
  exit 2
}

set +u
source "$ROS_SETUP"
source "$WS_DIR/devel/setup.bash"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
set -u

if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null; then
  echo "PX4 or Gazebo is already running. Stop the previous simulation first." >&2
  exit 2
fi

run_stamp=$(date +%Y%m%d_%H%M%S)
run_dir="$RUN_ROOT/${run_stamp}_circle_trajectory"
mkdir -p "$run_dir" "$RUN_ROOT"
printf '%s\n' "$run_dir" > "$RUN_ROOT/latest_run.txt"
printf '%s\n' "$$" > "$RUN_ROOT/active.pid"

export DISPLAY="$DISPLAY_VALUE"
if [[ -n "$XAUTHORITY_FILE" ]]; then export XAUTHORITY="$XAUTHORITY_FILE"; fi
export DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$(id -u)/bus}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}
export PX4_SIM_MODEL=gazebo-classic_iris

child_pids=()
px4_pids=()
cleanup() {
  set +e
  for pid in "${child_pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  for pid in "${px4_pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${px4_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi
  done
  rm -f "$RUN_ROOT/active.pid"
}
trap cleanup INT TERM EXIT

roscore > "$run_dir/roscore.log" 2>&1 &
child_pids+=("$!")
for _ in $(seq 1 40); do
  if rostopic list >/dev/null 2>&1; then break; fi
  sleep 0.25
done

gzserver "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world" \
  --verbose -s libgazebo_ros_api_plugin.so > "$run_dir/gzserver.log" 2>&1 &
child_pids+=("$!")
sleep 5

instance=1
working_dir="$PX4_DIR/build/px4_sitl_default/rootfs/$instance"
generated_sdf="$run_dir/iris_$instance.sdf"
mkdir -p "$working_dir"
(
  cd "$working_dir"
  "$PX4_DIR/build/px4_sitl_default/bin/px4" -i "$instance" \
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
gz model --spawn-file="$generated_sdf" --model-name="iris_$instance" -x 0 -y 0 -z 0.83

if [[ "$SHOW_GUI" == "1" ]]; then
  gzclient > "$run_dir/gzclient.log" 2>&1 &
  child_pids+=("$!")
fi

roslaunch png_planner circle_trajectory_multisim.launch \
  results_dir:="$run_dir" \
  minimum_speed:="$MINIMUM_SPEED" \
  maximum_speed:="$MAXIMUM_SPEED" \
  speed_step:="$SPEED_STEP" \
  start_radius:="$START_RADIUS" \
  radial_rate:="$RADIAL_RATE" \
  trajectory_error_limit:="$TRAJECTORY_ERROR_LIMIT" \
  clockwise:="$CLOCKWISE" \
  target_altitude_m:="$TARGET_ALTITUDE_M" \
  xy_velocity_limit_mps:="$XY_VELOCITY_LIMIT_MPS" \
  > "$run_dir/roslaunch.log" 2>&1 &
roslaunch_pid=$!
child_pids+=("$roslaunch_pid")

for _ in $(seq 1 120); do
  if rostopic info /circle_trajectory/finished >/dev/null 2>&1; then break; fi
  if ! kill -0 "$roslaunch_pid" 2>/dev/null; then
    echo "ROS launch exited before the test became ready. See $run_dir/roslaunch.log" >&2
    exit 1
  fi
  sleep 0.25
done

rosbag record -O "$run_dir/experiment.bag" \
  /interceptor/mavros/state \
  /interceptor/mavros/local_position/pose \
  /interceptor/mavros/local_position/velocity_local \
  /interceptor/mavros/setpoint_velocity/cmd_vel \
  /circle_trajectory/status \
  /circle_trajectory/sitl_manager_status \
  /circle_trajectory/desired_path \
  /circle_trajectory/actual_path \
  > "$run_dir/rosbag.log" 2>&1 &
child_pids+=("$!")

if ! timeout "${RUN_WALL_TIMEOUT_S}s" rostopic echo -n 1 /circle_trajectory/finished \
    > "$run_dir/finished.txt" 2>&1; then
  echo "Circle-trajectory simulation timed out after ${RUN_WALL_TIMEOUT_S}s." >&2
  echo "Logs: $run_dir" >&2
  exit 1
fi

if [[ ! -s "$run_dir/circle_trajectory_test.csv" ]]; then
  echo "Test finished without a CSV result. See $run_dir/roslaunch.log" >&2
  exit 1
fi

timeout 5s rostopic echo -n 1 /circle_trajectory/status \
  > "$run_dir/final_status.txt" 2>&1 || true
if ! grep -q 'FINISHED' "$run_dir/final_status.txt"; then
  echo "Circle-trajectory test ended without FINISHED status." >&2
  cat "$run_dir/final_status.txt" >&2
  echo "Logs: $run_dir" >&2
  exit 1
fi

echo "Circle-trajectory simulation complete."
echo "Results: $run_dir"
cat "$run_dir/circle_trajectory_test.csv"
