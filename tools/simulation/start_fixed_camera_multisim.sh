#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP=/opt/ros/noetic/setup.bash
PX4_DIR=/home/toast/PX4-Autopilot
WS_DIR=/home/toast/fixedcamera_multisim_ws
RUN_ROOT=/home/toast/fixedcamera_multisim_results
DISPLAY_VALUE=:1
XAUTHORITY_FILE=/run/user/1000/gdm/Xauthority

source "$ROS_SETUP"
source /home/toast/png_planner_ws/devel/setup.bash
source "$WS_DIR/devel/setup.bash"

if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null; then
  echo "PX4 or Gazebo is already running. Stop that run before starting this experiment."
  exit 2
fi

run_stamp=$(date +%Y%m%d_%H%M%S)
run_dir="$RUN_ROOT/$run_stamp"
mkdir -p "$run_dir"
mkdir -p "$RUN_ROOT"
printf '%s\n' "$run_dir" > "$RUN_ROOT/latest_run.txt"
printf '%s\n' "$$" > "$RUN_ROOT/active.pid"

export DISPLAY="$DISPLAY_VALUE"
export XAUTHORITY="$XAUTHORITY_FILE"
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export ROS_MASTER_URI=http://127.0.0.1:11311
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export PX4_SIM_MODEL=gazebo-classic_iris

child_pids=()
px4_pids=()

cleanup() {
  set +e
  for pid in "${child_pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  for pid in "${px4_pids[@]}"; do
    kill "$pid" 2>/dev/null || true
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

source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"

gzserver "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world" \
  --verbose -s libgazebo_ros_api_plugin.so > "$run_dir/gzserver.log" 2>&1 &
child_pids+=("$!")
sleep 5

spawn_vehicle() {
  local instance=$1
  local spawn_x=$2
  local working_dir="$PX4_DIR/build/px4_sitl_default/rootfs/$instance"
  local generated_sdf="$run_dir/iris_$instance.sdf"
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

  gz model --spawn-file="$generated_sdf" --model-name="iris_$instance" \
    -x "$spawn_x" -y 0 -z 0.83
}

# Interceptor starts at x=0; target starts 20 m straight ahead.
spawn_vehicle 1 0
spawn_vehicle 2 20
sleep 4

gzclient > "$run_dir/gzclient.log" 2>&1 &
child_pids+=("$!")

roslaunch png_planner fixed_camera_multisim.launch results_dir:="$run_dir" \
  > "$run_dir/roslaunch.log" 2>&1 &
roslaunch_pid=$!
child_pids+=("$roslaunch_pid")

for _ in $(seq 1 80); do
  if rostopic info /fixed_camera_sim/status >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

rosbag record --duration=90 -O "$run_dir/experiment.bag" \
  /interceptor/mavros/state \
  /interceptor/mavros/local_position/pose \
  /interceptor/mavros/local_position/velocity_local \
  /target/mavros/state \
  /target/mavros/local_position/pose \
  /target/mavros/local_position/velocity_local \
  /interceptor/mavros/setpoint_velocity/cmd_vel \
  /target/mavros/setpoint_velocity/cmd_vel \
  /object_kcf \
  /fixed_camera_ibvs/los_world \
  /fixed_camera_ibvs/fov_state \
  /fixed_camera_sim/status \
  /fixed_camera_sim/distance \
  /fixed_camera_sim/fov_ratio \
  > "$run_dir/rosbag.log" 2>&1 &
child_pids+=("$!")

rqt_image_view /fixed_camera_sim/image > "$run_dir/rqt_image_view.log" 2>&1 &
child_pids+=("$!")
rqt_plot /fixed_camera_sim/distance/data /fixed_camera_sim/fov_ratio/data \
  > "$run_dir/rqt_plot.log" 2>&1 &
child_pids+=("$!")

gnome-terminal --title="PX4 Fixed-Camera Experiment Status" -- bash -lc \
  "source /opt/ros/noetic/setup.bash; rostopic echo /fixed_camera_sim/status; exec bash" \
  > "$run_dir/status_terminal.log" 2>&1 || true

echo "Fixed-camera multisim run directory: $run_dir"
echo "Gazebo, camera view, plots and status terminal are open on DISPLAY=$DISPLAY_VALUE"
wait "$roslaunch_pid"
