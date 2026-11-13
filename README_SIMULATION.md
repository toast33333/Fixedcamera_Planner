# 固定相机 PX4/Gazebo 多机仿真代码包

## 1. 仿真专用子集包含什么

本文说明从主线代码中使用固定相机 PX4 SITL + Gazebo Classic 多机仿真部分，包含：

- 测试机与水平飞行靶机的多机仿真；
- 固定相机视觉检测话题模拟；
- 雷达抵近、视觉交接和直接视觉控制两类入口；
- 水平偏航、竖直速度、视野保持与指令限幅；
- 雷达、视觉、导航、制导延迟和随机误差模型；
- 单轴相机俯仰舵机控制、开环角度到 PWM 换算和仿真执行器模型；
- 批量工况、ROS bag、CSV 结果及分析脚本；
- 可选 INSLIB 状态估计代码。

制作独立的仿真交付包时，可以不包含：

- 圆圈/螺旋轨迹测试；
- AT-Link 通信节点及协议库。

GitHub 的 `main` 是完整主线，仍保留圆圈测试、AT-Link、实机舵机和外场脚本；
上述排除项只描述仿真专用交付边界。

舵机默认关闭，因此不修改原固定相机仿真行为。需要测试舵机时再显式打开。

## 2. 外部环境

推荐 Ubuntu 20.04、ROS Noetic、PX4 SITL 和 Gazebo Classic。PX4 需要已经完成：

```bash
cd /你的路径/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic
```

ROS 侧至少需要 catkin、MAVROS、Gazebo ROS、GeographicLib、Python 3、NumPy、
OpenCV、rosbag、rqt_image_view 和 rqt_plot。PX4-Autopilot 属于外部仿真平台，
不包含在本代码压缩包内。

## 3. 编译

解压后进入代码根目录：

```bash
cd Fixedcamera_Planner
catkin_make
source devel/setup.bash
```

如果把代码放在标准 catkin 工作空间的 `src/Fixedcamera_Planner` 下，则在工作空间
根目录执行同样的 `catkin_make`。一键脚本会自动识别这两种目录结构。

如果 PX4 不在默认的 `/home/toast/PX4-Autopilot`，启动时通过环境变量指定：

```bash
export PX4_DIR=/实际路径/PX4-Autopilot
```

## 4. 常用启动命令

基础固定相机多机演示，带 Gazebo、相机画面和状态窗口：

```bash
./tools/simulation/start_fixed_camera_multisim.sh
```

靶机一开始就在相机视野内，跳过雷达抵近、直接测试视觉控制：

```bash
SHOW_GUI=1 ./tools/simulation/start_intercept_only_multisim.sh
```

测试雷达抵近、视觉交接和后续控制的完整链路：

```bash
SHOW_GUI=1 ./tools/simulation/start_radar_approach_multisim.sh
```

停止基础演示：

```bash
./tools/simulation/stop_fixed_camera_multisim.sh
```

脚本默认拒绝与已经运行的 PX4/Gazebo 实例并行启动。每次试验完成后，应确认没有
遗留的 `px4`、`gzserver` 或控制节点，再开始下一次试验。

## 5. 舵机仿真

在直接视觉仿真中打开单轴俯仰舵机：

```bash
ENABLE_CAMERA_PITCH_SERVO=true SHOW_GUI=1 \
  ./tools/simulation/start_intercept_only_multisim.sh
```

在雷达抵近完整链路中打开舵机：

```bash
ENABLE_CAMERA_PITCH_SERVO=true SHOW_GUI=1 \
  ./tools/simulation/start_radar_approach_multisim.sh
```

主要参数集中在 `src/png_planner/config/core_tuning.yaml`：

- `enable_camera_pitch_servo`：舵机总开关；
- `camera_pitch_servo_min_deg/max_deg`：机械角度范围；
- `camera_pitch_servo_kp_per_sec/ki_per_sec2/kd`：视觉闭环参数；
- `camera_pitch_servo_max_rate_degps`：舵机指令速度上限；
- `camera_pitch_servo_sim_rate_degps`：仿真舵机速度上限；
- `camera_pitch_servo_sim_time_constant_sec`：仿真一阶滞后；
- `camera_pitch_servo_pwm_min_us/neutral_us/max_us`：开环 PWM 参数。

单独测试角度到 PWM 的换算节点：

```bash
roslaunch png_planner camera_pitch_servo_open_loop.launch
rostopic pub -1 /camera_pitch_servo/command_deg std_msgs/Float64 "data: 10.0"
rostopic echo /camera_pitch_servo/pulse_width_us
```

这一节点只发布脉宽数值，不访问真实硬件 PWM 引脚。

## 6. 参数和结果位置

- 统一核心参数：`src/png_planner/config/core_tuning.yaml`
- 固定相机参数：`src/png_planner/config/fixed_camera_ibvs.yaml`
- 雷达抵近参数：`src/png_planner/config/radar_approach_handover.yaml`
- 仿真入口：`tools/simulation/`
- 工况 CSV：`tools/simulation/*cases.csv`
- 舵机说明：`docs/SINGLE_AXIS_CAMERA_PITCH_SERVO.md`
- 固定相机说明：`docs/fixed_camera_v1.md`
- 雷达抵近说明：`docs/radar_approach_handover.md`

默认试验结果写到工作空间旁边的 `fixedcamera_multisim_results` 或
`fixedcamera_radar_multisim_results`。可以在命令前设置 `RUN_ROOT=/其他目录`
修改结果位置。

## 7. 使用提醒

1. 本包是仿真源码，不是已编译的 ROS 工作空间，也不包含 PX4-Autopilot。
2. 仿真脚本会直接启动两个 PX4 SITL 实例和 Gazebo Classic。
3. 不要同时运行两个会发布测试机速度指令的控制节点。
4. 默认舵机关闭；没有显式设置 `ENABLE_CAMERA_PITCH_SERVO=true` 时，控制结果与
   原固定相机逻辑一致。
5. 检测消息使用图像采集时间戳与 IMU 历史对齐，不使用固定的 0.08 秒延迟。
