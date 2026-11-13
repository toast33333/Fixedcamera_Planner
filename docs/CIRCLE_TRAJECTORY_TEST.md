# 圆圈/螺旋轨迹飞行测试

这个功能用于测试飞机在不同水平速度下的连续转弯、轨迹跟随和偏航响应能力。它独立于固定相机抵近/视觉控制节点，不能与其他向同一 MAVROS 速度话题发指令的控制节点同时运行。

## 实际运行流程

1. 记录开始测试时的悬停位置，作为圆心和返航点。
2. 先以 `positioning_speed` 飞到圆心正 x 方向、距离 `start_radius` 的外圆起点。
3. 在一个速度档位下做逐渐向圆心收紧的阿基米德螺旋转弯。
4. 到达圆心，或实际位置偏离期望轨迹超过 `trajectory_error_limit` 时，返回圆心悬停点。
5. 切换到下一个速度档并重复。

原始附件中“从圆心直接开始外圆轨迹”和“返航后结束全部测试”的两个状态流程问题已修正。现在会先到外圆起点，返航后也会正确进入下一速度档。

## 一键 PX4/Gazebo 仿真

先编译工作空间：

```bash
cd ~/your_catkin_ws
catkin_make --pkg png_planner
source devel/setup.bash
```

默认运行 5、7、9、11、13、15 m/s 六个速度档：

```bash
cd ~/your_catkin_ws/src/Fixedcamera_Planner
./tools/simulation/start_circle_trajectory_multisim.sh
```

无 Gazebo 画面运行：

```bash
SHOW_GUI=0 ./tools/simulation/start_circle_trajectory_multisim.sh
```

快速冒烟测试（只测 5 m/s，起始半径 8 m）：

```bash
SHOW_GUI=0 MINIMUM_SPEED=5 MAXIMUM_SPEED=5 START_RADIUS=8 RADIAL_RATE=1 \
RUN_WALL_TIMEOUT_S=120 ./tools/simulation/start_circle_trajectory_multisim.sh
```

结果默认保存在工作空间旁边的 `circle_trajectory_results/`，其中有 CSV、rosbag 和各节点日志。

## 已经起飞时单独启动节点

飞机需要已经解锁、悬停，并处于能接收 MAVROS 速度指令的模式。默认话题是单机 `/mavros/...`：

```bash
roslaunch png_planner circle_trajectory_test.launch \
  minimum_speed:=5.0 maximum_speed:=15.0 speed_step:=2.0 \
  start_radius:=20.0 radial_rate:=0.5 trajectory_error_limit:=3.0 \
  geofence_radius:=50.0 geofence_margin:=5.0
```

该启动方式不负责解锁、起飞或切换模式。对应的一键仿真脚本会用独立管理节点完成这些准备动作，然后通过 `/circle_trajectory/start` 交接速度指令权。

## 重要参数

- `minimum_speed / maximum_speed / speed_step`：速度档范围与步长。
- `start_radius`：每轮螺旋的起始半径。
- `radial_rate`：半径每秒缩小多少米。
- `trajectory_error_limit`：偏离期望轨迹多少米时结束本轮并返航。
- `position_kp / correction_speed_limit`：轨迹误差回拉强度和附加修正速度上限。
- `max_yaw_rate`：允许发送的最大偏航角速度。
- `geofence_radius / geofence_margin / stop_buffer`：以开始点为圆心的软件安全边界。

## 结果判读

CSV 每个速度档一行。`CENTER_REACHED` 表示已完成到圆心的螺旋；`ERROR_LIMIT` 表示轨迹误差超限后安全返航。两者都会继续下一速度档。`ABORTED` 则表示位置/速度反馈超时、越界或返航超时，节点会发送零速度并中止后续测试。

软件围栏不能替代飞控地理围栏、人工监控和急停措施。
