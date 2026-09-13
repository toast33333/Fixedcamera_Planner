# png_planner_ws

> Fixed-camera development now lives in the new `fixed_camera_ibvs` node. The
> original gimbal controllers remain unchanged as a comparison baseline.

## Fixed-camera V1

The fixed-camera controller keeps the existing visual interfaces:

- `kcf_msgs/Bbox` on `/object_kcf`
- `detection_msgs/Detection` on `/object_detections`

It adds quaternion-based aircraft-attitude compensation, configurable image
delay compensation, inertial LOS-rate estimation, and a paper-inspired FOV
barrier while continuing to publish
`geometry_msgs/TwistStamped` on `/mavros/setpoint_velocity/cmd_vel`.

The active FOV constraints use the measured test-aircraft half angles directly:
horizontal `+/-26.6 deg` and vertical `+/-20.6 deg`.

The fixed camera optical axis is mounted `20 deg` upward relative to aircraft
forward, with zero roll/yaw offset (`camera_mount_pitch_deg: -20.0` in ROS FLU).

Build and launch:

```bash
catkin_make
source devel/setup.bash
roslaunch png_planner fixed_camera_ibvs.launch
```

For `detection_msgs/Detection`:

```bash
roslaunch png_planner fixed_camera_ibvs.launch \
  detection_mode:=1 detection_topic:=/object_detections
```

Configuration is in `src/png_planner/config/fixed_camera_ibvs.yaml`. Design,
calibration order, and current limitations are documented in
`docs/fixed_camera_v1.md`.

基于 ROS1 catkin 的视觉制导工作区，核心目标是根据图像检测框驱动无人机进行偏航跟踪、2D PNG 导引和 3D PNG 导引。

当前工作区主要包含 3 个包：

- `png_planner`：控制节点、launch 文件、RViz 配置和测试脚本
- `kcf_msgs`：`kcf_msgs/Bbox` 消息定义
- `detection_msgs`：`detection_msgs/Detection` 消息定义

## 功能概览

`png_planner` 下已经实现多种版本的控制器，覆盖以下几类能力：

- 纯视觉偏航跟踪：`yaw_by_pixel_v1`、`yaw_by_pixel_v2`
- 2D PNG 导引：`pixel_dotpng_v2`、`pixel_dotpng_v2.2`
- 恒速前飞 + 偏航修正：`pixel_dotpng_v2.0`、`pixel_dotpng_v2.1`
- 3D 导引与高度通道控制：`pixel_dotpng_v2.1_3d`、`pixel_dotpng_3d_v2`、`pixel_dotpng_3d_v2.0`、`pixel_dotpng_3d_pro`

常用 launch 文件位于 `src/png_planner/launch/`：

- `yaw_tracker.launch`
- `pixel_dotpng_v2.launch`
- `pixel_dotpng_v2.1_3d.launch`
- `pixel_dotpng_3d_pro.launch`

更详细的节点差异、控制链路和双消息支持说明见 [README_dual_msg_support.md](/media/negroni/新加卷/png_planner_ws/README_dual_msg_support.md)。

## 双消息类型支持

控制节点支持两种检测消息：

- `kcf_msgs/Bbox`
- `detection_msgs/Detection`

两者字段保持一致：

- `x1`
- `y1`
- `x2`
- `y2`
- `score`
- `class_name`
- `track_id`
- `is_tracking`

常用参数：

- `detection_mode:=0` 使用 `kcf_msgs/Bbox`
- `detection_mode:=1` 使用 `detection_msgs/Detection`
- `detection_topic` 用于手动指定检测话题

默认话题约定：

- KCF: `/object_kcf`
- Detection: `/object_detections`

说明：大多数节点默认 `detection_mode=0`，但 `pixel_dotpng_v2.0.cpp` 默认值为 `1`。

## 目录结构

```text
png_planner_ws/
├── src/
│   ├── png_planner/
│   │   ├── launch/
│   │   ├── msg/
│   │   ├── rviz/
│   │   ├── scripts/
│   │   └── src/
│   ├── kcf_msgs/
│   └── detection_msgs/
├── build/
├── devel/
├── README.md
└── README_dual_msg_support.md
```

`build/` 和 `devel/` 是 catkin 构建产物目录。

## 编译

在工作区根目录执行：

```bash
catkin_make
source devel/setup.bash
```

当前 `png_planner` 依赖的主要 ROS 包包括：

- `roscpp`
- `rospy`
- `std_msgs`
- `quadrotor_msgs`
- `kcf_msgs`
- `detection_msgs`

## 快速启动

1. 编译并加载工作区环境：

```bash
cd /media/negroni/新加卷/png_planner_ws
catkin_make
source devel/setup.bash
```

2. 启动纯偏航跟踪：

```bash
roslaunch png_planner yaw_tracker.launch
```

3. 启动 3D PNG 工程版节点：

```bash
roslaunch png_planner pixel_dotpng_3d_pro.launch
```

4. 如果输入是 `detection_msgs/Detection`，显式切换：

```bash
roslaunch png_planner yaw_tracker.launch detection_mode:=1
roslaunch png_planner pixel_dotpng_3d_pro.launch detection_mode:=1
```

5. 自定义检测话题：

```bash
roslaunch png_planner yaw_tracker.launch detection_mode:=1 detection_topic:=/my_detection
```

## 相关脚本与资源

- `src/png_planner/scripts/pose_cmd_tf_broadcaster.py`
  - 将 `pose_cmd` 转成 `world -> uav` TF，便于 RViz 跟踪显示
- `src/png_planner/scripts/batch_target_test.py`
  - 用于单次 PNG 测试和轨迹/收敛图生成
- `src/png_planner/rviz/`
  - 保存 RViz 配置
- `src/png_planner/rosbag/`
  - 保存示例 bag 文件

## 注意事项

- 不同控制器在“目标丢失”后的行为不同，有的停止导引，有的保持前飞，有的继续保留 Z 通道控制。
- launch 文件中的默认参数和源码里的默认参数不一定完全一致，调试时应优先查看对应 launch。
- 修改消息定义后，建议清理后重新编译：

```bash
rm -rf build devel
catkin_make
source devel/setup.bash
```
