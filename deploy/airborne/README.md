# 固定相机算法机载部署包

本目录说明的是由 `tools/deployment/export_airborne_workspace.py` 生成的
精简 ROS1 工作区。它只面向测试机机载 Ubuntu/RK3588，不包含 Gazebo、SITL、
批量仿真、结果分析、历史控制器、旧视觉入口、备份脚本或备用模型。

## 1. 包内结构

```text
src/Fixedcamera_Planner/
├── src/detection_msgs/       # RK3588 检测与带采集时间戳 Detection 消息
├── src/kcf_msgs/             # 仅保留旧 Bbox 消息兼容定义
├── src/png_planner/          # 固定相机控制与雷达抵近/交接算法
└── tools/field/              # 三种无雷达视觉控制启动入口
```

当前唯一随包携带的模型是：
`detection_msgs/rknnModel/yolov8-NewDrone.rknn`。

## 2. 机载环境

- Ubuntu 20.04 / ROS Noetic；
- Python 3；
- MAVROS 已经能够连接 PX4，并发布本机的位姿、速度和 IMU 话题；
- RK3588 对应版本的 RKNN Lite2 Python wheel 和运行库；
- OpenCV、NumPy、GeographicLib。

ROS/系统依赖可在联网环境下安装：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-mavros ros-noetic-mavros-extras \
  python3-opencv python3-numpy libgeographic-dev
```

RKNN Lite2 必须使用板卡系统镜像/SDK配套的 `rknn_toolkit_lite2` wheel，不要在
不知道板卡 Python 与运行库版本时随意升级。

## 3. 编译和检查

```bash
cd ~/fixed_camera_airborne_ws
./build_airborne.sh
./verify_airborne.sh
source devel/setup.bash
```

## 4. 启动视觉

先确认相机编号。默认相机编号是 `40`：

```bash
v4l2-ctl --list-devices
roslaunch detection_msgs field_detector.launch camera_id:=40
```

视觉节点在相机读出帧后立即填写 `/object_detections.header.stamp`。控制器用这个
采集时间与 IMU 历史姿态对齐，不使用固定 `0.08 s` 延迟。

检查：

```bash
rostopic hz /object_detections
rostopic echo -n 1 /object_detections
```

## 5. 无雷达、直接视觉控制

先启动 ROS Master、MAVROS 和上面的视觉节点；两机悬停并让靶机进入测试机画面。
确认 `is_tracking: true` 后另开终端：

```bash
cd ~/fixed_camera_airborne_ws/src/Fixedcamera_Planner
./tools/field/start_intercept_full.sh
```

仅水平或仅竖直试验分别运行：

```bash
./tools/field/start_intercept_horizontal_only.sh
./tools/field/start_intercept_vertical_only.sh
```

这些脚本不会自动解锁、起飞或切换飞行模式。飞手确认空域、画面和遥控接管均正常
后，手动把测试机切入 `GUIDED`。

## 6. 雷达抵近后交接视觉控制

```bash
source ~/fixed_camera_airborne_ws/devel/setup.bash
roslaunch png_planner radar_approach_intercept.launch
```

默认雷达输入是配置文件指定的话题。接外场雷达前必须核对
`radar_approach_handover.yaml` 的输入模式、话题、坐标系、时间戳和误差参数。

## 7. 上机前必须核对

1. `core_tuning.yaml` 中相机安装上抬角和相机相对机体中心偏移；
2. `fixed_camera_ibvs.yaml` 中图像尺寸、内参与实际相机输出一致；
3. PX4/MAVROS与视觉计算机系统时钟同步；
4. `/object_detections` 的时间戳递增且 `frame_id` 正确；
5. 飞控的 GUIDED/OFFBOARD 速度接口、坐标系和失控保护已经台架验证；
6. 首飞使用限速、系留或足够大的隔离空域，并保留人工切回姿态/定点模式的能力。
