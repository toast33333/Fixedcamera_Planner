# 无雷达抵近阶段的外场启动脚本

前三个入口启动视觉检测、视觉控制器和完整舵机链路，不启动雷达抵近模块，
也不会自动解锁、起飞或切换飞行模式。舵机链路包括
`/camera_pitch_servo_open_loop` 和 `/camera_pitch_servo_sysfs_pwm` 两个节点。
视野保持在水平、竖直和完整三维模式中均保持开启，但只有
`/object_detections` 中 `score > 0.5` 的检测框能够产生附加视野保持修正。

使用前先让两架飞机稳定悬停，让靶机清楚地出现在测试机固定相机画面里，并确认视觉话题的 `is_tracking` 为 `true`。运行脚本后，飞手再次确认空域和画面安全，再手动将测试机切换到 `GUIDED`。

```bash
# 完整控制：前向飞行 + 水平修正 + 竖直修正
./tools/field/start_intercept_full.sh

# 仅水平闭环修正：保留前向飞行，竖直速度指令固定为 0
./tools/field/start_intercept_horizontal_only.sh

# 仅竖直闭环修正：保留前向飞行，横向速度和偏航角速度指令固定为 0
./tools/field/start_intercept_vertical_only.sh

# 圆圈飞行：只启动独立圆圈节点，不启动视觉、拦截或雷达抵近节点
./tools/field/start_circle_trajectory.sh

# 室外无桨上锁测试：真实GPS/EKF、视觉、舵机和最终MAVROS控制话题
./tools/field/start_intercept_outdoor_ground_test.sh

# 原有室内检查：假位姿/速度，隔离飞行指令，舵机 PWM 强制关闭
./tools/field/start_intercept_ground_test.sh

# 飞手手飞检查：只启动视觉和舵机，实时打印检测；不启动飞机拦截控制
./tools/field/start_visual_servo_manual_flight_test.sh
```

仅水平模式强制关闭舵机角度算法和 PWM，竖直速度指令恒为零；该模式的
视野保持只允许修正偏航。仅竖直模式不生成偏航修正，视野保持可修正竖直
速度，舵机正常参与。完整模式同时启用两个方向。

竖直模式和完整模式的一键入口默认会启动舵机角度控制，并完成两级 PWM
安全解锁；退出脚本时会先关闭 PWM。仅在明确要保持舵机硬件关闭时覆盖：

```bash
SERVO_HARDWARE_AUTO_ARM=0 ENABLE_CAMERA_PITCH_SERVO=false \
  ./tools/field/start_intercept_full.sh
```

室外地面脚本要求飞控上锁且非 GUIDED，但使用的是真实最终控制话题；窗口
会持续打印舵机期望角、最终飞行控制指令和视觉检测。原有室内地面脚本
仍然隔离飞行指令并强制关闭 PWM。可单独观察舵机角度：

```bash
rostopic echo /camera_pitch_servo/command_deg
```

手飞视觉舵机测试使用真实视觉、IMU、GPS/EKF 位姿和舵机 PWM。飞机始终由
飞手控制，专用节点 `/camera_pitch_visual_tracker` 在代码内部关闭飞行控制，
不会创建 `/mavros/setpoint_velocity/cmd_vel` 发布端。窗口只持续打印
`/object_detections` 和舵机期望角；`score <= 0.5` 的检测框不会改变舵机角度。
退出时脚本会先释放 PWM。该入口要求飞控已连接且 GPS/EKF 已有本地位姿，
因为舵机跟踪需要用真实飞机姿态进行补偿。

默认前向速度仍采用项目当前的保守值 `5.0 m/s`，丢失目标时前向速度为 `0 m/s`。如果试验任务明确需要其他速度，可在命令前覆盖，例如：

```bash
FORWARD_SPEED_MPS=11.5 ./tools/field/start_intercept_full.sh
```

常用可覆盖参数：

- `STATE_SOURCE=mavros`：直接使用 ArduCopter/MAVROS 状态，默认值。
- `STATE_SOURCE=apm`：`mavros` 的同义写法。
- `STATE_SOURCE=inslib`：同时启动 INSLIB 状态估计器。
- `DETECTION_TOPIC=/object_detections`：带采集时间戳的视觉检测话题。
- `DETECTION_MODE=1`：使用 `detection_msgs/Detection`，默认值。
- `MAVROS_STATE_TOPIC=/mavros/state`：飞控连接状态话题。
- `PREFLIGHT_TIMEOUT_S=10`：启动前等待状态和视觉消息的最长时间。
- `REQUEST_APM_STREAMS=0`：默认由RK3576容器入口统一请求遥测流；脱离容器单独运行时可设为 `1`。
- `ALLOW_ALREADY_GUIDED=0`：默认拒绝在已经处于 `GUIDED` 时启动，避免突然接管。
- `ENABLE_CAMERA_PITCH_SERVO=true|false`：覆盖当前模式是否生成舵机角度指令。
- `SERVO_HARDWARE_AUTO_ARM=0`：默认只启动舵机节点而不输出 PWM；设为 `1` 才执行两级硬件解锁。
- `WS_DIR=...`：ROS 工作空间根目录；默认按仓库所在位置自动推导。

实机默认按 `Pixhawk6C + ArduCopter 4.6.3 + USB` 使用。MAVROS 必须通过
`apm.launch` 启动，飞控USB当前映射为 `/dev/ttyACM0:115200`，并保持
`/mavros/setpoint_velocity/mav_frame=LOCAL_NED`。脚本会检查 ArduPilot 类型、
坐标系、IMU、本地位姿、本地速度和视觉跟踪；GPS/EKF 尚未建立位置解时不会启动控制器。

视觉节点应在相机帧读出时填写 `Detection.header.stamp`，不要填写推理完成
或消息发布时刻。视觉计算机与 MAVROS/IMU 若不在同一台主机上，必须先用
chrony、PTP 等方式同步系统时钟。

如外场确认图像时间戳无法与 IMU 正确对齐，可在
`src/png_planner/config/core_tuning.yaml` 中把：

```yaml
use_detection_header_stamp: true
image_delay_sec: 0.08
```

改为：

```yaml
use_detection_header_stamp: false
image_delay_sec: 0.08
```

停止当前拦截脚本后重新运行原脚本即可，不需要修改视觉节点或重新编译。
固定延迟模式使用“检测消息到达控制器的时刻减去 0.08 秒”查找当时的 IMU
姿态；恢复时间戳模式只需把布尔值改回 `true`。节点启动日志出现
`FIELD FALLBACK active` 表示固定延迟模式已经生效。

仅在台架调试且明确知道风险时，才可设置 `SKIP_PREFLIGHT_CHECKS=1` 跳过飞控连接和视觉跟踪检查。
