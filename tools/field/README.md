# 无雷达抵近阶段的外场启动脚本

这三个入口只启动视觉控制器，不启动雷达抵近模块，也不会自动解锁、起飞或切换飞行模式。

使用前先让两架飞机稳定悬停，让靶机清楚地出现在测试机固定相机画面里，并确认视觉话题的 `is_tracking` 为 `true`。运行脚本后，飞手再次确认空域和画面安全，再手动将测试机切换到 `GUIDED`。

```bash
# 完整控制：前向飞行 + 水平修正 + 竖直修正
./tools/field/start_intercept_full.sh

# 仅水平闭环修正：保留前向飞行，竖直速度指令固定为 0
./tools/field/start_intercept_horizontal_only.sh

# 仅竖直闭环修正：保留前向飞行，横向速度和偏航角速度指令固定为 0
./tools/field/start_intercept_vertical_only.sh
```

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
- `WS_DIR=...`：ROS 工作空间根目录；默认按仓库所在位置自动推导。

实机默认按 `Pixhawk6C + ArduCopter 4.6.3 + TELEM2` 使用。MAVROS 必须通过
`apm.launch` 启动，TELEM2 当前映射为 `/dev/ttyS8:115200`，并保持
`/mavros/setpoint_velocity/mav_frame=LOCAL_NED`。脚本会检查 ArduPilot 类型、
坐标系、IMU、本地位姿、本地速度和视觉跟踪；GPS/EKF 尚未建立位置解时不会启动控制器。

视觉节点应在相机帧读出时填写 `Detection.header.stamp`，不要填写推理完成
或消息发布时刻。视觉计算机与 MAVROS/IMU 若不在同一台主机上，必须先用
chrony、PTP 等方式同步系统时钟。

仅在台架调试且明确知道风险时，才可设置 `SKIP_PREFLIGHT_CHECKS=1` 跳过飞控连接和视觉跟踪检查。
