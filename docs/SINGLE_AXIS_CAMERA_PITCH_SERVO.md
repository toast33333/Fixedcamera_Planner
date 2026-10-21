# 单轴相机俯仰舵机接口

## 1. 作用和边界

该功能在现有固定相机控制器上增加一个可选的俯仰舵机层。舵机只有竖直方向一个自由度，并且角度始终是**相对机身**的机械角度；它没有地平自稳能力。飞机横滚、俯仰后，相机仍会随飞机一起转动，控制器继续使用 IMU 姿态补偿这部分运动。

默认 `enable_camera_pitch_servo: false`，所以当前固定相机实机和仿真行为不变。等舵机驱动接口完成后再显式打开。

控制分工如下：

- 水平方向：仍由飞机偏航控制，舵机不参与。
- 竖直视野：舵机根据目标在图像里的上下偏差快速转动，使目标回到期望位置。
- 竖直飞行：原有 `k_z_ff + N_nav_z` 控制保留，并始终以“舵机中位时的相机轴”为参考。因此飞机会较慢地跟上目标，把舵机逐渐卸回中位，而不是让舵机替代飞行控制。
- 舵机到达机械限位后：飞机原有竖直视野屏障仍然生效，可以继续帮助保持目标不出视野。

## 2. ROS 接口约定

### 指令

- 话题：`/camera_pitch_servo/command_deg`
- 类型：`std_msgs/Float64`
- 单位：度
- 含义：相对机械中位的角度，正数上抬，负数下压

后续硬件适配节点只需要订阅该话题，再换算成 PWM、串口数据或飞控舵机命令。硬件适配层负责处理舵机自身的零偏、PWM 中值、方向反相和脉宽限制，不应改变上述 ROS 侧的正负号定义。

### 角度反馈（可选）

- 话题：`/camera_pitch_servo/feedback_deg`
- 类型：`std_msgs/Float64`
- 单位和符号与指令相同

有角度传感器时设置 `camera_pitch_servo_use_feedback: true`。控制器使用反馈角更新相机几何；反馈超时后会告警，并临时退回到限速后的指令角估计。没有反馈的普通舵机保持该参数为 `false`。

当前占位反馈消息没有 Header，因此角度样本用 ROS 接收时刻写入历史；适配节点应高频、低延迟发布。若最终驱动协议能给出硬件采样时刻，可在接口定型时换成带 Header 的反馈消息，控制器内部的角度历史和插值逻辑不需要重做。

### 诊断

- 话题：`/fixed_camera_ibvs/camera_pitch_servo_state`
- 类型：`geometry_msgs/Vector3Stamped`
- `x`：相对角指令，度
- `y`：LOS 计算实际采用的相对角，度
- `z`：固定安装上抬角与舵机相对角之和，度

## 3. 控制公式

令图像中心偏差的正方向为向上，当前目标和期望目标的竖直角误差为

```text
e = atan2(v, fy) - atan2(v_des, fy)
```

其中 `v`、`v_des` 为以图像中心为原点的竖直像素量，`fy` 为竖直焦距。`e > 0` 表示目标在期望点上方，需要相机上抬。

舵机角速度指令为

```text
omega_s = sat(kp * e + ki * integral(e) + kd * de/dt,
              -omega_max, +omega_max)
theta_cmd(k+1) = sat(theta_cmd(k) + omega_s * dt,
                     theta_min, theta_max)
```

误差进入死区后积分按比例衰减；指令碰到机械限位时也衰减积分，避免积分饱和。目标丢失后，指令以 `camera_pitch_servo_return_rate_degps` 限速回到机械中位。

实际相机安装俯仰角进入 LOS 坐标变换时使用

```text
camera_mount_pitch = fixed_mount_pitch - theta_servo
```

这里 ROS 机体系绕 `+y` 的负角表示光轴上抬，所以正的舵机上抬角要从安装俯仰角中减去。检测消息仍使用采集时间戳；控制器同时保留舵机角历史，并用采集时刻对应的舵机角和 IMU 姿态计算世界系 LOS。

## 4. 主要参数

参数集中在 `src/png_planner/config/core_tuning.yaml` 第 6 组：

- `enable_camera_pitch_servo`：总开关。
- `camera_pitch_servo_neutral_deg`：机械中位对应的相对角，一般为 0。
- `camera_pitch_servo_min_deg/max_deg`：相对机械角限位。
- `camera_pitch_servo_kp_per_sec/ki_per_sec2/kd`：图像竖直误差闭环参数。
- `camera_pitch_servo_deadband_deg`：中心死区。
- `camera_pitch_servo_max_rate_degps`：控制器允许的最大转速。
- `camera_pitch_servo_return_on_loss`：丢失目标后是否回中。
- `camera_pitch_servo_return_rate_degps`：回中速度。

首次接实物时，先将角度范围临时限制为 `-5°~+5°`，飞机不上桨并固定机身，用手上下移动目标确认：目标向画面上方移动时，指令应为正并使相机上抬。符号确认后再逐步放宽机械范围。

## 5. 仿真开关

仿真器包含限速加一阶滞后的单轴舵机模型，并向反馈话题发布实际角度：

```bash
roslaunch png_planner fov_tuning_visual_multisim.launch \
  enable_camera_pitch_servo:=true
```

也可以在批处理入口前设置：

```bash
export ENABLE_CAMERA_PITCH_SERVO=true
```

仿真专用参数为 `camera_pitch_servo_sim_rate_degps` 和 `camera_pitch_servo_sim_time_constant_sec`。
