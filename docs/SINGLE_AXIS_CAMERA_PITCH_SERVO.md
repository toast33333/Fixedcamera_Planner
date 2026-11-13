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

当前已提供不依赖硬件引脚的开环换算节点：

```bash
roslaunch png_planner camera_pitch_servo_open_loop.launch
```

它把角度指令限速后换算为 `/camera_pitch_servo/pulse_width_us`
（`std_msgs/UInt16`）。默认使用50 Hz，并以1500 us严格对应机械中位0度。
`camera_pitch_servo_calibration_mode`可选择两套角度到脉宽关系：`measured`使用实物
标定`P = 1500 + 10.132249 * theta`，`datasheet`使用规格书500 us对应45度的
`P = 1500 + 11.111111 * theta`。默认仍为`measured`。随后把角度限制在-48度到
+48度、把脉宽限制在1000 us到2000 us。
该节点本身不访问RK3576的PWM寄存器或`pwmchip`。

为避免节点启动或驱动晚启动时产生意外动作，当前开环层采用两级安全门：

1. `camera_pitch_servo_output_enabled` 默认是 `false`，启动后不会发布任何 PWM 脉宽；
2. 显式使能后，节点仍会丢弃使能前的旧指令，必须收到一帧新的角度指令才开始发布。

使能服务为：

```bash
rosservice call /camera_pitch_servo/set_output_enabled "data: true"
```

停止输出为：

```bash
rosservice call /camera_pitch_servo/set_output_enabled "data: false"
```

`/camera_pitch_servo/pulse_width_us` 不再锁存；硬件驱动后启动时不会收到旧脉宽。
`/camera_pitch_servo/output_enabled` 表示安全门状态，
`/camera_pitch_servo/command_valid` 表示角度指令是否仍在超时时间内。

当前同时提供独立的 Linux sysfs PWM 适配节点：

```bash
roslaunch png_planner camera_pitch_servo_sysfs_pwm.launch
```

也可以一次启动角度换算层和硬件层；二者仍然保持关闭，不会输出PWM：

```bash
roslaunch png_planner camera_pitch_servo_hardware.launch
```

仅启动该节点不会导出、配置或使能 PWM。硬件层还需要显式调用第二个服务：

```bash
rosservice call /camera_pitch_servo/set_hardware_output_enabled "data: true"
```

硬件层武装以后，必须让开环换算层经历一次“关闭—重新打开”，然后再发送新的
角度指令；这样可防止硬件层把开环节点仍在周期发布的旧目标当成新指令。完成这
个握手并收到新的非锁存脉宽后，硬件层才会访问 sysfs。任意一层关闭都会停止并
释放 PWM。状态话题为
`/camera_pitch_servo/hardware_armed` 和 `/camera_pitch_servo/hardware_active`。

### GPIO0_B4_D硬件映射

板上舵机信号线接在 `GPIO0_B4_D`。RK3576中该针脚对应：

```text
GPIO编号：GPIO0_B4 = 12
PWM复用：PWM1_CH0_M0
设备树控制器：pwm1_6ch_0
OF节点：/pwm@2add0000
PWM通道：0
```

2026-10-24实机复核结果是：`uart4=disabled`、`pwm1_6ch_0=okay`，目标控制器
已经注册为`/sys/class/pwm/pwmchip0`且OF路径为`/pwm@2add0000`；原屏幕背光PWM
顺延为`pwmchip1`。对应的板级DTS互斥配置为：

```dts
&uart4 {
    status = "disabled";
};

&pwm1_6ch_0 {
    pinctrl-names = "active";
    pinctrl-0 = <&pwm1m0_ch0>;
    status = "okay";
};
```

硬件适配节点不依赖 `pwmchipN` 的动态编号，而是逐个核对
`device/uevent` 中的 `OF_FULLNAME=/pwm@2add0000`；找不到时拒绝输出。

硬件层还带有独立的PWM消息看门狗，默认超时为0.3秒。硬件输出已经激活后，
如果换算节点退出或PWM消息流中断，硬件层会自动撤销武装、关闭并释放该通道。

RK3576的Rockchip PWM通道每次重新export后采用完整初始化顺序：先关闭输出并把
`duty_cycle`清零，再设置周期和`normal`极性，随后写入目标脉宽并使能。节点还会
回读`enable/period/polarity/duty_cycle`；任一状态不匹配时立即关闭并释放通道。
这是为了避免sysfs数值看似正确、物理引脚却没有恢复有效波形的情况。

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

控制器先使用检测时间戳对应的IMU姿态和舵机角把视觉LOS变换到世界系，再用
当前飞机姿态和当前舵机角得到当前机体系LOS。随后把当前LOS与期望LOS都投影到
实际舵机铰链轴的法平面，计算绕该轴可实现的有符号角误差：

```text
e_axis = atan2(a · (l_perp × d_perp), l_perp · d_perp)
```

其中`a`是机体系中的实际舵机转轴，`l_perp`和`d_perp`分别是当前LOS和期望LOS
在转轴法平面上的投影。这样飞机滚转、俯仰、偏航，以及相机固定安装角和转轴
安装偏差都会进入舵机控制方向。关闭转轴补偿时，才退回原来的图像竖直角误差：

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
- `camera_pitch_servo_axis_compensation_enabled`：是否使用三维LOS到实际转轴的投影误差。
- `camera_pitch_servo_axis_mount_x/y/z`：中立安装坐标系中的物理铰链轴方向，默认`[0,1,0]`。
- `camera_pitch_servo_pwm_min_us/neutral_us/max_us`：S001M脉宽标定值。
- `camera_pitch_servo_calibration_mode`：选择`measured`实测标定或`datasheet`规格书标定。
- `camera_pitch_servo_measured_pwm_us_per_deg`：实测标定斜率，当前为10.132249 us/deg。
- `camera_pitch_servo_datasheet_pwm_us_per_deg`：规格书斜率，当前为500/45 us/deg。
- `camera_pitch_servo_output_enabled`：启动时输出安全门，实机保持 `false`。
- `camera_pitch_servo_enable_service`：运行中显式打开/关闭输出的 `SetBool` 服务。
- `camera_pitch_servo_pwm_reverse`：舵机机械安装方向反相。
- `camera_pitch_servo_driver_max_rate_degps`：开环PWM层的二次安全限速。
- `camera_pitch_servo_command_timeout_sec`：角度指令超时回中时间。
- `camera_pitch_servo_pwm_of_fullname`：GPIO0_B4_D对应的PWM控制器OF路径。
- `camera_pitch_servo_pwm_polarity`：舵机PWM的电气极性，当前固定为`normal`。
- `camera_pitch_servo_pwm_channel`：控制器内通道，本连接固定为0。
- `camera_pitch_servo_hardware_command_timeout_sec`：PWM消息中断后的硬件关闭超时。
- `camera_pitch_servo_hardware_enable_service`：硬件层独立安全门服务。
- `camera_pitch_servo_hardware_armed_topic`：硬件层已武装状态。
- `camera_pitch_servo_hardware_active_topic`：sysfs PWM实际已使能状态。

首次接实物时，先将角度范围临时限制为 `-5°~+5°`，飞机不上桨并固定机身；确认舵机使用独立 3.7--4.2 V 电源且与 RK3576 共地。先保持输出门关闭检查话题，再由现场人员明确同意后使能。用手上下移动目标确认：目标向画面上方移动时，指令应为正并使相机上抬。符号确认后再逐步放宽机械范围。

设备树映射虽已就绪，首次实物动作仍必须在飞机不上桨、机身固定、舵机独立供电
并与RK3576共地的条件下进行；完成角度—PWM标定前保持两级输出门关闭。

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
