# 固定相机 + INSLIB 版本说明

## 1. 这版代码改了什么

原来的PX4状态版本完整保留：

```bash
roslaunch png_planner fixed_camera_ibvs.launch
roslaunch png_planner radar_approach_intercept.launch
```

新增INSLIB版本：

```bash
roslaunch png_planner fixed_camera_ibvs_inslib.launch
roslaunch png_planner radar_approach_intercept_inslib.launch
```

两个版本使用同一个视觉检测接口、同一个固定相机控制器和同一套PN/FOV逻辑。区别只在状态来源：

- 原版读取PX4/MAVROS姿态、位置和速度；
- INSLIB版读取原始IMU、GNSS、气压计和可选磁力计，由`inslib_state_estimator`产生姿态、位置和速度；
- INSLIB状态无效、过期或姿态协方差过大时，视觉控制器输出零速度，不继续使用旧姿态；
- 雷达抵近阶段暂时仍使用原PX4局部位置，只有视觉拦截阶段切到INSLIB状态，避免一次改动两个尚未独立验证的坐标系。

INSLIB通过Git子模块固定版本。首次拉取后执行：

```bash
git submodule update --init --recursive
```

## 2. 数据流

```text
原始IMU / 原始GNSS / 气压计 / 可选磁力计
                    |
                    v
           inslib_state_estimator
          NED/FRD -> ENU/FLU转换
                    |
       +------------+-------------+
       |            |             |
/inslib/imu   /inslib/pose   /inslib/velocity
       |            |             |
       +------------+-------------+
                    |
           fixed_camera_ibvs
                    |
       PN + FOV约束 + yaw/速度指令
```

禁止把`/mavros/local_position/pose`或PX4融合后的局部速度作为INSLIB传感器输入，否则属于重复融合，无法证明姿态是由独立INSLIB估计得到的。

## 3. 坐标系

INSLIB内部使用：

- 世界系NED：北、东、下；
- 机体系FRD：前、右、下。

ROS/MAVROS控制器使用：

- 世界系ENU：东、北、上；
- 机体系FLU：前、左、上。

包装节点统一完成向量、姿态和协方差转换。四元数使用完整旋转矩阵转换，不能只交换四元数分量。

`outputs/world_yaw_alignment_deg`只用于INSLIB世界系与PX4局部世界系仍存在固定航向差时。正常GNSS NED和MAVROS ENU组合应保持为0；必须通过静态航向对准实验测量后才能修改。

## 4. 必须确认的输入话题

配置文件：`src/png_planner/config/inslib_fixed_camera.yaml`

| 参数 | 默认话题 | 含义 |
|---|---|---|
| `topics/imu` | `/mavros/imu/data_raw` | 原始角速度和比力，不读取其中姿态 |
| `topics/gnss_position` | `/mavros/global_position/raw/fix` | 原始GNSS经纬高和协方差 |
| `topics/gnss_velocity` | `/mavros/global_position/raw/gps_vel` | GNSS接收机速度，不能换成EKF局部速度 |
| `topics/magnetometer` | `/mavros/imu/mag` | 原始磁场，单位由ROS Tesla转换成INSLIB微特斯拉 |
| `topics/barometer` | `/mavros/imu/static_pressure` | 静压，单位Pa |

实机先执行：

```bash
rostopic list | grep mavros
rostopic hz /mavros/imu/data_raw
rostopic hz /mavros/global_position/raw/fix
rostopic hz /mavros/global_position/raw/gps_vel
rostopic echo -n 1 /mavros/imu/data_raw
```

如果原始GNSS速度话题不存在，应先确认飞控/MAVROS消息流配置，不能为了让节点启动就改成`/mavros/local_position/velocity_local`。

## 5. 参数怎么理解

### 5.1 IMU噪声

```yaml
imu:
  accelerometer_noise_density_mps2_sqrthz: 0.0
  gyroscope_noise_density_rps_sqrthz: 0.0
```

这两个量是噪声幅值密度，不是每帧标准差。INSLIB内部使用其平方作为PSD。当前0表示先用INSLIB的保守默认值；完成静置数据Allan方差分析后必须替换。

### 5.2 固定标定量

```yaml
calibration:
  accelerometer_fixed_bias_mps2: [0, 0, 0]
  gyroscope_fixed_bias_rps: [0, 0, 0]
  accelerometer_matrix_column_major: [0, ..., 0]
  gyroscope_matrix_column_major: [0, ..., 0]
```

INSLIB应用：

```text
corrected = M * (raw - fixed_bias)
```

全零矩阵在INSLIB中表示单位矩阵。这里的固定bias是实验标定出的永久偏置；滤波器内部bias状态用于估计飞行中剩余的慢变偏置，两者不是同一个参数。

可以使用INSLIB自带标定程序处理FRD格式的`imu.csv`：

```bash
python3 tools/inslib_imu_calib.py --csv DATASET_DIR -o config.yaml
```

输出矩阵为列主序，可填入本配置文件。

### 5.3 bias随机游走

```yaml
process:
  accelerometer_bias_random_walk_mps2_sqrthz: 0.0
  gyroscope_bias_random_walk_rps_sqrthz: 0.0
```

数值越大，滤波器允许bias变化得越快，但姿态也更容易被噪声扰动；数值越小，bias更稳定，但温漂跟踪更慢。当前0同样表示INSLIB默认值，最终需要静置、升温和电机振动数据共同确定。

### 5.4 GNSS延迟

```yaml
gnss:
  additional_delay_ms: 0
```

节点已经根据GNSS消息时间戳和当前IMU时间戳计算可见消息年龄。此参数只补偿GNSS接收机内部处理等没有体现在时间戳中的额外延迟。INSLIB历史缓冲最大处理500 ms；超过500 ms的样本会被丢弃而不是冒充当前测量。

延迟不能按“常见200 ms”直接填写，必须从实测数据估计。动态转弯时错误延迟会直接形成位置和航向偏差。

### 5.5 GNSS协方差和门限

- `position_covariance_scale`、`velocity_covariance_scale`：放大接收机报告的标准差；
- `*_stddev_floor_*`：防止接收机报告过度乐观；
- `start_max_*`：允许INSLIB进入完整导航状态的严格门限；
- `stop_max_*`：已经进入后允许继续工作的较宽门限；
- `init_dwell_sec`：GNSS质量必须连续合格的时间；
- `position_decimation`：GNSS位置和速度来自同一接收机且未知互相关时，位置降低融合频率，速度保持全频率。

默认门限只用于第一轮地面测试。必须根据所用GNSS接收机真实`eph/epv/speed accuracy`修改。

### 5.6 GNSS杆臂

```yaml
gnss:
  antenna_lever_arm_frd_m: [x_forward, y_right, z_down]
```

它是从IMU测量中心指向GNSS天线相位中心的FRD向量。快速转动时，天线和IMU存在不同线速度，杆臂填错会被滤波器误认为位置或速度误差。需要实际测量，不能长期保留全零占位值。

### 5.7 磁力计

磁力计默认关闭，因为固定相机最怕电机电流和机架金属导致yaw跳变。若启用，必须先完成整机状态下的硬铁/软铁标定，并更新`wmm_year`。不开磁力计时，初始yaw设为未知，INSLIB需要GNSS速度和机动逐渐约束航向；在航向方差降到门限前`/inslib/state_valid`保持false。

### 5.8 气压计

```yaml
barometer:
  altitude_stddev_m: 2.0
```

这是由气压换算高度的一阶标准差，不是压力传感器Pa噪声。气压计存在桨流动态压和天气漂移时应适当调大，并首先处理飞控安装位置和静压问题。

### 5.9 姿态质量门限

```yaml
outputs:
  max_attitude_stddev_deg: 5.0
```

只要roll、pitch、yaw任一1-sigma超过5度，INSLIB版本就判为不可用于闭环。这个5度是启动保护门限，不代表最终精度目标；固定相机实机验证应争取动态95%姿态误差不超过约1度。

## 6. INSLIB状态输出

| 话题 | 内容 |
|---|---|
| `/inslib/imu/data` | ENU/FLU姿态和原始角速度/加速度 |
| `/inslib/local_position/pose` | INSLIB局部ENU位置和姿态 |
| `/inslib/local_position/velocity_local` | INSLIB局部ENU速度 |
| `/inslib/odometry` | 合并后的里程计 |
| `/inslib/state_valid` | 控制器是否允许使用该状态 |
| `/inslib/mode` | `NONE/ATTITUDE_ONLY/COASTING/FULL` |
| `/inslib/attitude_stddev_deg` | roll、pitch、yaw的1-sigma角度 |

## 7. 第一次运行顺序

1. 只启动MAVROS和传感器，不启动飞行控制输出；
2. 确认所有原始话题存在、频率和单位正确；
3. 启动`inslib_state_estimator`，检查模式能否从`NONE`进入`FULL`；
4. 记录PX4状态和INSLIB状态的同一份rosbag；
5. 检查静态姿态、动态转动、时间延迟和惯性LOS残差；
6. 仿真和地面旋转试验通过后，才允许用`fixed_camera_ibvs_inslib.launch`闭环；
7. 原版PX4启动文件始终保留为基线和回退方案。

当前参数是可运行的首版配置，不是已经针对实机IMU完成的标定结果，禁止未经地面验证直接用于高速实机拦截。
