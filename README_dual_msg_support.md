# PNG Planner Dual Message Support

本文档基于当前仓库实际内容整理，覆盖：

- 双检测消息支持方式
- 当前 `rp_*` 控制节点职责
- 当前 `rpy_*` 系列代码状态
- 公共参数与 launch/config 组织方式
- 可执行文件与 launch 文件对应关系

当前有效代码目录：

- 源码：`src/png_planner/src`
- launch：`src/png_planner/launch`
- 公共参数：`src/png_planner/config/rp_common.yaml`

补充说明：

- `rp_*` 仍然是当前已经注册到 `CMakeLists.txt` 并配好 launch 的主控制链路
- `rpy_*` 相关源码也已经整理到 `src/png_planner/src`，但目前主要作为源码级控制链路保留，尚未在当前 `CMakeLists.txt` 中注册为可执行文件

## 1. 双消息支持

当前 6 个 `rp_*` 节点都支持两类检测消息：

- `kcf_msgs/Bbox`
- `detection_msgs/Detection`

两类消息使用相同字段：

```text
x1
y1
x2
y2
score
class_name
track_id
is_tracking
```

统一参数：

- `detection_mode`
  - `0`：订阅 `kcf_msgs/Bbox`
  - `1`：订阅 `detection_msgs/Detection`
- `detection_topic`
  - 默认 `/object_kcf`
  - 当 `detection_mode=1` 且仍为 `/object_kcf` 时，代码会自动切到 `/object_detections`

当前默认值已统一放在：

- `src/png_planner/config/rp_common.yaml`

对应默认项：

```yaml
detection_mode: 0
detection_topic: /object_kcf
velocity_topic: /mavros/local_position/velocity_local
```

## 2. 当前节点总览

当前 `CMakeLists.txt` 实际注册并编译的控制节点是 6 个：

| 可执行文件 | 控制形式 | 高度控制来源 | 是否输出云台 pitch | 是否有速度阈值 |
| --- | --- | --- | --- | --- |
| `rp_png` | 平面 PNG + yaw PD | 云台 pitch LOS | 是 | 是 |
| `rp_los_pitch_vz` | 前飞 + yaw PD | 云台 pitch LOS | 是 | 是 |
| `rp_png_bbox_vz` | 平面 PNG + yaw PD | 检测框中心 `y` | 否 | 是 |
| `rp_los_bbox_vz` | 前飞 + yaw PD | 检测框中心 `y` | 否 | 是 |
| `rp_z_only_pitch_yaw_test` | 无水平位移，仅 `yaw + z` | 云台 pitch LOS | 是 | 否 |
| `rp_z_only_bbox_yaw_test` | 无水平位移，仅 `yaw + z` | 检测框中心 `y` | 否 | 否 |

补充：

- `src/png_planner/src/gimbal_keyboard_rp_test.cpp` 当前未注册到 `CMakeLists.txt`
- 当前仓库不再保留旧 `pixel_dotpng_*` 系列源码，主维护入口已经切到 `rp_*`

## 3. 各节点职责说明

### 3.1 `rp_png`

文件：

- `src/png_planner/src/rp_png.cpp`

特点：

- 水平面输出 `vx/vy`
- 水平控制是平面 PNG
- yaw 用 `k_yaw * lambda + k_yaw_d * lambda_dot`
- 云台 pitch 由识别框中心 `y` 生成
- 高度 `vz` 根据云台 pitch 形成的 LOS 估计生成
- 适合“平面主动机动 + 云台俯仰辅助控高”

### 3.2 `rp_los_pitch_vz`

文件：

- `src/png_planner/src/rp_los_pitch_vz.cpp`

特点：

- 水平面不输出侧向 `vy_body` 机动，只保持前飞
- 主要靠 yaw 调整朝向目标
- 云台 pitch 由识别框中心 `y` 生成
- 高度 `vz` 根据云台 pitch LOS 生成
- 适合“工程化前飞拦截 + 云台辅助控高”

### 3.3 `rp_png_bbox_vz`

文件：

- `src/png_planner/src/rp_png_bbox_vz.cpp`

特点：

- 水平面仍是平面 PNG
- 不发云台控制
- 高度直接由检测框中心 `y -> gamma -> vz`
- 适合云台处于对地自稳、机体自己控高的场景

### 3.4 `rp_los_bbox_vz`

文件：

- `src/png_planner/src/rp_los_bbox_vz.cpp`

特点：

- 水平面保持前飞 + yaw 对准
- 不发云台控制
- 高度直接由检测框中心 `y -> gamma -> vz`
- 适合“恒速前飞 + 无云台高度链路”的场景

### 3.5 `rp_z_only_pitch_yaw_test`

文件：

- `src/png_planner/src/rp_z_only_pitch_yaw_test.cpp`

特点：

- `linear.x = 0`
- `linear.y = 0`
- 保留 `angular.z`
- 保留云台 pitch 控制
- 高度 `vz` 根据云台 pitch LOS 生成
- 用于单独验证 `yaw + z + gimbal pitch`

### 3.6 `rp_z_only_bbox_yaw_test`

文件：

- `src/png_planner/src/rp_z_only_bbox_yaw_test.cpp`

特点：

- `linear.x = 0`
- `linear.y = 0`
- 保留 `angular.z`
- 不发云台控制
- 高度直接由检测框中心 `y -> gamma -> vz`
- 用于单独验证 `yaw + z`

## 4. LOS 与 Kalman 约定

当前 `rp_*` 节点统一使用：

- `src/png_planner/src/los_rate_kalman.h`

核心滤波器：

- `ScalarRateKalmanFilter`

典型用法：

```cpp
lambda_filter_.update(lambda_meas, dt);
double lambda = lambda_filter_.value();
double lambda_dot = lambda_filter_.rate();
```

垂直通道 `gamma` 也是同一套。

兼容参数仍保留：

- `lambda_filt_alpha`
- `lambda_window_size`
- `gamma_filt_alpha`
- `gamma_window_size`

但当前更推荐直接调这些参数：

- `lambda_kalman_q_position`
- `lambda_kalman_q_rate`
- `lambda_kalman_r`
- `gamma_kalman_q_position`
- `gamma_kalman_q_rate`
- `gamma_kalman_r`

## 5. 速度阈值功能

当前有速度阈值门控的节点：

- `rp_png`
- `rp_los_pitch_vz`
- `rp_png_bbox_vz`
- `rp_los_bbox_vz`

相关参数：

- `velocity_topic`
- `yaw_enable_speed_ratio`

逻辑：

- 节点订阅 `velocity_topic`
- 当前水平速度达到 `V_default * yaw_enable_speed_ratio` 之后，才允许 yaw 控制真正输出

当前没有速度阈值门控的节点：

- `rp_z_only_pitch_yaw_test`
- `rp_z_only_bbox_yaw_test`

## 6. 公共参数组织方式

当前重复调用参数已经统一放到：

- `src/png_planner/config/rp_common.yaml`

主要分为几类。

### 6.1 检测与话题

- `detection_mode`
- `detection_topic`
- `velocity_topic`

### 6.2 相机与图像几何

- `fx`
- `fy`
- `cx`
- `cy`
- `bbox_center_is_offset`
- `y_up_positive`

### 6.3 机体 yaw 控制

- `k_yaw`
- `k_yaw_d`
- `max_yaw_rate`
- `yaw_enable_speed_ratio`

### 6.4 Kalman 滤波

- `lambda_kalman_q_position`
- `lambda_kalman_q_rate`
- `lambda_kalman_r`
- `gamma_kalman_q_position`
- `gamma_kalman_q_rate`
- `gamma_kalman_r`
- `lambda_filt_alpha`
- `lambda_window_size`
- `gamma_filt_alpha`
- `gamma_window_size`

### 6.5 高度控制

- `N_nav_z`
- `k_z_ff`
- `max_vz`
- `enable_height_control`
- `pixel_thr_y`
- `v_center_alpha`

### 6.6 云台控制

- `gimbal_enable`
- `gimbal_device`
- `gimbal_pitch_bias_deg`
- `gimbal_pitch_gain`
- `gimbal_pitch_sign`
- `gimbal_pitch_kp`
- `gimbal_pitch_ki`
- `gimbal_pitch_kd`
- `gimbal_pitch_i_limit_deg`
- `gimbal_pitch_d_alpha`
- `gimbal_pitch_integral_decay`
- `gimbal_pitch_delta_limit_deg`
- `gimbal_pitch_limit_deg`
- `reset_gimbal_on_target_loss`
- `enable_gimbal_pitch_cmd_generation`
- `gimbal_pitch_los_sign`
- `gimbal_pitch_los_thr_deg`

### 6.7 其他公共参数

- `V_default`
- `N_nav`
- `pixel_thr`
- `use_real_dt`
- `step_dt`

说明：

- 不是每个节点都会使用 `rp_common.yaml` 中的全部参数
- 未使用的参数被加载也不会出问题
- 当前设计是“保持 launch 数量不变，但把可复用参数收口到一个公共 config”

## 7. Launch 对应关系

当前每个节点都有单独 launch，数量不收缩：

| 可执行文件 | launch |
| --- | --- |
| `rp_png` | `src/png_planner/launch/rp_png.launch` |
| `rp_los_pitch_vz` | `src/png_planner/launch/rp_los_pitch_vz.launch` |
| `rp_png_bbox_vz` | `src/png_planner/launch/rp_png_bbox_vz.launch` |
| `rp_los_bbox_vz` | `src/png_planner/launch/rp_los_bbox_vz.launch` |
| `rp_z_only_pitch_yaw_test` | `src/png_planner/launch/rp_z_only_pitch_yaw_test.launch` |
| `rp_z_only_bbox_yaw_test` | `src/png_planner/launch/rp_z_only_bbox_yaw_test.launch` |

这 6 个 launch 当前结构一致：

- 只负责启动对应节点
- 统一加载 `config/rp_common.yaml`

例如：

```xml
<arg name="common_config" default="$(find png_planner)/config/rp_common.yaml" />

<node pkg="png_planner" type="rp_png" name="rp_png" output="screen">
  <rosparam command="load" file="$(arg common_config)" />
</node>
```

## 8. 当前建议

如果后续还要继续演进，建议按下面方式维护：

- 主控节点优先维护 `rp_png` / `rp_los_pitch_vz`
- 无云台版本优先维护 `rp_png_bbox_vz` / `rp_los_bbox_vz`
- 单独测试 `yaw + z` 时使用两个 `rp_z_only_*` 文件
- 参数优先改 `config/rp_common.yaml`
- 若某个节点需要特殊参数，再在对应 launch 中单独覆盖

## 9. `rpy_*` 系列当前状态

### 9.1 文件位置与定位

原来位于 `src/png_planner/src/rpy` 的文件，已经上移到：

- `src/png_planner/src/rpy_pid_ff_ctrl.cpp`
- `src/png_planner/src/rpy_search_scan.cpp`
- `src/png_planner/src/rpy_los.cpp`
- `src/png_planner/src/rpy_los_no_alt.cpp`
- `src/png_planner/src/rpy_png.cpp`
- `src/png_planner/src/rpy_png_no_alt.cpp`
- `src/png_planner/src/FSM.cpp`
- `src/png_planner/src/target_sitl_sim.cpp`

当前定位是：

- `rpy_pid_ff_ctrl.cpp` / `rpy_search_scan.cpp` 负责云台角度链路
- `rpy_los*` / `rpy_png*` 负责基于云台角度的机体控制
- `FSM.cpp` / `target_sitl_sim.cpp` 负责任务调度或联调辅助

补充：

- 当前 `rpy_*` 文件已经不在单独的 `rpy/` 子目录下
- 当前仓库里的 launch 和 `CMakeLists.txt` 仍然只覆盖 `rp_*` 系列

### 9.2 `rpy_pid_ff_ctrl.cpp` 当前语义

这个文件现在已经改成 XF(A5) 串口协议直连控制，不再依赖旧的外部云台封装。

当前行为：

- 支持和 `rp_*` 相同的双检测消息入口
- `detection_mode=0` 时订阅 `kcf_msgs/Bbox`
- `detection_mode=1` 时订阅 `detection_msgs/Detection`
- 串口下发云台角度控制帧
- 串口解析云台反馈帧
- 继续通过原有话题发布当前云台角度

当前角度话题默认仍是：

- `/gimbal_angles`

当前 `/gimbal_angles` 语义：

- `angular.y`：对地 `pitch`
- `angular.z`：相对初始机头方向的 `yaw`

需要特别注意：

- 这里的 `yaw` 不是对地绝对航向角
- 机体 `yaw` 订阅只用于节点内部做补偿，让云台 `yaw` 控制更稳定地对准目标
- 对外发布的角度话题仍保持“`pitch` 对地、`yaw` 相对初始方向”的语义

### 9.3 `rpy` 云台模式约定

当前实现约定是：

- 云台保持默认地平线模式
- 串口只发送角度指令
- 不在节点里强制切换到 `FPV` 或跟随模式

这样做的原因是：

- `roll/pitch` 本身已经由云台地平线模式做对地自稳
- `yaw` 由于只有相对初始方向角，因此通过机体姿态订阅在控制节点内部做补偿更合适

### 9.4 `rpy_los*` / `rpy_png*` 的现状

这 4 个文件现在已经和 `rp_*` 系列对齐，使用同一套 LOS 角速度卡尔曼滤波实现：

- `rpy_los.cpp`
- `rpy_los_no_alt.cpp`
- `rpy_png.cpp`
- `rpy_png_no_alt.cpp`

统一依赖：

- `src/png_planner/src/los_rate_kalman.h`

核心滤波器：

- `ScalarRateKalmanFilter`

因此它们当前不再是旧的简单窗口差分口径，而是和 `rp_*` 一样，基于卡尔曼估计：

- `lambda`
- `lambda_dot`
- `gamma`
- `gamma_dot`

### 9.5 当前推荐理解方式

如果按主链路理解，`rpy_*` 当前最核心的数据流是：

`检测框 -> rpy_pid_ff_ctrl.cpp -> /gimbal_angles -> rpy_los.cpp / rpy_png.cpp`

搜索场景下也可以替换成：

`rpy_search_scan.cpp -> /gimbal_angles -> rpy_los*.cpp / rpy_png*.cpp`

如果你只是想快速确认 `rpy` 系列的现状，建议优先看：

- `src/png_planner/src/rpy_pid_ff_ctrl.cpp`
- `src/png_planner/src/rpy_los.cpp`
- `src/png_planner/src/rpy_los_no_alt.cpp`
- `src/png_planner/src/rpy_png.cpp`
- `src/png_planner/src/rpy_png_no_alt.cpp`
- `src/png_planner/src/README.md`
