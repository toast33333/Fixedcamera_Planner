# rpy 系列文件说明

这些文件原本位于 `src/png_planner/src/rpy`，现在已经上移到 `src/png_planner/src`。
它们是一套基于 `roll / pitch / yaw` 云台角度话题的控制节点。

这套代码当前的核心特点：

- 云台控制节点使用 XF(A5) 串口协议
- 云台角度话题统一使用 `/gimbal_angles`
- `angular.y` 表示对地 `pitch`
- `angular.z` 表示相对初始方向的 `yaw`
- `rpy_los*` 和 `rpy_png*` 已经改成和 `rp_*` 系列同口径的卡尔曼滤波 LOS 估计

整体上可以分成 3 层：

1. 云台控制与角度发布层
2. 基于云台角度的机体控制层
3. 仿真 / 任务调度层

## 目录文件

### 1. 云台控制与角度发布

#### `rpy_pid_ff_ctrl.cpp`

作用：
- 订阅识别框消息
- 通过串口按 XF(A5) 协议向云台发送角度控制指令
- 从串口解析云台反馈
- 向外发布当前云台角度话题

当前角度话题语义：
- `angular.y`：云台对地 `pitch`
- `angular.z`：云台相对初始方向的 `yaw`

控制特点：
- `pitch` 根据图像中心 `y` 偏差做 PID
- `yaw` 根据图像中心 `x` 偏差做 PID
- 可订阅机体姿态，在本节点内部做机体 `yaw` 补偿
- 只发送角度控制量，不主动切换云台工作模式
- 串口返回帧会反解当前云台角度，并继续发布到 `/gimbal_angles`

典型输入：
- `/object_kcf` 或 `/object_detections`
- `/mavros/local_position/pose`

典型输出：
- `/gimbal_angles`
- 串口云台控制帧

#### `rpy_search_scan.cpp`

作用：
- 在无目标时驱动云台做搜索扫描
- 发布扫描过程中的云台角度到角度话题

控制特点：
- 按预设 `pitch` 层级和 `yaw` 往返扫动
- 适合搜索阶段使用

典型输出：
- `/gimbal_angles`

说明：
- 这个文件是“搜索角度发生器”
- 它和 `rpy_pid_ff_ctrl.cpp` 都会发布云台角度话题，但用途不同
- 正常运行时一般不要让两个节点同时控制同一个云台

### 2. 基于云台角度的机体控制

这一层节点都把 `/gimbal_angles` 当作输入。
也就是说，它们默认不直接控制云台，而是把云台当前角度当作 LOS / PNG 引导量来源。

#### `rpy_los.cpp`

作用：
- LOS 水平面控制
- 同时根据云台 `pitch` 做垂向速度控制

控制特点：
- 水平面：前飞 + `yaw` PD
- 垂向：由 `pitch` LOS 生成 `vz`
- 目标丢失时保持前飞，并冻结 `yaw`
- `lambda / gamma` 及其角速度已改成卡尔曼滤波估计，不再使用简单窗口差分

适用场景：
- 需要持续前飞
- 需要简单稳定的 LOS 引导

#### `rpy_los_no_alt.cpp`

作用：
- LOS 水平面控制
- 不做高度控制

控制特点：
- 水平面：前飞 + `yaw` PD
- `z = 0`
- 目标丢失时保持前飞，并冻结 `yaw`
- `lambda` 与 `lambda_dot` 已改成卡尔曼滤波估计

适用场景：
- 只想测试水平 LOS
- 高度由其他模块控制

#### `rpy_png.cpp`

作用：
- PNG 水平面控制
- 同时根据云台 `pitch` 做垂向速度控制

控制特点：
- 水平面：前向速度 + 横向速度 PNG
- 垂向：由 `pitch` 导引生成 `vz`
- 目标丢失时保持前飞，横向速度归零
- `lambda / gamma` 及其角速度已改成卡尔曼滤波估计

适用场景：
- 需要更强的横向机动能力
- 需要完整三维速度指令

#### `rpy_png_no_alt.cpp`

作用：
- PNG 水平面控制
- 不做高度控制

控制特点：
- 水平面：前向速度 + 横向速度 PNG
- `z = 0`
- 目标丢失时保持前飞，横向速度归零
- `lambda` 与 `lambda_dot` 已改成卡尔曼滤波估计

适用场景：
- 只测试水平 PNG
- 高度由其他控制器负责

### 3. 仿真 / 任务调度

#### `target_sitl_sim.cpp`

作用：
- 在 SITL / 联调环境中构造一个目标
- 发布目标 GPS、速度、局部位姿、路径和可视化 marker

适用场景：
- 没有真实目标源时做联调
- 给 FSM 或其他追踪逻辑提供目标运动输入

#### `FSM.cpp`

作用：
- 更高层的任务状态机
- 根据目标距离、速度、锁定情况，在不同阶段之间切换

当前定位：
- 它不是单纯的“云台控制节点”
- 它更像任务决策 / 模式切换层

适用场景：
- 需要“接近-搜索-锁定-进入攻击”这一类完整流程时使用

## 文件之间的关系

最常用的链路是：

`检测框 -> rpy_pid_ff_ctrl.cpp -> /gimbal_angles -> rpy_los.cpp / rpy_png.cpp`

也就是：

1. `rpy_pid_ff_ctrl.cpp` 负责把识别框变成云台角度
2. 云台反馈通过 `/gimbal_angles` 对外发布
3. `rpy_los*.cpp` 或 `rpy_png*.cpp` 再根据这个角度生成机体速度指令

这一条链路是当前最核心的主链路。

搜索阶段的替代链路是：

`rpy_search_scan.cpp -> /gimbal_angles -> rpy_los*.cpp / rpy_png*.cpp`

也就是：

1. 没有目标时，`rpy_search_scan.cpp` 人工生成搜索角度
2. 下游机体控制节点仍然只认 `/gimbal_angles`

更高层如果需要任务切换：

`target_sitl_sim.cpp / 实际目标源 -> FSM.cpp -> 其他控制节点`

## 推荐理解方式

可以把这些文件理解成下面几类：

- `rpy_pid_ff_ctrl.cpp`
  云台伺服节点，负责“看见目标后怎么转云台”

- `rpy_search_scan.cpp`
  云台搜索节点，负责“没目标时怎么扫云台”

- `rpy_los.cpp` / `rpy_los_no_alt.cpp`
  机体 LOS 引导节点，负责“拿到云台角后机体怎么飞”

- `rpy_png.cpp` / `rpy_png_no_alt.cpp`
  机体 PNG 引导节点，负责“拿到云台角后机体怎么机动”

- `FSM.cpp`
  任务流程节点，负责“什么时候搜索、什么时候切模式”

- `target_sitl_sim.cpp`
  联调 / 仿真辅助节点

## 当前建议

### 如果你想快速看总结

推荐先看这 5 个文件：

- `rpy_pid_ff_ctrl.cpp`
- `rpy_los.cpp`
- `rpy_los_no_alt.cpp`
- `rpy_png.cpp`
- `rpy_png_no_alt.cpp`

可以这样理解：

- `rpy_pid_ff_ctrl.cpp`
  负责“视觉框 -> 云台角度”

- `rpy_los*.cpp`
  负责“云台角度 -> LOS 机体控制”

- `rpy_png*.cpp`
  负责“云台角度 -> PNG 机体控制”

其中 `LOS / PNG` 这 4 个机体控制文件现在都已经用了卡尔曼滤波。

如果你当前目标是“真实云台 + 视觉目标居中 + 机体根据云台角跟踪”，建议优先关注：

- `rpy_pid_ff_ctrl.cpp`
- `rpy_los.cpp`
- `rpy_los_no_alt.cpp`
- `rpy_png.cpp`
- `rpy_png_no_alt.cpp`

如果你当前目标是“先把搜索链路跑通”，建议优先关注：

- `rpy_search_scan.cpp`
- `rpy_los_no_alt.cpp` 或 `rpy_png_no_alt.cpp`

## 协议文档

当前同级目录下还放了两份参考文档：

- `云台私有协议-XF(A5)V1.0.3.pdf`
  串口协议定义，重点是控制帧与返回帧格式

- `C-40T三轴云台用户手册-XF(A5)V1.1.pdf`
  用户手册，重点是云台模式、安装和使用说明

如果后面继续改串口控制或角度语义，应优先参考这两份 PDF。

## 当前状态总结

截至目前，这套 rpy 系列代码可以概括成：

- `rpy_pid_ff_ctrl.cpp`
  负责云台串口控制和角度反馈发布

- `rpy_search_scan.cpp`
  负责无目标时的搜索扫描

- `rpy_los.cpp` / `rpy_los_no_alt.cpp`
  负责基于云台角度的 LOS 控制

- `rpy_png.cpp` / `rpy_png_no_alt.cpp`
  负责基于云台角度的 PNG 控制

- `FSM.cpp`
  负责更高层任务流程

- `target_sitl_sim.cpp`
  负责联调 / 仿真目标生成

