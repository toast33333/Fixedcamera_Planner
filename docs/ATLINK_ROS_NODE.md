# AT-Link v3.2 MAVROS 与机载状态节点

节点通过串口接收 AT-Link v3 地面端帧。有效的 `0x0101` 起飞命令会先请求 `/mavros/cmd/arming` 解锁，再请求 `/mavros/cmd/takeoff` 起飞。协议载荷中的目标/返航高度不是起飞高度；起飞相对高度单独由 `takeoff_altitude`（米）配置，默认 5 米。

## 启动

```bash
roslaunch png_planner atlink_ros_node.launch \
  serial_device:=/dev/ttyS6 baud:=57600 local_address:=1 takeoff_altitude:=5.0
```

RK3576 容器必须映射或以 `privileged` 模式访问宿主机串口。当前默认 AT-Link
设备为 `/dev/ttyS6`、波特率为 `57600`；也可以通过环境变量 `ATLINK_DEVICE`
覆盖设备路径。飞控串口与 AT-Link 串口彼此独立，AT-Link 逻辑地址需与地面端
目标地址一致。

节点监听MAVROS状态、扩展状态、全球位置、相对高度、电池和本地速度。只有飞控
连接且GPS有效、飞机确认在地面时才执行起飞流程。服务调用失败会通过AT-Link
应答回传失败；起飞成功以MAVROS后续报告离地状态为准，协议库据此结束起飞请求。

节点保持原有 AT-Link 指令和状态输出，并在 `0x0202.fault_code` 中补充 MAVROS
时间同步延迟、视觉话题超时和 MAVROS 状态超时告警；视觉扩展信息通过 `0x0203`
发送。协议字段和兼容边界见 `src/png_planner/vendor/atlink/README.md`。

可通过 launch 参数覆盖默认 MAVROS 接口、健康阈值和视觉话题。外场调试前至少
确认 `/dev/ttyS6` 的收发计数、`/mavros/state.connected` 和地面端点名回包正常。
