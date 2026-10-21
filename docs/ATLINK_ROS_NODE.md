# AT-Link v3 MAVROS 起飞节点

节点通过串口接收 AT-Link v3 地面端帧。有效的 `0x0101` 起飞命令会先请求 `/mavros/cmd/arming` 解锁，再请求 `/mavros/cmd/takeoff` 起飞。协议载荷中的目标/返航高度不是起飞高度；起飞相对高度单独由 `takeoff_altitude`（米）配置，默认 5 米。

## 启动

```bash
roslaunch png_planner atlink_ros_node.launch \
  serial_device:=/dev/ttyUSB0 baud:=115200 local_address:=1 takeoff_altitude:=5.0
```

RK3576 容器使用 `privileged` 模式时可直接访问主机串口，默认 AT-Link 设备为
`/dev/ttyUSB0`。节点在设备尚未插入或USB重新枚举时不会退出，而是定时重连。
飞控USB串口与AT-Link串口彼此独立：当前默认飞控为 `/dev/ttyACM0`，AT-Link为
`/dev/ttyUSB0`。AT-Link逻辑地址需与地面端目标地址一致。

节点监听MAVROS状态、扩展状态、全球位置、相对高度、电池和本地速度。只有飞控
连接且GPS有效、飞机确认在地面时才执行起飞流程。服务调用失败会通过AT-Link
应答回传失败；起飞成功以MAVROS后续报告离地状态为准，协议库据此结束起飞请求。

可通过 `arming_service`、`takeoff_service`、`state_topic`、`extended_state_topic` 和 `gps_topic` launch 参数覆盖默认 MAVROS 接口。

