# ArduRover USV Agent Rules

## OVERVIEW

本目录承载 USV 定制 ArduRover 行为：mission script 采样触发、载荷 `NAMED_VALUE_FLOAT` 缓存、2 Hz 转发到 GCS、`USV_DONE` 闭环。

## WHERE TO LOOK

| 任务 | 位置 |
|---|---|
| 载荷字段缓存 | `GCS_MAVLink_Rover.cpp` 中 `MAVLINK_MSG_ID_NAMED_VALUE_FLOAT` |
| 载荷字段结构 | `Rover.h` 的 `usv_payload` |
| GCS 2 Hz 转发 | `sensors.cpp` 的 `usv_telemetry_send()` |
| mission script | `mode_auto.cpp` 的 `MAV_CMD_NAV_SCRIPT_TIME`、`USV_SMPL` |
| MAVLink 路由底层 | `../libraries/GCS_MAVLink/` |
| ROS 对端 | `../../src/usv_ros/scripts/usv_mavlink_router_bridge.py`、`mavlink_trigger_node.py` |

## CONVENTIONS

- 先遵守 `../AGENTS.md` 的 ArduPilot 安全和风格规则。
- 新增或改名载荷字段必须同步：`GCS_MAVLink_Rover.cpp`、`sensors.cpp`、`Rover.h`、ROS bridge、QGC FactGroup。
- `USV_SMPL` 由固件在 `NAV_SCRIPT_TIME(param1=1)` 触发；`USV_DONE` 由 ROS 回传。
- 只缓存和转发载荷状态；污染物浓度、历史记录、热力图不属于固件。

## ANTI-PATTERNS

- 把手动 `COMMAND_LONG 31010` 当作 mission item。
- 只在 ROS/QGC 改字段，不同步固件缓存和转发。
- 让通用 MAVLink routing 假定会稳定直转 `NAMED_VALUE_FLOAT` 到 GCS；当前采用固件缓存后转发。

## VERIFY

```bash
rg -n "USV_SMPL|USV_DONE|NAMED_VALUE_FLOAT|usv_payload|MAV_CMD_NAV_SCRIPT_TIME" GCS_MAVLink_Rover.cpp sensors.cpp Rover.h mode_auto.cpp
```
