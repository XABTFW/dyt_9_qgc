# DYT 导引头：地面站发送接口

本文档只说明地面站如何向 DYT 导引头相关模块发送命令。

## 1. 地面站通过 MAVLink 发送

### 1.1 中制导/末制导切换

发送 `DYT_GUIDANCE_COMMAND`，消息 ID `12925`。

| 字段 | 填写方式 |
| --- | --- |
| `request_id` | 非零递增请求号 |
| `target_system` | 目标飞机 sysid，多机时不能填 0 |
| `target_component` | 飞控组件，通常为 1 |
| `phase` | 2=中制导，3=末制导 |

发送后用 `DYT_SYSTEM_STATUS`（12926）确认：

- `command_sequence == request_id`
- `command_result`：1=处理中，2=接受，3=拒绝，4=失败
- 最终状态读取 `guidance_phase` 和 `guidance_state`

当前 QGC 发送函数：

```cpp
Vehicle::sendDytGuidanceCommand(int phase)
```

### 1.2 视频画面点选跟踪

正式功能发送 `DYT_TRACK_POINT_COMMAND`，消息 ID `12935`。

| 字段 | 填写方式 |
| --- | --- |
| `request_id` | 非零递增请求号 |
| `x_px` | 原始图像左上角坐标系的 X 像素 |
| `y_px` | 原始图像左上角坐标系的 Y 像素 |
| `image_width_px` | 原始图像宽度 |
| `image_height_px` | 原始图像高度 |
| `target_system` | 目标飞机 sysid |
| `target_component` | 飞控组件，通常为 1 |

发送后的判断顺序：

1. 接收 `DYT_TRACK_POINT_ACK`（12936）。
2. 12936 返回 `ACCEPTED` 只表示飞控接收成功。
3. 真正锁定必须检查 `DYT_TARGET_STATUS`（12927）：

```text
tracking_state == 1
target_flags bit0 == 1
```

当前 QGC 的 `Vehicle::sendDytTrackPointCommand()` 仍使用 Shell 发送 `dyt_gimbal trackxy`，应改成结构化发送 12935，否则无法正确匹配 12936 ACK。

### 1.3 导引头参数设置

导引头参数使用 QGC 标准 MAVLink 参数协议设置，不要通过 Shell 拼接 `param set`。

| 参数 | 作用 |
| --- | --- |
| `DYTG_AUTO_EN` | 1=检测到目标后自动锁定，0=不自动锁定 |
| `DYTG_AUTO_N` | 自动锁定前连续检测帧数 |
| `DYTG_LOCK_N` | 进入跟随前有效锁定次数 |
| `DYTG_LOCK_MS` | 锁定命令保护和重试间隔 |
| `DYT_TRK_VAL` | 导引头原始锁定状态值 |

## 2. 地面站通过 Shell 发送

以下导引头控制目前没有对应的结构化 MAVLink 消息。地面站需要这些功能时，通过 `SERIAL_CONTROL(126)` 向 PX4 Shell 发送文本，末尾必须带 `\n`。

| 导引头功能 | Shell 文本 |
| --- | --- |
| 自动锁定画面中心 | `dyt_gimbal autolock\n` |
| 停止跟踪 | `dyt_gimbal stoptrk\n` |
| 设置云台角度 | `dyt_gimbal angle <yaw_deg> <pitch_deg>\n` |
| 激光关闭 | `dyt_gimbal laser off\n` |
| OSD 开启 | `dyt_gimbal osd on\n` |
| OSD 关闭 | `dyt_gimbal osd off\n` |
| 可见光 | `dyt_gimbal imagemode vis\n` |
| 红外 | `dyt_gimbal imagemode ir\n` |
角度  dyt_gimbal angle 滚转角度 俯仰角度

点选跟踪可以临时通过 Shell 调试：

```text
dyt_gimbal trackxy <centered_x> <centered_y>\n
```

这里是以图像中心为原点的有符号坐标，`0 0` 为画面中心。正式 QGC 点选功能必须使用 MAVLink 12935。

查看导引头驱动状态使用：

```text
dyt_gimbal status\n
```

## 3. 地面站不需要发送

| 导引头动作 | 处理方式 |
| --- | --- |
| 解锁后启动目标检测 | PX4 在每次未解锁→解锁时自动发送固定帧 |
| 自动锁定重试 | `DYTG_AUTO_EN=1` 时由 PX4 自动执行 |
| 锁定后进入 `TRACK_FOLLOW` | 满足锁定条件后由 PX4 自动进入 |
| 失锁后重新捕获 | 由 PX4 导引状态机处理 |

解锁后 PX4 自动发送的导引头帧为：

```text
55 AA 0A 04 2B 00 01 00 00 00 00 00 00 DB
```

QGC 不要重复发送这条帧，也不要为它增加 Shell 命令。

## 4. 地面站接收的必要状态

| 消息 | 用途 |
| --- | --- |
| `DYT_SYSTEM_STATUS`（12926） | 制导切换结果、当前跟随/拦截状态 |
| `DYT_TARGET_STATUS`（12927） | 检测、锁定、目标有效性和云台状态 |
| `DYT_STATUS_REPLY`（12928） | 导引头原始回复，调试使用 |
| `DYT_TRACK_POINT_ACK`（12936） | 点选跟踪请求接收结果 |

`DYTG_AUTO_EN=1` 时，不拨末制导激活杆，检测并锁定成功后也会自动进入 `TRACK_FOLLOW`。进入 `TRACK_INTERCEPT` 仍需要拦截请求条件。
