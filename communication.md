
# 速度指令通信协议说明

## 1. 协议概述

本协议是一个用于传输机器人或载体速度指令（线速度和角速度）的**定长二进制协议**。

- **总长度**：固定为 **9个字节**。
- **字节序**：固定小端序（Little-Endian）。
- **坐标**：当前车身坐标，`+vx` 向前、`+vy` 向左、`+wz` 逆时针；旋转期间也使用该约定。
- **数据缩放**：速度数据均放大了 1000 倍进行传输，以提高精度并避免浮点数传输。

## 2. 帧结构定义

| 字段           | 数据类型 | 字节数 | 描述/缩放                               | 单位  | 数值范围 |
| :------------- | :------- | :----- | :-------------------------------------- | :---- | :------- |
| **帧头** | uint8    | 1      | 固定为`0xAA`                          | —    | —       |
| **长度** | uint8    | 1      | 固定为`0x06`（代表后续6字节有效数据） | —    | —       |
| **vx**   | int16    | 2      | 实际值 × 1000                          | m/s   | ±32.767 |
| **vy**   | int16    | 2      | 实际值 × 1000                          | m/s   | ±32.767 |
| **wz**   | int16    | 2      | 实际值 × 1000                          | rad/s | ±32.767 |
| **校验** | uint8    | 1      | 前 8 字节逐位异或（XOR）                | —    | —       |

## 3. 核心逻辑说明

### 3.1 数据缩放与物理极限

- 传输值 = 实际物理值 × 1000。例如：实际 `vx = 1.5 m/s`，传输值为 `1500`。
- 由于使用了 `int16`（有符号16位整数，最大值为 32767），因此：
  - 线速度 `vx` / `vy` 的最大量程为 **±32.767 m/s**。
  - 角速度 `wz` 的最大量程为 **±32.767 rad/s**。

### 3.2 校验算法

- 采用简单高效的逐位异或（XOR）校验。
- **校验范围**：第1至第8字节（包含帧头 `0xAA`、长度 `0x06` 以及 `vx`、`vy`、`wz` 的6个字节）。
- **校验逻辑**：发送端将前8字节进行异或，结果作为第9个字节发送。接收端收到后重新计算前8字节异或值，与第9字节比对，若不匹配则视为数据损坏。

## 4. 数据帧示例

**向前运动示例：** `vx = 1.0 m/s`, `vy = 0 m/s`, `wz = 0 rad/s`

1. **帧头**：`AA`
2. **长度**：`06`
3. **vx (1.0 * 1000 = 1000)**：十六进制 `03 E8` （小端序示例：`E8 03`）
4. **vy (0)**：`00 00`
5. **wz (0)**：`00 00`
6. **校验计算**：`AA ^ 06 ^ E8 ^ 03 ^ 00 ^ 00 ^ 00 ^ 00 = 47`
7. **最终发送完整报文**：`AA 06 E8 03 00 00 00 00 47`

## 5. 本工程接收端

- 接口：UART7，`huart7`，921600 baud，8N1；MCU RX 为 PE7，TX 为 PE8。
- 上位机 TX 接 MCU PE7，并共地；本协议仅接收，不发送应答。
- `app_robot.c` 的公共初始化注册 `HostVelocityProtocolRxCallback`，通过现有 DMA + 空闲中断接收。
- 接收模块为 `Skywalker/Application/Src/app_host_protocol.c`，支持拆包、多帧连续接收、噪声过滤和校验失败后重新同步。
- 仅校验通过的帧更新速度、接收时间和有效帧计数，UART7 速度现已用于控制底盘。
- 接收端完整支持 int16 范围，包含 `-32768`（对应 `-32.768`）。

任务中包含 `app_host_protocol.h` 后，可读取最新速度，例如：

```c
HostVelocityCommand command;
if (HostVelocityProtocolGetCommand(&command, 100U)) {
  // command.vx / command.vy: m/s，command.wz: rad/s。
  // command.last_update_ms: HAL_GetTick() 毫秒时间戳。
  // command.frame_count: 累计有效帧数。
}
```

`100U` 表示允许的最大帧龄为 100 ms，可按上位机发送周期调整。未收到有效帧或超时时返回 `false`，并将输出结构清零；错误帧不会刷新时间戳。读取接口在短临界区内复制完整样本，避免串口中断更新时读到混合数据。

`HostVelocityCommand.raw_frame` 保存同一有效帧的全部 9 个原始字节。诊断时可用 `HostVelocityProtocolGetLatestCommand(&command)` 读取最新有效帧（包含超时后的旧帧）；未收到过有效帧时返回 `false` 并清零输出。

UART1 当前输出 GM6020 PID 调参的 20 通道 FireWater 数据，其中通道 19 表示控制来源（0：DT7，1：UART7），不再输出旧版 UART7 原始帧/IMU 32 通道。协议诊断可读取 `HostVelocityProtocolGetLatestCommand()`。通道顺序见 `Skywalker/Application/FireWater_telemetry.md`。

## 6. 底盘控制

- DT7 在线时优先使用遥控器/键鼠，UART7 不参与底盘控制；DT7 从未收到帧或最后一帧超过 `kRobotDt7OfflineTimeoutMs = 100` ms 时视为离线，才允许 UART7 接管。DT7 恢复在线后自动切回遥控器，无需重新编译。
- `kRobotChassisHostControlEnabled = 1U`：允许 DT7 离线时使用 UART7 最新有效速度。设为 `0U` 时禁用上位机接管，DT7 离线即停车。切换来源时清除旧 PID、舵角与旋转参考，重新从当前反馈起步。
- UART7 上位机平移指令始终使用当前车身坐标，包括旋转期间：正 `vx` 指向当前车头，正 `vy` 指向当前车左，正 `wz` 为逆时针。帧内第 3–4 字节（索引 2–3）解析为 `command.vx`，第 5–6 字节（索引 4–5）解析为 `command.vy`，不交换、不取反。速度已经是 SI 单位，不使用摇杆归一化比例，也不叠加遥控器、键鼠或自动旋转输入。
- 每 2 ms 执行底盘控制；指令分别限幅到 vx ±5 m/s、vy ±5 m/s、wz ±10 rad/s，随后沿用 IMU 角速度补偿和四轮等比例限速（轮速上限 5 m/s）。
- 默认启用 `kRobotChassisSpinTranslationEnabled = 1U`：IMU 有效且 `|wz| > 0.0001 rad/s` 时，启用小陀螺平移处理。UART7 的速度已经是当前车身坐标，直接进入四轮解算，只用测量角速度乘 5 ms 得到预测转角，不再使用累计航向旋转速度分量。遥控器/键鼠始终以**当前云台朝向**为参考，通过 USART10 收到的实时相对 yaw 转换到当前车身坐标，不再使用累计航向转换。上位机应按新的前/左坐标发送车身速度；场地坐标速度应先在发送端转换成当前车身坐标。
- 小陀螺降速按坐标转换前的输入选择：横向 `vy` 输入非零时两轴速度乘 0.4，仅纵向 `vx` 输入非零时乘 0.3，避免同一输入的降速比例随航向变化。仍使用上位机给出的 `wz`，不自动增加旋转速度。将 `kRobotChassisSpinTranslationEnabled` 设为 `0U` 可关闭 5 ms 前馈和这项平移降速；遥控的云台相对角转换仍启用。
- `wz` 归零、静止、超时或安全停车会解除参考锁定；下一次旋转重新记录参考航向。IMU 不可用时，带非零 `wz` 的平移被置零，旋转指令保留；`wz=0` 时仍使用原车身坐标平移。运行中 IMU 读取失败会重试，恢复后重新记录参考航向；初始化或标定失败需要重新启动检查。协议帧仍为 9 字节，没有新增模式字段；所有非零旋转指令均按此规则处理。
- 保留四轮舵角对齐联锁、电机反馈超时检查、PID 与电流限制。三个速度均为零时发送零电流。
- DT7 离线且 UART7 未收到有效帧，或最后有效帧超过 `kRobotHostCommandTimeoutMs = 100` ms 时，清零目标并通过 FDCAN3 的 `0x200` 和 FDCAN1 的 `0x1FE` 发送零电流停车。建议上位机每 10–20 ms 发一帧，坏帧不刷新超时。
- 若启用已有选项 `kRobotRemoteSwitchSafeModeEnabled`，在线 DT7 的 sw1 下拨会停车，并保持遥控器优先，不允许 UART7 绕过此停车条件（当前默认该选项为 `0U`）。
- 收到有效 UART7 数据不代表取得控制权；DT7 在线时优先遥控。可通过 FireWater 通道 19 查看当前控制来源；UART7 帧龄和解析速度通过 `HostVelocityProtocolGetLatestCommand()` 检查。


## 7. 上位机发送与坐标迁移

报文顺序固定为：`AA 06 vx_lo vx_hi vy_lo vy_hi wz_lo wz_hi xor`。

| 指令 | `(vx, vy, wz)` | 完整报文（十六进制） |
| --- | --- | --- |
| 向前 1 m/s | `(1, 0, 0)` | `AA 06 E8 03 00 00 00 00 47` |
| 向左 1 m/s | `(0, 1, 0)` | `AA 06 00 00 E8 03 00 00 47` |
| 向后 1 m/s | `(-1, 0, 0)` | `AA 06 18 FC 00 00 00 00 48` |
| 向右 1 m/s | `(0, -1, 0)` | `AA 06 00 00 18 FC 00 00 48` |
| 逆时针 1 rad/s | `(0, 0, 1)` | `AA 06 00 00 00 00 E8 03 47` |
| 停止 | `(0, 0, 0)` | `AA 06 00 00 00 00 00 00 AC` |

Python 打包示例（`vx/vy` 为 m/s，`wz` 为 rad/s）：

```python
import math
import struct
from functools import reduce
from operator import xor


def pack_velocity(vx, vy, wz):
    values = (vx, vy, wz)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("velocity must be finite")
    raw = [round(v * 1000) for v in values]
    if any(v < -32768 or v > 32767 for v in raw):
        raise ValueError("velocity exceeds int16 range")
    payload = b"\xAA\x06" + struct.pack("<hhh", *raw)
    return payload + bytes([reduce(xor, payload, 0)])

# 将结果写入 921600 baud、8N1 串口；建议每 10–20 ms 更新一帧。
frame = pack_velocity(1.0, 0.0, 0.0)  # 向前
```

这是坐标语义变更，帧长度、校验和缩放不变。若旧发送端确实使用 `vx_old` 向右、`vy_old` 向前，迁移为 `vx_new = vy_old`、`vy_new = -vx_old`、`wz_new = wz_old`。若上位机原本就是前/左坐标，则直接按 `vx, vy, wz` 打包，删除旧的交换轴或取反补丁。此工作区没有独立上位机程序，以上约定需同步到外部发送端。


## 8. 云台板独立通信

板间业务协议已统一整理为 v4，完整帧格式、七类消息、时效规则和 UART/RS485/CAN 后端见 [底盘云台板间通信](docs/底盘云台板间通信.md)。本地底盘固件已在 USART10（PE2 RX、PE3 TX、921600/8N1）接入 v4 编解码、流式解析、Heartbeat/ChassisControl 控制和 Heartbeat/ChassisFeedback 回传；云台端同步升级及真实双板验证仍未完成，旧 V1 yaw/status 接口只保留作迁移兼容。UART7 的 `AA 06 vx vy wz XOR` 是独立的上位机车身速度协议，不属于板间 v4 消息。
