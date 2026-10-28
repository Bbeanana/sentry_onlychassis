# H7 Framework

## 哨兵1.0

2026-10-28 配套代码快照，云台仓库为 https://github.com/JJL0612/sentry_gimbal 。
包含遥控板间控制、小陀螺、外置 DM-IMU-L1 航向读取、滚轮大 Yaw 转向与 IMU 航向锁定。
重点回归测试和编译通过；实车当前报告四个驱动轮不转，原因仍待排查，本版本不视为
已完成整车验证。大 Yaw 软件使用速度模式，电机最近读回为 MIT（CTRL_MODE=1），
硬件需切换为 CTRL_MODE=3 才与固件匹配。

## 大 Yaw 外置 IMU 航向锁定

`kRobotBigYawGyroCompensationEnabled=1U` 现在启用外置云台 DM-IMU-L1 航向闭环，
不再给电机发送“负底盘角速度”补偿，也不使用小陀螺的固定旋转目标作前馈。
底盘右拨杆下档仍由云台板发送 +0.8 rad/s 旋转目标；滚轮独立控制大 Yaw 转向。

大 Yaw 为 J4310、底盘 CAN3，基础 ID `0x06`、速度帧 ID `0x206`、反馈 ID `0x11`。
电机内部速度环执行应用层闭环算出的速度。**硬件需 `CTRL_MODE=3`**；最近实机
读取仍为 1（MIT），用户暂时无法连接，本次只生成固件，未烧录、未改电机模式。
已知 PMAX=12.5、VMAX=30、TMAX=10、Gr=10 保留原配置。

启用且收到有效电机反馈后，将外置 IMU 当前航向作为锁定航向：

```text
拨动滚轮：目标航向 += 滚轮目标角速度 × dt
滚轮回中：锁定松开时 IMU 实测航向
航向误差 = wrap(目标航向 − IMU 实测航向)
世界角速度目标 = 滚轮目标角速度 + 航向 P 修正
速度误差 = 世界角速度目标 − IMU 实测 Z 轴角速度
电机相对转速 = 安装方向 × 外部传动比 × 速度误差 PI 输出
```

航向误差按 ±π 最短分支处理，目标超前误差最大 ±π/2。读取的样本年龄用于
预测当前航向；不使用电机编码器角度作为航向目标。初始增益：航向 Kp=2，
修正速度最大 ±1.5 rad/s，角速度 PI Kp=0.5、Ki=3，带饱和防积分累积。
滚轮最大 ±1 rad/s；最终电机速度最大 ±10 rad/s。安装方向仍为 -1、外部传动比 1，
电机内部 Gr=10 不重复乘算。这里没有 MIT ±3 N·m 应用层力矩限制，电流/力矩
限制取决于电机配置。

外置 IMU 数据过期（50ms）、非有限数值、运动权限撤销、电机反馈失效或故障时，
发送零速度、失能并清理锁定目标/积分，不退回原底盘角速度补偿。数据/使能恢复
后重新锁住当时云台实测朝向，避免追赶旧目标。IMU 零位或云台启动会话改变时，
也重建锁定目标。行进坐标基准仍由独立状态管理，普通丢帧不会重置行进方向。

遥控映射、IMU 读取/传输和四舵轮行进代码保持不变。云台固件已发送闭环所需
航向和角速度，本次只更新底盘固件。六轴 IMU 本身仍可能有长期 yaw 漂移，
此闭环锁定的是它报告的航向；增益需实车低速验证。

诊断变量：`g_robot.big_yaw_heading_target_rad`、`big_yaw_heading_error_rad`、
`big_yaw_world_rate_target_radps`、`big_yaw_imu_rate_integral`、`big_yaw_target_rate`，
以及 `g_robot.yaw_motor.state.given_omega/omega`。

将功能开关设为 `0U` 仍是原有板间/右杆 MIT 位置速度串级，硬件也必须匹配 MIT。

## 外置 DM-IMU-L1 与小陀螺行进

云台板 CAN3 读取外置 DM-IMU-L1，实机只读查询确认设备 ID `0x01`、MST 反馈
ID `0x11`、1 Mbps；使用 V1.2 只读查询帧 `CC RID 00 DD 00 00 00 00`，
交替查询欧拉角 RID=3 和角速度 RID=2。欧拉角 yaw 为反馈字节 4..5 的无符号
16 位小端映射值，范围 ±180°，统一换算 rad。CAN 电机仍在各自原总线上。

MC02 CAN3 口的 VCC 使用对外可控 5V。云台启动调用
`BSP_Power.Init(false, false, true)` 开启共享外部 5V，不开启两路 24V 输出。
该 5V 还与舵机/部分串口共用，需板卡输入电源为外部 DCDC 提供能源；
`PC15=1` 仅代表使能，不能代替对接口实际电压的测量。

上电将云台朝向与车头摆正，两块板保持静止。云台 IMU 启动至少 1 秒后，
连续 20 条有效静止角度样本记录软件零位（不向 IMU 写零位/校准/保存指令）。
云台板通过 USART10 -> 底盘 USART1 发送启动相对 yaw、yaw 角速度、样本年龄
和序号。局部 v4 扩展消息 `0x0404`，32 字节 payload：

| 偏移 | 字段 |
|---|---|
| 0 | valid，1..3 保留为 0 |
| 4、8 | yaw_delta（rad）、yaw_rate（rad/s），小端 float |
| 12、16、20 | 样本年龄 ms、IMU 样本序号、软件零位代数，小端 uint32 |
| 24 | 目标底盘 boot ID，小端 uint64 |

两块板必须一起更新。20ms 发送批次为心跳 34 + 底盘控制 46 + IMU 46 +
大 Yaw 请求 50 = 176 字节；IMU 与滚轮命令各自独立，不依赖同一份有效标志。

本地遥控和云台板转发遥控都以外置 IMU 当前绝对航向定义摇杆前/左方向。
这里的“绝对”是共同启动零位下的固定坐标，不是地理北向。启动时在静止、
禁用状态对齐两块 IMU 的坐标基准。控制分别保留云台航向 `gimbal_imu_absolute_yaw`
和底盘航向 `chassis_imu_absolute_yaw`，不使用电机编码器或云台/底盘相对角字段。

先由云台绝对航向 `g` 计算世界速度：
`vx_world = vx*cos(g) - vy*sin(g)`，`vy_world = vx*sin(g) + vy*cos(g)`；
再用底盘绝对航向 `b` 转成轮组速度：
`vx_body = vx_world*cos(b) + vy_world*sin(b)`，
`vy_body = -vx_world*sin(b) + vy_world*cos(b)`。
旋转时只对最终底盘航向增加 `chassis_yaw_rate * 0.007s` 预测，保留纵向 30%、
横向 40% 降速和四舵轮解算；UART7 保持车身坐标。无论底盘怎么转，推前都沿
云台当前朝向；滚轮转过云台后，行进方向也随外置 IMU 新航向更新。

云台角度/角速度或 UART 角度数据超过 50ms、底盘 IMU 失效时，云台坐标平移
停止，原旋转指令仍由底盘独立处理。同一启动会话的有效角度恢复后，继续使用
原有固定坐标基准；普通停机和重新使能也绝不重新置零。`gimbal_imu_origin_initialized`
与瞬时 `gimbal_imu_heading_valid` 分开管理，防止把恢复时的两种朝向误当作对齐。
云台板重启或 IMU 零位代数改变后禁止自动重新配对（即使左拨杆下档），应把
云台和车头摆正、重启两板重新建立基准。云台单独 IMU 断电重启没有可识别的
设备 boot ID，同样需要停机并重启两板重新对齐。
两颗六轴 IMU 的 yaw 都可能漂移，本方案不提供长期绝对航向修正。

验证状态：`Gimbal_DM_IMU` 的 euler/gyro 计数和零位、`dm_imu_tx_ok/failed`；
底盘 `g_robot.chassis.gimbal_imu_heading_valid/gimbal_imu_absolute_yaw/chassis_imu_absolute_yaw`。

这是一个基于 STM32H723、HAL、FreeRTOS 和 CMSIS-DSP 的 CMake 工程框架。
工程已经包含 STM32CubeMX 生成的底层代码，以及 `Skywalker` BSP、算法、设备和应用层代码。

## 初次配置

### 1. 修改工程名

打开根目录下的 `CMakeLists.txt`，修改项目名：

```cmake
set(CMAKE_PROJECT_NAME h7_framework)
```

例如改为：

```cmake
set(CMAKE_PROJECT_NAME team_a_robot)
```

这个名称会用于生成目标文件、ELF 文件和 map 文件。修改后建议删除旧的构建目录，再重新配置：

```powershell
Remove-Item -Recurse -Force build\Debug
```

如果工程使用 Release 配置，也需要删除 `build\Release`。

### 2. 应用文件

当前应用入口文件是：

```text
Skywalker/Application/Src/app_robot.c
```

这个文件不需要改名。队员直接在其中编写自己的应用逻辑即可。

文件提供四组任务接口：

```c
ManagerTaskInit();  ManagerTaskLoop();
GimbalTaskInit();   GimbalTaskLoop();
ChassisTaskInit();  ChassisTaskLoop();
ShootTaskInit();    ShootTaskLoop();
```

初始化函数返回任务周期，单位为毫秒；循环函数由 FreeRTOS 任务周期性调用。
任务调度实现位于 `Skywalker/Application/Src/app_task.c`，通常不需要修改。

如果需要保留多个应用版本，可以在工程外备份，或者在 `Skywalker/Application/Src` 中只保留当前正在使用的一个应用实现文件。多个文件同时定义同名任务接口会导致链接时重复定义。

## 编译链配置

工程使用以下编译工具：

- CMake 3.22 或更高版本
- Ninja
- ARM GNU Toolchain，提供 `arm-none-eabi-gcc`、`arm-none-eabi-g++`、`arm-none-eabi-objcopy` 和 `arm-none-eabi-size`
- Windows PowerShell（以下命令以 PowerShell 为例）

当前 `CMakePresets.json` 默认使用 STM32CubeCLT 1.22.0 的安装路径：

```text
C:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin/
```

如果本机安装路径不同，请修改 `CMakePresets.json` 中的两个编译器路径：

```json
"CMAKE_C_COMPILER": "C:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe",
"CMAKE_CXX_COMPILER": "C:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin/arm-none-eabi-g++.exe"
```

同时确保 ARM GNU Toolchain 的 `bin` 目录已经加入 `PATH`，因为 `cmake/gcc-arm-none-eabi.cmake` 还会使用 `arm-none-eabi-objcopy`、`arm-none-eabi-size` 等工具。

可以在 PowerShell 中检查：

```powershell
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

## 配置和构建

在工程根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

生成文件位于：

```text
build/Debug/
build/Release/
```

其中 ELF 文件名跟随 `CMakeLists.txt` 中的工程名。工程也会生成 `compile_commands.json`，可供 VS Code、clangd 等代码分析工具使用。

## 工程依赖

### 已经随仓库提供

以下依赖已经位于工程目录中，通常不需要额外下载：

- STM32H7xx HAL Driver
- CMSIS Core、Device 和 RTOS2 头文件
- CMSIS-DSP 源码
- FreeRTOS 内核和 CMSIS-RTOS V2 适配层
- STM32H723 的启动文件、链接脚本和 CubeMX 生成的底层代码
- Skywalker BSP、算法、设备和应用层代码

### 需要在电脑上安装

- CMake 3.22+
- Ninja
- ARM GNU Embedded Toolchain，或包含该工具链的 STM32CubeCLT

### 可选工具

- STM32CubeMX：只有在需要重新生成 `Core`、`cmake/stm32cubemx` 或外设配置时才需要。重新生成后应检查生成代码是否覆盖了手工修改。
- VS Code：不是构建必需品，但仓库提供了 `.vscode/c_cpp_properties.json`，默认读取 `build/Debug/compile_commands.json`。

## 硬件和芯片假设

当前工程针对 STM32H723xG，使用：

- Cortex-M7，硬件浮点 ABI
- `STM32H723xx` 编译宏
- `STM32H723XG_FLASH.ld` 链接脚本

如果更换芯片或 Flash/RAM 配置，不能只修改工程名，还需要同步修改 CubeMX 配置、设备宏、启动文件和链接脚本。

## 常见问题

### 找不到编译器

确认 `CMakePresets.json` 中的路径存在，并且下面的命令可以执行：

```powershell
arm-none-eabi-gcc --version
```

### 修改工程名后仍然使用旧文件

删除对应的 `build\Debug` 或 `build\Release` 目录，再重新执行 `cmake --preset ...`。CMake 会缓存旧的项目配置。

### 链接时出现任务函数重复定义

检查 `Skywalker/Application/Src` 中是否同时存在多个定义了 `ManagerTaskInit`、`GimbalTaskInit`、`ChassisTaskInit` 或 `ShootTaskInit` 的应用文件。只保留当前使用的应用实现。

### Ninja 无法写入构建目录

确认没有其他 CMake/Ninja 构建进程正在使用该目录，并检查目录权限。必要时关闭 IDE 的后台构建任务后，删除构建目录并重新配置。
