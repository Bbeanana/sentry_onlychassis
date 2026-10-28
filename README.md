# H7 Framework

## 哨兵1.0

2026-10-28 配套代码快照，云台仓库为 https://github.com/JJL0612/sentry_gimbal 。
包含遥控板间控制、小陀螺、IMU 遥测与底盘角速度反向补偿的大 Yaw 控制。
重点回归测试和编译通过；实车当前报告四个驱动轮不转，原因仍待排查，本版本不视为
已完成整车验证。当前大 Yaw 软件使用速度模式（CTRL_MODE=3）。

## 当前大 Yaw 与行进方向逻辑

当前启用底盘 IMU 角速度反向补偿：`kRobotBigYawGyroCompensationEnabled = 1U`。
不读取云台 IMU，不根据云台 IMU 航向或角速度修正目标，也不以它作为使能条件。

```
大 Yaw 相对目标角速度 = 滚轮目标角速度 - 底盘 IMU 实测角速度
电机目标角速度 = 安装方向 × 外部传动比 × 大 Yaw 相对目标角速度
```

滚轮回中时仍持续补偿底盘的实际旋转，底盘停止时目标速度归零。
输入使用物理逆时针为正，安装方向 -1、外部传动比 1，电机速度限幅 ±10 rad/s。
这是实测底盘角速度前馈补偿，不积分航向；实际自稳精度受底盘陀螺零偏和电机速度跟踪影响。
底盘 IMU 无效、控制权限撤销或电机反馈丢失/故障时，大 Yaw 停止输出并失能。
云台 IMU 遥测仍可运行，但数据漂移、无效或掉线均不影响当前控制。

CAN3 基础 ID `0x06`，速度帧 ID `0x206`，反馈 ID `0x11`。
速度指令为 4 字节小端 float，特殊命令保持 8 字节。
电机硬件需使用速度模式（CTRL_MODE=3），与上一版 MIT 编码器串级不同。
此修改只生成固件，不烧录、不修改电机硬件参数；实车效果待验证。
补偿开关关闭时仍可选择编码器位置/速度串级（MIT 模式）。

行进坐标变换改用大 Yaw 绝对编码器：

```
连续编码器角 = 原始反馈角 + 25 rad × 跨界计数
theta = wrap(安装方向 × (连续编码器角 - 车头零位角) / 外部传动比)
vx_body = vx_gimbal*cos(theta) - vy_gimbal*sin(theta)
vy_body = vx_gimbal*sin(theta) + vy_gimbal*cos(theta)
```

本地遥控和云台板转发遥控的前/左方向随云台当前朝向变化；UART7 保持车身坐标。
小陀螺保留前后 30%、左右 40% 平移比例和四舵轮解算。θ 始终直接用编码器
反馈先按 25 rad 解回绕，再减固定零位计算，不叠加速度预测或 IMU 积分角。编码器缺失、反馈超过
100 ms、角度/速度非有限或电机报错时停止云台坐标平移，旋转指令仍独立处理。
反馈恢复后使用当前角度，不把恢复位置误当成车头零点。

**车头零位已按用户确认设为 `-1.132792 rad`（约 -64.90°）。** 参数位于
`Skywalker/Application/Src/app_robot.c` 的 `kRobotBigYawForwardZeroRad`。
该值来自电机重新校准后，用户将云台对准底盘正前方时的五组实时反馈。
重新对齐后五组原始角均为 -1.132791519 rad，反馈年龄 4～17 ms，电机错误码为 0。
本次标定跨界计数为 0、连续角等于原始角；新零位采用原始角，
因此启用新固件时保持正前方对齐并重新启动，让跨界计数从 0 开始。
普通停机和重新使能不改零位；重启后的历史圈数限制仍适用。

底盘不再依赖云台传来的 IMU 航向/零位代数来转换行进方向；云台板发送的
原有 `0x0404` 帧不参与控制，串口控制和滚轮帧不变。两套 IMU 航向差的累计误差不再影响
坐标转换，但大 Yaw 实际朝向漂移、轮组误差、打滑仍可能影响世界方向上的直线行进。

编码器反馈已确认在 +12.5 rad / -12.5 rad 之间按 25 rad 回绕。
CAN 接收端逐帧比较原始角差：差值小于 -12.5 时跨界计数加一，大于 +12.5 时减一。
先恢复连续角，再按物理 2π 周期计算 θ，避免每次量程回绕引入约 7.61° 方向误差。
跨界计数不在普通停机、重新使能或 IMU 复位时清零；板卡重启后无法从单个读数
恢复历史跨界计数，需要已知分支的上电姿态/回零。相邻有效反馈间实际运动必须
小于 12.5 rad；长时间丢帧期间跨界或电机内部位置重置也无法仅靠两帧角度可靠恢复。

诊断变量：`g_robot.chassis.big_yaw_heading_valid`、`big_yaw_encoder_angle_rad`、
`big_yaw_encoder_age_ms`、`big_yaw_relative_yaw_rad`、`big_yaw_relative_rate_radps`。
测试：`python3 tests/test_gimbal_translation.py` 和 `python3 tests/test_big_yaw_gyro.py`。

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
