# STM32CubeIDE for VS Code 工程

本目录由原 Keil / ARMCC 5 工程迁移而来。原始工程位于仓库的 `单片机` 目录；这里保留原应用、HAL、CMSIS、USB 中间件及 FreeRTOS V10.0.1。

## 打开和编译

1. 在 VS Code 中打开本目录的 `standard_robot.code-workspace`，或通过“文件 → 打开文件夹”直接打开 `cube单片机`。仓库根目录原有的 CMake 设置指向仿真工程，因此建议使用这里的独立工作区。
2. 安装 ST 官方扩展包 `STMicroelectronics.stm32-vscode-extension`。在 STM32Cube Bundle Manager 中安装 `gnu-tools-for-stm32`、`cmake`、`ninja` 及 ST-LINK 调试工具。本次适配使用新版扩展包 3.11.0 的 `stlinkgdbtarget` 调试器。
3. 按 `Ctrl+Shift+B` 执行 `STM32: Build Debug`。选择“终端 → 运行任务 → STM32: Build Release”可编译 Release。

也可在本目录的 PowerShell 终端执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Configuration Debug
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Configuration Release
```

脚本自动寻找已安装扩展提供的 `cube` 命令，不依赖本机固定工具版本路径；若没有 Cube CLI，则使用 PATH 中的独立 CMake、Ninja 和 Arm GNU Toolchain。

在配置好 STM32Cube 工具环境的终端，也可直接使用：

```text
cmake --preset Debug
cmake --build --preset Debug --parallel
cmake --preset Release
cmake --build --preset Release --parallel
```

产物位于 `build/Debug/` 或 `build/Release/`：`standard_robot.elf`、`.hex`、`.bin`、`.map` 和 `compile_commands.json`。

## 只烧录 BIN

连接 ST-LINK / SWD 并给目标板供电后，选择“终端 → 运行任务 → STM32: Flash BIN (Release)”。任务先执行 `STM32: Build Release`，再将 `build/Release/standard_robot.bin` 写入 `0x08000000`，校验后软件复位运行。

这一入口位于 `.vscode/tasks.json`，通过 `tools/flash.ps1` 调用 STM32CubeProgrammer CLI。它采用 SWD、1 MHz，自动定位 Cube CLI，不启动 GDB 调试会话。`launch.json` 的 `F5` 配置用于源码调试。

也可以只烧录已有的 BIN：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\flash.ps1 -Configuration Release
```

脚本默认使用 Release，支持 `-Configuration Debug`。需要指定探针时传入 `-SerialNumber`；预览烧录命令可加 `-WhatIf`，预览不会访问设备。

## ST-LINK / SWD 调试

连接 ST-LINK 的 SWDIO、SWCLK、GND 和目标电压参考，目标板正常供电。在“运行和调试”中选择 `STM32Cube: ST-LINK (Debug)`，按 `F5`；它先构建 Debug，再下载并停在 `main`。

默认使用 SWD、1 MHz，启用下载校验。需要调整连接方式或探针序列号时修改 `.vscode/launch.json`。调试配置使用当前 ST 官方调试扩展，无需额外安装 Cortex-Debug。

## 保留的硬件与内存配置

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F407IGHx / Cortex-M4F |
| 外部晶振 / 系统时钟 | 12 MHz / 168 MHz，沿用 `SystemClock_Config()` |
| Flash | `0x08000000` 起，1024 KiB，无 Bootloader 偏移 |
| 主 SRAM | `0x20000000` 起，128 KiB |
| CCM | `0x10000000` 起，64 KiB，仅显式 `.ccmram` 段使用 |
| C 库堆 / 主线程及中断栈 | 各 `0x2000` 字节，与 Keil 启动文件一致 |
| FreeRTOS 堆 / tick | 沿用 `FreeRTOSConfig.h` 的 24576 字节 / 1000 Hz |
| 浮点 ABI | `fpv4-sp-d16`、hard，与原 AHRS/DSP 库匹配 |
| printf | USART1，与原 `retarget_printf.c` 一致，支持浮点格式 |

默认的全局变量、堆及 DMA 缓冲区放在主 SRAM。CCM 不能供 DMA 访问；如显式使用 `.ccmram`，该段用于零初始化数据，GCC 启动文件负责清零。

## 迁移文件

- `CMakeLists.txt`、`CMakePresets.json`：Debug / Release 构建，链接及产物生成。
- `cmake/sources.cmake`：从原 `../单片机/MDK-ARM/standard_robot.uvprojx` 提取的显式源码和头文件目录清单；新增源码时在这里登记，避免将不属于原目标的示例或其他移植层一并编译。
- `cmake/gcc-arm-none-eabi.cmake`、`cmake/target.cmake`：交叉工具链和 ST 插件识别的 MCU 元数据。
- `STM32F407IGHX_FLASH.ld`：与原 Keil scatter 布局对应的 GNU 链接脚本。
- `startup/startup_stm32f407xx.s`：GCC 启动文件，包含 SRAM 初始化、CCM 清零和完整中断向量表。
- `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/`：与现有内核同版本的 GCC 移植层。
- `Src/gcc_syscalls.c`：Newlib 堆边界、串口输出和基础系统调用，替代仅用于 ARMCC 的 printf 重定向文件。
- `.vscode/`、`tools/build.ps1`、`tools/flash.ps1`：构建、代码索引、BIN 烧录和 ST-LINK 调试入口。

保留并链接 `components/algorithm/AHRS.lib`。原 `arm_cortexM4lf_math.lib` 含 GNU 链接器无法合并的 ARMCC 对象属性，已从本目录移除，因此改为编译 `components/algorithm/cmsis_dsp/Source/` 中的 Arm 官方 DSP V1.4.5 源码，与现有 `arm_math.h` 版本一致。仅纳入原固件使用的三角函数、矩阵运算及其常量表，清单位于 `cmake/dsp.cmake`。

编译选项使用 `-fshort-enums` 保持原 ARMCC 工程的枚举布局。AHRS 的公开接口只含浮点、字节及其指针，因此保留 Newlib 的默认 `wchar_t` ABI，并关闭该旧库的 wchar 尺寸标签告警。编译时由 GCC 自己提供编译器宏，删除了 Kalman 头文件中人为定义的 `__CC_ARM`。BMI088、OLED 和掉线检测结构体的 packed 语法改为 CMSIS 的 `__PACKED_STRUCT`。定长整数改用 `<stdint.h>`，公开函数使用标准 C99 外部链接，UART 通信宏保留合法的标识符拼接。

## CubeMX 配置和再次生成代码

`standard_robot.ioc` 保留原引脚、时钟、外设和 `STM32Cube FW_F4 V1.24.1`，目标工具链改为 CMake / GCC，并启用备份、保留用户代码和固定固件版本。

当前工程可以直接编译，不需要先用 CubeMX 重新生成。旧工程包含位于 `USER CODE` 区域以外的手工修改；之后如需通过新版 CubeMX 调整外设，先在临时目录生成，再比较合入对应改动，同时保留本工程的源码清单和启动/链接配置。CubeMX 自动生成的构建文件不能完整代表所有应用代码。

## 新增源码的上游来源

- [ST CMSIS Device F4 v2.6.3 的 GCC 启动文件](https://github.com/STMicroelectronics/cmsis-device-f4/blob/v2.6.3/Source/Templates/gcc/startup_stm32f407xx.s)，Git blob `aeeeb22f6da24ca9cf4a20dd3579e8d16eecb39f`，本地补充 CCM 清零。
- [FreeRTOS V10.0.1 GCC ARM_CM4F](https://github.com/FreeRTOS/FreeRTOS/tree/V10.0.1/FreeRTOS/Source/portable/GCC/ARM_CM4F)，`port.c` blob `ba923c1f1f1e5ce56f94bb9137495de78f146b51`，`portmacro.h` blob `335732f460ea68d62edbfeba1c9fc1bec8b1e774`，原样引入并保留许可证。
- [Arm CMSIS v4.3.0 / DSP_Lib](https://github.com/ARM-software/CMSIS/tree/v4.3.0/CMSIS/DSP_Lib/Source)：DSP V1.4.5，源码原样引入并保留许可证，每个文件的 Git blob 记录在 `components/algorithm/cmsis_dsp/upstream.json`。

## 本次验证结果

旧工程文件与迁移验证缓存清理后，首次编译会重新生成 `build/`、固件及代码索引。按 `Ctrl+Shift+B` 即可完成。

清理后已在系统临时目录分别完成 Debug、Release 的全量构建，两种 BIN 的 SHA-256 均与清理前一致。项目目录仅保留源码、配置和参考资料，旧文件存入可恢复临时归档。

使用 ST 官方 `gnu-tools-for-stm32 14.3.1+st.2`、CMake 4.4.0 和 Ninja 1.13.2 完成 Debug 干净构建及 Release 构建，均生成 ELF / HEX / BIN / MAP。

| 配置 | Flash 占用 | 主 SRAM 占用 |
| --- | ---: | ---: |
| Debug（`-Og -g3`） | 132716 B（12.66%） | 69760 B（53.22%） |
| Release（`-O2 -g3`） | 124156 B（11.84%） | 69696 B（53.17%） |

SRAM 数值已计入 C 库堆与主栈的保留空间。最终 ELF 使用 Cortex-M4 硬浮点 ABI；FreeRTOS 的 SVC、PendSV 与 SysTick 入口已核对。

从原 Keil AXF 调试信息提取类型尺寸，并用当前 GCC 编译选项独立计算，13 项全部一致，包括 BMI088、OLED、掉线检测、遥控器、IMU/电机/机器人状态及系统辨识 USB 帧。系统辨识三种帧大小依次为 186、242、122 字节。

构建仍会报告原应用中的告警。需单独检查 `application/chassis/chassis_balance.c` 的 `ConsoleStandUp()`：局部数组 `joint_pos_l`、`joint_pos_r` 的填充调用已被注释，后续仍读取数组。`bsp/boards/bsp_i2c.c` 也提示非法 I2C 选择可能留下未初始化的句柄。本次保留这些运行逻辑，未将其作为编译迁移的一部分修改。

本机枚举结果未检测到 ST-LINK，因此尚未进行实际烧录、上板调试或控制周期验证。连接 ST-LINK 和目标板后，可按上述 F5 流程验证。
