# 轮腿机器人 STM32 工程

本目录用于 STM32CubeIDE for VS Code，目标芯片为 STM32F407IGHx，使用 CMake、ARM GCC 和 FreeRTOS V10.0.1。

1. 打开 `standard_robot.code-workspace`。
2. 按 `Ctrl+Shift+B` 构建 Debug；Release 可通过“终端 → 运行任务 → STM32: Build Release”构建。
3. 连接 ST-LINK / SWD 和目标板后，选择“终端 → 运行任务 → STM32: Flash BIN (Release)”完成编译、BIN 烧录、校验和复位运行。
4. 需要断点和源码调试时，按 `F5` 启动原有调试配置。

完整的工具安装、编译命令、硬件配置及已知问题见 [STM32Cube 使用与迁移说明](README_STM32CUBE.md)。首次构建会自动生成 `build/` 和代码索引。

## 目录

- `application/`：机器人控制、通信、IMU、参数，以及底盘增益计算所需的 MATLAB 脚本。
- `bsp/`、`components/`：板级驱动、设备、控制器和算法；`AHRS.lib` 仍是有效依赖。
- `Src/`、`Inc/`、`Drivers/`、`Middlewares/`：初始化代码、HAL/CMSIS、FreeRTOS 与 USB 中间件。
- `cmake/`、`startup/`、`STM32F407IGHX_FLASH.ld`：构建清单、GCC 启动文件和链接布局。
- `.vscode/`、`tools/`：构建、索引与 ST-LINK 调试配置。
- `doc/`：原框架的 API、设计和使用参考，入口为 [框架说明](doc/framework.md)。

## 清理范围

旧 `MDK-ARM/` 工程及产物、Keil 清理批处理、旧 CubeMX 生成缓存、RVDS 移植层、未使用的 ARMCC DSP 库，以及旧构建缓存和迁移临时文件，均已移出本项目并存入系统临时目录的 `wheelleg-cube-cleanup-*/archive/` 可恢复归档。实际编译使用 GCC 源码及移植层，需要原 Keil 工程时使用仓库的 [单片机目录](../单片机/README.md)。

应用源码、MATLAB 参数计算脚本、框架参考文档、许可证和实际依赖的库均予保留。

本项目源自 StandardRobot++，原框架介绍和作者信息见 [原项目说明](../单片机/README.md)，许可见 [LICENSE](LICENSE)。
