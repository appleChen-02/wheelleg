# LQR MuJoCo 仿真项目

本项目从原始 sim-to-sim 工程中剥离得到，仅保留经典 LQR 仿真内容，不包含 RL 策略、权重文件、策略推理或 RL 诊断功能。

## 依赖环境

默认使用以下依赖路径：

- MuJoCo：`D:/mujoco-3.13.0-windows-x86_64`
- GLFW 源码：`D:/mujoco-3.13.0-windows-x86_64/glfw-3.5.1`

如路径不同，可在 CMake 配置时通过 `-DMUJOCO_ROOT=...` 或 `-DGLFW_SOURCE_DIR=...` 覆盖。

## 编译

请在项目根目录执行：

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

修改 `.cpp` 或 `.h` 文件后，通常只需要重新编译：

```powershell
cmake --build build --parallel
```

可执行文件为 `build/LQRSimulation.exe`。CMake 会自动将 `mujoco.dll` 复制到可执行文件旁边。

## 运行

请从项目根目录运行，以确保模型和网格文件的相对路径能够正确解析：

```powershell
.\build\LQRSimulation.exe --help
.\build\LQRSimulation.exe --duration 10
```

常用参数：

```text
--model path                         指定 MuJoCo XML 模型路径
--duration seconds                   指定仿真时长，单位为秒
--mode notail|bipedal|tripod|proxtripod  指定初始 LQR 模式
--render-fps hz                      指定画面刷新频率
--print-period seconds               指定状态打印周期
--console-output                     输出控制状态信息
--zero-gravity                       启用零重力模式
--pulse actuator torque_nm           对指定执行器施加力矩脉冲
--pulse-start seconds                指定力矩脉冲开始时间
--pulse-duration seconds             指定力矩脉冲持续时间
```

## 键盘操作

- `M` 或 `N`：循环切换 LQR 模式
- `V`：在 0 m/s 和 1 m/s 之间切换速度指令
- `Left`：施加左转偏航力矩，再次按下关闭
- `Right`：施加右转偏航力矩，再次按下关闭
- `T`：施加短时 Tp 力矩脉冲
- `P` 或 `Space`：暂停/继续仿真
- `R`：重置仿真
- `E`：立即启用控制器
- `K`：启用/关闭执行器力矩输出
- `G`：切换正常重力和零重力
- `H`：显示/隐藏帮助面板
- `Esc`：退出程序

默认模式为 `Tripod`。控制器仍保留原工程中的 `NoTail`、`Bipedal`、`Tripod` 和 `ProXTripod` LQR 实现，以及对应的 VMC、MPC 和 PID 辅助控制。
