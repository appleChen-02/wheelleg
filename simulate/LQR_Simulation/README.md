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
--feedback truth|mcu                 真值反馈（默认）或固件观测器对照
--speed m/s                          初始前进速度指令
--yaw-rate rad/s                     初始航向角速度指令
--leg-length m                       bipedal 目标腿长（默认 0.18 m）
--tail-angle rad                     bipedal 尾角指令（默认 0 rad）
--headless                           无窗口运行；未指定正时长时运行 5 秒
--zero-gravity                       启用零重力模式
--pulse actuator torque_nm           对指定执行器施加力矩脉冲
--pulse-start seconds                指定力矩脉冲开始时间
--pulse-duration seconds             指定力矩脉冲持续时间
```

## 键盘操作

- `M` 或 `N`：循环切换 LQR 模式
- `W` / `S`：按住前进 / 后退，速度指令 ±0.5 m/s；松开置零，经过速度参考低通减速
- `A` / `D`：按住左转 / 右转，航向角速度指令 ±0.5 rad/s；松开保持最后的目标航向
- `Shift`：W/S/A/D使用较低的速度，0.2 m/s / 0.2 rad/s
- `Q` / `E`：bipedal 腿长减小 / 增大，按住以 0.02 m/s 调整，松开保持目标
- `Z` / `C`：bipedal 尾角减小 / 增大，按住以 0.4 rad/s 调整，松开保持目标
- `X`：停止前进和转向指令，保留腿长、尾角目标；W/S/A/D释放后才能重新运动
- `Home`：停止运动并将腿长 / 尾角目标恢复为 0.18 m / 0 rad
- `T`：施加短时 Tp 力矩脉冲
- `P` 或 `Space`：暂停/继续仿真
- `R`：重置仿真
- `K`：启用/关闭执行器力矩输出
- `G`：切换正常重力和零重力
- `H`：显示/隐藏帮助面板
- `Esc`：退出程序

默认模式为 `Bipedal`。腿长目标限制在 0.16～0.30 m，尾角指令限制在 -0.24～π/2 rad。W/S/A/D支持同时前进和转向；窗口失去焦点时停止运动指令。界面同时显示指令、平滑参考及实际反馈。

W/S/A/D直接改变速度 / 航向角速度参考，不叠加转向力矩。`T` 和 `--pulse` 用于独立的扰动测试；扰动之后仍执行关节 ±40、轮子 ±4.5、尾巴 ±7 N·m 的最终限幅，模型侧约束保持不变。

## 控制与反馈

物理步长固定为 1 ms，LQR、腿长 PID 和参考更新周期为 2 ms；两次控制更新之间保持力矩输出。PID 使用与固件相同的逐次离散积分、一阶误差差分和微分滤波，左右腿 Kd 分别为 2000 / 1200。前进速度参考采用 α=0.12 低通，尾角参考采用 α=0.15 低通。

默认 `--feedback truth` 直接读取 MuJoCo 的姿态、机体系角速度和沿水平航向的速度。纵向位置表示随转向累计的路程，运动时连续积分，不使用固件的目标速度门控、速度死区或 0.8 m/s 积分限制。模型没有传感器零偏，因此启动即启用控制，无需静止校准。

`--feedback mcu` 使用轮速 / 腿摆补偿 / 加速度卡尔曼融合、速度滞回死区和固件的位置积分门控，并在 1 kHz 更新 roll 逆系统观测器。传感量仍来自理想仿真，零偏按零处理；此选项用于观测器对照，不包含硬件启动校准或传感器噪声。

LQR 增益、平衡角与静态力矩系数采用现有固件 TAIL_VERSION=1 的表。离地控制默认不启用，与当前固件的实际标志更新行为一致。仍保留 `NoTail`、`Tripod` 和 `ProXTripod` 模式。

无窗口重复实验示例：

```powershell
.\build\LQRSimulation.exe --headless --duration 6 --speed 0.3 --yaw-rate 0.2 --console-output --print-period 1
.\build\LQRSimulation.exe --headless --duration 6 --feedback mcu --console-output --print-period 1
```
