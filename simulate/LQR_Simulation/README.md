# LQR MuJoCo 仿真项目

本项目从原始 sim-to-sim 工程中剥离得到，仅保留经典 LQR 仿真内容，不包含 RL 策略、权重文件、策略推理或 RL 诊断功能。

## 依赖环境

默认使用以下依赖路径：

- MSYS2 UCRT64 工具链：`D:/MSYS2/ucrt64/bin`（GCC、CMake、Ninja）
- MuJoCo：`D:/mujoco-3.13.0-windows-x86_64`
- GLFW 源码：`D:/mujoco-3.13.0-windows-x86_64/glfw-3.5.1`

如路径不同，可在 CMake 配置时通过 `-DMUJOCO_ROOT=...` 或 `-DGLFW_SOURCE_DIR=...` 覆盖。

## 编译

在仓库根目录执行以下命令，会自动配置、编译并打开 Bipedal 仿真窗口：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\simulate\LQR_Simulation\run.ps1
```

系统辨识同样自动编译，日志默认写入项目的 `logs/sysid_<时间戳>`：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\simulate\LQR_Simulation\run.ps1 -Mode sysid
```

启动脚本支持 `-Headless`、`-Duration 400`、`-Feedback mcu`、`-OutDir logs/实验目录` 和仅编译的 `-BuildOnly`。`-OutDir` 相对本项目目录解析；MSYS2 安装位置不同可传 `-Msys2Root 路径`。系统辨识完成提示出现后，按 Esc 退出并保存。

VS Code 打开整个仓库时，可以在“终端 → 运行任务”中选择 `LQR: Run Bipedal` 或 `LQR: Run System Identification`；`Ctrl+Shift+B` 执行构建，`LQR: Test` 执行集成测试。

手动构建时，在 `simulate/LQR_Simulation` 目录执行：

```powershell
$env:PATH = 'D:/MSYS2/ucrt64/bin;' + $env:PATH
cmake -S . -B build/msys2_ucrt64 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=D:/MSYS2/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=D:/MSYS2/ucrt64/bin/g++.exe -DCMAKE_MAKE_PROGRAM=D:/MSYS2/ucrt64/bin/ninja.exe -DLQR_BUILD_SYSID_TESTS=ON
cmake --build build/msys2_ucrt64 --parallel 4
```

修改 `.cpp` 或 `.h` 后，可重新运行启动脚本，或执行最后一条编译命令。统一使用 `build/msys2_ucrt64`，避开原 `build` 中的旧路径缓存。

可执行文件为 `build/msys2_ucrt64/LQRSimulation.exe`。CMake 自动为主程序和集成测试复制 MuJoCo 及 MinGW 运行库，无需修改系统 PATH。默认模型路径由 CMake 指向当前源码中的 `models/scene.xml`。

## 运行

在 `simulate/LQR_Simulation` 目录可直接运行已编译程序：

```powershell
.\build\msys2_ucrt64\LQRSimulation.exe --help
.\build\msys2_ucrt64\LQRSimulation.exe --duration 10
```

常用参数：

```text
--model path                         指定 MuJoCo XML 模型路径
--duration seconds                   指定仿真时长，单位为秒
--mode notail|bipedal|tripod|proxtripod|sysid  指定初始控制模式
--outdir directory                   sysid CSV 保存目录，默认 logs/sysid_<时间戳>
--render-fps hz                      指定画面刷新频率
--print-period seconds               指定状态打印周期
--console-output                     输出控制状态信息
--feedback truth|mcu                 真值反馈（默认）或固件观测器对照
--speed m/s                          初始前进速度指令
--yaw-rate rad/s                     初始航向角速度指令
--leg-length m                       bipedal 目标腿长（默认 0.18 m）
--tail-angle rad                     bipedal 尾角指令（默认 0 rad）
--headless                           无窗口运行；普通模式默认 5 秒，sysid 等待手动停止
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
.\build\msys2_ucrt64\LQRSimulation.exe --headless --duration 6 --speed 0.3 --yaw-rate 0.2 --console-output --print-period 1
.\build\msys2_ucrt64\LQRSimulation.exe --headless --duration 6 --feedback mcu --console-output --print-period 1
```

## 系统辨识数据采集

参数集中在 `src/system_identification_param.h`，修改后重新运行启动脚本或执行 `cmake --build build/msys2_ucrt64 --parallel`。
当前头文件配置为：固定双足工作点，位置参考 `-1.6 m`，右髋通道 `mask=0x01`、幅值 `5 N*m`、对数扫频 `0.2→20 Hz / 240 s`。实验配置以头文件为准。
该功能完成激励与数据采集，A/B 矩阵拟合仍在 MATLAB 中执行。

`SineSweep` 的瞬时频率为 `f(t)=f0*exp(ln(f1/f0)*t/T)`，正弦相位使用该频率的时间积分。
相同频率倍率的区间占用相同时间；上述参数在 `0.2～1 Hz` 段停留约 `83.9 s`、包含约 `41.7` 个周期，原线性扫频只有约 `9.7 s`、`5.8` 个周期。
起始频率必须大于零。对数扫频增加低频周期，但各倍频程的周期数仍不同；需要更多低频周期时可继续增加扫频时长。
实验记录 `simulation_experiment.txt` 新增 `sweep_law=logarithmic`，CSV 的模式编码仍为 `SineSweep=1`。

从本项目目录运行带窗口的手动停止实验：
$env:PATH = 'D:\MSYS2\ucrt64\bin;' + $env:PATH
cmake --build build --parallel
```powershell
cmake --build build/msys2_ucrt64 --parallel
.\build\msys2_ucrt64\LQRSimulation.exe --mode sysid --outdir logs/tail_sweep_001
```

流程与命令行提示：

1. `WAITING_FOR_STABILITY`：LQR 从启动起运行，固定所有参考，等待速度/角速度/腿长速度和连续窗口状态变化达到阈值。最少等待 5 s，连续稳定 3 s，等待超时 120 s。稳定允许姿态偏置，不要求实际状态为零。
2. `STABLE`：打开 CSV，保留 10 s 的无激励稳态基线；启动瞬态不写入 CSV。基线失去稳定时中止实验，不启动激励。
3. `EXCITATION_STARTED`：按头文件配置执行扫频或 PRBS。激励叠加在 LQR 广义力矩之后、VMC 和电机限幅之前。
4. `EXCITATION_FINISHED`：停止激励，继续记录至少 10 s 的结束响应，LQR 保持运行。
5. `IDENTIFICATION_CAPTURE_FINISHED`：采集所需阶段已结束，闭环和记录继续运行。此时按 **Esc** 或关闭仿真窗口，程序关闭文件并打印 `DATA_SAVED` 和保存路径。窗口中可暂停/继续和调整视角；运动、工作点、模式切换、重置、额外扰动及关闭输出操作被锁定。

命令行默认每 10 秒仿真时间输出一次进度，阶段切换时也立即输出，无需 `--console-output`：

```text
[SYSID] PROGRESS: collected=20.0/50.0 s (40.0%); phase=baseline_capture; remaining=30.0 s
```

已采集时长从稳定后开始写 CSV 时计算，总时长包含激励前基线、正式激励/无激励基线、结束响应，不包含等待稳定的时间。
等待稳定时显示等待时长和连续稳定时长；采集完成后显示 100% 和额外记录时长。
输出间隔可在头文件中修改 `SYS_ID_PROGRESS_PERIOD_SECONDS`，修改后重新编译。

无窗口手动实验也支持 `Ctrl+C` / `Ctrl+Break` 停止并保存，按实时运行：

```powershell
.\build\msys2_ucrt64\LQRSimulation.exe --mode sysid --headless --outdir logs/tail_sweep_002
```

可重复验证或批量采集时指定总仿真时长，无窗口会加速运行：

```powershell
.\build\msys2_ucrt64\LQRSimulation.exe --mode sysid --headless --duration 400 --outdir logs/log_sweep_003
.\build\msys2_ucrt64\LQRSimulation.exe --mode sysid --headless --duration 400 --feedback mcu --outdir logs/log_sweep_mcu_001
```

`--duration` 包含启动等待、基线、激励及结束响应。当前扫频的记录阶段至少需要 `10+240+10=260 s`，另外还要等待稳定；上例留出稳定等待时间，运行后仍应检查是否出现完成提示。
提前停止或超时/姿态越界会保存已有数据，并在实验记录中标记未完成/中止。已存在日志的目录会报错，避免覆盖旧实验。

常用头文件配置：

| 参数 | 操作 |
|---|---|
| `SYS_ID_EXCITATION_ENABLE` | `false` 采无激励基线；`true` 施加激励 |
| `SYS_ID_EXCITATION_MODE` | `SineSweep` 或 `Prbs` |
| `SYS_ID_EXCITATION_INPUT_MASK` | 单通道：右髋 `0x01`、左髋 `0x02`、右轮 `0x04`、左轮 `0x08`、尾巴 `0x10`；五通道 PRBS：`0x1F` |
| `SYS_ID_EXCITATION_AMPLITUDE` | 依次为右髋、左髋、右轮、左轮、尾巴的峰值力矩 |
| `SYS_ID_SINE_SWEEP_*` | 对数扫频起止频率（`0<f0<f1<125 Hz`）和时长；当前 240 s，分析时必须同步 |
| `SYS_ID_PRBS_BIT_HOLD_SAMPLES` / `SYS_ID_PRBS_DURATION` | 码元保持控制拍数 / 激励时长；默认 10 拍即 20 ms / 120 s |
| `SYS_ID_PRBS_SEEDS` | 与固件相同的 16 位 LFSR 算法；同配置可复现，验证实验可换种子 |
| `SYS_ID_LEG_LENGTH` / `SYS_ID_REFERENCE` | 腿长和 18 项参考；前 12 项为 LQR 状态，腿角参考还会叠加模型 `theta_eq`；参考速度必须为零 |
| `SYS_ID_BASELINE_DURATION` | 禁用激励时的基线时长，默认 30 s，另加前后各 10 s |

辨识模式以固定参考腿长计算 `K/theta_eq/T0`，不随实际腿长改变插值工作点，不启用动态腿长协调。
当前位置参考为 `-1.6 m`，与固件当前的 `-0.5 m` 不同；如需对应同一参考，请在头文件中修改。
`--speed`、`--yaw-rate`、`--leg-length`、`--tail-angle`、`--pulse` 和 `--zero-gravity` 不用于覆盖辨识工作点。

输出文件：

| 文件 | 内容 |
|---|---|
| `system_identification_12.csv` | 250 Hz、52 列，列名/顺序与 `上位机/usb_data_logger.py` 的 0x10 CSV 完全一致 |
| `system_identification_reference.csv` | 1 Hz、34 列，记录开始立即输出；参考与平衡输入，与 0x12 CSV 一致 |
| `simulation_experiment.txt` | 配置、种子、实际开始时间、计划结束时间、完成状态、停止原因与采集时序说明 |

时间轴使用仿真 `control_tick_ms`，每条高速记录增加 4 ms，`sequence` 每条增加 2。
为使加速运行仍能复用现有日志诊断，`host_time_s` 为启动主机时间加仿真时间的逻辑时间，不能用于测量计算耗时。
每行记录物理积分前的 `x[k]` 和新计算的 `u[k]`。当前模型是 `gear=1` 的无动态纯力矩电机，写入 ctrl 后用 `mj_fwdActuation` 同步取得本拍实际力矩，再反算髋部广义输入，避免混用上一拍反馈。
`command = controller + equilibrium + excitation` 保存限幅前广义输入；`actual` 保存模型实际执行输入。
电机反馈列转换到 MCU 原始反馈方向：J0/J1/J2/J3 对应仿真 actuator 的索引 1/0/3/2，饱和位采用相同的 MCU 电机顺序。

运行 MATLAB 激励分析前，修改 `matlab/系统辨识/analyze_sysid_excitation.m`：

```matlab
cfg.dataDir = 'C:\...\simulate\LQR_Simulation\logs\tail_sweep_001';
cfg.ampExpected = [5, 0, 0, 0, 0];
cfg.sweepF0 = 0.2;
cfg.sweepF1 = 20;
cfg.sweepT = 240;
cfg.sweepLaw = 'auto'; % 读取 simulation_experiment.txt 的 sweep_law；也可指定 logarithmic
cfg.legLength = 0.18;
```

无激励采集使用 `analyze_sysid_no_excitation.m`，同样设置 `cfg.dataDir` 和腿长。
`auto` 在元数据没有 `sweep_law` 时按 `linear` 分析，兼容旧仿真和当前固件数据；若只复制新实验的 CSV，需手动指定 `cfg.sweepLaw='logarithmic'`。
主分析直接输出 `state_metrics.csv`（相对激励前均值的波动/峰值/偏移）、`motor_margin.csv`（电机峰值/限幅比例/余量/饱和帧数）和 `input_consistency.csv`（五路输入分解及实际输入误差），并为每个激励通道保存 `fig6_frf_focus_<通道>.png`。
这些诊断共用主分析的时间截取和有效帧掩码，适用于各通道扫频及 PRBS；时域诊断包含有效饱和帧，频响仍只用连续无饱和分段。`cfg.focusStateIdx` 设置聚焦状态，`cfg.focusShowFigures` 控制图窗显示，`cfg.motorTorqueLimits` 设置电机限幅。缺少激励前基线或参考输入时，相应指标为 NaN。主分析不再调用 `review_right_hip_sweep`；历史复核脚本可单独手动运行。
CSV 兼容允许复用数据准备流程；完整开环 A/B 的估计还需要考虑其他闭环输入、MIMO 激励与独立验证，单通道频响不能直接当成完整开环模型。

## 辨识接口验证

集成检查覆盖对数扫频零交叉时刻和低频周期数、真实 MuJoCo 下的扫频/五通道 PRBS/无激励采集、固定增益工作点、电机限幅，以及稳定超时、提前停止、姿态越界：

```powershell
cmake -S . -B build/msys2_ucrt64 -DLQR_BUILD_SYSID_TESTS=ON
cmake --build build/msys2_ucrt64 --parallel
ctest --test-dir build/msys2_ucrt64 --output-on-failure
```

如 Python 已配置，可对完整实验校验 USB CSV 列顺序、250 Hz 采样、sequence、激励波形、输入分解和实际力矩（只用标准库）：

```powershell
python tests/validate_sysid_csv.py logs/log_sweep_003
```
