/**
 * @file system_identification_param.h
 * @brief MuJoCo 仿真的系统辨识工作点、激励和自动采集参数。
 *
 * 使用步骤：
 *   1. 修改本文件中的 SYS_ID_* 常量。
 *   2. 在 simulate/LQR_Simulation 目录执行：cmake --build build --parallel。
 *   3. 启动新进程：.\build\LQRSimulation.exe --mode sysid --outdir logs/实验目录。
 *   4. 等待 IDENTIFICATION_CAPTURE_FINISHED，再按 Esc 或关闭仿真窗口保存。
 *      无窗口运行时可用 Ctrl+C / Ctrl+Break 正常停止并保存。
 *
 * 本文件是 C++ 编译期配置，不是在运行时读取的配置文件：
 *   - 修改参数后必须重新编译，并重新启动程序；运行中的实验不会自动更新参数。
 *   - 仅修改数值时不需要重新执行 cmake -S ...，也不需要删除 build 目录。
 *   - 仅修改注释不影响行为；已编译程序可以继续使用原来的参数。
 *   - 分析时同步 MATLAB 的 cfg.ampExpected、sweepF0/sweepF1/sweepT、sweepLaw、
 *     prbsHoldS、legLength；这些 MATLAB 配置不会自动随本文件更新。
 *
 * 实验时序（各时长均为仿真时间，不是程序运行的墙钟时间）：
 *   等待连续稳定 → 记录激励前基线 → 激励/无激励正式采集 → 记录结束响应
 *   → 打印采集完成提示 → 继续保持 LQR 和记录，等待手动停止。
 *   总仿真时长 = 实际稳定等待时间 + PRE + 正式采集时长 + POST。
 *   --duration 限制的是上述总时间，过早退出会标记为未完成。
 *
 * 控制与日志约定：
 *   - 物理步长 1 ms，控制/激励更新周期 2 ms，高速 CSV 采样周期 4 ms。
 *   - 此模式保持 12 状态/5 输入 LQR 闭环，不会关闭控制器进行开环激励。
 *   - command = controller + equilibrium + excitation，随后进行 VMC 和电机限幅。
 *     command 是限幅前的广义输入，actual 是由实际电机输出反算的广义输入。
 *   - 此处“辨识结束”表示数据采集所需阶段结束；A/B 拟合在 MATLAB 中完成。
 */
#pragma once

#include <array>
#include <cstdint>

// 数值与固件的激励类型一致，也用于 CSV 的 excitation_config 编码。
// 仿真的 SineSweep 使用对数扫频；扫频规律另存于 simulation_experiment.txt。
enum class SystemIdentificationExcitation {
    Disabled = 0,   // 不施加激励，正式采集阶段采用 BASELINE_DURATION。
    SineSweep = 1,  // 对数正弦扫频：频率随时间指数增加，幅值保持不变。
    Prbs = 2       // 伪随机二进制序列：在 +幅值/-幅值之间按码元切换。
};

// -------------------- 1. 固定工作点与状态参考 --------------------

// 左右腿默认虚拟杆长，单位 m；当前允许范围为 0.16～0.30 m。
// 下方 reference[14]/reference[16] 默认都引用此值，修改后两腿目标一起改变。
// 实际用于 K、theta_eq、T0 插值的是这两个固定参考腿长，不是实时反馈腿长。
// 这里只固定目标和增益工作点，实际腿长仍通过 PID 动态跟踪，并非锁死关节。
inline constexpr double SYS_ID_LEG_LENGTH = 0.18;

// 状态顺序必须与 usb_data_logger.py 的 SYS_ID_STATE_COLUMNS 一致，不可换序。
// 前 12 项是 LQR 的物理状态参考；后 6 项补齐参考 CSV 的 18 项布局，
// 并提供 roll 检查参考以及左右腿长 PID 的固定目标，并不启用 18 状态 LQR。
// 所有角度为 rad，角速度为 rad/s；长度为 m，线速度为 m/s。
// 这是静态工作点实验，所有速度参考必须为 0，否则程序会拒绝启动。
//
// theta 是“额外修剪量”，最终 LQR/参考日志中的目标为：
//   左 theta_target = reference[4] + theta_eq_left；
//   右 theta_target = reference[6] + theta_eq_right。
// 因此不要把包含 theta_eq 的完整腿角再次填入此处，避免重复叠加。
// pitch、yaw、tail_beta 不额外叠加 theta_eq；姿态符号沿用 StateReader。
// body_x 是 StateReader 沿机身水平航向累计的纵向路程，并非世界坐标系 x。
// 默认位置参考为仿真起点 0 m；如需采用固件当前的 -0.5 m，修改第 0 项。
// 参考是目标，实际稳态可能有偏置；后续辨识应相对记录的实际工作点做偏差化。
inline constexpr std::array<double, 18> SYS_ID_REFERENCE = {
    -1.6,                // [ 0] body_x：纵向位置目标，m。
    0.0,                // [ 1] body_x_dot：纵向速度目标，m/s，保持为 0。
    0.0,                // [ 2] yaw：机身航向角目标，rad；控制误差会归一化到 ±pi。
    0.0,                // [ 3] yaw_dot：航向角速度目标，rad/s，保持为 0。
    0.0,                // [ 4] left_theta：左腿摆角附加修剪量，rad，不含 theta_eq。
    0.0,                // [ 5] left_theta_dot：左腿摆角速度目标，rad/s，保持为 0。
    0.0,                // [ 6] right_theta：右腿摆角附加修剪量，rad，不含 theta_eq。
    0.0,                // [ 7] right_theta_dot：右腿摆角速度目标，rad/s，保持为 0。
    0.0,                // [ 8] pitch：机身俯仰角目标，rad。
    0.0,                // [ 9] pitch_dot：俯仰角速度目标，rad/s，保持为 0。
    0.0,                // [10] tail_beta：尾巴相对模型零位的转角目标，rad。
    0.0,                // [11] tail_beta_dot：尾角速度目标，rad/s，保持为 0。
    0.0,                // [12] roll：横滚检查的参考角，rad；不进入 12 状态 LQR。
    0.0,                // [13] roll_dot：横滚角速度参考，rad/s，保持为 0。
    SYS_ID_LEG_LENGTH,  // [14] left_leg_length：左腿虚拟杆长目标，m。
    0.0,                // [15] left_leg_length_dot：左腿长速度参考，m/s，保持为 0。
    SYS_ID_LEG_LENGTH,  // [16] right_leg_length：右腿虚拟杆长目标，m。
    0.0                 // [17] right_leg_length_dot：右腿长速度参考，m/s，保持为 0。
};

// -------------------- 2. 激励使能、通道和幅值 --------------------

// 总使能：true 按 MODE/MASK/AMPLITUDE 生成激励；false 保持纯闭环无激励。
// false 时仍执行稳定等待、激励前基线、正式基线采集和结束响应记录。
// 首次采无激励基线只需改为 false，其他激励参数可保留。
inline constexpr bool SYS_ID_EXCITATION_ENABLE = true;

// 有效激励需要 ENABLE=true 且 MODE!=Disabled，两者任一关闭都不会施加激励。
// 默认从尾巴单通道 SineSweep 开始；全通道实验建议改为 Prbs。
inline constexpr auto SYS_ID_EXCITATION_MODE = SystemIdentificationExcitation::SineSweep;

// 5 位输入掩码，置 1 才施加该通道激励；未选通道仍由 LQR 正常输出力矩。
// 位序/单通道值：bit0 右髋 Tp=0x01；bit1 左髋 Tp=0x02；
//                bit2 右轮 T=0x04；bit3 左轮 T=0x08；bit4 尾巴 T=0x10。
// 组合示例：0x03 双髋，0x0C 双轮，0x1F 五通道。不要使用 bit5/bit6。
// 多通道扫频具有相同相位和频率，输入高度相关，不宜用来拟合完整 MIMO 模型；
// 单通道探索用扫频，全通道实验用不同种子的 PRBS，并检查输入相关性。
inline constexpr std::uint8_t SYS_ID_EXCITATION_INPUT_MASK = 0x01;

// 顺序为 [右髋 Tp, 左髋 Tp, 右轮 T, 左轮 T, 尾巴 T]，单位 N*m。
// 每个值是非负峰值 A，不是峰峰值；正弦范围 [-A,A]，PRBS 两电平为 ±A。
// 髋部值是虚拟髋广义力矩，不是某一个实体关节电机的直接力矩。
// MASK 未选中时对应幅值不生效；ENABLE=true 时至少选中一路非零幅值。
// 增大幅值前检查状态偏移和饱和日志；VMC 映射后的电机力矩可能先达到限幅。
// 同步 MATLAB cfg.ampExpected；默认尾巴实验可设为 [0,0,0,0,0.05]。
inline constexpr std::array<double, 5> SYS_ID_EXCITATION_AMPLITUDE = {7.0, 0.2, 0.1, 0.1, 0.05};

// -------------------- 3. 正弦扫频 --------------------

// 起始瞬时频率 f0，Hz，必须 >0（对数扫频不能从 0 Hz 开始）。
// 通常取 0.2 Hz 以覆盖低频动态。
inline constexpr double SYS_ID_SINE_SWEEP_START_FREQ = 0.2;

// 结束瞬时频率 f1，Hz；必须 >f0 且 <125 Hz（250 Hz 日志的奈奎斯特频率）。
// 低于奈奎斯特频率只是基本限制，分析还要检查信号功率和可辨识频带。
inline constexpr double SYS_ID_SINE_SWEEP_END_FREQ = 20.0;

// 从 f0 到 f1 的扫频时长 T，s，必须 >0；只在 SineSweep 模式决定激励时长。
// 激励起点 tau=0 是激励前基线结束的时刻，不是程序启动时刻。
//   r=f1/f0，L=ln(r)；f(tau)=f0*exp(L*tau/T)；
//   e(tau)=A*sin(2*pi*f0*T/L*(exp(L*tau/T)-1))。
// 相同频率倍率的区间占用相同时间，因此低频比线性扫频获得更多周期。
// 0.2→20 Hz、240 s 时，0.2～1 Hz 段约有 41.7 周期（线性约 5.8）。
// 到 T 后激励归零，进入结束响应阶段，控制器继续运行。
// 当前时长 240 s；分析时同步 cfg.sweepT，并使用 logarithmic 扫频规律。
inline constexpr double SYS_ID_SINE_SWEEP_DURATION = 240.0;

// -------------------- 4. PRBS --------------------

// 一个 PRBS 码元保持的“控制拍数”，不是 CSV 行数；必须 >=1。
// 码元时间 = 此值 * 0.002 s；默认 10 拍=20 ms，20 拍=40 ms。
// 连续相邻码元可能具有相同符号，因此可见波形的电平保持时长可能超过一码元。
// 同步 MATLAB cfg.prbsHoldS；码元越短，高频成分通常越多。
inline constexpr unsigned SYS_ID_PRBS_BIT_HOLD_SAMPLES = 10;

// 五路非零 16 位 LFSR 初值，顺序同 AMPLITUDE，算法与固件相同：
// x^16+x^14+x^13+x^11+1，移位反馈使用 0xB400。
// 每次启动新实验从这些种子开始；相同参数和种子可复现激励序列。
// 不同种子有助于减少输入相关性，但仍须用采集数据检查 MIMO 输入矩阵条件。
// 另采独立验证数据时可换一组非零种子；扫频模式不使用这些种子。
inline constexpr std::array<std::uint16_t, 5> SYS_ID_PRBS_SEEDS = {0xACE1, 0x1D0F, 0xBEEF, 0x5A5A, 0x1357};

// PRBS 正式激励时长，s，必须 >0；不包括稳定等待和前后基线。
// 与固件持续到退出模式的行为不同，仿真规定终点，以便输出采集完成提示。
// 到期自动停止激励，采集结束响应后等待手动退出保存。
inline constexpr double SYS_ID_PRBS_DURATION = 120.0;

// -------------------- 5. 采集时序与稳定等待 --------------------

// 无激励“正式基线阶段”的时长，s，必须 >0。
// ENABLE=false 或 MODE=Disabled 时生效，扫频/PRBS 实验不使用此时长。
// 默认稳定后的总记录阶段为 10+30+10=50 s，再进入等待手动停止阶段。
inline constexpr double SYS_ID_BASELINE_DURATION = 30.0;

// 连续稳定确认后、正式激励前的无激励记录时长，s，必须 >=0。
// MATLAB 需要此段估计实际工作点，正式辨识建议保留 >=5 s，默认 10 s。
// 该阶段如果失去稳定，实验会中止；不会直接带着不稳定基线启动激励。
inline constexpr double SYS_ID_PRE_EXCITATION_SECONDS = 10.0;

// 正式采集到期后，关闭激励并记录自由恢复响应的最短时长，s，必须 >=0。
// 到期输出 IDENTIFICATION_CAPTURE_FINISHED，此后仍记录，直到手动停止。
inline constexpr double SYS_ID_POST_EXCITATION_SECONDS = 10.0;

// 命令行进度输出间隔，单位为仿真秒，必须 >0；无需打开 --console-output。
// “已采集/总时长”从稳定后打开 CSV 时开始计时，总时长为 PRE+正式采集+POST，
// 不含启动等待。无激励时正式采集采用 BASELINE_DURATION，扫频/PRBS 用各自时长。
// 阶段切换时额外立即打印一次；完成后显示 100% 和额外记录时长，等待手动保存。
inline constexpr double SYS_ID_PROGRESS_PERIOD_SECONDS = 10.0;
static_assert(SYS_ID_PROGRESS_PERIOD_SECONDS > 0.0, "SYS_ID progress period must be positive");

// 所有稳定条件连续满足的时间，s，必须 >0；任一条件不满足就重新计时。
// 稳定条件包括速度、腿长误差、相对候选起点的状态变化、有限值、
// 可用的腿部雅可比以及上一拍没有电机饱和，不仅检查机身 pitch。
inline constexpr double SYS_ID_STABLE_REQUIRED_SECONDS = 3.0;

// 从仿真 t=0 起，至少等待这么久后才开始稳定确认计时，s，必须 >=0。
// 这段时间 LQR 已在运行，只是还不开始记录/施加辨识激励。
inline constexpr double SYS_ID_MIN_SETTLE_SECONDS = 5.0;

// 从仿真 t=0 起计算的稳定等待超时，s，必须 >0。
// 只约束 Waiting 阶段，不是整个实验的时长上限；应大于 MIN_SETTLE+STABLE_REQUIRED。
// 到期仍未确认稳定则打印 ABORTED，保持 LQR，不启动激励，等待手动停止。
inline constexpr double SYS_ID_SETTLE_TIMEOUT_SECONDS = 120.0;

// -------------------- 6. 稳定条件阈值 --------------------
// 以下为编译期阈值，Waiting 和激励前基线阶段使用。
// 它们用于判断状态是否已收敛，允许非零的实际稳态姿态和位置偏置。
// 当前实现还要求 |pitch-reference_pitch|<=0.15 rad，此界限在 IsStable 中固定。

// |body_x_dot| 的上限，m/s。
inline constexpr double SYS_ID_STABLE_SPEED = 0.01;

// |yaw_dot/theta_L_dot/theta_R_dot/pitch_dot/tail_beta_dot/roll_dot| 的共同上限，rad/s。
inline constexpr double SYS_ID_STABLE_ANGULAR_RATE = 0.03;

// 左右虚拟杆长速度绝对值的上限，m/s。
inline constexpr double SYS_ID_STABLE_LEG_SPEED = 0.005;

// 左右实际腿长相对各自固定参考腿长的误差上限，m；不是腿角误差。
inline constexpr double SYS_ID_STABLE_LEG_ERROR = 0.005;

// body_x 相对本次连续稳定候选起点位置的最大变化，m；不是位置参考误差，
// 也不是每秒漂移率。确认稳定后的激励前基线仍沿用该候选起点进行检查。
inline constexpr double SYS_ID_STABLE_POSITION_CHANGE = 0.01;

// yaw、左右 theta、pitch、tail_beta、roll 相对稳定候选起点的最大角度变化，rad。
// 角差按 ±pi 归一化；此值不是标准差，也不是相邻两个采样点之间的差值阈值。
inline constexpr double SYS_ID_STABLE_ANGLE_CHANGE = 0.005;

// -------------------- 7. 激励期间的中止条件 --------------------
// 正式采集阶段检查状态偏移和有限值；越界则当拍停止激励并打印 ABORTED。
// 中止后继续保持 LQR 和记录，直到手动停止；不会报告完整实验已完成。
// 电机饱和由限幅和 motor_saturation_mask 单独处理，当前不会单凭饱和立即中止。

// pitch 相对“激励/正式基线开始那一拍的实际 pitch”的最大偏移，rad。
// 比较基准不是 reference_pitch，也不是激励前整个基线段的均值。
inline constexpr double SYS_ID_ABORT_PITCH_CHANGE = 0.1;

// 左右 theta 各自相对正式采集开始那一拍实际值的最大偏移，rad。
inline constexpr double SYS_ID_ABORT_THETA_CHANGE = 0.3;

// |roll-reference_roll| 的上限，rad；不同于上面两个相对实际工作点的阈值。
// 此限制在稳定检查中也使用，不只在正式激励阶段生效。
inline constexpr double SYS_ID_ABORT_ROLL = 0.15;

// -------------------- 8. 实际输入反算与日志协议 --------------------

// 左右腿雅可比行列式绝对值的最小阈值，用于逆 VMC 的数值有效性检查。
// 行列式非有限或小于此值时，不反算髋部实际输入；输出 NaN 和 JACOBIAN_BAD，
// 同时取消 ACTUAL_INPUT_VALID 标志。它不是一个力矩幅值或姿态角阈值。
inline constexpr double SYS_ID_JACOBIAN_DET_MIN = 1e-6;

// 高速 CSV 记录周期，ms：固定 4 ms=250 Hz；每条记录 sequence 增加 2。
// 名称沿用固件 SEND_PERIOD，但仿真直接写 CSV，不通过 USB 发送。
inline constexpr unsigned SYS_ID_SAMPLE_SEND_PERIOD_MS = 4;

// 参考 CSV 记录周期，ms：固定 1000 ms=1 Hz，记录开始时立即输出第一行。
// 参考帧包含 18 项参考和 7 项平衡输入，以匹配 USB 0x12 CSV 的完整布局。
inline constexpr unsigned SYS_ID_REFERENCE_SEND_PERIOD_MS = 1000;

// 现有 MATLAB 脚本和验证脚本按 250 Hz/1 Hz 约定工作，所以禁止只改这两个值。
// 如确需改变采样率，须一并修改采集时序、序列约定、分析及验证，再调整此断言。
static_assert(SYS_ID_SAMPLE_SEND_PERIOD_MS == 4 && SYS_ID_REFERENCE_SEND_PERIOD_MS == 1000,
              "Keep the USB/MATLAB 250 Hz sample and 1 Hz reference format");

// -------------------- 9. 一次实验的配置对象 --------------------
// SYS_ID_* 是用户修改的默认值；本结构体把工作点、激励和时序装入一个对象，
// 供 BalanceController 和 SystemIdentificationExperiment 接收并保存。
// 这样无需在每个函数间分别传递大量参数，控制器/激励发生器/日志可使用同一配置。
//
// 创建 SystemIdentificationParameters p; 时，每个成员从上方常量取得默认值。
// 普通仿真 main.cpp 使用这些默认值，因此用户只需修改上方常量并重新编译，
// 不需要在此处再填一遍参数。结构体是参数的传递方式，不是第二套配置文件。
//
// 测试程序可写 p.excitation_mode=Prbs; p.prbs_duration=2.0; 来构造短实验，
// 并把同一个 p 传给控制器和实验对象，无需修改正式实验默认值。
// 这种覆盖发生在程序代码中；当前主程序没有从命令行/配置文件覆盖这些成员的接口。
// 稳定速度等阈值、中止阈值、雅可比阈值、日志周期和进度周期未打包，仍直接使用上方常量。
struct SystemIdentificationParameters {
    std::array<double, 18> reference = SYS_ID_REFERENCE;  // 固定物理状态参考，theta 为附加量。
    bool excitation_enable = SYS_ID_EXCITATION_ENABLE;   // 总使能。
    SystemIdentificationExcitation excitation_mode = SYS_ID_EXCITATION_MODE;  // 激励类型。
    std::uint8_t input_mask = SYS_ID_EXCITATION_INPUT_MASK;  // 五路输入选择位。
    std::array<double, 5> amplitude = SYS_ID_EXCITATION_AMPLITUDE;  // 五路峰值，N*m。
    double sweep_f0 = SYS_ID_SINE_SWEEP_START_FREQ;       // 扫频起始频率，Hz。
    double sweep_f1 = SYS_ID_SINE_SWEEP_END_FREQ;         // 扫频结束频率，Hz。
    double sweep_duration = SYS_ID_SINE_SWEEP_DURATION;  // 扫频时长，s。
    unsigned prbs_hold_samples = SYS_ID_PRBS_BIT_HOLD_SAMPLES;  // PRBS 码元保持控制拍数。
    std::array<std::uint16_t, 5> prbs_seeds = SYS_ID_PRBS_SEEDS;  // 五路初始种子。
    double prbs_duration = SYS_ID_PRBS_DURATION;         // PRBS 时长，s。
    double baseline_duration = SYS_ID_BASELINE_DURATION;  // 无激励正式采集时长，s。
    double pre_seconds = SYS_ID_PRE_EXCITATION_SECONDS;  // 激励前基线时长，s。
    double post_seconds = SYS_ID_POST_EXCITATION_SECONDS;  // 最短结束响应记录时长，s。
    double stable_seconds = SYS_ID_STABLE_REQUIRED_SECONDS;  // 连续稳定确认时长，s。
    double min_settle_seconds = SYS_ID_MIN_SETTLE_SECONDS;  // 最早开始稳定确认的仿真时刻，s。
    double settle_timeout_seconds = SYS_ID_SETTLE_TIMEOUT_SECONDS;  // 稳定等待截止时刻，s。
};
