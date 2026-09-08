/**
  * @file       system_identification_param.h
  * @brief      系统辨识工作点与激励信号参数
  *
  * @note 使用流程：
  *  1. 先保持 SYS_ID_EXCITATION_ENABLE=0，在 CH5、CH8 同时上档时确认机器人
  *     能够稳定保持下面设置的工作点。
  *  2. 设置激励通道、类型和幅值，再将 SYS_ID_EXCITATION_ENABLE 改为 1。
  *  3. 重新编译下载。每次重新进入系统辨识模式，扫频计时和 PRBS 序列都会复位。
  *  4. 任一开关离开上档即退出系统辨识模式，激励随之停止。
  *
  * @warning 所有角度均使用 rad，不是 degree；所有激励幅值均为广义力矩 N*m。
  *          首次测试应只开启一个通道并使用较小幅值，确认无持续饱和或失稳后再增大。
  */

#ifndef SYSTEM_IDENTIFICATION_PARAM_H
#define SYSTEM_IDENTIFICATION_PARAM_H

/*-------------------- USB output profile --------------------*/
#define SYS_ID_PROFILE_12_STATE_5_INPUT  (1U)
#define SYS_ID_PROFILE_18_STATE_7_INPUT  (2U)

// 选择系统辨识高速采样帧布局。修改后需要重新编译下载。
// 12状态profile发送0x10帧，输入为5维广义力矩；18状态profile发送0x11帧，
// 输入在前5维力矩后追加左、右腿轴向力。两种profile共用0x12参考配置帧。
// 当前实际控制器仍为12状态LQR；18状态profile用于协议和数据采集预留。
#define SYS_ID_OUTPUT_PROFILE             SYS_ID_PROFILE_12_STATE_5_INPUT

#if (SYS_ID_OUTPUT_PROFILE != SYS_ID_PROFILE_12_STATE_5_INPUT) && \
    (SYS_ID_OUTPUT_PROFILE != SYS_ID_PROFILE_18_STATE_7_INPUT)
#error "Invalid SYS_ID_OUTPUT_PROFILE"
#endif

#define SYS_ID_SAMPLE_SEND_PERIOD_MS     (4U)     // 高速采样帧发送周期，250Hz
#define SYS_ID_REFERENCE_SEND_PERIOD_MS  (1000U)  // 参考配置帧周期，1Hz
#define SYS_ID_JACOBIAN_DET_MIN          (1.0e-6f) // 小于该值时不反算腿部广义力

/*-------------------- Fixed operating point --------------------*/
// 固定腿长同时用于左右腿参考值，以及系统辨识模式下K、theta_eq和T0的插值工作点。
// 必须保持在机器人允许的 MIN_LEG_LENGTH ~ MAX_LEG_LENGTH 范围内。
#define SYS_ID_LEG_LENGTH               (0.18f)  // 左右腿虚拟杆长目标，单位m

// 下面依次对应当前尾巴离地LQR的12维目标状态：
// [body_x, body_x_dot, body_yaw, body_yaw_dot,
//  left_theta, left_theta_dot, right_theta, right_theta_dot,
//  body_pitch, body_pitch_dot, tail_beta, tail_beta_dot]。
// LQR始终使用“实际反馈值 - 此处目标值”计算误差，并没有固定实际反馈状态。
// 左右腿theta目标还会分别叠加模型在 SYS_ID_LEG_LENGTH 下计算出的theta_eq。
#define SYS_ID_REF_BODY_X               (-0.5f)  // 机体纵向位置目标，m；当前标定值为-0.5
#define SYS_ID_REF_BODY_X_DOT           (0.0f)   // 机体纵向速度目标，m/s；静态辨识应为0
#define SYS_ID_REF_BODY_YAW             (0.0f)   // 机体航向角目标，rad
#define SYS_ID_REF_BODY_YAW_DOT         (0.0f)   // 机体航向角速度目标，rad/s；静态辨识应为0
#define SYS_ID_REF_LEFT_LEG_THETA       (0.0f)   // 左腿摆角附加目标，rad，不包含theta_eq
#define SYS_ID_REF_LEFT_LEG_THETA_DOT   (0.0f)   // 左腿摆角速度目标，rad/s
#define SYS_ID_REF_RIGHT_LEG_THETA      (0.0f)   // 右腿摆角附加目标，rad，不包含theta_eq
#define SYS_ID_REF_RIGHT_LEG_THETA_DOT  (0.0f)   // 右腿摆角速度目标，rad/s
#define SYS_ID_REF_BODY_PITCH           (0.0f)   // 机体俯仰角目标，rad
#define SYS_ID_REF_BODY_PITCH_DOT       (0.0f)   // 机体俯仰角速度目标，rad/s
#define SYS_ID_REF_TAIL_BETA            (0.0f)   // 尾巴相对模型定义零位的角度目标，rad
#define SYS_ID_REF_TAIL_BETA_DOT        (0.0f)   // 尾巴角速度目标，rad/s

// roll不属于上述12维LQR状态，但会进入左右腿长协调控制，因此单独配置。
#define SYS_ID_REF_BODY_ROLL            (0.0f)   // 机体横滚角目标，rad
#define SYS_ID_REF_BODY_ROLL_DOT        (0.0f)   // 机体横滚角速度目标，rad/s

// 18状态profile追加的左右腿长参考；默认都引用12状态控制器的统一腿长。
#define SYS_ID_REF_LEFT_LEG_LENGTH       SYS_ID_LEG_LENGTH // 左腿虚拟杆长目标，m
#define SYS_ID_REF_LEFT_LEG_LENGTH_DOT   (0.0f)             // 左腿长速度目标，m/s
#define SYS_ID_REF_RIGHT_LEG_LENGTH      SYS_ID_LEG_LENGTH // 右腿虚拟杆长目标，m
#define SYS_ID_REF_RIGHT_LEG_LENGTH_DOT  (0.0f)             // 右腿长速度目标，m/s

/*-------------------- Excitation selection --------------------*/
#define SYS_ID_EXCITATION_DISABLED      0U  // 不生成激励
#define SYS_ID_EXCITATION_SINE_SWEEP    1U  // 线性正弦扫频
#define SYS_ID_EXCITATION_PRBS          2U  // 伪随机二进制序列

// 总使能开关：0=即使进入辨识模式也不叠加激励，1=按下面的MODE和MASK叠加激励。
// 调整工作点或排查控制问题时保持为0；开始正式试验前最后再改成1。
#define SYS_ID_EXCITATION_ENABLE        0U

// 激励类型只能选择上面三个常量之一。DISABLED与将ENABLE设为0的效果相同。
#define SYS_ID_EXCITATION_MODE          SYS_ID_EXCITATION_SINE_SWEEP

// 七位输入掩码，置1的通道施加激励，置0的通道保持纯控制器输出：
// bit0=右髋Tp，bit1=左髋Tp，bit2=右轮T，bit3=左轮T，bit4=尾巴Tt，
// bit5=左腿轴向力F，bit6=右腿轴向力F。
// 常用示例：0x01=仅右髋，0x02=仅左髋，0x04=仅右轮，0x08=仅左轮，
//           0x10=仅尾巴，0x20=仅左腿轴向力，0x40=仅右腿轴向力，
//           0x0C=双轮，0x03=双髋，0x60=双腿轴向力，0x1F=原五通道同时激励。
#define SYS_ID_EXCITATION_INPUT_MASK    0x1FU

/*-------------------- Sine sweep --------------------*/
// 当前实现是线性chirp：瞬时频率在DURATION内从START_FREQ线性变化到END_FREQ。
// 扫频从进入系统辨识模式的第一个控制周期开始；超过DURATION后激励自动变为0，
// 但机器人仍停留在系统辨识模式并由LQR保持工作点。
// 多个通道同时开启时，各通道扫频相位和频率相同，只是幅值不同，输入相关性较高；
// 因此正弦扫频建议一次只开启一个通道，多输入同时辨识优先使用独立PRBS。
#define SYS_ID_SINE_SWEEP_START_FREQ   (0.2f)  // 扫频起始频率，Hz，必须>=0
#define SYS_ID_SINE_SWEEP_END_FREQ     (20.0f) // 扫频终止频率，Hz，应低于250Hz奈奎斯特频率
#define SYS_ID_SINE_SWEEP_DURATION     (60.0f) // 从起始频率扫到终止频率所需时间，s，必须>0

/*-------------------- PRBS --------------------*/
// PRBS在 +amplitude 与 -amplitude 之间切换，进入辨识模式后持续生成，不自动停止。
// 每个码元保持时间 = BIT_HOLD_SAMPLES * CHASSIS_CONTROL_TIME_MS。
// 当前控制周期为2ms，因此10个采样点对应20ms，码元更新率为50Hz。
// HOLD越小，高频成分越多；但过小会增加执行器负担，也更容易激发未建模高频动态。
#define SYS_ID_PRBS_BIT_HOLD_SAMPLES   (10U)   // 每个PRBS码元保持的控制周期数，必须>=1

// 五路16位LFSR初值。初值必须非0；不同通道使用不同种子可降低多输入相关性。
// 保持种子不变可使每次重新进入辨识模式时重现完全相同的输入序列。
#define SYS_ID_PRBS_SEED_RIGHT_HIP     (0xACE1U) // 右髋PRBS种子
#define SYS_ID_PRBS_SEED_LEFT_HIP      (0x1D0FU) // 左髋PRBS种子
#define SYS_ID_PRBS_SEED_RIGHT_WHEEL   (0xBEEFU) // 右轮PRBS种子
#define SYS_ID_PRBS_SEED_LEFT_WHEEL    (0x5A5AU) // 左轮PRBS种子
#define SYS_ID_PRBS_SEED_TAIL          (0x1357U) // 尾巴PRBS种子
#define SYS_ID_PRBS_SEED_LEFT_LEG_FORCE  (0x2468U) // 左腿轴向力PRBS种子
#define SYS_ID_PRBS_SEED_RIGHT_LEG_FORCE (0x369CU) // 右腿轴向力PRBS种子

/*-------------------- Per-channel amplitude --------------------*/
// 每个数值都是峰值而非峰峰值，单位N*m；PRBS输出为+/-该值，正弦范围也为+/-该值。
// 激励先叠加到5维LQR广义力矩，再将髋力矩通过VMC映射为四个关节电机力矩。
// 这些值不是最终实体电机力矩，最终指令仍可能被VMC关系和电机限幅改变。
// 建议从当前值或更小值开始，逐通道观察姿态、速度、电机反馈力矩和饱和情况。
#define SYS_ID_EXCITATION_AMP_RIGHT_HIP   (0.2f) // 右髋虚拟力矩激励峰值，N*m
#define SYS_ID_EXCITATION_AMP_LEFT_HIP    (0.2f) // 左髋虚拟力矩激励峰值，N*m
#define SYS_ID_EXCITATION_AMP_RIGHT_WHEEL (0.1f) // 右轮力矩激励峰值，N*m
#define SYS_ID_EXCITATION_AMP_LEFT_WHEEL  (0.1f) // 左轮力矩激励峰值，N*m
#define SYS_ID_EXCITATION_AMP_TAIL        (0.1f) // 尾巴力矩激励峰值，N*m
#define SYS_ID_EXCITATION_AMP_LEFT_LEG_FORCE  (2.0f) // 左腿轴向力激励峰值，N
#define SYS_ID_EXCITATION_AMP_RIGHT_LEG_FORCE (2.0f) // 右腿轴向力激励峰值，N

#endif  // SYSTEM_IDENTIFICATION_PARAM_H
