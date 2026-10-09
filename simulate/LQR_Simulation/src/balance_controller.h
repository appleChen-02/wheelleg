// 平衡控制器的接口定义与配置结构，供主循环调用。
#pragma once

#include <array>
#include <string>

#include "constants.h"
#include "state_reader.h"
#include "system_identification_param.h"

// 控制模式枚举：无尾、双足、三足（或三组）等。
enum class ControlMode {
    NoTail = 0,
    Bipedal = 1,
    Tripod = 2,
    ProXTripod = 3,
    SystemIdentification = 4,
};

// 控制输出命令，包含所有致动器的目标力矩。
struct ControlCommand {
    std::array<double, 5> controller_output{};  // [Tp_R,Tp_L,T_R,T_L,T_tail]，限幅前
    std::array<double, 5> excitation{};
    std::array<double, 5> generalized_command{};
    std::array<double, 7> equilibrium_input{};
    std::array<double, 18> reference_state{};  // theta 已包含 theta_eq
    std::uint8_t motor_saturation_mask = 0;
    std::array<double, 7> actuator_torque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> vmc_torque = {0.0, 0.0, 0.0, 0.0};  // [L_phi1, L_phi4, R_phi1, R_phi4]
    std::array<double, 2> vmc_F = {0.0, 0.0};    // [L, R] radial force
    std::array<double, 2> vmc_Tp = {0.0, 0.0};   // [L, R] tangential force
    std::array<double, 2> leg_length_error = {0.0, 0.0};  // [L, R] target - actual
    std::array<double, 2> leg_length_F_pid = {0.0, 0.0};  // [L, R] PID output
    std::array<double, 2> leg_length_F_ff = {0.0, 0.0};   // [L, R] feedforward
    double x_ref = 0.0;        // current position reference
    double speed_ref = 0.0;    // current speed reference
    double yaw_ref = 0.0;
    double yaw_rate_ref = 0.0;
    double yaw_pid_out = 0.0;  // yaw速度PID输出: + → 左减右加
    std::array<double, 2> ref_l0 = {0.0, 0.0};  // [L, R] roll补偿后目标腿长
    double coord_out = 0.0;   // 两腿theta协调PID输出
    double ref_theta = 0.0;   // Tripod模式：几何求解器给出的目标theta
    double ref_pitch = 0.0;   // Tripod模式：几何求解器给出的目标pitch
    double ref_beta = 0.0;    // Tripod模式：尾巴目标角度
    // ProXTripod VMC 尾巴中间量
    double tail_z_comp = 0.0;   // tail_up PID 输出
    double fdb_tail_z = 0.0;    // 反馈尾轮高度
    double ref_tail_z = 0.0;    // 目标尾轮高度
    double tail_F_ff = 0.0;     // 尾巴前馈力
    double tail_F_comp = 0.0;   // 尾巴补偿力
    double tail_cos_term = 0.0; // cos(β-φ+γ)
};

// MCU pid.c 的位置式 PID 状态。Ki/Kd 是“每次调用”的离散参数，不再乘除 dt。
struct PidState {
    double kp = 500.0;
    double ki = 50.0;
    double kd = 10.0;
    double max_output = 30.0;
    double max_iout = 20.0;

    double n = 0.0;               // 微分一阶低通系数
    // MCU 的全局 CHASSIS 结构零初始化；未显式设积分分离范围的 PID 为 [0,0]。
    double error_sum_low = 0.0;
    double error_sum_up = 0.0;
    double error_deadband_low = 0.0;
    double error_deadband_up = 0.0;
    bool reset_iout_outside_range = true;  // PID_calc=true，PID_calc_Leg=false

    double iout = 0.0;
    double dbuf[3] = {0.0, 0.0, 0.0};
    double error_hist[3] = {0.0, 0.0, 0.0};  // 误差历史: [0]当前 [1]上一次 [2]上上次

    double Compute(double error, bool clear_integral = false);
    void Reset();
};

// 控制器可配置参数集合（质量、重力、最大力矩和尾巴控制增益等）。
struct BalanceControllerConfig {
    SystemIdentificationParameters sys_id{};
    double body_mass = kDefaultBodyMass;
    double gravity = kDefaultGravity;
    double max_joint_torque = kDefaultMaxJointTorque;
    double max_wheel_torque = kDefaultMaxWheelTorque;
    double max_tail_torque = kDefaultMaxTailTorque;
    double tail_kp = kDefaultTailKp;
    double tail_kd = kDefaultTailKd;
    double yaw_velocity_kp = kDefaultYawVelocityKp;
    double yaw_velocity_ki = kDefaultYawVelocityKi;
    double yaw_velocity_kd = kDefaultYawVelocityKd;
    double yaw_velocity_max_output = kDefaultYawVelocityMaxOutput;
    double yaw_velocity_max_iout = kDefaultYawVelocityMaxIout;

    // 腿长 PID 参数（直接对齐 MCU 的每拍离散参数，左右腿独立）
    double ref_leg_length_tripod = 0.185;
    double ref_leg_length_bipedal = 0.18;
    double ref_leg_length_default = 0.17;
    double ff_ratio = 1.0;
    double tail_F_ff_ratio = kDefaultTripodTailFfRatio;
    std::array<double, 2> leg_length_kp = {600.0, 600.0};
    std::array<double, 2> leg_length_ki = {0.7, 0.7};
    std::array<double, 2> leg_length_kd = {2000.0, 1200.0};
    std::array<double, 2> leg_length_n = {0.1, 0.1};
    std::array<double, 2> leg_length_error_sum_limit = {0.05, 0.05};
    double leg_length_max_output = 80.0;
    double leg_length_max_iout = 40.0;

    // 两腿 theta 协调 PID 参数（每次调用的离散参数）
    double leg_coord_kp = kDefaultLegCoordKp;
    double leg_coord_ki = kDefaultLegCoordKi;
    double leg_coord_kd = kDefaultLegCoordKd;
    double leg_coord_max_output = kDefaultLegCoordMaxOutput;
    double leg_coord_max_iout = kDefaultLegCoordMaxIout;

    // tail_comp PID 参数（Tripod 模式，pitch→tail torque 补偿，对齐 MCU）
    double tail_comp_kp = kDefaultTailCompKp;
    double tail_comp_ki = kDefaultTailCompKi;
    double tail_comp_kd = kDefaultTailCompKd;
    double tail_comp_max_output = kDefaultTailCompMaxOutput;
    double tail_comp_max_iout = kDefaultTailCompMaxIout;

    // ProX_Tripod: tail_up PID 参数（对齐 MCU TAIL_UP）
    double proX_tail_up_kp = kDefaultProXTailUpKp;
    double proX_tail_up_ki = kDefaultProXTailUpKi;
    double proX_tail_up_kd = kDefaultProXTailUpKd;
    double proX_tail_up_max_output = kDefaultProXTailUpMaxOutput;
    double proX_tail_up_max_iout = kDefaultProXTailUpMaxIout;

    // ProX_Tripod: tail_z PID 参数（对齐 MCU TAIL_Z）
    double proX_tail_z_kp = kDefaultProXTailZKp;
    double proX_tail_z_ki = kDefaultProXTailZKi;
    double proX_tail_z_kd = kDefaultProXTailZKd;
    double proX_tail_z_max_output = kDefaultProXTailZMaxOutput;
    double proX_tail_z_max_iout = kDefaultProXTailZMaxIout;

    // ProX_Tripod: 参考腿长
    double ref_leg_length_proX = kDefaultProXRefLegLength;
};

// BalanceController 提供设置模式与根据当前机器人状态计算控制命令。
class BalanceController {
public:
    explicit BalanceController(BalanceControllerConfig config = {});
    void SetMode(ControlMode mode);
    void Reset();
    ControlMode mode() const;
    std::string ModeName() const;
    ControlCommand Compute(const RobotState& state, const std::array<double, 5>& excitation = {});

    // 设置前进速度目标 (m/s)。位置参考像 MCU 一样保持连续。
    void SetSpeedRef(double v);
    double speed_ref() const;
    void SetYawRateRef(double rate);
    double yaw_rate_ref() const { return yaw_rate_command_; }
    void SetLegLengthRef(double length);
    double leg_length_ref() const;
    void SetTailAngleRef(double angle);
    double tail_angle_ref() const { return tail_angle_command_; }
    // 停止速度/转向，保持位置、航向以及腿长/尾角目标。
    void StopMotion();

private:
    BalanceControllerConfig config_;
    ControlMode mode_ = ControlMode::Tripod;
    PidState leg_length_pid_[2];       // [0]=left, [1]=right
    PidState leg_coordination_pid_;    // 两腿theta协调
    PidState yaw_velocity_pid_;        // yaw角速度闭环（替代原纯阻尼）
    PidState tail_comp_pid_;           // Tripod模式pitch→tail torque补偿
    PidState tail_up_pid_;             // ProX_Tripod: pitch→tail_z offset
    PidState tail_z_pid_;              // ProX_Tripod: tail_z→竖直补偿力
    double speed_command_ = 0.0;       // 遥控/键盘给出的原始前进速度目标 (m/s)
    double speed_ref_ = 0.0;           // MCU 0.12 一阶低通后的速度目标 (m/s)
    double x_ref_ = 0.0;              // 积分的位置参考 (m)
    double filtered_leg_diff_ = 0.0;   // MCU CHASSIS.lpf.leg_diff
    double yaw_rate_command_ = 0.0;
    double yaw_ref_ = 0.0;
    double leg_length_command_ = 0.0;  // 0 表示使用当前模式默认腿长。
    double tail_angle_command_ = 0.0;
    double tail_angle_ref_ = 0.0;
    bool reference_initialized_ = false;
};

// 将虚拟力 (F, Tp) 转换为关节力矩（通过雅可比矩阵）。
// F 为沿腿长方向的径向力，Tp 为垂直于腿的切向力矩。
inline void CalcVmc(double F, double Tp, const double jacobian[2][2], double torque[2]) {
    torque[0] = jacobian[0][0] * F - jacobian[1][0] * Tp;
    torque[1] = jacobian[0][1] * F - jacobian[1][1] * Tp;
}
