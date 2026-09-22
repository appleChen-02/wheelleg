// 平衡控制器的接口定义与配置结构，供主循环调用。
#pragma once

#include <array>
#include <string>

#include "constants.h"
#include "state_reader.h"

// 控制模式枚举：无尾、双足、三足（或三组）等。
enum class ControlMode {
    NoTail = 0,
    Bipedal = 1,
    Tripod = 2,
    ProXTripod = 3,
};

// 控制输出命令，包含所有致动器的目标力矩。
struct ControlCommand {
    std::array<double, 7> actuator_torque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> vmc_torque = {0.0, 0.0, 0.0, 0.0};  // [L_phi1, L_phi4, R_phi1, R_phi4]
    std::array<double, 2> vmc_F = {0.0, 0.0};    // [L, R] radial force
    std::array<double, 2> vmc_Tp = {0.0, 0.0};   // [L, R] tangential force
    std::array<double, 2> leg_length_error = {0.0, 0.0};  // [L, R] target - actual
    std::array<double, 2> leg_length_F_pid = {0.0, 0.0};  // [L, R] PID output
    std::array<double, 2> leg_length_F_ff = {0.0, 0.0};   // [L, R] feedforward
    double x_ref = 0.0;        // current position reference
    double speed_ref = 0.0;    // current speed reference
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

// 简易 PID 控制器，带输出限幅和积分抗饱和。
struct PidState {
    double kp = 500.0;
    double ki = 50.0;
    double kd = 10.0;
    double max_output = 30.0;
    double max_iout = 20.0;

    double integral = 0.0;
    double error_hist[3] = {0.0, 0.0, 0.0};  // 误差历史: [0]当前 [1]上一次 [2]上上次
    bool initialized = false;

    double Compute(double error, double dt);
    void Reset();
};

// 控制器可配置参数集合（质量、重力、最大力矩和尾巴控制增益等）。
struct BalanceControllerConfig {
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

    // 腿长 PID 参数（对齐 MCU，Kd 二阶差分形式，Ki 已做 dt 量纲换算）
    double ref_leg_length_tripod = 0.185;
    double ref_leg_length_bipedal = 0.18;
    double ref_leg_length_default = 0.17;
    double ff_ratio = 1.0;
    double tail_F_ff_ratio = kDefaultTripodTailFfRatio;
    double leg_length_kp = 600.0;
    double leg_length_ki = 1750.0;
    double leg_length_kd = 620.0;
    double leg_length_max_output = 80.0;
    double leg_length_max_iout = 40.0;

    // 两腿 theta 协调 PID 参数（对齐 MCU，Kd 二阶差分形式，Ki 已做 dt 量纲换算）
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
    ControlMode mode() const;
    std::string ModeName() const;
    ControlCommand Compute(const RobotState& state);

    // 设置前进速度目标 (m/s)，同时会重置积分的位置参考
    void SetSpeedRef(double v);
    double speed_ref() const;

private:
    BalanceControllerConfig config_;
    ControlMode mode_ = ControlMode::Tripod;
    PidState leg_length_pid_[2];       // [0]=left, [1]=right
    PidState leg_coordination_pid_;    // 两腿theta协调
    PidState yaw_velocity_pid_;        // yaw角速度闭环（替代原纯阻尼）
    PidState tail_comp_pid_;           // Tripod模式pitch→tail torque补偿
    PidState tail_up_pid_;             // ProX_Tripod: pitch→tail_z offset
    PidState tail_z_pid_;              // ProX_Tripod: tail_z→竖直补偿力
    double speed_ref_ = 0.0;           // 前进速度目标 (m/s)
    double x_ref_ = 0.0;              // 积分的位置参考 (m)
};

// 将虚拟力 (F, Tp) 转换为关节力矩（通过雅可比矩阵）。
// F 为沿腿长方向的径向力，Tp 为垂直于腿的切向力矩。
inline void CalcVmc(double F, double Tp, const double jacobian[2][2], double torque[2]) {
    torque[0] = jacobian[0][0] * F - jacobian[1][0] * Tp;
    torque[1] = jacobian[0][1] * F - jacobian[1][1] * Tp;
}
