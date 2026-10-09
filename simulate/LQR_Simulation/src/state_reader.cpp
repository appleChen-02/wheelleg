#include "state_reader.h"
#include "constants.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

// 状态读取模块需要的一些内部常量和辅助函数。
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// 将值约束到周期区间 [min_value, max_value]（用于角度环绕处理）。
double LoopConstrain(double value, double min_value, double max_value) {
    if (max_value < min_value) {
        return value;
    }

    const double range = max_value - min_value;
    while (value > max_value) {
        value -= range;
    }
    while (value < min_value) {
        value += range;
    }
    return value;
}

// 根据偏移与方向对关节角进行变换并做环绕（duration=1 表示 [-pi,pi]）。
double ThetaTransform(double angle, double offset, double direction, int duration) {
    return LoopConstrain(angle * direction + offset, -kPi * duration, kPi * duration);
}

// 获取关节位置/速度的便捷函数（从数据数组读取）。
double JointPosition(const mjData* data, const JointRef& joint) {
    return data->qpos[joint.qpos_adr];
}

double JointVelocity(const mjData* data, const JointRef& joint) {
    return data->qvel[joint.qvel_adr];
}

// 计算 l0 和 phi0（腿部第 0 段长度与角度）的几何函数。
void GetL0AndPhi0(double phi1, double phi4, double* l0, double* phi0) {
    const double xb = kLegL1 * std::cos(phi1);
    const double yb = kLegL1 * std::sin(phi1);
    const double xd = kLegL5 + kLegL4 * std::cos(phi4);
    const double yd = kLegL4 * std::sin(phi4);

    const double lbd2 = (xd - xb) * (xd - xb) + (yd - yb) * (yd - yb);
    const double a0 = 2.0 * kLegL2 * (xd - xb);
    const double b0 = 2.0 * kLegL2 * (yd - yb);
    const double c0 = kLegL2 * kLegL2 + lbd2 - kLegL3 * kLegL3;
    const double radicand = std::max(0.0, a0 * a0 + b0 * b0 - c0 * c0);
    const double phi2 = 2.0 * std::atan2(b0 + std::sqrt(radicand), a0 + c0);
    const double xc = kLegL1 * std::cos(phi1) + kLegL2 * std::cos(phi2);
    const double yc = kLegL1 * std::sin(phi1) + kLegL2 * std::sin(phi2);

    *l0 = std::sqrt((xc - kLegL5 / 2.0) * (xc - kLegL5 / 2.0) + yc * yc);
    *phi0 = std::atan2(yc, xc - kLegL5 / 2.0);
}

// 计算腿部雅可比矩阵。
void CalcJacobian(double phi1, double phi4, double jacobian[2][2]) {
    const double xb = kLegL1 * std::cos(phi1);
    const double yb = kLegL1 * std::sin(phi1);
    const double xd = kLegL5 + kLegL4 * std::cos(phi4);
    const double yd = kLegL4 * std::sin(phi4);

    const double lbd2 = (xd - xb) * (xd - xb) + (yd - yb) * (yd - yb);
    const double a0 = 2.0 * kLegL2 * (xd - xb);
    const double b0 = 2.0 * kLegL2 * (yd - yb);
    const double c0 = kLegL2 * kLegL2 + lbd2 - kLegL3 * kLegL3;
    const double radicand = std::max(0.0, a0 * a0 + b0 * b0 - c0 * c0);
    const double phi2 = 2.0 * std::atan2(b0 + std::sqrt(radicand), a0 + c0);
    const double phi3 = std::atan2(yb - yd + kLegL2 * std::sin(phi2),
                                   xb - xd + kLegL2 * std::cos(phi2));

    const double xc = kLegL1 * std::cos(phi1) + kLegL2 * std::cos(phi2);
    const double yc = kLegL1 * std::sin(phi1) + kLegL2 * std::sin(phi2);
    const double l0 = std::sqrt((xc - kLegL5 / 2.0) * (xc - kLegL5 / 2.0) + yc * yc);
    const double phi0 = std::atan2(yc, xc - kLegL5 / 2.0);

    const double sigma1 = std::sin(phi3 - phi2);
    const double sigma2 = std::sin(phi3 - phi4);
    const double sigma3 = std::sin(phi1 - phi2);
    const double sigma4 = std::sin(phi0 - phi3);
    const double sigma5 = std::cos(phi0 - phi3);
    const double sigma6 = std::sin(phi0 - phi2);
    const double sigma7 = std::cos(phi0 - phi2);

    jacobian[0][0] = (kLegL1 * sigma4 * sigma3) / sigma1;
    jacobian[0][1] = (kLegL4 * sigma6 * sigma2) / sigma1;
    jacobian[1][0] = (kLegL1 * sigma5 * sigma3) / (l0 * sigma1);
    jacobian[1][1] = (kLegL4 * sigma7 * sigma2) / (l0 * sigma1);
}

// 根据雅可比矩阵计算 l0 与 phi0 的时间导数。
void GetdL0AnddPhi0(const double jacobian[2][2], double dphi1, double dphi4, double* dl0, double* dphi0) {
    *dl0 = jacobian[0][0] * dphi1 + jacobian[0][1] * dphi4;
    *dphi0 = jacobian[1][0] * dphi1 + jacobian[1][1] * dphi4;
}

// 组合上述步骤计算单条腿的运动学量并填充到 LegKinematics 结构中。
void CalcLegKinematics(double phi1, double phi4, double dphi1, double dphi4, LegKinematics* leg) {
    leg->phi1 = phi1;
    leg->phi4 = phi4;
    leg->dphi1 = dphi1;
    leg->dphi4 = dphi4;
    GetL0AndPhi0(phi1, phi4, &leg->l0, &leg->phi0);
    CalcJacobian(phi1, phi4, leg->jacobian);
    GetdL0AnddPhi0(leg->jacobian, dphi1, dphi4, &leg->dl0, &leg->dphi0);
}

// 四元数到欧拉角的转换（roll, pitch, yaw）。
void QuaternionToEuler(double qw, double qx, double qy, double qz,
                       double* roll, double* pitch, double* yaw) {
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    *roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::fabs(sinp) >= 1.0) {
        *pitch = std::copysign(kPi / 2.0, sinp);
    } else {
        *pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    *yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

RobotMap MakeDefaultRobotMap() {
    RobotMap map{
        .actuators = {{
            {"left_hip_motor", -1, -1.0},
            {"left_parallel_1_motor", -1, -1.0},
            {"right_hip_motor", -1, -1.0},
            {"right_parallel_1_motor", -1, -1.0},
            {"left_podarthrum_motor", -1, 1.0},
            {"right_podarthrum_motor", -1, 1.0},
            {"front_hip_motor", -1, 1.0},
        }},
        .joints = {{
            {"left_hip_joint", -1, -1, -1, -1.0},
            {"left_parallel_1_joint", -1, -1, -1, -1.0},
            {"right_hip_joint", -1, -1, -1, -1.0},
            {"right_parallel_1_joint", -1, -1, -1, -1.0},
            {"left_podarthrum_joint", -1, -1, -1, 1.0},
            {"right_podarthrum_joint", -1, -1, -1, 1.0},
            {"front_hip_joint", -1, -1, -1, 1.0},
            {"robot_body_joint", -1, -1, -1, 1.0},
        }},
    };
    return map;
}

void ResolveRobotMap(const mjModel* model, RobotMap* map) {
    for (auto& actuator : map->actuators) {
        actuator.id = mj_name2id(model, mjOBJ_ACTUATOR, actuator.name);
        if (actuator.id < 0) {
            throw std::runtime_error(std::string("Missing actuator: ") + actuator.name);
        }
    }

    for (auto& joint : map->joints) {
        joint.id = mj_name2id(model, mjOBJ_JOINT, joint.name);
        if (joint.id < 0) {
            throw std::runtime_error(std::string("Missing joint: ") + joint.name);
        }
        joint.qpos_adr = model->jnt_qposadr[joint.id];
        joint.qvel_adr = model->jnt_dofadr[joint.id];
    }
}

int FindActuatorByName(const RobotMap& map, const std::string& name) {
    for (int i = 0; i < static_cast<int>(map.actuators.size()); ++i) {
        if (name == map.actuators[i].name) {
            return i;
        }
    }
    return -1;
}

void ClearCtrl(const mjModel* model, mjData* data) {
    std::fill(data->ctrl, data->ctrl + model->nu, 0.0);
}

void SetActuatorTorque(const RobotMap& map, mjData* data, int actuator_index, double torque_algorithm_frame) {
    const auto& actuator = map.actuators[actuator_index];
    data->ctrl[actuator.id] = torque_algorithm_frame * actuator.direction;
}

void StateReader::Reset() {
    velocity_deadzone_active_ = true;
    initialized_ = false;
    previous_ = {};
    accumulated_x_ = 0.0;
    speed_reference_ = 0.0;
    previous_body_pitch_ = 0.0;
    previous_body_pitch_rate_ = 0.0;
    velocity_observer_state_ = {};
    velocity_observer_covariance_ = {{{{100000.0, 0.0}}, {{0.0, 100000.0}}}};
    roll_smoother_initialized_ = false;
    roll_smoother_last_raw_ = 0.0;
    roll_smoother_state_ = {};
    roll_smoother_covariance_ = {};
}

void StateReader::SetSpeedReference(double speed_reference) {
    speed_reference_ = speed_reference;
}

void StateReader::ResetMotionStateForModeSwitch() {
    accumulated_x_ = 0.0;
    velocity_observer_state_ = {};
    for (auto& previous : previous_) {
        previous.x = 0.0;
        previous.x_dot = 0.0;
        previous.delta_x = 0.0;
        previous.delta_x_dot = 0.0;
    }
}

double StateReader::UpdateRollSmoother(double raw_roll, double raw_roll_rate) {
    if (!roll_smoother_initialized_) {
        roll_smoother_state_ = {raw_roll, 0.0, raw_roll, 0.0};
        roll_smoother_covariance_ = {};
        for (int i = 0; i < 4; ++i) roll_smoother_covariance_[i][i] = 1.0;
        roll_smoother_last_raw_ = raw_roll;
        roll_smoother_initialized_ = true;
        return raw_roll;
    }

    const double wrapped_last = std::remainder(roll_smoother_last_raw_, 2.0 * kPi);
    const double raw_unwrapped = roll_smoother_last_raw_
                               + std::remainder(raw_roll - wrapped_last, 2.0 * kPi);
    roll_smoother_last_raw_ = raw_unwrapped;

    double f[4][4] = {};
    f[0][0] = 1.0;
    f[0][1] = kMcuImuPeriod;
    f[1][1] = 1.0;
    f[2][2] = 1.0;
    f[2][3] = kMcuImuPeriod;
    f[3][0] = kMcuImuPeriod * kRollWheelNaturalFrequency * kRollWheelNaturalFrequency;
    f[3][2] = -f[3][0];
    f[3][3] = 1.0 - 2.0 * kMcuImuPeriod * kRollWheelDampingRatio
                    * kRollWheelNaturalFrequency;

    double x_minus[4] = {};
    double fp[4][4] = {};
    double p_minus[4][4] = {};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            x_minus[i] += f[i][j] * roll_smoother_state_[j];
            for (int n = 0; n < 4; ++n) {
                fp[i][j] += f[i][n] * roll_smoother_covariance_[n][j];
            }
        }
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int n = 0; n < 4; ++n) p_minus[i][j] += fp[i][n] * f[j][n];
        }
        p_minus[i][i] += kRollProcessNoise[i];
    }

    double gain[4] = {};
    double s = std::max(p_minus[2][2] + kRollAngleMeasurementVariance, 1.0e-9);
    for (int i = 0; i < 4; ++i) gain[i] = p_minus[i][2] / s;
    const double angle_innovation = raw_unwrapped - x_minus[2];
    for (int i = 0; i < 4; ++i) {
        roll_smoother_state_[i] = x_minus[i] + gain[i] * angle_innovation;
        for (int j = 0; j < 4; ++j) {
            roll_smoother_covariance_[i][j] = p_minus[i][j] - gain[i] * p_minus[2][j];
        }
    }

    // 第二个标量测量按 MCU 原代码顺序用 gyro roll rate 更新 x[3]。
    s = std::max(roll_smoother_covariance_[3][3] + kRollRateMeasurementVariance, 1.0e-9);
    for (int i = 0; i < 4; ++i) gain[i] = roll_smoother_covariance_[i][3] / s;
    const double rate_innovation = raw_roll_rate - roll_smoother_state_[3];
    const auto covariance_row3 = roll_smoother_covariance_[3];
    for (int i = 0; i < 4; ++i) {
        roll_smoother_state_[i] += gain[i] * rate_innovation;
        for (int j = 0; j < 4; ++j) {
            roll_smoother_covariance_[i][j] -= gain[i] * covariance_row3[j];
        }
    }
    return std::remainder(roll_smoother_state_[0], 2.0 * kPi);
}

void StateReader::UpdateImuRoll(const mjData* data, const RobotMap& map) {
    if (source_ == FeedbackSource::Truth) return;
    const auto& body = map.joints[7];
    const double qw = data->qpos[body.qpos_adr + 3];
    const double qx = data->qpos[body.qpos_adr + 4];
    const double qy = data->qpos[body.qpos_adr + 5];
    const double qz = data->qpos[body.qpos_adr + 6];
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    QuaternionToEuler(qw, qx, qy, qz, &roll, &pitch, &yaw);

    // Free-joint qvel 的旋转三项已经是局部机体系陀螺角速度 p/q/r。
    const double wxb = data->qvel[body.qvel_adr + 3];
    const double wyb = data->qvel[body.qvel_adr + 4];
    const double wzb = data->qvel[body.qvel_adr + 5];
    const double cp = std::cos(pitch);
    const double roll_rate = wxb + wyb * std::sin(roll) * std::tan(pitch)
                           + wzb * std::cos(roll) * std::tan(pitch);
    UpdateRollSmoother(roll, std::fabs(cp) > 1.0e-6 ? roll_rate : wxb);
}

double StateReader::UpdateVelocityObserver(double speed_measurement,
                                           double acceleration_measurement,
                                           double dt) {
    // 与 MCU Kalman_Filter_Update 相同的 2x2 线性 KF：
    // x=[v,a], F=[1 dt;0 1], H=I。
    const double x_minus[2] = {
        velocity_observer_state_[0] + dt * velocity_observer_state_[1],
        velocity_observer_state_[1],
    };

    const double p00 = velocity_observer_covariance_[0][0];
    const double p01 = velocity_observer_covariance_[0][1];
    const double p10 = velocity_observer_covariance_[1][0];
    const double p11 = velocity_observer_covariance_[1][1];
    const double pm00 = p00 + dt * (p10 + p01) + dt * dt * p11
                      + kVelocityProcessNoise;
    const double pm01 = p01 + dt * p11;
    const double pm10 = p10 + dt * p11;
    const double pm11 = p11 + kAccelerationProcessNoise;

    // S=P-+R。R 为对角阵，显式求逆可避免引入额外矩阵库。
    const double s00 = pm00 + kVelocityMeasurementNoise;
    const double s01 = pm01;
    const double s10 = pm10;
    const double s11 = pm11 + kAccelerationMeasurementNoise;
    const double determinant = s00 * s11 - s01 * s10;
    const double inv_s00 = s11 / determinant;
    const double inv_s01 = -s01 / determinant;
    const double inv_s10 = -s10 / determinant;
    const double inv_s11 = s00 / determinant;

    const double k00 = pm00 * inv_s00 + pm01 * inv_s10;
    const double k01 = pm00 * inv_s01 + pm01 * inv_s11;
    const double k10 = pm10 * inv_s00 + pm11 * inv_s10;
    const double k11 = pm10 * inv_s01 + pm11 * inv_s11;
    const double innovation0 = speed_measurement - x_minus[0];
    const double innovation1 = acceleration_measurement - x_minus[1];
    velocity_observer_state_[0] = x_minus[0] + k00 * innovation0 + k01 * innovation1;
    velocity_observer_state_[1] = x_minus[1] + k10 * innovation0 + k11 * innovation1;

    // MCU 实现采用 P=(I-KH)P-（H=I），保持相同更新形式。
    velocity_observer_covariance_[0][0] = (1.0 - k00) * pm00 - k01 * pm10;
    velocity_observer_covariance_[0][1] = (1.0 - k00) * pm01 - k01 * pm11;
    velocity_observer_covariance_[1][0] = -k10 * pm00 + (1.0 - k11) * pm10;
    velocity_observer_covariance_[1][1] = -k10 * pm01 + (1.0 - k11) * pm11;
    return velocity_observer_state_[0];
}

RobotState StateReader::Read(const mjData* data, const RobotMap& map, double dt) {
    RobotState state;

    const auto& j0 = map.joints[0];
    const auto& j1 = map.joints[1];
    const auto& j2 = map.joints[2];
    const auto& j3 = map.joints[3];
    const auto& w0 = map.joints[4];
    const auto& w1 = map.joints[5];
    const auto& tail = map.joints[6];
    const auto& body = map.joints[7];

    double left_phi1 = kPi - JointPosition(data, j1);
    double left_phi4 = -JointPosition(data, j0);

    double right_phi1 = kPi - JointPosition(data, j3);
    double right_phi4 = -JointPosition(data, j2);

    CalcLegKinematics(left_phi1, left_phi4, JointVelocity(data, j1) * j1.direction,
                      JointVelocity(data, j0) * j0.direction, &state.left_leg);
    CalcLegKinematics(right_phi1, right_phi4, JointVelocity(data, j3) * j3.direction,
                      JointVelocity(data, j2) * j2.direction, &state.right_leg);

    state.left_wheel_velocity = JointVelocity(data, w0) * w0.direction;
    state.right_wheel_velocity = JointVelocity(data, w1) * w1.direction;
    state.beta = ThetaTransform(JointPosition(data, tail), kTailAngleOffset, tail.direction, 1);
    state.dbeta = JointVelocity(data, tail) * tail.direction;

    const double qw = data->qpos[body.qpos_adr + 3];
    const double qx = data->qpos[body.qpos_adr + 4];
    const double qy = data->qpos[body.qpos_adr + 5];
    const double qz = data->qpos[body.qpos_adr + 6];
    QuaternionToEuler(qw, qx, qy, qz, &state.body.roll, &state.body.pitch, &state.body.yaw);
    state.body.pitch = -state.body.pitch;
    // yaw 不再取反

    // 四元数旋转矩阵仅供后面的世界系线加速度转机体系使用。
    const double r11 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r21 = 2.0 * (qx * qy + qw * qz);
    const double r31 = 2.0 * (qx * qz - qw * qy);

    // MCU GetImuVelocity 直接返回机体系陀螺仪 p/q/r，并未转换成欧拉角导数。
    // pitch 角在仿真控制约定中取反，所以对应 q 轴角速度也取反。
    const auto controller_rates = ControllerRatesFromFreeJointAngularVelocity(
        data->qvel[body.qvel_adr + 3],
        data->qvel[body.qvel_adr + 4],
        data->qvel[body.qvel_adr + 5]);
    state.body.roll_rate = controller_rates[0];
    state.body.pitch_rate = controller_rates[1];
    state.body.yaw_rate = controller_rates[2];
    if (source_ == FeedbackSource::McuObserver && !roll_smoother_initialized_) {
        UpdateRollSmoother(state.body.roll, state.body.roll_rate);
    }
    state.body.smooth_roll = source_ == FeedbackSource::Truth ? state.body.roll
        : std::remainder(roll_smoother_state_[0], 2.0 * kPi);

    const LegKinematics legs[2] = {state.left_leg, state.right_leg};
    // 先构造腿角；MCU 轮速观测量包含 l*cos(theta)*dtheta。
    for (int i = 0; i < 2; ++i) {
        state.leg_state[i].theta = kPi / 2.0 - legs[i].phi0 - state.body.pitch;
        state.leg_state[i].theta_dot = -legs[i].dphi0 - state.body.pitch_rate;
    }

    const double observer_theta[2] = {
        kPi / 2.0 - legs[0].phi0 - previous_body_pitch_,
        kPi / 2.0 - legs[1].phi0 - previous_body_pitch_,
    };
    const double observer_theta_dot[2] = {
        -legs[0].dphi0 - previous_body_pitch_rate_,
        -legs[1].dphi0 - previous_body_pitch_rate_,
    };
    const double speed_measurement =
        (kWheelRadius * state.left_wheel_velocity
         + std::cos(observer_theta[0]) * legs[0].l0 * observer_theta_dot[0]
         + kWheelRadius * state.right_wheel_velocity
         + std::cos(observer_theta[1]) * legs[1].l0 * observer_theta_dot[1]) / 2.0;

    // qacc 的自由关节平移分量是世界系质心坐标加速度；旋转到机体系后
    // 等价于 MCU 中已完成重力补偿的 x_accel。
    const double acceleration_measurement =
        r11 * data->qacc[body.qvel_adr + 0]
        + r21 * data->qacc[body.qvel_adr + 1]
        + r31 * data->qacc[body.qvel_adr + 2];
    if (source_ == FeedbackSource::Truth) {
        // LQR 的 x 是沿水平航向的纵向路程，随转向累计；连续运动时也积分。
        state.body.x_dot = std::cos(state.body.yaw) * data->qvel[body.qvel_adr]
                         + std::sin(state.body.yaw) * data->qvel[body.qvel_adr + 1];
        accumulated_x_ += state.body.x_dot * dt;
    } else {
        // 仿真传感量无零偏，直接复现固件 KF、速度滞回死区和积分门控。
        state.body.x_dot = UpdateVelocityObserver(speed_measurement, acceleration_measurement, dt);
        if (velocity_deadzone_active_) {
            if (std::fabs(state.body.x_dot) > kVelocityDeadzoneExit) velocity_deadzone_active_ = false;
        } else if (std::fabs(state.body.x_dot) < kVelocityDeadzoneEnter) {
            velocity_deadzone_active_ = true;
        }
        if (velocity_deadzone_active_) state.body.x_dot = 0.0;
        // Observer 在 Reference 之前运行，使用上一周期的速度目标。
        if (std::fabs(speed_reference_) < kMcuWheelDeadzone && std::fabs(state.body.x_dot) < 0.8) {
            accumulated_x_ += state.body.x_dot * dt;
        }
    }
    state.body.x = accumulated_x_;
    previous_body_pitch_ = state.body.pitch;
    previous_body_pitch_rate_ = state.body.pitch_rate;

    for (int i = 0; i < 2; ++i) {
        state.leg_state[i].x = state.body.x;
        state.leg_state[i].x_dot = state.body.x_dot;
        state.leg_state[i].phi = state.body.pitch;
        state.leg_state[i].phi_dot = state.body.pitch_rate;

        if (initialized_ && dt > 0.0) {
            state.leg_state[i].delta_theta = state.leg_state[i].theta - previous_[i].theta;
            state.leg_state[i].delta_theta_dot = state.leg_state[i].theta_dot - previous_[i].theta_dot;
            state.leg_state[i].delta_x = state.leg_state[i].x - previous_[i].x;
            state.leg_state[i].delta_x_dot = state.leg_state[i].x_dot - previous_[i].x_dot;
            state.leg_state[i].delta_phi = state.leg_state[i].phi - previous_[i].phi;
            state.leg_state[i].delta_phi_dot = state.leg_state[i].phi_dot - previous_[i].phi_dot;
        }
    }

    previous_[0] = state.leg_state[0];
    previous_[1] = state.leg_state[1];
    initialized_ = true;

    return state;
}
