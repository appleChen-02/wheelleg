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
    initialized_ = false;
    previous_ = {};
    accumulated_x_ = 0.0;
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
    const double raw_yaw = state.body.yaw;   // 真实几何角度，用于 x_dot 旋转变换
    state.body.pitch = -state.body.pitch;
    // yaw 不再取反

    // 世界系线速度（MuJoCo 自由关节的 qvel 在世界系下）
    const double vx_world = data->qvel[body.qvel_adr + 0];
    const double vy_world = data->qvel[body.qvel_adr + 1];
    // 转回机身系：前进方向速度（使用真实 yaw 做几何旋转）
    const double cy = std::cos(raw_yaw);
    const double sy = std::sin(raw_yaw);
    state.body.x_dot =  vx_world * cy + vy_world * sy;
    // side slip（暂不存，但可用于调试）
    // double vy_body = -vx_world * sy + vy_world * cy;

    // 机身系位移：积分前进速度（仿 MCU 逻辑，仅在低速时积分，高速时冻结）
    if (std::fabs(state.body.x_dot) < 0.8) {
        accumulated_x_ += state.body.x_dot * dt;
    }
    state.body.x = accumulated_x_;

    // 世界系角速度 → 机体系角速度 → 欧拉角速率
    // MuJoCo 自由关节 qvel[3:5] = 世界系角速度 [wx_w, wy_w, wz_w]
    const double wx_w = data->qvel[body.qvel_adr + 3];
    const double wy_w = data->qvel[body.qvel_adr + 4];
    const double wz_w = data->qvel[body.qvel_adr + 5];

    // 用四元数构建 R_body_to_world，取 R^T 把 ω_world 转到 ω_body
    const double r11 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r12 = 2.0 * (qx * qy - qw * qz);
    const double r13 = 2.0 * (qx * qz + qw * qy);
    const double r21 = 2.0 * (qx * qy + qw * qz);
    const double r22 = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double r23 = 2.0 * (qy * qz - qw * qx);
    const double r31 = 2.0 * (qx * qz - qw * qy);
    const double r32 = 2.0 * (qy * qz + qw * qx);
    const double r33 = 1.0 - 2.0 * (qx * qx + qy * qy);

    const double wxb = r11 * wx_w + r21 * wy_w + r31 * wz_w;
    const double wyb = r12 * wx_w + r22 * wy_w + r32 * wz_w;
    const double wzb = r13 * wx_w + r23 * wy_w + r33 * wz_w;

    // ZYX 欧拉角速率（使用 flip 前的真实 pitch）
    const double pitch_true = -state.body.pitch;
    const double cr = std::cos(state.body.roll);
    const double sr = std::sin(state.body.roll);
    const double cp = std::cos(pitch_true);
    const double tp = (std::fabs(cp) > 1e-6) ? std::sin(pitch_true) / cp : 0.0;

    state.body.roll_rate  = wxb + wyb * sr * tp + wzb * cr * tp;
    state.body.pitch_rate = -(wyb * cr - wzb * sr);   // pitch 有符号 flip
    state.body.yaw_rate   = ((std::fabs(cp) > 1e-6) ? (wyb * sr + wzb * cr) / cp : 0.0);

    const LegKinematics legs[2] = {state.left_leg, state.right_leg};
    for (int i = 0; i < 2; ++i) {
        state.leg_state[i].theta = kPi / 2.0 - legs[i].phi0 - state.body.pitch;
        state.leg_state[i].theta_dot = -legs[i].dphi0 - state.body.pitch_rate;
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
