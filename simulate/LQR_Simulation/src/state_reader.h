// 从 MuJoCo 的 `mjData` 中读取机器人状态，并构造控制器友好的
// `RobotState` 数据结构的工具函数和类型定义。
#pragma once

#include <array>
#include <string>

#include <mujoco/mujoco.h>

// MuJoCo 模型中致动器的引用。`direction` 用于将算法轴力方向映射
// 到 MuJoCo 中对应致动器的符号方向。
struct ActuatorRef {
    const char* name;
    int id;
    double direction;
};

// MuJoCo 模型中关节的引用，以及对应 qpos/qvel 在数组中的地址。
struct JointRef {
    const char* name;
    int id;
    int qpos_adr;
    int qvel_adr;
    double direction;
};

// 机器人使用的所有致动器与关节的映射表。
struct RobotMap {
    std::array<ActuatorRef, 7> actuators;
    std::array<JointRef, 8> joints;
};

// 单条腿的运动学量。
struct LegKinematics {
    double phi1 = 0.0;
    double phi4 = 0.0;
    double dphi1 = 0.0;
    double dphi4 = 0.0;
    double l0 = 0.0;
    double phi0 = 0.0;
    double dl0 = 0.0;
    double dphi0 = 0.0;
    double jacobian[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
};

// 控制器需要的高层身体状态描述。
struct BodyState {
    double x = 0.0;
    double x_dot = 0.0;
    double roll = 0.0;
    double smooth_roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double roll_rate = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;
};

// 控制器通常使用的每条腿组合状态向量。
struct LegStateVector {
    double theta = 0.0;
    double theta_dot = 0.0;
    double x = 0.0;
    double x_dot = 0.0;
    double phi = 0.0;
    double phi_dot = 0.0;
    double delta_theta = 0.0;
    double delta_theta_dot = 0.0;
    double delta_x = 0.0;
    double delta_x_dot = 0.0;
    double delta_phi = 0.0;
    double delta_phi_dot = 0.0;
};

// StateReader::Read 返回的完整机器人状态摘要。
struct RobotState {
    LegKinematics left_leg;
    LegKinematics right_leg;
    LegStateVector leg_state[2];
    BodyState body;
    double left_wheel_velocity = 0.0;
    double right_wheel_velocity = 0.0;
    double beta = 0.0;
    double dbeta = 0.0;
};

// MuJoCo free-joint 的旋转 qvel 已是局部机体系 p/q/r。经典控制器的
// pitch 轴定义与 MuJoCo 相反，因此只翻转中间分量，绝不再做姿态旋转。
inline std::array<double, 3> ControllerRatesFromFreeJointAngularVelocity(
    double body_p, double body_q, double body_r) {
    return {body_p, -body_q, body_r};
}

// 从 MuJoCo 模型构造和解析 `RobotMap` 的辅助函数。
RobotMap MakeDefaultRobotMap();
void ResolveRobotMap(const mjModel* model, RobotMap* map);
int FindActuatorByName(const RobotMap& map, const std::string& name);

// 控制相关工具：清空控制量并通过映射表设置致动器力矩。
void ClearCtrl(const mjModel* model, mjData* data);
void SetActuatorTorque(const RobotMap& map, mjData* data, int actuator_index, double torque_algorithm_frame);

// 读取当前 `mjData` 并计算适合平衡控制器使用的 `RobotState`。参数 `dt`
// 是控制周期，用于计算有限差分的增量量（delta）。
enum class FeedbackSource { Truth, McuObserver };

class StateReader {
public:
    explicit StateReader(FeedbackSource source = FeedbackSource::Truth) : source_(source) {}
    FeedbackSource source() const { return source_; }
    void Reset();
    // 对齐 MCU ResetXStateOnModeSwitch：清零 x 和 KF 估计，但保留已经
    // 收敛的协方差、IMU/腿姿态历史及速度参考滤波状态。
    void ResetMotionStateForModeSwitch();
    // BodyMotionObserve 在 ChassisReference 之前执行，因此这里保存的是上一控制周期
    // 已滤波的速度参考，用它复现 MCU 的位置反馈积分门控。
    void SetSpeedReference(double speed_reference);
    // 由 1 ms 物理循环调用，独立复现 MCU IMU_task 的 roll 平滑器。
    void UpdateImuRoll(const mjData* data, const RobotMap& map);
    RobotState Read(const mjData* data, const RobotMap& map, double dt);

private:
    FeedbackSource source_;
    bool velocity_deadzone_active_ = true;
    double UpdateVelocityObserver(double speed_measurement, double acceleration_measurement, double dt);
    double UpdateRollSmoother(double raw_roll, double raw_roll_rate);

    bool initialized_ = false;
    std::array<LegStateVector, 2> previous_{};
    double accumulated_x_ = 0.0;   // 机身系 x 位移积分
    double speed_reference_ = 0.0;
    // MCU ChassisObserver 先 UpdateLegStatus、后 UpdateBodyStatus；因此速度
    // 融合式里的 rod.Theta/dTheta 使用上一周期的机体 pitch/pitch_rate。
    double previous_body_pitch_ = 0.0;
    double previous_body_pitch_rate_ = 0.0;
    std::array<double, 2> velocity_observer_state_{};
    std::array<std::array<double, 2>, 2> velocity_observer_covariance_{{
        {{100000.0, 0.0}},
        {{0.0, 100000.0}},
    }};
    bool roll_smoother_initialized_ = false;
    double roll_smoother_last_raw_ = 0.0;
    std::array<double, 4> roll_smoother_state_{};
    std::array<std::array<double, 4>, 4> roll_smoother_covariance_{};
};
