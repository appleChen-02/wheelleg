// 仿真与控制器的集中可调常量。
// 将常用的数值放到此处，便于统一查看与修改。
#pragma once

// 控制循环周期（秒）
// Match the MuJoCo physics timestep used by the LQR loop.
inline constexpr double kControlPeriod = 0.00125;

// Rendering is deliberately decoupled from physics.  Keeping this at 60 Hz
// avoids a GPU swap for every 800 Hz physics step and lets simulation run near
// real time without changing the control timing.
inline constexpr double kDefaultRenderFps = 60.0;

// 默认运行时长；负值表示无限运行
inline constexpr double kDefaultDuration = -1.0;

// 重置时施加的初始体姿角（度）
inline constexpr double kInitialThetaDeg = 10.0;

// 在启用控制前，载入模型后需至少等待此时长（秒）
inline constexpr double kInitialDelay = 0.0;

// 判断机器人“稳定”的速度阈值（m/s 或 rad/s）
inline constexpr double kStableVelThreshold = 0.02;

// 连续稳定所需时间（秒），达到后启用控制器
inline constexpr double kStableRequired = 0.5;

// 默认相机参数
inline constexpr double kDefaultCameraAzimuth = -130.0;
inline constexpr double kDefaultCameraElevation = -20.0;
inline constexpr double kDefaultCameraDistance = 1.2;
inline constexpr double kDefaultCameraLookatX = 0.0;
inline constexpr double kDefaultCameraLookatY = 0.0;
inline constexpr double kDefaultCameraLookatZ = 0.15;

// 默认的脉冲致动器参数（可由命令行覆盖）
// 脉冲力矩（算法帧，单位：Nm）
inline constexpr double kDefaultPulseTorque = 0.0;
// 脉冲开始时间（秒）
inline constexpr double kDefaultPulseStart = 0.05;
// 脉冲持续时间（秒）
inline constexpr double kDefaultPulseDuration = 0.10;

// 默认打印周期（秒），用于周期性状态输出
inline constexpr double kDefaultPrintPeriod = 0.02;

// 腿部几何参数（单位：米），用于状态解析与运动学计算
inline constexpr double kLegL1 = 0.215;
inline constexpr double kLegL2 = 0.258;
inline constexpr double kLegL3 = kLegL2;
inline constexpr double kLegL4 = kLegL1;
inline constexpr double kLegL5 = 0.0;

// 关节角偏移，用于统一角度参考系
inline constexpr double kJ0AngleOffset = 3.14159265358979323846;
inline constexpr double kJ1AngleOffset = 0.0;
inline constexpr double kJ2AngleOffset = -3.14159265358979323846;
inline constexpr double kJ3AngleOffset = 6.28318530717958647692;
inline constexpr double kTailAngleOffset = 0.0;

// 平衡控制器默认配置
inline constexpr double kDefaultBodyMass = 13.0;  // MCU BODY_MASS
inline constexpr double kDefaultGravity = 9.81;
inline constexpr double kDefaultMaxJointTorque = 40.0;
inline constexpr double kDefaultMaxWheelTorque = 4.5;
inline constexpr double kDefaultMaxTailTorque = 7.0;
inline constexpr double kDefaultTailKp = 8.0;
inline constexpr double kDefaultTailKd = 0.8;

// Yaw 速度 PID 默认参数（对齐 MCU robot_param_balanced_infantry.h）
inline constexpr double kDefaultYawVelocityKp = 3.0;
inline constexpr double kDefaultYawVelocityKi = 250.0;
inline constexpr double kDefaultYawVelocityKd = 0.1;
inline constexpr double kDefaultYawVelocityMaxOutput = 2.0;
inline constexpr double kDefaultYawVelocityMaxIout = 0.5;

// 两腿协调参数
// 轮距 (m)，用于腿长协调算法中的roll角几何计算
inline constexpr double kWheelBase = 0.2;  // MuJoCo 模型实际轮距

// 两腿 theta 协调 PID 默认参数（对齐 MCU N_CHASSIS_LEG_COOR=0.0）
inline constexpr double kDefaultLegCoordKp = 50.0;
inline constexpr double kDefaultLegCoordKi = 0.0;
inline constexpr double kDefaultLegCoordKd = 25.0;
inline constexpr double kDefaultLegCoordMaxOutput = 6.0;
inline constexpr double kDefaultLegCoordMaxIout = 1.0;

// 腿长范围限制 (m)
inline constexpr double kMinLegLength = 0.16;  // 对齐 MCU MIN_LEG_LENGTH
inline constexpr double kMaxLegLength = 0.30;  // 对齐 MCU MAX_LEG_LENGTH

// Roll 角速度阻尼因子 (N per rad/s)，用于腿部力分配
inline constexpr double kDefaultRollVelLimitFactor = 0.1;  // 对齐 MCU ROLL_VEL_LIMIT_FACTOR

// Tail compensation PID 默认参数（对齐 MCU TAIL_COMP，Ki=0 → dt换算无影响）
inline constexpr double kDefaultTailCompKp = 0.15;
inline constexpr double kDefaultTailCompKi = 0.0;
inline constexpr double kDefaultTailCompKd = 0.01;
inline constexpr double kDefaultTailCompMaxOutput = 0.08;
inline constexpr double kDefaultTailCompMaxIout = 0.0;

// Tripod 尾巴前馈比例（对齐 MCU TAIL_F_ff_RATIO）
inline constexpr double kDefaultTripodTailFfRatio = 0.03;

// Tripod 几何求解器用常数（对齐 MCU robot_param_balanced_infantry.h TAIL_VERSION=1）
inline constexpr double kWheelRadius = 0.13;
inline constexpr double kTailWheelRadius = 0.029;
inline constexpr double kTailLength = 0.240;
inline constexpr double kTailPosOffsetVertical = 0.1105;
inline constexpr double kTailPosOffsetHorizon = 0.07125;
inline constexpr double kTailBetaOmniToHand = 0.383972;
inline constexpr double kTailBetaInitDeg = -14.0;

// ProX_Tripod: tail_up PID（pitch调节，对齐 MCU KP_CHASSIS_TAIL_UP）
inline constexpr double kDefaultProXTailUpKp = 0.08;
inline constexpr double kDefaultProXTailUpKi = 0.0;
inline constexpr double kDefaultProXTailUpKd = 0.01;
inline constexpr double kDefaultProXTailUpMaxOutput = 0.12;
inline constexpr double kDefaultProXTailUpMaxIout = 0.0;

// ProX_Tripod: tail_z PID（竖直虚拟力，对齐 MCU KP_CHASSIS_TAIL_Z）
inline constexpr double kDefaultProXTailZKp = 25.0;
inline constexpr double kDefaultProXTailZKi = 0.0;
inline constexpr double kDefaultProXTailZKd = 3.0;
inline constexpr double kDefaultProXTailZMaxOutput = 5.0;
inline constexpr double kDefaultProXTailZMaxIout = 0.0;

// ProX_Tripod: 腿长PID目标值（不同于基础Tripod，ProX中由tail_z动态调整）
inline constexpr double kDefaultProXRefLegLength = 0.185;
