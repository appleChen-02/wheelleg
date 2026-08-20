/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.c/h
  * @brief      平衡底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#ifndef CHASSIS_BIPEDAL_H
#define CHASSIS_BIPEDAL_H

#include "robot_param.h"
#include "IMU_task.h"
#include "custom_typedef.h"
#include "kalman_filter.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "motion_control.h"

/*-------------------- Error flags --------------------*/
// 底盘运行过程中使用的错误位标志，便于统一组合与清除
// clang-format off
#define JOINT_ERROR_OFFSET   ((uint8_t)1)   // 关节电机错误偏移量
#define WHEEL_ERROR_OFFSET   ((uint8_t)2)   // 驱动轮电机错误偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)4)   // dbus错误偏移量
#define IMU_ERROR_OFFSET     ((uint8_t)8)   // imu错误偏移量
#define FLOATING_OFFSET      ((uint8_t)16)  // 悬空状态偏移量
// clang-format on

/*-------------------- Structural definition --------------------*/

/*-------------------- Chassis mode --------------------*/
// 底盘总状态机：从关闭、保护、起立到不同工作模式的切换
typedef enum {
    CHASSIS_OFF,        // 底盘关闭
    CHASSIS_SAFE,       // 底盘无力，所有控制量置0，出错了进入
    CHASSIS_STAND_UP,   // 底盘起立，从倒地状态到站立状态的中间过程
    CHASSIS_NOTAIL,     // 不装尾巴的测试模式（读尾巴输入但是最终输出扭矩先为0）
    CHASSIS_BIPEDAL,    // 单体 装尾巴 尾巴离地抓取
    CHASSIS_TRIPOD,     // 单体 装尾巴 尾巴着地
    CHASSIS_JOINED      // 多体平衡
} ChassisMode_e;

typedef struct Leg
{
    struct rod
    {
        float Phi0;    // rad
        float dPhi0;   // rad/s
        float ddPhi0;  // rad/s^2

        float L0;    // m
        float dL0;   // m/s
        float ddL0;  // m/s^2

        float Theta;    // rad
        float dTheta;   // rad/s
        float ddTheta;  // rad/s^2

        float F;   // N
        float Tp;  // N*m
    } rod;

    struct joint
    {
        float T1, T2;        // N*m
        float Phi1, Phi4;    // rad
        float dPhi1, dPhi4;  // rad/s
    } joint;

    struct wheel
    {
        float Angle;     // rad
        float Velocity;  // rad/s
    } wheel;

    float J[2][2];           // 雅可比矩阵
    float Fn;                // N 支撑力
    uint32_t take_off_time;  // 离地时间
    uint32_t touch_time;     // 触地时间
    bool is_take_off;        // 是否离地
} Leg_t;

typedef struct Body
{
    float x;          // (m) 机体在x轴方向的位置
    float x_dot;      // (m/s) 机体x轴速度直接反馈值
    float x_dot_obv;  // (m/s) 机体x轴速度观测值
    float x_acc;      // (m/s^2) 机体x轴加速度直接反馈值
    float x_acc_obv;  // (m/s^2) 机体x轴加速度观测值

    float x_accel;  // 机体坐标系下x轴加速度
    float y_accel;  // 机体坐标系下y轴加速度
    float z_accel;  // 机体坐标系下z轴加速度

    float gx, gy, gz;  // 重力加速度在机体坐标系下的分量，用于消除重力项影响

    float phi;       // 机体横滚/摆动相关角度
    float phi_dot;   // 机体横滚/摆动角速度

    float roll;        // 横滚角
    float roll_dot;    // 横滚角速度
    float smooth_roll; // 横滚角（IMU Kalman平滑值，供腿长控制用）
    float pitch;     // 俯仰角
    float pitch_dot; // 俯仰角速度
    float yaw;       // 航向角
    float yaw_dot;   // 航向角速度
} Body_t;

typedef struct
{
    float x_accel;  // 世界坐标系下x轴加速度
    float y_accel;  // 世界坐标系下y轴加速度
    float z_accel;  // 世界坐标系下z轴加速度
} World_t;

//状态向量
typedef struct LegState
{
    float theta;      // 腿角/关节角状态
    float theta_dot;  // 腿角角速度
    float x;          // 腿长方向位移
    float x_dot;      // 腿长方向速度
    float phi;        // 杆件/机体相关姿态角
    float phi_dot;    // 姿态角速度

    float Delta_theta;      // 相对目标腿角误差
    float Delta_theta_dot;  // 相对目标腿角速度误差
    float Delta_x;          // 相对目标腿长误差
    float Delta_x_dot;      // 相对目标腿长速度误差
    float Delta_phi;        // 相对目标姿态角误差
    float Delta_phi_dot;    // 相对目标姿态角速度误差

    float last_theta;      // 上一次腿角
    float last_theta_dot;  // 上一次腿角速度
    float last_x;          // 上一次腿长
    float last_x_dot;      // 上一次腿长速度
    float last_phi;        // 上一次姿态角
    float last_phi_dot;    // 上一次姿态角速度
} LegState_t;

// 尾巴控制状态量
typedef struct TailState
{
    float beta;      // 尾巴摆角
    float beta_dot;  // 尾巴摆角速度
} TailState_t;

typedef struct Tail
{
    float Beta;            // rad 尾巴摆角
    float dBeta;           // rad/s 尾巴摆角速度
    float ddBeta;          // rad/s^2 尾巴摆角加速度
    float Tt;              // N*m 尾巴输出扭矩
    float Fn;              // N 支撑力
    uint32_t take_off_time;  // 离地时间
    bool is_take_off;        // 是否离地
    uint32_t touch_time;     // 触地时间
} Tail_t;

/**
 * @brief 状态
 */
typedef struct
{
    Body_t body;              // 机体状态
    World_t world;            // 世界坐标系状态
    Leg_t leg[2];             // 0-左 1-右
    LegState_t leg_state[2];  // 0-左 1-右
    Tail_t tail;              // 尾巴状态
    TailState_t tail_state;    // 尾巴状态量
    ChassisSpeedVector_t speed_vector;  // 底盘速度向量
    float motor_pos[7];           // rad 电机实时角度: [0..3]=J0..J3, [4..5]=W0..W1, [6]=Tail
    float torque[5];            // N*m 控制力矩输出: [0]=T_r_to_b(右髋), [1]=T_l_to_b(左髋), [2]=T_wr_to_r(右轮), [3]=T_wl_to_l(左轮), [4]=T_t_to_b(尾)
} Fdb_t;

/**
 * @brief 期望
 */
typedef struct
{
    Body_t body;              // 期望机体状态
    LegState_t leg_state[2];  // 0-左 1-右
    TailState_t tail_state;   // 期望尾巴状态
    float rod_L0[2];          // 0-左 1-右 杆长期望
    float rod_dL0[2];         // 0-左 1-右 杆长变化率期望
    float rod_Angle[2];       // 0-左 1-右 杆角期望
    ChassisSpeedVector_t speed_vector;  // 期望速度向量
} Ref_t;

typedef struct Cmd
{
    struct leg
    {
        struct rod_cmd
        {
            float F;   // N 杆件输出力
            float Tp;  // N*m 杆件输出力矩
        } rod;
        struct joint_cmd
        {
            float T[2];    // N*m 关节力矩
            float Pos[2];  // rad 关节位置
        } joint;
        struct wheel_cmd
        {
            float T;  // N*m 轮电机力矩
        } wheel;
    } leg[2];  // 0-左 1-右
    struct tail
    {
        float Tt;   // 尾巴力矩
        float Pos;  // 尾巴位置
    } tail;
} Cmd_t;

// PID控制器集合，按功能拆分为速度环、姿态环、腿长/腿角环等
typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;

    pid_type_def vel_add;

#if LOCATION_CONTROL
    pid_type_def roll_angle;

    pid_type_def pitch_angle;
    pid_type_def pitch_vel;
#else

    pid_type_def roll_angle;
    // pid_type_def roll_velocity;

    pid_type_def pitch_angle;
    // pid_type_def pitch_vel;

    pid_type_def leg_length_length[2];
    pid_type_def leg_length_speed[2];

    pid_type_def leg_angle_angle;
    pid_type_def leg_coordation;

    pid_type_def theta_comp;
#endif

    pid_type_def tail_comp;
    pid_type_def tail_up;
    pid_type_def tail_z;
    pid_type_def tail_x;
    pid_type_def tail_T;
    pid_type_def leg_T;
    pid_type_def pitch_dot;
    pid_type_def stand_up;
    pid_type_def wheel_stop[2];
} PID_t;

// 低通滤波器集合，用于腿部加速度、支撑力和姿态量平滑
typedef struct LPF
{
    LowPassFilter_t leg_l0_accel_filter[2];
    LowPassFilter_t leg_phi0_accel_filter[2];
    LowPassFilter_t leg_theta_accel_filter[2];
    LowPassFilter_t support_force_filter[2];
    LowPassFilter_t roll;
    LowPassFilter_t leg_diff;   // 腿长差低通滤波(切断反馈回路正反馈)
} LPF_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    const Imu_t * imu;     // imu数据
    ChassisMode_e mode;    // 底盘模式
    uint8_t error_code;    // 底盘错误代码
    int8_t step;           // 底盘运行步骤号
    uint32_t step_time;    // (ms)底盘步骤运行时间

    /*-------------------- Motors --------------------*/
    // 平衡底盘有2个驱动轮电机和4个关节电机
    Motor_s joint_motor[4];
    Motor_s wheel_motor[2];  // 驱动轮电机 0-左轮，1-右轮
    Motor_s tail_motor;  // 尾巴电机
    /*-------------------- Values --------------------*/

    Ref_t ref;  // 期望值
    Fdb_t fdb;  // 状态值
    Cmd_t cmd;  // 控制量

    PID_t pid;  // PID控制器
    LPF_t lpf;  // 低通滤波器

    uint32_t last_time;  // (ms)上一次更新时间
    uint32_t duration;   // (ms)任务周期
    float dyaw;  // (rad)(feedback)当前位置与云台中值角度差（用于坐标转换）
    uint16_t yaw_mid;  // (ecd)(preset)云台中值角度
} Chassis_s;

typedef struct GroundTouch
{
    uint32_t touch_time;     //记录触地时间
    float force[2];          //记录腿上的力
    float support_force[2];  //(N)地面的支持力 0-左 1-右

    bool touch;  //是否触地
} GroundTouch_s;

typedef struct
{
    struct
    {
        /*
         * 四维机体运动滤波器。状态顺序固定，因为 C 文件会按下标写入 F/H/Q/R
         * 并读取 xhat：[纵向速度、加速度计零偏、偏航角速度、陀螺仪零偏]。
         */
        KalmanFilter_t motion_kf;
    } body;
} Observer_t;

typedef enum {
    WHEEL_SPEED_NORMAL = 0,  /* 轮速正常可信，使用标称测量权重。 */
    WHEEL_SPEED_SUSPECT,     /* 打滑候选需持续一段时间才确认。 */
    WHEEL_SPEED_SLIPPING,    /* 已确认打滑，轮速观测正在降权。 */
    WHEEL_SPEED_RECOVERY,    /* 打滑后保持降权，避免过早恢复可信。 */
} WheelSpeedTrustState_e;

typedef enum {
    CHASSIS_OBSERVER_HEALTH_OK = 0,
    /* 单侧腾空、打滑，或轮速偏航与 IMU 不一致。 */
    CHASSIS_OBSERVER_HEALTH_WHEEL_DEGRADED = 1U << 0,
    /* 两侧均腾空，轮速测量将被近似忽略。 */
    CHASSIS_OBSERVER_HEALTH_AIRBORNE = 1U << 1,
    /* 主动注入零速度/零偏航观测，以约束静止时的漂移。 */
    CHASSIS_OBSERVER_HEALTH_STATIONARY = 1U << 2,
    /* 矩阵运算或状态值无效，已写入有界的回退初值。 */
    CHASSIS_OBSERVER_HEALTH_NUMERIC_FALLBACK = 1U << 3,
} ChassisObserverHealth_e;

typedef struct {
    float vx;                  /* 融合后的纵向速度，m/s。 */
    float accel_bias;          /* 在线估计的加速度计 x 轴零偏，m/s^2。 */
    float wz;                  /* 融合后的底盘偏航角速度，rad/s。 */
    float gyro_bias;           /* 在线估计的 IMU 偏航角速度零偏，rad/s。 */
    float covariance_diag[4];  /* 按上述状态顺序排列的 P 矩阵对角线。 */
    float wheel_vx;            /* 原始左右轮平均速度，m/s。 */
    float wheel_wz;            /* 原始差速轮速推导的偏航角速度，rad/s。 */
    uint8_t health;            /* ChassisObserverHealth_e 标志位的按位或。 */
} ChassisMotionObserverOutput_t;

typedef struct {
    WheelSpeedTrustState_e state[2];  /* [左、右] 轮速状态机状态。 */
    uint8_t slip_mask;                /* SLIPPING 或 RECOVERY 时置位第 i 位。 */
    float slip_speed[2];              /* |轮速 - IMU 参考速度|，m/s。 */
    float imu_velocity;               /* 扣除零偏后的加速度积分速度，m/s。 */
    uint32_t state_time_ms[2];        /* 在各自当前状态中累计的时间，ms。 */
} ChassisWheelSlipStatus_t;

typedef struct {
    float vx_cmd;          /* 原始速度指令，m/s。 */
    float vx_ref;          /* 整形后送入双足 LQR 的速度参考，m/s。 */
    float ax_ref;          /* 整形器输出的纵向加速度参考，m/s^2。 */
    float wheel_torque_ff; /* 叠加到每侧轮子的共模前馈扭矩，N*m。 */
} ChassisVxReferenceOutput_t;

/* 对外观测器调试快照，每个底盘观测器周期更新一次。 */
extern ChassisMotionObserverOutput_t CHASSIS_MOTION_OBSERVER;
extern ChassisWheelSlipStatus_t CHASSIS_WHEEL_SLIP_STATUS;

/* 对外双足速度参考快照，每个参考生成周期更新一次。 */
extern ChassisVxReferenceOutput_t CHASSIS_VX_REFERENCE;

/*-------------------- Public interfaces --------------------*/
// 初始化底盘相关状态、滤波器和控制器参数
extern void ChassisInit(void);
// 处理底盘异常并进入安全状态
extern void ChassisHandleException(void);
// 根据遥控器、状态和任务条件切换底盘模式
extern void ChassisSetMode(void);
// 观测器更新，计算速度等不可直接测得的状态量
extern void ChassisObserver(void);
// 生成本周期参考量
extern void ChassisReference(void);
// 电机离线掩码（供 USB 任务读取）
extern uint8_t g_motor_offline_mask;

// 控制台/调试输出
extern void ChassisConsole(void);
// 将控制量发送到电机驱动
extern void ChassisSendCmd(void);

#endif /* CHASSIS_BIPEDAL_H */
/*------------------------------ End of File ------------------------------*/
