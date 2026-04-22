/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.c/h
  * @brief      平衡底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.0.2     Sep-16-2024     Penguin         1. 添加速度观测器并测试效果
  *  V1.0.3     Nov-20-2024     Penguin         1. 完善离地检测
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  
  @todo:
    2.添加状态清零，当运行过程中出现异常时可以手动将底盘状态清零
    3.在浮空时通过动量守恒维持底盘的平衡，并调整合适的触地姿态

  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "chassis_balance.h"

#include "CAN_communication.h"
#include "clamp_control.h"
#include "IMU.h"
#include "bsp_delay.h"
#include "chassis.h"
#include "chassis_balance_extras.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "detect_task.h"
#include "kalman_filter.h"
#include "macro_typedef.h"
#include "signal_generator.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "usb_debug.h"
#include "user_lib.h"

// 一些内部的配置
#define TAKE_OFF_DETECT 1  // 启用离地检测
#define CLOSE_LEG_LEFT 0   // 关闭左腿输出
#define CLOSE_LEG_RIGHT 0  // 关闭右腿输出
#define LIFTED_UP 0        // 被架起
#define PRO_CONTROLLER 1   //
#define VX_REF_ALPHA 0.12f

// Parameters on ---------------------
#define MS_TO_S 0.001f

#define VEL_PROCESS_NOISE 25   // 速度过程噪声
#define VEL_MEASURE_NOISE 800  // 速度测量噪声
// 同时估计加速度和速度时对加速度的噪声
// 更好的方法是设置为动态,当有冲击时/加加速度大时更相信轮速
#define ACC_PROCESS_NOISE 2000  // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.01  // 加速度测量噪声

// 支持力阈值，当支持力小于这个值时认为离地
#define TAKE_OFF_FN_THRESHOLD (3.0f)
// 触地状态切换时间阈值，当时间接触或离地时间超过这个值时切换触地状态
#define TOUCH_TOGGLE_THRESHOLD (100)
// Parameters off ---------------------

// Step definitions on ---------------------
// clang-format off
#define NORMAL_STEP        0  // 正常状态
#define JUMP_STEP_SQUST    1  // 跳跃状态——蹲下
#define JUMP_STEP_JUMP     2  // 跳跃状态——跳跃
#define JUMP_STEP_RECOVERY 3  // 跳跃状态——收腿
// clang-format on
// Step definitions on ---------------------

// Step time definitions on ---------------------
// clang-format off
#define MAX_STEP_TIME           5000  // 最大步骤时间

#define NORMAL_STEP_TIME        0  // 正常状态
#define JUMP_STEP_TIME_SQUST    50  // 跳跃状态——蹲下
#define JUMP_STEP_TIME_JUMP     50  // 跳跃状态——跳跃
#define JUMP_STEP_TIME_RECOVERY 200  // 跳跃状态——收腿
// clang-format on
// Step time definitions on ---------------------

#define rc_deadband_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }

static Observer_t OBSERVER;

Chassis_s CHASSIS = {
    .mode = CHASSIS_OFF,
    .error_code = 0,
    .yaw_mid = 0,
    .dyaw = 0.0f,
};

int8_t TRANSITION_MATRIX[10] = {0};

float last_dBeta;
float leg_ref;
int32_t counter_beta = 0;
int32_t counter_beta_tripod = 0;
ChassisMode_e last_mode = CHASSIS_TRIPOD;

/*-------------------- Publish --------------------*/

void ChassisPublish(void)
{
    Publish(&CHASSIS.fdb.speed_vector, CHASSIS_FDB_SPEED_NAME);
    Publish(&CHASSIS.ref.speed_vector, CHASSIS_REF_SPEED_NAME);
}
static void ResetXStateOnModeSwitch(void);

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      ChassisInit                                */
/* auxiliary function: None                                       */
/******************************************************************/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    CHASSIS.rc = get_remote_control_point();  // 获取遥控器指针
    CHASSIS.imu = Subscribe(IMU_NAME);        // 获取IMU数据指针
    /*-------------------- 初始化状态转移矩阵 --------------------*/
    TRANSITION_MATRIX[NORMAL_STEP] = NORMAL_STEP;
    TRANSITION_MATRIX[JUMP_STEP_SQUST] = JUMP_STEP_JUMP;
    TRANSITION_MATRIX[JUMP_STEP_JUMP] = JUMP_STEP_RECOVERY;
    TRANSITION_MATRIX[JUMP_STEP_RECOVERY] = NORMAL_STEP;
    /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(
        &CHASSIS.joint_motor[0], 1, JOINT_CAN, MG_6012, J0_DIRECTION, LK_RATIO_MG6012, 0);  //左前
    MotorInit(
        &CHASSIS.joint_motor[1], 2, JOINT_CAN, MG_6012, J1_DIRECTION, LK_RATIO_MG6012, 0);  //左后
    MotorInit(
        &CHASSIS.joint_motor[2], 3, JOINT_CAN, MG_6012, J2_DIRECTION, LK_RATIO_MG6012, 0);  //右前
    MotorInit(
        &CHASSIS.joint_motor[3], 4, JOINT_CAN, MG_6012, J3_DIRECTION, LK_RATIO_MG6012, 0);  //右后

    MotorInit(&CHASSIS.wheel_motor[0], 1, WHEEL_CAN, MF_9025, W0_DIRECTION, 1, 0);  //左轮
    MotorInit(&CHASSIS.wheel_motor[1], 2, WHEEL_CAN, MF_9025, W1_DIRECTION, 1, 0);  //右轮

    MotorInit(&CHASSIS.tail_motor, 3, WHEEL_CAN, MG_5010, T_DIRECTION, LK_RATIO_MG5010, 0);  //尾巴

    // CHASSIS.fdb.leg[0].joint.Phi1 = M_PI;
    // CHASSIS.fdb.leg[0].joint.Phi4 = 0.0f;
    // CHASSIS.fdb.leg[1].joint.Phi1 = M_PI;
    // CHASSIS.fdb.leg[1].joint.Phi4 = 0.0f;

    /*-------------------- 值归零 --------------------*/
    memset(&CHASSIS.fdb, 0, sizeof(CHASSIS.fdb));
    memset(&CHASSIS.ref, 0, sizeof(CHASSIS.ref));

    CHASSIS.fdb.leg[0].is_take_off = false;
    CHASSIS.fdb.leg[1].is_take_off = false;

    /*-------------------- 初始化底盘PID --------------------*/
    // yaw轴跟踪pid
    float yaw_angle_pid[3] = {KP_CHASSIS_YAW_ANGLE, KI_CHASSIS_YAW_ANGLE, KD_CHASSIS_YAW_ANGLE};
    float yaw_velocity_pid[3] = {
        KP_CHASSIS_YAW_VELOCITY, KI_CHASSIS_YAW_VELOCITY, KD_CHASSIS_YAW_VELOCITY};
    PID_init(
        &CHASSIS.pid.yaw_angle, PID_POSITION, yaw_angle_pid, MAX_OUT_CHASSIS_YAW_ANGLE,
        MAX_IOUT_CHASSIS_YAW_ANGLE);
    PID_init(
        &CHASSIS.pid.yaw_velocity, PID_POSITION, yaw_velocity_pid, MAX_OUT_CHASSIS_YAW_VELOCITY,
        MAX_IOUT_CHASSIS_YAW_VELOCITY);

    // 速度增量pid
    float vel_add_pid[3] = {KP_CHASSIS_VEL_ADD, KI_CHASSIS_VEL_ADD, KD_CHASSIS_VEL_ADD};
    PID_init(
        &CHASSIS.pid.vel_add, PID_POSITION, vel_add_pid, MAX_OUT_CHASSIS_VEL_ADD,
        MAX_IOUT_CHASSIS_VEL_ADD);

    // 两腿一致pid
    float LegCoordination_pid[3] = {KP_CHASSIS_LEG_COOR, KI_CHASSIS_LEG_COOR, KD_CHASSIS_LEG_COOR};
    PID_init(
        &CHASSIS.pid.leg_coordation, PID_POSITION, LegCoordination_pid, MAX_OUT_CHASSIS_LEG_COOR,
        MAX_IOUT_CHASSIS_LEG_COOR);
    PID_PositionSetEkSumRange(
        &CHASSIS.pid.leg_coordation, ERRORSUM_LOW_LEG_COOR, ERRORSUM_UP_LEG_COOR);  //积分分离

    // 尾巴越障pid
    float tail_comp_pid[3] = {KP_CHASSIS_TAIL_COMP, KI_CHASSIS_TAIL_COMP, KD_CHASSIS_TAIL_COMP};
    PID_init(
        &CHASSIS.pid.tail_comp, PID_POSITION, tail_comp_pid, MAX_OUT_CHASSIS_TAIL_COMP,
        MAX_IOUT_CHASSIS_TAIL_COMP);

    // 尾巴pitch pid
    float tail_up_pid[3] = {KP_CHASSIS_TAIL_UP, KI_CHASSIS_TAIL_UP, KD_CHASSIS_TAIL_UP};
    PID_init(
        &CHASSIS.pid.tail_up, PID_POSITION, tail_up_pid, MAX_OUT_CHASSIS_TAIL_UP,
        MAX_IOUT_CHASSIS_TAIL_UP);
    //    PID_PositionSetEkSumRange(
    //        &CHASSIS.pid.tail_comp, ERRORSUM_LOW_TAIL_COMP, ERRORSUM_UP_TAIL_COMP);  //积分分离

    // 尾巴竖直虚拟力pid
    float tail_z_pid[3] = {KP_CHASSIS_TAIL_Z, KI_CHASSIS_TAIL_Z, KD_CHASSIS_TAIL_Z};
    PID_init(
        &CHASSIS.pid.tail_z, PID_POSITION, tail_z_pid, MAX_OUT_CHASSIS_TAIL_Z,
        MAX_IOUT_CHASSIS_TAIL_Z);

    // 尾巴位移补偿
    float tail_x_pid[3] = {KP_CHASSIS_TAIL_X, KI_CHASSIS_TAIL_X, KD_CHASSIS_TAIL_X};
    PID_init(
        &CHASSIS.pid.tail_x, PID_POSITION, tail_x_pid, MAX_OUT_CHASSIS_TAIL_X,
        MAX_IOUT_CHASSIS_TAIL_X);

    // 尾巴扭矩补偿
    float tail_T_pid[3] = {KP_CHASSIS_TAIL_TORQUE, KI_CHASSIS_TAIL_TORQUE, KD_CHASSIS_TAIL_TORQUE};
    PID_init(
        &CHASSIS.pid.tail_T, PID_POSITION, tail_T_pid, MAX_OUT_CHASSIS_TAIL_TORQUE,
        MAX_IOUT_CHASSIS_TAIL_TORQUE);

    // 尾巴水平虚拟力pid
    float leg_T_pid[3] = {KP_CHASSIS_LEG_T, KI_CHASSIS_LEG_T, KD_CHASSIS_LEG_T};
    PID_init(
        &CHASSIS.pid.leg_T, PID_POSITION, leg_T_pid, MAX_OUT_CHASSIS_LEG_T, MAX_IOUT_CHASSIS_LEG_T);

    // 尾巴水平虚拟力pid
    float pitch_dot_pid[3] = {KP_CHASSIS_PITCH_DOT, KI_CHASSIS_PITCH_DOT, KD_CHASSIS_PITCH_DOT};
    PID_init(
        &CHASSIS.pid.pitch_dot, PID_POSITION, pitch_dot_pid, MAX_OUT_CHASSIS_PITCH_DOT,
        MAX_IOUT_CHASSIS_PITCH_DOT);

    /*========== Start of locomotion control pid ==========*/

    float roll_angle_pid[3] = {KP_CHASSIS_ROLL_ANGLE, KI_CHASSIS_ROLL_ANGLE, KD_CHASSIS_ROLL_ANGLE};
    // float roll_velocity_pid[3] = {
    //     KP_CHASSIS_ROLL_VELOCITY, KI_CHASSIS_ROLL_VELOCITY, KD_CHASSIS_ROLL_VELOCITY};

    float leg_length_length_pid[3] = {
        KP_CHASSIS_LEG_LENGTH_LENGTH, KI_CHASSIS_LEG_LENGTH_LENGTH, KD_CHASSIS_LEG_LENGTH_LENGTH};

    PID_init(
        &CHASSIS.pid.roll_angle, PID_POSITION, roll_angle_pid, MAX_OUT_CHASSIS_ROLL_ANGLE,
        MAX_IOUT_CHASSIS_ROLL_ANGLE);
    CHASSIS.pid.roll_angle.N = N_CHASSIS_ROLL_ANGLE;

    float theta_comp_pid[3] = {KP_CHASSIS_THETA_COMP, KI_CHASSIS_THETA_COMP, KD_CHASSIS_THETA_COMP};
    PID_init(
        &CHASSIS.pid.theta_comp, PID_POSITION, theta_comp_pid, MAX_OUT_CHASSIS_THETA_COMP,
        MAX_IOUT_CHASSIS_THETA_COMP);
    CHASSIS.pid.roll_angle.N = N_CHASSIS_THETA_COMP;

    // PID_init(
    //     &CHASSIS.pid.roll_velocity, PID_POSITION, roll_velocity_pid, MAX_OUT_CHASSIS_ROLL_VELOCITY,
    //     MAX_IOUT_CHASSIS_ROLL_VELOCITY);

    PID_init(
        &CHASSIS.pid.leg_length_length[0], PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    CHASSIS.pid.leg_length_length[0].N = N_LEG_LENGTH_LENGTH;
    PID_PositionSetEkSumRange(
        &CHASSIS.pid.leg_length_length[0], ERRORSUM_LOW_LEG_LENGTH, ERRORSUM_UP_LEG_LENGTH);

    PID_init(
        &CHASSIS.pid.leg_length_length[1], PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    CHASSIS.pid.leg_length_length[1].N = N_LEG_LENGTH_LENGTH;
    PID_PositionSetEkSumRange(
        &CHASSIS.pid.leg_length_length[1], ERRORSUM_LOW_LEG_LENGTH, ERRORSUM_UP_LEG_LENGTH);

    /*========== End of locomotion control pid ==========*/

    float stand_up_pid[3] = {KP_CHASSIS_STAND_UP, KI_CHASSIS_STAND_UP, KD_CHASSIS_STAND_UP};
    PID_init(
        &CHASSIS.pid.stand_up, PID_POSITION, stand_up_pid, MAX_OUT_CHASSIS_STAND_UP,
        MAX_IOUT_CHASSIS_STAND_UP);

    float wheel_stop_pid[3] = {KP_CHASSIS_WHEEL_STOP, KI_CHASSIS_WHEEL_STOP, KD_CHASSIS_WHEEL_STOP};
    PID_init(
        &CHASSIS.pid.wheel_stop[0], PID_POSITION, wheel_stop_pid, MAX_OUT_CHASSIS_WHEEL_STOP,
        MAX_IOUT_CHASSIS_WHEEL_STOP);
    PID_init(
        &CHASSIS.pid.wheel_stop[1], PID_POSITION, wheel_stop_pid, MAX_OUT_CHASSIS_WHEEL_STOP,
        MAX_IOUT_CHASSIS_WHEEL_STOP);

    // 初始化低通滤波器
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[0], LEG_DDL0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[1], LEG_DDL0_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[0], LEG_DDPHI0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[1], LEG_DDPHI0_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[0], LEG_DDTHETA_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[1], LEG_DDTHETA_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[0], LEG_SUPPORT_FORCE_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[1], LEG_SUPPORT_FORCE_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.roll, CHASSIS_ROLL_ALPHA);

    // 初始化机体速度观测器
    // 使用kf同时估计速度和加速度
    Kalman_Filter_Init(&OBSERVER.body.v_kf, 2, 0, 2);
    float F[4] = {1, 0.005, 0, 1};
    float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};
    float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};
    float P[4] = {100000, 0, 0, 100000};
    float H[4] = {1, 0, 0, 1};
    memcpy(OBSERVER.body.v_kf.F_data, F, sizeof(F));
    memcpy(OBSERVER.body.v_kf.P_data, P, sizeof(P));
    memcpy(OBSERVER.body.v_kf.Q_data, Q, sizeof(Q));
    memcpy(OBSERVER.body.v_kf.R_data, R, sizeof(R));
    memcpy(OBSERVER.body.v_kf.H_data, H, sizeof(H));
}

/******************************************************************/
/* Handle exception                                               */
/*----------------------------------------------------------------*/
/* main function:      ChassisHandleException                     */
/* auxiliary function:                                            */
/******************************************************************/

/**
 * @brief          异常处理
 * @param[in]      none
 * @retval         none
 */
void ChassisHandleException(void)
{
    if (GetRcOffline()) {
        CHASSIS.error_code |= DBUS_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~DBUS_ERROR_OFFSET;
    }

    if (CHASSIS.imu == NULL) {
        CHASSIS.error_code |= IMU_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~IMU_ERROR_OFFSET;
    }

    for (uint8_t i = 0; i < 4; i++) {
        if (fabs(CHASSIS.joint_motor[i].fdb.tor) > MAX_TORQUE_PROTECT) {
            CHASSIS.error_code |= JOINT_ERROR_OFFSET;
            break;
        }
    }

    if ((CHASSIS.mode == CHASSIS_OFF || CHASSIS.mode == CHASSIS_SAFE) &&
        fabs(CHASSIS.pid.stand_up.max_out) != 0.0f) {
        PID_clear(&CHASSIS.pid.stand_up);
    }
}

/******************************************************************/
/* Set mode                                                       */
/*----------------------------------------------------------------*/
/* main function:      ChassisSetMode                             */
/* auxiliary function: None                                       */
/******************************************************************/

static void ResetXStateOnModeSwitch(void)
{
    CHASSIS.fdb.body.x = 0.0f;
    CHASSIS.ref.body.x = 0.0f;

    CHASSIS.fdb.body.x_dot_obv = 0.0f;
    CHASSIS.fdb.body.x_acc_obv = 0.0f;

    for (uint8_t i = 0; i < 2; i++) {
        CHASSIS.fdb.leg_state[i].x = 0.0f;
        CHASSIS.fdb.leg_state[i].x_dot = 0.0f;
        CHASSIS.fdb.leg_state[i].last_x = 0.0f;
        CHASSIS.fdb.leg_state[i].last_x_dot = 0.0f;
        CHASSIS.fdb.leg_state[i].Delta_x = 0.0f;
        CHASSIS.fdb.leg_state[i].Delta_x_dot = 0.0f;

        CHASSIS.ref.leg_state[i].x = 0.0f;
        CHASSIS.ref.leg_state[i].x_dot = 0.0f;
    }

    // 把KF内部估计也清掉，避免切模式后立刻又带回旧状态
    OBSERVER.body.v_kf.xhat_data[0] = 0.0f;
    OBSERVER.body.v_kf.xhat_data[1] = 0.0f;

    // 和x相关的PID也顺手清一下，避免积分残留
    PID_clear(&CHASSIS.pid.tail_x);
    PID_clear(&CHASSIS.pid.tail_T);
    PID_clear(&CHASSIS.pid.vel_add);
    PID_clear(&CHASSIS.pid.wheel_stop[0]);
    PID_clear(&CHASSIS.pid.wheel_stop[1]);
}

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
// void ChassisSetMode(void)
// {
//     if (CHASSIS.error_code & DBUS_ERROR_OFFSET) {  // 遥控器出错时的状态处理
//         CHASSIS.mode = CHASSIS_SAFE;
//         return;
//     }

//     if (CHASSIS.error_code & IMU_ERROR_OFFSET) {  // IMU出错时的状态处理
//         CHASSIS.mode = CHASSIS_SAFE;
//         return;
//     }

//     if (CHASSIS.error_code & JOINT_ERROR_OFFSET) {  // 关节电机出错时的状态处理
//         CHASSIS.mode = CHASSIS_SAFE;
//         return;
//     }
// #if TAKE_OFF_DETECT
//     // 离地状态切换
//     for (uint8_t i = 0; i < 2; i++) {
//         if (CHASSIS.fdb.leg[i].is_take_off &&
//             CHASSIS.fdb.leg[i].touch_time > TOUCH_TOGGLE_THRESHOLD) {
//             CHASSIS.fdb.leg[i].is_take_off = false;
//         } else if (
//             !CHASSIS.fdb.leg[i].is_take_off &&
//             CHASSIS.fdb.leg[i].take_off_time > TOUCH_TOGGLE_THRESHOLD) {
//             CHASSIS.fdb.leg[i].is_take_off = true;
//         }
//     }
// #endif

//     if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL]))
//         CHASSIS.mode =
//             CHASSIS_NOTAIL;  //后面可以变成多体，改改lqr输入变量就行，因为这个模式尾巴相当于没有
//     else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
//         if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_FUNCTION]))
//             CHASSIS.mode = CHASSIS_BIPEDAL;
//         else if (
//             switch_is_up(CHASSIS.rc->rc.s[CHASSIS_FUNCTION]) ||
//             switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_FUNCTION])) {
//             CHASSIS.mode = CHASSIS_TRIPOD;
//         }
//     } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL]))
//         CHASSIS.mode = CHASSIS_JOINED;
// }
void ChassisSetMode(void)
{
    ChassisMode_e old_mode = CHASSIS.mode;
    ChassisMode_e new_mode = CHASSIS.mode;

    if (CHASSIS.error_code & DBUS_ERROR_OFFSET) {
        new_mode = CHASSIS_SAFE;
    } else if (CHASSIS.error_code & IMU_ERROR_OFFSET) {
        new_mode = CHASSIS_SAFE;
    } else if (CHASSIS.error_code & JOINT_ERROR_OFFSET) {
        new_mode = CHASSIS_SAFE;
    } else {
#if TAKE_OFF_DETECT
    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.fdb.leg[i].is_take_off &&
            CHASSIS.fdb.leg[i].touch_time > TOUCH_TOGGLE_THRESHOLD) {
            CHASSIS.fdb.leg[i].is_take_off = false;
        } else if (
            !CHASSIS.fdb.leg[i].is_take_off &&
            CHASSIS.fdb.leg[i].take_off_time > TOUCH_TOGGLE_THRESHOLD) {
            CHASSIS.fdb.leg[i].is_take_off = true;
        }
    }
#endif

        if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
            new_mode = CHASSIS_NOTAIL;
        } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_FUNCTION]))
                new_mode = CHASSIS_BIPEDAL;
            else
                new_mode = CHASSIS_TRIPOD;
        } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
            new_mode = CHASSIS_JOINED;
        }
    }

    CHASSIS.mode = new_mode;

    if (new_mode != old_mode) {
        ResetXStateOnModeSwitch();
        last_mode = new_mode;
    }
}

/******************************************************************/
/* Observe                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisObserver                            */
/* auxiliary function: UpdateBodyStatus                           */
/*                     UpdateLegStatus                            */
/*                     UpdateMotorStatus                          */
/*                     BodyMotionObserve                          */
/******************************************************************/

#define ZERO_POS_THRESHOLD 0.001f

static void UpdateBodyStatus(void);
static void UpdateLegStatus(void);
static void UpdateMotorStatus(void);
static void UpdateStepStatus(void);
static void BodyMotionObserve(void);

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{
    CHASSIS.duration = xTaskGetTickCount() - CHASSIS.last_time;
    CHASSIS.last_time = xTaskGetTickCount();

    UpdateMotorStatus();
    UpdateLegStatus();
    UpdateBodyStatus();
    UpdateStepStatus();

    BodyMotionObserve();
}

/**
 * @brief  更新底盘电机数据
 * @param  none
 */
static void UpdateMotorStatus(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        GetMotorMeasure(&CHASSIS.joint_motor[i]);
    }

    for (uint8_t i = 0; i < 2; i++) {
        GetMotorMeasure(&CHASSIS.wheel_motor[i]);
    }

    GetMotorMeasure(&CHASSIS.tail_motor);
}

static void UpdateBodyStatus(void)  //主要是imu和磁力计那些
{
    // 更新陀螺仪反馈数据
    CHASSIS.fdb.body.roll = GetImuAngle(AX_ROLL);
    CHASSIS.fdb.body.roll_dot = GetImuVelocity(AX_ROLL);

    CHASSIS.fdb.body.pitch = GetImuAngle(AX_PITCH);
    CHASSIS.fdb.body.pitch_dot = GetImuVelocity(AX_PITCH);

    CHASSIS.fdb.body.yaw = GetImuAngle(AX_YAW);
    CHASSIS.fdb.body.yaw_dot = GetImuVelocity(AX_YAW);

    LowPassFilterCalc(&CHASSIS.lpf.roll, CHASSIS.fdb.body.roll);

    // 更新加速度反馈数据，记录下来方便使用
    float ax = GetImuAccel(AX_X);
    float ay = GetImuAccel(AX_Y);
    float az = GetImuAccel(AX_Z);
    // 计算几个常用的三角函数值，减少重复计算
    float cos_roll = cosf(CHASSIS.fdb.body.roll);
    float sin_roll = sinf(CHASSIS.fdb.body.roll);
    float cos_pitch = cosf(CHASSIS.fdb.body.pitch);
    float sin_pitch = sinf(CHASSIS.fdb.body.pitch);
    float cos_yaw = cosf(CHASSIS.fdb.body.yaw);
    float sin_yaw = sinf(CHASSIS.fdb.body.yaw);

    // 计算重力加速度在各个轴上的分量相反值
    CHASSIS.fdb.body.gx = GRAVITY * sin_pitch;
    CHASSIS.fdb.body.gy = -GRAVITY * sin_roll * cos_pitch;
    CHASSIS.fdb.body.gz = -GRAVITY * cos_roll * cos_pitch;

    // 消除重力加速度的影响，获取机体坐标系下的加速度
    CHASSIS.fdb.body.x_accel = ax + CHASSIS.fdb.body.gx;
    CHASSIS.fdb.body.y_accel = ay + CHASSIS.fdb.body.gy;
    CHASSIS.fdb.body.z_accel = az + CHASSIS.fdb.body.gz;

    // 计算从机体坐标系到大地坐标系的旋转矩阵
    // clang-format off
    float R[3][3] = {
        {cos_pitch * cos_yaw, sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw, cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw},
        {cos_pitch * sin_yaw, sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw, cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw},
        {-sin_pitch         , sin_roll * cos_pitch                               , cos_roll * cos_pitch                               }
    };
    // clang-format on

    // 更新大地坐标系下的加速度
    CHASSIS.fdb.world.x_accel = R[0][0] * ax + R[0][1] * ay + R[0][2] * az;
    CHASSIS.fdb.world.y_accel = R[1][0] * ax + R[1][1] * ay + R[1][2] * az;
    CHASSIS.fdb.world.z_accel = R[2][0] * ax + R[2][1] * ay + R[2][2] * az - GRAVITY;

    // 更新...
    CHASSIS.fdb.body.phi = CHASSIS.fdb.body.pitch;
    CHASSIS.fdb.body.phi_dot = CHASSIS.fdb.body.pitch_dot;

    CHASSIS.fdb.body.x_acc = CHASSIS.fdb.body.x_accel;
}

/**
 * @brief  更新腿部状态
 * @param  none
 */
static void UpdateLegStatus(void)
{
    uint8_t i = 0;
    // =====更新关节姿态=====
    // 左前
    CHASSIS.fdb.leg[0].joint.Phi1 =
        theta_transform(CHASSIS.joint_motor[0].fdb.pos, J0_ANGLE_OFFSET, J0_DIRECTION, 1);
    if (CHASSIS.fdb.leg[0].joint.Phi1 < 0.0f) CHASSIS.fdb.leg[0].joint.Phi1 += DOUBLE_PI;
    // 左后
    CHASSIS.fdb.leg[0].joint.Phi4 =
        theta_transform(CHASSIS.joint_motor[1].fdb.pos, J1_ANGLE_OFFSET, J1_DIRECTION, 1);
    if (CHASSIS.fdb.leg[0].joint.Phi4 > M_PI) CHASSIS.fdb.leg[0].joint.Phi4 -= DOUBLE_PI;
    // 右前
    CHASSIS.fdb.leg[1].joint.Phi1 =
        theta_transform(CHASSIS.joint_motor[2].fdb.pos, J2_ANGLE_OFFSET, J2_DIRECTION, 1);
    if (CHASSIS.fdb.leg[1].joint.Phi1 < 0.0f) CHASSIS.fdb.leg[1].joint.Phi1 += DOUBLE_PI;
    // 右后
    CHASSIS.fdb.leg[1].joint.Phi4 =
        theta_transform(CHASSIS.joint_motor[3].fdb.pos, J3_ANGLE_OFFSET, J3_DIRECTION, 1);
    if (CHASSIS.fdb.leg[1].joint.Phi4 > M_PI) CHASSIS.fdb.leg[1].joint.Phi4 -= DOUBLE_PI;

    CHASSIS.fdb.leg[0].joint.dPhi1 = CHASSIS.joint_motor[0].fdb.vel * (J0_DIRECTION);
    CHASSIS.fdb.leg[0].joint.dPhi4 = CHASSIS.joint_motor[1].fdb.vel * (J1_DIRECTION);
    CHASSIS.fdb.leg[1].joint.dPhi1 = CHASSIS.joint_motor[2].fdb.vel * (J2_DIRECTION);
    CHASSIS.fdb.leg[1].joint.dPhi4 = CHASSIS.joint_motor[3].fdb.vel * (J3_DIRECTION);

    CHASSIS.fdb.leg[0].joint.T1 = CHASSIS.joint_motor[0].fdb.tor * (J0_DIRECTION);
    CHASSIS.fdb.leg[0].joint.T2 = CHASSIS.joint_motor[1].fdb.tor * (J1_DIRECTION);
    CHASSIS.fdb.leg[1].joint.T1 = CHASSIS.joint_motor[2].fdb.tor * (J2_DIRECTION);
    CHASSIS.fdb.leg[1].joint.T2 = CHASSIS.joint_motor[3].fdb.tor * (J3_DIRECTION);

    // =====更新驱动轮姿态=====
    CHASSIS.fdb.leg[0].wheel.Velocity = CHASSIS.wheel_motor[0].fdb.vel * (W0_DIRECTION);
    CHASSIS.fdb.leg[1].wheel.Velocity = CHASSIS.wheel_motor[1].fdb.vel * (W1_DIRECTION);

    // =====更新尾部姿态=====
    CHASSIS.fdb.tail.Beta =
        theta_transform(CHASSIS.tail_motor.fdb.pos, T_ANGLE_OFFSET, T_DIRECTION, 1);
    CHASSIS.fdb.tail.dBeta = CHASSIS.tail_motor.fdb.vel * (T_DIRECTION);
    CHASSIS.fdb.tail.Tt = CHASSIS.tail_motor.fdb.tor;
    // =====更新摆杆姿态=====
    float L0_Phi0[2];
    float dL0_dPhi0[2];
    for (i = 0; i < 2; i++) {
        float last_dL0 = CHASSIS.fdb.leg[i].rod.dL0;
        float last_dPhi0 = CHASSIS.fdb.leg[i].rod.dPhi0;
        float last_dTheta = CHASSIS.fdb.leg[i].rod.dTheta;

        // 更新位置信息
        // Leg_Forward_Kinematics_Solution(
        //     CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, L0_Phi0,
        //     CHASSIS.fdb.leg[i].joint.dPhi1, CHASSIS.fdb.leg[i].joint.dPhi4, dL0_dPhi0);
        GetL0AndPhi0(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, L0_Phi0);

        CHASSIS.fdb.leg[i].rod.L0 = L0_Phi0[0];
        CHASSIS.fdb.leg[i].rod.Phi0 = L0_Phi0[1];
        CHASSIS.fdb.leg[i].rod.Theta = M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;

        // 计算雅可比矩阵
        CalcJacobian(
            CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, CHASSIS.fdb.leg[i].J);

        // 更新速度信息
        GetdL0AnddPhi0(
            CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.dPhi1, CHASSIS.fdb.leg[i].joint.dPhi4,
            dL0_dPhi0);
        CHASSIS.fdb.leg[i].rod.dL0 = dL0_dPhi0[0];
        CHASSIS.fdb.leg[i].rod.dPhi0 = dL0_dPhi0[1];
        CHASSIS.fdb.leg[i].rod.dTheta = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;

        // 更新加速度信息
        float accel = (CHASSIS.fdb.leg[i].rod.dL0 - last_dL0) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddL0 = accel;

        accel = (CHASSIS.fdb.leg[i].rod.dPhi0 - last_dPhi0) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddPhi0 = accel;

        accel = (CHASSIS.fdb.leg[i].rod.dTheta - last_dTheta) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddTheta = accel;

        // 差分计算腿长变化率和腿角速度
        float ddot_z_M = CHASSIS.fdb.world.z_accel;
        float l0 = CHASSIS.fdb.leg[i].rod.L0;
        float v_l0 = CHASSIS.fdb.leg[i].rod.dL0;
        float theta = CHASSIS.fdb.leg[i].rod.Theta;
        float w_theta = CHASSIS.fdb.leg[i].rod.dTheta;

        float dot_v_l0 = CHASSIS.fdb.leg[i].rod.ddL0;
        float dot_w_theta = CHASSIS.fdb.leg[i].rod.ddTheta;
        // clang-format off
        float ddot_z_w = ddot_z_M 
                    - dot_v_l0 * cosf(theta) 
                    + 2.0f * v_l0 * w_theta * sinf(theta) 
                    + l0 * dot_w_theta * sinf(theta) 
                    + l0 * w_theta * w_theta * cosf(theta);
        // clang-format on

        // 计算支撑力
        float F[2];
        // GetLegForce(
        //     CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4,
        //     CHASSIS.fdb.leg[i].joint.T1, CHASSIS.fdb.leg[i].joint.T2, F);
        GetLegForce(
            CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.T1, CHASSIS.fdb.leg[i].joint.T2, F);
        float F0 = F[0];
        float Tp = F[1];

        float P = F0 * cosf(theta) + Tp * sinf(theta) / l0;
        CHASSIS.fdb.leg[i].Fn = P + WHEEL_MASS * (9.8f + ddot_z_w);
        if (CHASSIS.fdb.leg[i].Fn < TAKE_OFF_FN_THRESHOLD) {
            CHASSIS.fdb.leg[i].touch_time = 0;
            CHASSIS.fdb.leg[i].take_off_time += CHASSIS.duration;
        } else {
            CHASSIS.fdb.leg[i].touch_time += CHASSIS.duration;
            CHASSIS.fdb.leg[i].take_off_time = 0;
        }
    }

    CHASSIS.fdb.tail.ddBeta = (CHASSIS.fdb.tail.dBeta - last_dBeta) / (CHASSIS.duration * MS_TO_S);
    last_dBeta = CHASSIS.fdb.tail.dBeta;
    // CHASSIS.fdb.tail.TT
    //加入尾巴支撑力解算
    // printf("phi1 = %.3f, phi4 = %.3f, L = %.3f, dL = %.3f, dphi1 = %.3f, dphi4 = %.3f, theta = %.3f, dtheta = %.3f\r\n",
    // CHASSIS.fdb.leg[1].joint.Phi1, CHASSIS.fdb.leg[1].joint.Phi4, CHASSIS.fdb.leg[1].rod.L0, CHASSIS.fdb.leg[1].rod.dL0,
    // CHASSIS.fdb.leg[1].joint.dPhi1, CHASSIS.fdb.leg[1].joint.dPhi4, CHASSIS.fdb.leg[1].rod.Theta, CHASSIS.fdb.leg[1].rod.dTheta);
}

#define StateTransfer()    \
    CHASSIS.step_time = 0; \
    CHASSIS.step = TRANSITION_MATRIX[CHASSIS.step];

static void UpdateStepStatus(void)  //感觉只是跳
{
    CHASSIS.step_time += CHASSIS.duration;

    if (CHASSIS.mode == CHASSIS_BIPEDAL) {
        if (0 && (GetDt7RcCh(DT7_CH_RH) < -0.9f)) {  // 遥控器左侧水平摇杆打到左边切换至跳跃状态
            CHASSIS.step_time = 0;
            CHASSIS.step = JUMP_STEP_SQUST;
        } else if (CHASSIS.step == JUMP_STEP_SQUST) {  // 跳跃——蹲下蓄力状态
            if (CHASSIS.fdb.leg[0].rod.L0 < MIN_LEG_LENGTH + 0.02f &&
                CHASSIS.fdb.leg[1].rod.L0 < MIN_LEG_LENGTH + 0.02f) {
                StateTransfer();
            }
        } else if (CHASSIS.step == JUMP_STEP_JUMP) {  // 跳跃——起跳状态
            if (CHASSIS.fdb.leg[0].rod.L0 > MAX_LEG_LENGTH - 0.03f &&
                CHASSIS.fdb.leg[1].rod.L0 > MAX_LEG_LENGTH - 0.03f) {
                StateTransfer();
            }
        } else if (CHASSIS.step == JUMP_STEP_RECOVERY) {  // 跳跃——收腿状态
            if (CHASSIS.step_time > 1000) {               // 1000ms后切换状态
                StateTransfer();
            }
        } else if (CHASSIS.step != NORMAL_STEP && CHASSIS.step_time > MAX_STEP_TIME) {
            // 状态持续时间超过 MAX_STEP_TIME ，自动切换到NORMAL状态
            CHASSIS.step_time = 0;
            CHASSIS.step = NORMAL_STEP;
        }
    } else {
        CHASSIS.step_time = 0;
        CHASSIS.step = NORMAL_STEP;
    }
}
#undef StateTransfer

/**
 * @brief  机体运动状态观测器，主要是得到速度和位移
 * @param  none
 */
static void BodyMotionObserve(void)
{
    // clang-format off
    float speed = WHEEL_RADIUS * (CHASSIS.fdb.leg[0].wheel.Velocity + CHASSIS.fdb.leg[1].wheel.Velocity) / 2;
    // clang-format on

    // 使用kf同时估计加速度和速度,滤波更新
    OBSERVER.body.v_kf.MeasuredVector[0] = speed;                   // 输入轮速
    OBSERVER.body.v_kf.MeasuredVector[1] = CHASSIS.fdb.body.x_acc;  // 输入加速度
    OBSERVER.body.v_kf.F_data[1] = CHASSIS.duration * MS_TO_S;      // 更新采样时间

    Kalman_Filter_Update(&OBSERVER.body.v_kf);
    CHASSIS.fdb.body.x_dot_obv = OBSERVER.body.v_kf.xhat_data[0];
    CHASSIS.fdb.body.x_acc_obv = OBSERVER.body.v_kf.xhat_data[1];

    // 更新行驶距离
    if (fabs(CHASSIS.ref.speed_vector.vx) < WHEEL_DEADZONE &&
        fabs(CHASSIS.fdb.body.x_dot_obv) < 0.8f) {
        // 当目标速度为0，且速度小于阈值时，计算反馈距离
        CHASSIS.fdb.body.x += CHASSIS.fdb.body.x_dot_obv * CHASSIS.duration * MS_TO_S;
    } else {
        //CHASSIS.fdb.body.x = 0;
    }

    // 更新2条腿的状态向量
    uint8_t i = 0;
    for (i = 0; i < 2; i++) {
        // clang-format off
        CHASSIS.fdb.leg_state[i].theta     =  M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;
        CHASSIS.fdb.leg_state[i].theta_dot = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;
        CHASSIS.fdb.leg_state[i].x         =  CHASSIS.fdb.body.x;
        CHASSIS.fdb.leg_state[i].x_dot     =  CHASSIS.fdb.body.x_dot_obv;
        CHASSIS.fdb.leg_state[i].phi       =  CHASSIS.fdb.body.phi;
        CHASSIS.fdb.leg_state[i].phi_dot   =  CHASSIS.fdb.body.phi_dot;
        
        CHASSIS.fdb.leg_state[i].Delta_theta = CHASSIS.fdb.leg_state[i].theta - CHASSIS.fdb.leg_state[i].last_theta;
		CHASSIS.fdb.leg_state[i].Delta_theta_dot = CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.fdb.leg_state[i].last_theta_dot;
        CHASSIS.fdb.leg_state[i].Delta_x = CHASSIS.fdb.leg_state[i].x - CHASSIS.fdb.leg_state[i].last_x;
		CHASSIS.fdb.leg_state[i].Delta_x_dot = CHASSIS.fdb.leg_state[i].x_dot - CHASSIS.fdb.leg_state[i].last_x_dot;
        CHASSIS.fdb.leg_state[i].Delta_phi = CHASSIS.fdb.leg_state[i].phi - CHASSIS.fdb.leg_state[i].last_phi;
		CHASSIS.fdb.leg_state[i].Delta_phi_dot = CHASSIS.fdb.leg_state[i].phi_dot - CHASSIS.fdb.leg_state[i].last_phi_dot;
        
        CHASSIS.fdb.leg_state[i].last_theta = CHASSIS.fdb.leg_state[i].theta;
		CHASSIS.fdb.leg_state[i].last_theta_dot = CHASSIS.fdb.leg_state[i].theta_dot;
		CHASSIS.fdb.leg_state[i].last_x = CHASSIS.fdb.leg_state[i].x;
		CHASSIS.fdb.leg_state[i].last_x_dot = CHASSIS.fdb.leg_state[i].x_dot;
        CHASSIS.fdb.leg_state[i].last_phi = CHASSIS.fdb.leg_state[i].phi;
		CHASSIS.fdb.leg_state[i].last_phi_dot = CHASSIS.fdb.leg_state[i].phi_dot;
        // clang-format on
    }

    CHASSIS.fdb.tail_state.beta = CHASSIS.fdb.tail.Beta;
    CHASSIS.fdb.tail_state.beta_dot = CHASSIS.fdb.tail.dBeta;
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:      ChassisReference                           */
/* auxiliary function: None                                       */
/******************************************************************/

/**
 * @brief          基于竖直几何关系算尾巴角度期望，仅尾落地时
 * @param[in]      none
 * @retval         none
 */
float Get_beta(float l_l, float theta_l, float l_r, float theta_r, float pitch)
{
    // float s = (length_ref * cosf(theta_ref) + WHEEL_RADIUS - TAIL_WHEEL_RADIUS -
    //            TAIL_POS_OFFSET * cosf(pitch_ref)) /
    //           TAIL_LENGTH;
    float s1 = WHEEL_RADIUS + l_l * cosf(theta_l) * 0.5f + l_r * cosf(theta_r) * 0.5f -
               TAIL_POS_OFFSET_VERTICAL * cosf(pitch) + TAIL_POS_OFFSET_HORIZON * sinf(pitch) -
               TAIL_WHEEL_RADIUS;
    float s2 = s1 / TAIL_LENGTH;
    float s = fp32_constrain(s2, -1.0f, 1.0f);
    return asinf(s) + pitch - TAIL_BETA_OMNI_to_HAND;
}

/**
 * @brief          基于几何关系算尾巴角度期望，仅尾落地时
 * @param[in]      none
 * @retval         none
 */
float Get_beta_ref(float length_ref, float theta_ref, float pitch_ref)
{
    float s = (length_ref * cosf(theta_ref) + WHEEL_RADIUS - TAIL_WHEEL_RADIUS -
               TAIL_POS_OFFSET * cosf(pitch_ref)) /
              TAIL_LENGTH;
    s = fp32_constrain(s, -1.0f, 1.0f);
    return asinf(s) + pitch_ref - TAIL_BETA_OMNI_to_HAND;
}

float Get_tail_x_fdb(float l_l, float theta_l, float l_r, float theta_r, float pitch, float beta)
{
    float s = l_l * sinf(theta_l) * 0.5f + l_r * sinf(theta_r) * 0.5f +
              TAIL_POS_OFFSET_HORIZON * cosf(pitch) + TAIL_POS_OFFSET_VERTICAL * sinf(pitch) +
              TAIL_LENGTH * cosf(beta - pitch + TAIL_BETA_OMNI_to_HAND);
    return s;
}

float Get_tail_x_ref(float l_l, float theta_l, float l_r, float theta_r, float pitch)
{
    float beta = Get_beta(l_l, theta_l, l_r, theta_r, pitch);
    float s = l_l * sinf(theta_l) * 0.5f + l_r * sinf(theta_r) * 0.5f +
              TAIL_POS_OFFSET_HORIZON * cosf(pitch) + TAIL_POS_OFFSET_VERTICAL * sinf(pitch) +
              TAIL_LENGTH * cosf(beta - pitch + TAIL_BETA_OMNI_to_HAND);
    return s;
}

float Get_tail_wheel_speed(
    float l_l, float theta_l, float l_r, float theta_r, float dl_l, float dtheta_l, float dl_r,
    float dtheta_r, float pitch, float beta, float dpitch, float dbeta)
{
    float s = l_l * sinf(theta_l) * 0.5f + l_r * sinf(theta_r) * 0.5f +
              TAIL_POS_OFFSET_HORIZON * cosf(pitch) + TAIL_POS_OFFSET_VERTICAL * sinf(pitch) +
              TAIL_LENGTH * cosf(beta - pitch + TAIL_BETA_OMNI_to_HAND);
    float s1 = l_l * cosf(theta_l) * dtheta_l + dl_l + sinf(theta_l);
    float s2 = l_r * cosf(theta_r) * dtheta_r + dl_r + sinf(theta_r);
    float s3 = -TAIL_POS_OFFSET_HORIZON * dpitch * sinf(pitch);
    float s4 = TAIL_POS_OFFSET_VERTICAL * dpitch * cosf(pitch);
    float s5 = -TAIL_LENGTH * sinf(beta - pitch + TAIL_BETA_OMNI_to_HAND) * (dbeta - dpitch);
    return s = s1 + s2 + s3 + s4 + s5;
}

float Get_theta_ref_tripod(float l)
{
    float l2 = l * l;
    float l3 = l2 * l;
    float l4 = l3 * l;
    float l5 = l4 * l;
    return -4303.41570f * l5 + 5185.08042f * l4 - 2540.45096f * l3 + 636.648114f * l2 -
           83.4929610f * l + 4.85644774f;
}

static float WrapToPi(float a)
{
    while (a >= M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static float LegCOMDist(float l);
static float LegCOMOffset(float l);

static float LegCOMDist(float l)
{
    float l2 = l * l;
    float l3 = l2 * l;
    float l4 = l3 * l;
    return 1.6572f * l4 - 1.8119f * l3 + 0.754f * l2 + 0.4594f * l + 0.0099f;
}

// 你自己替换，单位 rad
static float LegCOMOffset(float l)
{
    float l2 = l * l;
    float l3 = l2 * l;
    float l4 = l3 * l;
    return 277.69f * l4 - 304.33f * l3 + 130.52f * l2 - 27.562f * l + 2.7591f;
}

// static float ThetaResidual(float l, float phi, float theta)
// {
//     float lc = LegCOMDist(l);
//     float d = LegCOMOffset(l);
//     float beta = Get_beta(l, theta, l, theta, phi);

//     float lhs = 2.0f * WHEEL_MASS * l * sinf(theta) + 2.0f * LEG_MASS * lc * sinf(theta + d);

//     float rhs = TAIL_MASS * (TAIL_COM_to_MOTOR * cosf(beta - phi + TAIL_BETA_COM_to_HAND) +
//                              TAIL_POS_OFFSET_VERTICAL * sinf(phi));

//     return lhs - rhs;
// }

static float ThetaResidual_Beta5Deg(float l, float phi, float theta)
{
    float lc = LegCOMDist(l);
    float d = LegCOMOffset(l);
    const float beta = TAIL_BETA_INIT * PI / 180.0f;

    float lhs = 2.0f * WHEEL_MASS * l * sinf(theta) + 2.0f * LEG_MASS * lc * sinf(theta + d);

    float rhs = TAIL_MASS * (TAIL_COM_to_MOTOR * cosf(beta - phi + TAIL_BETA_COM_to_HAND) +
                             TAIL_POS_OFFSET_VERTICAL * sinf(phi));

    return lhs - rhs;
}

/**
 * @brief  固定 beta = 5°，求 theta 和 phi
 * @param  l         腿长
 * @param  phi_guess phi初值/上一时刻phi
 * @param  theta_ref 输出 theta
 * @param  phi_ref   输出 phi
 */
void SolveThetaBetaSimple(float l, float phi_guess, float * theta_ref, float * phi_ref)
{
    float best_theta = 0.0f;
    float best_phi = phi_guess;
    float best_res = 1e9f;

    const float beta = TAIL_BETA_INIT * PI / 180.0f;

    const float phi_min = phi_guess - 0.5f;
    const float phi_max = phi_guess + 0.5f;
    const int N = 80;

    for (int i = 0; i <= N; i++) {
        float phi = phi_min + (phi_max - phi_min) * (float)i / (float)N;

        // 固定 beta = 5° 的几何关系
        float c = (TAIL_WHEEL_RADIUS + TAIL_LENGTH * sinf(beta - phi + TAIL_BETA_OMNI_to_HAND) +
                   TAIL_POS_OFFSET_VERTICAL * cosf(phi) - WHEEL_RADIUS) /
                  l;

        if (c < -1.0f || c > 1.0f) {
            continue;
        }

        c = fp32_constrain(c, -1.0f, 1.0f);

        float theta1 = acosf(c);
        float theta2 = -acosf(c);

        float r1 = fabsf(ThetaResidual_Beta5Deg(l, phi, theta1));
        if (r1 < best_res) {
            best_res = r1;
            best_theta = theta1;
            best_phi = phi;
        }

        float r2 = fabsf(ThetaResidual_Beta5Deg(l, phi, theta2));
        if (r2 < best_res) {
            best_res = r2;
            best_theta = theta2;
            best_phi = phi;
        }
    }

    *theta_ref = best_theta;
    *phi_ref = best_phi;
}

static float PitchResidual_Beta0(float l_l, float theta_l, float l_r, float theta_r, float pitch)
{
    return Get_beta(l_l, theta_l, l_r, theta_r, pitch);  // 目标 beta = 0
}

/**
 * @brief  根据每个周期的腿长反馈和腿摆角反馈，令 beta = 0，反解 pitch
 * @param  l_l         左腿长反馈
 * @param  theta_l     左腿摆角反馈
 * @param  l_r         右腿长反馈
 * @param  theta_r     右腿摆角反馈
 * @param  pitch_guess pitch初值，建议传上一周期的 pitch_ref 或当前机身pitch
 * @retval 求得的 pitch
 */
float SolvePitchFromBeta0(float l_l, float theta_l, float l_r, float theta_r, float pitch_guess)
{
    float pitch = pitch_guess;

    // 牛顿迭代几次就够了
    for (int i = 0; i < 6; i++) {
        float f = PitchResidual_Beta0(l_l, theta_l, l_r, theta_r, pitch);

        // 已经很接近了就直接返回
        if (fabsf(f) < 1e-4f) {
            break;
        }

        // 数值求导
        const float h = 0.002f;
        float fp = PitchResidual_Beta0(l_l, theta_l, l_r, theta_r, pitch + h);
        float fm = PitchResidual_Beta0(l_l, theta_l, l_r, theta_r, pitch - h);
        float df = (fp - fm) / (2.0f * h);

        // 防止除零
        if (fabsf(df) < 1e-5f) {
            break;
        }

        pitch -= f / df;

        // 限幅，避免跑飞
        pitch = fp32_constrain(pitch, MIN_PITCH, MAX_PITCH);
    }

    return pitch;
}

static float PitchResidual_Beta5Deg(float l, float theta, float pitch)
{
    const float beta_ref = TAIL_BETA_INIT * PI / 180.0f;

    float beta_now = Get_beta(l, theta, l, theta, pitch);
    return WrapToPi(beta_now - beta_ref);
}

/**
 * @brief  用腿长反馈、腿夹角反馈，固定 beta=5°，反解 pitch
 * @param  l          当前反馈腿长（建议传 length_fdb）
 * @param  theta      当前反馈腿夹角（建议传 angle_fdb）
 * @param  pitch_guess pitch初值/上一时刻pitch
 * @param  pitch_ref  输出 pitch
 */
void SolvePitchBeta5Simple(float l, float theta, float pitch_guess, float * pitch_ref)
{
    float best_pitch = pitch_guess;
    float best_res = 1e9f;

    const float pitch_min = pitch_guess - 0.4f;
    const float pitch_max = pitch_guess + 0.4f;
    const int N = 60;

    for (int i = 0; i <= N; i++) {
        float pitch = pitch_min + (pitch_max - pitch_min) * (float)i / (float)N;
        float r = fabsf(PitchResidual_Beta5Deg(l, theta, pitch));

        if (r < best_res) {
            best_res = r;
            best_pitch = pitch;
        }
    }

    *pitch_ref = best_pitch;
}

float ThetaResidual_BodyCenter(float l, float phi, float theta)
{
    const float beta = TAIL_BETA_INIT * PI / 180.0f;

    // 体心重合原则：
    // l*sin(theta) + b*sin(phi) + a*cos(phi) + L*cos(beta - phi + gamma) = 3*l*sin(theta)
    // =>
    // b*sin(phi) + a*cos(phi) + L*cos(beta - phi + gamma) = 2*l*sin(theta)

    float lhs = TAIL_POS_OFFSET_VERTICAL * sinf(phi) +
                TAIL_POS_OFFSET_HORIZON  * cosf(phi) +
                TAIL_LENGTH * cosf(beta - phi + TAIL_BETA_OMNI_to_HAND);

    float rhs = 2.0f * l * sinf(theta);

    return lhs - rhs;
    }

/**
 * @brief  固定 beta = TAIL_BETA_INIT，基于“触地约束 + 体心重合”求 theta 和 phi
 * @param  l         腿长
 * @param  phi_guess phi初值/上一时刻phi
 * @param  theta_ref 输出 theta
 * @param  phi_ref   输出 phi
 */
void SolveThetaBetaBodyCenter(float l, float phi_guess, float *theta_ref, float *phi_ref)
{
    float best_theta = 0.0f;
    float best_phi   = phi_guess;
    float best_res   = 1e9f;

    const float beta = TAIL_BETA_INIT * PI / 180.0f;

    // 直接限制 pitch 为负
    const float phi_min = -1.0f;
    const float phi_max =  0.0f;
    const int N = 160;

    for (int i = 0; i <= N; i++) {
        float phi = phi_min + (phi_max - phi_min) * (float)i / (float)N;

        // 触地约束（带水平偏置）
        float c = (TAIL_POS_OFFSET_VERTICAL * cosf(phi)
                 - TAIL_POS_OFFSET_HORIZON  * sinf(phi)
                 + TAIL_LENGTH * sinf(beta - phi + TAIL_BETA_OMNI_to_HAND)
                 + TAIL_WHEEL_RADIUS - WHEEL_RADIUS) / l;

        if (c < -1.0f || c > 1.0f) continue;

        // 只取 theta 正支，这样自动保证 theta < pi/2
        float theta = acosf(c);

        float lhs = TAIL_POS_OFFSET_VERTICAL * sinf(phi)
                  + TAIL_POS_OFFSET_HORIZON  * cosf(phi)
                  + TAIL_LENGTH * cosf(beta - phi + TAIL_BETA_OMNI_to_HAND);

        float rhs = 2.0f * l * sinf(theta);

        float r = fabsf(lhs - rhs);

        if (r < best_res) {
            best_res   = r;
            best_theta = theta;
            best_phi   = phi;
        }
    }

    *theta_ref = best_theta;
    *phi_ref   = best_phi;
}

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
    int16_t rc_x = 0, rc_wz = 0;
    int16_t rc_length = 0, rc_angle = 0, rc_tail = 0;
    int16_t rc_roll = 0, rc_pitch = 0;

    // 0-右平, 1-右竖, 2-左平, 3-左竖, 4-左滚轮
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL], rc_x, CHASSIS_RC_DEADLINE);    //3
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz, CHASSIS_RC_DEADLINE);  //2

    if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_FUNCTION])) {
        rc_deadband_limit(
            CHASSIS.rc->rc.ch[CHASSIS_PITCH_CHANNEL], rc_pitch, CHASSIS_RC_DEADLINE);  //1
        rc_deadband_limit(
            CHASSIS.rc->rc.ch[CHASSIS_ROLL_CHANNEL], rc_roll, CHASSIS_RC_DEADLINE);  //0
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_FUNCTION])) {
        rc_deadband_limit(
            CHASSIS.rc->rc.ch[CHASSIS_ANGLE_CHANNEL], rc_angle, CHASSIS_RC_DEADLINE);  //1
        rc_deadband_limit(
            CHASSIS.rc->rc.ch[CHASSIS_LENGTH_CHANNEL], rc_length, CHASSIS_RC_DEADLINE);  //0
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_FUNCTION])) {
        rc_deadband_limit(
            CHASSIS.rc->rc.ch[CHASSIS_TAIL_POS_CHANNEL], rc_tail, CHASSIS_RC_DEADLINE);  //1
        rc_deadband_limit(
            CHASSIS.rc->rc.ch[CHASSIS_LENGTH_CHANNEL], rc_length, CHASSIS_RC_DEADLINE);  //0
    }

    // 计算速度向量

    ChassisSpeedVector_t v_set = {0.0f, 0.0f, 0.0f};
    v_set.vx = rc_x * RC_TO_ONE * MAX_SPEED_VECTOR_VX;
    v_set.vy = 0;
    v_set.wz = -rc_wz * RC_TO_ONE * MAX_SPEED_VECTOR_WZ;
    // CHASSIS.ref.body.yaw = -rc_wz * RC_TO_ONE * MAX_SPEED_VECTOR_WZ;
    switch (CHASSIS.mode) {
        case CHASSIS_NOTAIL:
        case CHASSIS_TRIPOD:
        case CHASSIS_BIPEDAL:
        case CHASSIS_JOINED: {
            // CHASSIS.ref.speed_vector.vx = v_set.vx;

            // 原始速度指令
            float vx_cmd = v_set.vx;

            // 一阶低通后的速度期望
            CHASSIS.ref.speed_vector.vx += VX_REF_ALPHA * (vx_cmd - CHASSIS.ref.speed_vector.vx);
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = v_set.wz;
            CHASSIS.ref.body.yaw += CHASSIS.ref.speed_vector.wz * CHASSIS_CONTROL_TIME_S;
            CHASSIS.ref.body.yaw = WrapToPi(CHASSIS.ref.body.yaw);
            CHASSIS.ref.body.x += CHASSIS.ref.speed_vector.vx * CHASSIS_CONTROL_TIME_S;
        } break;

        case CHASSIS_STAND_UP:
        case CHASSIS_SAFE:
        case CHASSIS_OFF:
        default: {
            CHASSIS.ref.speed_vector.vx = 0;
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = 0;
            CHASSIS.ref.body.yaw = 0;
        } break;
    }

    CHASSIS.ref.body.roll = fp32_constrain(-rc_roll * RC_TO_ONE * MAX_ROLL, MIN_ROLL, MAX_ROLL);
    CHASSIS.ref.body.pitch =
        fp32_constrain(-rc_pitch * RC_TO_ONE * MAX_PITCH, MIN_PITCH, MAX_PITCH);

    // 腿部控制
    static float angle = 0;
    static float tail_angle = 0;  //水平朝前
    static float length = 0.16f;

    float angle_fdb = (CHASSIS.fdb.leg_state[0].theta + CHASSIS.fdb.leg_state[1].theta) / 2.0f;
    float length_fdb = (CHASSIS.fdb.leg[0].rod.L0 + CHASSIS.fdb.leg[1].rod.L0) / 2.0f;

    switch (CHASSIS.mode) {
        case CHASSIS_STAND_UP: {
            length = 0.148f;
            angle = 0;
            tail_angle = 0;
        } break;
        case CHASSIS_NOTAIL: {
            length = 0.17f + rc_length * RC_TO_ONE * 0.03f;
            angle = 0.0f;
            tail_angle = 0.0f;
            if (CHASSIS.step == JUMP_STEP_SQUST) {
                length = MIN_LEG_LENGTH;
            } else if (CHASSIS.step == JUMP_STEP_JUMP) {
                length = MAX_LEG_LENGTH;
            } else if (CHASSIS.step == JUMP_STEP_RECOVERY) {
                length = MIN_LEG_LENGTH + 0.05f;
            }
        } break;
        case CHASSIS_TRIPOD: {  //尾巴落地 扭矩控制
            length = 0.185f + rc_length * RC_TO_ONE * 0.03f;
            // angle = rc_angle * RC_TO_ONE * 0.3f;
            // tail_angle = Get_beta(length, angle, CHASSIS.ref.body.pitch);
            // tail_angle = Get_beta(
            //     CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg_state[0].theta,
            //     CHASSIS.fdb.leg[1].rod.L0, CHASSIS.fdb.leg_state[1].theta, CHASSIS.fdb.body.pitch);
            // tail_angle = Get_beta_ref(length, angle, CHASSIS.ref.body.pitch);
            // SolveThetaBetaSimple(length, CHASSIS.ref.body.pitch, &angle, &tail_angle);
            // SolveThetaBetaSimple(length, 0.0f, &angle, &CHASSIS.ref.body.pitch);
            // CHASSIS.ref.body.pitch = SolvePitchFromBeta0(
            //     CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg_state[0].theta,
            //     CHASSIS.fdb.leg[1].rod.L0, CHASSIS.fdb.leg_state[1].theta, CHASSIS.ref.body.pitch);
            tail_angle = TAIL_BETA_INIT * PI / 180.0f;

            // 用当前反馈腿长、腿角，反解 pitch
            // SolvePitchBeta5Simple(
            //     length, angle, CHASSIS.ref.body.pitch, &CHASSIS.ref.body.pitch);
            // SolveThetaBetaSimple(length, CHASSIS.ref.body.pitch, &angle, &CHASSIS.ref.body.pitch);
            SolveThetaBetaBodyCenter(length,  CHASSIS.ref.body.pitch, &angle, &CHASSIS.ref.body.pitch);
            // if (last_mode == CHASSIS_BIPEDAL) {
            //     CHASSIS.fdb.body.x = 0.0f;
            //     last_mode = CHASSIS_TRIPOD;
            // }

            angle += rc_angle * RC_TO_ONE * 0.3f;
        } break;
        case CHASSIS_BIPEDAL: {  //尾巴离地 位置控制
            length = 0.17f + rc_length * RC_TO_ONE * 0.03f;
            angle = rc_angle * RC_TO_ONE * 0.3f;  // 待改正，引入重心调节
            // angle = 0.0f;  // 加入质心调节，尾巴上摆，腿往后撤维持重心在同一竖直位置
            // tail_angle = Get_beta_ref(length_fdb, angle_fdb, CHASSIS.fdb.body.pitch) -
            //              rc_tail * RC_TO_ONE * 0.4f;
            tail_angle = Get_beta(
                             CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg_state[0].theta,
                             CHASSIS.fdb.leg[1].rod.L0, CHASSIS.fdb.leg_state[1].theta,
                             CHASSIS.fdb.body.pitch) -
                         rc_tail * RC_TO_ONE * 0.4f;
            // if (last_mode == CHASSIS_TRIPOD) {
            //     CHASSIS.fdb.body.x = 0.0f;
            //     last_mode = CHASSIS_BIPEDAL;
            // }
        } break;
        default: {
            angle = 0;
            length = 0.16f;
            tail_angle = 0;
        } break;
    }
    length = fp32_constrain(length, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    angle = fp32_constrain(angle, MIN_LEG_ANGLE, MAX_LEG_ANGLE);
    tail_angle = fp32_constrain(tail_angle, MIN_TAIL_ANGLE, M_PI_2);

    CHASSIS.ref.rod_L0[0] = length;
    CHASSIS.ref.rod_L0[1] = length;
    CHASSIS.ref.rod_Angle[0] = angle;
    CHASSIS.ref.rod_Angle[1] = angle;

    // 计算期望状态
    // clang-format off
    for (uint8_t i = 0; i < 2; i++) {
        CHASSIS.ref.leg_state[i].theta     =  CHASSIS.ref.rod_Angle[i];
        CHASSIS.ref.leg_state[i].theta_dot =  0;
        CHASSIS.ref.leg_state[i].x        +=  CHASSIS.ref.speed_vector.vx * CHASSIS_CONTROL_TIME_S;
        CHASSIS.ref.leg_state[i].x_dot     =  CHASSIS.ref.speed_vector.vx;
        CHASSIS.ref.leg_state[i].phi       =  CHASSIS.ref.body.pitch;
        CHASSIS.ref.leg_state[i].phi_dot   =  0;
    }
    CHASSIS.ref.tail_state.beta        =  tail_angle;
    CHASSIS.ref.tail_state.beta_dot    =  0;
    // clang-format on

    // 夹爪目标量：CH7 二档控制，低档闭合，高档张开。
    static uint8_t clamp_target_position = 0x00;
    static bool clamp_target_valid = false;
    const Sbus_t *sbus = get_sbus_point();
    const uint16_t clamp_ch7_raw = (sbus == NULL) ? 0U : sbus->ch[6];
    const uint8_t clamp_target = (clamp_ch7_raw >= 1500U) ? 0xFFU : 0x00U;

    if (sbus != NULL) {
        if ((!clamp_target_valid) || (clamp_target_position != clamp_target)) {
            clamp_target_valid = true;
            clamp_target_position = clamp_target;
            ClampSetTarget(clamp_target_position, 0xFF, 0xFF);
        }
    }
}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisConsole                             */
/* auxiliary function: LocomotionController_Bipedal               */
/*                     LegTorqueController                        */
/*                     LegFeedForward                             */
/*                     CalcLQR_Bipedal                            */
/*                     ConsoleZeroForce                           */
/*                     ConsoleBipedal                             */
/*                     ConsoleDebug                               */
/*                     ConsoleTripod                              */
/*                     ConsoleStandUp                             */
/******************************************************************/

static void LocomotionController_NoTail(void);
static void LocomotionController_Pro_NoTail(void);
static void LocomotionController_Bipedal(void);
static void LocomotionController_Pro_Bipedal(void);
static void LocomotionController_Tripod(void);
static void LocomotionController_Pro_Tripod(void);
static void LocomotionController_ProX_Tripod(void);
// static void LegPositionController(void);
static void LegTorqueController(void);
static float LegFeedForward(float theta);
static float LegFeedForward_Tripod(float theta);
static void CalcLQR_NoTail(float k[2][6], float x[6], float t[2]);
static void CalcLQR_Tail(float k[3][8], float x[8], float t[3]);
static void CalcLQR_Pro_NoTail(float k[4][10], float x[10], float Tp_T[4]);
static void CalcLQR_Pro_Tail(float k[5][12], float x[12], float Tp_T_Tt[5]);
static void CalcMPC(float k[2][6], float x[6], float * Delta_Tp);
static void CalcLQR_Pro_Tripod(float k[4][9], float x[9], float Tp_T_Tt[4]);

static void ConsoleZeroForce(void);
static void ConsoleNoTail(void);
static void ConsoleBipedal(void);
static void ConsoleTripod(void);
static void ConsoleStandUp(void);

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_NOTAIL: {
            ConsoleNoTail();
        } break;
        case CHASSIS_BIPEDAL: {
            ConsoleBipedal();
        } break;
        case CHASSIS_TRIPOD: {
            ConsoleTripod();
        } break;
        case CHASSIS_STAND_UP: {
            ConsoleStandUp();
        } break;
        case CHASSIS_OFF:
        case CHASSIS_SAFE:
        default: {
            ConsoleZeroForce();
        } break;
    }

#if CLOSE_LEG_LEFT
    memset(&CHASSIS.joint_motor[0].set, 0, sizeof(CHASSIS.joint_motor[0].set));
    memset(&CHASSIS.joint_motor[1].set, 0, sizeof(CHASSIS.joint_motor[1].set));
    memset(&CHASSIS.wheel_motor[0].set, 0, sizeof(CHASSIS.wheel_motor[0].set));
#endif

#if CLOSE_LEG_RIGHT
    memset(&CHASSIS.joint_motor[2].set, 0, sizeof(CHASSIS.joint_motor[2].set));
    memset(&CHASSIS.joint_motor[3].set, 0, sizeof(CHASSIS.joint_motor[3].set));
    memset(&CHASSIS.wheel_motor[1].set, 0, sizeof(CHASSIS.wheel_motor[1].set));
#endif

#if LIFTED_UP
    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
    CHASSIS.wheel_motor[0].set.value = 0;
    CHASSIS.wheel_motor[1].set.value = 0;
#endif
}

/**
 * @brief      运动控制器 纯轮足模式Pro版
 */
static void LocomotionController_Pro_NoTail(void)
{
    // 计算LQR增益=============================================
    float k[4][10];
    float MPC_k[2][6];
    float x[10];
    float MPC_x[6];
    float Tp_T[4];
    float theta_eq[3];
    float Delta_Tp;
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif
    // if (CHASSIS.step == JUMP_STEP_RECOVERY) {
    //     is_take_off = true;
    // }

    // LQR 计算
    GetK_Pro_NoTail(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, k, MPC_k, is_take_off);
    GetTheta_Pro_NoTail(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, theta_eq);

    x[0] = X0_OFFSET + (CHASSIS.fdb.body.x - CHASSIS.ref.body.x);
    x[1] = X1_OFFSET + (CHASSIS.fdb.body.x_dot_obv - CHASSIS.ref.speed_vector.vx);
    x[2] = X2_OFFSET + WrapToPi(CHASSIS.fdb.body.yaw - CHASSIS.ref.body.yaw);
    x[3] = X3_OFFSET + (CHASSIS.fdb.body.yaw_dot - CHASSIS.ref.speed_vector.wz);
    x[4] =
        X4_OFFSET + (CHASSIS.fdb.leg_state[0].theta - CHASSIS.ref.leg_state[0].theta - theta_eq[0]);
    x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[0].theta_dot - 0.0f);
    x[6] =
        X6_OFFSET + (CHASSIS.fdb.leg_state[1].theta - CHASSIS.ref.leg_state[1].theta - theta_eq[1]);
    x[7] = X7_OFFSET + (CHASSIS.fdb.leg_state[1].theta_dot - 0.0f);
    x[8] = X8_OFFSET + (CHASSIS.fdb.body.phi - CHASSIS.ref.body.pitch);
    x[9] = X9_OFFSET + (CHASSIS.fdb.body.phi_dot - 0.0f);
    CalcLQR_Pro_NoTail(k, x, Tp_T);

    // MPC 计算
    MPC_x[0] = CHASSIS.fdb.leg_state[0].Delta_theta;
    MPC_x[1] = CHASSIS.fdb.leg_state[0].Delta_theta_dot;
    MPC_x[2] = CHASSIS.fdb.leg_state[0].Delta_x;
    MPC_x[3] = CHASSIS.fdb.leg_state[0].Delta_x_dot;
    MPC_x[4] = CHASSIS.fdb.leg_state[0].Delta_phi;
    MPC_x[5] = CHASSIS.fdb.leg_state[0].Delta_phi_dot;
    GetK_MPC(CHASSIS.fdb.leg[0].rod.L0, MPC_k);
    CalcMPC(MPC_k, MPC_x, &Delta_Tp);
    CHASSIS.cmd.leg[0].rod.Tp = Tp_T[1] + Delta_Tp;

    MPC_x[0] = CHASSIS.fdb.leg_state[1].Delta_theta;
    MPC_x[1] = CHASSIS.fdb.leg_state[1].Delta_theta_dot;
    MPC_x[2] = CHASSIS.fdb.leg_state[1].Delta_x;
    MPC_x[3] = CHASSIS.fdb.leg_state[1].Delta_x_dot;
    MPC_x[4] = CHASSIS.fdb.leg_state[1].Delta_phi;
    MPC_x[5] = CHASSIS.fdb.leg_state[1].Delta_phi_dot;
    GetK_MPC(CHASSIS.fdb.leg[1].rod.L0, MPC_k);
    CalcMPC(MPC_k, MPC_x, &Delta_Tp);
    CHASSIS.cmd.leg[1].rod.Tp = Tp_T[0] + Delta_Tp;

    CHASSIS.cmd.leg[0].wheel.T = Tp_T[3];
    CHASSIS.cmd.leg[1].wheel.T = Tp_T[2];

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = -CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // PID补偿稳态误差
    float delta_L0 = 0.0f;

    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);
}

/**
 * @brief      运动控制器 纯轮足模式
 */
static void LocomotionController_NoTail(void)
{
    // 计算LQR增益=============================================
    float k[2][6];
    float MPC_k[2][6];
    float x[6];
    float T_Tp[2];
    float Delta_Tp;
    float theta_eq[3];
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif
    // if (CHASSIS.step == JUMP_STEP_RECOVERY) {
    //     is_take_off = true;
    // }
    GetTheta_Pro_NoTail(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, theta_eq);
    for (uint8_t i = 0; i < 2; i++) {
        GetK_NoTail(CHASSIS.fdb.leg[i].rod.L0, k, MPC_k, is_take_off);

        // clang-format off
        x[0] = X0_OFFSET + (CHASSIS.fdb.leg_state[i].theta     - CHASSIS.ref.leg_state[i].theta - theta_eq[i]);
        x[1] = X1_OFFSET + (CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.ref.leg_state[i].theta_dot);
        x[2] = X2_OFFSET + (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x);
        x[3] = X3_OFFSET + (CHASSIS.fdb.leg_state[i].x_dot     - CHASSIS.ref.leg_state[i].x_dot);
        x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[i].phi       - CHASSIS.ref.leg_state[i].phi);
        x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[i].phi_dot   - CHASSIS.ref.leg_state[i].phi_dot);
        // clang-format on
        CalcLQR_NoTail(k, x, T_Tp);

        x[0] = CHASSIS.fdb.leg_state[i].Delta_theta;
        x[1] = CHASSIS.fdb.leg_state[i].Delta_theta_dot;
        x[2] = CHASSIS.fdb.leg_state[i].Delta_x;
        x[3] = CHASSIS.fdb.leg_state[i].Delta_x_dot;
        x[4] = CHASSIS.fdb.leg_state[i].Delta_phi;
        x[5] = CHASSIS.fdb.leg_state[i].Delta_phi_dot;
        CalcMPC(MPC_k, x, &Delta_Tp);
        // PID_calc(&CHASSIS.pid.theta_comp, CHASSIS.fdb.leg_state[i].theta, 0.0f);
        CHASSIS.cmd.leg[i].wheel.T = T_Tp[0];
        CHASSIS.cmd.leg[i].rod.Tp = T_Tp[1] + Delta_Tp;  // + CHASSIS.pid.theta_comp.out;
    }

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = -CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // PID补偿稳态误差
    float delta_L0 = 0.0f;

    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);

    // 两腿协调一致
    PID_calc(
        &CHASSIS.pid.leg_coordation,
        CHASSIS.fdb.leg_state[0].theta - CHASSIS.fdb.leg_state[1].theta, 0.0f);
    CHASSIS.cmd.leg[0].rod.Tp += CHASSIS.pid.leg_coordation.out;
    CHASSIS.cmd.leg[1].rod.Tp -= CHASSIS.pid.leg_coordation.out;

    // 转向控制================================================
    if (!is_take_off) {
        // float Tau = KP_CHASSIS_YAW_ANGLE * (CHASSIS.fdb.body.yaw - CHASSIS.ref.body.yaw) +
        //             KD_CHASSIS_YAW_ANGLE * CHASSIS.fdb.body.yaw_dot;
        // if (Tau > 7.0f) Tau = 7.0f;
        // if (Tau < -7.0f) Tau = -7.0f;
        // CHASSIS.cmd.leg[0].wheel.T += Tau / 2.0f;
        // CHASSIS.cmd.leg[1].wheel.T -= Tau / 2.0f;

        PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
        // PID_calc(&CHASSIS.pid.yaw_angle, CHASSIS.fdb.body.yaw, CHASSIS.ref.body.yaw);
        CHASSIS.cmd.leg[0].wheel.T -= CHASSIS.pid.yaw_velocity.out;
        CHASSIS.cmd.leg[1].wheel.T += CHASSIS.pid.yaw_velocity.out;
    }
}

float Get_Tp0_Tripod(float l)
{
    float l2 = l * l;
    float l3 = l2 * l;
    float l4 = l3 * l;
    float l5 = l4 * l;
    return 19320.9947f * l5 - 21001.0592f * l4 + 9361.21004f * l3 - 2135.16412f * l2 +
           257.663909f * l - 15.4361906f;
}

float Get_Tt0_Tripod(float l)
{
    float l2 = l * l;
    float l3 = l2 * l;
    float l4 = l3 * l;
    float l5 = l4 * l;
    return -38610.1537f * l5 + 41964.6919f * l4 - 18705.2788f * l3 + 4266.51892f * l2 -
           514.918720f * l + 30.8554846f;
}

/**
 * @brief      运动控制器 尾巴离地模式Pro版——VMC
 */
static void LocomotionController_ProX_Tripod(void)
{
    // 计算LQR增益=============================================
    float k[4][9];
    float x[9];
    float Tp_T[4];
    float T0_eq[2];
    float theta_eq[4];
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif
    // LQR 计算
    GetK_Pro_Tripod(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, k, is_take_off);
    GetT0_Pro_Tripod(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, T0_eq);
    GetTheta_Pro_Bipedal(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, theta_eq);

    x[0] = X0_OFFSET + (CHASSIS.fdb.body.x - CHASSIS.ref.body.x);
    x[1] = X1_OFFSET + (CHASSIS.fdb.body.x_dot_obv - CHASSIS.ref.speed_vector.vx);
    x[2] = X2_OFFSET + WrapToPi(CHASSIS.fdb.body.yaw - CHASSIS.ref.body.yaw);
    x[3] = X3_OFFSET + (CHASSIS.fdb.body.yaw_dot - CHASSIS.ref.speed_vector.wz);
    x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[0].theta - CHASSIS.ref.rod_Angle[0] - theta_eq[0]);
    x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[0].theta_dot - 0.0f);
    x[6] = X6_OFFSET + (CHASSIS.fdb.leg_state[1].theta - CHASSIS.ref.rod_Angle[1] - theta_eq[1]);
    x[7] = X7_OFFSET + (CHASSIS.fdb.leg_state[1].theta_dot - 0.0f);
    x[8] = X8_OFFSET + (CHASSIS.fdb.body.phi_dot - 0.0f);
    CalcLQR_Pro_Tripod(k, x, Tp_T);
    // float Tt_comp = PID_calc(&CHASSIS.pid.tail_x, CHASSIS.fdb.body.x, CHASSIS.ref.body.x);
    // float T_comp =
    //     PID_calc(&CHASSIS.pid.pitch_dot, CHASSIS.fdb.body.phi, CHASSIS.ref.body.pitch + Tt_comp);
    CHASSIS.cmd.leg[0].wheel.T = Tp_T[3];// + T_comp;
    CHASSIS.cmd.leg[1].wheel.T = Tp_T[2];// + T_comp;
    CHASSIS.cmd.leg[0].rod.Tp = Tp_T[1] + T0_eq[0];
    CHASSIS.cmd.leg[1].rod.Tp = Tp_T[0] + T0_eq[0];
    // CHASSIS.cmd.leg[0].wheel.T = 0.0f;
    // CHASSIS.cmd.leg[1].wheel.T = 0.0f;

    // float Tp_L = PID_calc(&CHASSIS.pid.leg_T, CHASSIS.fdb.leg_state[0].theta, CHASSIS.ref.rod_Angle[0]);
    // float Tp_R = PID_calc(&CHASSIS.pid.leg_T, CHASSIS.fdb.leg_state[1].theta, CHASSIS.ref.rod_Angle[1]);
    // CHASSIS.cmd.leg[0].rod.Tp = Tp_L;
    // CHASSIS.cmd.leg[1].rod.Tp = Tp_R;

    // // 两腿协调一致
    // PID_calc(
    //     &CHASSIS.pid.leg_coordation,
    //     CHASSIS.fdb.leg_state[0].theta - CHASSIS.fdb.leg_state[1].theta, 0.0f);
    // CHASSIS.cmd.leg[0].rod.Tp += CHASSIS.pid.leg_coordation.out;
    // CHASSIS.cmd.leg[1].rod.Tp -= CHASSIS.pid.leg_coordation.out;

    float F_ff = BODY_MASS * GRAVITY * TAIL_F_ff_RATIO;
    float tail_z_comp =
        PID_calc(&CHASSIS.pid.tail_up, CHASSIS.fdb.body.phi, CHASSIS.ref.body.pitch );//+ Tt_comp
    float fdb_tail_z =
        TAIL_LENGTH *
            sinf(CHASSIS.fdb.tail_state.beta - CHASSIS.fdb.body.phi + TAIL_BETA_OMNI_to_HAND);
    float average_ll_lr_ref = (CHASSIS.ref.rod_L0[0] + CHASSIS.ref.rod_L0[1]) / 2.0f;
    float average_thetal_thetar_ref =
        (CHASSIS.ref.leg_state[0].theta + CHASSIS.ref.leg_state[1].theta) / 2.0f;
    float ref_tail_z =
        TAIL_LENGTH *
        sinf(CHASSIS.ref.tail_state.beta - CHASSIS.ref.body.pitch + TAIL_BETA_OMNI_to_HAND);
    ref_tail_z += tail_z_comp;
    float ref_tail_z_max = average_ll_lr_ref * cosf(average_thetal_thetar_ref) + WHEEL_RADIUS;
    ref_tail_z_max -= TAIL_POS_OFFSET_VERTICAL * cosf(CHASSIS.ref.body.pitch);
    ref_tail_z_max += TAIL_POS_OFFSET_HORIZON * sinf(CHASSIS.ref.body.pitch);
    ref_tail_z_max -= TAIL_WHEEL_RADIUS;
    float ref_leglength_z = average_ll_lr_ref * cosf(average_thetal_thetar_ref);
    float ref_leglength_x = average_ll_lr_ref * sinf(average_thetal_thetar_ref);
    if (ref_tail_z < 0.0f) {
        ref_tail_z = 0.0f;
        ref_leglength_z -= 2.0f * tail_z_comp;
    } else {
        ref_leglength_z -= tail_z_comp;
    }
    float F_compensate = PID_calc(&CHASSIS.pid.tail_z, fdb_tail_z, ref_tail_z);
    CHASSIS.ref.rod_L0[0] =
        sqrt(ref_leglength_z * ref_leglength_z + ref_leglength_x * ref_leglength_x);
    CHASSIS.ref.rod_L0[1] =
        sqrt(ref_leglength_z * ref_leglength_z + ref_leglength_x * ref_leglength_x);
    // float F_x = PID_calc(&CHASSIS.pid.tail_T, CHASSIS.fdb.body.x, CHASSIS.ref.body.x);
    CHASSIS.cmd.tail.Tt =
        TAIL_LENGTH *
        cosf(CHASSIS.fdb.tail_state.beta - CHASSIS.fdb.body.phi + TAIL_BETA_OMNI_to_HAND) *
        (F_ff + F_compensate);

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = -CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // PID补偿稳态误差
    float delta_L0 = 0.0f;

    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);
}

/**
 * @brief      运动控制器 尾巴离地模式Pro版——LQR
 */
static void LocomotionController_Pro_Tripod(void)
{
    //     // 计算LQR增益=============================================
    //     float k[5][12];
    //     float x[12];
    //     float Tp_T_Tt[5];
    //     float theta_eq[4];
    //     float T0_eq[2];
    //     bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
    // #if LIFTED_UP
    //     is_take_off = true;
    // #endif
    //     // if (CHASSIS.step == JUMP_STEP_RECOVERY) {
    //     //     is_take_off = true;
    //     // }

    //     // LQR 计算
    //     GetK_Pro_Bipedal(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, k, is_take_off);
    //     GetTheta_Pro_Bipedal(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, theta_eq);
    //     GetT0_Pro_Bipedal(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, T0_eq);

    //     x[0] = X0_OFFSET + (CHASSIS.fdb.body.x - CHASSIS.ref.body.x);
    //     x[1] = X1_OFFSET + (CHASSIS.fdb.body.x_dot_obv - CHASSIS.ref.speed_vector.vx);
    //     x[2] = X2_OFFSET + WrapToPi(CHASSIS.fdb.body.yaw - CHASSIS.ref.body.yaw);
    //     x[3] = X3_OFFSET + (CHASSIS.fdb.body.yaw_dot - CHASSIS.ref.speed_vector.wz);
    //     x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[0].theta - CHASSIS.ref.leg_state[0].theta -
    //                         theta_eq[0]);  // - theta_eq[0]
    //     x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[0].theta_dot - 0.0f);
    //     x[6] = X6_OFFSET + (CHASSIS.fdb.leg_state[1].theta - CHASSIS.ref.leg_state[1].theta -
    //                         theta_eq[1]);  // - theta_eq[1]
    //     x[7] = X7_OFFSET + (CHASSIS.fdb.leg_state[1].theta_dot - 0.0f);
    //     x[8] = X8_OFFSET + (CHASSIS.fdb.body.phi - CHASSIS.ref.body.pitch);
    //     x[9] = X9_OFFSET + (CHASSIS.fdb.body.phi_dot - 0.0f);
    //     x[10] = X10_OFFSET + (CHASSIS.fdb.tail_state.beta - CHASSIS.ref.tail_state.beta);
    //     x[11] = X11_OFFSET + (CHASSIS.fdb.tail_state.beta_dot - CHASSIS.ref.tail_state.beta_dot);
    //     CalcLQR_Pro_Tail(k, x, Tp_T_Tt);

    //     CHASSIS.cmd.leg[0].rod.Tp = Tp_T_Tt[1] + T0_eq[0];
    //     CHASSIS.cmd.leg[1].rod.Tp = Tp_T_Tt[0] + T0_eq[0];

    //     CHASSIS.cmd.leg[0].wheel.T = Tp_T_Tt[3];
    //     CHASSIS.cmd.leg[1].wheel.T = Tp_T_Tt[2];

    //     float F_ff = BODY_MASS * GRAVITY * TAIL_F_ff_RATIO;
    //     float T_ground =
    //         TAIL_LENGTH *
    //         cosf(CHASSIS.fdb.tail_state.beta - CHASSIS.fdb.body.phi + TAIL_BETA_OMNI_to_HAND) * (F_ff);
    //     CHASSIS.cmd.tail.Tt = Tp_T_Tt[4] + T0_eq[1];

    // 计算LQR增益=============================================
    float k[4][10];
    float MPC_k[2][6];
    float x[10];
    float MPC_x[6];
    float Tp_T[4];
    float theta_eq[3];
    float Delta_Tp;
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif
    // if (CHASSIS.step == JUMP_STEP_RECOVERY) {
    //     is_take_off = true;
    // }

    // LQR 计算
    GetK_Pro_NoTail(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, k, MPC_k, is_take_off);
    GetTheta_Pro_NoTail(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, theta_eq);

    x[0] = X0_OFFSET + (CHASSIS.fdb.body.x - CHASSIS.ref.body.x);
    x[1] = X1_OFFSET + (CHASSIS.fdb.body.x_dot_obv - CHASSIS.ref.speed_vector.vx);
    x[2] = X2_OFFSET + WrapToPi(CHASSIS.fdb.body.yaw - CHASSIS.ref.body.yaw);
    x[3] = X3_OFFSET + (CHASSIS.fdb.body.yaw_dot - CHASSIS.ref.speed_vector.wz);
    x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[0].theta - CHASSIS.ref.leg_state[0].theta -
                        theta_eq[0]);  // - theta_eq[0]
    x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[0].theta_dot - 0.0f);
    x[6] = X6_OFFSET + (CHASSIS.fdb.leg_state[1].theta - CHASSIS.ref.leg_state[1].theta -
                        theta_eq[1]);  // - theta_eq[1]
    x[7] = X7_OFFSET + (CHASSIS.fdb.leg_state[1].theta_dot - 0.0f);
    x[8] = X8_OFFSET + (CHASSIS.fdb.body.phi - CHASSIS.ref.body.pitch);
    x[9] = X9_OFFSET + (CHASSIS.fdb.body.phi_dot - 0.0f);
    CalcLQR_Pro_NoTail(k, x, Tp_T);

    // MPC 计算
    MPC_x[0] = CHASSIS.fdb.leg_state[0].Delta_theta;
    MPC_x[1] = CHASSIS.fdb.leg_state[0].Delta_theta_dot;
    MPC_x[2] = CHASSIS.fdb.leg_state[0].Delta_x;
    MPC_x[3] = CHASSIS.fdb.leg_state[0].Delta_x_dot;
    MPC_x[4] = CHASSIS.fdb.leg_state[0].Delta_phi;
    MPC_x[5] = CHASSIS.fdb.leg_state[0].Delta_phi_dot;
    GetK_MPC(CHASSIS.fdb.leg[0].rod.L0, MPC_k);
    CalcMPC(MPC_k, MPC_x, &Delta_Tp);
    CHASSIS.cmd.leg[0].rod.Tp = Tp_T[1] + Delta_Tp;

    MPC_x[0] = CHASSIS.fdb.leg_state[1].Delta_theta;
    MPC_x[1] = CHASSIS.fdb.leg_state[1].Delta_theta_dot;
    MPC_x[2] = CHASSIS.fdb.leg_state[1].Delta_x;
    MPC_x[3] = CHASSIS.fdb.leg_state[1].Delta_x_dot;
    MPC_x[4] = CHASSIS.fdb.leg_state[1].Delta_phi;
    MPC_x[5] = CHASSIS.fdb.leg_state[1].Delta_phi_dot;
    GetK_MPC(CHASSIS.fdb.leg[1].rod.L0, MPC_k);
    CalcMPC(MPC_k, MPC_x, &Delta_Tp);
    CHASSIS.cmd.leg[1].rod.Tp = Tp_T[0] + Delta_Tp;

    CHASSIS.cmd.leg[0].wheel.T = Tp_T[3];
    CHASSIS.cmd.leg[1].wheel.T = Tp_T[2];

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = -CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // PID补偿稳态误差
    float delta_L0 = 0.0f;

    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);

    float F_ff = BODY_MASS * GRAVITY * TAIL_F_ff_RATIO;
    float T_ground =
        TAIL_LENGTH *
        cosf(CHASSIS.fdb.tail_state.beta - CHASSIS.fdb.body.phi + TAIL_BETA_OMNI_to_HAND) * (F_ff);
    CHASSIS.cmd.tail.Tt = T_ground;
}

/**
 * @brief      运动控制器 有尾模式
 */
static void LocomotionController_Tripod(void)
{
    // 计算LQR增益=============================================
    float k[3][8];
    float x[8];
    float T_Tp_Tt[3];
    // bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off ||
    //                    CHASSIS.fdb.tail.is_take_off;
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif
    // if (CHASSIS.step == JUMP_STEP_RECOVERY) {
    //     is_take_off = true;
    // }

    for (uint8_t i = 0; i < 2; i++) {
        GetK_Tripod(CHASSIS.fdb.leg[i].rod.L0, k, is_take_off);

        // clang-format off
        x[0] = X0_OFFSET + (CHASSIS.fdb.leg_state[i].theta     - CHASSIS.ref.leg_state[i].theta);
        x[1] = X1_OFFSET + (CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.ref.leg_state[i].theta_dot);
        x[2] = X2_OFFSET + (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x);
        x[3] = X3_OFFSET + (CHASSIS.fdb.leg_state[i].x_dot     - CHASSIS.ref.leg_state[i].x_dot);
        x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[i].phi       - CHASSIS.ref.leg_state[i].phi);
        x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[i].phi_dot   - CHASSIS.ref.leg_state[i].phi_dot);
        x[6] = X6_OFFSET + (CHASSIS.fdb.tail_state.beta       - CHASSIS.ref.tail_state.beta);
        x[7] = X7_OFFSET + (CHASSIS.fdb.tail_state.beta_dot   - CHASSIS.ref.tail_state.beta_dot);
        // clang-format on
        CalcLQR_Tail(k, x, T_Tp_Tt);

        CHASSIS.cmd.leg[i].wheel.T = T_Tp_Tt[0];
        CHASSIS.cmd.leg[i].rod.Tp = T_Tp_Tt[1];  // + Get_Tp0_Tripod(CHASSIS.fdb.leg[i].rod.L0);
        // CHASSIS.cmd.tail.Tt = T_Tp_Tt[2];
    }

    x[0] = X0_OFFSET + (CHASSIS.fdb.leg_state[0].theta + CHASSIS.fdb.leg_state[1].theta -
                        CHASSIS.ref.leg_state[0].theta - CHASSIS.ref.leg_state[1].theta) /
                           2;
    x[1] = X1_OFFSET + (CHASSIS.fdb.leg_state[0].theta_dot + CHASSIS.fdb.leg_state[1].theta_dot -
                        CHASSIS.ref.leg_state[0].theta_dot - CHASSIS.ref.leg_state[1].theta_dot) /
                           2;
    x[2] = X2_OFFSET + (CHASSIS.fdb.leg_state[0].x + CHASSIS.fdb.leg_state[1].x -
                        CHASSIS.ref.leg_state[0].x - CHASSIS.ref.leg_state[1].x) /
                           2;
    x[3] = X3_OFFSET + (CHASSIS.fdb.leg_state[0].x_dot + CHASSIS.fdb.leg_state[1].x_dot -
                        CHASSIS.ref.leg_state[0].x_dot - CHASSIS.ref.leg_state[1].x_dot) /
                           2;
    x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[0].phi + CHASSIS.fdb.leg_state[1].phi -
                        CHASSIS.ref.leg_state[0].phi - CHASSIS.ref.leg_state[1].phi) /
                           2;
    x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[0].phi_dot + CHASSIS.fdb.leg_state[1].phi_dot -
                        CHASSIS.ref.leg_state[0].phi_dot - CHASSIS.ref.leg_state[1].phi_dot) /
                           2;
    x[6] = X6_OFFSET + (CHASSIS.fdb.tail_state.beta - CHASSIS.ref.tail_state.beta);
    x[7] = X7_OFFSET + (CHASSIS.fdb.tail_state.beta_dot - CHASSIS.ref.tail_state.beta_dot);
    GetK_Tripod((CHASSIS.fdb.leg[0].rod.L0 + CHASSIS.fdb.leg[1].rod.L0) / 2.0f, k, is_take_off);
    CalcLQR_Tail(k, x, T_Tp_Tt);
    CHASSIS.cmd.tail.Tt = T_Tp_Tt
        [2];  // + Get_Tt0_Tripod((CHASSIS.fdb.leg[0].rod.L0 + CHASSIS.fdb.leg[1].rod.L0) / 2.0f);

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = -CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // PID补偿稳态误差
    float delta_L0 = 0.0f;

    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);

    // 两腿协调一致
    PID_calc(
        &CHASSIS.pid.leg_coordation,
        CHASSIS.fdb.leg_state[0].theta - CHASSIS.fdb.leg_state[1].theta, 0.0f);
    CHASSIS.cmd.leg[0].rod.Tp += CHASSIS.pid.leg_coordation.out;
    CHASSIS.cmd.leg[1].rod.Tp -= CHASSIS.pid.leg_coordation.out;

    // 转向控制================================================
    if (!is_take_off) {
        // float Tau = KP_CHASSIS_YAW_ANGLE * (CHASSIS.fdb.body.yaw - CHASSIS.ref.body.yaw) +
        //             KD_CHASSIS_YAW_ANGLE * CHASSIS.fdb.body.yaw_dot;
        // if (Tau > 7.0f) Tau = 7.0f;
        // if (Tau < -7.0f) Tau = -7.0f;
        // CHASSIS.cmd.leg[0].wheel.T += Tau / 2.0f;
        // CHASSIS.cmd.leg[1].wheel.T -= Tau / 2.0f;
        PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
        // PID_calc(&CHASSIS.pid.yaw_angle, CHASSIS.fdb.body.yaw, CHASSIS.ref.body.yaw);
        CHASSIS.cmd.leg[0].wheel.T -= CHASSIS.pid.yaw_velocity.out;
        CHASSIS.cmd.leg[1].wheel.T += CHASSIS.pid.yaw_velocity.out;
    }

    PID_calc(&CHASSIS.pid.tail_comp, CHASSIS.fdb.body.pitch, CHASSIS.ref.body.pitch);
    CHASSIS.cmd.tail.Tt -= CHASSIS.pid.tail_comp.out;

    // PID_calc(&CHASSIS.pid.tail_up, CHASSIS.fdb.tail_state.beta, CHASSIS.ref.tail_state.beta);
    // CHASSIS.cmd.tail.Tt -= CHASSIS.pid.tail_up.out;
}

/**
 * @brief      运动控制器 尾巴离地模式Pro版
 */
static void LocomotionController_Pro_Bipedal(void)
{
    // 计算LQR增益=============================================
    float k[5][12];
    float x[12];
    float Tp_T_Tt[5];
    float theta_eq[4];
    float T0_eq[2];
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif
    // if (CHASSIS.step == JUMP_STEP_RECOVERY) {
    //     is_take_off = true;
    // }

    // LQR 计算
    GetK_Pro_Bipedal(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, k, is_take_off);
    GetTheta_Pro_Bipedal(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, theta_eq);
    GetT0_Pro_Bipedal(CHASSIS.fdb.leg[0].rod.L0, CHASSIS.fdb.leg[1].rod.L0, T0_eq);

    x[0] = X0_OFFSET + (CHASSIS.fdb.body.x - CHASSIS.ref.body.x);
    x[1] = X1_OFFSET + (CHASSIS.fdb.body.x_dot_obv - CHASSIS.ref.speed_vector.vx);
    x[2] = X2_OFFSET + WrapToPi(CHASSIS.fdb.body.yaw - CHASSIS.ref.body.yaw);
    x[3] = X3_OFFSET + (CHASSIS.fdb.body.yaw_dot - CHASSIS.ref.speed_vector.wz);
    x[4] =
        X4_OFFSET + (CHASSIS.fdb.leg_state[0].theta - CHASSIS.ref.leg_state[0].theta - theta_eq[0]);
    x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[0].theta_dot - 0.0f);
    x[6] =
        X6_OFFSET + (CHASSIS.fdb.leg_state[1].theta - CHASSIS.ref.leg_state[1].theta - theta_eq[1]);
    x[7] = X7_OFFSET + (CHASSIS.fdb.leg_state[1].theta_dot - 0.0f);
    x[8] = X8_OFFSET + (CHASSIS.fdb.body.phi - CHASSIS.ref.body.pitch);
    x[9] = X9_OFFSET + (CHASSIS.fdb.body.phi_dot - 0.0f);
    x[10] = X10_OFFSET + (CHASSIS.fdb.tail_state.beta - CHASSIS.ref.tail_state.beta);
    x[11] = X11_OFFSET + (CHASSIS.fdb.tail_state.beta_dot - CHASSIS.ref.tail_state.beta_dot);
    CalcLQR_Pro_Tail(k, x, Tp_T_Tt);

    CHASSIS.cmd.leg[0].rod.Tp = Tp_T_Tt[1] + T0_eq[0];
    CHASSIS.cmd.leg[1].rod.Tp = Tp_T_Tt[0] + T0_eq[0];

    CHASSIS.cmd.leg[0].wheel.T = Tp_T_Tt[3];
    CHASSIS.cmd.leg[1].wheel.T = Tp_T_Tt[2];

    CHASSIS.cmd.tail.Tt = Tp_T_Tt[4] + T0_eq[1];

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = -CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // float e_beta = CHASSIS.ref.tail_state.beta - CHASSIS.fdb.tail_state.beta;
    // float tail_z_comp;

    // if (e_beta > 0.17f)
    //     counter_beta++;
    // else
    //     counter_beta = 0;
    // if (counter_beta > 200)
    //     tail_z_comp = PID_calc(&CHASSIS.pid.tail_comp, e_beta, 0);
    // else
    //     tail_z_comp = 0.0f;

    // // PID补偿稳态误差
    float delta_L0 = 0.0f;
    // CHASSIS.ref.rod_L0[0] += (-tail_z_comp);
    // CHASSIS.ref.rod_L0[1] += (-tail_z_comp);
    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);
}

/**
 * @brief      运动控制器 有尾模式
 */
static void LocomotionController_Bipedal(void)
{
    // 计算LQR增益=============================================
    float k[3][8];
    float x[8];
    float T_Tp_Tt[3];
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif
    // if (CHASSIS.step == JUMP_STEP_RECOVERY) {
    //     is_take_off = true;
    // }

    for (uint8_t i = 0; i < 2; i++) {
        GetK_Bipedal(CHASSIS.fdb.leg[i].rod.L0, k, is_take_off);

        // clang-format off
        x[0] = X0_OFFSET + (CHASSIS.fdb.leg_state[i].theta     - CHASSIS.ref.leg_state[i].theta);
        x[1] = X1_OFFSET + (CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.ref.leg_state[i].theta_dot);
        x[2] = X2_OFFSET + (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x);
        x[3] = X3_OFFSET + (CHASSIS.fdb.leg_state[i].x_dot     - CHASSIS.ref.leg_state[i].x_dot);
        x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[i].phi       - CHASSIS.ref.leg_state[i].phi);
        x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[i].phi_dot   - CHASSIS.ref.leg_state[i].phi_dot);
        x[6] = X6_OFFSET + (CHASSIS.fdb.tail_state.beta       - CHASSIS.ref.tail_state.beta);
        x[7] = X7_OFFSET + (CHASSIS.fdb.tail_state.beta_dot   - CHASSIS.ref.tail_state.beta_dot);
        // clang-format on
        CalcLQR_Tail(k, x, T_Tp_Tt);

        CHASSIS.cmd.leg[i].wheel.T = T_Tp_Tt[0];
        CHASSIS.cmd.leg[i].rod.Tp = T_Tp_Tt[1];
        // CHASSIS.cmd.tail.Tt = T_Tp_Tt[2];
    }

    x[0] = X0_OFFSET + (CHASSIS.fdb.leg_state[0].theta + CHASSIS.fdb.leg_state[1].theta -
                        CHASSIS.ref.leg_state[0].theta - CHASSIS.ref.leg_state[1].theta) /
                           2;
    x[1] = X1_OFFSET + (CHASSIS.fdb.leg_state[0].theta_dot + CHASSIS.fdb.leg_state[1].theta_dot -
                        CHASSIS.ref.leg_state[0].theta_dot - CHASSIS.ref.leg_state[1].theta_dot) /
                           2;
    x[2] = X2_OFFSET + (CHASSIS.fdb.leg_state[0].x + CHASSIS.fdb.leg_state[1].x -
                        CHASSIS.ref.leg_state[0].x - CHASSIS.ref.leg_state[1].x) /
                           2;
    x[3] = X3_OFFSET + (CHASSIS.fdb.leg_state[0].x_dot + CHASSIS.fdb.leg_state[1].x_dot -
                        CHASSIS.ref.leg_state[0].x_dot - CHASSIS.ref.leg_state[1].x_dot) /
                           2;
    x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[0].phi + CHASSIS.fdb.leg_state[1].phi -
                        CHASSIS.ref.leg_state[0].phi - CHASSIS.ref.leg_state[1].phi) /
                           2;
    x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[0].phi_dot + CHASSIS.fdb.leg_state[1].phi_dot -
                        CHASSIS.ref.leg_state[0].phi_dot - CHASSIS.ref.leg_state[1].phi_dot) /
                           2;
    x[6] = X6_OFFSET + (CHASSIS.fdb.tail_state.beta - CHASSIS.ref.tail_state.beta);
    x[7] = X7_OFFSET + (CHASSIS.fdb.tail_state.beta_dot - CHASSIS.ref.tail_state.beta_dot);
    CalcLQR_Tail(k, x, T_Tp_Tt);
    CHASSIS.cmd.tail.Tt =
        T_Tp_Tt[2] -
        (0.81f * 9.81f * 0.22f) * arm_cos_f32(CHASSIS.fdb.tail_state.beta - CHASSIS.fdb.body.pitch);

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = -CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // PID补偿稳态误差
    float delta_L0 = 0.0f;

    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);

    // 两腿协调一致
    PID_calc(
        &CHASSIS.pid.leg_coordation,
        CHASSIS.fdb.leg_state[0].theta - CHASSIS.fdb.leg_state[1].theta, 0.0f);
    CHASSIS.cmd.leg[0].rod.Tp += CHASSIS.pid.leg_coordation.out;
    CHASSIS.cmd.leg[1].rod.Tp -= CHASSIS.pid.leg_coordation.out;

    // 转向控制================================================
    if (!is_take_off) {
        PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
        // PID_calc(&CHASSIS.pid.yaw_angle, CHASSIS.fdb.body.yaw, CHASSIS.ref.body.yaw);
        CHASSIS.cmd.leg[0].wheel.T -= CHASSIS.pid.yaw_velocity.out;
        CHASSIS.cmd.leg[1].wheel.T += CHASSIS.pid.yaw_velocity.out;
        // float Tau = KP_CHASSIS_YAW_ANGLE * (CHASSIS.fdb.body.yaw - CHASSIS.ref.body.yaw) +
        //             KD_CHASSIS_YAW_ANGLE * CHASSIS.fdb.body.yaw_dot;
        // if (Tau > 7.0f) Tau = 7.0f;
        // if (Tau < -7.0f) Tau = -7.0f;
        // CHASSIS.cmd.leg[0].wheel.T += Tau / 2.0f;
        // CHASSIS.cmd.leg[1].wheel.T -= Tau / 2.0f;
    }
}

/**
 * @brief Roll控制
 */
void Roll_Control(
    float * LeftLeg_DeltaL0, float * RightLeg_DeltaL0, float * LeftLeg_DeltaF,
    float * RightLeg_DeltaF)
{
    float Delta_L0 = CHASSIS.fdb.leg[1].rod.L0 - CHASSIS.fdb.leg[0].rod.L0;
    float A = Delta_L0 * arm_cos_f32(CHASSIS.fdb.body.roll - CHASSIS.ref.body.roll) +
              WHEEL_BASE * arm_sin_f32(CHASSIS.fdb.body.roll - CHASSIS.ref.body.roll);
    float B = -Delta_L0 * arm_sin_f32(CHASSIS.fdb.body.roll - CHASSIS.ref.body.roll) +
              WHEEL_BASE * arm_cos_f32(CHASSIS.fdb.body.roll - CHASSIS.ref.body.roll);
    float tan_delta, L0d_r, L0d_l;
    if (B == 0) {
        L0d_r = L0d_l = 0;
    } else {
        tan_delta = A / B;

        L0d_r = WHEEL_BASE_2 * tan_delta;
        L0d_l = -WHEEL_BASE_2 * tan_delta;
    }
    (*LeftLeg_DeltaL0) = L0d_l;
    (*RightLeg_DeltaL0) = L0d_r;

    PID_calc(&CHASSIS.pid.roll_angle, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);
    (*LeftLeg_DeltaF) = CHASSIS.pid.roll_angle.out;
    (*RightLeg_DeltaF) = -CHASSIS.pid.roll_angle.out;
}

float LegLength_Limit(float L, float min, float max)
{
    if (L < min) return min;
    if (L > max) return max;
    return L;
}

/**
 * @brief 腿部力矩控制
 */
static void LegTorqueController(void)
{
    // 腿长控制
    float F_ff, F_compensate;
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;

    float roll_vel_limit_f =
        fp32_constrain(CHASSIS.fdb.body.roll_dot * ROLL_VEL_LIMIT_FACTOR, -0.2, 0.2);

    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.step == JUMP_STEP_JUMP) {
            // 直接给一个超大力F起飞
            CHASSIS.cmd.leg[i].rod.F = 40;
        } else {
            // 计算前馈力
            switch (CHASSIS.mode) {
                case CHASSIS_TRIPOD: {
                    F_ff = LegFeedForward_Tripod(CHASSIS.fdb.leg_state[i].theta) * FF_RATIO;
                } break;
                case CHASSIS_STAND_UP:
                case CHASSIS_NOTAIL:
                case CHASSIS_BIPEDAL:
                default: {
            F_ff = LegFeedForward(CHASSIS.fdb.leg_state[i].theta) * FF_RATIO;
                } break;
            }

            // PID补偿
            F_compensate = PID_calc_Leg(
                &CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0, CHASSIS.ref.rod_L0[i],
                is_take_off);
            // 计算总力
            CHASSIS.cmd.leg[i].rod.F = F_ff + F_compensate;
        }
        // CHASSIS.cmd.leg[i].rod.F = F_ff + F_compensate - F_ff;
    }

    CHASSIS.cmd.leg[0].rod.F -= roll_vel_limit_f;
    CHASSIS.cmd.leg[1].rod.F += roll_vel_limit_f;

    // 转换为关节力矩
    CalcVmc(
        CHASSIS.cmd.leg[0].rod.F, CHASSIS.cmd.leg[0].rod.Tp, CHASSIS.fdb.leg[0].J,
        CHASSIS.cmd.leg[0].joint.T);
    CalcVmc(
        CHASSIS.cmd.leg[1].rod.F, CHASSIS.cmd.leg[1].rod.Tp, CHASSIS.fdb.leg[1].J,
        CHASSIS.cmd.leg[1].joint.T);
}

/**
 * @brief        前馈控制
 * @param[in]    theta 当前腿与竖直方向夹角
 * @return       前馈量
 */
static float LegFeedForward(float theta) { return BODY_MASS * GRAVITY * cosf(theta) / 2; }

static float LegFeedForward_Tripod(float theta)
{
    return (1 - TAIL_F_ff_RATIO) * BODY_MASS * GRAVITY * cosf(theta) / 2;
}

/**
 * @brief         矩阵相乘，计算LQR输出
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    T_Tp 反馈数据T和Tp
 * @return        none
 */
static void CalcLQR_NoTail(float k[2][6], float x[6], float T_Tp[2])
{
    T_Tp[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] +
              k[0][5] * x[5];
    T_Tp[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] +
              k[1][5] * x[5];
}

/**
 * @brief         矩阵相乘，计算LQR输出
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    Tp_T 反馈数据T和Tp
 * @return        none
 */
static void CalcLQR_Pro_NoTail(float k[4][10], float x[10], float Tp_T[4])
{
    // for (int i = 0; i < 4; i++) {
    //     Tp_T[i] = 0;
    //     for (int j = 0; j < 10; j++) {
    //         Tp_T[i] -= k[i][j] * x[j];
    //     }
    // }

    Tp_T[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] +
              k[0][5] * x[5] + k[0][6] * x[6] + k[0][7] * x[7] - k[0][8] * x[8] - k[0][9] * x[9];

    Tp_T[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] +
              k[1][5] * x[5] + k[1][6] * x[6] + k[1][7] * x[7] - k[1][8] * x[8] - k[1][9] * x[9];

    Tp_T[2] = k[2][0] * x[0] + k[2][1] * x[1] + k[2][2] * x[2] + k[2][3] * x[3] + k[2][4] * x[4] +
              k[2][5] * x[5] + k[2][6] * x[6] + k[2][7] * x[7] - k[2][8] * x[8] - k[2][9] * x[9];

    Tp_T[3] = k[3][0] * x[0] + k[3][1] * x[1] + k[3][2] * x[2] + k[3][3] * x[3] + k[3][4] * x[4] +
              k[3][5] * x[5] + k[3][6] * x[6] + k[3][7] * x[7] - k[3][8] * x[8] - k[3][9] * x[9];
}

/**
 * @brief         矩阵相乘，计算LQR输出 尾着地
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    T_Tp 反馈数据T和Tp
 * @return        none
 */
static void CalcLQR_Tail(float k[3][8], float x[8], float T_Tp_Tt[3])
{
    T_Tp_Tt[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] +
                 k[0][4] * x[4] + k[0][5] * x[5] + k[0][6] * x[6] + k[0][7] * x[7];
    T_Tp_Tt[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] +
                 k[1][4] * x[4] + k[1][5] * x[5] + k[1][6] * x[6] + k[1][7] * x[7];
    T_Tp_Tt[2] = k[2][0] * x[0] + k[2][1] * x[1] + k[2][2] * x[2] + k[2][3] * x[3] +
                 k[2][4] * x[4] + k[2][5] * x[5] + k[2][6] * x[6] + k[2][7] * x[7];
}

static void CalcLQR_Pro_Tail(float k[5][12], float x[12], float Tp_T_Tt[5])
{
    // for (int i = 0; i < 4; i++) {
    //     Tp_T[i] = 0;
    //     for (int j = 0; j < 10; j++) {
    //         Tp_T[i] -= k[i][j] * x[j];
    //     }
    // }

    Tp_T_Tt[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] +
                 k[0][4] * x[4] + k[0][5] * x[5] + k[0][6] * x[6] + k[0][7] * x[7] -
                 k[0][8] * x[8] - k[0][9] * x[9] + k[0][10] * x[10] + k[0][11] * x[11];

    Tp_T_Tt[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] +
                 k[1][4] * x[4] + k[1][5] * x[5] + k[1][6] * x[6] + k[1][7] * x[7] -
                 k[1][8] * x[8] - k[1][9] * x[9] + k[1][10] * x[10] + k[1][11] * x[11];

    Tp_T_Tt[2] = k[2][0] * x[0] + k[2][1] * x[1] + k[2][2] * x[2] + k[2][3] * x[3] +
                 k[2][4] * x[4] + k[2][5] * x[5] + k[2][6] * x[6] + k[2][7] * x[7] -
                 k[2][8] * x[8] - k[2][9] * x[9] + k[2][10] * x[10] + k[2][11] * x[11];

    Tp_T_Tt[3] = k[3][0] * x[0] + k[3][1] * x[1] + k[3][2] * x[2] + k[3][3] * x[3] +
                 k[3][4] * x[4] + k[3][5] * x[5] + k[3][6] * x[6] + k[3][7] * x[7] -
                 k[3][8] * x[8] - k[3][9] * x[9] + k[3][10] * x[10] + k[3][11] * x[11];

    Tp_T_Tt[4] = k[4][0] * x[0] + k[4][1] * x[1] + k[4][2] * x[2] + k[4][3] * x[3] +
                 k[4][4] * x[4] + k[4][5] * x[5] + k[4][6] * x[6] + k[4][7] * x[7] -
                 k[4][8] * x[8] - k[4][9] * x[9] + k[4][10] * x[10] + k[4][11] * x[11];
}

static void CalcLQR_Pro_Tripod(float k[4][9], float x[9], float Tp_T_Tt[4])
{
    // for (int i = 0; i < 4; i++) {
    //     Tp_T[i] = 0;
    //     for (int j = 0; j < 10; j++) {
    //         Tp_T[i] -= k[i][j] * x[j];
    //     }
    // }

    Tp_T_Tt[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] +
                 k[0][4] * x[4] + k[0][5] * x[5] + k[0][6] * x[6] + k[0][7] * x[7] - k[0][8] * x[8];

    Tp_T_Tt[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] +
                 k[1][4] * x[4] + k[1][5] * x[5] + k[1][6] * x[6] + k[1][7] * x[7] - k[1][8] * x[8];

    Tp_T_Tt[2] = k[2][0] * x[0] + k[2][1] * x[1] + k[2][2] * x[2] + k[2][3] * x[3] +
                 k[2][4] * x[4] + k[2][5] * x[5] + k[2][6] * x[6] + k[2][7] * x[7] - k[2][8] * x[8];

    Tp_T_Tt[3] = k[3][0] * x[0] + k[3][1] * x[1] + k[3][2] * x[2] + k[3][3] * x[3] +
                 k[3][4] * x[4] + k[3][5] * x[5] + k[3][6] * x[6] + k[3][7] * x[7] - k[3][8] * x[8];
}

/**
 * @brief         计算MPC输出
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    Delta_Tp Tp补偿量
 * @return        none
 */
static void CalcMPC(float k[2][6], float x[6], float * Delta_Tp)
{
    *Delta_Tp = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] +
                k[0][5] * x[5];
}

//* 各个模式下的控制

static void ConsoleZeroForce(void)
{
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;

    CHASSIS.tail_motor.set.tor = 0;

    // CHASSIS.wheel_motor[0].set.value =
    //     PID_calc(&CHASSIS.pid.wheel_stop[0], CHASSIS.wheel_motor[0].fdb.vel, 0);
    // CHASSIS.wheel_motor[1].set.value =
    //     PID_calc(&CHASSIS.pid.wheel_stop[1], CHASSIS.wheel_motor[1].fdb.vel, 0);
}

static void ConsoleNoTail(void)  //仅两条腿
{
    if (PRO_CONTROLLER)
        LocomotionController_Pro_NoTail();
    else
        LocomotionController_NoTail();

    LegTorqueController();

    // 给关节电机赋值
    CHASSIS.joint_motor[0].set.tor = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    CHASSIS.joint_motor[1].set.tor = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    CHASSIS.joint_motor[2].set.tor = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    CHASSIS.joint_motor[3].set.tor = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

    for (uint8_t i = 0; i < 4; i++) {
        if (CHASSIS.step == JUMP_STEP_JUMP) {
            CHASSIS.joint_motor[i].set.tor = fp32_constrain(
                CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE_JUMP, MAX_JOINT_TORQUE_JUMP);
        } else {
            CHASSIS.joint_motor[i].set.tor =
                fp32_constrain(CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        }
    }

    // 给驱动轮电机赋值
    CHASSIS.wheel_motor[0].set.tor = CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION);
    CHASSIS.wheel_motor[1].set.tor = CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION);

    CHASSIS.tail_motor.set.tor = 0.0f;
}

static void ConsoleBipedal(void)  //尾离地
{
    if (PRO_CONTROLLER)
        LocomotionController_Pro_Bipedal();
    else
        LocomotionController_Bipedal();

    LegTorqueController();

    // 给关节电机赋值 待验证
    CHASSIS.joint_motor[0].set.tor = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    CHASSIS.joint_motor[1].set.tor = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    CHASSIS.joint_motor[2].set.tor = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    CHASSIS.joint_motor[3].set.tor = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

    // for (uint8_t i = 0; i < 4; i++) {
    //     if (CHASSIS.step == JUMP_STEP_JUMP) {
    //         CHASSIS.joint_motor[i].set.tor = fp32_constrain(
    //             CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE_JUMP, MAX_JOINT_TORQUE_JUMP);
    //     } else {
    //         CHASSIS.joint_motor[i].set.tor =
    //             fp32_constrain(CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    //     }
    // }

    // 给驱动轮电机赋值
    CHASSIS.wheel_motor[0].set.tor = CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION);
    CHASSIS.wheel_motor[1].set.tor = CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION);

    // CHASSIS.tail_motor->set.pos = CHASSIS.ref.tail_state.beta;
    CHASSIS.tail_motor.set.tor = CHASSIS.cmd.tail.Tt * T_DIRECTION;
}

static void ConsoleTripod(void)
{
    if (false) LocomotionController_Pro_Tripod();

    if (PRO_CONTROLLER) LocomotionController_ProX_Tripod();
    // LocomotionController_ProX_Tripod();
    else
        LocomotionController_Tripod();
    LegTorqueController();

    // 给关节电机赋值 待验证
    CHASSIS.joint_motor[0].set.tor = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    CHASSIS.joint_motor[1].set.tor = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    CHASSIS.joint_motor[2].set.tor = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    CHASSIS.joint_motor[3].set.tor = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

    // for (uint8_t i = 0; i < 4; i++) {
    //     if (CHASSIS.step == JUMP_STEP_JUMP) {
    //         CHASSIS.joint_motor[i].set.tor = fp32_constrain(
    //             CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE_JUMP, MAX_JOINT_TORQUE_JUMP);
    //     } else {
    //         CHASSIS.joint_motor[i].set.tor =
    //             fp32_constrain(CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    //     }
    // }

    // 给驱动轮电机赋值
    CHASSIS.wheel_motor[0].set.tor = CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION);
    CHASSIS.wheel_motor[1].set.tor = CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION);

    CHASSIS.tail_motor.set.tor = CHASSIS.cmd.tail.Tt * T_DIRECTION;
}

static void ConsoleStandUp(void)  // 待修改
{
    // ===腿部位置控制===

    double joint_pos_l[2], joint_pos_r[2];
    // LegController(joint_pos_l, joint_pos_r);

    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(joint_pos_l[0]) || isnan(joint_pos_l[1]) || isnan(joint_pos_r[0]) ||
          isnan(joint_pos_r[1]))) {
        CHASSIS.joint_motor[0].set.pos =
            theta_transform(joint_pos_l[1], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        CHASSIS.joint_motor[1].set.pos =
            theta_transform(joint_pos_l[0], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        CHASSIS.joint_motor[2].set.pos =
            theta_transform(joint_pos_r[1], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        CHASSIS.joint_motor[3].set.pos =
            theta_transform(joint_pos_r[0], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);
    }
    // 检测设定角度是否超过电机角度限制
    CHASSIS.joint_motor[0].set.pos =
        fp32_constrain(CHASSIS.joint_motor[0].set.pos, MIN_J0_ANGLE, MAX_J0_ANGLE);
    CHASSIS.joint_motor[1].set.pos =
        fp32_constrain(CHASSIS.joint_motor[1].set.pos, MIN_J1_ANGLE, MAX_J1_ANGLE);
    CHASSIS.joint_motor[2].set.pos =
        fp32_constrain(CHASSIS.joint_motor[2].set.pos, MIN_J2_ANGLE, MAX_J2_ANGLE);
    CHASSIS.joint_motor[3].set.pos =
        fp32_constrain(CHASSIS.joint_motor[3].set.pos, MIN_J3_ANGLE, MAX_J3_ANGLE);

    // ===驱动轮pid控制===
    float feedforward = -220;
    PID_calc(&CHASSIS.pid.stand_up, CHASSIS.fdb.body.phi, 0);
    CHASSIS.wheel_motor[0].set.value = (feedforward + CHASSIS.pid.stand_up.out) * W0_DIRECTION;
    CHASSIS.wheel_motor[1].set.value = (feedforward + CHASSIS.pid.stand_up.out) * W1_DIRECTION;
}

/******************************************************************/
/* Cmd                                                            */
/*----------------------------------------------------------------*/
/* main function:      ChassisSendCmd                             */
/* auxiliary function: SendJointMotorCmd                          */
/*                     SendWheelMotorCmd                          */
/******************************************************************/

#define DM_DELAY 250

#define DEBUG_KP 17
#define DEBUG_KD 2

static void SendJointMotorCmd(void);
static void SendWheelMotorCmd(void);

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisSendCmd(void)
{
    SendJointMotorCmd();
    SendWheelMotorCmd();
}

/**
 * @brief 发送关节电机控制指令
 * @param[in] chassis
 */
static void SendJointMotorCmd(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_NOTAIL:
        case CHASSIS_BIPEDAL:
        case CHASSIS_TRIPOD: {
            LkMultipleTorqueControl_MG6012(
                JOINT_CAN, CHASSIS.joint_motor[0].set.tor, CHASSIS.joint_motor[1].set.tor,
                CHASSIS.joint_motor[2].set.tor, CHASSIS.joint_motor[3].set.tor);

            // LkMultipleTorqueControl_MG6012(JOINT_CAN, 0.0f, 0.0f, 0.0f, 0.0f);
        } break;
        case CHASSIS_STAND_UP: {
            LkSinglePostionControl(&CHASSIS.joint_motor[0]);
            LkSinglePostionControl(&CHASSIS.joint_motor[1]);
            LkSinglePostionControl(&CHASSIS.joint_motor[2]);
            LkSinglePostionControl(&CHASSIS.joint_motor[3]);
        } break;
        case CHASSIS_OFF:
        case CHASSIS_SAFE:
        default: {
            LkMultipleTorqueControl_MG6012(JOINT_CAN, 0, 0, 0, 0);
        } break;
    }
}

/**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
static void SendWheelMotorCmd(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_NOTAIL:
        case CHASSIS_BIPEDAL:
        case CHASSIS_TRIPOD: {
            if (CHASSIS.tail_motor.set.tor > 7.0f) CHASSIS.tail_motor.set.tor = 7.0f;
            if (CHASSIS.tail_motor.set.tor < -7.0f) CHASSIS.tail_motor.set.tor = -7.0f;
            LkMultipleTorqueControl_MF9025_MG5010(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor,
                CHASSIS.tail_motor.set.tor, 0);

            // LkMultipleTorqueControl_MF9025_MG5010(WHEEL_CAN, 0.0f, 0.0f, 3.0f, 0.0f);
            // printf("yaw = %.3f\r\n", CHASSIS.fdb.body.yaw);
        } break;
        case CHASSIS_STAND_UP: {
            LkMultipleTorqueControl_MF9025_MG5010(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0,
                0);
        } break;
        case CHASSIS_OFF: {
            LkMultipleTorqueControl_MF9025_MG5010(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_SAFE:
        default: {
            LkMultipleTorqueControl_MF9025_MG5010(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0,
                0);
        } break;
    }
}

/******************************************************************/
/* Public                                                         */
/*----------------------------------------------------------------*/
/* main function:      None                                       */
/*                     ChassisGetStatus                           */
/*                     ChassisGetDuration                         */
/*                     ChassisGetSpeedVx                          */
/*                     ChassisGetSpeedVy                          */
/*                     ChassisGetSpeedWz                          */
/******************************************************************/

inline uint8_t ChassisGetStatus(void) { return 0; }
inline uint32_t ChassisGetDuration(void) { return CHASSIS.duration; }
inline float ChassisGetSpeedVx(void) { return CHASSIS.fdb.speed_vector.vx; }
inline float ChassisGetSpeedVy(void) { return CHASSIS.fdb.speed_vector.vy; }
inline float ChassisGetSpeedWz(void) { return CHASSIS.fdb.speed_vector.wz; }
/*------------------------------ End of File ------------------------------*/
