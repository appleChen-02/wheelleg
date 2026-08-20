/**
  * @file       robot_param_balanced_infantry.h
  * @brief      这里是平衡步兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

// clang-format off
#define TAIL_VERSION 1        // 尾巴版本
#define __SELF_BOARD_ID C_BOARD_BALANCE_CHASSIS // 本板ID
#define __GYRO_BIAS_YAW  0.003096855f           // 陀螺仪零飘，单位rad/s

// IMU角度静态偏移标定 (rad) —— 补偿放置不水平或其他恒定角度偏差
// 使用方法：将机器人放在期望的"零位"，开机后读取稳定角度值，取反填入
#define __ANGLE_OFFSET_ROLL   0.0361f   // roll 角度静态偏移 (rad)
#define __ANGLE_OFFSET_PITCH  0.0f   // pitch 角度静态偏移 (rad)
#define __ANGLE_OFFSET_YAW    -0.016f   // yaw 角度静态偏移 (rad)

#define __CONTROL_LINK_RC  CL_RC_DIRECT  // 控制链路选择：RC遥控器
#define __CONTROL_LINK_KM  CL_KM_NONE      // 控制链路选择：键鼠数据

/*-------------------- Chassis --------------------*/
// 底盘任务相关宏定义
#define CHASSIS_TASK_INIT_TIME 357   // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS 2    // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)   // 底盘任务控制间隔

// 底盘的遥控器相关宏定义 ---------------------
// HT8A 1:1映射: rc.ch[i] = sbus.ch[i] - 中值
// CH1(ch0)=右平, CH2(ch1)=右竖, CH3(ch2)=左竖, CH4(ch3)=左平
// CH5(ch4)=开关1, CH6(ch5)=开关2, CH7(ch6)=开关3, CH8(ch7)=开关4
#define CHASSIS_MODE_CHANNEL   4  // 选择底盘状态 开关通道号 (CH5)
#define CHASSIS_FUNCTION       7  // 进一步选择底盘状态 开关通道号 (CH8)

#define CHASSIS_X_CHANNEL      2  // 前后的遥控器通道号码 (CH3 左竖)
#define CHASSIS_Y_CHANNEL      3  // 左右的遥控器通道号码 (CH4 左平)
#define CHASSIS_WZ_CHANNEL     3  // 旋转的遥控器通道号码 (CH4 左平)
#define CHASSIS_ANGLE_CHANNEL  1  // 腿摆角的遥控器通道号码 (CH2 右竖)
#define CHASSIS_LENGTH_CHANNEL 0  // 腿长的遥控器通道号码 (CH1 右平)
#define CHASSIS_ROLL_CHANNEL   0  // ROLL角的遥控器通道号码 (CH1 右平)
#define CHASSIS_PITCH_CHANNEL  1  // PITCH角的遥控器通道号码 (CH2 右竖)
#define CHASSIS_TAIL_POS_CHANNEL 1  // 尾巴位置的遥控器通道号码 (CH2 右竖)
#define CHASSIS_HAND_CHANNEL   0  // 夹爪遥控器通道号码 (CH1 右平)
#define CHASSIS_RC_DEADLINE    16 // 摇杆死区 (SBUS偏移单位, 16/800=2%)

// deadzone parameters ---------------------
#define WHEEL_DEADZONE (0.01f)  // (m/s)轮子速度死区

// ratio parameters ---------------------
#define FF_RATIO               (1.0f)   // 前馈比例系数
#define ROLL_VEL_LIMIT_FACTOR  (0.1f)    // roll角速度抑制比例系数

// motor parameters ---------------------
#define JOINT_CAN (2)
#define WHEEL_CAN (1)

#define J0_DIRECTION (-1)
#define J1_DIRECTION (1)
#define J2_DIRECTION (1)
#define J3_DIRECTION (-1)

#define W0_DIRECTION ( 1)
#define W1_DIRECTION (-1)

#define T_DIRECTION (-1)

//physical parameters ---------------------
#define LEG_L1 (0.215f)  // (m)腿1长度
#define LEG_L2 (0.258f)  // (m)腿2长度
#define LEG_L3 (LEG_L2)  // (m)腿3长度
#define LEG_L4 (LEG_L1)  // (m)腿4长度
#define LEG_L5 (0.0f)    // (m)关节间距

#define BODY_MASS            (13.0f)      // (kg)机身重量
#define WHEEL_MASS           (0.823f)      // (kg)轮子重量
#define LEG_MASS             (2.2f)      // (kg)单腿重量
#define WHEEL_RADIUS         (0.13f)    // (m)轮子半径
#define TAIL_WHEEL_RADIUS    (0.029f)
#define WHEEL_START_TORQUE   (0.3f)      // (Nm)轮子起动力矩
#define WHEEL_BASE           (0.414f)  // (m)驱动轮轴距
#define WHEEL_BASE_2         (0.207f)  // (m)驱动轮轴距

#if TAIL_VERSION
#define TAIL_COM_to_MOTOR        (0.17755f)
#define TAIL_BETA_COM_to_HAND    (0.06597f)
#define TAIL_BETA_OMNI_to_HAND   (0.383972f)
#define TAIL_POS_OFFSET          (0.089f)
#define TAIL_POS_OFFSET_HORIZON  (0.07125f)
#define TAIL_POS_OFFSET_VERTICAL (0.1105f)
#define TAIL_LENGTH              (0.240f)
#define TAIL_MASS                (0.83f)
#define TAIL_BETA_INIT           (-20.0f)

#else
#define TAIL_COM_to_MOTOR        (0.23985f)
#define TAIL_BETA_COM_to_HAND    (0.1034f)
#define TAIL_BETA_OMNI_to_HAND   (0.3141f)
#define TAIL_POS_OFFSET          (0.089f)
#define TAIL_POS_OFFSET_HORIZON  (0.0f)
#define TAIL_POS_OFFSET_VERTICAL (0.089f)
#define TAIL_LENGTH              (0.330f)
#define TAIL_MASS                (0.86f)
#define TAIL_BETA_INIT           (2.0f)

#endif

/*
 * 机体运动 KF 状态：[vx (m/s), accel_bias (m/s^2),
 * wz (rad/s), gyro_bias (rad/s)]。Q 为每秒过程噪声，实际使用时乘以
 * 任务周期。增大某一项会使对应状态更快跟随新测量，但估计结果也会更不平滑。
 */
#define MOTION_Q_VX                 (0.20f)
#define MOTION_Q_ACCEL_BIAS         (0.0025f)
#define MOTION_Q_WZ                 (0.40f)
#define MOTION_Q_GYRO_BIAS          (0.0004f)

/* 标称测量方差：轮速推导的 vx/wz 以及 IMU 偏航角速度。 */
#define MOTION_R_WHEEL_VX           (0.0025f)
#define MOTION_R_WHEEL_WZ           (0.0100f)
#define MOTION_R_GYRO_WZ            (0.0004f)

/*
 * 轮速测量 R 的倍率。增大 R 会降低该测量的卡尔曼增益，而不是突兀地删除
 * 该测量。AIRBORNE 的倍率显著更大，因为此时两个轮子均无法代表地面运动；
 * 在重新接触地面前，由 IMU 和预测模型维持状态估计。
 */
#define MOTION_R_DEGRADED_SCALE     (100.0f)
#define MOTION_R_AIRBORNE_SCALE     (10000.0f)

/*
 * 静止判定阈值：实测运动和期望运动均低于这些值时，才将零轮速作为高置信度
 * 观测量。该约束可防止底盘停放时速度及零偏持续漂移。
 */
#define MOTION_STILL_VX             (0.03f)  /* m/s */
#define MOTION_STILL_WZ             (0.05f)  /* rad/s */
#define MOTION_STILL_ACCEL          (0.30f)  /* m/s^2 */

/* 轮速偏航角速度与扣除零偏后的陀螺偏航角速度的最大允许差值，超过后降低轮速权重。 */
#define MOTION_WHEEL_GYRO_DISAGREE  (1.00f)  /* rad/s */

/*
 * 单轮打滑检测使能条件。仅在期望平移速度足够大、偏航需求较小且不处于制动瞬态时
 * 启用检测，避免将正常转向、制动或腾空时的轮速变化误判为纵向轮胎打滑。
 */
#define WHEEL_SLIP_VX_MIN               (0.10f)  /* m/s */
#define WHEEL_SLIP_WZ_MAX               (1.50f)  /* rad/s */
#define WHEEL_SLIP_DRIVE_TORQUE_MIN_NM  (0.80f)  /* N*m */

/*
 * 进入和退出使用不同的速度残差阈值 (m/s)，形成迟滞。残差必须持续
 * CONFIRM_MS 才进入 SLIPPING；恢复后仍在 RECOVERY_MS 内保持降权，
 * 防止一次含噪观测周期就将轮速恢复为完全可信。
 */
#define WHEEL_SLIP_ENTER_SPEED_MPS      (0.30f)
#define WHEEL_SLIP_EXIT_SPEED_MPS       (0.18f)
#define WHEEL_SLIP_BRAKE_ACCEL_MAX      (0.50f)  /* m/s^2 */
#define WHEEL_SLIP_CONFIRM_MS           (80U)
#define WHEEL_SLIP_RECOVERY_MS          (250U)

/*
 * 两侧轮速均正常时，使用这个很小的互补校正限制短时 IMU 积分速度漂移。
 * 该系数必须足够小，避免短暂漏检的打滑将检测参考速度拉向打滑轮速。
 */
#define WHEEL_SLIP_IMU_CORRECTION_ALPHA (0.02f)

/*
 * 双足 vx 参考轨迹器参数。速度指令先受 TRACK_TIME 限制得到目标加速度，
 * 再受 ACCEL_MAX 和 JERK_MAX 限制后积分为 LQR 速度参考。该整形避免遥控
 * 阶跃直接激励速度-俯仰慢耦合模态及非最小相位零点。
 */
#define VX_REF_TRACK_TIME       (0.20f)  /* s，速度误差转换为目标加速度的时间常数。 */
#define VX_REF_ACCEL_MAX        (1.20f)  /* m/s^2，期望纵向加速度上限。 */
#define VX_REF_JERK_MAX         (6.00f)  /* m/s^3，期望纵向加加速度上限。 */

/*
 * 双足纵向加速度前馈。前馈仅作为左右轮相同的共模扭矩叠加到原 LQR 输出，
 * 不参与偏航差动、髋关节或尾巴控制。增益应从较小值开始实验整定；限幅和
 * 变化率限制用于为 LQR、偏航和姿态控制预留轮端扭矩裕量。
 */
#define VX_ACCEL_FF_GAIN        (0.20f)  /* 每侧轮子 N*m/(m/s^2)。 */
#define VX_ACCEL_FF_TORQUE_MAX  (0.60f)  /* 每侧轮子的前馈扭矩上限，N*m。 */
#define VX_ACCEL_FF_TORQUE_RATE (6.00f)  /* 每侧轮子的前馈扭矩变化率上限，N*m/s。 */

// v1.1
// #define TAIL_COM_to_MOTOR    (0.17755f)
// #define TAIL_BETA_COM_to_HAND    (0.065972f)
// #define TAIL_BETA_OMNI_to_HAND    (0.38397f)
// v1.0
// #define TAIL_COM_to_MOTOR        (0.23985f)
// #define TAIL_BETA_COM_to_HAND    (0.1034f)
// #define TAIL_BETA_OMNI_to_HAND   (0.3141f)
// #define TAIL_POS_OFFSET          (0.089f)
// #define TAIL_POS_OFFSET_HORIZON  (0.0f)
// #define TAIL_POS_OFFSET_VERTICAL (0.089f)
// #define TAIL_LENGTH              (0.330f)
// #define TAIL_MASS                (0.86f)

#define J0_ANGLE_OFFSET     (0.0f + M_PI) // (rad)关节0角度偏移量(电机0点到水平线的夹角)
#define J1_ANGLE_OFFSET     (0.0f)         // (rad)关节1角度偏移量(电机0点到水平线的夹角)
#define J2_ANGLE_OFFSET     (0.0f - M_PI)  // (rad)关节2角度偏移量(电机0点到水平线的夹角)
#define J3_ANGLE_OFFSET     (0.0f + DOUBLE_PI)        // (rad)关节3角度偏移量(电机0点到水平线的夹角)

#define T_ANGLE_OFFSET     (0.0f)

#define DLENGTH_DIRECTION  (-1) // ROLL角补偿量方向(腿长增加方向)
//upper_limit parameters ---------------------

#define MAX_DELTA_ROD_ANGLE (0.3f) // (rad)腿摆角最大变化量
#define MAX_TORQUE_PROTECT  (25.0f)  // (Nm)最大扭矩保护

#define MAX_DELTA_VEL_FDB_TO_REF (0.8f) // (m/s)速度反馈到参考速度的最大变化量

#define MAX_THETA      (1.0f)
#define MAX_THETA_DOT  (2.0f)
#define MAX_X          (1.0f)
#define MAX_X_DOT      (10.0f)
#define MAX_PHI        (1.0f)
#define MAX_PHI_DOT    (2.0f)

#define MAX_SPEED_INTEGRAL  (0.5f)
#define MAX_ROLL            (0.3f)
#define MAX_PITCH           (0.3f)
#define MAX_ROLL_VELOCITY   (1.0f)
#define MAX_YAW             (M_PI)
#define MAX_YAW_VELOCITY    (3.0f)

#define MAX_J0_ANGLE  (1.2f) // (rad)关节角度上限
#define MAX_J1_ANGLE  (1.2f) // (rad)关节角度上限
#define MAX_J2_ANGLE  (1.2f) // (rad)关节角度上限
#define MAX_J3_ANGLE  (1.2f) // (rad)关节角度上限

#define MAX_LEG_LENGTH       (0.3f)
#define MAX_LEG_ANGLE        (MAX_DELTA_ROD_ANGLE)
#define MAX_TAIL_ANGLE       (M_PI_2)
#define MAX_SPEED            (3.5f)
#define MAX_SPEED_VECTOR_VX  (1.8f)
#define MAX_SPEED_VECTOR_VY  (1.8f)
#define MAX_SPEED_VECTOR_WZ  (5.0f)

#define MAX_JOINT_TORQUE      (10.0f)   // (Nm)关节最大扭矩
#define MAX_JOINT_TORQUE_JUMP (20.0f)  // (Nm)跳跃时的关节最大扭矩
#define MAX_VEL_ADD           (1.0f)   // (m/s)速度增量上限
#define MAX_PITCH_VEL         (0.1f)   // (rad/s)pitch轴速度上限

#define MAX_TOUCH_INTERVAL    (200)    // (ms)最大离地时间，超过这个时间认为离地

//lower_limit parameters ---------------------

#define MIN_DELTA_ROD_ANGLE (-MAX_DELTA_ROD_ANGLE) // (rad)腿摆角最小变化量

#define MIN_DELTA_VEL_FDB_TO_REF (-MAX_DELTA_VEL_FDB_TO_REF) // (m/s)速度反馈到参考速度的最小变化量

#define MIN_THETA      (-MAX_THETA)
#define MIN_THETA_DOT  (-MAX_THETA_DOT)
#define MIN_X          (-MAX_X)
#define MIN_X_DOT      (-MAX_X_DOT)
#define MIN_PHI        (-MAX_PHI)
#define MIN_PHI_DOT    (-MAX_PHI_DOT)

#define MIN_SPEED_INTEGRAL  (-MAX_SPEED_INTEGRAL)
#define MIN_ROLL            (-MAX_ROLL)
#define MIN_PITCH           (-MAX_PITCH)
#define MIN_ROLL_VELOCITY   (-MAX_ROLL_VELOCITY)
#define MIN_YAW             (-MAX_YAW)
#define MIN_YAW_VELOCITY    (-MAX_YAW_VELOCITY)

#define MIN_J0_ANGLE (-0.47f) // (rad)关节角度下限
#define MIN_J1_ANGLE (-0.27f) // (rad)关节角度下限
#define MIN_J2_ANGLE (-0.47f) // (rad)关节角度下限
#define MIN_J3_ANGLE (-0.27f) // (rad)关节角度下限

#define MIN_LEG_LENGTH       ( 0.16f)
#define MIN_LEG_ANGLE        ( - MAX_DELTA_ROD_ANGLE)
#define MIN_TAIL_ANGLE       ( -0.24f)
#define MIN_SPEED            (-MAX_SPEED)
#define MIN_SPEED_VECTOR_VX  (-MAX_SPEED_VECTOR_VX)
#define MIN_SPEED_VECTOR_VY  (-MAX_SPEED_VECTOR_VY)
#define MIN_SPEED_VECTOR_WZ  (-MAX_SPEED_VECTOR_WZ)

#define MIN_JOINT_TORQUE      (-MAX_JOINT_TORQUE)  // 
#define MIN_JOINT_TORQUE_JUMP (-MAX_JOINT_TORQUE_JUMP)  // 
#define MIN_VEL_ADD           (-MAX_VEL_ADD)    // (m/s)速度增量下限
#define MIN_PITCH_VEL         (-MAX_PITCH_VEL)  // (rad/s)pitch轴速度下限

//PID parameters ---------------------
//yaw轴跟踪角度环PID参数
#define KP_CHASSIS_YAW_ANGLE        (31.6227766016838f)
#define KI_CHASSIS_YAW_ANGLE        (0.0f)
#define KD_CHASSIS_YAW_ANGLE        (4.93619232696934f)
#define MAX_IOUT_CHASSIS_YAW_ANGLE  (0.0f)
#define MAX_OUT_CHASSIS_YAW_ANGLE   (3.5f)

// #define KP_CHASSIS_YAW_ANGLE        (2.3f)
// #define KI_CHASSIS_YAW_ANGLE        (1.0f)
// #define KD_CHASSIS_YAW_ANGLE        (0.0f)
// #define MAX_IOUT_CHASSIS_YAW_ANGLE  (0.5f)
// #define MAX_OUT_CHASSIS_YAW_ANGLE   (5.0f)

//yaw轴跟踪速度环PID参数
// #define KP_CHASSIS_YAW_VELOCITY        (1.0f)
// #define KI_CHASSIS_YAW_VELOCITY        (0.01f)
// #define KD_CHASSIS_YAW_VELOCITY        (0.02f)
// #define MAX_IOUT_CHASSIS_YAW_VELOCITY  (0.1f)
// #define MAX_OUT_CHASSIS_YAW_VELOCITY   (2.0f)

#define KP_CHASSIS_YAW_VELOCITY        (3.0f)
#define KI_CHASSIS_YAW_VELOCITY        (0.5f)
#define KD_CHASSIS_YAW_VELOCITY        (0.1f)
#define MAX_IOUT_CHASSIS_YAW_VELOCITY  (0.5f)
#define MAX_OUT_CHASSIS_YAW_VELOCITY   (2.0f)

// vel_add PID参数
#define KP_CHASSIS_VEL_ADD        (0.1f) //0.1
#define KI_CHASSIS_VEL_ADD        (0.005f)//0.005
#define KD_CHASSIS_VEL_ADD        (0.001f)//0.001
#define MAX_IOUT_CHASSIS_VEL_ADD  (0.8f)//0.5
#define MAX_OUT_CHASSIS_VEL_ADD   (1.5f)//1.0

/*========== Start of locomotion control pid ==========*/

//roll轴跟踪角度环PID参数
#define KP_CHASSIS_ROLL_ANGLE        (0.0f)
#define KI_CHASSIS_ROLL_ANGLE        (0.0f)
#define KD_CHASSIS_ROLL_ANGLE        (0.0f)
#define MAX_IOUT_CHASSIS_ROLL_ANGLE  (0.0f)
#define MAX_OUT_CHASSIS_ROLL_ANGLE   (20.0f)
#define N_CHASSIS_ROLL_ANGLE         (0.1f)

// 两腿一致PID参数
#define KP_CHASSIS_LEG_COOR        (50.0f)
#define KI_CHASSIS_LEG_COOR        (0.0f)
#define KD_CHASSIS_LEG_COOR        (25.0f)
#define MAX_IOUT_CHASSIS_LEG_COOR  (1.0f)
#define MAX_OUT_CHASSIS_LEG_COOR   (6.0f)
#define N_CHASSIS_LEG_COOR         (0.0f)
#define ERRORSUM_UP_LEG_COOR       (0.20f)
#define ERRORSUM_LOW_LEG_COOR      (-ERRORSUM_UP_LEG_COOR)

#define TAIL_F_ff_RATIO 0.03f

// 尾巴适应地形补偿PID参数 Bipedal用来防止爬坡卡死的
#define KP_CHASSIS_TAIL_COMP        (0.15f)
#define KI_CHASSIS_TAIL_COMP        (0.0f)
#define KD_CHASSIS_TAIL_COMP        (0.01f)
#define MAX_IOUT_CHASSIS_TAIL_COMP  (0.0f)
#define MAX_OUT_CHASSIS_TAIL_COMP   (0.08f)
#define N_CHASSIS_TAIL_COMP         (0.0f)

// 尾巴抬升补偿PID参数 Tripod pitch调节
#define KP_CHASSIS_TAIL_UP        (0.08f)
#define KI_CHASSIS_TAIL_UP        (0.0f)
#define KD_CHASSIS_TAIL_UP        (0.01f)
#define MAX_IOUT_CHASSIS_TAIL_UP  (0.0f)
#define MAX_OUT_CHASSIS_TAIL_UP   (0.12f)
#define N_CHASSIS_TAIL_UP         (0.1f)

// 尾巴末端轮子离地高度PID参数 Tripod 竖直虚拟力
#define KP_CHASSIS_TAIL_Z        (25.0f)
#define KI_CHASSIS_TAIL_Z        (0.0f)
#define KD_CHASSIS_TAIL_Z        (3.0f)
#define MAX_IOUT_CHASSIS_TAIL_Z  (0.0f)
#define MAX_OUT_CHASSIS_TAIL_Z   (5.0f)
#define N_CHASSIS_TAIL_Z         (0.1f)

// 尾巴末端轮子离地高度PID参数
#define KP_CHASSIS_LEG_T        (3.0f)
#define KI_CHASSIS_LEG_T        (0.0f)
#define KD_CHASSIS_LEG_T        (0.01f)
#define MAX_IOUT_CHASSIS_LEG_T  (0.0f)
#define MAX_OUT_CHASSIS_LEG_T   (15.0f)
#define N_CHASSIS_LEG_T         (0.1f)

// 轮 调pitch_dot的补偿扭矩
#define KP_CHASSIS_PITCH_DOT        (30.0f)
#define KI_CHASSIS_PITCH_DOT        (0.0f)
#define KD_CHASSIS_PITCH_DOT        (2.0f)
#define MAX_IOUT_CHASSIS_PITCH_DOT  (0.0f)
#define MAX_OUT_CHASSIS_PITCH_DOT   (5.0f)
#define N_CHASSIS_PITCH_DOT         (0.1f)

// 尾 调x的补偿扭矩
#define KP_CHASSIS_TAIL_X        (0.05f)
#define KI_CHASSIS_TAIL_X        (0.0f)
#define KD_CHASSIS_TAIL_X        (0.001f)
#define MAX_IOUT_CHASSIS_TAIL_X  (0.0f)
#define MAX_OUT_CHASSIS_TAIL_X   (1.0f)
#define N_CHASSIS_TAIL_X         (0.1f)

// 尾 调x的补偿扭矩
#define KP_CHASSIS_TAIL_TORQUE        (0.0f)
#define KI_CHASSIS_TAIL_TORQUE        (0.0f)
#define KD_CHASSIS_TAIL_TORQUE        (0.0f)
#define MAX_IOUT_CHASSIS_TAIL_TORQUE  (0.0f)
#define MAX_OUT_CHASSIS_TAIL_TORQUE   (5.0f)
#define N_CHASSIS_TAIL_TORQUE         (0.1f)

// ================= 台阶检测 / 台阶抬升补偿 =================
#define STAIR_DETECT_WINDOW_S        (0.20f)   // 判定时间窗：0.20s
#define STAIR_HOLD_TIME_S            (0.15f)   // 触发后保持时间：0.15s

#define STAIR_VX_CMD_MIN             (0.10f)   // 至少有这么大的前进速度命令才考虑“卡住”
#define STAIR_DX_REF_MIN             (0.010f)  // 时间窗内期望位移至少增加 10 mm
#define STAIR_DX_FDB_MIN             (0.002f)  // 时间窗内实际位移小于 2 mm 认为几乎没动
#define STAIR_DX_FDB_BACKWARD        (-0.001f) // 时间窗内实际位移小于 -1 mm 认为反向了

#define TAIL_UP_X_DB                 (0.005f)  // 水平距离死区：5 mm
#define TAIL_UP_X_KP                 (0.5f)   // 水平误差 -> 竖直补偿 比例
#define TAIL_UP_X_MAX                (0.1f)  // 最大额外抬升补偿：25 mm
#define TAIL_UP_X_ALPHA              (0.10f)   // 一阶低通系数

//roll轴跟踪速度环PID参数
// #define KP_CHASSIS_ROLL_VELOCITY        (0.6f)
// #define KI_CHASSIS_ROLL_VELOCITY        (0.0001f)
// #define KD_CHASSIS_ROLL_VELOCITY        (0.005f)
// #define MAX_IOUT_CHASSIS_ROLL_VELOCITY  (0.01f)
// #define MAX_OUT_CHASSIS_ROLL_VELOCITY   (0.1f)

// 腿长跟踪长度环PID参数（左腿 leg0）
#define KP_CHASSIS_LEG_LENGTH_LENGTH_L        (600.0f)
#define KI_CHASSIS_LEG_LENGTH_LENGTH_L        (0.7f)
#define KD_CHASSIS_LEG_LENGTH_LENGTH_L        (2000.0f)
#define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH_L  (40.0f)
#define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH_L   (80.0f)
#define N_LEG_LENGTH_LENGTH_L                 (0.1f)
#define ERRORSUM_UP_LEG_LENGTH_L              (0.05f)   // 积分分离上限 5cm (原25cm, 防止积分饱和)
#define ERRORSUM_LOW_LEG_LENGTH_L             (-ERRORSUM_UP_LEG_LENGTH_L)

// 腿长跟踪长度环PID参数（右腿 leg1）
// 左右腿因机械装配差异可独立调参
#define KP_CHASSIS_LEG_LENGTH_LENGTH_R        (600.0f)
#define KI_CHASSIS_LEG_LENGTH_LENGTH_R        (0.7f)
#define KD_CHASSIS_LEG_LENGTH_LENGTH_R        (1200.0f)
#define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH_R  (40.0f)
#define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH_R   (80.0f)
#define N_LEG_LENGTH_LENGTH_R                 (0.1f)
#define ERRORSUM_UP_LEG_LENGTH_R              (0.05f)   // 积分分离上限 5cm (原25cm, 防止积分饱和)
#define ERRORSUM_LOW_LEG_LENGTH_R             (-ERRORSUM_UP_LEG_LENGTH_R)

// roll死区保护 —— 角度误差小于阈值时不产生腿长差，避免微小IMU噪声被高增益PID放大为力指令振荡
#define CHASSIS_ROLL_DEADBAND                 (0.015f)  // rad, ~0.86°

// roll角速度阻尼 —— roll_dot > 0 时右腿伸长/左腿缩短以抵抗倾斜趋势
// roll_dot ≈ 1 rad/s 时产生 0.05m 腿长差补偿 (原0.01→0.05: 阻尼比ζ从~0.1提升到~0.5)
// 调参: 仍有等幅振荡→增大(0.07); 腿长高频抖动→减小(0.03)
#define ROLL_DOT_DAMPING_GAIN                 (0.05f)  // m/(rad/s)

// theta补偿PID参数
#define KP_CHASSIS_THETA_COMP        (0.0f)
#define KI_CHASSIS_THETA_COMP        (0.02f)
#define KD_CHASSIS_THETA_COMP        (0.0f)
#define MAX_IOUT_CHASSIS_THETA_COMP  (3.0f)
#define MAX_OUT_CHASSIS_THETA_COMP   (3.0f)
#define N_CHASSIS_THETA_COMP                 (0.1f)
#define ERRORSUM_UP_THETA_COMP              (0.3f)
#define ERRORSUM_LOW_THETA_COMP             (-ERRORSUM_UP_LEG_LENGTH_L)

// 腿长跟踪长度环PID参数
// #define KP_CHASSIS_LEG_LENGTH_LENGTH        (150.0f)
// #define KI_CHASSIS_LEG_LENGTH_LENGTH        (0.0f)
// #define KD_CHASSIS_LEG_LENGTH_LENGTH        (1500.0f)
// #define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH  (0.0f)
// #define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH   (40.0f)
// #define N_LEG_LENGTH_LENGTH                 (0.1f)

// 腿长跟踪速度环PID参数
// #define KP_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define KI_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define KD_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define MAX_OUT_CHASSIS_LEG_LENGTH_SPEED 0.0f

/*========== End of locomotion control pid ==========*/

// 起立用的pid
#define KP_CHASSIS_STAND_UP       (2000.0f)
#define KI_CHASSIS_STAND_UP       (0.0f)
#define KD_CHASSIS_STAND_UP       (10.0f)
#define MAX_IOUT_CHASSIS_STAND_UP (0.0f)
#define MAX_OUT_CHASSIS_STAND_UP  (2000.0f)

// 轮子停止用的pid
#define KP_CHASSIS_WHEEL_STOP       (0.3f)
#define KI_CHASSIS_WHEEL_STOP       (0.0f)
#define KD_CHASSIS_WHEEL_STOP       (0.1f)
#define MAX_IOUT_CHASSIS_WHEEL_STOP (0.0f)
#define MAX_OUT_CHASSIS_WHEEL_STOP  (200.0f)

//LPF parameters ---------------------
#define LEG_DDL0_LPF_ALPHA           (0.9f)
#define LEG_DDPHI0_LPF_ALPHA         (0.9f)
#define LEG_DDTHETA_LPF_ALPHA        (0.9f)
#define LEG_SUPPORT_FORCE_LPF_ALPHA  (0.9f)
#define CHASSIS_ROLL_ALPHA           (0.90f)  // roll低通滤波系数(原0.5→0.90, 截止频率~16Hz,抑制运动振荡)
#define LEG_DIFF_LPF_ALPHA           (0.92f)  // 腿长差低通滤波系数(截止频率~6Hz,切断正反馈回路)

//offest parameters ---------------------

#define X0_OFFSET (0.0f)   // 目标theta偏移量
#define X1_OFFSET (0.0f)   // 目标theta_dot偏移量
#define X2_OFFSET (0.0f)   // 目标x偏移量
#define X3_OFFSET (0.0f)   // 目标x_dot偏移量
#define X4_OFFSET (0.0f)   // 目标phi偏移量
#define X5_OFFSET (0.0f)   // 目标phi_dot偏移量
#define X6_OFFSET (0.0f)   // 目标beta偏移量
#define X7_OFFSET (0.0f)   // 目标beta_dot偏移量
#define X8_OFFSET (0.0f)   // 目标phi偏移量
#define X9_OFFSET (0.0f)   // 目标phi_dot偏移量
#define X10_OFFSET (0.0f)  // 目标beta偏移量
#define X11_OFFSET (0.0f)  // 目标beta_dot偏移量

//other parameters ---------------------

// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */
