/**
  * @file       system_identification_param.h
  * @brief      系统辨识工作点与激励信号参数
  */

#ifndef SYSTEM_IDENTIFICATION_PARAM_H
#define SYSTEM_IDENTIFICATION_PARAM_H

/*-------------------- Fixed operating point --------------------*/
#define SYS_ID_LEG_LENGTH               (0.18f)  // (m)

// 12维LQR状态目标量，单位与对应状态一致；默认工作点全部为0。
// 腿角目标会在控制器中叠加模型计算得到的theta_eq几何偏置。
#define SYS_ID_REF_BODY_X               (-0.5f)  // (m)
#define SYS_ID_REF_BODY_X_DOT           (0.0f)   // (m/s)
#define SYS_ID_REF_BODY_YAW             (0.0f)   // (rad)
#define SYS_ID_REF_BODY_YAW_DOT         (0.0f)   // (rad/s)
#define SYS_ID_REF_LEFT_LEG_THETA       (0.0f)   // (rad)
#define SYS_ID_REF_LEFT_LEG_THETA_DOT   (0.0f)   // (rad/s)
#define SYS_ID_REF_RIGHT_LEG_THETA      (0.0f)   // (rad)
#define SYS_ID_REF_RIGHT_LEG_THETA_DOT  (0.0f)   // (rad/s)
#define SYS_ID_REF_BODY_PITCH           (0.0f)   // (rad)
#define SYS_ID_REF_BODY_PITCH_DOT       (0.0f)   // (rad/s)
#define SYS_ID_REF_TAIL_BETA            (0.0f)   // (rad)
#define SYS_ID_REF_TAIL_BETA_DOT        (0.0f)   // (rad/s)
#define SYS_ID_REF_BODY_ROLL            (0.0f)   // (rad)

/*-------------------- Excitation selection --------------------*/
#define SYS_ID_EXCITATION_DISABLED      0U
#define SYS_ID_EXCITATION_SINE_SWEEP    1U
#define SYS_ID_EXCITATION_PRBS          2U

// 通过修改此项选择激励类型；默认关闭激励，避免误操作导致机器人运动。
#define SYS_ID_EXCITATION_ENABLE        0U
#define SYS_ID_EXCITATION_MODE          SYS_ID_EXCITATION_SINE_SWEEP

// bit0~4: [右髋, 左髋, 右轮, 左轮, 尾巴]
#define SYS_ID_EXCITATION_CHANNEL_MASK  0x1FU

/*-------------------- Sine sweep --------------------*/
#define SYS_ID_SINE_SWEEP_START_FREQ   (0.2f)  // (Hz)
#define SYS_ID_SINE_SWEEP_END_FREQ     (20.0f) // (Hz)
#define SYS_ID_SINE_SWEEP_DURATION     (60.0f) // (s)

/*-------------------- PRBS --------------------*/
#define SYS_ID_PRBS_BIT_HOLD_SAMPLES   (10U)   // 10 * 2ms = 20ms
#define SYS_ID_PRBS_SEED_RIGHT_HIP     (0xACE1U)
#define SYS_ID_PRBS_SEED_LEFT_HIP      (0x1D0FU)
#define SYS_ID_PRBS_SEED_RIGHT_WHEEL   (0xBEEF)
#define SYS_ID_PRBS_SEED_LEFT_WHEEL    (0x5A5AU)
#define SYS_ID_PRBS_SEED_TAIL          (0x1357U)

/*-------------------- Per-channel amplitude --------------------*/
// 单位为N*m，激励叠加在对应的5维广义力矩上。
#define SYS_ID_EXCITATION_AMP_RIGHT_HIP   (0.2f)
#define SYS_ID_EXCITATION_AMP_LEFT_HIP    (0.2f)
#define SYS_ID_EXCITATION_AMP_RIGHT_WHEEL (0.1f)
#define SYS_ID_EXCITATION_AMP_LEFT_WHEEL  (0.1f)
#define SYS_ID_EXCITATION_AMP_TAIL        (0.1f)

#endif  // SYSTEM_IDENTIFICATION_PARAM_H
