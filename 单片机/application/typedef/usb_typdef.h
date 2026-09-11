#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

#define DEBUG_PACKAGE_NUM 10

#define DATA_DOMAIN_OFFSET 0x08

// clang-format off
#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

#define IMU_DATA_SEND_ID          ((uint8_t)0x02)
#define PID_DEBUG_SEND_ID         ((uint8_t)0x05)
#define ROBOT_MOTION_DATA_SEND_ID ((uint8_t)0x08)
#define ROBOT_TARGET_DATA_SEND_ID ((uint8_t)0x0B)
#define TORQUE_DATA_SEND_ID       ((uint8_t)0x0C)
#define MOTOR_ERROR_SEND_ID       ((uint8_t)0x0D)
#define MOTOR_DATA_SEND_ID        ((uint8_t)0x0E)
#define RL_STATE_DATA_SEND_ID     ((uint8_t)0x0F)
#define SYSTEM_IDENTIFICATION_12_DATA_SEND_ID ((uint8_t)0x10)
#define SYSTEM_IDENTIFICATION_18_DATA_SEND_ID ((uint8_t)0x11)
#define SYSTEM_IDENTIFICATION_REFERENCE_SEND_ID ((uint8_t)0x12)

#define SYS_ID_PROTOCOL_VERSION         ((uint8_t)0x01)
#define SYS_ID_STATUS_MODE_VALID        ((uint8_t)0x01)
#define SYS_ID_STATUS_EXCITATION_ACTIVE ((uint8_t)0x02)
#define SYS_ID_STATUS_SWEEP_FINISHED    ((uint8_t)0x04)
#define SYS_ID_STATUS_MOTOR_OFFLINE     ((uint8_t)0x08)
#define SYS_ID_STATUS_LEFT_JACOBIAN_BAD ((uint8_t)0x10)
#define SYS_ID_STATUS_RIGHT_JACOBIAN_BAD ((uint8_t)0x20)
#define SYS_ID_STATUS_ACTUAL_INPUT_VALID ((uint8_t)0x40)
#define SYS_ID_STATUS_REFERENCE_VALID   ((uint8_t)0x80)

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)
#define VIRTUAL_RC_DATA_RECEIVE_ID ((uint8_t)0x03)
#define RL_LOW_CMD_DATA_RECEIVE_ID ((uint8_t)0x04)
// clang-format on

typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/

// IMU 数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad（原始值）

        float roll_smooth; // rad（IMU Kalman平滑值）

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s

        // float x_accel;  // m/s^2
        // float y_accel;  // m/s^2
        // float z_accel;  // m/s^2
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    struct
    {
        float fdb;
        float ref;
        float pid_out;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataPidDebug_s;

// 机器人运动数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x08
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
        
        struct
        {
            float beta;
            float beta_dot;
        } __packed__ tail_state;

        struct
        {
            float roll;
            float pitch;
            float yaw;
            float x;
        } __packed__ body_state;

        struct
        {
            float phi;
            float phi_dot;
            float legx;
            float legx_dot;
            float theta;
            float theta_dot;
        } __packed__ leg_state[2];
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotMotion_s;

// 机器人目标速度数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0B
    uint32_t time_stamp;

    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
        
        struct
        {
            float beta;
            float beta_dot;
        } __packed__ tail_state;

        struct
        {
            float roll;
            float pitch;
            float yaw;
            float x;
        } __packed__ body_state;

        struct
        {
            float phi;
            float phi_dot;
            float legx;
            float legx_dot;
            float theta;
            float theta_dot;
        } __packed__ leg_state[2];
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotTarget_s;

// 控制力矩数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0C
    uint32_t time_stamp;
    struct
    {
        float T_r_to_b;    // N*m 右髋力矩
        float T_l_to_b;    // N*m 左髋力矩
        float T_wr_to_r;   // N*m 右轮力矩
        float T_wl_to_l;   // N*m 左轮力矩
        float T_t_to_b;    // N*m 尾电机力矩
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataTorque_s;

// 电机离线错误数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0D
    uint32_t time_stamp;
    uint8_t  offline_mask;       // bit0~3: J0~J3关节, bit4~5: W0~W1轮子, bit6: 尾巴
    uint8_t  reserved[3];
    uint16_t crc;
} __packed__ SendDataMotorError_s;

// 电机数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0E
    uint32_t time_stamp;
    struct
    {
        float position[7];  // rad [0..3]=J0..J3, [4..5]=W0..W1, [6]=Tail
        float torque[7];    // N*m 减速器输出轴反馈力矩
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataMotor_s;

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    uint16_t sequence;
    struct
    {
        float quat_wxyz[4];
        float rpy[3];
        float gyro_body[3];
        float vx_long;
        float height;
        float heightdot;
        float joint_pos[7];
        float joint_vel[7];
        float remote_vx;
        float remote_wz;
        uint8_t motor_offline_mask;
        uint8_t control_mode;
        uint16_t remote_height_mm;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRlState_s;

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t control_tick_ms;
    uint32_t sequence;
    uint32_t experiment_id;
    uint32_t reference_sequence;
    uint8_t protocol_version;
    uint8_t profile_id;
    uint8_t state_dimension;
    uint8_t input_dimension;
    uint8_t excitation_config;
    uint8_t excitation_input_mask;
    uint8_t status_flags;
    uint8_t motor_saturation_mask;
} __packed__ SystemIdentificationFrameCommon_t;

typedef struct
{
    SystemIdentificationFrameCommon_t common;
    float state_feedback[12];
    float controller_output[5];
    float excitation[5];
    float generalized_command[5];
    float generalized_actual_torque[5];
    float motor_feedback_torque[7];  // N*m 减速器输出轴反馈力矩
    uint16_t crc;
} __packed__ SendDataSystemIdentification12_s;

typedef struct
{
    SystemIdentificationFrameCommon_t common;
    float state_feedback[18];
    float controller_output[7];
    float excitation[7];
    float generalized_command[7];
    float generalized_actual_torque[7];
    float motor_feedback_torque[7];  // N*m 减速器输出轴反馈力矩
    uint16_t crc;
} __packed__ SendDataSystemIdentification18_s;

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t experiment_id;
    uint32_t reference_sequence;
    uint8_t protocol_version;
    uint8_t profile_id;
    uint8_t state_dimension;
    uint8_t input_dimension;
    uint8_t excitation_config;
    uint8_t excitation_input_mask;
    uint8_t reserved[2];
    float state_reference[18];
    float equilibrium_input[7];
    uint16_t crc;
} __packed__ SendDataSystemIdentificationReference_s;

// 帧头len只有8位；若未来扩展字段导致payload超过255字节，编译必须失败。
typedef char SysId12PayloadMustFitUint8[
    ((sizeof(SendDataSystemIdentification12_s) - 6U) <= 255U) ? 1 : -1];
typedef char SysId18PayloadMustFitUint8[
    ((sizeof(SendDataSystemIdentification18_s) - 6U) <= 255U) ? 1 : -1];
typedef char SysIdReferencePayloadMustFitUint8[
    ((sizeof(SendDataSystemIdentificationReference_s) - 6U) <= 255U) ? 1 : -1];

/*-------------------- Receive --------------------*/
// 机器人控制指令数据包
typedef struct RobotCmdData
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
        struct
        {
            float roll;
            float pitch;
            float yaw;
            float leg_lenth;
        } __packed__ chassis;
    } __packed__ data;
    uint16_t checksum;
} __packed__ ReceiveDataRobotCmd_s;

// 虚拟遥控器数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    RC_ctrl_t data;
    uint16_t crc;
} __packed__ ReceiveDataVirtualRc_s;

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    uint16_t sequence;
    uint8_t enable;
    uint8_t reserved;
    struct
    {
        float q_des[7];
        float dq_des[7];
        float kp[7];
        float kd[7];
        float tau_ff[7];
        float q_fdb_offset[7];
    } __packed__ data;
    uint16_t crc;
} __packed__ ReceiveDataRlLowCmd_s;
#endif  // USB_TYPEDEF_H
