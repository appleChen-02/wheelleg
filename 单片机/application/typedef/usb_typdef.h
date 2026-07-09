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
#define ROBOT_MOTION_DATA_SEND_ID ((uint8_t)0x08)
#define ROBOT_TARGET_DATA_SEND_ID ((uint8_t)0x0B)
#define TORQUE_DATA_SEND_ID       ((uint8_t)0x0C)
#define SBUS_RAW_DATA_SEND_ID     ((uint8_t)0x0D)

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)
#define VIRTUAL_RC_DATA_RECEIVE_ID ((uint8_t)0x03)
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
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s

        // float x_accel;  // m/s^2
        // float y_accel;  // m/s^2
        // float z_accel;  // m/s^2
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

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

// SBUS 遥控器原始数据包（16通道原始值 + 连接标志，用于量程标定）
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0D
    uint32_t time_stamp;
    struct
    {
        uint16_t ch[16];       // 16个SBUS通道原始值 (11-bit, 0~2047)
        uint8_t  connect_flag; // 连接标志（遥控器型号相关）
        uint8_t  reserved;     // 对齐填充
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataSbusRaw_s;

/*-------------------- Receive --------------------*/
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
#endif  // USB_TYPEDEF_H
