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
#define ROBOT_STATUS_SEND_ID      ((uint8_t)0x0B)
#define SOLVED_RC_CMD_SEND_ID     ((uint8_t)0x0C)

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)
#define VIRTUAL_RC_DATA_RECEIVE_ID ((uint8_t)0x03)
// clang-format on

typedef struct
{
    uint8_t sof;
    uint8_t len;
    uint8_t id;
    uint8_t crc;
} __packed__ FrameHeader_t;

/*-------------------- Send --------------------*/
typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    struct
    {
        float yaw;
        float pitch;
        float roll;
        float yaw_vel;
        float pitch_vel;
        float roll_vel;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotMotion_s;

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    struct
    {
        uint8_t mode;
        uint8_t step;
        uint8_t rc_offline;
        uint8_t reserved;
        float vx;
        float vy;
        float wz;
        float roll;
        float pitch;
        float yaw;
        float leg_length_l;
        float leg_length_r;
        float leg_angle_l;
        float leg_angle_r;
        float tail_beta;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataSolvedRcCmd_s;

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    struct
    {
        float robot_pos_x;
        float robot_pos_y;
        float robot_pos_angle;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotStatus_s;

/*-------------------- Receive --------------------*/
typedef struct RobotCmdData
{
    FrameHeader_t frame_header;
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

typedef struct
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;
    RC_ctrl_t data;
    uint16_t crc;
} __packed__ ReceiveDataVirtualRc_s;

#endif  // USB_TYPEDEF_H

