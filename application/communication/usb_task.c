/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "chassis_balance.h"
#include "data_exchange.h"
#include "macro_typedef.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "supervisory_computer_cmd.h"
#include "IMU.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

// clang-format off

#define SEND_DURATION_Imu         5   // ms
#define SEND_DURATION_RobotStateInfo   10  // ms
#define SEND_DURATION_RobotMotion 10  // ms
#define SEND_DURATION_RobotStatus  10// ms
#define SEND_DURATION_SolvedRcCmd 10  // ms

// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.send_name) >= SEND_DURATION_##send_name) {   \
            LAST_SEND_TIME.send_name = HAL_GetTick();                                    \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
// clang-format off
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
static SendDataSolvedRcCmd_s SEND_DATA_SOLVED_RC_CMD;

// clang-format on

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

// 发送数据间隔时间
typedef struct
{
    uint32_t Imu;
    uint32_t RobotMotion;
    uint32_t SolvedRcCmd;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/
static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/
static void UsbSendImuData(void);
static void UsbSendRobotMotionData(void);
static void UsbSendSolvedRcCmdData(void);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/
static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);

/******************************************************************/
/* Task                                                           */
/******************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    Publish(&ROBOT_CMD_DATA, ROBOT_CMD_DATA_NAME);
    Publish(&USB_OFFLINE, USB_OFFLINE_NAME);
    Publish(&VIRTUAL_RC_CTRL, VIRTUAL_RC_NAME);

    MX_USB_DEVICE_Init();

    vTaskDelay(10);
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();
        GetCmdData();
        GetVirtualRcCtrlData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    IMU = Subscribe(IMU_NAME);
    FDB_SPEED_VECTOR = Subscribe(CHASSIS_FDB_SPEED_NAME);

    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));

    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));

    SEND_DATA_SOLVED_RC_CMD.frame_header.sof = SEND_SOF;
    SEND_DATA_SOLVED_RC_CMD.frame_header.len = (uint8_t)(sizeof(SendDataSolvedRcCmd_s) - 6);
    SEND_DATA_SOLVED_RC_CMD.frame_header.id = SOLVED_RC_CMD_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_SOLVED_RC_CMD.frame_header),
        sizeof(SEND_DATA_SOLVED_RC_CMD.frame_header));
}

/**
 * @brief      通过USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    CheckDurationAndSend(Imu);
    CheckDurationAndSend(RobotMotion);
    CheckDurationAndSend(SolvedRcCmd);
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t *rx_data_start_address = USB_RX_BUF;
    static uint8_t *rx_data_end_address = NULL;
    uint8_t *sof_address = USB_RX_BUF;
    uint8_t *rx_buffer_limit;
    uint16_t valid_len;

    len = USB_RECEIVE_LEN;
    USB_Receive(rx_data_start_address, &len);

    valid_len = (len > USB_RECEIVE_LEN) ? USB_RECEIVE_LEN : (uint16_t)len;
    if (valid_len == 0U) {
        return;
    }

    rx_buffer_limit = rx_data_start_address + valid_len;
    rx_data_end_address = rx_buffer_limit - 1;

    while (sof_address <= rx_data_end_address) {
        while ((sof_address <= rx_data_end_address) && (*sof_address != RECEIVE_SOF)) {
            sof_address++;
        }

        if (sof_address > rx_data_end_address) {
            break;
        }

        if ((uint16_t)(rx_data_end_address - sof_address + 1) < HEADER_SIZE) {
            break;
        }

        if (verify_CRC8_check_sum(sof_address, HEADER_SIZE)) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            uint16_t frame_len = (uint16_t)(HEADER_SIZE + data_len + 2);

            if ((uint16_t)(rx_data_end_address - sof_address + 1) < frame_len) {
                break;
            }

            if (verify_CRC16_check_sum(sof_address, frame_len)) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                    } break;
                    default:
                        break;
                }

                if ((frame_len >= (HEADER_SIZE + sizeof(uint32_t))) &&
                    (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP)) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = HAL_GetTick();
                }
            }
            sof_address += frame_len;
        } else {
            sof_address++;
        }
    }

    if (sof_address >= rx_buffer_limit) {
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = (uint16_t)(rx_buffer_limit - sof_address);
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
static void UsbSendImuData(void)
{
    if (IMU == NULL) {
        return;
    }

    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    SEND_DATA_IMU.data.yaw = IMU->angle[AX_Z];
    SEND_DATA_IMU.data.pitch = IMU->angle[AX_Y];
    SEND_DATA_IMU.data.roll = IMU->angle[AX_X];

    SEND_DATA_IMU.data.yaw_vel = IMU->gyro[AX_Z];
    SEND_DATA_IMU.data.pitch_vel = IMU->gyro[AX_Y];
    SEND_DATA_IMU.data.roll_vel = IMU->gyro[AX_X];

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 发送机器人运动数据
 * @param duration 发送周期
 */
static void UsbSendRobotMotionData(void)
{
    if (FDB_SPEED_VECTOR == NULL) {
        return;
    }

    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}

static void UsbSendSolvedRcCmdData(void)
{
    SEND_DATA_SOLVED_RC_CMD.time_stamp = HAL_GetTick();
    SEND_DATA_SOLVED_RC_CMD.data.mode = (uint8_t)CHASSIS.mode;
    SEND_DATA_SOLVED_RC_CMD.data.step = (uint8_t)CHASSIS.step;
    SEND_DATA_SOLVED_RC_CMD.data.rc_offline = (uint8_t)GetRcOffline();
    SEND_DATA_SOLVED_RC_CMD.data.reserved = 0U;
    SEND_DATA_SOLVED_RC_CMD.data.vx = CHASSIS.ref.speed_vector.vx;
    SEND_DATA_SOLVED_RC_CMD.data.vy = CHASSIS.ref.speed_vector.vy;
    SEND_DATA_SOLVED_RC_CMD.data.wz = CHASSIS.ref.speed_vector.wz;
    SEND_DATA_SOLVED_RC_CMD.data.roll = CHASSIS.ref.body.roll;
    SEND_DATA_SOLVED_RC_CMD.data.pitch = CHASSIS.ref.body.pitch;
    SEND_DATA_SOLVED_RC_CMD.data.yaw = CHASSIS.ref.body.yaw;
    SEND_DATA_SOLVED_RC_CMD.data.leg_length_l = CHASSIS.ref.rod_L0[0];
    SEND_DATA_SOLVED_RC_CMD.data.leg_length_r = CHASSIS.ref.rod_L0[1];
    SEND_DATA_SOLVED_RC_CMD.data.leg_angle_l = CHASSIS.ref.rod_Angle[0];
    SEND_DATA_SOLVED_RC_CMD.data.leg_angle_r = CHASSIS.ref.rod_Angle[1];
    SEND_DATA_SOLVED_RC_CMD.data.tail_beta = CHASSIS.ref.tail_state.beta;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_SOLVED_RC_CMD, sizeof(SendDataSolvedRcCmd_s));
    USB_Transmit((uint8_t *)&SEND_DATA_SOLVED_RC_CMD, sizeof(SendDataSolvedRcCmd_s));
}

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;
}

static void GetVirtualRcCtrlData(void)
{
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
inline float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X)
    {
        return ROBOT_CMD_DATA.speed_vector.vx;
    } 
    else if (axis == AX_Y) 
    {
        return ROBOT_CMD_DATA.speed_vector.vy;
    }
    else if (axis == AX_Z)
    {
        return 0;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动角速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (rad/s) 底盘坐标系下axis方向运动角速度
 */
inline float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z)
    {
        return ROBOT_CMD_DATA.speed_vector.wz;
    } 
    return 0.0f;
}


/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
inline float GetScCmdChassisHeight(void)
{
    return ROBOT_CMD_DATA.chassis.leg_length;
}
/*------------------------------ End of File ------------------------------*/
