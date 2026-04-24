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
#include "data_exchange.h"
#include "macro_typedef.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "supervisory_computer_cmd.h"
#include "IMU.h"
#include "chassis_balance.h"

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
#define SEND_DURATION_RobotTarget 10  // ms

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


// 判断USB连接状态用到的一些变量
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
// clang-format off
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
static SendDataRobotTarget_s SEND_ROBOT_TARGET_DATA;

// clang-format on

// 文件级静态缓冲，避免在USB任务栈上分配大体积快照结构
static Fdb_t USB_FDB_SNAPSHOT;
static Ref_t USB_REF_SNAPSHOT;

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
    uint32_t RobotTarget;
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
static void UsbSendRobotTargetData(void);

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
    MX_USB_DEVICE_Init();

    vTaskDelay(10);  //等待USB设备初始化完成
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();
        GetCmdData();
        GetVirtualRcCtrlData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT <= USB_CONNECT_CNT) {
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
    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/    
    // 2.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    
    // 8.初始化机器人运动数据
    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));

    // 11.初始化机器人目标速度数据
    SEND_ROBOT_TARGET_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_TARGET_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotTarget_s) - 6);
    SEND_ROBOT_TARGET_DATA.frame_header.id = ROBOT_TARGET_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_TARGET_DATA.frame_header),
        sizeof(SEND_ROBOT_TARGET_DATA.frame_header));
}   

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    uint32_t now = HAL_GetTick();
    bool send_imu = ((now - LAST_SEND_TIME.Imu) >= SEND_DURATION_Imu);
    bool send_motion = ((now - LAST_SEND_TIME.RobotMotion) >= SEND_DURATION_RobotMotion);
    bool send_target = ((now - LAST_SEND_TIME.RobotTarget) >= SEND_DURATION_RobotTarget);

    if (send_imu) {
        LAST_SEND_TIME.Imu = now;
        UsbSendImuData();
    }

    // 运动和目标包共用同一帧快照，避免同一轮采样时间不一致
    if (send_motion || send_target) {
        if (ChassisSnapshotRead(
                &USB_FDB_SNAPSHOT, sizeof(Fdb_t), &USB_REF_SNAPSHOT, sizeof(Ref_t), NULL) !=
            CHASSIS_SNAPSHOT_OK) {
            return;
        }
    }

    if (send_motion) {
        LAST_SEND_TIME.RobotMotion = now;
        UsbSendRobotMotionData();
    }
    if (send_target) {
        LAST_SEND_TIME.RobotTarget = now;
        UsbSendRobotTargetData();
    }
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;  // 接收数据包时存放于缓存区的起始位置
    static uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    uint8_t * sof_address = USB_RX_BUF;
    uint8_t * rx_buffer_limit;
    uint16_t valid_len;

    // 读取数据，len 由底层返回本次可处理长度
    len = USB_RECEIVE_LEN;
    USB_Receive(rx_data_start_address, &len);

    valid_len = (len > USB_RECEIVE_LEN) ? USB_RECEIVE_LEN : (uint16_t)len;
    if (valid_len == 0) {
        return;
    }

    rx_buffer_limit = rx_data_start_address + valid_len;
    rx_data_end_address = rx_buffer_limit - 1;

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while ((sof_address <= rx_data_end_address) && (*(sof_address) != RECEIVE_SOF)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }

        // 剩余长度不足以构成帧头，留到下一轮
        if ((uint16_t)(rx_data_end_address - sof_address + 1) < HEADER_SIZE) {
            break;
        }

        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            uint16_t frame_len = (uint16_t)(HEADER_SIZE + data_len + 2);

            // 剩余长度不足整帧，留到下一轮
            if ((uint16_t)(rx_data_end_address - sof_address + 1) < frame_len) {
                break;
            }

            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, frame_len);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        memcpy(
                            &RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
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
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }

    // 更新下一次接收数据的起始位置
    if (sof_address >= rx_buffer_limit) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = (uint16_t)(rx_buffer_limit - sof_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
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
    Imu_t imu_data = {0};
    // 从IMU双缓冲读取最新完整帧
    if (ImuSnapshotRead(&imu_data) != CHASSIS_SNAPSHOT_OK) {
        return;
    }

    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    SEND_DATA_IMU.data.yaw = imu_data.angle[AX_Z];
    SEND_DATA_IMU.data.pitch = imu_data.angle[AX_Y];
    SEND_DATA_IMU.data.roll = imu_data.angle[AX_X];

    SEND_DATA_IMU.data.yaw_vel = imu_data.gyro[AX_Z];
    SEND_DATA_IMU.data.pitch_vel = imu_data.gyro[AX_Y];
    SEND_DATA_IMU.data.roll_vel = imu_data.gyro[AX_X];

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 发送机器人运动数据
 * @param duration 发送周期
 */
static void UsbSendRobotMotionData(void)
{
    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = USB_FDB_SNAPSHOT.speed_vector.vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = USB_FDB_SNAPSHOT.speed_vector.vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = USB_FDB_SNAPSHOT.speed_vector.wz;

    SEND_ROBOT_MOTION_DATA.data.tail_state.beta = USB_FDB_SNAPSHOT.tail_state.beta;
    SEND_ROBOT_MOTION_DATA.data.tail_state.beta_dot = USB_FDB_SNAPSHOT.tail_state.beta_dot;

    SEND_ROBOT_MOTION_DATA.data.body_state.roll = USB_FDB_SNAPSHOT.body.roll;
    SEND_ROBOT_MOTION_DATA.data.body_state.pitch = USB_FDB_SNAPSHOT.body.pitch;
    SEND_ROBOT_MOTION_DATA.data.body_state.yaw = USB_FDB_SNAPSHOT.body.yaw;
    SEND_ROBOT_MOTION_DATA.data.body_state.x = USB_FDB_SNAPSHOT.body.x;

    SEND_ROBOT_MOTION_DATA.data.leg_state[0].phi = USB_FDB_SNAPSHOT.leg_state[0].phi;
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].phi_dot = USB_FDB_SNAPSHOT.leg_state[0].phi_dot;
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].legx = USB_FDB_SNAPSHOT.leg_state[0].x;
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].legx_dot = USB_FDB_SNAPSHOT.leg_state[0].x_dot;
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].theta = USB_FDB_SNAPSHOT.leg_state[0].theta;
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].theta_dot = USB_FDB_SNAPSHOT.leg_state[0].theta_dot;

    SEND_ROBOT_MOTION_DATA.data.leg_state[1].phi = USB_FDB_SNAPSHOT.leg_state[1].phi;
    SEND_ROBOT_MOTION_DATA.data.leg_state[1].phi_dot = USB_FDB_SNAPSHOT.leg_state[1].phi_dot;
    SEND_ROBOT_MOTION_DATA.data.leg_state[1].legx = USB_FDB_SNAPSHOT.leg_state[1].x;
    SEND_ROBOT_MOTION_DATA.data.leg_state[1].legx_dot = USB_FDB_SNAPSHOT.leg_state[1].x_dot;
    SEND_ROBOT_MOTION_DATA.data.leg_state[1].theta = USB_FDB_SNAPSHOT.leg_state[1].theta;
    SEND_ROBOT_MOTION_DATA.data.leg_state[1].theta_dot = USB_FDB_SNAPSHOT.leg_state[1].theta_dot;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}

/**
 * @brief 发送机器人目标速度数据
 * @param duration 发送周期
 */
static void UsbSendRobotTargetData(void)
{
    SEND_ROBOT_TARGET_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_TARGET_DATA.data.speed_vector.vx = USB_REF_SNAPSHOT.speed_vector.vx;
    SEND_ROBOT_TARGET_DATA.data.speed_vector.vy = USB_REF_SNAPSHOT.speed_vector.vy;
    SEND_ROBOT_TARGET_DATA.data.speed_vector.wz = USB_REF_SNAPSHOT.speed_vector.wz;

    SEND_ROBOT_TARGET_DATA.data.tail_state.beta = USB_REF_SNAPSHOT.tail_state.beta;
    SEND_ROBOT_TARGET_DATA.data.tail_state.beta_dot = USB_REF_SNAPSHOT.tail_state.beta_dot;

    SEND_ROBOT_TARGET_DATA.data.body_state.roll = USB_REF_SNAPSHOT.body.roll;
    SEND_ROBOT_TARGET_DATA.data.body_state.pitch = USB_REF_SNAPSHOT.body.pitch;
    SEND_ROBOT_TARGET_DATA.data.body_state.yaw = USB_REF_SNAPSHOT.body.yaw;
    SEND_ROBOT_TARGET_DATA.data.body_state.x = USB_REF_SNAPSHOT.body.x;

    SEND_ROBOT_TARGET_DATA.data.leg_state[0].phi = USB_REF_SNAPSHOT.leg_state[0].phi;
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].phi_dot = USB_REF_SNAPSHOT.leg_state[0].phi_dot;
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].legx = USB_REF_SNAPSHOT.leg_state[0].x;
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].legx_dot = USB_REF_SNAPSHOT.leg_state[0].x_dot;
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].theta = USB_REF_SNAPSHOT.leg_state[0].theta;
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].theta_dot = USB_REF_SNAPSHOT.leg_state[0].theta_dot;

    SEND_ROBOT_TARGET_DATA.data.leg_state[1].phi = USB_REF_SNAPSHOT.leg_state[1].phi;
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].phi_dot = USB_REF_SNAPSHOT.leg_state[1].phi_dot;
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].legx = USB_REF_SNAPSHOT.leg_state[1].x;
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].legx_dot = USB_REF_SNAPSHOT.leg_state[1].x_dot;
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].theta = USB_REF_SNAPSHOT.leg_state[1].theta;
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].theta_dot = USB_REF_SNAPSHOT.leg_state[1].theta_dot;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_TARGET_DATA, sizeof(SendDataRobotTarget_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_TARGET_DATA, sizeof(SendDataRobotTarget_s));
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
