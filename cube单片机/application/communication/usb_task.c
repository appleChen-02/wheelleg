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

#include <math.h>
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
#define SEND_DURATION_Torque      10  // ms
#define SEND_DURATION_MotorData   10  // ms
#define SEND_DURATION_PidDebug    10  // ms
#define SEND_DURATION_RlState     5   // ms

// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 192   // byte; RL low command frame is 182 bytes
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
static SendDataTorque_s     SEND_TORQUE_DATA;
static SendDataPidDebug_s   SEND_PID_DEBUG_DATA;
static SendDataMotorError_s SEND_MOTOR_ERROR_DATA;
static SendDataMotor_s      SEND_MOTOR_DATA;
static SendDataRlState_s    SEND_RL_STATE_DATA;
static SendDataSystemIdentification12_s SEND_SYS_ID_12_DATA;
static SendDataSystemIdentification18_s SEND_SYS_ID_18_DATA;
static SendDataSystemIdentificationReference_s SEND_SYS_ID_REFERENCE_DATA;

// clang-format on

// 文件级静态缓冲，避免在USB任务栈上分配大体积快照结构
static Fdb_t USB_FDB_SNAPSHOT;
static Ref_t USB_REF_SNAPSHOT;
static float USB_SYS_ID_STATE_BUFFER[18];
static float USB_SYS_ID_REFERENCE_BUFFER[18];

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;
static ReceiveDataRlLowCmd_s RECEIVE_RL_LOW_CMD_DATA;
static uint32_t RL_LOW_CMD_RECEIVE_TIME = 0;
static uint16_t RL_LOW_CMD_SEQUENCE = 0;
static bool RL_LOW_CMD_RECEIVED = false;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

// 发送数据间隔时间
typedef struct
{
    uint32_t Imu;
    uint32_t RobotMotion;
    uint32_t RobotTarget;
    uint32_t Torque;
    uint32_t MotorData;
    uint32_t PidDebug;
    uint32_t RlState;
    uint32_t SystemIdentification;
    uint32_t SystemIdentificationReference;
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
static void UsbSendTorqueData(void);
static void UsbSendPidDebugData(void);
static void UsbSendMotorErrorData(void);
static void UsbSendMotorData(void);
static void UsbSendRlStateData(void);
static void UsbSendSystemIdentificationData(void);
static void UsbSendSystemIdentificationReferenceData(void);
static void FillSystemIdentificationCommon(
    SystemIdentificationFrameCommon_t * common, uint8_t state_dimension,
    uint8_t input_dimension);
static void FillSystemIdentificationState(float state[18]);
static void FillSystemIdentificationReference(float reference[18]);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/
static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);
static bool RlLowCmdIsFinite(const ReceiveDataRlLowCmd_s * command);
static bool IsSequenceNewer(uint16_t sequence, uint16_t previous);

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
    memset(&RECEIVE_RL_LOW_CMD_DATA, 0, sizeof(ReceiveDataRlLowCmd_s));
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

    // 12.初始化控制力矩数据包
    SEND_TORQUE_DATA.frame_header.sof = SEND_SOF;
    SEND_TORQUE_DATA.frame_header.len = (uint8_t)(sizeof(SendDataTorque_s) - 6);
    SEND_TORQUE_DATA.frame_header.id = TORQUE_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_TORQUE_DATA.frame_header),
        sizeof(SEND_TORQUE_DATA.frame_header));

    // 13.初始化电机离线错误数据包
    SEND_PID_DEBUG_DATA.frame_header.sof = SEND_SOF;
    SEND_PID_DEBUG_DATA.frame_header.len = (uint8_t)(sizeof(SendDataPidDebug_s) - 6);
    SEND_PID_DEBUG_DATA.frame_header.id = PID_DEBUG_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_PID_DEBUG_DATA.frame_header), sizeof(SEND_PID_DEBUG_DATA.frame_header));

    SEND_MOTOR_ERROR_DATA.frame_header.sof = SEND_SOF;
    SEND_MOTOR_ERROR_DATA.frame_header.len = (uint8_t)(sizeof(SendDataMotorError_s) - 6);
    SEND_MOTOR_ERROR_DATA.frame_header.id = MOTOR_ERROR_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_MOTOR_ERROR_DATA.frame_header),
        sizeof(SEND_MOTOR_ERROR_DATA.frame_header));

    // 14.初始化电机数据包
    SEND_RL_STATE_DATA.frame_header.sof = SEND_SOF;
    SEND_RL_STATE_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRlState_s) - 6);
    SEND_RL_STATE_DATA.frame_header.id = RL_STATE_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_RL_STATE_DATA.frame_header), sizeof(SEND_RL_STATE_DATA.frame_header));

    SEND_MOTOR_DATA.frame_header.sof = SEND_SOF;
    SEND_MOTOR_DATA.frame_header.len = (uint8_t)(sizeof(SendDataMotor_s) - 6);
    SEND_MOTOR_DATA.frame_header.id = MOTOR_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_MOTOR_DATA.frame_header), sizeof(SEND_MOTOR_DATA.frame_header));

    SEND_SYS_ID_12_DATA.common.frame_header.sof = SEND_SOF;
    SEND_SYS_ID_12_DATA.common.frame_header.len =
        (uint8_t)(sizeof(SendDataSystemIdentification12_s) - 6U);
    SEND_SYS_ID_12_DATA.common.frame_header.id = SYSTEM_IDENTIFICATION_12_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)&SEND_SYS_ID_12_DATA.common.frame_header, sizeof(FrameHeader_t));

    SEND_SYS_ID_18_DATA.common.frame_header.sof = SEND_SOF;
    SEND_SYS_ID_18_DATA.common.frame_header.len =
        (uint8_t)(sizeof(SendDataSystemIdentification18_s) - 6U);
    SEND_SYS_ID_18_DATA.common.frame_header.id = SYSTEM_IDENTIFICATION_18_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)&SEND_SYS_ID_18_DATA.common.frame_header, sizeof(FrameHeader_t));

    SEND_SYS_ID_REFERENCE_DATA.frame_header.sof = SEND_SOF;
    SEND_SYS_ID_REFERENCE_DATA.frame_header.len =
        (uint8_t)(sizeof(SendDataSystemIdentificationReference_s) - 6U);
    SEND_SYS_ID_REFERENCE_DATA.frame_header.id = SYSTEM_IDENTIFICATION_REFERENCE_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)&SEND_SYS_ID_REFERENCE_DATA.frame_header, sizeof(FrameHeader_t));
}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    static bool was_system_identification = false;

    // 存在电机离线时，只发送错误包，跳过其他所有数据
    if (g_motor_offline_mask != 0) {
        UsbSendMotorErrorData();
        return;
    }

    uint32_t now = HAL_GetTick();
    const bool system_identification = ChassisIsSystemIdentificationMode();

    if (system_identification) {
        if (!was_system_identification) {
            LAST_SEND_TIME.SystemIdentification = now - SYS_ID_SAMPLE_SEND_PERIOD_MS;
            LAST_SEND_TIME.SystemIdentificationReference =
                now - SYS_ID_REFERENCE_SEND_PERIOD_MS;
        }
        was_system_identification = true;

        if (ChassisSnapshotRead(
                &USB_FDB_SNAPSHOT, sizeof(Fdb_t), &USB_REF_SNAPSHOT, sizeof(Ref_t), NULL) !=
            CHASSIS_SNAPSHOT_OK) {
            return;
        }
        if ((USB_FDB_SNAPSHOT.sys_id_status_flags & SYS_ID_STATUS_MODE_VALID) == 0U) {
            return;
        }
        if ((now - LAST_SEND_TIME.SystemIdentificationReference) >=
            SYS_ID_REFERENCE_SEND_PERIOD_MS) {
            LAST_SEND_TIME.SystemIdentificationReference = now;
            UsbSendSystemIdentificationReferenceData();
        }
        if ((now - LAST_SEND_TIME.SystemIdentification) >= SYS_ID_SAMPLE_SEND_PERIOD_MS) {
            LAST_SEND_TIME.SystemIdentification = now;
            UsbSendSystemIdentificationData();
        }
        return;
    }
    if (was_system_identification) {
        LAST_SEND_TIME.Imu = now;
        LAST_SEND_TIME.RobotMotion = now;
        LAST_SEND_TIME.RobotTarget = now;
        LAST_SEND_TIME.Torque = now;
        LAST_SEND_TIME.MotorData = now;
        LAST_SEND_TIME.PidDebug = now;
        LAST_SEND_TIME.RlState = now;
        was_system_identification = false;
        return;
    }

    bool send_imu = ((now - LAST_SEND_TIME.Imu) >= SEND_DURATION_Imu);
    bool send_motion = ((now - LAST_SEND_TIME.RobotMotion) >= SEND_DURATION_RobotMotion);
    bool send_target = ((now - LAST_SEND_TIME.RobotTarget) >= SEND_DURATION_RobotTarget);
    bool send_torque = ((now - LAST_SEND_TIME.Torque) >= SEND_DURATION_Torque);
    bool send_motor_data = ((now - LAST_SEND_TIME.MotorData) >= SEND_DURATION_MotorData);
    bool send_pid_debug = ((now - LAST_SEND_TIME.PidDebug) >= SEND_DURATION_PidDebug);
    bool send_rl_state = ((now - LAST_SEND_TIME.RlState) >= SEND_DURATION_RlState);

    if (send_imu) {
        LAST_SEND_TIME.Imu = now;
        UsbSendImuData();
    }

    // 运动、目标、力矩和角度包共用同一帧快照，避免同一轮采样时间不一致
    if (send_motion || send_target || send_torque || send_motor_data || send_rl_state) {
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
    if (send_torque) {
        LAST_SEND_TIME.Torque = now;
        UsbSendTorqueData();
    }
    if (send_motor_data) {
        LAST_SEND_TIME.MotorData = now;
        UsbSendMotorData();
    }
    if (send_pid_debug) {
        LAST_SEND_TIME.PidDebug = now;
        UsbSendPidDebugData();
    }
    if (send_rl_state) {
        LAST_SEND_TIME.RlState = now;
        UsbSendRlStateData();
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
    len = USB_RX_DATA_SIZE - (uint32_t)(rx_data_start_address - USB_RX_BUF);
    if (len > USB_RECEIVE_LEN) len = USB_RECEIVE_LEN;
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
                        if (data_len == (sizeof(ReceiveDataRobotCmd_s) - 6)) {
                            memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                        }
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        if (frame_len == sizeof(ReceiveDataVirtualRc_s)) {
                            memcpy(
                                &RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                        }
                    } break;
                    case RL_LOW_CMD_DATA_RECEIVE_ID: {
                        if (frame_len == sizeof(ReceiveDataRlLowCmd_s)) {
                            ReceiveDataRlLowCmd_s command;
                            memcpy(&command, sof_address, sizeof(command));
                            const bool previous_command_timed_out = RL_LOW_CMD_RECEIVED &&
                                ((HAL_GetTick() - RL_LOW_CMD_RECEIVE_TIME) > USB_OFFLINE_THRESHOLD);
                            if (RlLowCmdIsFinite(&command) &&
                                (!RL_LOW_CMD_RECEIVED || previous_command_timed_out ||
                                 IsSequenceNewer(command.sequence, RL_LOW_CMD_SEQUENCE))) {
                                memcpy(&RECEIVE_RL_LOW_CMD_DATA, &command, sizeof(command));
                                RL_LOW_CMD_SEQUENCE = command.sequence;
                                RL_LOW_CMD_RECEIVE_TIME = HAL_GetTick();
                                RL_LOW_CMD_RECEIVED = true;
                            }
                        }
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

    SEND_DATA_IMU.data.roll_smooth = imu_data.angle_smooth[AX_X];

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
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].legx = USB_FDB_SNAPSHOT.leg[0].rod.L0;
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].legx_dot = USB_FDB_SNAPSHOT.leg[0].rod.dL0;
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].theta = USB_FDB_SNAPSHOT.leg_state[0].theta;
    SEND_ROBOT_MOTION_DATA.data.leg_state[0].theta_dot = USB_FDB_SNAPSHOT.leg_state[0].theta_dot;

    SEND_ROBOT_MOTION_DATA.data.leg_state[1].phi = USB_FDB_SNAPSHOT.leg_state[1].phi;
    SEND_ROBOT_MOTION_DATA.data.leg_state[1].phi_dot = USB_FDB_SNAPSHOT.leg_state[1].phi_dot;
    SEND_ROBOT_MOTION_DATA.data.leg_state[1].legx = USB_FDB_SNAPSHOT.leg[1].rod.L0;
    SEND_ROBOT_MOTION_DATA.data.leg_state[1].legx_dot = USB_FDB_SNAPSHOT.leg[1].rod.dL0;
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
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].legx = USB_REF_SNAPSHOT.rod_L0[0];
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].legx_dot = USB_REF_SNAPSHOT.rod_dL0[0];
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].theta = USB_REF_SNAPSHOT.rod_Angle[0];
    SEND_ROBOT_TARGET_DATA.data.leg_state[0].theta_dot = USB_REF_SNAPSHOT.leg_state[0].theta_dot;

    SEND_ROBOT_TARGET_DATA.data.leg_state[1].phi = USB_REF_SNAPSHOT.leg_state[1].phi;
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].phi_dot = USB_REF_SNAPSHOT.leg_state[1].phi_dot;
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].legx = USB_REF_SNAPSHOT.rod_L0[1];
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].legx_dot = USB_REF_SNAPSHOT.rod_dL0[1];
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].theta = USB_REF_SNAPSHOT.rod_Angle[1];
    SEND_ROBOT_TARGET_DATA.data.leg_state[1].theta_dot = USB_REF_SNAPSHOT.leg_state[1].theta_dot;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_TARGET_DATA, sizeof(SendDataRobotTarget_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_TARGET_DATA, sizeof(SendDataRobotTarget_s));
}

/**
 * @brief 发送控制力矩数据
 */
static void UsbSendTorqueData(void)
{
    SEND_TORQUE_DATA.time_stamp = HAL_GetTick();

    // K[0] → T_r_to_b (右髋力矩)
    SEND_TORQUE_DATA.data.T_r_to_b = USB_FDB_SNAPSHOT.torque[0];
    // K[1] → T_l_to_b (左髋力矩)
    SEND_TORQUE_DATA.data.T_l_to_b = USB_FDB_SNAPSHOT.torque[1];
    // K[2] → T_wr_to_r (右轮力矩)
    SEND_TORQUE_DATA.data.T_wr_to_r = USB_FDB_SNAPSHOT.torque[2];
    // K[3] → T_wl_to_l (左轮力矩)
    SEND_TORQUE_DATA.data.T_wl_to_l = USB_FDB_SNAPSHOT.torque[3];
    // K[4] → T_t_to_b (尾电机力矩)
    SEND_TORQUE_DATA.data.T_t_to_b = USB_FDB_SNAPSHOT.torque[4];

    append_CRC16_check_sum((uint8_t *)&SEND_TORQUE_DATA, sizeof(SendDataTorque_s));
    USB_Transmit((uint8_t *)&SEND_TORQUE_DATA, sizeof(SendDataTorque_s));
}

/**
 * @brief 发送电机离线错误数据
 */
static void UsbSendMotorErrorData(void)
{
    SEND_MOTOR_ERROR_DATA.time_stamp = HAL_GetTick();
    SEND_MOTOR_ERROR_DATA.offline_mask = g_motor_offline_mask;
    SEND_MOTOR_ERROR_DATA.reserved[0] = 0;
    SEND_MOTOR_ERROR_DATA.reserved[1] = 0;
    SEND_MOTOR_ERROR_DATA.reserved[2] = 0;

    append_CRC16_check_sum((uint8_t *)&SEND_MOTOR_ERROR_DATA, sizeof(SendDataMotorError_s));
    USB_Transmit((uint8_t *)&SEND_MOTOR_ERROR_DATA, sizeof(SendDataMotorError_s));
}

/**
 * @brief 发送电机数据 (从 snapshot 读取)
 */
static void UsbSendMotorData(void)
{
    uint8_t i;

    SEND_MOTOR_DATA.time_stamp = HAL_GetTick();
    for (i = 0; i < 7; i++) {
        SEND_MOTOR_DATA.data.position[i] = USB_FDB_SNAPSHOT.motor_pos[i];
        SEND_MOTOR_DATA.data.torque[i] = USB_FDB_SNAPSHOT.motor_torque[i];
    }

    append_CRC16_check_sum((uint8_t *)&SEND_MOTOR_DATA, sizeof(SendDataMotor_s));
    USB_Transmit((uint8_t *)&SEND_MOTOR_DATA, sizeof(SendDataMotor_s));
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

static void UsbSendPidDebugData(void)
{
    float fdb = 0.0f;
    float ref = 0.0f;
    float pid_out = 0.0f;
    if (!ChassisGetRlPdDebug(&fdb, &ref, &pid_out)) return;

    SEND_PID_DEBUG_DATA.time_stamp = HAL_GetTick();
    SEND_PID_DEBUG_DATA.data.fdb = fdb;
    SEND_PID_DEBUG_DATA.data.ref = ref;
    SEND_PID_DEBUG_DATA.data.pid_out = pid_out;
    append_CRC16_check_sum((uint8_t *)&SEND_PID_DEBUG_DATA, sizeof(SendDataPidDebug_s));
    USB_Transmit((uint8_t *)&SEND_PID_DEBUG_DATA, sizeof(SendDataPidDebug_s));
}

static void UsbSendRlStateData(void)
{
    const float roll = USB_FDB_SNAPSHOT.body.roll;
    const float pitch = USB_FDB_SNAPSHOT.body.pitch;
    const float yaw = USB_FDB_SNAPSHOT.body.yaw;
    const float pitch_dot = USB_FDB_SNAPSHOT.body.pitch_dot;
    const float geometry_offset = 0.00316f * cosf(pitch) - 0.00138f * sinf(pitch);
    float height_sum = 0.0f;
    float heightdot_sum = 0.0f;

    for (uint8_t i = 0; i < 2; ++i) {
        const float l0 = USB_FDB_SNAPSHOT.leg[i].rod.L0;
        const float dl0 = USB_FDB_SNAPSHOT.leg[i].rod.dL0;
        const float theta = USB_FDB_SNAPSHOT.leg_state[i].theta;
        const float dtheta = USB_FDB_SNAPSHOT.leg_state[i].theta_dot;
        height_sum += WHEEL_RADIUS + l0 * cosf(theta) - geometry_offset;
        heightdot_sum += dl0 * cosf(theta) - l0 * sinf(theta) * dtheta +
                         (0.00316f * sinf(pitch) + 0.00138f * cosf(pitch)) * pitch_dot;
    }

    const float cr = cosf(0.5f * roll);
    const float sr = sinf(0.5f * roll);
    const float cp = cosf(0.5f * pitch);
    const float sp = sinf(0.5f * pitch);
    const float cy = cosf(0.5f * yaw);
    const float sy = sinf(0.5f * yaw);

    SEND_RL_STATE_DATA.time_stamp = HAL_GetTick();
    ++SEND_RL_STATE_DATA.sequence;
    SEND_RL_STATE_DATA.data.quat_wxyz[0] = cr * cp * cy + sr * sp * sy;
    SEND_RL_STATE_DATA.data.quat_wxyz[1] = sr * cp * cy - cr * sp * sy;
    SEND_RL_STATE_DATA.data.quat_wxyz[2] = cr * sp * cy + sr * cp * sy;
    SEND_RL_STATE_DATA.data.quat_wxyz[3] = cr * cp * sy - sr * sp * cy;
    SEND_RL_STATE_DATA.data.rpy[0] = roll;
    SEND_RL_STATE_DATA.data.rpy[1] = pitch;
    SEND_RL_STATE_DATA.data.rpy[2] = yaw;
    SEND_RL_STATE_DATA.data.gyro_body[0] = USB_FDB_SNAPSHOT.body.roll_dot;
    SEND_RL_STATE_DATA.data.gyro_body[1] = USB_FDB_SNAPSHOT.body.pitch_dot;
    SEND_RL_STATE_DATA.data.gyro_body[2] = USB_FDB_SNAPSHOT.body.yaw_dot;
    SEND_RL_STATE_DATA.data.vx_long = USB_FDB_SNAPSHOT.speed_vector.vx;
    SEND_RL_STATE_DATA.data.height = 0.5f * height_sum;
    SEND_RL_STATE_DATA.data.heightdot = 0.5f * heightdot_sum;

    const uint8_t rl_to_fdb[7] = {0, 1, 4, 2, 3, 5, 6};
    for (uint8_t i = 0; i < 7; ++i) {
        SEND_RL_STATE_DATA.data.joint_pos[i] = USB_FDB_SNAPSHOT.motor_pos[rl_to_fdb[i]];
        SEND_RL_STATE_DATA.data.joint_vel[i] = USB_FDB_SNAPSHOT.motor_vel[rl_to_fdb[i]];
    }

    float remote_vx = 0.0f;
    float remote_wz = 0.0f;
    float remote_height = 0.30f;
    ChassisGetRlRemoteCommand(&remote_vx, &remote_wz, &remote_height);
    SEND_RL_STATE_DATA.data.remote_vx = remote_vx;
    SEND_RL_STATE_DATA.data.remote_wz = remote_wz;
    SEND_RL_STATE_DATA.data.motor_offline_mask = g_motor_offline_mask;
    SEND_RL_STATE_DATA.data.control_mode = ChassisIsRlControlSelected() ? 1U : 0U;
    SEND_RL_STATE_DATA.data.remote_height_mm = (uint16_t)(remote_height * 1000.0f + 0.5f);
    append_CRC16_check_sum((uint8_t *)&SEND_RL_STATE_DATA, sizeof(SendDataRlState_s));
    USB_Transmit((uint8_t *)&SEND_RL_STATE_DATA, sizeof(SendDataRlState_s));
}

static void FillSystemIdentificationCommon(
    SystemIdentificationFrameCommon_t * common, uint8_t state_dimension,
    uint8_t input_dimension)
{
    common->control_tick_ms = USB_FDB_SNAPSHOT.control_tick_ms;
    common->sequence = USB_FDB_SNAPSHOT.control_sequence;
    common->experiment_id = USB_FDB_SNAPSHOT.experiment_id;
    common->reference_sequence = USB_FDB_SNAPSHOT.reference_sequence;
    common->protocol_version = SYS_ID_PROTOCOL_VERSION;
    common->profile_id = SYS_ID_OUTPUT_PROFILE;
    common->state_dimension = state_dimension;
    common->input_dimension = input_dimension;
    common->excitation_config = USB_FDB_SNAPSHOT.sys_id_excitation_config;
    common->excitation_input_mask = USB_FDB_SNAPSHOT.sys_id_excitation_input_mask;
    common->status_flags = USB_FDB_SNAPSHOT.sys_id_status_flags;
    common->motor_saturation_mask = USB_FDB_SNAPSHOT.sys_id_saturation_mask;
}

static void FillSystemIdentificationState(float state[18])
{
    state[0] = USB_FDB_SNAPSHOT.body.x;
    state[1] = USB_FDB_SNAPSHOT.body.x_dot_obv;
    state[2] = USB_FDB_SNAPSHOT.body.yaw;
    state[3] = USB_FDB_SNAPSHOT.body.yaw_dot;
    state[4] = USB_FDB_SNAPSHOT.leg_state[0].theta;
    state[5] = USB_FDB_SNAPSHOT.leg_state[0].theta_dot;
    state[6] = USB_FDB_SNAPSHOT.leg_state[1].theta;
    state[7] = USB_FDB_SNAPSHOT.leg_state[1].theta_dot;
    state[8] = USB_FDB_SNAPSHOT.body.pitch;
    state[9] = USB_FDB_SNAPSHOT.body.pitch_dot;
    state[10] = USB_FDB_SNAPSHOT.tail_state.beta;
    state[11] = USB_FDB_SNAPSHOT.tail_state.beta_dot;
    state[12] = USB_FDB_SNAPSHOT.body.roll;
    state[13] = USB_FDB_SNAPSHOT.body.roll_dot;
    state[14] = USB_FDB_SNAPSHOT.leg[0].rod.L0;
    state[15] = USB_FDB_SNAPSHOT.leg[0].rod.dL0;
    state[16] = USB_FDB_SNAPSHOT.leg[1].rod.L0;
    state[17] = USB_FDB_SNAPSHOT.leg[1].rod.dL0;
}

static void FillSystemIdentificationReference(float reference[18])
{
    reference[0] = USB_REF_SNAPSHOT.body.x;
    reference[1] = USB_REF_SNAPSHOT.speed_vector.vx;
    reference[2] = USB_REF_SNAPSHOT.body.yaw;
    reference[3] = USB_REF_SNAPSHOT.speed_vector.wz;
    reference[4] = USB_REF_SNAPSHOT.rod_Angle[0];
    reference[5] = USB_REF_SNAPSHOT.leg_state[0].theta_dot;
    reference[6] = USB_REF_SNAPSHOT.rod_Angle[1];
    reference[7] = USB_REF_SNAPSHOT.leg_state[1].theta_dot;
    reference[8] = USB_REF_SNAPSHOT.body.pitch;
    reference[9] = USB_REF_SNAPSHOT.body.pitch_dot;
    reference[10] = USB_REF_SNAPSHOT.tail_state.beta;
    reference[11] = USB_REF_SNAPSHOT.tail_state.beta_dot;
    reference[12] = USB_REF_SNAPSHOT.body.roll;
    reference[13] = USB_REF_SNAPSHOT.body.roll_dot;
    reference[14] = USB_REF_SNAPSHOT.rod_L0[0];
    reference[15] = USB_REF_SNAPSHOT.rod_dL0[0];
    reference[16] = USB_REF_SNAPSHOT.rod_L0[1];
    reference[17] = USB_REF_SNAPSHOT.rod_dL0[1];
}

static void UsbSendSystemIdentificationData(void)
{
#if SYS_ID_OUTPUT_PROFILE == SYS_ID_PROFILE_12_STATE_5_INPUT
    FillSystemIdentificationCommon(&SEND_SYS_ID_12_DATA.common, 12U, 5U);
    FillSystemIdentificationState(USB_SYS_ID_STATE_BUFFER);
    memcpy(
        (void *)SEND_SYS_ID_12_DATA.state_feedback, USB_SYS_ID_STATE_BUFFER,
        sizeof(SEND_SYS_ID_12_DATA.state_feedback));
    for (uint8_t i = 0; i < 5U; ++i) {
        SEND_SYS_ID_12_DATA.controller_output[i] = USB_FDB_SNAPSHOT.lqr_output[i];
        SEND_SYS_ID_12_DATA.excitation[i] = USB_FDB_SNAPSHOT.excitation[i];
        SEND_SYS_ID_12_DATA.generalized_command[i] = USB_FDB_SNAPSHOT.generalized_command[i];
        SEND_SYS_ID_12_DATA.generalized_actual_torque[i] =
            USB_FDB_SNAPSHOT.generalized_actual_torque[i];
    }
    memcpy(
        (void *)SEND_SYS_ID_12_DATA.motor_feedback_torque, USB_FDB_SNAPSHOT.motor_torque,
        sizeof(SEND_SYS_ID_12_DATA.motor_feedback_torque));
    append_CRC16_check_sum(
        (uint8_t *)&SEND_SYS_ID_12_DATA, sizeof(SendDataSystemIdentification12_s));
    USB_Transmit(
        (uint8_t *)&SEND_SYS_ID_12_DATA, sizeof(SendDataSystemIdentification12_s));
#else
    FillSystemIdentificationCommon(&SEND_SYS_ID_18_DATA.common, 18U, 7U);
    FillSystemIdentificationState(SEND_SYS_ID_18_DATA.state_feedback);
    for (uint8_t i = 0; i < 7U; ++i) {
        SEND_SYS_ID_18_DATA.controller_output[i] = USB_FDB_SNAPSHOT.lqr_output[i];
        SEND_SYS_ID_18_DATA.excitation[i] = USB_FDB_SNAPSHOT.excitation[i];
        SEND_SYS_ID_18_DATA.generalized_command[i] = USB_FDB_SNAPSHOT.generalized_command[i];
        SEND_SYS_ID_18_DATA.generalized_actual_torque[i] =
            USB_FDB_SNAPSHOT.generalized_actual_torque[i];
        SEND_SYS_ID_18_DATA.motor_feedback_torque[i] = USB_FDB_SNAPSHOT.motor_torque[i];
    }
    append_CRC16_check_sum(
        (uint8_t *)&SEND_SYS_ID_18_DATA, sizeof(SendDataSystemIdentification18_s));
    USB_Transmit(
        (uint8_t *)&SEND_SYS_ID_18_DATA, sizeof(SendDataSystemIdentification18_s));
#endif
}

static void UsbSendSystemIdentificationReferenceData(void)
{
    SEND_SYS_ID_REFERENCE_DATA.experiment_id = USB_FDB_SNAPSHOT.experiment_id;
    SEND_SYS_ID_REFERENCE_DATA.reference_sequence = USB_FDB_SNAPSHOT.reference_sequence;
    SEND_SYS_ID_REFERENCE_DATA.protocol_version = SYS_ID_PROTOCOL_VERSION;
    SEND_SYS_ID_REFERENCE_DATA.profile_id = SYS_ID_OUTPUT_PROFILE;
#if SYS_ID_OUTPUT_PROFILE == SYS_ID_PROFILE_12_STATE_5_INPUT
    SEND_SYS_ID_REFERENCE_DATA.state_dimension = 12U;
    SEND_SYS_ID_REFERENCE_DATA.input_dimension = 5U;
#else
    SEND_SYS_ID_REFERENCE_DATA.state_dimension = 18U;
    SEND_SYS_ID_REFERENCE_DATA.input_dimension = 7U;
#endif
    SEND_SYS_ID_REFERENCE_DATA.excitation_config =
        USB_FDB_SNAPSHOT.sys_id_excitation_config;
    SEND_SYS_ID_REFERENCE_DATA.excitation_input_mask =
        USB_FDB_SNAPSHOT.sys_id_excitation_input_mask;
    SEND_SYS_ID_REFERENCE_DATA.reserved[0] = 0U;
    SEND_SYS_ID_REFERENCE_DATA.reserved[1] = 0U;
    FillSystemIdentificationReference(USB_SYS_ID_REFERENCE_BUFFER);
    memcpy(
        (void *)SEND_SYS_ID_REFERENCE_DATA.state_reference, USB_SYS_ID_REFERENCE_BUFFER,
        sizeof(SEND_SYS_ID_REFERENCE_DATA.state_reference));
    memcpy(
        (void *)SEND_SYS_ID_REFERENCE_DATA.equilibrium_input,
        USB_FDB_SNAPSHOT.equilibrium_input,
        sizeof(SEND_SYS_ID_REFERENCE_DATA.equilibrium_input));
    append_CRC16_check_sum(
        (uint8_t *)&SEND_SYS_ID_REFERENCE_DATA,
        sizeof(SendDataSystemIdentificationReference_s));
    USB_Transmit(
        (uint8_t *)&SEND_SYS_ID_REFERENCE_DATA,
        sizeof(SendDataSystemIdentificationReference_s));
}

static void GetVirtualRcCtrlData(void)
{
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
}

static bool RlLowCmdIsFinite(const ReceiveDataRlLowCmd_s * command)
{
    for (uint8_t i = 0; i < 7; ++i) {
        if (!isfinite(command->data.q_des[i]) || !isfinite(command->data.dq_des[i]) ||
            !isfinite(command->data.kp[i]) || !isfinite(command->data.kd[i]) ||
            !isfinite(command->data.tau_ff[i]) || !isfinite(command->data.q_fdb_offset[i])) {
            return false;
        }
    }
    return true;
}

static bool IsSequenceNewer(uint16_t sequence, uint16_t previous)
{
    return ((int16_t)(sequence - previous) > 0);
}

bool GetRlLowCmdData(ReceiveDataRlLowCmd_s * command)
{
    if ((command == NULL) || !RL_LOW_CMD_RECEIVED || !RECEIVE_RL_LOW_CMD_DATA.enable ||
        ((HAL_GetTick() - RL_LOW_CMD_RECEIVE_TIME) > USB_OFFLINE_THRESHOLD)) {
        return false;
    }
    memcpy(command, &RECEIVE_RL_LOW_CMD_DATA, sizeof(ReceiveDataRlLowCmd_s));
    return true;
}

bool IsRlLowCmdSessionEnabled(void)
{
    return RL_LOW_CMD_RECEIVED && (RECEIVE_RL_LOW_CMD_DATA.enable != 0);
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
float GetScCmdChassisSpeed(uint8_t axis)
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
float GetScCmdChassisVelocity(uint8_t axis)
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
float GetScCmdChassisHeight(void)
{
    return ROBOT_CMD_DATA.chassis.leg_length;
}
/*------------------------------ End of File ------------------------------*/
