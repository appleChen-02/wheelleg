/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_receive.c/h
  * @brief      CAN中断接收函数，接收电机数据.
  * @note       支持DJI电机 GM3508 GM2006 GM6020
  *             支持小米电机 Cybergear
  *             支持达妙电机 DM8009
  *             支持瓴控电机 MF9025
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Mar-27-2024     Penguin         1. 添加CAN发送函数和新的电机控制函数，解码中将CAN1 CAN2分开。
  *  V2.1.0     Mar-20-2024     Penguin         1. 添加DM电机的适配
  *  V2.2.0     May-22-2024     Penguin         1. 添加LK电机的适配
  *  V2.3.0     May-22-2024     Penguin         1. 添加板间通信数据解码
  *  V2.3.1     Apr-01-2024     Penguin         1. 添加了DJI电机离线的判断
  *
  @verbatim
  ==============================================================================
    dm电机设置：
    为了配合本框架，请在使用上位机进行设置时，将dm电机的master id 设置为 slave id + 0x50
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "CAN_receive.h"

#include "bsp_can.h"
#include "can_typedef.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "robot_param.h"
#include "string.h"
#include "usb_debug.h"
#include "user_lib.h"

#define DATA_NUM 10

#define CAN_OFFLINE_TIME 100  // ms

// 接收数据
static CybergearMeasure_s CAN1_CYBERGEAR_MEASURE[CYBERGEAR_NUM + 1];
static CybergearMeasure_s CAN2_CYBERGEAR_MEASURE[CYBERGEAR_NUM + 1];

static LkMeasure_s CAN1_LK_MEASURE[LK_NUM];
static LkMeasure_s CAN2_LK_MEASURE[LK_NUM];

// static uint8_t OTHER_BOARD_DATA_ANY[DATA_NUM][8];

static uint32_t LAST_RECEIVE_TIME = 0;  // 上次接收时间

/*-------------------- Decode --------------------*/
/**
 * @brief        LkFdbData: 获取LK电机反馈数据函数
 * @param[out]   dm_measure 电机数据缓存
 * @param[in]    rx_data 指向包含反馈数据的数组指针
 * @note         从接收到的数据中提取LK电机的反馈信息
 */
void LkFdbData(LkMeasure_s * lk_measure, uint8_t * rx_data)
{
    lk_measure->ctrl_id = rx_data[0];
    lk_measure->temprature = rx_data[1];
    lk_measure->iq = (uint16_t)(rx_data[3] << 8 | rx_data[2]);
    lk_measure->speed = (uint16_t)(rx_data[5] << 8 | rx_data[4]);
    lk_measure->encoder = (uint16_t)(rx_data[7] << 8 | rx_data[6]);

    lk_measure->last_fdb_time = HAL_GetTick();
}

/**
 * @brief          若接收到的数据标识符为StdId则对应解码
 * @note           解码数据包括DJI电机数据与板间通信数据
 * @param[in]      CAN CAN口(CAN_1或CAN_2)
 * @param[in]      rx_header CAN接收数据头
 * @param[in]      rx_data CAN接收数据
 */
static void DecodeStdIdData(hcan_t * CAN, CAN_RxHeaderTypeDef * rx_header, uint8_t rx_data[8])
{
    switch (rx_header->StdId) {  //电机解码
        case LK_M1_ID:
        case LK_M2_ID:
        case LK_M3_ID:
        case LK_M4_ID: {  // 以上ID为LK电机标识符
            static uint8_t i = 0;
            i = rx_header->StdId - LK_M1_ID;
            if (CAN == &hcan1)  // 接收到的数据是通过 CAN1 接收的
            {
                LkFdbData(&CAN1_LK_MEASURE[i], rx_data);
            } else if (CAN == &hcan2)  // 接收到的数据是通过 CAN2 接收的
            {
                LkFdbData(&CAN2_LK_MEASURE[i], rx_data);
            }
            // detect_hook(CHASSIS_MOTOR1_TOE + i);
            return;
        }
        default: {
            break;
        }
    }
}

/**
  * @brief          小米电机反馈帧解码（通信类型2）
  * @param[in]      p_motor 电机结构体
  * @param[in]      rx_data[8] CAN线接收到的数据
  * @note           将接收到的CAN线数据解码到电机结构体中
  * @retval         none
  */
static void CybergearRxDecode(Motor_s * p_motor, uint8_t rx_data[8])
{
    uint16_t decode_temp_mi;  //小米电机反馈数据解码缓冲
    decode_temp_mi = (rx_data[0] << 8 | rx_data[1]);
    p_motor->fdb.pos = ((float)decode_temp_mi - 32767.5f) / 32767.5f * 4 * 3.1415926f;

    decode_temp_mi = (rx_data[2] << 8 | rx_data[3]);
    p_motor->fdb.vel = ((float)decode_temp_mi - 32767.5f) / 32767.5f * 30.0f;

    decode_temp_mi = (rx_data[4] << 8 | rx_data[5]);
    p_motor->fdb.tor = ((float)decode_temp_mi - 32767.5f) / 32767.5f * 12.0f;

    decode_temp_mi = (rx_data[6] << 8 | rx_data[7]);
    p_motor->fdb.temp = (float)decode_temp_mi / 10.0f;
}

/**
 * @brief          若接收到的数据标识符为ExtId则对应解码
 * @note           解码数据包括cybergear电机数据与板间通信数据
 * @param[in]      CAN CAN口(CAN_1或CAN_2)
 * @param[in]      rx_header CAN接收数据头
 * @param[in]      rx_data CAN接收数据
 */
static void DecodeExtIdData(hcan_t * CAN, CAN_RxHeaderTypeDef * rx_header, uint8_t rx_data[8])
{
    uint8_t motor_id = 0;
    if (((RxCanInfo_s *)(&rx_header->ExtId))->communication_type == 2) {  //通信类型2
        motor_id = ((RxCanInfoType_2_s *)(&rx_header->ExtId))->motor_id;
    }

    if (CAN == &hcan1)  // 接收到的数据是通过 CAN1 接收的
    {
        memcpy(&CAN1_CYBERGEAR_MEASURE[motor_id].ext_id, &rx_header->ExtId, 4);
        memcpy(CAN1_CYBERGEAR_MEASURE[motor_id].rx_data, rx_data, 8);
    } else if (CAN == &hcan2)  // 接收到的数据是通过 CAN2 接收的
    {
        memcpy(&CAN2_CYBERGEAR_MEASURE[motor_id].ext_id, &rx_header->ExtId, 4);
        memcpy(CAN2_CYBERGEAR_MEASURE[motor_id].rx_data, rx_data, 8);
    }
}

/*-------------------- Callback --------------------*/

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t * hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.IDE == CAN_ID_STD)  // 接收到的数据标识符为StdId
    {
        DecodeStdIdData(hcan, &rx_header, rx_data);
    } else if (rx_header.IDE == CAN_ID_EXT)  // 接收到的数据标识符为ExtId
    {
        DecodeExtIdData(hcan, &rx_header, rx_data);
    }
}

/*-------------------- Get data --------------------*/

/**
 * @brief          获取cybergear电机反馈数据
 * @param[out]     p_motor 电机结构体 
 * @param[in]      p_cybergear_measure 电机反馈数据缓存区
 * @return         none
 */
static void GetCybergearFdbData(Motor_s * p_motor, CybergearMeasure_s * p_cybergear_measure)
{
    CybergearRxDecode(p_motor, p_cybergear_measure->rx_data);
    RxCanInfoType_2_s * rx_info =
        (RxCanInfoType_2_s *)(&CAN1_CYBERGEAR_MEASURE[p_motor->id].ext_id);
    p_motor->fdb.state = rx_info->mode_state;
}

CybergearModeState_e GetCybergearModeState(Motor_s * p_motor)
{
    if (p_motor->type != CYBERGEAR_MOTOR) return UNDEFINED_MODE;

    // clang-format off
    if (p_motor->can == 1) {
        return (CybergearModeState_e)(((RxCanInfoType_2_s *)(&CAN1_CYBERGEAR_MEASURE[p_motor->id].ext_id))->mode_state);
    } else {
        return (CybergearModeState_e)(((RxCanInfoType_2_s *)(&CAN2_CYBERGEAR_MEASURE[p_motor->id].ext_id))->mode_state);
    }
    // clang-format on
}

/**
 * @brief          获取LK电机反馈数据
 * @param[out]     motor 电机结构体 
 * @param[in]      lk_measure 电机反馈数据缓存区
 * @return         none
 */
static void GetLkFdbData(Motor_s * motor, const LkMeasure_s * lk_measure)
{
    // motor->fdb.pos = uint_to_float(lk_measure->encoder, -M_PI, M_PI, 16);
    motor->fdb.pos = lk_measure->encoder * DOUBLE_PI / 65536;  //角度位置

    if (motor->type == MF_9025) {
        motor->fdb.tor = lk_measure->iq * MF_CONTROL_TO_CURRENT * LK_TORQUE_CONSTANT_MF9025;
        motor->fdb.vel = lk_measure->speed * DEGREE_TO_RAD;
    } else if (motor->type == MG_6012) {
        motor->fdb.tor =
            lk_measure->iq * MG_CONTROL_TO_CURRENT * LK_TORQUE_CONSTANT_MG6012;// * LK_RATIO_MG6012;
        motor->fdb.vel = lk_measure->speed * DEGREE_TO_RAD / LK_RATIO_MG6012;
    } else if (motor->type == MG_5010) {
        motor->fdb.tor =
            lk_measure->iq * MG_CONTROL_TO_CURRENT * LK_TORQUE_CONSTANT_MG5010;// * LK_RATIO_MG5010;
        motor->fdb.vel = lk_measure->speed * DEGREE_TO_RAD / LK_RATIO_MG5010;
    }

    motor->fdb.temp = lk_measure->temprature;

    uint32_t now = HAL_GetTick();
    if (now - lk_measure->last_fdb_time > MOTOR_STABLE_RUNNING_TIME) {
        motor->offline = true;
    } else {
        motor->offline = false;
    }
}

/**
 * @brief          获取接收数据
 * @param[out]     p_motor 电机结构体
 * @return         none
 */
void GetMotorMeasure(Motor_s * p_motor)
{
    switch (p_motor->type) {
        case CYBERGEAR_MOTOR: {
            if (p_motor->can == 1) {
                GetCybergearFdbData(p_motor, &CAN1_CYBERGEAR_MEASURE[p_motor->id]);
            } else {
                GetCybergearFdbData(p_motor, &CAN2_CYBERGEAR_MEASURE[p_motor->id]);
            }
        } break;
        case MF_9025: {
            if (p_motor->can == 1) {
                GetLkFdbData(p_motor, &CAN1_LK_MEASURE[p_motor->id - 1]);
            } else {
                GetLkFdbData(p_motor, &CAN2_LK_MEASURE[p_motor->id - 1]);
            }
        } break;
        case MG_6012: {
            if (p_motor->can == 1) {
                GetLkFdbData(p_motor, &CAN1_LK_MEASURE[p_motor->id - 1]);
            } else {
                GetLkFdbData(p_motor, &CAN2_LK_MEASURE[p_motor->id - 1]);
            }
        } break;
        case MG_5010: {
            if (p_motor->can == 1) {
                GetLkFdbData(p_motor, &CAN1_LK_MEASURE[p_motor->id - 1]);
            } else {
                GetLkFdbData(p_motor, &CAN2_LK_MEASURE[p_motor->id - 1]);
            }
        } break;
        default:
            break;
    }
}

bool GetBoardCanOffline(void)
{
    if (HAL_GetTick() - LAST_RECEIVE_TIME > CAN_OFFLINE_TIME) return true;
    return false;
}

// bool GetCanRcOffline(void)
// {
//     if (GetBoardCanOffline()) return true;
//     return RECEIVE_CBC.rc_data.rc.packed.offline;
// }
/************************ END OF FILE ************************/
