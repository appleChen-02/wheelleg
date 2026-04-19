/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_lingkong.c/h
  * @brief      CAN发送函数，通过CAN信号控制瓴控电机 MF9025 + MG6012e + MG5010.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-16-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_cmd_lingkong.h"

#include "bsp_can.h"
#include "stm32f4xx_hal.h"
#include "user_lib.h"

#define STDID_OFFESET ((uint16_t)0x140)

#define TORQUE_COEFFICIENT 0.32f                  // (N*m/A)转矩系数
#define CURRENT_TO_MULTICONTROL 62.5f             // (2000/32)(1/A)电流转换为控制量
#define CURRENT_TO_MF_CONTROL 124.1212121212121f  // (2048/16.5)(1/A)电流转换为控制量

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/*-------------------- Private functions --------------------*/

/**
 * @brief        电机失能
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void DisableMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x80;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        停止电机
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void StopMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x81;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        电机使能
 * @param[in]    hcan     指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id 电机ID，指定目标电机
 */
static void EnableMotor(hcan_t * hcan, uint16_t motor_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0x88;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = 0x00;
    CAN_CTRL_DATA.tx_data[5] = 0x00;
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        单电机转矩闭环控制命令
 * @param[in]    hcan      指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id  电机ID，指定目标电机
 * @param[in]    iqControl 转矩电流 -2048~2048
 */
static void SingleTorqueControl(hcan_t * hcan, uint16_t motor_id, int16_t iqControl)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0xA1;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&iqControl);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&iqControl) + 1);
    CAN_CTRL_DATA.tx_data[6] = 0x00;
    CAN_CTRL_DATA.tx_data[7] = 0x00;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        单电机位置闭环控制命令
 * @param[in]    hcan      指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id  电机ID，指定目标电机
 * @param[in]    iqControl 转矩电流 -2048~2048
 */
static void SinglePostionControl(hcan_t * hcan, uint16_t motor_id, int32_t posControl, uint16_t speedControl)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0xA4;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = *(uint8_t *)(&speedControl);
    CAN_CTRL_DATA.tx_data[3] = *((uint8_t *)(&speedControl) + 1);
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&posControl);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&posControl) + 1);
    CAN_CTRL_DATA.tx_data[6] = *((uint8_t *)(&posControl) + 2);
    CAN_CTRL_DATA.tx_data[7] = *((uint8_t *)(&posControl) + 3);

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        单电机速度闭环控制命令
 * @param[in]    hcan      指向CAN_HandleTypeDef结构的指针
 * @param[in]    motor_id  电机ID，指定目标电机
 * @param[in]    speedControl 
 */
static void SingleSpeedControl(hcan_t * hcan, uint16_t motor_id, int32_t speedControl)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + STDID_OFFESET;

    CAN_CTRL_DATA.tx_data[0] = 0xA2;
    CAN_CTRL_DATA.tx_data[1] = 0x00;
    CAN_CTRL_DATA.tx_data[2] = 0x00;
    CAN_CTRL_DATA.tx_data[3] = 0x00;
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&speedControl);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&speedControl) + 1);
    CAN_CTRL_DATA.tx_data[6] = *((uint8_t *)(&speedControl) + 2);
    CAN_CTRL_DATA.tx_data[7] = *((uint8_t *)(&speedControl) + 3);

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief        多电机转矩闭环控制命令
 * @param[in]    hcan        指向CAN_HandleTypeDef结构的指针
 * @param[in]    iqControl_1 转矩电流 -2000\\~2000
 * @param[in]    iqControl_2 转矩电流 -2000\\~2000
 * @param[in]    iqControl_3 转矩电流 -2000\\~2000
 * @param[in]    iqControl_4 转矩电流 -2000\\~2000
 */
static void MultipleTorqueControl(hcan_t * hcan, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = 0x280;

    CAN_CTRL_DATA.tx_data[0] = *(uint8_t *)(&iqControl_1);
    CAN_CTRL_DATA.tx_data[1] = *((uint8_t *)(&iqControl_1) + 1);
    CAN_CTRL_DATA.tx_data[2] = *(uint8_t *)(&iqControl_2);
    CAN_CTRL_DATA.tx_data[3] = *((uint8_t *)(&iqControl_2) + 1);
    CAN_CTRL_DATA.tx_data[4] = *(uint8_t *)(&iqControl_3);
    CAN_CTRL_DATA.tx_data[5] = *((uint8_t *)(&iqControl_3) + 1);
    CAN_CTRL_DATA.tx_data[6] = *(uint8_t *)(&iqControl_4);
    CAN_CTRL_DATA.tx_data[7] = *((uint8_t *)(&iqControl_4) + 1);

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/*-------------------- Check functions --------------------*/

/**
 * @brief      获取can总线句柄
 * @param[in]  motor 电机结构体
 * @return     can总线句柄
 * @note       获取电机结构体中的can号，返回对应的can总线句柄，同时检测电机类型是否为达妙电机
 */
static hcan_t * GetHcanPoint(Motor_s * motor)
{
    if ((motor->type != MF_9025) && (motor->type != MG_6012) && (motor->type != MG_5010)) return NULL;

    if (motor->can == 1)
        return &hcan1;
    else if (motor->can == 2)
        return &hcan2;

    return NULL;
}

/*-------------------- User functions --------------------*/

void LkDisable(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    DisableMotor(hcan, p_motor->id);
}

void LkStop(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    StopMotor(hcan, p_motor->id);
}

void LkEnable(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    EnableMotor(hcan, p_motor->id);
}

fp32 Torque2iqControl_MG6012(fp32 tor)
{
    if (tor == 0.0f) return 0.0f;

    float sign = (tor > 0.0f) ? 1.0f : -1.0f;
    float x = (tor > 0.0f) ? tor : -tor;   // fabsf(Torque) 也行

    // Horner: ((((a6*x + a5)*x + a4)*x + a3)*x + a2)*x + a1)*x + a0
    float y =
        ((((((0.000005f  * x
            - 0.0006f)  * x
            + 0.0283f)  * x
            - 0.6985f)  * x
            + 8.6584f)  * x
            - 31.982f)  * x
            + 87.13f);

    return sign * y;
}

fp32 Torque2iqControl_MG5010(fp32 tor)
{
    if (tor == 0.0f) return 0.0f;

    float sign = (tor > 0.0f) ? 1.0f : -1.0f;
    float x = (tor > 0.0f) ? tor : -tor;   // fabsf(Torque) 也行

    // Horner: ((((a6*x + a5)*x + a4)*x + a3)*x + a2)*x + a1)*x + a0
    float y =
        ((((((-0.007f  * x
            - 0.0388f)  * x
            + 1.5394f)  * x
            - 7.7984f)  * x
            + 3.9453f)  * x
            + 228.07f)  * x
            + 1.2446f);

    return sign * y;
}
// 逐个电机  A1  扭矩控制
void LkSingleTorqueControl(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    if(p_motor->type == MF_9025)
    {
        SingleTorqueControl(
        hcan, p_motor->id,
        fp32_constrain(p_motor->set.tor, LK_MIN_MF9025_TORQUE, LK_MAX_MF9025_TORQUE) / TORQUE_COEFFICIENT * CURRENT_TO_MULTICONTROL);
    }
    else if(p_motor->type == MG_6012)
    {
        SingleTorqueControl(
        hcan, p_motor->id,
        Torque2iqControl_MG6012(fp32_constrain(p_motor->set.tor, LK_MIN_MG6012_TORQUE, LK_MAX_MG6012_TORQUE)));
    }
    else if(p_motor->type == MG_5010)
    {
        SingleTorqueControl(
        hcan, p_motor->id,
        Torque2iqControl_MG5010(fp32_constrain(p_motor->set.tor, LK_MIN_MG5010_TORQUE, LK_MAX_MG5010_TORQUE)));
    }
}

void LkSingleSpeedControl(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    SingleSpeedControl(hcan, p_motor->id, p_motor->set.vel * RAD_TO_DEGREE * 100);//应该只有轮子用
}

void LkSinglePostionControl(Motor_s * p_motor)
{
    hcan_t * hcan = GetHcanPoint(p_motor);
    if (hcan == NULL) return;

    if (p_motor->type == MG_6012) SinglePostionControl(hcan, p_motor->id, 
        p_motor->set.pos * RAD_TO_DEGREE * 100 * LK_RATIO_MG6012, 200);
    else if (p_motor->type == MG_5010) SinglePostionControl(hcan, p_motor->id, 
        p_motor->set.pos * RAD_TO_DEGREE * 100 * LK_RATIO_MG5010, 200);
}

void LkMultipleTorqueControl_MF9025(
    uint8_t can, float torque_1, float torque_2, float torque_3, float torque_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;

    if (hcan == NULL) return;

    int16_t iqControl[4];
    iqControl[0] = int16_constrain(
        (torque_1 / TORQUE_COEFFICIENT) * CURRENT_TO_MULTICONTROL, LK_MIN_MULTICONTROL_IQ,
        LK_MAX_MULTICONTROL_IQ);
    iqControl[1] = int16_constrain(
        (torque_2 / TORQUE_COEFFICIENT) * CURRENT_TO_MULTICONTROL, LK_MIN_MULTICONTROL_IQ,
        LK_MAX_MULTICONTROL_IQ);
    iqControl[2] = int16_constrain(
        (torque_3 / TORQUE_COEFFICIENT) * CURRENT_TO_MULTICONTROL, LK_MIN_MULTICONTROL_IQ,
        LK_MAX_MULTICONTROL_IQ);
    iqControl[3] = int16_constrain(
        (torque_4 / TORQUE_COEFFICIENT) * CURRENT_TO_MULTICONTROL, LK_MIN_MULTICONTROL_IQ,
        LK_MAX_MULTICONTROL_IQ);

    MultipleTorqueControl(hcan, iqControl[0], iqControl[1], iqControl[2], iqControl[3]);
}

void LkMultipleTorqueControl_MF9025_MG5010(
    uint8_t can, float torque_1, float torque_2, float torque_3, float torque_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;

    if (hcan == NULL) return;

    int16_t iqControl[4];
    iqControl[0] = int16_constrain(
        (torque_1 / TORQUE_COEFFICIENT) * CURRENT_TO_MULTICONTROL, LK_MIN_MULTICONTROL_IQ,
        LK_MAX_MULTICONTROL_IQ);
    iqControl[1] = int16_constrain(
        (torque_2 / TORQUE_COEFFICIENT) * CURRENT_TO_MULTICONTROL, LK_MIN_MULTICONTROL_IQ,
        LK_MAX_MULTICONTROL_IQ);
    iqControl[2] = int16_constrain(Torque2iqControl_MG5010(torque_3), 
    LK_MIN_MULTICONTROL_IQ, LK_MAX_MULTICONTROL_IQ);
    iqControl[3] = 0;

    MultipleTorqueControl(hcan, iqControl[0], iqControl[1], iqControl[2], iqControl[3]);
}

void LkMultipleTorqueControl_MG6012(
    uint8_t can, float torque_1, float torque_2, float torque_3, float torque_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;

    if (hcan == NULL) return;

    int16_t iqControl[4];
    iqControl[0] = int16_constrain(Torque2iqControl_MG6012(torque_1), 
    LK_MIN_MULTICONTROL_IQ, LK_MAX_MULTICONTROL_IQ);
    iqControl[1] = int16_constrain(Torque2iqControl_MG6012(torque_2), 
    LK_MIN_MULTICONTROL_IQ, LK_MAX_MULTICONTROL_IQ);
    iqControl[2] = int16_constrain(Torque2iqControl_MG6012(torque_3), 
    LK_MIN_MULTICONTROL_IQ, LK_MAX_MULTICONTROL_IQ);
    iqControl[3] = int16_constrain(Torque2iqControl_MG6012(torque_4), 
    LK_MIN_MULTICONTROL_IQ, LK_MAX_MULTICONTROL_IQ);

    MultipleTorqueControl(hcan, iqControl[0], iqControl[1], iqControl[2], iqControl[3]);
}

void LkMultipleIqControl_MF9025(uint8_t can, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)      hcan = &hcan1;
    else if (can == 2) hcan = &hcan2;

    if (hcan == NULL) return;

    int16_t current[4];
    current[0] = int16_constrain(iqControl_1, -2000, 2000);
    current[1] = int16_constrain(iqControl_2, -2000, 2000);
    current[2] = int16_constrain(iqControl_3, -2000, 2000);
    current[3] = int16_constrain(iqControl_4, -2000, 2000);

    MultipleTorqueControl(hcan, current[0], current[1], current[2], current[3]);
}

void LkMultipleIqControl_MG6012(uint8_t can, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)      hcan = &hcan1;
    else if (can == 2) hcan = &hcan2;

    if (hcan == NULL) return;

    int16_t current[4];
    current[0] = int16_constrain(iqControl_1, -2000, 2000);
    current[1] = int16_constrain(iqControl_2, -2000, 2000);
    current[2] = int16_constrain(iqControl_3, -2000, 2000);
    current[3] = int16_constrain(iqControl_4, -2000, 2000);

    MultipleTorqueControl(hcan, current[0], current[1], current[2], current[3]);
}
/************************ END OF FILE ************************/
