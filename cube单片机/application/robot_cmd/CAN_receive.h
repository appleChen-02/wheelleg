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
  *
  @verbatim
  ==============================================================================
    dm电机设置：
    为了配合本框架，请在使用上位机进行设置时，将dm电机的master id 设置为 slave id + 0x50
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "motor.h"
#include "stm32f4xx_hal.h"
#include "remote_control.h"

#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

// clang-format off

typedef enum __LkMotorType{
    LK_M1_ID = 0x141,
    LK_M2_ID,
    LK_M3_ID,
    LK_M4_ID,
    // LK_M5_ID,
    // LK_M6_ID,
    // LK_M7_ID,
} LkMotorType_e;
// clang-format on

extern CybergearModeState_e GetCybergearModeState(Motor_s * p_motor);

extern void GetMotorMeasure(Motor_s * p_motor);

// extern bool GetCanRcOffline(void);

#endif
/*------------------------------ End of File ------------------------------*/
