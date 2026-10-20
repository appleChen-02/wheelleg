/**
 * @file CAN_cmd_lingkong.h
 * @author apple ()
 * @brief MG6012-i36v3-24v,MF902516T,MG5010E-i10电机CAN控制命令
 * @version 0.1
 * @date 2026-09-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef CAN_CMD_LINGKONG_H
#define CAN_CMD_LINGKONG_H

#include "motor.h"

extern void LkDisable(Motor_s * p_motor);
extern void LkStop(Motor_s * p_motor);
extern void LkEnable(Motor_s * p_motor);
extern void LkSingleSpeedControl(Motor_s * p_motor);
extern void LkSingleTorqueControl(Motor_s * p_motor);
extern void LkSinglePostionControl(Motor_s * p_motor);
extern void LkMultipleTorqueControl_MF9025(uint8_t can, float torque_1, float torque_2, float torque_3, float torque_4);
extern void LkMultipleIqControl_MF9025(uint8_t can, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4);
extern void LkMultipleTorqueControl_MG6012(uint8_t can, float torque_1, float torque_2, float torque_3, float torque_4);
extern void LkMultipleIqControl_MG6012(uint8_t can, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4);
extern void LkMultipleTorqueControl_MF9025_MG5010(
    uint8_t can, float torque_1, float torque_2, float torque_3, float torque_4);
#endif /* CAN_CMD_LINGKONG_H */
/************************ END OF FILE ************************/
