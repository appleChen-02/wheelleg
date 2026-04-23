/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       usb.c/h
  * @brief      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-25-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef USB_H
#define USB_H

#include "stdbool.h"
#include "struct_typedef.h"

//API

extern inline uint8_t GetUsbStatus(void);
extern inline float GetScCmdChassisSpeed(uint8_t axis);
extern inline float GetScCmdChassisVelocity(uint8_t axis);
extern inline float GetScCmdChassisAngle(uint8_t axis);
extern inline float GetScCmdChassisHeight(void);
extern inline bool GetScCmdFire(void);
extern inline bool GetScCmdFricOn(void);
extern inline float GetVirtualRcCh(uint8_t channel);
extern inline char GetVirtualRcSw(uint8_t channel);

#endif  // USB_H
/*------------------------------ End of File ------------------------------*/
