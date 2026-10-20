/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 添加云台和发射机构类型
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H


#include "robot_typedef.h"
#include "struct_typedef.h"

//导入具体的机器人参数配置文件
#include "robot_param_balanced_infantry.h"

// 选择机器人的各种类型
#define __RC_TYPE RC_HT8A          // 遥控器类型
#define __DEVELOP 0                // 开发模式
#define __DEBUG 0                  // 调试模式
#define __TUNING 0                 // 调参模式
#define __MUSIC_ON 0               // 开启音乐
#define __TUNING_MODE TUNING_NONE  // 调参模式
#define __HEAT_IMU 1  // 加热IMU(防止Debug时因断点导致pid失效产生过热，烧坏IMU)
#define __IMU_CONTROL_TEMPERATURE 35 // (度)IMU目标控制温度

#define __BOARD_INSTALL_SPIN_MATRIX    \
{0.0f, 1.0f, 0.0f},                     \
{-1.0f, 0.0f, 0.0f},                     \
{0.0f, 0.0f, 1.0f} \

// USB通信的部分选项
#define __USB_SEND_DEBUG 1  // 发送DEBUG数据

// 本板id
#ifndef __SELF_BOARD_ID
#define __SELF_BOARD_ID C_BOARD_DEFAULT
#endif

// 控制链路选择
#ifndef __CONTROL_LINK_RC
#define __CONTROL_LINK_RC CL_RC_DIRECT
#endif

#ifndef __CONTROL_LINK_KM
#define __CONTROL_LINK_KM CL_KM_RC
#endif

#ifndef __CONTROL_LINK_PS2
#define __CONTROL_LINK_PS2 CL_PS2_NONE
#endif

#endif /* ROBOT_PARAM_H */
