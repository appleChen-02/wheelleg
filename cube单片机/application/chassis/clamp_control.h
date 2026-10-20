/**
 * @file clamp_control.h
 * @brief 夹爪控制模块对外接口声明
 * @note
 * 1. ClampInit: 上电初始化夹爪（含激活流程）。
 * 2. ClampSetTarget: 设置夹爪目标位置/力/速度（只更新目标缓存，不直接发送）。
 * 3. Clampcontrol: 初始化后的唯一控制入口，读取目标缓存并发送控制指令。
 * 4. ClampUartOnReceiveComplete: UART接收完成回调分发函数。
 */
#ifndef CLAMP_CONTROL_H
#define CLAMP_CONTROL_H

#include "usart.h"

// 初始化夹爪（内部会完成激活流程，超时后跳过）
extern void ClampInit(void);
// 设置夹爪目标（position/force/speed均为0~255）
extern void ClampSetTarget(uint8_t position, uint8_t force, uint8_t speed);
// 夹爪唯一控制入口
extern void Clampcontrol(void);
// 标记存在新目标并触发控制状态机立即处理
extern void settarget_pending(void);
// UART中断接收完成后的数据处理
extern void ClampUartOnReceiveComplete(UART_HandleTypeDef * huart);

#endif
