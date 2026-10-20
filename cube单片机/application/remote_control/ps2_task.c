/**
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
  * @file       ps2_task.c/h
  * @brief      主要负责和ps2手柄进行通信，接收数据
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-05-2025     Penguin         1. done
  *  V1.0.1     May-07-2025     Penguin         1. 添加了手柄数据解码
  *  V1.0.2     May-08-2025     Penguin         1. 增加了API
  *                                             2. 完善了文档说明
  *
  @verbatim
  ==============================================================================
  本文件中创建一个ps2_task任务，定时请求ps2手柄的数据，并进行解码
  其中
      Spi2RequestPs2Data 函数用于请求ps2手柄的数据
      Ps2Decode 函数用于解码手柄数据
      ps2_task 函数为ps2手柄任务函数，其中完成
          - 请求数据
          - 解码数据
          - 记录手柄数据变化的时间
  API
      GetPs2Status 获取手柄工作状态
      GetPs2IdleTime 获取手柄空闲时间
      GetPs2Joystick 获取手柄摇杆数据
      GetPs2Button 获取手柄按键数据
      UpdatePs2Button 更新手柄按键数据
      UpdatePs2Buttons 更新手柄按键数据组

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
*/
#include "ps2_task.h"

#include <stdbool.h>
#include <string.h>

#include "bsp_delay.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "ps2.h"
#include "ps2_typedef.h"
#include "stm32f4xx_hal.h"

#define PS2_TASK_TIME_MS 10  // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t ps2_high_water;
#endif

// PS2数据结构体
Ps2_s ps2 = {
    .mode = PS2_MODE_ERROR,
    // clang-format off
    .button = {false, false, false, false, false, false, false, false,
               false, false, false, false, false, false, false, false},
    // clang-format on
    .joystick = {0.0f, 0.0f, 0.0f, 0.0f},
    .ps2_data =
        {
            .raw.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        },
};

/**
  * @brief          使用Spi2请求ps2的数据
  * @param[in]      pRxData :接收数据的指针，长度为9
  * @retval         
  */
void Spi2RequestPs2Data(uint8_t * pRxData)
{
    uint8_t cmd[3] = {0x01, 0x42, 0x00};  // 请求接受数据

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  //CS_L
    delay_us(10);

    // 发送0x01，请求接受数据
    HAL_SPI_TransmitReceive(&hspi2, &cmd[0], &pRxData[0], 1, 100);
    delay_us(10);

    // 发送0x42，接收手柄模式（PS2表示开始通信）
    HAL_SPI_TransmitReceive(&hspi2, &cmd[1], &pRxData[1], 1, 100);
    delay_us(10);

    // 发送0x00，接受0x5A（红绿灯模式）
    HAL_SPI_TransmitReceive(&hspi2, &cmd[2], &pRxData[2], 1, 100);
    delay_us(10);

    for (uint8_t i = 3; i < 9; i++) {  // 接受数据
        HAL_SPI_TransmitReceive(&hspi2, &cmd[2], &pRxData[i], 1, 100);
        delay_us(10);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  //CS_H

    // 更新模式
    switch (pRxData[1]) {
        case PS2_MODE_DIGITAL:
        case PS2_MODE_ANALOG:
        case PS2_MODE_VIBRATION:
        case PS2_MODE_CONFIG:
            ps2.mode = pRxData[1];
            break;
        default:
            ps2.mode = PS2_MODE_ERROR;
    }
}

/**
  * @brief          对ps2的数据进行解码
  * @retval         
  */
void Ps2Decode(void)
{
    switch (ps2.mode) {
        case PS2_MODE_ANALOG:
        case PS2_MODE_VIBRATION: {
            for (uint8_t i = 0; i < 16; i++) {
                ps2.button[i] = !((ps2.ps2_data.raw.data[3 + i / 8] >> (i % 8)) & 0x01);
            }
            ps2.joystick[0] = (float)(ps2.ps2_data.raw.data[5] - 128) / 128.0f;
            ps2.joystick[1] = (float)(ps2.ps2_data.raw.data[6] - 127) / 128.0f;
            ps2.joystick[2] = (float)(ps2.ps2_data.raw.data[7] - 128) / 128.0f;
            ps2.joystick[3] = (float)(ps2.ps2_data.raw.data[8] - 127) / 128.0f;
        } break;

        case PS2_MODE_DIGITAL: {
            for (uint8_t i = 0; i < 16; i++) {
                ps2.button[i] = !((ps2.ps2_data.raw.data[3 + i / 8] >> (i % 8)) & 0x01);
            }
            for (uint8_t i = 0; i < 4; i++) {
                ps2.joystick[i] = 0.0f;
            }
        } break;

        default: {
            for (uint8_t i = 0; i < 4; i++) {
                ps2.joystick[i] = 0.0f;
            }
            for (uint8_t i = 0; i < 16; i++) {
                ps2.button[i] = false;
            }
        }
    }
}

void ps2_task(void const * pvParameters)
{
    vTaskDelay(10);

    while (1) {
        memcpy(ps2.last_raw, ps2.ps2_data.raw.data, 9);  // 转移数据

        Spi2RequestPs2Data(ps2.ps2_data.raw.data);  // SPI请求数据
        Ps2Decode();

        for (uint8_t i = 0; i < 9; i++) {  //判断数据变化
            if (ps2.ps2_data.raw.data[i] != ps2.last_raw[i]) {
                ps2.last_operate_time = HAL_GetTick();  // 更新上次操作时间
                break;
            }
        }

        // 系统延时
        vTaskDelay(PS2_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        ps2_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/******************************************************************/
/* API                                                            */
/*----------------------------------------------------------------*/
/* function:      GetPs2Status                                    */
/*                GetPs2IdleTime                                  */
/*                GetPs2Joystick                                  */
/*                GetPs2Button                                    */
/*                UpdatePs2Button                                 */
/*                UpdatePs2Buttons                                */
/******************************************************************/

Ps2Status_e GetPs2Status(void)
{
    switch (ps2.mode) {
        case PS2_MODE_CONFIG: {
            return PS2_CONFIG;
        }

        case PS2_MODE_DIGITAL:
        case PS2_MODE_ANALOG:
        case PS2_MODE_VIBRATION: {
            return PS2_OK;
        }

        case PS2_MODE_ERROR:
        default: {
            return PS2_ERROR;
        }
    }
}

uint32_t GetPs2IdleTime(void) { return HAL_GetTick() - ps2.last_operate_time; }

float GetPs2Joystick(Ps2Joystick_e joystick) { return ps2.joystick[joystick]; }

bool GetPs2Button(Ps2Button_e button) { return ps2.button[button]; }

void UpdatePs2Button(Ps2Button_t * p_ps2_button, Ps2Button_e button)
{
    p_ps2_button->last = p_ps2_button->now;  // 转移按键状态
    p_ps2_button->now = ps2.button[button];  // 更新按键状态
}

void UpdatePs2Buttons(Ps2Buttons_t * p_ps2_buttons)
{
    for (uint8_t i = 0; i < 16; i++) {
        p_ps2_buttons->button[i].last = p_ps2_buttons->button[i].now;  // 转移按键状态
        p_ps2_buttons->button[i].now = ps2.button[i];                  // 更新按键状态
    }
}

/*------------------------------ End of File ------------------------------*/
