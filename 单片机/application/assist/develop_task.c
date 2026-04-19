// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "signal_generator.h"
#include "remote_control.h"
#include "usb_debug.h"

const Sbus_t* SBUS;
const RC_ctrl_t* RC_CTRL;

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(500);

    SBUS = get_sbus_point();
    RC_CTRL = get_remote_control_point();

    while (1) {
        
        vTaskDelay(1);
    }
}
