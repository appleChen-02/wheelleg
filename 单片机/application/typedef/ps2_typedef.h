#ifndef PS2_TYPEDEF_H__
#define PS2_TYPEDEF_H__

#include "attribute_typedef.h"

// clang-format off
#define PS2_MODE_ERROR     0x00 // 异常模式，手柄工作异常
#define PS2_MODE_DIGITAL   0x41 // 数字模式，只返回按键信息，摇杆无效
#define PS2_MODE_ANALOG    0x73 // 模拟模式，包含按键和摇杆数据
#define PS2_MODE_VIBRATION 0x79 // 模拟模式，加入了手柄马达的控制
#define PS2_MODE_CONFIG    0xF3 // 设置模式，用于配置手柄
// clang-format on

typedef struct
{
    uint16_t mode;  // 模式

    bool button[16];    //按键
    float joystick[4];  //摇杆

    uint32_t last_operate_time;  // 上次操作时间

    union {
        struct
        {
            uint8_t id;
            uint8_t mode;
            uint8_t flag;

            uint8_t select : 1;
            uint8_t l3 : 1;
            uint8_t r3 : 1;
            uint8_t start : 1;
            uint8_t up : 1;
            uint8_t right : 1;
            uint8_t down : 1;
            uint8_t left : 1;

            uint8_t l2 : 1;
            uint8_t r2 : 1;
            uint8_t l1 : 1;
            uint8_t r1 : 1;
            uint8_t triangle : 1;
            uint8_t circle : 1;
            uint8_t cross : 1;
            uint8_t square : 1;

            uint8_t rx;
            uint8_t ry;
            uint8_t lx;
            uint8_t ly;
        } __packed__ val;
        struct
        {
            uint8_t data[9];  //数据
        } __packed__ raw;
    } ps2_data;

    uint8_t last_raw[9];
} Ps2_s;

#endif  // PS2_TYPEDEF_H__
/*------------------------------ End of File ------------------------------*/
