/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       uart2_typedef.c/h
  * @brief      uart2的通信部分的相关定义（communication.c内部文件，不对外开放）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-10      Harry_Wong      1.初始化uart2的传输结构体
  *  V1.1.0     2025-04-01      Penguin         1.优化宏
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#ifndef UART2_TYPEDEF_H
#define UART2_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

// clang-format off
// UART2通信协议相关定义
#define UART2_COMMUNICATE_SOF      ((uint8_t)0xA5) // 数据帧起始字节，固定值为 0xA5
#define UART2_COMMUNICATE_TIMEOUT  ((uint32_t)50)  // 超时等待时间

#define Uart2_Data_Test_ID              ((uint8_t)0x01)
#define Uart2_Data_Rc_ID                ((uint8_t)0x02)

#define Uart2_Data_Test_Duration        ((uint32_t)20) // ms
#define Uart2_Data_Rc_Duration          ((uint32_t)16) // ms

// UART2通信协议数据包长度定义
#define UART2_FRAME_MAX_SIZE            ((uint8_t)250) // Byte

#define UART2_FRAME_HEADER_SIZE         sizeof(FrameHeader_t)
#define UART2_FRAME_TIMESTAMP_SIZE      ((uint8_t)sizeof(uint32_t))
#define UART2_FRAME_CRC16_SIZE          ((uint8_t)sizeof(uint16_t))
#define UART2_HEADER_CRC_LEN            (UART2_FRAME_HEADER_SIZE + UART2_FRAME_CRC16_SIZE)
#define UART2_HEADER_CRC_TIMESTAMP_LEN  (UART2_FRAME_HEADER_SIZE + UART2_FRAME_CRC16_SIZE + UART2_FRAME_TIMESTAMP_SIZE)
#define UART2_HEADER_TIMESTAMP_LEN      (UART2_FRAME_HEADER_SIZE + UART2_FRAME_TIMESTAMP_SIZE)
// clang-format on

/*-------------------- Send & Receive --------------------*/

typedef enum {
    STEP_HEADER_SOF = 0,
    STEP_LENGTH = 1,
    STEP_ID = 2,
    STEP_TYPE = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} UnpackStep_e;

typedef struct
{
    uint32_t Interrupt;
    uint32_t Data_Test;
    uint32_t Data_Rc;
} LastTime_t;

typedef struct
{
    uint8_t sof;   // 数据帧起始字节，固定值为 0xA5
    uint8_t len;   // 数据段长度
    uint8_t id;    // 数据段id
    uint8_t type;  // 数据段类型
    uint8_t crc;   // 数据帧头的 CRC8
} __attribute__((packed)) FrameHeader_t;

typedef struct
{
    FrameHeader_t * p_header;
    uint8_t data_len;
    uint8_t protocol_packet[UART2_FRAME_MAX_SIZE];
    UnpackStep_e unpack_step;
    uint16_t index;
} UnpackData_t;

//测试用数据包
typedef struct
{
    FrameHeader_t frame_header;

    uint32_t time_stamp;  //数据段时间戳

    struct
    {
        uint32_t index;
        float test_float;
        uint8_t reserved[42];
    } __attribute__((packed)) data;

    uint16_t crc16;  //crc16校验
} __attribute__((packed)) Data_Test_s;

//遥控器数据包
typedef struct
{
    FrameHeader_t frame_header;

    uint32_t time_stamp;  //数据段时间戳

    struct
    {
        RC_ctrl_t rc_ctrl;
        bool rc_offline;
    } __attribute__((packed)) data;

    uint16_t crc16;  //crc16校验
} __attribute__((packed)) Data_Rc_s;
#endif
/*------------------------------ End of File ------------------------------*/
