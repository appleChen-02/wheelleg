/**
 * @file clamp_control.c
 * @brief 夹爪控制与UART2通信模块
 *
 * 主要逻辑：
 * 1. 采用中断方式按字节接收UART2数据，并用IDLE中断做变长帧收尾（最大16字节）。
 * 2. ClampInit中执行夹爪激活状态机（rAct=0 -> rAct=1 -> 轮询状态）。
 * 3. ClampSetTarget仅更新目标缓存；Clampcontrol读取目标缓存并发送控制指令。
 */
#include "clamp_control.h"

#include <string.h>

#include "bsp_uart.h"
#include "cmsis_os.h"  


#define CLAMP_UART_RX_MAX_LEN 16
#define CLAMP_INIT_TOTAL_TIMEOUT_MS 5000
#define CLAMP_STAGE_TIMEOUT_MS 100
#define CLAMP_STATUS_POLL_INTERVAL_MS 20
#define CLAMP_CTRL_ACK_TIMEOUT_MS 100
#define CLAMP_MODBUS_SLAVE_ADDR 0x09
#define CLAMP_MODBUS_FUNC_READ_HOLDING_REGS 0x04
#define CLAMP_MODBUS_FUNC_WRITE_MULTI_REGS 0x10
#define CLAMP_REG_ACTION_BASE 0x03E8
#define CLAMP_REG_STATUS_BASE 0x07D0

typedef enum {
    CLAMP_INIT_SEND_CLEAR = 0,
    CLAMP_INIT_WAIT_CLEAR_ACK,
    CLAMP_INIT_SEND_SET,
    CLAMP_INIT_WAIT_SET_ACK,
    CLAMP_INIT_SEND_READ_STATUS,
    CLAMP_INIT_WAIT_STATUS,
    CLAMP_INIT_DONE,
    CLAMP_INIT_SKIPPED,
} ClampInitState_e; // 初始化状态机状态

typedef enum {
    CLAMP_CTRL_IDLE = 0,// 等待目标
    CLAMP_CTRL_WAIT_ACK,// 已发送目标，等待ACK
} ClampCtrlState_e; // 目标控制状态机状态

static uint8_t clamp_cmd_clear_ract[CLAMP_UART_RX_MAX_LEN]; // 预生成的命令帧缓冲和长度
static uint8_t clamp_cmd_set_ract[CLAMP_UART_RX_MAX_LEN];
static uint8_t clamp_cmd_read_status[CLAMP_UART_RX_MAX_LEN];
static uint8_t clamp_cmd_target[CLAMP_UART_RX_MAX_LEN];
static uint8_t clamp_cmd_clear_ract_len;
static uint8_t clamp_cmd_set_ract_len;
static uint8_t clamp_cmd_read_status_len;
static uint8_t clamp_cmd_target_len;

static uint8_t clamp_rsp_write_ack[CLAMP_UART_RX_MAX_LEN];
static uint8_t clamp_rsp_write_target_prefix[CLAMP_UART_RX_MAX_LEN];
static uint8_t clamp_rsp_status_not_ready[CLAMP_UART_RX_MAX_LEN];
static uint8_t clamp_rsp_status_ready[CLAMP_UART_RX_MAX_LEN];
static uint8_t clamp_rsp_write_ack_len;
static uint8_t clamp_rsp_write_target_prefix_len;
static uint8_t clamp_rsp_status_not_ready_len;
static uint8_t clamp_rsp_status_ready_len;

// UART接收缓冲：工作缓冲用于逐字节拼帧，frame缓冲用于任务上下文读取
static uint8_t clamp_uart_rx_byte;
static uint8_t clamp_uart_work_buf[CLAMP_UART_RX_MAX_LEN];
static uint8_t clamp_uart_frame_buf[CLAMP_UART_RX_MAX_LEN];
static volatile uint8_t clamp_uart_work_len;
static volatile uint8_t clamp_uart_frame_len;
static volatile uint8_t clamp_uart_frame_ready;
static volatile uint8_t clamp_uart_rx_armed;

static ClampInitState_e clamp_init_state;
static ClampCtrlState_e clamp_ctrl_state;
static uint32_t clamp_init_start_tick;
static uint32_t clamp_stage_deadline_tick;
static uint32_t clamp_poll_next_tick;

// 对外设置的目标量（由ClampSetTarget写入，由Clampcontrol读取）
static volatile uint8_t clamp_target_position;
static volatile uint8_t clamp_target_force;
static volatile uint8_t clamp_target_speed;
static volatile uint8_t clamp_target_pending;

// 当前等待ACK的目标量（用于超时重发）
static uint8_t clamp_active_position;
static uint8_t clamp_active_force;
static uint8_t clamp_active_speed;

// 标准Modbus RTU CRC16（多项式0xA001，低字节先发送）
static uint16_t ClampCalcCrc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001U) {
                crc = (crc >> 1) ^ 0xA001U;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

// 拼装读保持寄存器指令帧
static uint8_t ClampBuildReadRegsCmd(
    uint8_t slave_addr, uint16_t start_reg, uint16_t reg_count, uint8_t *out_buf, uint8_t out_buf_size)
{
    uint16_t crc;

    if (out_buf == NULL || out_buf_size < 8) {
        return 0;
    }

    out_buf[0] = slave_addr;
    out_buf[1] = CLAMP_MODBUS_FUNC_READ_HOLDING_REGS;
    out_buf[2] = (uint8_t)(start_reg >> 8);
    out_buf[3] = (uint8_t)(start_reg & 0xFF);
    out_buf[4] = (uint8_t)(reg_count >> 8);
    out_buf[5] = (uint8_t)(reg_count & 0xFF);

    crc = ClampCalcCrc16(out_buf, 6);
    out_buf[6] = (uint8_t)(crc & 0xFF);
    out_buf[7] = (uint8_t)(crc >> 8);

    return 8;
}

// 拼装写多个寄存器指令帧（功能码0x10）
static uint8_t ClampBuildWriteRegsCmd(
    uint8_t slave_addr,
    uint16_t start_reg,
    const uint16_t *reg_values,
    uint8_t reg_count,
    uint8_t *out_buf,
    uint8_t out_buf_size)
{
    uint8_t payload_len;
    uint8_t frame_no_crc_len;
    uint16_t crc;

    if (out_buf == NULL || reg_values == NULL || reg_count == 0) {
        return 0;
    }

    payload_len = (uint8_t)(reg_count * 2U);
    frame_no_crc_len = (uint8_t)(7U + payload_len);
    if (out_buf_size < (uint8_t)(frame_no_crc_len + 2U)) {
        return 0;
    }

    out_buf[0] = slave_addr;
    out_buf[1] = CLAMP_MODBUS_FUNC_WRITE_MULTI_REGS;
    out_buf[2] = (uint8_t)(start_reg >> 8);
    out_buf[3] = (uint8_t)(start_reg & 0xFF);
    out_buf[4] = 0;
    out_buf[5] = reg_count;
    out_buf[6] = payload_len;

    for (uint8_t i = 0; i < reg_count; i++) {
        out_buf[(uint8_t)(7U + 2U * i)] = (uint8_t)(reg_values[i] >> 8);
        out_buf[(uint8_t)(8U + 2U * i)] = (uint8_t)(reg_values[i] & 0xFF);
    }

    crc = ClampCalcCrc16(out_buf, frame_no_crc_len);
    out_buf[frame_no_crc_len] = (uint8_t)(crc & 0xFF);
    out_buf[(uint8_t)(frame_no_crc_len + 1U)] = (uint8_t)(crc >> 8);

    return (uint8_t)(frame_no_crc_len + 2U);// 总长度=无CRC长度+CRC长度
}

// 按协议字段动态生成初始化阶段命令帧
static uint8_t ClampBuildAllCommands(void)
{
    static const uint16_t clear_ract_regs[] = {0x0000};
    static const uint16_t set_ract_regs[] = {0x0001};

    clamp_cmd_clear_ract_len = ClampBuildWriteRegsCmd(
        CLAMP_MODBUS_SLAVE_ADDR,
        CLAMP_REG_ACTION_BASE,
        clear_ract_regs,
        (uint8_t)(sizeof(clear_ract_regs) / sizeof(clear_ract_regs[0])),
        clamp_cmd_clear_ract,
        sizeof(clamp_cmd_clear_ract));

    clamp_cmd_set_ract_len = ClampBuildWriteRegsCmd(
        CLAMP_MODBUS_SLAVE_ADDR,
        CLAMP_REG_ACTION_BASE,
        set_ract_regs,
        (uint8_t)(sizeof(set_ract_regs) / sizeof(set_ract_regs[0])),
        clamp_cmd_set_ract,
        sizeof(clamp_cmd_set_ract));

    clamp_cmd_read_status_len = ClampBuildReadRegsCmd(
        CLAMP_MODBUS_SLAVE_ADDR,
        CLAMP_REG_STATUS_BASE,
        1,
        clamp_cmd_read_status,
        sizeof(clamp_cmd_read_status));

    if (clamp_cmd_clear_ract_len == 0 || clamp_cmd_set_ract_len == 0 || clamp_cmd_read_status_len == 0) {
        return 0;
    }

    return 1;
}

// 生成目标控制命令帧（position/force/speed均为0~255）
static uint8_t ClampBuildTargetCommand(uint8_t position, uint8_t force, uint8_t speed, uint8_t *out_buf, uint8_t out_buf_size)
{
    uint16_t target_regs[3];

    target_regs[0] = 0x0009;
    target_regs[1] = (uint16_t)(((uint16_t)position << 8) | 0x00);
    target_regs[2] = (uint16_t)(((uint16_t)force << 8) | speed);

    return ClampBuildWriteRegsCmd(
        CLAMP_MODBUS_SLAVE_ADDR,
        CLAMP_REG_ACTION_BASE,
        target_regs,
        (uint8_t)(sizeof(target_regs) / sizeof(target_regs[0])),
        out_buf,
        out_buf_size);
}

// 生成功能码0x10应答帧（从站地址+功能码+起始地址+寄存器数量+CRC）
static uint8_t ClampBuildWriteRegsAck(
    uint8_t slave_addr, uint16_t start_reg, uint16_t reg_count, uint8_t *out_buf, uint8_t out_buf_size)
{
    uint16_t crc;

    if (out_buf == NULL || out_buf_size < 8) {
        return 0;
    }

    out_buf[0] = slave_addr;
    out_buf[1] = CLAMP_MODBUS_FUNC_WRITE_MULTI_REGS;
    out_buf[2] = (uint8_t)(start_reg >> 8);
    out_buf[3] = (uint8_t)(start_reg & 0xFF);
    out_buf[4] = (uint8_t)(reg_count >> 8);
    out_buf[5] = (uint8_t)(reg_count & 0xFF);

    crc = ClampCalcCrc16(out_buf, 6);
    out_buf[6] = (uint8_t)(crc & 0xFF);
    out_buf[7] = (uint8_t)(crc >> 8);

    return 8;
}

// 生成读保持寄存器应答帧（从站地址+功能码+字节数+数据+CRC）
static uint8_t ClampBuildReadRegsRsp(uint8_t slave_addr, uint16_t reg_value, uint8_t *out_buf, uint8_t out_buf_size)
{
    uint16_t crc;

    if (out_buf == NULL || out_buf_size < 7) {
        return 0;
    }

    out_buf[0] = slave_addr;
    out_buf[1] = CLAMP_MODBUS_FUNC_READ_HOLDING_REGS;
    out_buf[2] = 0x02;
    out_buf[3] = (uint8_t)(reg_value >> 8);
    out_buf[4] = (uint8_t)(reg_value & 0xFF);

    crc = ClampCalcCrc16(out_buf, 5);
    out_buf[5] = (uint8_t)(crc & 0xFF);
    out_buf[6] = (uint8_t)(crc >> 8);

    return 7;
}

// 按协议字段动态生成应答匹配模板
static uint8_t ClampBuildAllResponses(void)
{
    clamp_rsp_write_ack_len = ClampBuildWriteRegsAck(
        CLAMP_MODBUS_SLAVE_ADDR, CLAMP_REG_ACTION_BASE, 1, clamp_rsp_write_ack, sizeof(clamp_rsp_write_ack));

    clamp_rsp_status_not_ready_len = ClampBuildReadRegsRsp(
        CLAMP_MODBUS_SLAVE_ADDR, 0x0014, clamp_rsp_status_not_ready, sizeof(clamp_rsp_status_not_ready));

    clamp_rsp_status_ready_len = ClampBuildReadRegsRsp(
        CLAMP_MODBUS_SLAVE_ADDR, 0x0031, clamp_rsp_status_ready, sizeof(clamp_rsp_status_ready));

    if (sizeof(clamp_rsp_write_target_prefix) < 6U) {
        clamp_rsp_write_target_prefix_len = 0;
    } else {
        clamp_rsp_write_target_prefix[0] = CLAMP_MODBUS_SLAVE_ADDR;
        clamp_rsp_write_target_prefix[1] = CLAMP_MODBUS_FUNC_WRITE_MULTI_REGS;
        clamp_rsp_write_target_prefix[2] = (uint8_t)(CLAMP_REG_ACTION_BASE >> 8);
        clamp_rsp_write_target_prefix[3] = (uint8_t)(CLAMP_REG_ACTION_BASE & 0xFF);
        clamp_rsp_write_target_prefix[4] = 0x00;
        clamp_rsp_write_target_prefix[5] = 0x03;
        clamp_rsp_write_target_prefix_len = 6;
    }

    if (clamp_rsp_write_ack_len == 0 || clamp_rsp_write_target_prefix_len == 0 ||
        clamp_rsp_status_not_ready_len == 0 || clamp_rsp_status_ready_len == 0) {
        return 0;
    }

    return 1;
}

// 在接收帧中查找目标子帧，容忍前导/后缀噪声和粘包
static uint8_t ClampFrameContains(const uint8_t *data, uint8_t len, const uint8_t *expect, uint8_t expect_len)
{
    if (expect_len == 0 || len < expect_len) {
        return 0;
    }

    for (uint8_t i = 0; i <= (uint8_t)(len - expect_len); i++) {
        if (memcmp(data + i, expect, expect_len) == 0) {
            return 1;
        }
    }
    return 0;
}

// 统一的UART2发送封装
static void ClampSendFrame(const uint8_t *data, uint16_t len)
{
    UartSendTxMessage(&UART2, (uint8_t *)data, len, 10);
}

// 从中断上下文转移出来的一帧数据中取出完整帧（原子拷贝）
static uint8_t ClampTakeFrame(uint8_t *out_buf, uint8_t *out_len)
{
    if (clamp_uart_frame_ready == 0) {
        return 0;
    }

    __disable_irq();
    *out_len = clamp_uart_frame_len;
    if (*out_len > CLAMP_UART_RX_MAX_LEN) {
        *out_len = CLAMP_UART_RX_MAX_LEN;
    }
    memcpy(out_buf, clamp_uart_frame_buf, *out_len);
    clamp_uart_frame_ready = 0;
    clamp_uart_frame_len = 0;
    __enable_irq();

    return 1;
}

// 启动1字节中断接收；若已在接收中则不重复启动
static void ClampUartStartReceive(void)
{
    if (clamp_uart_rx_armed) {
        return;
    }

    clamp_uart_work_len = 0;
    if (HAL_UART_Receive_IT(&huart1, &clamp_uart_rx_byte, 1) == HAL_OK) {
        clamp_uart_rx_armed = 1;
    }
}

// 激活流程单步推进（由ClampInit循环调用）
static void ClampInitStep(void)
{
    uint8_t rx_buf[CLAMP_UART_RX_MAX_LEN];
    uint8_t rx_len = 0;
    uint32_t now_tick = HAL_GetTick();

    // 总超时保护：避免初始化长期阻塞系统
    if (now_tick - clamp_init_start_tick > CLAMP_INIT_TOTAL_TIMEOUT_MS) {
        clamp_init_state = CLAMP_INIT_SKIPPED;
        return;
    }

    switch (clamp_init_state) {
    case CLAMP_INIT_SEND_CLEAR:
        ClampSendFrame(clamp_cmd_clear_ract, clamp_cmd_clear_ract_len);
        clamp_stage_deadline_tick = now_tick + CLAMP_STAGE_TIMEOUT_MS;
        clamp_init_state = CLAMP_INIT_WAIT_CLEAR_ACK;
        break;

    case CLAMP_INIT_WAIT_CLEAR_ACK:
    {
        uint8_t ack_ok = 0;
        if (ClampTakeFrame(rx_buf, &rx_len)) {
            if (ClampFrameContains(rx_buf, rx_len, clamp_rsp_write_ack, clamp_rsp_write_ack_len)) {
                ack_ok = 1;
                clamp_init_state = CLAMP_INIT_SEND_SET;
            }
        }
        if (!ack_ok && now_tick >= clamp_stage_deadline_tick) {
            clamp_init_state = CLAMP_INIT_SEND_CLEAR;
        }
        break;
    }

    case CLAMP_INIT_SEND_SET:
        ClampSendFrame(clamp_cmd_set_ract, clamp_cmd_set_ract_len);
        clamp_stage_deadline_tick = now_tick + CLAMP_STAGE_TIMEOUT_MS;
        clamp_init_state = CLAMP_INIT_WAIT_SET_ACK;
        break;

    case CLAMP_INIT_WAIT_SET_ACK:
    {
        uint8_t ack_ok = 0;
        if (ClampTakeFrame(rx_buf, &rx_len)) {
            if (ClampFrameContains(rx_buf, rx_len, clamp_rsp_write_ack, clamp_rsp_write_ack_len)) {
                ack_ok = 1;
                clamp_poll_next_tick = now_tick;
                clamp_init_state = CLAMP_INIT_SEND_READ_STATUS;
            }
        }
        if (!ack_ok && now_tick >= clamp_stage_deadline_tick) {
            clamp_init_state = CLAMP_INIT_SEND_SET;
        }
        break;
    }

    case CLAMP_INIT_SEND_READ_STATUS:
        if (now_tick >= clamp_poll_next_tick) {
            ClampSendFrame(clamp_cmd_read_status, clamp_cmd_read_status_len);
            clamp_stage_deadline_tick = now_tick + CLAMP_STAGE_TIMEOUT_MS;
            clamp_init_state = CLAMP_INIT_WAIT_STATUS;
        }
        break;

    case CLAMP_INIT_WAIT_STATUS:
    {
        uint8_t status_ok = 0;
        if (ClampTakeFrame(rx_buf, &rx_len)) {
            // 收到0x31状态，表示夹爪已激活，初始化完成
            if (ClampFrameContains(rx_buf, rx_len, clamp_rsp_status_ready, clamp_rsp_status_ready_len)) {
                status_ok = 1;
                clamp_init_state = CLAMP_INIT_DONE;
            } else if (ClampFrameContains(rx_buf, rx_len, clamp_rsp_status_not_ready, clamp_rsp_status_not_ready_len)) {
                status_ok = 1;
                clamp_poll_next_tick = now_tick + CLAMP_STATUS_POLL_INTERVAL_MS;
                clamp_init_state = CLAMP_INIT_SEND_READ_STATUS;
            }
        }
        if (!status_ok && now_tick >= clamp_stage_deadline_tick) {
            clamp_poll_next_tick = now_tick + CLAMP_STATUS_POLL_INTERVAL_MS;
            clamp_init_state = CLAMP_INIT_SEND_READ_STATUS;
        }
        break;
    }

    case CLAMP_INIT_DONE:
    case CLAMP_INIT_SKIPPED:
    default:
        break;
    }

    ClampUartStartReceive();
}

// UART接收完成回调：逐字节拼接到工作缓冲，满16字节即封帧
void ClampUartOnReceiveComplete(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART1) {
        return;
    }

    if (clamp_uart_work_len < CLAMP_UART_RX_MAX_LEN) {
        clamp_uart_work_buf[clamp_uart_work_len++] = clamp_uart_rx_byte;
    }

    if (clamp_uart_work_len >= CLAMP_UART_RX_MAX_LEN) {
        memcpy(clamp_uart_frame_buf, clamp_uart_work_buf, CLAMP_UART_RX_MAX_LEN);
        clamp_uart_frame_len = CLAMP_UART_RX_MAX_LEN;
        clamp_uart_frame_ready = 1;
        clamp_uart_work_len = 0;
        clamp_uart_rx_armed = 0;
    } else {
        if (HAL_UART_Receive_IT(&huart1, &clamp_uart_rx_byte, 1) == HAL_OK) {
            clamp_uart_rx_armed = 1;
        }
    }
}

// 更新目标缓存；由Clampcontrol异步读取并下发
void ClampSetTarget(uint8_t position, uint8_t force, uint8_t speed)
{
    __disable_irq();
    clamp_target_position = position;
    clamp_target_force = force;
    clamp_target_speed = speed;
    clamp_target_pending = 1;
    __enable_irq();
}

// 夹爪初始化入口：离开本函数前，激活流程已完成或超时跳过
void ClampInit(void)
{
    if (ClampBuildAllCommands() == 0 || ClampBuildAllResponses() == 0) {
        clamp_init_state = CLAMP_INIT_SKIPPED;
        return;
    }

    clamp_uart_work_len = 0;
    clamp_uart_frame_len = 0;
    clamp_uart_frame_ready = 0;
    clamp_uart_rx_armed = 0;

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    ClampUartStartReceive();

    clamp_init_start_tick = HAL_GetTick();
    clamp_stage_deadline_tick = clamp_init_start_tick;
    clamp_poll_next_tick = clamp_init_start_tick;
    clamp_ctrl_state = CLAMP_CTRL_IDLE;
    clamp_init_state = CLAMP_INIT_SEND_CLEAR;

    // 初始化完成后默认目标设为中位，避免上电后先突发到0x00（全开端）。
    clamp_target_position = 0x00;
    clamp_target_force = 0xFF;
    clamp_target_speed = 0xFF;
    // 初始化完成后先下发一帧默认目标，避免控制状态机在IDLE长期空转。
    clamp_target_pending = 1;
    clamp_active_position = 0x00;
    clamp_active_force = 0xFF;
    clamp_active_speed = 0xFF;

    while (clamp_init_state != CLAMP_INIT_DONE && clamp_init_state != CLAMP_INIT_SKIPPED) {
        ClampInitStep();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// 初始化后的唯一控制入口：读取目标缓存并发送控制指令
void Clampcontrol(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint8_t rx_buf[CLAMP_UART_RX_MAX_LEN];
    uint8_t rx_len = 0;

    // if (clamp_init_state != CLAMP_INIT_DONE && clamp_init_state != CLAMP_INIT_SKIPPED) {
    //     return;
    // }

    switch (clamp_ctrl_state) {
    case CLAMP_CTRL_IDLE:
    {

        uint8_t target_position;
        uint8_t target_force;
        uint8_t target_speed;

        if (clamp_target_pending == 0) {
            break;
        }
        
        __disable_irq();
        target_position = clamp_target_position;
        target_force = clamp_target_force;
        target_speed = clamp_target_speed;
        clamp_target_pending = 0;
        __enable_irq();

        clamp_cmd_target_len = ClampBuildTargetCommand(target_position, target_force, target_speed, clamp_cmd_target, sizeof(clamp_cmd_target));
        // 如果命令帧生成失败（极端情况），则放弃本次目标更新，等待下一次目标设置
        if (clamp_cmd_target_len == 0) {
            break;
        }

        clamp_active_position = target_position;
        clamp_active_force = target_force;
        clamp_active_speed = target_speed;

        ClampSendFrame(clamp_cmd_target, clamp_cmd_target_len);
        
        clamp_stage_deadline_tick = now_tick + CLAMP_CTRL_ACK_TIMEOUT_MS;
        clamp_ctrl_state = CLAMP_CTRL_WAIT_ACK;
        break;
    }

    case CLAMP_CTRL_WAIT_ACK:
    {
        uint8_t target_position;
        uint8_t target_force;
        uint8_t target_speed;

        // 即使旧目标ACK未到，也允许新目标抢占，避免状态机长期停在WAIT_ACK。
        if (clamp_target_pending != 0) {
            __disable_irq();
            target_position = clamp_target_position;
            target_force = clamp_target_force;
            target_speed = clamp_target_speed;
            clamp_target_pending = 0;
            __enable_irq();

            clamp_cmd_target_len = ClampBuildTargetCommand(
                target_position,
                target_force,
                target_speed,
                clamp_cmd_target,
                sizeof(clamp_cmd_target));
            if (clamp_cmd_target_len != 0) {
                clamp_active_position = target_position;
                clamp_active_force = target_force;
                clamp_active_speed = target_speed;
                ClampSendFrame(clamp_cmd_target, clamp_cmd_target_len);
                clamp_stage_deadline_tick = now_tick + CLAMP_CTRL_ACK_TIMEOUT_MS;
            }
        }

        if (ClampTakeFrame(rx_buf, &rx_len)) {
            if (ClampFrameContains(rx_buf, rx_len, clamp_rsp_write_target_prefix, clamp_rsp_write_target_prefix_len)) {
                clamp_ctrl_state = CLAMP_CTRL_IDLE;
            }
        }

        if (now_tick >= clamp_stage_deadline_tick) {
            clamp_cmd_target_len = ClampBuildTargetCommand(
                clamp_active_position,
                clamp_active_force,
                clamp_active_speed,
                clamp_cmd_target,
                sizeof(clamp_cmd_target));
            if (clamp_cmd_target_len != 0) {
                ClampSendFrame(clamp_cmd_target, clamp_cmd_target_len);
                clamp_stage_deadline_tick = now_tick + CLAMP_CTRL_ACK_TIMEOUT_MS;
            }
        }
        break;
    }

    default:
        clamp_ctrl_state = CLAMP_CTRL_IDLE;
        break;
    }

    ClampUartStartReceive();
}

// HAL UART接收完成回调入口
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ClampUartOnReceiveComplete(huart);
}

// UART1中断：利用IDLE中断作为变长输入帧结束标志
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        if (clamp_uart_work_len > 0 && clamp_uart_work_len < CLAMP_UART_RX_MAX_LEN) {
            memcpy(clamp_uart_frame_buf, clamp_uart_work_buf, clamp_uart_work_len);
            clamp_uart_frame_len = clamp_uart_work_len;
            clamp_uart_frame_ready = 1;
            clamp_uart_work_len = 0;
        }
    }

    HAL_UART_IRQHandler(&huart1);
}
