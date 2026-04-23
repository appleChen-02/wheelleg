# USB Solved RC Command Protocol

本文档说明板端通过 USB CDC 反向发送给上位机的“遥控器解算后控制目标量”数据帧。

## Frame Format

所有 USB 数据帧沿用工程现有格式：

| Offset | Size | Type | Name | Description |
|---:|---:|---|---|---|
| 0 | 1 | uint8 | sof | 固定 `0x5A` |
| 1 | 1 | uint8 | len | 数据段长度，不含 4 字节帧头和 2 字节 CRC16 |
| 2 | 1 | uint8 | id | 数据 ID |
| 3 | 1 | uint8 | crc8 | 对 offset 0-2 计算 CRC8 |
| 4 | len | bytes | payload | 数据段 |
| 4 + len | 2 | uint16 | crc16 | 对整帧前 `4 + len` 字节计算 CRC16，低字节在前 |

大小端：所有 `uint16_t`、`uint32_t`、`float` 均为小端。  
浮点格式：单精度 IEEE754 float。

## New Data ID

| Direction | ID | Name | Total Size | len |
|---|---:|---|---:|---:|
| board -> PC | `0x0C` | `SOLVED_RC_CMD_DATA_SEND_ID` | 58 bytes | 52 bytes |

## Payload Layout

`id = 0x0C` 的 payload 为：

| Frame Offset | Payload Offset | Size | Type | Name | Unit | Description |
|---:|---:|---:|---|---|---|---|
| 4 | 0 | 4 | uint32 | time_stamp | ms | `HAL_GetTick()` |
| 8 | 4 | 1 | uint8 | mode | - | 底盘模式，来自 `CHASSIS.mode` |
| 9 | 5 | 1 | uint8 | step | - | 底盘步态/流程阶段，来自 `CHASSIS.step` |
| 10 | 6 | 1 | uint8 | rc_offline | bool | `1` 表示遥控器离线 |
| 11 | 7 | 1 | uint8 | reserved | - | 预留，目前为 0 |
| 12 | 8 | 4 | float | vx | m/s | 解算后的目标 x 方向速度 |
| 16 | 12 | 4 | float | vy | m/s | 解算后的目标 y 方向速度 |
| 20 | 16 | 4 | float | wz | rad/s | 解算后的目标 yaw 角速度 |
| 24 | 20 | 4 | float | roll | rad | 解算后的目标 roll |
| 28 | 24 | 4 | float | pitch | rad | 解算后的目标 pitch |
| 32 | 28 | 4 | float | yaw | rad | 由 `wz` 积分得到的目标 yaw |
| 36 | 32 | 4 | float | leg_length_l | m | 左腿目标腿长 |
| 40 | 36 | 4 | float | leg_length_r | m | 右腿目标腿长 |
| 44 | 40 | 4 | float | leg_angle_l | rad | 左腿目标腿角 |
| 48 | 44 | 4 | float | leg_angle_r | rad | 右腿目标腿角 |
| 52 | 48 | 4 | float | tail_beta | rad | 尾巴目标角度 |
| 56 | - | 2 | uint16 | crc16 | - | 整包 CRC16 |

## Receiver Parsing Steps

1. 在字节流中查找 `sof == 0x5A`。
2. 读取 `len`、`id`、`crc8`。
3. 对前 3 字节计算 CRC8，确认结果等于 offset 3 的 `crc8`。
4. 若 `id == 0x0C`，期望 `len == 52`，整包长度为 `4 + 52 + 2 = 58`。
5. 对整包前 56 字节计算 CRC16，确认结果等于最后 2 字节。CRC16 低字节在前。
6. 按上表解析 payload。

## Packed C Struct Example

```c
#pragma pack(push, 1)
typedef struct
{
    uint8_t sof;
    uint8_t len;
    uint8_t id;
    uint8_t crc8;
} UsbFrameHeader_t;

typedef struct
{
    uint8_t mode;
    uint8_t step;
    uint8_t rc_offline;
    uint8_t reserved;

    float vx;
    float vy;
    float wz;
    float roll;
    float pitch;
    float yaw;
    float leg_length_l;
    float leg_length_r;
    float leg_angle_l;
    float leg_angle_r;
    float tail_beta;
} UsbSolvedRcCmdPayload_t;

typedef struct
{
    UsbFrameHeader_t header;
    uint32_t time_stamp;
    UsbSolvedRcCmdPayload_t data;
    uint16_t crc16;
} UsbSolvedRcCmdFrame_t;
#pragma pack(pop)
```

## Notes

- `wz` 是当前控制器直接使用的目标 yaw 角速度。
- `yaw` 是由 `wz * dt` 积分得到的目标航向角，可用于上位机观察目标朝向。
- 本帧发送周期当前为 10 ms。
- 本帧不包含遥控器原始通道值；原始通道值仍属于 `RC_ctrl_t` 语义。
