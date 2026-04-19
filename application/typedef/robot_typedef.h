#ifndef ROBOT_TYPEDEF_H
#define ROBOT_TYPEDEF_H

// clang-format off
// 可用调参模式
#define TUNING_NONE     0
#define TUNING_CHASSIS  1

// 自定义控制器类型
#define CC_RECEIVER 0  // 接收器
#define CC_SENDER   1  // 发送器

// 遥控器类型
#define RC_DT7      0  // DT7遥控器
#define RC_AT9S_PRO 1  // AT9S PRO遥控器
#define RC_HT8A     2  // HT8A遥控器
#define RC_ET08A    3  // ET08A遥控器

// C板id
#define C_BOARD_DEFAULT                  1  // C板默认id
#define C_BOARD_BALANCE_CHASSIS          2  // 平衡底盘C板

// 控制链路相关
#define CL_RC_NONE           0x100  // 无遥控器链路
#define CL_RC_DIRECT         0x101  // 直连遥控器（通过dbus口获取直接的遥控器数据）
#define CL_RC_CAN            0x102  // 通过CAN口获取遥控器数据
#define CL_RC_UART2          0x103  // 通过UART2口获取遥控器数据

#define CL_KM_NONE     0x200  // 无键鼠数据
#define CL_KM_RC       0x201  // 仅通过遥控器获取键鼠数据
#define CL_KM_VT       0x202  // 仅通过图传链路获取键鼠数据(Video Transmission)
#define CL_KM_RC_VT    0x203  // 同时使用图传链路和遥控器获取键鼠数据（互补操作，遥控器优先）
#define CL_KM_VT_RC    0x204  // 同时使用图传链路和遥控器获取键鼠数据（互补操作，图传链路优先）

#define CL_PS2_NONE     0x300  // 无PS2链路
#define CL_PS2_DIRECT   0x301  // PS2直接连接（通过 8pin 自定义io口获取直接的PS2数据）
#define CL_PS2_CAN      0x302  // 通过CAN口获取PS2数据
#define CL_PS2_UART2    0x303  // 通过UART2口获取PS2数据

// 虚拟云台数据源
#define VG_FROM_NONE      0x000 // 无数据源
#define VG_FROM_UART2     0x001 // uart2串口数据源
#define VG_FROM_CAN       0x002 // can口数据源
#define VG_FROM_YAW_MOTOR 0x003 // 直接使用yaw电机数据

// 可用电机类型
typedef enum __MotorType {
    MF_9025,
    MG_6012,
    MG_5010,
    CYBERGEAR_MOTOR,
} MotorType_e;
// clang-format on

#endif /* ROBOT_TYPEDEF_H */
/*------------------------------ End of File ------------------------------*/
