#ifndef __CUSTOM_TYPEDEF_H
#define __CUSTOM_TYPEDEF_H

#include "stdbool.h"

typedef struct __Imu
{
    float angle[3];  // rad 欧拉角数据
    float gyro[3];   // rad/s 陀螺仪数据
    float accel[3];  // m/s^2 加速度计数据
} Imu_t;

typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;

typedef struct
{
    ChassisSpeedVector_t speed_vector;
    struct
    {
        float roll;
        float pitch;
        float yaw;
        float leg_length;
    } chassis;

    struct
    {
        bool fire;
        bool fric_on;
    } shoot;

} RobotCmdData_t;

#endif  // __CUSTOM_TYPEDEF_H
