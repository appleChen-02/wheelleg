/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       data_exchange.c/h
  * @brief      数据交换中心，用于各个模块之间的数据交换，不用相互调用头文件，加强各模块之间的解耦
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-16-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef __DATA_EXCHANGE_H
#define __DATA_EXCHANGE_H

#include "struct_typedef.h"
#include "custom_typedef.h"

/**
 * @brief 双缓冲读写状态码
 */
typedef enum ChassisSnapshotStatus {
    CHASSIS_SNAPSHOT_FAIL = 0,
    CHASSIS_SNAPSHOT_OK,
    CHASSIS_SNAPSHOT_NOT_READY,
    CHASSIS_SNAPSHOT_SIZE_ERROR
} ChassisSnapshotStatus_e;

/**
 * @brief 初始化底盘状态/目标双缓冲
 */
extern void ChassisSnapshotInit(void);

/**
 * @brief 发布一帧底盘状态与目标快照
 * @param[in] fdb 当前状态量，类型需为 Fdb_t
 * @param[in] ref 当前目标量，类型需为 Ref_t
 * @retval ChassisSnapshotStatus_e
 */
extern uint8_t ChassisSnapshotPublish(const void * fdb, const void * ref);

/**
 * @brief 读取完整底盘快照（同一front槽位的一致帧）
 * @param[out] fdb_out 状态量输出缓存
 * @param[in] fdb_size fdb_out 缓存大小，必须为 sizeof(Fdb_t)
 * @param[out] ref_out 目标量输出缓存
 * @param[in] ref_size ref_out 缓存大小，必须为 sizeof(Ref_t)
 * @param[out] seq 快照序号，可传 NULL
 * @retval ChassisSnapshotStatus_e
 * @note 读期间若front槽位切换，会自动重试；超过重试次数返回CHASSIS_SNAPSHOT_FAIL
 */
extern uint8_t ChassisSnapshotRead(
    void * fdb_out, uint32_t fdb_size, void * ref_out, uint32_t ref_size, uint32_t * seq);

/**
 * @brief 读取底盘反馈/目标速度向量（同一front槽位的一致帧）
 * @param[out] fdb_speed 反馈速度向量
 * @param[out] ref_speed 目标速度向量
 * @param[out] seq 快照序号，可传 NULL
 * @retval ChassisSnapshotStatus_e
 * @note 读期间若front槽位切换，会自动重试；超过重试次数返回CHASSIS_SNAPSHOT_FAIL
 */
extern uint8_t ChassisSnapshotReadSpeedVector(
    ChassisSpeedVector_t * fdb_speed, ChassisSpeedVector_t * ref_speed, uint32_t * seq);

/**
 * @brief 初始化IMU双缓冲
 */
extern void ImuSnapshotInit(void);

/**
 * @brief 发布一帧IMU数据
 * @param[in] imu IMU数据指针，类型为 Imu_t
 * @retval ChassisSnapshotStatus_e
 */
extern uint8_t ImuSnapshotPublish(const Imu_t * imu);

/**
 * @brief 读取最新IMU数据
 * @param[out] imu_out 输出缓存
 * @retval ChassisSnapshotStatus_e
 */
extern uint8_t ImuSnapshotRead(Imu_t * imu_out);

/**
 * @brief 查询IMU双缓冲是否已就绪
 * @retval 1-已就绪 0-未就绪
 */
extern uint8_t ImuSnapshotIsReady(void);

#endif  // __DATA_EXCHANGE_H
