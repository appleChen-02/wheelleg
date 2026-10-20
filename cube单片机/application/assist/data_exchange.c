/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       data_exchange.c/h
  * @brief      数据交换中心，用于各个模块之间的数据交换，不用相互调用头文件，加强各模块之间的解耦
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-22-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  @todo 
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "data_exchange.h"

#include "FreeRTOS.h"
#include "chassis_balance.h"
#include "string.h"
#include "task.h"

typedef struct
{
    Fdb_t fdb;
    Ref_t ref;
    uint32_t seq;
} ChassisSnapshot_t;

// 双缓冲槽位: front 对外可读, back 由写端更新
static ChassisSnapshot_t CHASSIS_SNAPSHOT_BUFFER[2] = {0};
static volatile uint8_t CHASSIS_FRONT_INDEX = 0;
static volatile uint8_t CHASSIS_SNAPSHOT_READY = 0;
static volatile uint32_t CHASSIS_SNAPSHOT_SEQ = 0;

#define SNAPSHOT_READ_RETRY_MAX 3u



// IMU同样采用双缓冲: 写端更新 back, 读端读取 front
static Imu_t IMU_SNAPSHOT_BUFFER[2] = {0};
static volatile uint8_t IMU_FRONT_INDEX = 0;
static volatile uint8_t IMU_SNAPSHOT_READY = 0;

void ChassisSnapshotInit(void)
{
    // 上电/模式切换时重置快照状态
    memset(CHASSIS_SNAPSHOT_BUFFER, 0, sizeof(CHASSIS_SNAPSHOT_BUFFER));
    CHASSIS_FRONT_INDEX = 0;
    CHASSIS_SNAPSHOT_READY = 0;
    CHASSIS_SNAPSHOT_SEQ = 0;
}

uint8_t ChassisSnapshotPublish(const void * fdb, const void * ref)
{
    if ((fdb == NULL) || (ref == NULL)) {
        return CHASSIS_SNAPSHOT_FAIL;
    }

    uint8_t back_index = (uint8_t)(CHASSIS_FRONT_INDEX ^ 0x01u);
    ChassisSnapshot_t * back = &CHASSIS_SNAPSHOT_BUFFER[back_index];

    // 先完整写 back 槽, 避免读端看到半更新数据
    memcpy(&back->fdb, fdb, sizeof(Fdb_t));
    memcpy(&back->ref, ref, sizeof(Ref_t));
    back->seq = CHASSIS_SNAPSHOT_SEQ + 1u;

    // 临界区仅用于 front 索引切换, 缩短关中断时间
    taskENTER_CRITICAL();
    CHASSIS_SNAPSHOT_SEQ = back->seq;
    CHASSIS_FRONT_INDEX = back_index;
    CHASSIS_SNAPSHOT_READY = 1;
    taskEXIT_CRITICAL();

    return CHASSIS_SNAPSHOT_OK;
}

void ImuSnapshotInit(void)
{
    memset(IMU_SNAPSHOT_BUFFER, 0, sizeof(IMU_SNAPSHOT_BUFFER));
    IMU_FRONT_INDEX = 0;
    IMU_SNAPSHOT_READY = 0;
}

uint8_t ImuSnapshotPublish(const Imu_t * imu)
{
    if (imu == NULL) {
        return CHASSIS_SNAPSHOT_FAIL;
    }

    uint8_t back_index = (uint8_t)(IMU_FRONT_INDEX ^ 0x01u);
    memcpy(&IMU_SNAPSHOT_BUFFER[back_index], imu, sizeof(Imu_t));

    // IMU切换也使用短临界区, 避免front索引竞争
    taskENTER_CRITICAL();
    IMU_FRONT_INDEX = back_index;
    IMU_SNAPSHOT_READY = 1;
    taskEXIT_CRITICAL();

    return CHASSIS_SNAPSHOT_OK;
}

uint8_t ImuSnapshotRead(Imu_t * imu_out)
{
    if (imu_out == NULL) {
        return CHASSIS_SNAPSHOT_FAIL;
    }
    if (IMU_SNAPSHOT_READY == 0) {
        return CHASSIS_SNAPSHOT_NOT_READY;
    }

    uint8_t front_index = IMU_FRONT_INDEX;
    memcpy(imu_out, &IMU_SNAPSHOT_BUFFER[front_index], sizeof(Imu_t));
    return CHASSIS_SNAPSHOT_OK;
}

uint8_t ImuSnapshotIsReady(void)
{
    return IMU_SNAPSHOT_READY;
}

uint8_t ChassisSnapshotRead(
    void * fdb_out, uint32_t fdb_size, void * ref_out, uint32_t ref_size, uint32_t * seq)
{
    if ((fdb_out == NULL) || (ref_out == NULL)) {
        return CHASSIS_SNAPSHOT_FAIL;
    }
    if ((fdb_size != sizeof(Fdb_t)) || (ref_size != sizeof(Ref_t))) {
        return CHASSIS_SNAPSHOT_SIZE_ERROR;
    }
    if (CHASSIS_SNAPSHOT_READY == 0) {
        return CHASSIS_SNAPSHOT_NOT_READY;
    }

    for (uint8_t retry = 0; retry < SNAPSHOT_READ_RETRY_MAX; retry++) {
        uint32_t seq_before = CHASSIS_SNAPSHOT_SEQ;
        uint8_t front_index = CHASSIS_FRONT_INDEX;
        const ChassisSnapshot_t * front = &CHASSIS_SNAPSHOT_BUFFER[front_index];

        memcpy(fdb_out, &front->fdb, sizeof(Fdb_t));
        memcpy(ref_out, &front->ref, sizeof(Ref_t));
        uint32_t front_seq = front->seq;

        uint32_t seq_after = CHASSIS_SNAPSHOT_SEQ;
        uint8_t front_index_after = CHASSIS_FRONT_INDEX;

        if ((seq_before == seq_after) && (front_index == front_index_after)) {
            if (seq != NULL) {
                *seq = front_seq;
            }
            return CHASSIS_SNAPSHOT_OK;
        }
    }

    return CHASSIS_SNAPSHOT_FAIL;
}

uint8_t ChassisSnapshotReadSpeedVector(
    ChassisSpeedVector_t * fdb_speed, ChassisSpeedVector_t * ref_speed, uint32_t * seq)
{
    if ((fdb_speed == NULL) || (ref_speed == NULL)) {
        return CHASSIS_SNAPSHOT_FAIL;
    }
    if (CHASSIS_SNAPSHOT_READY == 0) {
        return CHASSIS_SNAPSHOT_NOT_READY;
    }

    for (uint8_t retry = 0; retry < SNAPSHOT_READ_RETRY_MAX; retry++) {
        uint32_t seq_before = CHASSIS_SNAPSHOT_SEQ;
        uint8_t front_index = CHASSIS_FRONT_INDEX;
        const ChassisSnapshot_t * front = &CHASSIS_SNAPSHOT_BUFFER[front_index];

        memcpy(fdb_speed, &front->fdb.speed_vector, sizeof(ChassisSpeedVector_t));
        memcpy(ref_speed, &front->ref.speed_vector, sizeof(ChassisSpeedVector_t));
        uint32_t front_seq = front->seq;

        uint32_t seq_after = CHASSIS_SNAPSHOT_SEQ;
        uint8_t front_index_after = CHASSIS_FRONT_INDEX;

        if ((seq_before == seq_after) && (front_index == front_index_after)) {
            if (seq != NULL) {
                *seq = front_seq;
            }
            return CHASSIS_SNAPSHOT_OK;
        }
    }

    return CHASSIS_SNAPSHOT_FAIL;
}

