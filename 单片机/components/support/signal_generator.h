/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       signal_generator.c/h
  * @brief      信号发生器，用于生成测试的各种信号.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2024     Penguin         1. 添加
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef __SIGNAL_GENERATOR_H
#define __SIGNAL_GENERATOR_H

#include <stdint.h>
#include <stddef.h>

extern float GenerateSinWave(float amplitude, float offset, float period);

extern float GenerateStepWave(float a0, float a1, float t0);

extern float GenerateRampWave(float a0, float a1, float t0, float t1);

extern float GeneratePulseWave(float a1, float a2, float t1, float t2);

extern float GenerateSawtoothWave(float a0, float a1, float T);

/**
 * @brief 生成正弦扫频（线性chirp）
 * @param amplitude 振幅
 * @param offset 输出偏置
 * @param start_frequency 起始频率 (Hz)
 * @param end_frequency 结束频率 (Hz)
 * @param duration 扫频时长 (s)
 * @param elapsed_time 当前扫频时间 (s)
 */
extern float GenerateSinSweepWave(float amplitude, float offset, float start_frequency,
                                  float end_frequency, float duration, float elapsed_time);

typedef struct
{
    uint16_t lfsr;
    uint32_t hold_count;
} PrbsGenerator_t;

/**
 * @brief 初始化PRBS发生器
 * @param generator 发生器状态
 * @param seed 非零LFSR初值
 */
extern void PrbsGeneratorInit(PrbsGenerator_t * generator, uint16_t seed);

/**
 * @brief 生成PRBS信号
 * @param generator 发生器状态
 * @param amplitude 振幅，输出为 offset +/- amplitude
 * @param offset 输出偏置
 * @param hold_samples 每个伪随机码保持的采样周期数
 */
extern float GeneratePrbsWave(PrbsGenerator_t * generator, float amplitude, float offset,
                              uint32_t hold_samples);

#endif  // __SIGNAL_GENERATOR_H
        /************************ END OF FILE ************************/
