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

#include "signal_generator.h"

#include "math.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

/**
 * @brief          正弦信号发生器
 * @param[in]      amplitude 振幅
 * @param[in]      offset 输出偏移量
 * @param[in]      period (s)周期
 * @return         SinWave
 */
float GenerateSinWave(float amplitude, float offset, float period)
{
    float t = HAL_GetTick() / 1000.0f;
    return amplitude * sin((2 * M_PI / period) * t) + offset;
}

/**
 * @brief          阶跃信号发生器
 * @param[in]      a0 初始值
 * @param[in]      a1 最终值
 * @param[in]      t0 (s)起始时间
 * @return         StepWave
 */
float GenerateStepWave(float a0, float a1, float t0)
{
    float t = HAL_GetTick() / 1000.0f;
    return (t >= t0) ? a1 : a0;
}

/**
 * @brief          生成斜坡波信号
 * @param[in]      a0 信号的初始幅度
 * @param[in]      a1 信号的最终幅度
 * @param[in]      t0 (s)信号的起始时间
 * @param[in]      t1 (s)信号的结束时间
 * @return         当前时间下的信号幅度
 */
float GenerateRampWave(float a0, float a1, float t0, float t1)
{
    float t = HAL_GetTick() / 1000.0f;
    return (t >= t0 && t <= t1) ? a0 + (a1 - a0) * (t - t0) / (t1 - t0) : a0;
}

/**
 * @brief          生成脉冲波形信号
 * @param[in]      a1 脉冲波形的幅值1
 * @param[in]      a2 脉冲波形的幅值2
 * @param[in]      t1 (s)脉冲幅值1的持续时间
 * @param[in]      t2 (s)脉冲幅值2的持续时间
 * @return         当前时间在脉冲波形时间范围内时返回幅值1，否则返回幅值0
 */
float GeneratePulseWave(float a1, float a2, float t1, float t2)
{
    uint32_t t = HAL_GetTick();
    uint32_t t1_ = t1 * 1000;
    uint32_t t2_ = t2 * 1000;
    t = t % (t1_ + t2_);
    return (t < t1_) ? a1 : a2;
}

/**
 * @brief          生成锯齿波形
 * @param[in]      a0 波形起始值
 * @param[in]      a1 波形结束值
 * @param[in]      T (s)波形周期
 * @return         生成的锯齿波形值
 */
float GenerateSawtoothWave(float a0, float a1, float T)
{
    float t = HAL_GetTick() / 1000.0f;
    return a0 + (a1 - a0) * (t / T - floor(t / T));
}

/**
 * @brief          生成正弦扫频（线性chirp）
 * @param[in]      amplitude 振幅
 * @param[in]      offset 输出偏移量
 * @param[in]      start_frequency 起始频率 (Hz)
 * @param[in]      end_frequency 结束频率 (Hz)
 * @param[in]      duration 扫频时长 (s)
 * @param[in]      elapsed_time 当前扫频时间 (s)
 * @return         当前时刻的扫频信号
 */
float GenerateSinSweepWave(float amplitude, float offset, float start_frequency,
                           float end_frequency, float duration, float elapsed_time)
{
    if (duration <= 0.0f) {
        return offset;
    }

    float t = elapsed_time;
    if (t < 0.0f) t = 0.0f;
    if (t > duration) t = duration;

    const float frequency_slope = (end_frequency - start_frequency) / duration;
    const float phase = 2.0f * (float)M_PI *
                        (start_frequency * t + 0.5f * frequency_slope * t * t);
    return amplitude * sinf(phase) + offset;
}

/**
 * @brief          初始化PRBS发生器
 */
void PrbsGeneratorInit(PrbsGenerator_t * generator, uint16_t seed)
{
    if (generator == NULL) return;
    generator->lfsr = (seed == 0U) ? 0xACE1U : seed;
    generator->hold_count = 0U;
}

/**
 * @brief          生成PRBS信号。采用x^16+x^14+x^13+x^11+1多项式。
 */
float GeneratePrbsWave(PrbsGenerator_t * generator, float amplitude, float offset,
                       uint32_t hold_samples)
{
    if (generator == NULL) return offset;
    if (hold_samples == 0U) hold_samples = 1U;

    const float output = ((generator->lfsr & 0x0001U) != 0U) ? amplitude : -amplitude;
    generator->hold_count++;
    if (generator->hold_count >= hold_samples) {
        const uint16_t lsb = generator->lfsr & 1U;
        generator->lfsr >>= 1U;
        if (lsb != 0U) generator->lfsr ^= 0xB400U;
        if (generator->lfsr == 0U) generator->lfsr = 0xACE1U;
        generator->hold_count = 0U;
    }
    return output + offset;
}

/************************ END OF FILE ************************/
