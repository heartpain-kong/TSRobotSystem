/* ============================================================================
 * bsp_pwm.h — PWM 通用接口(仿 cod-h7-template-main/BSP/bsp_pwm 与
 * 说明 / 用法 / 坑 见同目录 bsp_pwm.md
 * ========================================================================== */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 本层约定的计数器频率(1MHz),见文件头 */
#define BSP_PWM_COUNTER_HZ      1000000u

/**
 * @brief 让某个 TIM 通道以 1MHz 计数器运行并启动 PWM 输出
 * @param htim             TIM 句柄(如 &htim3)
 * @param channel          通道(如 TIM_CHANNEL_4)
 * @param timer_clock_hz   该 TIM 的计数时钟(Hz)。APBx 预分频 ≠ 1 时,
 *                         定时器时钟 = 2 × PCLKx;不确定就传 160000000(本工程 APB1/APB2 定时器都是 160MHz)
 * @return true = 成功
 * @note 内部会:设 PSC 使计数器 = 1MHz → 设 ARR 为默认 1kHz → HAL_TIM_PWM_Init → Start
 */
bool BSP_PWM_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timer_clock_hz);

/**
 * @brief 启动/停止某个通道的 PWM 输出
 * @param htim    TIM 句柄
 * @param channel 通道
 * @return true = 成功
 */
bool BSP_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
bool BSP_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 设置频率(只改 ARR;计数器已由 BSP_PWM_Init 定在 1MHz)
 * @param htim TIM 句柄
 * @param freq_hz 频率(Hz),限幅到 [20, 20000];传 0 = 停止输出(占空比置 0)
 * @return true = 成功
 * @note ⚠️ 改频率会改变周期,占空比也随之变化 —— 需要保持响度就重设一次 compare
 */
bool BSP_PWM_Set_Freq(TIM_HandleTypeDef *htim, uint32_t freq_hz);

/**
 * @brief 设置占空比(0.0 ~ 1.0)
 * @param htim    TIM 句柄
 * @param channel 通道
 * @param duty    占空比(0~1,超出会限幅)
 * @return true = 成功
 */
bool BSP_PWM_Set_Duty(TIM_HandleTypeDef *htim, uint32_t channel, float duty);

/**
 * @brief 直接设置比较值(需要精确控制时用,比如 WS2812 的位编码)
 * @param htim    TIM 句柄
 * @param channel 通道
 * @param compare 比较值(0 ~ ARR)
 * @return true = 成功
 */
bool BSP_PWM_Set_Compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t compare);

/**
 * @brief 取当前 ARR(比较值的上限)
 * @param htim TIM 句柄
 * @return ARR 值
 */
uint32_t BSP_PWM_Get_Autoreload(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif // BSP_PWM_H
