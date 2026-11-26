/* ============================================================================
 * dev_buzzer.h — 板载蜂鸣器(PWM 驱动,挂在某个 TIM 通道上)
 * 说明 / 用法 / 坑 见同目录 dev_buzzer.md
 * ========================================================================== */

#ifndef DEV_BUZZER_H
#define DEV_BUZZER_H

#include "bsp_pwm.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 常用音名(Hz)—— 仿 basic_framework 的写法,够用即可 */
#define BUZZER_FREQ_C4      262.0f
#define BUZZER_FREQ_D4      294.0f
#define BUZZER_FREQ_E4      330.0f
#define BUZZER_FREQ_F4      349.0f
#define BUZZER_FREQ_G4      392.0f
#define BUZZER_FREQ_A4      440.0f
#define BUZZER_FREQ_B4      494.0f
#define BUZZER_FREQ_C5      523.0f
#define BUZZER_FREQ_D5      587.0f
#define BUZZER_FREQ_E5      659.0f
#define BUZZER_FREQ_G5      784.0f
#define BUZZER_FREQ_A5      880.0f
#define BUZZER_FREQ_C6     1047.0f

/* 默认参数 */
#define BUZZER_DEFAULT_FREQ     4000.0f     /* 参考工程默认 4kHz */
#define BUZZER_DEFAULT_LOUDNESS 0.5f

/**
 * @brief 初始化蜂鸣器(把某个 TIM 通道按 1MHz 计数器起来)
 * @param htim           TIM 句柄(蜂鸣器所在定时器)
 * @param channel        通道(如 TIM_CHANNEL_2)
 * @param timer_clock_hz 该 TIM 的计数时钟(本工程 APB1/APB2 定时器都是 160MHz)
 * @return true = 成功
 */
bool Buzzer_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timer_clock_hz);

/**
 * @brief 立即设置声音(频率 + 响度)
 * @param freq_hz  频率(Hz),0 = 静音
 * @param loudness 响度 0~1(占空比的一半,和参考工程一致)
 * @return void
 */
void Buzzer_Set_Sound(float freq_hz, float loudness);

/**
 * @brief 立即静音
 * @return void
 */
void Buzzer_Stop(void);

/**
 * @brief **非阻塞**响一声:ms 之后由 Buzzer_Task() 自动停
 * @param freq_hz  频率(Hz)
 * @param loudness 响度 0~1
 * @param ms       持续毫秒
 * @return true = 已开始
 */
bool Buzzer_Beep(float freq_hz, float loudness, uint32_t ms);

/**
 * @brief ★ 跑一拍:处理 Buzzer_Beep() 的自动停止
 * @note 挂在周期任务里(1kHz/500Hz 都行);不在中断里调
 */
void Buzzer_Task(void);

/**
 * @brief 当前是否在响
 * @return true = 正在发声
 */
bool Buzzer_Is_Sounding(void);

#ifdef __cplusplus
}
#endif

#endif // DEV_BUZZER_H
