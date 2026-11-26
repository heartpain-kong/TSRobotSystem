//
// bsp_pwm.cpp — PWM 通用接口
// 说明 / 用法 / 坑 见同目录 bsp_pwm.md

#include "bsp_pwm.h"

#include <stddef.h>

/**
 * @brief 让 TIM 通道以 1MHz 计数并启动 PWM
 * @param htim           TIM 句柄
 * @param channel        通道
 * @param timer_clock_hz TIM 计数时钟(Hz)
 * @return true = 成功
 */
bool BSP_PWM_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timer_clock_hz)
{
    TIM_OC_InitTypeDef oc = {0};

    if (htim == nullptr || timer_clock_hz == 0u || timer_clock_hz < BSP_PWM_COUNTER_HZ)
    {
        return false;
    }

    /* ① 预分频:让计数器 = 1MHz */
    htim->Init.Prescaler = (timer_clock_hz / BSP_PWM_COUNTER_HZ) - 1u;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = (BSP_PWM_COUNTER_HZ / 1000u) - 1u;      /* 默认 1kHz(PWM 频率) */
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
        return false;
    }

    /* ② 通道配置:占空比从 0 开始(要响再设) */
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0u;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(htim, &oc, channel) != HAL_OK)
    {
        return false;
    }

    return (HAL_TIM_PWM_Start(htim, channel) == HAL_OK);
}

/**
 * @brief 启动通道 PWM
 * @return true = 成功
 */
bool BSP_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim == nullptr)
    {
        return false;
    }
    return (HAL_TIM_PWM_Start(htim, channel) == HAL_OK);
}

/**
 * @brief 停止通道 PWM
 * @return true = 成功
 */
bool BSP_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim == nullptr)
    {
        return false;
    }
    return (HAL_TIM_PWM_Stop(htim, channel) == HAL_OK);
}

/**
 * @brief 设置 PWM 频率(只改 ARR)
 * @param htim    TIM 句柄
 * @param freq_hz 频率;0 = 停(占空比置 0)
 * @return true = 成功
 */
bool BSP_PWM_Set_Freq(TIM_HandleTypeDef *htim, uint32_t freq_hz)
{
    uint32_t arr;

    if (htim == nullptr)
    {
        return false;
    }

    if (freq_hz == 0u)
    {
        __HAL_TIM_SET_AUTORELOAD(htim, BSP_PWM_COUNTER_HZ / 1000u);
        return true;        /* 频率归零靠占空比(见器件层),ARR 给个安全值 */
    }

    /* 限幅 20Hz~20kHz:低于 20Hz 蜂鸣器听不见,高于 20kHz 也没意义 */
    if (freq_hz < 20u)     { freq_hz = 20u; }
    if (freq_hz > 20000u)  { freq_hz = 20000u; }

    arr = (BSP_PWM_COUNTER_HZ / freq_hz);
    if (arr > 0u)
    {
        arr -= 1u;
    }
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    return true;
}

/**
 * @brief 设置占空比(0~1)
 * @return true = 成功
 */
bool BSP_PWM_Set_Duty(TIM_HandleTypeDef *htim, uint32_t channel, float duty)
{
    uint32_t arr;
    uint32_t compare;

    if (htim == nullptr)
    {
        return false;
    }
    if (duty < 0.0f) { duty = 0.0f; }
    if (duty > 1.0f) { duty = 1.0f; }

    arr = __HAL_TIM_GET_AUTORELOAD(htim);
    compare = (uint32_t)((float)(arr + 1u) * duty);
    if (compare > arr)
    {
        compare = arr;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, compare);
    return true;
}

/**
 * @brief 直接设比较值
 * @return true = 成功
 */
bool BSP_PWM_Set_Compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t compare)
{
    if (htim == nullptr)
    {
        return false;
    }
    __HAL_TIM_SET_COMPARE(htim, channel, compare);
    return true;
}

/**
 * @brief 取 ARR
 * @return ARR
 */
uint32_t BSP_PWM_Get_Autoreload(TIM_HandleTypeDef *htim)
{
    if (htim == nullptr)
    {
        return 0u;
    }
    return __HAL_TIM_GET_AUTORELOAD(htim);
}
