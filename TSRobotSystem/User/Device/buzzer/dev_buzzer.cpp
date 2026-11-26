//
// dev_buzzer.cpp — 板载蜂鸣器(PWM)
// 说明 / 用法 / 坑 见同目录 dev_buzzer.md

#include "dev_buzzer.h"

#include "bsp_timestamp.h"

/* ---- 状态 ---- */
static TIM_HandleTypeDef *buzzer_htim = nullptr;
static uint32_t buzzer_channel = 0u;
static bool sounding = false;

static bool beep_active = false;            /* 有没有在跑"响一声" */
static uint32_t beep_stop_us = 0u;          /* 该停的时刻 */
static float beep_freq = BUZZER_DEFAULT_FREQ;
static float beep_loudness = BUZZER_DEFAULT_LOUDNESS;

/**
 * @brief 初始化蜂鸣器
 * @param htim           TIM 句柄
 * @param channel        通道
 * @param timer_clock_hz 该 TIM 计数时钟(Hz)
 * @return true = 成功
 */
bool Buzzer_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timer_clock_hz)
{
    if (!BSP_PWM_Init(htim, channel, timer_clock_hz))
    {
        return false;
    }

    buzzer_htim = htim;
    buzzer_channel = channel;

    Buzzer_Stop();
    return true;
}

/**
 * @brief 立即设置声音
 * @param freq_hz  频率(Hz),0 = 静音
 * @param loudness 响度 0~1
 * @return void
 */
void Buzzer_Set_Sound(float freq_hz, float loudness)
{
    if (buzzer_htim == nullptr)
    {
        return;
    }

    if (freq_hz <= 0.0f)
    {
        Buzzer_Stop();
        return;
    }

    if (loudness < 0.0f) { loudness = 0.0f; }
    if (loudness > 1.0f) { loudness = 1.0f; }

    /* 改频率会改 ARR ⇒ 占空比要跟着重设,所以顺序是:先频率,再按新 ARR 设比较值 */
    BSP_PWM_Set_Freq(buzzer_htim, (uint32_t)freq_hz);

    {
        uint32_t arr = BSP_PWM_Get_Autoreload(buzzer_htim);
        uint32_t compare = (uint32_t)((float)arr * loudness * 0.5f);
        BSP_PWM_Set_Compare(buzzer_htim, buzzer_channel, compare);
    }

    sounding = (loudness > 0.0f);
}

/**
 * @brief 立即静音(比较值置 0)
 * @return void
 */
void Buzzer_Stop(void)
{
    if (buzzer_htim == nullptr)
    {
        return;
    }
    BSP_PWM_Set_Compare(buzzer_htim, buzzer_channel, 0u);
    sounding = false;
    beep_active = false;
}

/**
 * @brief 非阻塞响一声
 * @param freq_hz  频率
 * @param loudness 响度
 * @param ms       持续毫秒
 * @return true = 已开始
 */
bool Buzzer_Beep(float freq_hz, float loudness, uint32_t ms)
{
    if (buzzer_htim == nullptr || ms == 0u)
    {
        return false;
    }

    beep_freq = freq_hz;
    beep_loudness = loudness;
    beep_stop_us = Timestamp_Get_Us32() + (ms * 1000u);
    beep_active = true;

    Buzzer_Set_Sound(beep_freq, beep_loudness);
    return true;
}

/**
 * @brief 跑一拍:到点停
 * @return void
 */
void Buzzer_Task(void)
{
    if (!beep_active)
    {
        return;
    }

    /* Timestamp_Get_Us32() 是 32 位微秒,71.6 分钟回绕;用"差值"判断即可 */
    if ((int32_t)(Timestamp_Get_Us32() - beep_stop_us) >= 0)
    {
        Buzzer_Stop();          /* 内部会把 beep_active 清掉 */
    }
}

/**
 * @brief 当前是否在响
 * @return true = 正在发声
 */
bool Buzzer_Is_Sounding(void)
{
    return sounding;
}
