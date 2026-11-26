//
// bsp_timestamp.cpp — TIM5 自由运行计数器做长时间戳(无中断)
// 说明 / 用法 / 坑 见同目录 bsp_timestamp.md

#include "bsp_timestamp.h"

/* TIM5 句柄(定义在 Core/Src/tim.c) */
extern "C" {
extern TIM_HandleTypeDef htim5;
}

/* ---- 状态 ---- */
static TIM_HandleTypeDef *s_htim = nullptr;
static volatile uint32_t s_wrap = 0u;        /* 回绕次数(1 小时 +1) */
static uint32_t s_last_cnt = 0u;             /* 上次读到的 CNT(判回绕用) */

/**
 * @brief 初始化并启动计数器(不开中断)
 * @param htim TIM 句柄;NULL = 用 &htim5
 * @return void
 */
void Timestamp_Init(TIM_HandleTypeDef *htim)
{
    s_htim = (htim != nullptr) ? htim : &htim5;

    /* 计数器清零后再启动,让时间从 0 开始(方便日志看"开机多少 µs") */
    __HAL_TIM_SET_COUNTER(s_htim, 0u);

    /* ★ HAL_TIM_Base_Start,不是 HAL_TIM_Base_Start_IT —— 不开中断 */
    (void)HAL_TIM_Base_Start(s_htim);

    s_last_cnt = __HAL_TIM_GET_COUNTER(s_htim);
    s_wrap = 0u;
}

/**
 * @brief 是否已初始化
 * @return true = 已启动
 */
bool Timestamp_Is_Inited(void)
{
    return (s_htim != nullptr);
}

/**
 * @brief 直接读计数器(µs)
 * @return CNT(µs)
 */
uint32_t Timestamp_Get_Us32(void)
{
    if (s_htim == nullptr)
    {
        return 0u;
    }
    return __HAL_TIM_GET_COUNTER(s_htim);        /* 1 MHz ⇒ 1 计数 = 1 µs */
}

/**
 * @brief 软件计回绕(周期任务里每拍调一次)
 * @return void
 */
void Timestamp_Update(void)
{
    uint32_t primask;
    uint32_t now;

    if (s_htim == nullptr)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();                              /* 极短临界区,可嵌套 */
    now = __HAL_TIM_GET_COUNTER(s_htim);
    if (now < s_last_cnt)
    {
        s_wrap++;                                 /* 刚跨过一次回绕 */
    }
    s_last_cnt = now;
    __set_PRIMASK(primask);
}

/**
 * @brief 64 位微秒时间戳(软件计回绕)
 * @return 时间(µs),量程约 49 万年
 */
uint64_t Timestamp_Get_Us(void)
{
    uint64_t us;
    uint32_t primask;
    uint32_t now;
    uint32_t wrap;

    if (s_htim == nullptr)
    {
        return 0ULL;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    now = __HAL_TIM_GET_COUNTER(s_htim);
    if (now < s_last_cnt)
    {
        s_wrap++;                                 /* 把"刚刚发生的回绕"补上,避免少算 1 小时 */
    }
    s_last_cnt = now;
    wrap = s_wrap;

    __set_PRIMASK(primask);

    us = (uint64_t)wrap * TIMESTAMP_WRAP_US + (uint64_t)now;
    return us;
}

/**
 * @brief 64 位毫秒时间戳
 * @return 时间(ms)
 */
uint64_t Timestamp_Get_Ms(void)
{
    return Timestamp_Get_Us() / 1000ULL;
}

/**
 * @brief 从 start 到现在过了多少 µs(自动跨一次回绕)
 * @param start 起点
 * @return 经过的微秒数
 * @note 无符号减法天然处理回绕:1 MHz 下 32 位能表示 71.6 分钟,只要间隔小于它就一定对
 */
uint32_t Timestamp_Elapsed_Us32(uint32_t start)
{
    return (uint32_t)(Timestamp_Get_Us32() - start);
}
