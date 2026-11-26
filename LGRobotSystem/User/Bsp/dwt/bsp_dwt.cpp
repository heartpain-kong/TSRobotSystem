#include "bsp_dwt.h"

/* ---------------- 内部状态 ---------------- */

//CPU(内核)时钟 Hz —— 注意是 D1CPRE 之后的**内核时钟**,不是 HCLK
static uint32_t s_cpu_freq_hz = 0u;

//每微秒的周期数(仅在主频是 1MHz 整数倍时有效;否则为 0,走更通用的换算)
static uint32_t s_cyc_per_us  = 0u;

//CYCCNT 回绕次数(32 位计数器,640MHz 下 6.71s 回绕一次)
static uint32_t s_wrap_count  = 0u;

//上一次读到的 CYCCNT(判回绕用)
static uint32_t s_last_cnt    = 0u;

//时间轴缓存(s/ms/us)
static DWT_Time_t s_sys_time  = {0u, 0u, 0u};

static bool s_inited = false;

/* ---------------- 内部函数 ---------------- */

/**
 * @brief 取 D1CPRE 分频系数(内核时钟 = SYSCLK / 该值)
 *        寄存器编码同 HPRE:0b0xxx = 1 分频,0b1000 = 2,0b1001 = 4 ……
 */
static uint32_t DWT_D1CPRE_Div(void)
{
    uint32_t v = (RCC->D1CFGR & RCC_D1CFGR_D1CPRE) >> RCC_D1CFGR_D1CPRE_Pos;

    return (v < 8u) ? 1u : (1u << (v - 7u));
}

/**
 * @brief 读 64 位周期数(顺带维护回绕计数)
 *        临界区保护:任务和中断都会读时间戳,回绕计数必须原子更新
 *        (参考工程用的是 bit_locker 标志位,中断里打断仍可能丢一次回绕)
 */
static inline uint64_t DWT_CycCnt64(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < s_last_cnt)          //计数变小 = 回绕了一次
    {
        s_wrap_count++;
    }
    s_last_cnt = cnt_now;

    uint64_t cyc = ((uint64_t)s_wrap_count << 32) | (uint64_t)cnt_now;

    __set_PRIMASK(primask);            //恢复调用前的中断状态(可嵌套)

    return cyc;
}

/**
 * @brief 周期数 → 微秒
 *        主频是 1MHz 整数倍(640MHz 就是)时直接整除;否则分两步算,避免溢出
 */
static inline uint64_t DWT_Cyc2Us(uint64_t cyc)
{
    if (s_cyc_per_us != 0u)
    {
        return cyc / (uint64_t)s_cyc_per_us;
    }
    if (s_cpu_freq_hz == 0u)
    {
        return 0u;
    }
    return (cyc / (uint64_t)s_cpu_freq_hz) * 1000000ULL
         + ((cyc % (uint64_t)s_cpu_freq_hz) * 1000000ULL) / (uint64_t)s_cpu_freq_hz;
}

/* ---------------- 对外接口 ---------------- */

/**
 * @brief 初始化 DWT 周期计数器
 * @param CPU_Freq_MHz CPU(内核)时钟 MHz;传 0 = 自动取
 * @return void
 */
void DWT_Init(uint32_t CPU_Freq_MHz)
{
    if (CPU_Freq_MHz != 0u)
    {
        s_cpu_freq_hz = CPU_Freq_MHz * 1000000u;
    }
    else
    {
        /* 内核时钟 = SYSCLK / D1CPRE。
           ⚠️ 不是 HAL_RCC_GetHCLKFreq(),也不是 SystemCoreClock(那两个是 HCLK=160MHz) */
        s_cpu_freq_hz = HAL_RCC_GetSysClockFreq() / DWT_D1CPRE_Div();
    }

    s_cyc_per_us = ((s_cpu_freq_hz != 0u) && ((s_cpu_freq_hz % 1000000u) == 0u))
                 ? (s_cpu_freq_hz / 1000000u)
                 : 0u;

    /* 使能跟踪与周期计数器(标准三步) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;    //使能 DWT 外设
    DWT->CYCCNT = 0u;                                  //计数清零
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;              //开 CYCCNT

    s_wrap_count = 0u;
    s_last_cnt   = DWT->CYCCNT;
    s_sys_time.s = 0u;
    s_sys_time.ms = 0u;
    s_sys_time.us = 0u;
    s_inited     = true;
}

bool DWT_Is_Inited(void)
{
    return s_inited;
}

uint32_t DWT_Get_CPU_Freq_MHz(void)
{
    return s_cpu_freq_hz / 1000000u;
}

uint64_t DWT_GetTimeline_us(void)
{
    if (!s_inited)
    {
        return 0u;
    }
    return DWT_Cyc2Us(DWT_CycCnt64());
}

uint32_t DWT_Get_Us32(void)
{
    return (uint32_t)DWT_GetTimeline_us();
}

uint64_t DWT_Get_Ms(void)
{
    return DWT_GetTimeline_us() / 1000ULL;
}

float DWT_GetTimeline_s(void)
{
    return (float)((double)DWT_GetTimeline_us() * 1e-6);
}

float DWT_GetTimeline_ms(void)
{
    return (float)((double)DWT_GetTimeline_us() * 1e-3);
}

void DWT_SysTimeUpdate(void)
{
    uint64_t us = DWT_GetTimeline_us();

    s_sys_time.s  = (uint32_t)(us / 1000000ULL);
    s_sys_time.ms = (uint16_t)((us / 1000ULL) % 1000ULL);
    s_sys_time.us = (uint16_t)(us % 1000ULL);
}

void DWT_Get_SysTime(DWT_Time_t *out)
{
    if (out == nullptr)
    {
        return;
    }
    DWT_SysTimeUpdate();
    *out = s_sys_time;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    if ((cnt_last == nullptr) || (s_cpu_freq_hz == 0u))
    {
        return 0.0f;
    }

    uint32_t cnt_now = (uint32_t)DWT_CycCnt64();       //顺便维护回绕计数
    uint32_t dcyc    = (uint32_t)(cnt_now - *cnt_last); //回绕安全
    *cnt_last = cnt_now;

    return (float)dcyc / (float)s_cpu_freq_hz;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    if ((cnt_last == nullptr) || (s_cpu_freq_hz == 0u))
    {
        return 0.0;
    }

    uint32_t cnt_now = (uint32_t)DWT_CycCnt64();
    uint32_t dcyc    = (uint32_t)(cnt_now - *cnt_last);
    *cnt_last = cnt_now;

    return (double)dcyc / (double)s_cpu_freq_hz;
}

void DWT_Delay_Us(uint32_t us)
{
    if ((us == 0u) || (!s_inited))
    {
        return;
    }

    uint32_t ticks = (s_cyc_per_us != 0u)
                   ? (us * s_cyc_per_us)
                   : (uint32_t)(((uint64_t)us * (uint64_t)s_cpu_freq_hz) / 1000000ULL);

    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks)
    {
        //忙等:给 ISR / 极短硬时序用;任务里请用 osDelay/osDelayUntil
    }
}

void DWT_Delay(float Delay_s)
{
    if ((Delay_s <= 0.0f) || (!s_inited))
    {
        return;
    }

    uint32_t ticks   = (uint32_t)((double)Delay_s * (double)s_cpu_freq_hz);
    uint32_t start   = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks)
    {
        //忙等:不受中断开关影响,可以在临界区里替代 HAL_Delay
    }
}

int32_t DWT_Elapsed_Us32(uint32_t now_us, uint32_t t0_us)
{
    return (int32_t)(now_us - t0_us);
}
