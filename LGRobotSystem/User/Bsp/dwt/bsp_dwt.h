#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "struct_typedef.h"
#include "main.h"
#include <stdbool.h>   //本头文件被 main.c(C)包含,bool 需要它

/* 本头文件会被 C(main.c)与 C++(各电机类)同时包含 ——
   实现是 .cpp,所以必须 extern "C",否则 C 侧链接时找不到被修饰过的符号名 */
#ifdef __cplusplus
extern "C" {
#endif

/** 时间轴(s / ms / us 分解形式,与参考工程同名) */
typedef struct
{
    uint32_t s;      //秒
    uint16_t ms;     //毫秒(0~999)
    uint16_t us;     //微秒(0~999)
} DWT_Time_t;

/**
 * @brief 取一段代码的执行时间(秒),结果写进 dt
 *        用法: float dt; DWT_TIME_ELAPSE(dt, { 被测代码 });
 *        与参考工程的区别:这里不打印日志(本工程没有 RTT/日志模块)
 */
#define DWT_TIME_ELAPSE(dt, code)          \
    do                                     \
    {                                      \
        float _dwt_t0 = DWT_GetTimeline_s(); \
        code;                              \
        (dt) = DWT_GetTimeline_s() - _dwt_t0; \
    } while (0)

/**
 * @brief 初始化 DWT 周期计数器
 * @param CPU_Freq_MHz CPU(内核)时钟频率,单位 MHz;**传 0 = 自动取**
 *                     (HAL_RCC_GetSysClockFreq() / D1CPRE,本工程 = 640MHz)
 * @note 必须在调度器启动前调用
 */
void DWT_Init(uint32_t CPU_Freq_MHz);

/** DWT 是否已经初始化 */
bool DWT_Is_Inited(void);

/** 实际使用的 CPU 主频(MHz)—— 诊断用:如果时间明显差整数倍,先看这个值 */
uint32_t DWT_Get_CPU_Freq_MHz(void);

/**
 * @brief 绝对时间戳(微秒,64 位)—— 精确到 1 个周期(640MHz 下 1.56ns),截断到 us
 */
uint64_t DWT_GetTimeline_us(void);

/**
 * @brief 时间戳的低 32 位(= 1us 步进的 32 位计数器,71.6 分钟回绕)
 *        给 Control_Freq 调度这类只需要相对时间的场合用,省一次 64 位运算;
 *        差值比较仍然环绕安全
 */
uint32_t DWT_Get_Us32(void);

/** 绝对时间戳(毫秒,64 位) */
uint64_t DWT_Get_Ms(void);

/** 绝对时间戳(秒,浮点)—— ⚠️ float 尾数 24 位:开机 1 小时分辨率约 0.24ms,
 *  24 小时约 5ms。要精确请用 DWT_GetTimeline_us()。 */
float DWT_GetTimeline_s(void);

/** 绝对时间戳(毫秒,浮点) */
float DWT_GetTimeline_ms(void);

/**
 * @brief 刷新内部时间轴缓存(与参考工程同名)
 *        参考工程要求"长时间不读时间轴就手动调一次",本工程每次读都会更新,
 *        这个函数只是给"想先把缓存刷好再取"的场合留的接口。
 */
void DWT_SysTimeUpdate(void);

/** 取 s/ms/us 分解形式的时间(内部先刷新缓存) */
void DWT_Get_SysTime(DWT_Time_t *out);

/**
 * @brief 取两次调用之间的间隔(秒)
 * @param cnt_last 传入上一次的 CYCCNT 快照(调用方保存),函数内部会更新它
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/** 同上,双精度版 */
double DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief 微秒级忙等(ISR 或极短硬时序用)
 *        ⚠️ 任务里不要用它 —— 不让出 CPU,会饿死低优先级任务;任务请用 osDelay/osDelayUntil
 *        ⚠️ 单次不要超过一个回绕周期(640MHz 下约 6.7s)
 */
void DWT_Delay_Us(uint32_t us);

/**
 * @brief 秒级忙等(与参考工程同名,参数单位是秒)
 * @note 不受中断开关影响,可以在临界区里用(临界区里不能用 HAL_Delay)
 * @warning 内部 ticks 也是 32 位:主频 640MHz 时 Delay_s **不要超过约 6.7 秒**
 *          (超过一个 CYCCNT 回绕周期,比较值会失效)。要等更久请用 osDelay。
 */
void DWT_Delay(float Delay_s);

/**
 * @brief 求两个 32 位时间戳的间隔(微秒),自动处理回绕
 * @return now - t0,负值表示 now 早于 t0(差值在 ±35 分钟内始终正确)
 */
int32_t DWT_Elapsed_Us32(uint32_t now_us, uint32_t t0_us);

#ifdef __cplusplus
}
#endif

#endif //BSP_DWT_H
