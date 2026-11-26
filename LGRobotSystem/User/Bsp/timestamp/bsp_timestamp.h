/* ============================================================================
 * bsp_timestamp.h — 长时间戳(TIM5 自由运行计数器,1 MHz ⇒ 1 计数 = 1 µs)
 * 说明 / 用法 / 坑 见同目录 bsp_timestamp.md
 * ========================================================================== */

#ifndef BSP_TIMESTAMP_H
#define BSP_TIMESTAMP_H

#include "tim.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 计数器频率:1 MHz ⇒ CNT 的值就是微秒数 */
#define TIMESTAMP_CNT_HZ        1000000u

/* 一次回绕的时间(µs):ARR = 3600000000-1 ⇒ 3600 s */
#define TIMESTAMP_WRAP_US       3600000000ULL

/**
 * @brief 初始化并启动时间戳计数器(只启动计数,**不开中断**)
 * @param htim TIM 句柄;传 NULL 表示用本工程默认的 &htim5
 * @return void
 * @note 内部就是 HAL_TIM_Base_Start(不是 _IT),并把计数器清零
 */
void Timestamp_Init(TIM_HandleTypeDef *htim);

/**
 * @brief 是否已初始化
 * @return true = 已经启动
 */
bool Timestamp_Is_Inited(void);

/**
 * @brief ★ 主力接口:直接读计数器(µs)
 * @return 当前计数值(µs);每 71.6 分钟数值回绕一次
 * @note **判断超时请用差值**(见 Timestamp_Elapsed_Us32),不要直接比大小
 */
uint32_t Timestamp_Get_Us32(void);

/**
 * @brief 64 位微秒时间戳(软件计回绕 ⇒ 量程约 49 万年)
 * @return 当前时间(µs)
 * @note 内部有极短临界区(可嵌套),读 CNT 后立刻判断回绕,避免"跨回绕读"造成整 1 小时的误差
 */
uint64_t Timestamp_Get_Us(void);

/**
 * @brief 64 位毫秒时间戳
 * @return 当前时间(ms)
 */
uint64_t Timestamp_Get_Ms(void);

/**
 * @brief 软件计回绕:在周期任务里每拍调一次(1 kHz 下最多 1 小时后才会用到)
 * @return void
 * @note 不调也不会立刻出错(Get_Us 内部也会维护),但周期调用可让 64 位时间戳完全无死角
 */
void Timestamp_Update(void);

/**
 * @brief 从某个起点到现在过了多少微秒(自动处理一次回绕)
 * @param start 起点(Timestamp_Get_Us32() 的返回值)
 * @return 经过的微秒数(只要间隔 < 71.6 分钟就绝对正确)
 */
uint32_t Timestamp_Elapsed_Us32(uint32_t start);

#ifdef __cplusplus
}
#endif

#endif // BSP_TIMESTAMP_H
