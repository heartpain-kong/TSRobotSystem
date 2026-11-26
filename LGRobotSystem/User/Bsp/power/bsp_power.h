/* ============================================================================
 * bsp_power.h — 板上电源开关(5V 输出使能,PC15)
 * 说明 / 用法 / 坑 见同目录 bsp_power.md
 * ========================================================================== */

#ifndef BSP_POWER_H
#define BSP_POWER_H

#include "bsp_gpio.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 打开/关闭板上 5V 输出
 * @param enable true = 打开,false = 关闭
 * @return void
 * @note 5V 使能是高电平有效(POWER_5V 为高 = 输出 5V)
 */
void Power_5V_Enable(bool enable);

/**
 * @brief 当前 5V 输出是否打开
 * @return true = 已打开
 * @note 读的是引脚当前电平
 */
bool Power_5V_Is_Enabled(void);

#ifdef __cplusplus
}
#endif

/* ============================================================================
 * 电源电压检测(ADC1 / PC4,16 位)
 *   · 板上电源经分压后接到 ADC1_INP4 ⇒ 采样由 User/Bsp/adc 的循环 DMA 负责;
 *   · 换算与同板工程 dm02_test 的 bsp_power 一致:V = adc/2^16 * 3.3 * 分压比;
 *   · 分压比按硬件改 POWER_DIVIDER(同板工程用 11)。
 * ========================================================================== */
#define POWER_DIVIDER           11.0f
#define POWER_ADC_SAMPLES       2u          /* ADC1 每周期采几个点(它是 2 个 rank) */
#define POWER_TASK_PERIOD_US    10000u      /* 刷新周期(µs):10ms = 100Hz */
#define POWER_UNDER_VOLTAGE_V   20.0f       /* 欠压阈值(V) */
#define POWER_OVER_VOLTAGE_V    30.0f       /* 过压阈值(V) */

/**
 * @brief 初始化电压检测(绑定 ADC1)
 * @return true = ADC1 已在采样
 * @note 前置:BSP_ADC_Init(&hadc1, 2) 已经调过
 */
bool Power_Voltage_Init(void);

/**
 * @brief 周期跑:刷新电压 + 过欠压判定(内部按 POWER_TASK_PERIOD_US 限速)
 * @return void
 */
void Power_Task(void);

/**
 * @brief 取电源电压(V)
 * @return 电压;未初始化返回 0
 */
float Power_Get_Voltage(void);

/**
 * @brief 电压检测是否就绪
 * @return true = 就绪
 */
bool Power_Voltage_Is_Ready(void);

/**
 * @brief 改过/欠压阈值(V)
 * @return void
 */
void Power_Set_Under_Voltage(float __voltage);
void Power_Set_Over_Voltage(float __voltage);

/**
 * @brief 是否欠压 / 过压
 * @return true = 触发
 */
bool Power_Is_Under_Voltage(void);
bool Power_Is_Over_Voltage(void);

#endif // BSP_POWER_H
