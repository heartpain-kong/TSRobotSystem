//
// bsp_power.cpp — 板上电源开关(5V 输出使能,PC15)
// 说明 / 用法 / 坑 见同目录 bsp_power.md

#include "bsp_power.h"

#include "bsp_adc.h"
#include "bsp_timestamp.h"
#include "adc.h"        /* hadc1 */
/* 20:39 那次 CubeMX 导出把 PC15 的标签(POWER_5V)丢了,main.h 里暂时没有这两个宏。
   这里自带一份(与 CubeMX 里的 PC15 / POWER_5V 一致),并用 #ifndef 守卫:
   下次导出把标签带回来后,main.h 的定义先生效,这里自动让位。
   ⚠️ 这个脚无论如何都是 GPIO_Output 且初值 Low(见 tools/cubemx_post_export.ps1 第 4 步)。 */
#ifndef POWER_5V_Pin
#define POWER_5V_Pin        GPIO_PIN_15
#endif

#ifndef POWER_5V_GPIO_Port
#define POWER_5V_GPIO_Port  GPIOC
#endif

/**
 * @brief 打开/关闭板上 5V 输出
 * @param enable true = 打开
 * @return void
 */
void Power_5V_Enable(bool enable)
{
    /* 高电平有效:true → SET,false → RESET */
    BSP_GPIO_Write(POWER_5V_GPIO_Port, POWER_5V_Pin,
                   enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 当前 5V 输出是否打开
 * @return true = 已打开(引脚为高)
 */
bool Power_5V_Is_Enabled(void)
{
    return (BSP_GPIO_Read(POWER_5V_GPIO_Port, POWER_5V_Pin) == GPIO_PIN_SET);
}

/* ============================================================================
 * 电源电压检测(ADC1 / PC4,16 位)
 *   采样由 bsp_adc 的循环 DMA 负责,这里只做换算 + 低通 + 过欠压判定。
 *   换算:BSP_ADC_Get_Mv_Average() 已经做了 adc/65535*3300 ⇒ 再乘分压比即可。
 * ========================================================================== */

/* 电压一阶低通系数(每 10ms 一次 ⇒ 时间常数约 60ms) */
#define POWER_LPF_ALPHA     0.15f

static bool voltage_ready = false;
static float voltage_value = 0.0f;
static float under_voltage_threshold = POWER_UNDER_VOLTAGE_V;
static float over_voltage_threshold = POWER_OVER_VOLTAGE_V;
static bool under_voltage_flag = false;
static bool over_voltage_flag = false;
static uint32_t next_voltage_update_us = 0u;

/**
 * @brief 初始化电压检测
 */
bool Power_Voltage_Init(void)
{
    voltage_value = 0.0f;
    under_voltage_threshold = POWER_UNDER_VOLTAGE_V;
    over_voltage_threshold = POWER_OVER_VOLTAGE_V;
    under_voltage_flag = false;
    over_voltage_flag = false;

    voltage_ready = BSP_ADC_Is_Inited(&hadc1);

    if (voltage_ready)
    {
        /* 先读一次拿初值,免得低通从 0 慢慢爬 */
        voltage_value = BSP_ADC_Get_Mv_Average(&hadc1, POWER_ADC_SAMPLES) * POWER_DIVIDER / 1000.0f;
        next_voltage_update_us = Timestamp_Get_Us32();
    }

    return voltage_ready;
}

/**
 * @brief 周期跑(内部限速)
 */
void Power_Task(void)
{
    float raw;

    if (!voltage_ready)
    {
        return;
    }

    /* 没到刷新时刻就直接返回(无符号差值,跨 71.6 分钟回绕也安全) */
    if ((int32_t)(Timestamp_Get_Us32() - next_voltage_update_us) < 0)
    {
        return;
    }
    next_voltage_update_us += POWER_TASK_PERIOD_US;

    raw = BSP_ADC_Get_Mv_Average(&hadc1, POWER_ADC_SAMPLES) * POWER_DIVIDER / 1000.0f;
    voltage_value += (raw - voltage_value) * POWER_LPF_ALPHA;

    under_voltage_flag = (voltage_value < under_voltage_threshold);
    over_voltage_flag = (voltage_value > over_voltage_threshold);
}

/**
 * @brief 取电源电压(V)
 */
float Power_Get_Voltage(void)
{
    return voltage_value;
}

/**
 * @brief 电压检测是否就绪
 */
bool Power_Voltage_Is_Ready(void)
{
    return voltage_ready;
}

/**
 * @brief 改欠压阈值(V)
 */
void Power_Set_Under_Voltage(float __voltage)
{
    under_voltage_threshold = __voltage;
}

/**
 * @brief 改过压阈值(V)
 */
void Power_Set_Over_Voltage(float __voltage)
{
    over_voltage_threshold = __voltage;
}

/**
 * @brief 是否欠压
 */
bool Power_Is_Under_Voltage(void)
{
    return under_voltage_flag;
}

/**
 * @brief 是否过压
 */
bool Power_Is_Over_Voltage(void)
{
    return over_voltage_flag;
}
