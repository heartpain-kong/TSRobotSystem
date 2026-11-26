/* ============================================================================
 * bsp_adc.h — ADC 底层(仿 bsp_fdcan 的写法:管理对象 + Init + 取值接口)
 * 说明 / 用法 / 坑 见同目录 bsp_adc.md
 * ========================================================================== */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "adc.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 每个 ADC 的采样缓冲长度(元素个数,uint16_t) */
#define BSP_ADC_BUFFER_SIZE     128u

/* 参考电压(mV)。如果板子上 VDDA 不是 3.3V,改这里;要更准可以用 VREFINT 校准 */
#define BSP_ADC_VREF_MV         3300.0f

/**
 * @brief ADC 管理对象(每个 ADC 一个,和 CAN_Manage_Object 一个套路)
 */
struct BSP_ADC_Manage_Object
{
    ADC_HandleTypeDef *handler;                     /* 绑定的 ADC 句柄 */
    uint16_t data[BSP_ADC_BUFFER_SIZE];             /* DMA 循环写入的采样值 */
    uint16_t sample_number;                          /* 一周期采样点数 = 规则通道数 */
    bool inited;
};

/**
 * @brief 初始化 ADC:校准 + 启动 DMA 循环采样
 * @param hadc          ADC 句柄(如 &hadc1)
 * @param sample_number 一周期采样点数(= 规则通道数;ADC1 配了 2 个 rank 就传 2)
 * @return true = 成功
 * @note ★ H7 必须先 HAL_ADCEx_Calibration_Start 再启动,否则精度很差(参考工程同样做法)
 * @note 缓冲在 .bss(RAM_D1)⇒ 普通 DMA 可访问 ✓;BDMA(ADC3)不可,故 ADC3 不走这里 ✗
 */
bool BSP_ADC_Init(ADC_HandleTypeDef *hadc, uint16_t sample_number);

/**
 * @brief 是否已初始化(且正在循环采样)
 * @return true = 已启动
 */
bool BSP_ADC_Is_Inited(ADC_HandleTypeDef *hadc);

/**
 * @brief 取原始采样值(循环 DMA 一直在刷新,直接读即可)
 * @param hadc  ADC 句柄
 * @param index 第几个采样点(0 ~ sample_number-1)
 * @return 原始值(ADC1 是 16 位、ADC3 是 12 位);越界返回 0
 */
uint16_t BSP_ADC_Get_Raw(ADC_HandleTypeDef *hadc, uint16_t index);

/**
 * @brief 取某个采样点换算后的电压(mV)
 * @param hadc  ADC 句柄
 * @param index 采样点序号
 * @return 电压(mV)
 * @note 按该 ADC 的分辨率换算:16 位满量程 65535、12 位满量程 4095
 */
float BSP_ADC_Get_Mv(ADC_HandleTypeDef *hadc, uint16_t index);

/**
 * @brief 取"前 N 个采样点"的平均电压(mV)—— 我们的两个 rank 是同一路,平均就是降噪
 * @param hadc   ADC 句柄
 * @param number 平均的点数(0 或超过缓冲长度则按整缓冲)
 * @return 平均电压(mV)
 */
float BSP_ADC_Get_Mv_Average(ADC_HandleTypeDef *hadc, uint16_t number);

/**
 * @brief 读 MCU 内部温度传感器(ADC3),按 H7 出厂两点校准换算
 * @return 温度(℃);未初始化/读失败返回 -273.15f
 * @note 这是**按需阻塞单次转换**(ADC3 在 BDMA 上,不能用普通 DMA 写 RAM_D1)
 */
float BSP_ADC_Get_Temperature(void);

/**
 * @brief 绑定 ADC3(内部温度传感器用):**只记句柄,不启动 DMA**
 * @param hadc ADC3 句柄(如 &hadc3)
 * @return void
 * @note 为什么不用 DMA:ADC3 在 H7 上挂 BDMA,BDMA 只能访问 D3 SRAM(0x38000000),
 *       而本驱动的缓冲在 RAM_D1 ⇒ 硬要 DMA 必须把缓冲放进 D3 SRAM 专用段。
 *       温度不需要高速,所以 BSP_ADC_Get_Temperature() 用**按需阻塞单次转换**。
 */
void BSP_ADC_Bind_ADC3(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif // BSP_ADC_H
