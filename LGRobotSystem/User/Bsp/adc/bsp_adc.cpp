//
// bsp_adc.cpp — ADC 底层(仿 bsp_fdcan:管理对象 + Init + 取值)
// 说明 / 用法 / 坑 见同目录 bsp_adc.md

#include "bsp_adc.h"

#include <string.h>

/* ============================================================================
 * 管理对象(每个 ADC 一个)
 * ========================================================================== */
static BSP_ADC_Manage_Object ADC1_Manage_Object;
static BSP_ADC_Manage_Object ADC2_Manage_Object;
static BSP_ADC_Manage_Object ADC3_Manage_Object;

/**
 * @brief 按句柄找管理对象
 * @param hadc ADC 句柄
 * @return 管理对象指针(不认识返回 nullptr)
 */
static BSP_ADC_Manage_Object *BSP_ADC_Get_Object(ADC_HandleTypeDef *hadc)
{
    if (hadc == nullptr)
    {
        return nullptr;
    }

    if (hadc->Instance == ADC1)
    {
        return &ADC1_Manage_Object;
    }
    if (hadc->Instance == ADC2)
    {
        return &ADC2_Manage_Object;
    }
    if (hadc->Instance == ADC3)
    {
        return &ADC3_Manage_Object;
    }
    return nullptr;
}

/**
 * @brief 该 ADC 的满量程(按分辨率)
 * @param hadc ADC 句柄
 * @return 满量程值(16 位 = 65535,12 位 = 4095)
 */
static float BSP_ADC_Get_Full_Scale(ADC_HandleTypeDef *hadc)
{
    if (hadc == nullptr)
    {
        return 4095.0f;
    }
    return (hadc->Init.Resolution == ADC_RESOLUTION_16B) ? 65535.0f
         : (hadc->Init.Resolution == ADC_RESOLUTION_14B) ? 16383.0f
         : (hadc->Init.Resolution == ADC_RESOLUTION_12B) ? 4095.0f
         : (hadc->Init.Resolution == ADC_RESOLUTION_10B) ? 1023.0f
                                                         : 255.0f;
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化 ADC:校准 + 启动 DMA 循环
 * @param hadc ADC 句柄
 * @param sample_number 一周期采样点数(规则通道数)
 * @return true = 成功
 */
bool BSP_ADC_Init(ADC_HandleTypeDef *hadc, uint16_t sample_number)
{
    BSP_ADC_Manage_Object *object = BSP_ADC_Get_Object(hadc);
    uint16_t number = sample_number;

    if (object == nullptr || number == 0u)
    {
        return false;
    }
    if (number > BSP_ADC_BUFFER_SIZE)
    {
        number = BSP_ADC_BUFFER_SIZE;
    }

    object->handler = hadc;
    object->sample_number = number;
    memset(object->data, 0, sizeof(object->data));

    /* ★ H7 必须先做偏移校准,否则读数偏差很大(参考工程 dm02_test 同样先校准) */
    if (HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return false;
    }

    /* 循环 DMA:硬件持续把采样值写进 object->data —— 之后 CPU 只读不写 */
    if (HAL_ADC_Start_DMA(hadc, (uint32_t *)object->data, (uint32_t)number) != HAL_OK)
    {
        return false;
    }

    object->inited = true;
    return true;
}

/**
 * @brief 是否已初始化
 * @return true = 已启动
 */
bool BSP_ADC_Is_Inited(ADC_HandleTypeDef *hadc)
{
    BSP_ADC_Manage_Object *object = BSP_ADC_Get_Object(hadc);
    return (object != nullptr) && object->inited;
}

/**
 * @brief 取原始采样值
 * @return 原始值;越界/未初始化返回 0
 */
uint16_t BSP_ADC_Get_Raw(ADC_HandleTypeDef *hadc, uint16_t index)
{
    BSP_ADC_Manage_Object *object = BSP_ADC_Get_Object(hadc);

    if (object == nullptr || !object->inited || index >= object->sample_number)
    {
        return 0u;
    }
    return object->data[index];
}

/**
 * @brief 取某个采样点的电压(mV)
 * @return 电压(mV)
 */
float BSP_ADC_Get_Mv(ADC_HandleTypeDef *hadc, uint16_t index)
{
    BSP_ADC_Manage_Object *object = BSP_ADC_Get_Object(hadc);

    if (object == nullptr || !object->inited)
    {
        return 0.0f;
    }
    return (float)BSP_ADC_Get_Raw(hadc, index) * BSP_ADC_VREF_MV / BSP_ADC_Get_Full_Scale(hadc);
}

/**
 * @brief 取前 N 个采样点的平均电压(mV)
 * @return 平均电压(mV)
 * @note 我们的 ADC1 两个 rank 是同一路(CH4),平均一下正好降噪
 */
float BSP_ADC_Get_Mv_Average(ADC_HandleTypeDef *hadc, uint16_t number)
{
    BSP_ADC_Manage_Object *object = BSP_ADC_Get_Object(hadc);
    float sum = 0.0f;
    uint16_t i;
    uint16_t n;

    if (object == nullptr || !object->inited)
    {
        return 0.0f;
    }

    n = (number == 0u || number > object->sample_number) ? object->sample_number : number;
    if (n == 0u)
    {
        return 0.0f;
    }

    for (i = 0u; i < n; i++)
    {
        sum += (float)object->data[i];
    }

    return (sum / (float)n) * BSP_ADC_VREF_MV / BSP_ADC_Get_Full_Scale(hadc);
}

/**
 * @brief 读 MCU 内部温度传感器(ADC3)
 * @return 温度(℃);失败返回 -273.15f
 * @note 按 H7 出厂两点校准换算:TS_CAL1(30℃)与 TS_CAL2(110℃)存在系统区
 */
float BSP_ADC_Get_Temperature(void)
{
    /* H7 出厂校准值(见 RM0468):都是 12 位下测的 */
    static const uint16_t *const ts_cal1 = (const uint16_t *)0x1FF1E820UL;   /* 30 ℃ */
    static const uint16_t *const ts_cal2 = (const uint16_t *)0x1FF1E840UL;   /* 110 ℃ */
    uint16_t raw;
    float t1;
    float t2;

    if (ADC3_Manage_Object.handler == nullptr)
    {
        return -273.15f;
    }

    /* ADC3 不走 DMA:按需单次转换(阻塞,几个 µs) */
    if (HAL_ADC_Start(&hadc3) != HAL_OK)
    {
        return -273.15f;
    }
    if (HAL_ADC_PollForConversion(&hadc3, 5u) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc3);
        return -273.15f;
    }
    raw = (uint16_t)HAL_ADC_GetValue(&hadc3);
    (void)HAL_ADC_Stop(&hadc3);

    t1 = (float)(*ts_cal1);
    t2 = (float)(*ts_cal2);
    if (t2 <= t1)
    {
        return -273.15f;
    }

    /* 线性插值:(raw - CAL1) / (CAL2 - CAL1) * (110 - 30) + 30 */
    return ((float)raw - t1) * (110.0f - 30.0f) / (t2 - t1) + 30.0f;
}

/**
 * @brief 注册 ADC3(温度用),不启动 DMA —— 只记句柄
 * @return void
 * @note 在 Robot_Init() 里调一次即可;想用 ADC3 的 DMA 就得把缓冲挪到 D3 SRAM(见 .md)
 */
extern "C" void BSP_ADC_Bind_ADC3(ADC_HandleTypeDef *hadc)
{
    if (hadc != nullptr && hadc->Instance == ADC3)
    {
        ADC3_Manage_Object.handler = hadc;
        ADC3_Manage_Object.inited = true;      /* 只表示"已绑定",采样是按需做的 */
    }
}
