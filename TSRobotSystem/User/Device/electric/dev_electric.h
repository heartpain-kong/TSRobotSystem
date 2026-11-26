/* ============================================================================
 * dev_electric.h — 继电器 / 电源输出设备接口
 * 说明 / 用法 / 坑 见同目录 dev_electric.md
 * ========================================================================== */

#ifndef DEV_ELECTRIC_H
#define DEV_ELECTRIC_H

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 继电器/输出通道编号(表是定长的,没配的通道运行期返回 false)
 */
typedef enum
{
    ELECTRIC_CH_5V = 0,     /* 板载 5V 输出(转发 bsp_power,PC15) */
    ELECTRIC_CH_RELAY_1,    /* 标签 RELAY_1(需在 CubeMX 里配) */
    ELECTRIC_CH_RELAY_2,    /* 标签 RELAY_2 */
    ELECTRIC_CH_RELAY_3,    /* 标签 RELAY_3 */
    ELECTRIC_CH_COUNT
} Enum_Electric_Channel;

/* 输入通道:光电找零(PE2/PE3,CHASSIS_PHOTO_ENABLE=1 时启用) */
typedef enum
{
    ELECTRIC_IN_PHOTO_1 = 0,
    ELECTRIC_IN_PHOTO_2,
    ELECTRIC_IN_COUNT
} Enum_Electric_Input;

/**
 * @brief 一路输出的硬件描述
 */
typedef struct
{
    GPIO_TypeDef *GPIOx;    /* 端口;nullptr = 该通道未配置 */
    uint16_t GPIO_Pin;      /* 引脚 */
    bool active_high;       /* true = 高电平合上;false = 低电平合上(继电器模块常见) */
    const char *Name;       /* 通道名(调试/日志用) */
} Electric_Channel_t;

typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    bool active_low;        /* true = 低电平表示遮挡/触发 */
    const char *Name;
} Electric_Input_Channel_t;

/**
 * @brief 初始化:把**所有已配置**的通道置为"断开"
 * @return true = 至少有一路已配置
 * @note 5V 通道转调 Power_5V_Enable(false)
 */
bool Electric_Init(void);

/** @brief 初始化 PE2/PE3 为 GPIO 输入(上拉) */
void Electric_Input_Init(void);

/** @brief 读输入是否有效(低有效):true = 光电遮挡 */
bool Electric_Input_Get(Enum_Electric_Input __channel);

/** @brief 读输入原始电平:1 = 高,0 = 低 */
uint8_t Electric_Input_Get_Raw(Enum_Electric_Input __channel);

/**
 * @brief 合上/断开某一路
 * @param __channel 通道(见 Enum_Electric_Channel)
 * @param __on      true = 合上(输出有效电平),false = 断开
 * @return true = 成功;false = 通道无效 / 未配置 / 未初始化
 */
bool Electric_Set(Enum_Electric_Channel __channel, bool __on);

/**
 * @brief 读某一路当前状态(读的是引脚电平,不是"命令值")
 * @param __channel 通道
 * @return true = 当前处于"合上"状态
 */
bool Electric_Get(Enum_Electric_Channel __channel);

/**
 * @brief 翻转某一路
 * @param __channel 通道
 * @return true = 成功
 */
bool Electric_Toggle(Enum_Electric_Channel __channel);

/**
 * @brief 全部合上/断开(只作用于已配置的通道)
 * @param __on true = 全部合上
 * @return void
 */
void Electric_Set_All(bool __on);

/**
 * @brief 该通道是否已在 CubeMX 里配置
 * @param __channel 通道
 * @return true = 已配置
 */
bool Electric_Is_Configured(Enum_Electric_Channel __channel);

/**
 * @brief 已配置的通道数
 * @return 数量
 */
uint8_t Electric_Get_Channel_Count(void);

/**
 * @brief 取通道名(调试/日志用)
 * @param __channel 通道
 * @return 名字;无效通道返回 "N/A"
 */
const char *Electric_Get_Name(Enum_Electric_Channel __channel);

/**
 * @brief 取第 index 个**已配置**通道的编号(遍历用)
 * @param __index 0 ~ Electric_Get_Channel_Count()-1
 * @return 通道编号;越界返回 ELECTRIC_CH_COUNT
 */
Enum_Electric_Channel Electric_Get_Channel(uint8_t __index);

#ifdef __cplusplus
}
#endif

#endif // DEV_ELECTRIC_H
