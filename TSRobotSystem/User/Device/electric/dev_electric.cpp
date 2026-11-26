//
// dev_electric.cpp — 继电器 / 电源输出设备接口
// 说明 / 用法 / 坑 见同目录 dev_electric.md

#include "dev_electric.h"

#include "bsp_power.h"

/* ============================================================================
 * 通道表
 *   GPIOx == nullptr ⇒ 该通道未配置(操作一律返回 false)
 *   active_high: true = 高电平合上;false = 低电平合上(继电器模块常见)
 * ========================================================================== */

static const Electric_Channel_t s_channels[ELECTRIC_CH_COUNT] = {
    /* 0: 板载 5V —— 引脚由 bsp_power 管(POWER_5V / PC15),这里只占位 */
    {nullptr, 0u, true, "5V"},

    /* 1: RELAY_1(要在 CubeMX 里把某脚设成 GPIO_Output、标签写 RELAY_1) */
#ifdef RELAY_1_Pin
    {RELAY_1_GPIO_Port, RELAY_1_Pin, false, "RL1"},
#else
    {nullptr, 0u, false, "RL1"},
#endif

    /* 2: RELAY_2 */
#ifdef RELAY_2_Pin
    {RELAY_2_GPIO_Port, RELAY_2_Pin, false, "RL2"},
#else
    {nullptr, 0u, false, "RL2"},
#endif

    /* 3: RELAY_3 */
#ifdef RELAY_3_Pin
    {RELAY_3_GPIO_Port, RELAY_3_Pin, false, "RL3"},
#else
    {nullptr, 0u, false, "RL3"},
#endif
};

/* 输入通道:PE2/PE3 光电找零 */
static const Electric_Input_Channel_t s_inputs[ELECTRIC_IN_COUNT] = {
    {GPIOE, GPIO_PIN_2, true, "PHOTO1"},
    {GPIOE, GPIO_PIN_3, true, "PHOTO2"},
};

/* 是否已初始化(初始化前不允许操作,免得在 GPIO 还没配好时乱写) */
static bool s_inited = false;

/**
 * @brief 该通道是否可用
 */
static bool Channel_Is_Configured(Enum_Electric_Channel __channel)
{
    if (__channel >= ELECTRIC_CH_COUNT)
    {
        return false;
    }

    /* 5V 通道永远可用:bsp_power 里有 PC15 兜底 */
    if (__channel == ELECTRIC_CH_5V)
    {
        return true;
    }

    return (s_channels[__channel].GPIOx != nullptr);
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化:所有已配置通道置"断开"
 * @return true = 至少有一路可用
 */
void Electric_Input_Init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0u; i < (uint8_t)ELECTRIC_IN_COUNT; i++)
    {
        gpio.Pin = s_inputs[i].GPIO_Pin;
        HAL_GPIO_Init(s_inputs[i].GPIOx, &gpio);
    }
}

bool Electric_Input_Get(Enum_Electric_Input __channel)
{
    if (__channel >= ELECTRIC_IN_COUNT)
    {
        return false;
    }

    const Electric_Input_Channel_t *ch = &s_inputs[__channel];
    const GPIO_PinState level = HAL_GPIO_ReadPin(ch->GPIOx, ch->GPIO_Pin);
    return ch->active_low ? (level == GPIO_PIN_RESET) : (level == GPIO_PIN_SET);
}

uint8_t Electric_Input_Get_Raw(Enum_Electric_Input __channel)
{
    if (__channel >= ELECTRIC_IN_COUNT)
    {
        return 0u;
    }

    const Electric_Input_Channel_t *ch = &s_inputs[__channel];
    return (HAL_GPIO_ReadPin(ch->GPIOx, ch->GPIO_Pin) == GPIO_PIN_SET) ? 1u : 0u;
}

bool Electric_Init(void)
{
    s_inited = true;

    /* 上电先全部断开(和 POWER_5V 出厂关闭一个思路) */
    for (uint8_t i = 0u; i < (uint8_t)ELECTRIC_CH_COUNT; i++)
    {
        (void)Electric_Set((Enum_Electric_Channel)i, false);
    }

    return (Electric_Get_Channel_Count() > 0u);
}

/**
 * @brief 合上/断开某一路
 */
bool Electric_Set(Enum_Electric_Channel __channel, bool __on)
{
    if ((!s_inited) || (__channel >= ELECTRIC_CH_COUNT))
    {
        return false;
    }

    /* 5V 转发给 bsp_power(PC15 只有一个负责人) */
    if (__channel == ELECTRIC_CH_5V)
    {
        Power_5V_Enable(__on);
        return true;
    }

    if (s_channels[__channel].GPIOx == nullptr)
    {
        return false;
    }

    /* on == active_high ⇒ 输出高;否则输出低 */
    HAL_GPIO_WritePin(s_channels[__channel].GPIOx,
                      s_channels[__channel].GPIO_Pin,
                      (__on == s_channels[__channel].active_high) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

/**
 * @brief 读某一路当前状态
 */
bool Electric_Get(Enum_Electric_Channel __channel)
{
    if (__channel >= ELECTRIC_CH_COUNT)
    {
        return false;
    }

    if (__channel == ELECTRIC_CH_5V)
    {
        return Power_5V_Is_Enabled();
    }

    if (s_channels[__channel].GPIOx == nullptr)
    {
        return false;
    }

    return ((HAL_GPIO_ReadPin(s_channels[__channel].GPIOx, s_channels[__channel].GPIO_Pin) == GPIO_PIN_SET)
            == s_channels[__channel].active_high);
}

/**
 * @brief 翻转某一路
 */
bool Electric_Toggle(Enum_Electric_Channel __channel)
{
    if ((!s_inited) || (__channel >= ELECTRIC_CH_COUNT))
    {
        return false;
    }

    return Electric_Set(__channel, !Electric_Get(__channel));
}

/**
 * @brief 全部合上/断开
 */
void Electric_Set_All(bool __on)
{
    if (!s_inited)
    {
        return;
    }

    for (uint8_t i = 0u; i < (uint8_t)ELECTRIC_CH_COUNT; i++)
    {
        (void)Electric_Set((Enum_Electric_Channel)i, __on);
    }
}

/**
 * @brief 该通道是否已配置
 */
bool Electric_Is_Configured(Enum_Electric_Channel __channel)
{
    return Channel_Is_Configured(__channel);
}

/**
 * @brief 已配置的通道数
 */
uint8_t Electric_Get_Channel_Count(void)
{
    uint8_t count = 0u;

    for (uint8_t i = 0u; i < (uint8_t)ELECTRIC_CH_COUNT; i++)
    {
        if (Channel_Is_Configured((Enum_Electric_Channel)i))
        {
            count++;
        }
    }

    return count;
}

/**
 * @brief 取通道名
 */
const char *Electric_Get_Name(Enum_Electric_Channel __channel)
{
    if (__channel >= ELECTRIC_CH_COUNT)
    {
        return "N/A";
    }

    return s_channels[__channel].Name;
}

/**
 * @brief 取第 index 个已配置通道的编号(遍历用)
 */
Enum_Electric_Channel Electric_Get_Channel(uint8_t __index)
{
    uint8_t count = 0u;

    for (uint8_t i = 0u; i < (uint8_t)ELECTRIC_CH_COUNT; i++)
    {
        if (Channel_Is_Configured((Enum_Electric_Channel)i))
        {
            if (count == __index)
            {
                return (Enum_Electric_Channel)i;
            }
            count++;
        }
    }

    return ELECTRIC_CH_COUNT;
}
