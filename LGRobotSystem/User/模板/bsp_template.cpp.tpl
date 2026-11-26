//
// bsp_xxx.cpp — 一句话说清这是哪个外设(BSP 层:只包 HAL,不含业务)
// 说明 / 用法 / 坑 见同目录 bsp_xxx.md
//

#include "bsp_xxx.h"

#include <string.h>

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static bool s_inited = false;

/* ============================================================================
 * 对外接口
 * ========================================================================== */

bool BSP_Xxx_Init(SPI_HandleTypeDef *__hspi)
{
    if (__hspi == nullptr)
    {
        return false;
    }

    s_inited = true;
    return true;
}

bool BSP_Xxx_Transfer(SPI_HandleTypeDef *__hspi, const uint8_t *__tx, uint8_t *__rx, uint16_t __len)
{
    if ((!s_inited) || (__hspi == nullptr) || (__len == 0u))
    {
        return false;
    }

    /* 例:阻塞收发(具体用 HAL 的哪个 API,按你的时序要求选)
       return (HAL_SPI_TransmitReceive(__hspi, (uint8_t *)__tx, __rx, __len, 100u) == HAL_OK); */
    (void)__tx;
    (void)__rx;
    return false;
}

bool BSP_Xxx_Is_Ready(SPI_HandleTypeDef *__hspi)
{
    return (s_inited && (__hspi != nullptr));
}
