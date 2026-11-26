/* ============================================================================
 * bsp_xxx.h — 一句话说清这是哪个外设(BSP 层:只包 HAL,不含业务)
 * 说明 / 用法 / 坑 见同目录 bsp_xxx.md
 * ❌ 本层(Bsp)禁止:出现'底盘/云台/舵轮'这类业务词、在这里解析协议
 * ★ 用法:复制到 User/Bsp/<模块>/ → 去掉 .tpl → 全文替换 xxx/Xxx
 * ========================================================================== */

#ifndef BSP_XXX_H
#define BSP_XXX_H

#include "spi.h"                /* 只 include 你要用的 CubeMX 句柄头 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化(注册回调/启动 DMA 等)
 * @note 外设本身的参数由 CubeMX 决定,这里不要偷偷改 ✗
 */
bool BSP_Xxx_Init(SPI_HandleTypeDef *__hspi);

/**
 * @brief 收发(单位/语义写清)
 */
bool BSP_Xxx_Transfer(SPI_HandleTypeDef *__hspi, const uint8_t *__tx, uint8_t *__rx, uint16_t __len);

/**
 * @brief 是否就绪
 */
bool BSP_Xxx_Is_Ready(SPI_HandleTypeDef *__hspi);

#ifdef __cplusplus
}
#endif

#endif // BSP_XXX_H
