/* ============================================================================
 * bsp_spi.h — SPI 总线驱动(仿 E:\rm\dm02_test 的 1_Middleware/Driver/SPI)
 * 说明 / 用法 / 坑 见同目录 bsp_spi.md
 * ========================================================================== */

#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "spi.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 收发缓冲长度 */
#define BSP_SPI_BUFFER_SIZE     512u

/* 中断/完成回调(中断上下文:只置标志) */
typedef void (*BSP_SPI_Callback)(void);

/**
 * @brief SPI 管理对象(每个 SPI 实例一个)
 */
typedef struct
{
    SPI_HandleTypeDef *handler;
    BSP_SPI_Callback callback;              /* 中断收发完成回调(可为 NULL) */

    GPIO_TypeDef *cs_gpiox;                 /* 最近一次用到的片选(IT 版收尾时复位用) */
    uint16_t cs_pin;
    GPIO_PinState cs_active_level;

    uint8_t tx_buffer[BSP_SPI_BUFFER_SIZE];
    uint8_t rx_buffer[BSP_SPI_BUFFER_SIZE];
    uint16_t tx_length;
    uint16_t rx_length;
} BSP_SPI_Object;

/**
 * @brief 初始化 SPI 驱动:绑定句柄 + 注册回调
 * @param hspi     SPI 句柄(如 &hspi2)
 * @param callback 中断收发完成回调(可空)
 * @return void
 * @note 前置条件:MX_SPIx_Init() 已经跑过
 */
void BSP_SPI_Init(SPI_HandleTypeDef *hspi, BSP_SPI_Callback callback);

/**
 * @brief 取发送/接收缓冲
 * @param hspi SPI 句柄
 * @return 缓冲首地址(未注册返回 NULL)
 */
uint8_t *BSP_SPI_Get_Tx_Buffer(SPI_HandleTypeDef *hspi);
uint8_t *BSP_SPI_Get_Rx_Buffer(SPI_HandleTypeDef *hspi);

/**
 * @brief 全双工收发(阻塞):CS 有效 → 收发 → CS 无效
 * @param hspi       SPI 句柄
 * @param cs_gpiox   片选 GPIO(如 GPIOC)
 * @param cs_pin     片选引脚(如 GPIO_PIN_0)
 * @param cs_level   片选**有效**电平(BMI088 是低电平有效 ⇒ GPIO_PIN_RESET)
 * @param tx_length  发送字节数
 * @param rx_length  接收字节数(一般 == tx_length)
 * @param timeout_ms 超时(ms)
 * @return true = 成功
 * @note tx/rx 数据分别放在 BSP_SPI_Get_Tx_Buffer()/Get_Rx_Buffer() 里
 */
bool BSP_SPI_Transmit_Receive(SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, GPIO_PinState cs_level,
                              uint16_t tx_length, uint16_t rx_length, uint32_t timeout_ms);

/**
 * @brief 只发不收(阻塞,CS 同样由本函数包住)
 * @return true = 成功
 */
bool BSP_SPI_Transmit(SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, GPIO_PinState cs_level,
                      uint16_t tx_length, uint32_t timeout_ms);

/**
 * @brief 全双工收发(中断):立即返回,收完进回调
 * @return true = 已发起
 * @note 回调里请自行处理数据;片选在"收完回调"里由本驱动拉回(见 .cpp 注释)
 */
bool BSP_SPI_Transmit_Receive_IT(SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, GPIO_PinState cs_level,
                                 uint16_t tx_length, uint16_t rx_length);

/**
 * @brief 全双工收发(DMA):立即返回,搬运交给 DMA,收完进回调
 * @return true = 已发起
 * @note 与 _IT 版**共用同一套回调**(HAL_SPI_TxRxCpltCallback → 拉回片选 → 转给注册的回调),
 *       所以调用方行为完全一样,只是 CPU 不再逐字节搬运。
 * @note 依赖 CubeMX 已给该 SPI 配好 DMA 并 __HAL_LINKDMA(本工程 SPI2 = hdma_spi2_tx/rx 已配,
 *       SPI6 = hdma_spi6_tx 已配)。缓冲在 RAM_D1(MPU 非缓存)不需要 cache 维护。
 */
bool BSP_SPI_Transmit_Receive_DMA(SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, GPIO_PinState cs_level,
                                  uint16_t tx_length, uint16_t rx_length);

/**
 * @brief 只发不收(DMA):立即返回,发完进回调(WS2812 这类"只往外推"的场景用它)
 * @return true = 已发起
 */
bool BSP_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, GPIO_PinState cs_level,
                          uint16_t tx_length);
#ifdef __cplusplus
}
#endif

#endif // BSP_SPI_H
