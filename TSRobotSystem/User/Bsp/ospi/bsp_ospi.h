/* ============================================================================
 * bsp_ospi.h — OCTOSPI 总线驱动(仿 E:\rm\dm02_test 的 1_Middleware/Driver/OSPI)
 * 说明 / 用法 / 坑 见同目录 bsp_ospi.md
 * ========================================================================== */

#ifndef BSP_OSPI_H
#define BSP_OSPI_H

#include "octospi.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 收发缓冲长度(够放一整页 256B + 命令开销) */
#define BSP_OSPI_BUFFER_SIZE    512u

/* ---- 三个回调(都在中断里跑,只置标志/记时间)---- */
typedef void (*BSP_OSPI_Polling_Callback)(void);    /* 自动轮询结束(状态匹配) */
typedef void (*BSP_OSPI_Rx_Callback)(void);         /* DMA 接收完成 */
typedef void (*BSP_OSPI_Tx_Callback)(void);         /* DMA 发送完成 */

/**
 * @brief OCTOSPI 管理对象(每个 OSPI 实例一个)
 */
typedef struct
{
    OSPI_HandleTypeDef *handler;                    /* 绑定的 OSPI 句柄 */
    BSP_OSPI_Polling_Callback polling_callback;     /* 自动轮询完成回调 */
    BSP_OSPI_Rx_Callback rx_callback;               /* 接收完成回调 */
    BSP_OSPI_Tx_Callback tx_callback;               /* 发送完成回调 */

    uint8_t tx_buffer[BSP_OSPI_BUFFER_SIZE];        /* 发送缓冲(在 RAM_D1,MPU 已设非缓存 ⇒ DMA 安全) */
    uint8_t rx_buffer[BSP_OSPI_BUFFER_SIZE];        /* 接收缓冲 */
} BSP_OSPI_Object;

/**
 * @brief 初始化 OSPI 驱动:绑定句柄并注册三个回调
 * @param hospi             OSPI 句柄(如 &hospi2)
 * @param polling_callback  自动轮询完成回调(可为 NULL)
 * @param rx_callback       接收完成回调(可为 NULL)
 * @param tx_callback       发送完成回调(可为 NULL)
 * @return void
 * @note 前置条件:MX_OCTOSPIx_Init() 已经跑过(HAL_OSPI_Init + HAL_OSPIM_Config)
 */
void BSP_OSPI_Init(OSPI_HandleTypeDef *hospi,
                   BSP_OSPI_Polling_Callback polling_callback,
                   BSP_OSPI_Rx_Callback rx_callback,
                   BSP_OSPI_Tx_Callback tx_callback);

/**
 * @brief 取该 OSPI 的发送/接收缓冲(器件驱动往里填数据、从里读数据)
 * @param hospi OSPI 句柄
 * @return 缓冲首地址(未注册的句柄返回 NULL)
 */
uint8_t *BSP_OSPI_Get_Tx_Buffer(OSPI_HandleTypeDef *hospi);
uint8_t *BSP_OSPI_Get_Rx_Buffer(OSPI_HandleTypeDef *hospi);

/**
 * @brief 只发命令(不传数据),阻塞到命令发完。用于 WRITE_ENABLE / 擦除 / 读状态
 * @param hospi OSPI 句柄
 * @param cmd   命令配置(OSPI_RegularCmdTypeDef)
 * @return HAL 状态
 */
HAL_StatusTypeDef BSP_OSPI_Command(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *cmd);

/**
 * @brief 命令 + DMA 发送 tx_buffer 里的 NbData 字节
 * @note 调用后立即返回;发完会进 tx_callback(在那里发起"等 WIP"自动轮询)
 */
HAL_StatusTypeDef BSP_OSPI_Command_Transmit_DMA(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *cmd);

/**
 * @brief 命令 + DMA 接收 NbData 字节到 rx_buffer
 * @note 调用后立即返回;收完会进 rx_callback
 */
HAL_StatusTypeDef BSP_OSPI_Command_Receive_DMA(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *cmd);

/**
 * @brief 启动中断方式的自动轮询(等 Flash 的 WIP 位清零等)
 * @param hospi OSPI 句柄
 * @param cfg   轮询配置(OSPI_AutoPollingTypeDef)
 * @return HAL 状态
 * @note 调用前必须先用 BSP_OSPI_Command() 把"读状态寄存器"的命令发好(HAL 的轮询用的是
 *       OSPI 里当前配置的命令);轮询结束会进 polling_callback。
 *       ⚠️ 依赖 Core/Src/stm32h7xx_it.c 里的 HAL_OSPI_IRQHandler(&hospi2)
 */
HAL_StatusTypeDef BSP_OSPI_AutoPolling_IT(OSPI_HandleTypeDef *hospi, OSPI_AutoPollingTypeDef *cfg);

/**
 * @brief OSPI 是否空闲(可以接受新命令)
 * @return true = HAL 状态机处于 READY
 */
bool BSP_OSPI_Is_Ready(OSPI_HandleTypeDef *hospi);

#ifdef __cplusplus
}
#endif

#endif // BSP_OSPI_H
