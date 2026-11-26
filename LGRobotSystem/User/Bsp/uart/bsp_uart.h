#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"                         // 先经 main.h -> stm32h7xx_hal.h 完成 HAL 标准初始化顺序,并提供 huart1~huart10 extern 声明
#include "stm32h7xx_ll_usart.h"            // LL 寄存器级接口(须在 HAL 初始化顺序之后)

/* 各口单缓冲大小 = 协议单帧长度
   (原 1 字节占位 = 每字节一次中断,已废弃;DBM 引擎下每帧只进一次中断) */
#define USART1_recv_Data_N   136u   /* 裁判系统最大帧 5+2+127+2            */
#define SBUS_RX_BUF_NUM      18u   /* DBUS 遥控帧                          */
#define USART7_recv_Data_N    64u   /* 预留(VOFA/调试)                    */
#define USART10_recv_Data_N   64u   /* 预留                                 */
#define RS485_Send_Data_N     17u   /* 宇树 GO-M8010-6 发送帧长               */
#define RS485_recv_Data_N     16u   /* 宇树 GO-M8010-6 接收帧长               */
/* 485 口单缓冲容量上限:不同协议帧长不同(8010 = 16,IM6014 = 26),
   引擎按 frame_len 收帧,缓冲按容量开 —— USART_RX485_init_Ex() 可指定帧长 */
#define RS485_recv_Data_N_MAX 64u

/* 接收回调:size = 本帧实收字节数(IDLE 事件判定),协议完整性由上层校验
   注意:此 typedef 传入的 buf 仅在 ISR 上下文有效,重负载解析请拷贝后处理 */
typedef void (*USART_Callback)(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size);

/* 统一入口:注册回调并启动 DBM+IDLE 接收引擎(要求 RX DMA 流为 Circular 模式) */
void USART_init(UART_HandleTypeDef *huart, USART_Callback back);

/* 兼容旧名:RS485 口同样走 DBM 引擎,DE 方向由 HAL_RS485Ex_Init 硬件自动管理 */
void USART_RX485_init(UART_HandleTypeDef *huart, USART_Callback back);

/* ★ 指定接收帧长的 485 注册(定长协议用)
   huart    : 必须是已在本 BSP 端口表里的口(huart1/2/3/5/7/10)
   recv_len : 该协议的单帧长度,必须 <= 本口缓冲容量(485 口 = RS485_recv_Data_N_MAX)
   例:宇树 GO-M8010-6 → 16;宇树 IM6014 → 26
   注意:同一个口只能注册一种帧长,换协议请改 CubeMX/工程里对应的 recv_len */
void USART_RX485_init_Ex(UART_HandleTypeDef *huart, uint16_t recv_len, USART_Callback back);

/* DMA 发送(TX 缓冲在 AXI_SRAM,D-Cache 安全) */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, const uint8_t *Data, uint8_t len);

#endif // BSP_UART_H
