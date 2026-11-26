#include "bsp_uart.h"
#include <cstring>

/* 485 发送诊断(给调试时看) */
volatile uint32_t g_uart_tx_ok = 0u;
volatile uint32_t g_uart_tx_fail = 0u;

namespace {

/* 每个 UART 的端口描述:双缓冲对 + 帧长 + 回调槽 */
struct UartPort {
    UART_HandleTypeDef *huart;
    uint8_t            *buf[2];      /* M0 / M1 双缓冲                    */
    uint16_t            frame_len;   /* 单缓冲容量(= 协议最大帧)          */
    uint16_t            min_len;     /* 有效帧下限(定长口 = frame_len)    */
    uint16_t            cap;         /* 单缓冲实际容量(字节); _Ex 用它校验 */
    USART_Callback      cb;
};

/* 接收双缓冲:必须落在 AXI_SRAM 段(链接脚本映射至 0x24000000,NOLOAD)
   485 两口按"最大协议帧"开(= RS485_recv_Data_N_MAX),实际帧长由 frame_len 决定 */
__attribute__((section(".AXI_SRAM"))) static uint8_t rx1 [2][USART1_recv_Data_N];
__attribute__((section(".AXI_SRAM"))) static uint8_t rx2 [2][RS485_recv_Data_N_MAX];
__attribute__((section(".AXI_SRAM"))) static uint8_t rx3 [2][RS485_recv_Data_N_MAX];
__attribute__((section(".AXI_SRAM"))) static uint8_t rx5 [2][SBUS_RX_BUF_NUM];
__attribute__((section(".AXI_SRAM"))) static uint8_t rx7 [2][USART7_recv_Data_N];

static UartPort ports[] = {
    /* huart    双缓冲对        帧长                 最短有效帧          容量                      回调  */
    {&huart1,  {rx1[0],  rx1[1]},  USART1_recv_Data_N,  10,               USART1_recv_Data_N,   nullptr},
    {&huart2,  {rx2[0],  rx2[1]},  RS485_recv_Data_N,   RS485_recv_Data_N, RS485_recv_Data_N_MAX, nullptr},
    {&huart3,  {rx3[0],  rx3[1]},  RS485_recv_Data_N,   RS485_recv_Data_N, RS485_recv_Data_N_MAX, nullptr},
    {&huart5,  {rx5[0],  rx5[1]},  SBUS_RX_BUF_NUM,     SBUS_RX_BUF_NUM,   SBUS_RX_BUF_NUM,      nullptr},
    {&huart7,  {rx7[0],  rx7[1]},  USART7_recv_Data_N,  1,                 USART7_recv_Data_N,   nullptr},
};

inline UartPort *find_port(UART_HandleTypeDef *h){
    for (auto &p : ports) if (p.huart == h) return &p;
    return nullptr;
}

/* ===== 引擎核心:手写 DMA 双缓冲(DBM)+ IDLE 帧结束 =====
   前提:RX DMA 流在 CubeMX 中配置为 Circular、字节对齐、外设→内存 */
void rx_dma_multibuffer_start(UartPort *p){
    UART_HandleTypeDef *huart  = p->huart;
    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;   /* 借 HAL 的 RxEvent 分发器        */
    huart->RxXferSize    = p->frame_len * 2;            /* 使 Size = RxXferSize-NDTR = 实收 */

    do { __HAL_DMA_DISABLE(huart->hdmarx); }
    while (stream->CR & DMA_SxCR_EN);                   /* 等 EN 真正清零(RM0468 要求)    */

    stream->PAR  = (uint32_t)&huart->Instance->RDR;
    stream->M0AR = (uint32_t)p->buf[0];
    stream->M1AR = (uint32_t)p->buf[1];
    stream->NDTR = p->frame_len;                        /* 首帧收满时硬件自动乒乓一次       */
    SET_BIT(stream->CR, DMA_SxCR_DBM);                  /* 双缓冲模式                       */
    __HAL_DMA_ENABLE(huart->hdmarx);

    /* 先清掉上电/重新初始化时残留的 IDLE/错误标志,再开中断。
       否则接收机已经开着时,H7 上电会从半帧开始,立刻产生 ORE/IDLE,
       HAL 没有 ErrorCallback 时接收会永久停住。 */
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);      /* UART→DMA 请求连通               */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);          /* 帧结束 = 线路空闲 1 字符时间     */
    __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);           /* ORE/FE/NE 等错误中断            */
    __HAL_UART_ENABLE_IT(huart, UART_IT_PE);            /* 校验错误中断                     */
}

} // namespace

/**
 * @brief 注册回调并启动 DBM+IDLE 接收引擎
 */
void USART_init(UART_HandleTypeDef *huart, USART_Callback back){
    UartPort *p = find_port(huart);
    if (!p) return;
    p->cb = back;
    rx_dma_multibuffer_start(p);
}

/**
 * @brief 兼容旧名:RS485 口统一走 DBM 引擎(默认帧长 = 宇树 8010 的 16 B)
 */
void USART_RX485_init(UART_HandleTypeDef *huart, USART_Callback back){
    USART_RX485_init_Ex(huart, RS485_recv_Data_N, back);
}

/**
 * @brief 指定接收帧长的 485 注册(定长协议)
 *        不同协议的帧长不一样:宇树 GO-M8010-6 = 16 B、宇树 IM6014 = 26 B。
 *        引擎的 NDTR/乒乓都按 frame_len 算,所以必须与真实帧长**完全相等**
 *        (回调里 Size 落在 [min_len, frame_len] 才交上层)。
 * @param huart    BSP 端口表里的 485 口
 * @param recv_len 单帧字节数
 * @param back     帧回调
 * @return void
 */
void USART_RX485_init_Ex(UART_HandleTypeDef *huart, uint16_t recv_len, USART_Callback back){
    UartPort *p = find_port(huart);
    if (!p) return;
    if (recv_len == 0u || recv_len > p->cap) return;   /* 超容量就直接不启动,避免 DMA 越界 */

    p->frame_len = recv_len;    /* 定长口:下限 = 上限 = 帧长 */
    p->min_len   = recv_len;
    USART_init(huart, back);
}

/**
 * @brief DMA 发送(发送缓冲在 AXI_SRAM,规避 D-Cache 一致性问题)
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, const uint8_t *Data, uint8_t len){
    __attribute__((section(".AXI_SRAM"))) static uint8_t tx[64];
    __attribute__((section(".AXI_SRAM"))) static uint8_t tx_485[64];
    if (len > sizeof(tx)) return HAL_ERROR;
    uint8_t *tx_data = tx;
    if (huart->Instance == USART2)
    {
        /* Do not overwrite a DMA transfer still in flight. */
        if (huart->gState != HAL_UART_STATE_READY) { g_uart_tx_fail++; return HAL_BUSY; }
        tx_data = tx_485;
    }
    memcpy(tx_data, Data, len);

    /* 确保 RS485 的 DE 模式真的打开(CubeMX/HAL 初始化异常时 CR3.DEM 可能被清) */
    if ((huart->Instance == USART2) || (huart->Instance == USART3))
    {
        SET_BIT(huart->Instance->CR3, USART_CR3_DEM);
    }

    const uint8_t ret = HAL_UART_Transmit_DMA(huart, tx_data, len);
    if (ret == HAL_OK) { g_uart_tx_ok++; }
    else               { g_uart_tx_fail++; }
    return ret;
}

/**
 * @brief IDLE 事件回调(帧结束)
 *
 * 机制要点(与 COD 原版一致):
 *  - ISR 入口读 DMA 的 CT 位 = 本帧实际写入的缓冲(不变量:稳态下无硬件乒乓);
 *  - 解析后手动翻转 CT、重装 NDTR = 2×frame_len,使硬件乒乓此后永不触发;
 *  - RxXferSize 初始化为 2×L,故 HAL 传入的 Size 恰为本帧实收字节数。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    UartPort *p = find_port(huart);
    if (p){
        DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;
        const bool in_m1  = (stream->CR & DMA_SxCR_CT) != 0;
        uint8_t    *frame = p->buf[in_m1 ? 1 : 0];

        if (Size >= p->min_len && Size <= p->frame_len && p->cb != nullptr){
            p->cb(huart, frame, Size);                      /* 帧长界内才交上层                */
        }

        /* 手动乒乓 + 重装计数(先等流停稳再改 CT/NDTR —— 补上 COD 原版缺失的规范等待) */
        __HAL_DMA_DISABLE(huart->hdmarx);
        while (stream->CR & DMA_SxCR_EN) { }
        if (in_m1) stream->CR &= ~DMA_SxCR_CT;
        else       stream->CR |=  DMA_SxCR_CT;
        stream->NDTR = p->frame_len * 2;
        __HAL_DMA_ENABLE(huart->hdmarx);
    }

    /* re-arm:重新武装 IDLE 中断与 DMA 请求(循环模式下 HAL 不拆除,此处为保险) */
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
 * @brief UART 错误恢复(尤其是接收机先开、H7 后上电时的 ORE/FE)
 *
 * 现象:先开遥控器再给 H7 上电,接收机已经在连续发帧,H7 的 UART5
 * 从半帧开始接收,产生 Overrun/Framing Error;HAL 默认把接收停掉,
 * 之后遥控器一直 rc_rx=0。这里清错误并重启 DMA+IDLE。
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UartPort *p = find_port(huart);
    if (p == nullptr)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    rx_dma_multibuffer_start(p);
}
