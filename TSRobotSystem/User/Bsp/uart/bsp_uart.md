# bsp_uart — 串口 BSP：手写 DMA 双缓冲（DBM）+ IDLE 帧结束 引擎

每个串口**注册一个回调函数指针**（查表分发），引擎用 **DMA 双缓冲（DBM）+ IDLE 中断**
判定帧结束：收满一帧只进一次中断，回调里拿到「本帧实收字节数」。

引擎移植自 `COD_H7_Template`（`BSP/Src/bsp_uart.c`），适配 TSRobotSystem 架构。

| 文件 | 内容 |
|---|---|
| `bsp_uart.h` | 帧长宏、回调类型与对外接口声明 |
| `bsp_uart.cpp` | 端口表、DBM 启动、DMA 发送、IDLE 回调（帧分发与手动乒乓）|

## 移植时的四处适配

1. 保留「每口注册回调函数指针」架构，**查表分发**；
2. 回调签名**携带 `size`**（IDLE 事件实收字节数）；
3. `HAL_UARTEx_RxEventCallback` 以 **`extern "C"`** 覆盖弱符号（C++ 修饰名问题）；
4. 全部 DMA 缓冲放 **`.AXI_SRAM`** 段（DTCM 对 DMA1/2 不可达；MPU 已将 `0x24000000`
   配为非缓存）。

## 缓冲区与帧长宏

各口单缓冲大小 = 协议单帧长度。（原 1 字节占位 = 每字节一次中断，**已废弃**；
DBM 引擎下每帧只进一次中断。）

| 宏 | 值 | 说明 |
|---|---|---|
| `USART1_recv_Data_N` | `136u` | 裁判系统最大帧 `5+2+127+2` |
| `SBUS_RX_BUF_NUM` | `18u` | DBUS 遥控帧 |
| `USART7_recv_Data_N` | `64u` | 预留（VOFA / 调试）|
| `USART10_recv_Data_N` | `64u` | 预留 |
| `RS485_Send_Data_N` | `17u` | 宇树 GO-M8010-6 发送帧长 |
| `RS485_recv_Data_N` | `16u` | 宇树 GO-M8010-6 接收帧长 |
| `RS485_recv_Data_N_MAX` | `32u` | 485 口**单缓冲容量上限**：不同协议帧长不同（8010 = 16，IM6014 = 26），引擎按 `frame_len` 收帧、缓冲按容量开 —— `USART_RX485_init_Ex()` 可指定帧长 |

## 端口表（BSP 认识哪些口）

只有表里的口能被 `USART_init()` / `USART_RX485_init_Ex()` 接上（查不到就静默返回）：

| 口 | 帧长 `frame_len` | 最短有效帧 `min_len` | 单缓冲容量 `cap` |
|---|---|---|---|
| `huart1` | `USART1_recv_Data_N`（136）| 10 | 136 |
| `huart2` | `RS485_recv_Data_N`（16）| 16 | `RS485_recv_Data_N_MAX`（32）|
| `huart3` | `RS485_recv_Data_N`（16）| 16 | `RS485_recv_Data_N_MAX`（32）|
| `huart5` | `SBUS_RX_BUF_NUM`（18）| 18 | 18 |
| `huart7` | `USART7_recv_Data_N`（64）| 1 | 64 |
| `huart10` | `USART10_recv_Data_N`（64）| 1 | 64 |

接收双缓冲数组（`rx1`/`rx2`/`rx3`/`rx5`/`rx7`/`rx10`，各 `[2][cap]`）都在 `.AXI_SRAM`
段；485 两个口按“最大协议帧”开（= `RS485_recv_Data_N_MAX`），实际帧长由 `frame_len`
决定。

## 接口一览

| 接口 | 作用 | 返回值 |
|---|---|---|
| `USART_Callback` | 接收回调类型：`void (*)(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size)`；`size` = 本帧实收字节数（IDLE 事件判定），协议完整性由上层校验 | — |
| `void USART_init(UART_HandleTypeDef *huart, USART_Callback back)` | 统一入口：注册回调并启动 DBM+IDLE 接收引擎（要求 RX DMA 流为 **Circular** 模式）| void（口不在端口表里则不启动）|
| `void USART_RX485_init(UART_HandleTypeDef *huart, USART_Callback back)` | 兼容旧名：RS485 口同样走 DBM 引擎，DE 方向由 `HAL_RS485Ex_Init` 硬件自动管理。默认帧长 = 宇树 8010 的 16 B | void |
| `void USART_RX485_init_Ex(UART_HandleTypeDef *huart, uint16_t recv_len, USART_Callback back)` | 指定接收帧长的 485 注册（定长协议用）| void（`recv_len == 0` 或超过本口 `cap` 时**直接不启动**，避免 DMA 越界）|
| `uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, const uint8_t *Data, uint8_t len)` | DMA 发送（TX 缓冲在 AXI_SRAM，D-Cache 安全）| `HAL_UART_Transmit_DMA()` 的返回码；`len > 64` 时直接返回 `HAL_ERROR` |

> `USART_RX485_init_Ex()` 的参数注意：`huart` 必须是已在本 BSP 端口表里的口
> （`huart1/2/3/5/7/10`）；`recv_len` 是该协议的单帧长度，必须 `<=` 本口缓冲容量
> （485 口 = `RS485_recv_Data_N_MAX`）。例：宇树 GO-M8010-6 → 16；宇树 IM6014 → 26。
> **同一个口只能注册一种帧长**，换协议请改 CubeMX / 工程里对应的 `recv_len`。

## 引擎机制

### 启动（`rx_dma_multibuffer_start()`）

前提：**RX DMA 流在 CubeMX 中配置为 Circular、字节对齐、外设→内存**。

1. `huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;` —— 借 HAL 的 RxEvent 分发器；
2. `huart->RxXferSize = p->frame_len * 2;` —— 使 `Size = RxXferSize - NDTR` = 实收字节数；
3. `SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);` —— UART→DMA 请求连通；
4. `__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);` —— 帧结束 = 线路空闲 1 字符时间；
5. 先 `__HAL_DMA_DISABLE()` 并等 `stream->CR` 的 EN 位真正清零（RM0468 要求）；
6. 写 `stream->PAR = &huart->Instance->RDR`、`M0AR = buf[0]`、`M1AR = buf[1]`、
   `NDTR = frame_len`（**首帧收满时硬件自动乒乓一次**）；
7. `SET_BIT(stream->CR, DMA_SxCR_DBM);` 置双缓冲模式，然后 `__HAL_DMA_ENABLE()`。

### IDLE 事件回调（`HAL_UARTEx_RxEventCallback`，帧结束）—— 与 COD 原版一致

* ISR 入口读 DMA 的 **CT 位** = 本帧实际写入的缓冲（不变量：**稳态下无硬件乒乓**）；
* 解析后**手动翻转 CT、重装 `NDTR = 2 × frame_len`**，使硬件乒乓此后永不触发；
* `RxXferSize` 初始化为 `2 × L`，故 HAL 传入的 `Size` **恰为本帧实收字节数**；
* `Size` 落在 `[min_len, frame_len]` 且回调非空，才把帧交给上层
  （`p->cb(huart, frame, Size)`，帧长界内才交）；
* 手动乒乓 + 重装计数是**先等流停稳（等 EN 清零）再改 CT / NDTR** —— 补上了 COD 原版
  缺失的规范等待；
* 最后 **re-arm**：重新武装 IDLE 中断、`USART_CR3_DMAR` 与 DMA
  （循环模式下 HAL 不拆除，此处为保险）。

## 注意

* 回调在**中断上下文**里跑（`buf` 仅在 ISR 上下文有效），**不能阻塞**：重负载解析请
  **拷贝后处理**；要等发送 / 会阻塞的操作放到任务里。
* 帧长必须与真实协议**完全相等**：引擎的 `NDTR` / 乒乓都按 `frame_len` 算（见
  `USART_RX485_init_Ex()`），注册错帧长会收不到或收半帧。
* 换 485 口的协议时，除了 `recv_len`，`huart` 本身也必须是端口表里的口。
* `UART_Transmit_Data()` 用的是文件内 `static uint8_t tx[64]` 缓冲：**所有调用共用这一个**，
  而且 `HAL_UART_Transmit_DMA()` 是异步的，所以 `len` 不得超过 64。

## 最小用法

```c
/* 1. 写一个回调:huart 是哪个口、buf/size 是本帧数据 */
void USART2_Callback(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size)
{
    /* 在此解析本帧(必要时先拷走),不要在 ISR 里阻塞 */
}

/* 2. 注册(上电一次):帧长按型号 —— 8010 = 16 B,IM6014 = 26 B */
USART_RX485_init_Ex(&huart2, ys.Get_Recv_Len(), USART2_Callback);
```

## 相关

* `bsp_uart.h` / `bsp_uart.cpp` —— 本模块。
* `#include "usart.h"` 要**先于** `#include "stm32h7xx_ll_usart.h"`：前者经 `main.h` →
  `stm32h7xx_hal.h` 完成 HAL 标准初始化顺序，并提供 `huart1~huart10` 的 extern 声明；
  后者是 LL 寄存器级接口，须在 HAL 初始化顺序之后。
* 上层使用：`User\Device\motor\motor_YS`（宇树 485 电机）、`motor_DM`（UART 版达妙）。
* 引擎来源：`COD_H7_Template` 的 `BSP/Src/bsp_uart.c`。
