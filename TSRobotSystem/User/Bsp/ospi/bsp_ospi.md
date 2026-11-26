# bsp_ospi — OCTOSPI 总线驱动

**定位**：只管总线，不管芯片。文件：`bsp_ospi.h` / `bsp_ospi.cpp`。
参照：`E:\rm\dm02_test\User_File\1_Middleware\Driver\OSPI\drv_ospi.cpp`（同板可跑工程，
板级配置一致：**OCTOSPI2 + quad + MDMA**），但按本工程的 RTOS 形态做了调整（见下）。

## 它提供什么

| 接口 | 作用 |
| --- | --- |
| `BSP_OSPI_Init(&hospi2, 轮询回调, 收回调, 发回调)` | 绑定句柄 + 注册三个回调 |
| `BSP_OSPI_Get_Tx_Buffer / Get_Rx_Buffer` | 取 512B 收发缓冲（器件驱动往里填/往外读）|
| `BSP_OSPI_Command(cmd)` | 只发命令（阻塞到发出）：写使能、擦除、读状态 |
| `BSP_OSPI_Command_Transmit_DMA(cmd)` | 命令 + DMA 发 `tx_buffer` |
| `BSP_OSPI_Command_Receive_DMA(cmd)` | 命令 + DMA 收进 `rx_buffer` |
| `BSP_OSPI_AutoPolling_IT(cfg)` | 中断方式的自动轮询（等 Flash 状态位）|
| `BSP_OSPI_Is_Ready()` | OSPI 是否空闲 |

`HAL_OSPI_StatusMatchCallback / RxCpltCallback / TxCpltCallback / ErrorCallback`
在本文件里被重定义（覆盖 HAL 的 `__weak`），转给注册进来的回调。
> 已用 `nm` 验证：四个符号在 ELF 里是 **`T`（强符号，来自 `bsp_ospi.cpp.obj`）**，
> 说明覆盖成功 —— 否则中断永远进不了驱动。

## ★ 与参考工程的**有意差异**：中断里不做逻辑

参考工程是裸机，把"轮询超时"放在 1ms 定时器中断里；本工程是 **RTOS**：

* **中断/DMA 回调里只置标志、记时间戳** —— 不阻塞、不 `osDelay`、不做长逻辑；
* **超时与状态推进**放到器件的"跑一拍"函数（如 `W25Q64_Task()`），由 RTOS 任务按周期调用。

⇒ 中断和任务之间不需要加锁，也不会出现"ISR 里调用了非 FromISR 版 RTOS API"这类问题。

## 缓冲区与 DMA/Cache（重要）

收发缓冲在 `.bss` 里，链接到 **RAM_D1(0x24000000)**；`main.c::MPU_Config()` 把这一整块
配成了 **Normal Non-cacheable** ⇒ **DMA 直接可用，不需要任何 cache 维护** ✓
（注意这条前提：**别把缓冲放到 DTCM 或其它可缓存区**，否则要手工 `SCB_CleanDCache/InvalidateDCache`）。

## 依赖与前置条件

1. `Core/Src/octospi.c`：`MX_OCTOSPI2_Init()`（已由 CubeMX 生成，`main()` 里调用）
   —— 其中两项**必须**是对齐过的值（见下）；
2. `Core/Src/stm32h7xx_it.c`：`OCTOSPI2_IRQHandler` 里要有 `HAL_OSPI_IRQHandler(&hospi2);`
   （否则自动轮询/DMA 完成中断收不到）；
3. `Core/Src/main.c`：`MX_MDMA_Init()` + `MX_OCTOSPI2_Init()`（MDMA 已 `__HAL_LINKDMA` 给 `hospi2`）。

### 两个必须对齐的参数（CubeMX 默认值是不对的）

| 参数 | CubeMX 默认 | 应该 | 原因 |
| --- | --- | --- | --- |
| `SampleShifting` | `NONE` | **`HALFCYCLE`** | 采样点不对 ⇒ 读回数据错位 |
| `ClockPrescaler` | `1`（SCK=80MHz）| **`3`（SCK=40MHz，与厂家例程一致）** | 先低速打通；W25Q64 的 0xEB 上限 133MHz，确认无误后可改回 1 |

这两项已写进 `TSRobotSystem.ioc` 和 `tools/cubemx_post_export.ps1`（每次导出后自动补回）。

## 相关

* `User/Device/w25q64/dev_w25q64.md` —— 用它的芯片驱动（W25Q64 Flash）
* `User/Bsp/spi/bsp_spi.md` —— SPI 总线驱动（挂 BMI088）
