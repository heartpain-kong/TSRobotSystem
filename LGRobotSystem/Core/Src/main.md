# main.c — LGRobotSystem 的程序入口

来源：`Core/Src/main.c` 中 `/* USER CODE BEGIN Header */ … /* USER CODE END Header */` 之间那段
"本工程的补充说明"（写在 USER CODE 段内，CubeMX 重新生成不会丢）。

---

## 1. 这是什么

LGRobotSystem 的入口：CubeMX 生成的外设初始化 + FreeRTOS 启动。

## 2. 启动顺序

```
HAL_Init
  → SystemClock_Config / PeriphCommonClock_Config
  → MX_xxx_Init（各外设）
  → DWT_Init(0)           ← 微秒级时间戳，电机频率调度靠它（必须在调度器之前）
  → Robot_Init()          ← 机器人层：各 BSP（CAN 过滤器）+ 各子系统 Init（电机/接收回调）
  → osKernelInitialize    ← CMSIS-RTOS2 必须先初始化内核
  → MX_FREERTOS_Init      ← 内部（freertos.c 的 RTOS_THREADS 段）调 OSTaskInit() 建任务
  → osKernelStart         ← 调度器接管，此后 main 里的代码不再执行
```

> ⚠️ **为什么建任务在 `freertos.c` 而不在 `main.c`？**
> CMSIS-RTOS2 要求先 `osKernelInitialize()` 才能建线程,而 `USER CODE 2` 在它之前;
> `MX_FREERTOS_Init()` 正好在它之后、`osKernelStart()` 之前 ⇒ `OSTaskInit()` 只能放那儿。
> 硬件/子系统初始化没有这个限制,留在 `main.c` 更直观。
> 详见 `User/App/robot/robot.md`。
>
> ⚠️ `main.c` 是 **C 文件**:只能包 C 兼容头(`robot.h`/`bsp_dwt.h`),
> **绝不能** include `motor_mit.h`/`motor_DM.h` 等 C++ 类头。

| 步骤 | 调用 | 说明 |
|---|---|---|
| 1 | `HAL_Init()` | 复位所有外设、初始化 Flash 接口与 SysTick |
| 2 | `SystemClock_Config()` | 配置系统时钟（其后还有 `PeriphCommonClock_Config()` 配置外设公共时钟） |
| 3 | `MX_GPIO_Init()` / `MX_DMA_Init()` / `MX_BDMA_Init()` / `MX_FDCANx_Init()` / `MX_TIMx_Init()` / `MX_USARTx_UART_Init()` / `MX_UARTx_Init()` / `MX_SPI2_Init()` / `MX_ADCx_Init()` … | 各外设初始化 |
| 4 | `DWT_Init(0)` | 微秒级时间戳（DWT->CYCCNT，内核时钟）。**必须在调度器启动前调用**（时间戳的基准）。传 `0` = 自动取主频 |
| 5 | `osKernelInitialize()` → `MX_FREERTOS_Init()` → `osKernelStart()` | 初始化内核 → 建任务/队列/信号量（在 `freertos.c`）→ 启动调度器 |

## 3. 自己的代码放哪

| 要加的东西 | 放哪 |
|---|---|
| 外设初始化的补充 | 各外设 `.c` 的 USER CODE 段 |
| 任务 / 队列 / 信号量的创建 | `Core/Src/freertos.c` 的 USER CODE 段 |
| 机器人对象与电机 | `User/` 下（见 `User/Device/motor/电机使用说明.md`） |
| 算法与驱动 | `User/algorithm`、`User/Bsp` |

## 4. 注意

* `osKernelStart()` 之后的代码**永远不会执行**，别在那儿写逻辑。
* `DWT_Init()` 必须在调度器启动前调用（时间戳的基准）。
* 本文件由 CubeMX 管理：改动只写在 `USER CODE BEGIN/END` 标记之间，否则下次生成会被覆盖。

## 5. 相关

* `Core/Src/freertos.c` —— FreeRTOS 初始化与任务创建
* `User/Device/motor/电机使用说明.md` —— 电机 / 机器人对象的使用说明
