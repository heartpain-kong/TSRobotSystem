# freertos.c — FreeRTOS 的初始化与任务创建

来源：`Core/Src/freertos.c` 中 `/* USER CODE BEGIN Header */ … /* USER CODE END Header */` 之间那段
"本工程的补充说明"（写在 USER CODE 段内，CubeMX 重新生成不会丢）。

---

## 1. 这是什么

FreeRTOS 的初始化与任务创建：`osKernelInitialize()` 之后由 `main.c` 调用 `MX_FREERTOS_Init()`，
本文件负责建任务 / 队列 / 信号量。

## 2. 任务往哪加

在 `RTOS_THREADS` 标记之间追加 `osThreadNew(...)`，任务函数写在 `Application` 段或单独放
`User/App/` 下。

`MX_FREERTOS_Init()` 里可用的 USER CODE 段（按出现顺序）：

| 标记 | 用途 |
|---|---|
| `USER CODE BEGIN/END Init` | 函数入口处的补充初始化 |
| `USER CODE BEGIN/END RTOS_MUTEX` | 加互斥量 |
| `USER CODE BEGIN/END RTOS_SEMAPHORES` | 加信号量 |
| `USER CODE BEGIN/END RTOS_TIMERS` | 启动 / 新增定时器 |
| `USER CODE BEGIN/END RTOS_QUEUES` | 加队列 |
| `USER CODE BEGIN/END RTOS_THREADS` | **加线程**（`osThreadNew`） |
| `USER CODE BEGIN/END RTOS_EVENTS` | 加事件 |
| `USER CODE BEGIN/END Application` | 私有应用代码 |

CubeMX 自带创建的 `defaultTask`（`osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes)`）
在 `RTOS_THREADS` 段**之前**；其线程体 `StartDefaultTask()` 里先调 `MX_USB_DEVICE_Init()`，
然后进 `for(;;) osDelay(1);`。

## 3. 本项目打算怎么用

* **电机控制任务 1 kHz**（`osDelayUntil` 定时）：大疆整组一帧、MIT 类各自按频率发 ——
  具体写法见 `User/Device/motor/电机使用说明.md` 第 3 节。
* **反馈接收不建任务**：走 BSP 的中断回调 → `motor.recv()`（见说明文档第 2 节）。
* 电机 / 机器人对象建议放全局（`Class_Motor_DJ_Group` ≈ 1.5 KB，别放任务栈上）。

## 4. 注意

* 本文件由 CubeMX 管理：改动只写 `USER CODE BEGIN/END` 之间。
* 默认任务的栈大小在 CubeMX 里配（osThreadNew 的 .stack_size）。本工程当前是 **1024 * 4 字节 = 4 KB**，跑电机协议解析够用；加任务时按需要调。

## 5. FreeRTOS 移植层与"导出后要做的三件事"

**移植层 = `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1`**（M7 专用），
内核 = **FreeRTOS V10.6.2**。两个文件（`port.c` / `portmacro.h`）来自上游 V10.6.2，
已按 git blob SHA 校验字节一致；正本另存一份在 `tools/port/ARM_CM7/r0p1/`（那份不会被 CubeMX 动）。

> ⚠️ **CubeMX 重新导出会把 `portable/` 整个换成 `ARM_CM4F` 并删掉 CM7 目录。**
> 更阴的是：**头文件被删不会触发重编**（GCC `-MP` 会给缺失头文件生成伪目标），
> 于是靠旧的 `port.c.obj` 工程照样能编过 —— 直到某天你清理重建，
> 或者**旧移植层对象和新内核一起被链进固件**（本工程真踩过：排查"运行跳错误"很久）。
> 所以导出后**必须**跑一次：

```powershell
powershell -ExecutionPolicy Bypass -File tools\cubemx_post_export.ps1        # 修复 + 自检 + 编译
powershell -ExecutionPolicy Bypass -File tools\cubemx_post_export.ps1 -NoBuild
```

脚本会做 6 件事：
1. `Core/Inc/FreeRTOSConfig.h`：堆 **32768** + `configUSE_MALLOC_FAILED_HOOK 1` + `configCHECK_FOR_STACK_OVERFLOW 2`（导出会冲掉）；
2. 移植层：从 `tools/port/` **重装 CM7**、与正本比对字节，并把 CM4F 等其它移植层**移出编译树**
   （`file(GLOB_RECURSE "Middlewares/*.*")` 会把它们都收进去 ⇒ 两份 `port.c` 必然符号重复）；
3. 清掉构建目录里**陈旧的 `ARM_CM*` 对象**（源码已不存在的那种）；
4. 确认关键 USER CODE 段还在（`Robot_Init` / `OSTaskInit` / 两个错误钩子 / HardFault 现场记录）；
5. 确认 PC15（POWER_5V，板上 5V 输出使能）初始化成 RESET；
6. 确认 User 目录与 include 列表，然后编译验证。

**要在 CubeMX GUI 里设、否则每次导出都会丢**（`.ioc` 手改会被 GUI 覆盖）：
`TOTAL_HEAP_SIZE = 32768`、`USE_MALLOC_FAILED_HOOK = Enabled`、`CHECK_FOR_STACK_OVERFLOW = Option 2`、
`defaultTask` 栈 **256 Words**。

## 6. 相关

* `Core/Src/main.c` —— `osKernelInitialize()` 与 `osKernelStart()` 的调用方
* `User/Device/motor/电机使用说明.md` —— 电机 / 机器人对象的使用说明（第 2、3 节）
* `tools/cubemx_post_export.ps1` —— CubeMX 导出后的修复 + 自检脚本（上一节）
* `User/App/robot/robot.md` —— 机器人层（`Robot_Init` / `OSTaskInit` / 任务表）
