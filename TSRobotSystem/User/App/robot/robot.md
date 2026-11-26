# robot — 机器人层(初始化 + RTOS 任务创建)

**位置**:`User/App/robot/`(与 `chassis` / `gimbal` / `motor_task` / `robot` 等子系统同级)。

**一句话**:仿 `basic_framework` 的 `application/robot.c` —— **`Robot_Init()` 是唯一需要
放进 `main()` 的函数**,内部把各 BSP/子系统初始化好,再由 `OSTaskInit()` 统一创建所有
持续运行的 RTOS 任务;`Robot_Task()` 是高层决策(跑一拍)。

| 文件 | 说明 |
| --- | --- |
| `robot.h` | **C 兼容**(`extern "C"`):`Robot_Init()` / `Robot_Task()` —— main.c 只包这个 |
| `robot_def.h` | 这台机器人的配置:子系统开关 + 任务周期/优先级/栈 |
| `robot_task.h` | 任务入口与句柄声明。**只应被 `robot.cpp` 包含**(照参考工程的约定) |
| `robot_task.cpp` | `OSTaskInit()` 建任务 + 各任务循环(含 DWT 超时监测) |
| `robot.cpp` | `Robot_Init()` / `Robot_Task()` |
| `robot.md` | 本文件(说明放同目录,不写进源码开头) |

## 调用链

```
main.c  USER CODE BEGIN 2
    DWT_Init(0);                      // ① 时间戳先起(电机频率调度用)
osKernelInitialize();                 // ② CMSIS-RTOS2 必须先初始化内核
MX_FREERTOS_Init()  → freertos.c 的 USER CODE BEGIN RTOS_THREADS
    Robot_Init();                     // ③ 机器人初始化(唯一合法位置:内核已初始化、调度器还没跑)
        ├─ can_filter_init()                  // CAN 过滤器(上电一次,要先于各总线 Start)
        ├─ motor_task_init()                   // 电机:只初始化 + 注册接收回调(不建任务)
        └─ OSTaskInit()                       // ★ 统一建所有任务(放在最后)
osKernelStart();                      // ④ 调度器接管,此后 main 里的代码不再执行
    ├─ MOTOR 任务 (1kHz)  → motor_task()      // 电机:算闭环 + 按各自频率发帧   ← 由 OSTaskInit 建
    └─ ROBOT 任务 (500Hz) → Robot_Task()      // 决策:遥控/视觉 → 给子系统发参考值 ← 由 OSTaskInit 建
```

> **任务统一在 `OSTaskInit()` 里建**（仿 `basic_framework`：所有持续运行的任务集中创建，
> 建任务的时机/参数只有一处）—— 子系统模块只提供"跑一拍"的函数（`motor_task()` 等），
> 自己不碰 `osThreadNew`。**周期/优先级/栈统一在 `robot_def.h` 配。**

> ⚠️ **为什么建任务必须放在 `freertos.c` 里（而不是 `main.c` 的 `USER CODE 2`）?**
> CMSIS-RTOS2 规定**先 `osKernelInitialize()` 才能建线程**,而 `main.c` 的 `USER CODE 2`
> 在 `osKernelInitialize()` **之前** ⇒ `OSTaskInit()` 只能发生在 `MX_FREERTOS_Init()` 里
> (它正好在 `osKernelInitialize()` 之后、`osKernelStart()` 之前)。
> 硬件/子系统初始化不受这个限制，所以留在 `main.c` 里更直观。
> （参考工程 `basic_framework` 用的是 CMSIS-RTOS **v1** 的 `osThreadCreate`,
> 那个在 FreeRTOS 下不要求先 init,所以它的 `RobotInit()` 能把建任务一起放 main 里 —— v2 不行。）

## 任务表

| 任务 | 周期 | 优先级 | 栈 | 任务体 | 配置宏(`robot_def.h`) | 由谁建 |
| --- | --- | --- | --- | --- | --- |
| MOTOR | 1 ms(1 kHz) | `osPriorityHigh` | 2 KB | `motor_task()` | `MOTOR_TASK_PERIOD_MS` / `_PRIORITY` / `_STACK` | `OSTaskInit()`(`start_motor_task`) |
| ROBOT | 2 ms(500 Hz) | `osPriorityNormal` | 2 KB | `Robot_Task()` | `ROBOT_TASK_PERIOD_MS` / `_PRIORITY` / `_STACK` | `OSTaskInit()`(`start_robot_task`) |

* 任务用 **`osDelayUntil(next)`** 按绝对时刻等待 ⇒ **不会累积漂移**(参考工程用的是 `osDelay`)。
* 每个任务循环里都用 DWT 量本拍耗时,超过周期就给 `xxx_task_overrun++` ——
  **调试时看这几个变量**:一直涨说明这一拍根本算不完(要么减东西,要么提优先级/加频率余量)。
* 任务句柄 `motor_task_handle` / `robot_task_handle` 也在全局,可确认任务真的建起来了。

## 加一个新任务(比如 IMU 姿态解算)

1. `robot_def.h`:`#define ROBOT_HAVE_INS 1` + 加 `INS_TASK_PERIOD_MS/_PRIORITY/_STACK`
2. 子系统模块提供**跑一拍**的函数(如 `ins_task.cpp` 的 `INS_Task()`,内部不要写死循环)
3. `robot_task.cpp`:加一个 `Start_INS_TASK()` 循环(照抄 MOTOR 那个)+ 在 `OSTaskInit()` 里
   `osThreadNew` 一下
4. `robot.cpp` 的 `Robot_Init()` 里加 `INS_Init()`(只初始化)
5. 重新 configure(`file(GLOB)` 是配置期展开的)

## ⚠️ 与参考工程的一处**有意差异**:不关中断

参考工程 `RobotInit()` 里有 `__disable_irq()`(并声明"初始化期间不许用中断和延时,
非要用只能用 `DWT_Delay()`")。本工程**没有关中断**,原因:

* 灵足(LZ)等模块的发送接口内部用 **`HAL_GetTick()`** 做 TX FIFO 超时等待
  (`fdcan_send_data_Exten_wait`),关中断后 SysTick 不涨 ⇒ **超时永远不触发,直接卡死**;
* 我们的初始化路径本来就不依赖中断,开着更安全。

如果你确定初始化路径完全不碰 `HAL_GetTick`/中断,想关也可以自己加。

## C / C++ 分层规则(踩过一次,别再踩)

* `main.c` / `freertos.c` / 各 CubeMX 外设 `.c` 都是 **C 文件** ⇒ **只能包 C 兼容头**
  (`robot.h`、`motor_task.h` 这种 `extern "C"` 纯声明)。
* **绝不能** include `motor_mit.h` / `motor_DM.h` 等 —— 那些是 C++ 类,
  报错是一长串 `unknown type name 'class'` / `'motor_Control_mode'`。
* 需要给 C 侧用就写一个 `extern "C"` 函数(本目录的 `Robot_Init()` 就是范例)。

## 相关

* `User/Device/motor/电机使用说明.md` —— 电机怎么用(初始化/接收/1kHz/PD/坑)
* `User/App/motor_task/motor_task.md` —— 电机子系统模块
* `Core/Src/main.md`、`Core/Src/freertos.md` —— CubeMX 侧的两个文件
