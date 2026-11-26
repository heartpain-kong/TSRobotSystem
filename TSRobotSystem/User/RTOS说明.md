# RTOS 说明（FreeRTOS + CMSIS-RTOS2）

> 一句话：**FreeRTOS Kernel V10.6.2**，用 CubeMX 生成的 **CMSIS-RTOS2 封装**（`osThreadNew` / `osDelayUntil`），
> 9 个周期任务 + 1 个消息中枢，全部按 **绝对时刻** 调度，不累积漂移。
> 配置在 `Core/Inc/FreeRTOSConfig.h`，任务表在 `User/App/robot/robot_def.h`。

---

## 1. 用的是哪套 RTOS，在哪

| 项 | 值 | 文件 |
|---|---|---|
| 内核 | FreeRTOS Kernel **V10.6.2** | `Middlewares/Third_Party/FreeRTOS/` |
| 应用层接口 | **CMSIS-RTOS2**（`osKernelInitialize` / `osThreadNew` / `osDelayUntil`）| `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/` |
| 内核配置 | tick 1 ms、56 级优先级、heap_4、32 KB 堆 | `Core/Inc/FreeRTOSConfig.h` |
| 任务创建 | `OSTaskInit()`（`robot_task.cpp:269`），由 `MX_FREERTOS_Init()` 在 `osKernelInitialize()` **之后**调用 | `User/App/robot/robot_task.cpp`、`Core/Src/freertos.c` |
| 启动 | `main()` → 各 `MX_*_Init()` → `Robot_Init()`（设备/子系统）→ `osKernelInitialize()` → `MX_FREERTOS_Init()`（⇒ `OSTaskInit()` 建任务）→ `osKernelStart()` | `Core/Src/main.c` / `Core/Src/freertos.c` |

> **为什么建任务不在 `Robot_Init()` 里**：CMSIS-RTOS2 要求先 `osKernelInitialize()` 才能 `osThreadNew()`，
> 而 `main.c` 里 `Robot_Init()` 在 `osKernelInitialize()` **之前** ⇒ 建任务只能放 `freertos.c` 的
> `MX_FREERTOS_Init()` 里。`Robot_Init()` 只做"初始化设备/子系统"，不建任务。

> 为什么用 CMSIS-RTOS2 而不是裸 FreeRTOS API：CubeMX 默认就这么生成，`osThreadNew` 的
> `attr.stack_size` 直接是**字节**、`osDelayUntil` 直接吃 tick，换内核版本时业务代码不用改。

---

## 2. 内核配置（`Core/Inc/FreeRTOSConfig.h`）

| 配置项 | 值 | 含义 / 影响 |
|---|---|---|
| `configTICK_RATE_HZ` | **1000** | 1 tick = 1 ms，所有任务周期都是它的整数倍 |
| `configMAX_PRIORITIES` | **56** | CMSIS 的 `osPriorityHigh=40` / `Normal=24` / `Low=8` 都够用 |
| `configUSE_PREEMPTION` | 1 | 抢占式：高优先级任务就绪立刻抢 |
| `configTOTAL_HEAP_SIZE` | **32768 B** | FreeRTOS 堆；**任务栈就从这里分配**，见第 5 节 |
| `heap_4.c` | — | 能合并碎片，适合"启动时一次性建任务"的用法 |
| `configMINIMAL_STACK_SIZE` | 128 | 只给空闲任务用，业务任务的栈都是显式指定的 |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | 溢出会进 `vApplicationStackOverflowHook`（`Core/Src/freertos.c`）|
| `configUSE_MALLOC_FAILED_HOOK` | 1 | 堆不够进 `vApplicationMallocFailedHook` |
| `configGENERATE_RUN_TIME_STATS` | 1 | 可以用 `uxTaskGetSystemState` 看各任务 CPU 占用，配合 `bsp_dwt` 的基准 |

---

## 3. 任务表（`User/App/robot/robot_def.h`）

| 任务名 | 周期 | 优先级 | 栈 (B) | 干什么 | 代码 |
|---|---|---|---|---|---|
| `motor` | **1 ms (1 kHz)** | High | 2048 | 写电机目标、跑各电机闭环、按各自频率发 CAN 帧 | `App/motor_task` |
| `imu` | **1 ms (1 kHz)** | High | 2048 | BMI088（SPI2）读数 → 四元数 EKF → 发 IMU 话题 | `App/imu` |
| `robot` | 2 ms (500 Hz) | Normal | 2048 | 高层决策：模式、门控（`Motion_Allowed`）、给各子系统发参考 | `App/robot` |
| `cmd` | 2 ms (500 Hz) | Normal | 3072 | 遥控/裁判/上位机/485 → 统一 `Cmd_Data_t`；**也是上下 MCU 链路的 500 Hz 收发节拍** | `App/cmd` |
| `chassis` | 2 ms (500 Hz) | Normal | 2048 | 底盘解算、舵向/驱动目标、里程计、云台 6020 保持、找零 | `App/chassis` |
| `gimbal` | 2 ms (500 Hz) | Normal | 2048 | 云台 yaw/pitch 关节角 + 前馈（上板：pitch = DM4310）| `App/gimbal` |
| `highRise` | 2 ms (500 Hz) | Normal | 2048 | 摩擦轮 / 拨弹（或升降）| `App/highRise` |
| `debug` | 10 ms (100 Hz) | Low | **4096** | `0x5A` 遥测帧、JSON、`0x5C` 调参回包 | `App/debug` |
| `error` | 10 ms (100 Hz) | Low | **4096** | 错误级别评估、蜂鸣器/RGB、关键数据写日志环形 | `App/error_task` |

> 为什么 `error` / `debug` 要 4 KB：里面跑 `snprintf` / `vsnprintf`，newlib 的实现吃栈（3 KB 实测偏紧）。
> 为什么 `motor` / `imu` 给 High：1 kHz 的任务被挤掉一拍就是一个控制周期，抖动直接反映到电流环。

---

## 4. 调度方式：绝对时刻 + 不累积漂移

每个周期任务都是这个骨架（`robot_task.cpp`）：

```c
void start_motor_task(void *argument)
{
    motor_task_handle = osThreadGetId();
    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        Motor_Task();
        next += MOTOR_TASK_PERIOD_MS;      // ★ 累加"理论时刻",不是从现在开始算
        if ((int32_t)(osKernelGetTickCount() - next) > 0) { motor_task_overrun++; }
        osDelayUntil(next);                // 绝对时刻等:单拍晚一点,后面的节拍不跟着漂
    }
}
```

* **`osDelayUntil` + 累加 `next`**：即使某拍算得慢，后面的节拍点仍是 1 ms / 2 ms 的整数倍，
  500 Hz 不会变成 499 Hz（用 `osDelay(1)` 会累积漂移）。
* **超拍计数**：`motor_task_overrun` / `robot_task_overrun` / `imu_task_overrun`（`robot_task.h`）
  涨 ⇒ 这一拍没算完，下一拍已经到点。`imu_task_overrun` 一直涨说明 1 kHz 跑 EKF 太重，
  把 `IMU_TASK_PERIOD_MS` 改成 2（500 Hz）。
* **抢占**：高优先级就绪立刻切，所以 `cmd`/`chassis` 不会被 `debug` 的串口打印拖住。

---

## 5. 栈与堆：钱从哪来、怎么看出事

**任务栈不是独立数组，而是从 FreeRTOS 堆（`configTOTAL_HEAP_SIZE = 32 KB`）里分的**，
所以任务栈总和 + 内核对象不能超过 32 KB。当前任务表加起来约 **23 KB**，留 ~9 KB 余量。

| 症状 | 含义 | 处理 |
|---|---|---|
| `osThreadNew` 返回 `NULL` | 堆不够（**不报错，只是任务不存在**）| `g_thread_create_failed++`（`robot_task.h`）；加大 `configTOTAL_HEAP_SIZE` 或减栈 |
| 进 `vApplicationStackOverflowHook` | 某任务栈溢出（`configCHECK_FOR_STACK_OVERFLOW = 2` 查的是栈尾哨兵）| 加大该任务的 `*_TASK_STACK` |
| 进 `vApplicationMallocFailedHook` | 运行期申请堆失败 | 同上，或改成启动时一次性分配 |

**栈水位**（`robot.cpp` 每拍刷新，单位 = word = 4 B，越小越接近溢出）：

```c
hwm_motor / hwm_robot / hwm_error / hwm_cmd / hwm_imu / hwm_chassis / hwm_gimbal / hwm_highRise
```

遥测里能直接看（`debug_task` 的 `hwm_*` 字段）。**经验值：水位 < 32 word（128 B）就该加栈。**

---

## 6. 任务间通信：为什么没有队列和互斥锁

本工程用 **`msg_center` 的环形数组 + 关中断临界区**，不用 `osMessageQueue` / `osMutex`：

```text
ISR（CAN/UART 回调）──Msg_Publish(topic, &data, len)──► 环形槽位
任务（500Hz/1kHz）───Msg_Latest(topic, &buf, &seq)────► 取最新一份
```

| 设计 | 原因 |
|---|---|
| 广播型（`Msg_Latest` 不弹出）| 一个 IMU 帧要被 chassis / gimbal / robot 同时读，队列"取走就没了"不适用 |
| 发布/读取只在 `__disable_irq()` 里挪指针 | 临界区只有几条指令，比 mutex 便宜，也不会出现优先级反转 |
| 定长槽位 + `static_assert(sizeof(T) <= MSG_SLOT_SIZE)` | 编译期就挡住"结构体变大把消息挤爆" |
| 序号 `Seq` 判新 | 任务不需要"每条都处理"，只要"最新一条别用旧的" |
| 覆盖计数 `Overwritten` | 消费者跟不上时能看出来（涨 = 丢帧） |

> ISR 里**不调用任何 FreeRTOS API**（也不用 `*FromISR` 版本）：回调只做"解析 + 推进环形 + 置标志"，
> 真正的控制逻辑全在任务里。

---

## 7. 加一个任务要做的事

1. `robot_def.h` 加三个宏：`XXX_TASK_PERIOD_MS` / `XXX_TASK_PRIORITY` / `XXX_TASK_STACK`
2. `robot_task.cpp` 加 `start_xxx_task()`（抄第 4 节的骨架）+ `osThreadNew`，**必须检查返回值**
3. 栈从 2 KB 起，看 `hwm_xxx` 再调
4. 任务里**不许**：`HAL_Delay`、忙等、`printf` 大字符串、在中断里调 RTOS API
5. 更新本文第 3 节的表 + `代码架构框架.md`

---

## 8. ❌ 禁止（RTOS 红线）

| 禁止 | 后果 | 正确做法 |
|---|---|---|
| 周期任务里 `HAL_Delay` / 忙等 / 死循环 | 整个节拍崩，1 kHz 任务被顶掉 | 拆成状态机，靠周期推进 |
| 用 `osDelay(1)` 当周期 | 累积漂移，500 Hz 变 480 Hz | `osDelayUntil(next)` + `next += PERIOD` |
| ISR 里调 FreeRTOS API（含 `*FromISR`）| 优先级/临界区错误，偶发卡死 | ISR 只搬运 + 置标志，任务里处理 |
| `osThreadNew` 不判 `nullptr` | 堆不够是**静默失败**，任务直接不存在 | 每次都要判，失败计入 `g_thread_create_failed` |
| 任务栈按 word 估 | 单位是**字节**，会差 4 倍 | `attr.stack_size = 2048` 就是 2 KB |
| 在 `Robot_Init()` 里 `osThreadNew` | 那时还没 `osKernelInitialize()` | 建任务放 `OSTaskInit()`（`MX_FREERTOS_Init` 里调）|
| 跨任务共享可写全局变量 | 数据竞争，读到的可能是半截数据 | 走 `msg_center`；只读常量用 `extern const` |
| 往 1 kHz 任务里加活 | `*_overrun` 涨，控制周期抖动 | 见任务表预算；重活挪到 100 Hz 任务 |

---

## 9. 踩过的坑

* **`osThreadNew` 失败是静默的**：堆不够只返回 `NULL`，现象是"任务完全没跑"，不是报错。所以每次都判 `nullptr`。
* **栈单位是字节**：`attr.stack_size = 2048` 就是 2 KB，别按 word 算。
* **`osDelay(1)` 会漂**：1 ms tick 下每次至少等 1 ms、加上自身的执行时间，500 Hz 会慢慢变成 480 Hz。
* **别在 1 kHz 任务里做重活**：`imu` 任务里塞 EKF + 校准就够了，再多会顶掉 `motor`。
* **默认钩子是 `for(;;)`**：`vApplicationStackOverflowHook` / `vApplicationMallocFailedHook` 不实现的话，
  现象和 HardFault / `configASSERT` 死循环一模一样，调试器里分不出来 ⇒ 本工程在 `Core/Src/freertos.c` 里实现并打标记。
