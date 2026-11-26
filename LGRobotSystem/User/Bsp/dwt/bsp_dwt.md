# bsp_dwt — Cortex-M7 内核自带的 DWT->CYCCNT 微秒级时间戳

不占用任何定时器：直接用内核里的 **DWT 周期计数器 `DWT->CYCCNT`** 做时基，对外提供
64 位绝对时间戳、1us 步进的 32 位计数器、两次采样之间的间隔，以及 ISR 可用的忙等延时。

本模块模仿参考工程 `basic_framework-master\bsp\dwt`（王洪喜那套 DWT 时间戳），按本工程的
实际情况做了四处改造（见下）。

| 文件 | 内容 |
|---|---|
| `bsp_dwt.h` | 类型、宏与接口声明 |
| `bsp_dwt.cpp` | 实现（初始化、回绕拼接、周期→微秒换算、延时） |

## 相对参考工程的四处改造

### ① 主频自动取 —— 不用像参考工程那样手填 MHz（填错了时间就是错的）

取法：

```c
CPU(内核)时钟 = HAL_RCC_GetSysClockFreq() / D1CPRE 分频
```

> ⚠️⚠️ **H7 上 `SystemCoreClock` 是 `HCLK = 160 MHz`，而 `CYCCNT` 走的是内核时钟
> `D1CPRE = 640 MHz`，两者差 4 倍 —— 绝不能拿 `SystemCoreClock` 换算！**
>
> 同时注意：内核时钟也**不是** `HAL_RCC_GetHCLKFreq()`、**不是** `SystemCoreClock`
> —— 那两个都是 HCLK = 160 MHz。

想手动指定也行：`DWT_Init(640)` 传 MHz 即可。

### ② 回绕扩展用临界区（PRIMASK 存取）保护

替换掉参考工程里那个 `bit_locker` 标志位（标志位在中断打断时仍可能丢一次回绕）；
64 位拼接写成：

```c
((uint64)回绕次数 << 32) | CYCCNT
```

参考工程写的是 `* UINT32_MAX`，每回绕一次就少 1 个周期，是它的一处笔误。

### ③ 忙等用整数周期比较

不在 `while` 循环里做浮点乘法（参考工程是把 float 乘法写在 `while` 里）。

### ④ 另外提供本工程调度需要的 `DWT_Get_Us32()`

除了参考工程那套 s/ms/us 分解，本模块还提供 `DWT_Get_Us32()` —— 它是
**“1us 步进的 32 位计数器”**（71.6 分钟回绕，**差值比较环绕安全**）。
`motor_mit` 的 `Control_Freq` 调度用的就是它（等价于原来的 `BSP_Timestamp_Get_Us32`）。

## 三条时间线，各管各的，别混用

| 时间线 | 来源 | 步进 | 用途 |
|---|---|---|---|
| FreeRTOS tick | SysTick | 1 ms | 任务周期 / 延时：`osDelayUntil`、`osKernelGetTickCount` |
| HAL tick | TIM4 | 1 ms | HAL 超时：`HAL_GetTick`、`HAL_Delay`（**别改它频率**）|
| 本模块 | DWT->CYCCNT | 1 us | 控制周期细分、任意频率调度、耗时 / 抖动测量 |

## 接口一览

### 类型与宏

| 名称 | 作用 | 说明 |
|---|---|---|
| `DWT_Time_t` | 时间轴结构体（s / ms / us 分解形式，与参考工程同名） | `uint32_t s` 秒；`uint16_t ms` 毫秒(0~999)；`uint16_t us` 微秒(0~999) |
| `DWT_TIME_ELAPSE(dt, code)` | 取一段代码的执行时间（秒），结果写进 `dt` | 用法：`float dt; DWT_TIME_ELAPSE(dt, { 被测代码 });`。与参考工程的区别：这里**不打印日志**（本工程没有 RTT / 日志模块）|

### 初始化与诊断

| 接口 | 作用 | 返回值 |
|---|---|---|
| `void DWT_Init(uint32_t CPU_Freq_MHz)` | 初始化 DWT 周期计数器。`CPU_Freq_MHz` = CPU(内核)时钟频率，**传 0 = 自动取**（`HAL_RCC_GetSysClockFreq() / D1CPRE`，本工程 = 640 MHz）。必须在调度器启动前调用 | void |
| `bool DWT_Is_Inited(void)` | DWT 是否已经初始化 | 已初始化 = true |
| `uint32_t DWT_Get_CPU_Freq_MHz(void)` | 实际使用的 CPU 主频（MHz）—— 诊断用：**如果时间明显差整数倍，先看这个值** | 主频，单位 MHz |

### 取时间

| 接口 | 作用 | 返回值 |
|---|---|---|
| `uint64_t DWT_GetTimeline_us(void)` | 绝对时间戳（微秒，64 位）。精确到 1 个周期（640 MHz 下 1.56 ns），截断到 us | 微秒；未初始化时返回 0 |
| `uint32_t DWT_Get_Us32(void)` | 时间戳的低 32 位（= 1us 步进的 32 位计数器，71.6 分钟回绕）。给 `Control_Freq` 调度这类只需要相对时间的场合用，省一次 64 位运算；差值比较仍然环绕安全 | 微秒的低 32 位 |
| `uint64_t DWT_Get_Ms(void)` | 绝对时间戳（毫秒，64 位） | 毫秒 |
| `float DWT_GetTimeline_s(void)` | 绝对时间戳（秒，浮点）| 秒 |
| `float DWT_GetTimeline_ms(void)` | 绝对时间戳（毫秒，浮点）| 毫秒 |
| `void DWT_SysTimeUpdate(void)` | 刷新内部时间轴缓存（与参考工程同名）。参考工程要求“长时间不读时间轴就手动调一次”，本工程**每次读都会更新**，这个函数只是给“想先把缓存刷好再取”的场合留的接口 | void |
| `void DWT_Get_SysTime(DWT_Time_t *out)` | 取 s/ms/us 分解形式的时间（内部先刷新缓存）| void（`out` 为 `nullptr` 时直接返回）|

> ⚠️ `DWT_GetTimeline_s()` 是浮点：**float 尾数 24 位**，开机 1 小时分辨率约 0.24 ms，
> 24 小时约 5 ms。**要精确请用 `DWT_GetTimeline_us()`。**

### 间隔与延时

| 接口 | 作用 | 返回值 |
|---|---|---|
| `float DWT_GetDeltaT(uint32_t *cnt_last)` | 取两次调用之间的间隔（秒）。`cnt_last` 传入上一次的 CYCCNT 快照（调用方保存），函数内部会更新它 | 秒；`cnt_last == nullptr` 或主频为 0 时返回 `0.0f` |
| `double DWT_GetDeltaT64(uint32_t *cnt_last)` | 同上，双精度版 | 秒；异常输入返回 `0.0` |
| `void DWT_Delay_Us(uint32_t us)` | 微秒级忙等（ISR 或极短硬时序用）| void |
| `void DWT_Delay(float Delay_s)` | 秒级忙等（与参考工程同名，**参数单位是秒**）。**不受中断开关影响，可以在临界区里用**（临界区里不能用 `HAL_Delay`）| void |
| `int32_t DWT_Elapsed_Us32(uint32_t now_us, uint32_t t0_us)` | 求两个 32 位时间戳的间隔（微秒），自动处理回绕 | `now - t0`；负值表示 `now` 早于 `t0`（差值在 ±35 分钟内始终正确）|

> ⚠️ **任务里不要用 `DWT_Delay_Us()`** —— 它不让出 CPU，会饿死低优先级任务；任务请用
> `osDelay` / `osDelayUntil`。单次忙等不要超过一个回绕周期（640 MHz 下约 6.7 s）。

## 使用注意

* **CYCCNT 只有 32 位**：640 MHz 下 **6.71 秒回绕一次**。回绕计数靠“每次读时间时累加”，
  所以 **至少每 6.7 秒要读一次时间**（控制任务 1 kHz 在读，不会漏）。
* **调试器复位内核 / 内核进入 debug 停走时 CYCCNT 会清零或暂停** —— 只在调试时才遇到。
* **`DWT_Init()` 必须在用到时间戳之前调用**（`main.c` 的 `USER CODE BEGIN 2` 里，
  调度器启动前；本工程实际调用为 `DWT_Init(0)`）。
* **临界区保护**：回绕计数在每次读时更新，任务和中断都会读时间戳，所以更新动作用
  PRIMASK 临界区包住，可嵌套（保存 / 恢复调用前的中断状态）。

## 实现要点（`bsp_dwt.cpp`）

内部状态（均为文件内 static）：

| 变量 | 含义 |
|---|---|
| `s_cpu_freq_hz` | CPU(内核)时钟 Hz —— 注意是 D1CPRE 之后的**内核时钟**，不是 HCLK |
| `s_cyc_per_us` | 每微秒的周期数；**仅在主频是 1 MHz 整数倍时有效**，否则为 0，走更通用的换算 |
| `s_wrap_count` | CYCCNT 回绕次数（32 位计数器，640 MHz 下 6.71 s 回绕一次）|
| `s_last_cnt` | 上一次读到的 CYCCNT（判回绕用）|
| `s_sys_time` | 时间轴缓存（s / ms / us）|
| `s_inited` | 是否已初始化 |

**D1CPRE 分频系数**：读 `RCC->D1CFGR` 的 `RCC_D1CFGR_D1CPRE` 字段，寄存器编码同 HPRE
（`0b0xxx` = 1 分频，`0b1000` = 2，`0b1001` = 4 ……），即内核时钟 = SYSCLK / 该值。

**读 64 位周期数**：`DWT->CYCCNT` 比上次小 ⇒ 认为回绕了一次，回绕计数 +1；拼接成
`((uint64_t)s_wrap_count << 32) | cnt_now`。整段用 `__get_PRIMASK()` / `__disable_irq()` /
`__set_PRIMASK()` 保护。

**周期数 → 微秒**：主频是 1 MHz 整数倍（640 MHz 就是）时直接整除 `s_cyc_per_us`；
否则分两步算（先整除、余数再乘 1000000 后除），避免溢出；主频为 0 时返回 0。

**初始化三步（使能跟踪与周期计数器）**：

```c
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;    /* 使能 DWT 外设 */
DWT->CYCCNT = 0u;                                  /* 计数清零 */
DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;              /* 开 CYCCNT */
```

随后清回绕计数与时间轴缓存、以当前 CYCCNT 作为 `s_last_cnt`、置 `s_inited = true`。
`s_cyc_per_us` 只在 `s_cpu_freq_hz` 非 0 且能被 1000000 整除时取
`s_cpu_freq_hz / 1000000`，否则为 0。

**忙等实现**：换算成周期数 `ticks`，然后

```c
uint32_t start = DWT->CYCCNT;
while ((uint32_t)(DWT->CYCCNT - start) < ticks) { /* 忙等 */ }
```

`DWT_GetDeltaT()` / `DWT_GetDeltaT64()` 里的差值同样按
`(uint32_t)(cnt_now - *cnt_last)` 算（回绕安全），并把 `cnt_now` 写回 `*cnt_last`。

## 最小用法

```c
/* main.c 的 USER CODE BEGIN 2 里（调度器启动前）—— 传 0 = 自动取内核时钟 */
DWT_Init(0);

uint32_t us = DWT_Get_Us32();               /* 1us 步进的 32 位计数器 */
uint64_t t  = DWT_GetTimeline_us();         /* 64 位绝对微秒时间戳 */
bool     ok = DWT_Is_Inited();              /* 初始化是否成功 */
uint32_t f  = DWT_Get_CPU_Freq_MHz();       /* 回读实际主频（诊断"差整数倍"用）*/

uint32_t last;
float    dt = DWT_GetDeltaT(&last);         /* 两次调用之间的秒数 */

float dt2;
DWT_TIME_ELAPSE(dt2, { /* 被测代码 */ });    /* 量一段代码的耗时 */

DWT_Delay_Us(5);                            /* ISR 里的微秒忙等（任务里别用）*/
```

## 相关

* `bsp_dwt.h` / `bsp_dwt.cpp` —— 本模块。
* `main.c` —— `USER CODE BEGIN 2` 中调用 `DWT_Init(0)`。
* `motor_mit.h` —— 频率调度（`Is_Send_Due()` / `Mark_Sent()`）用 `DWT_Get_Us32()` 当时基。
* 参考工程：`basic_framework-master\bsp\dwt`。
* 更上层的电机侧说明：`User\Device\motor\电机使用说明.md` 第 10 节「时间基准（DWT 微秒时间戳）」。
