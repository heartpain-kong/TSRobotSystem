# bsp_timestamp — 长时间戳（TIM5 自由运行计数器，**无中断**）

文件：`bsp_timestamp.h` / `bsp_timestamp.cpp`。
同源参考：`E:\rm\dm02_test\User_File\1_Middleware\System\Timestamp\sys_timestamp.*`
（头文件写的"PSC 分频到 1MHz、ARR = 3600000000、1 小时溢出一次"就是我们 TIM5 的配置 ✓）。

## 1. 原理（就这么简单）

```
TIM5:  PSC = 160-1        → 计数器 1 MHz  ⇒  **CNT 的值就是微秒数**（读到直接用，不用换算）
       ARR = 3600000000-1 → 3600 s(1 小时)才回绕一次
```

`Timestamp_Init(&htim5)` 内部只做两件事：**计数器清零** + **`HAL_TIM_Base_Start`**（**不是** `_IT` ✗）。

## 2. ★ 为什么**不开中断**（和 `dm02_test` 的唯一大区别）

| | DWT（`bsp_dwt`，保留 ✓）| TIM5（本模块 ✓）|
|---|---|---|
| 回绕周期 | 32 位 @640 MHz ⇒ **6.7 s** ✗ | 32 位 @1 MHz ⇒ **3600 s** ✓ |
| "必须多久读一次才不丢" | **6.7 s** ✗ | **1 小时** ✓✓（**537 倍**宽容）|
| 开中断的必要性 | —— | **不需要** ✓：软件读法的窗口正好也是 1 小时，**根本不会漏** |

* 少一条 NVIC 线 = 少一个要按 FreeRTOS 规矩排优先级、只能在 ISR 里调 `...FromISR` 的中断源 ✓
* DWT 的真实隐患正好在这里 ✓：`DWT_Delay` 用裸 `CYCCNT`、不更新回绕计数 ✗ ⇒ 一旦出现 **>6.7 s 的空档**
  （例如**调试器 halt 十几秒** ✗），时间轴会**跳 6.7 s** ✗ → 所有超时判断误触发 ✗。
  换成 TIM5 后这个窗口变成 1 小时 ⇒ 实际等于**没有了** ✓。

## 3. 与 `bsp_dwt` 的分工（两个都留，各管一段）

| 用途 | 用谁 | 为什么 |
|---|---|---|
| 短延时 `DWT_Delay(0.03f)`、测代码/任务耗时 `DWT_TIME_ELAPSE` | **DWT** ✓ | 1 个 CPU 周期 = 1.56 ns（ns 是它白送的，不是我们需要的 ✓）|
| 超时判断、节流、日志时间戳（µs / ms）| **TIM5** ✓ | 1 µs 分辨率够控制用；回绕窗口 1 小时；**不受 CPU 变频/Stop 影响** ✓ |

> **需要 ns 级吗？不需要** ✓ —— 控制环 1 kHz = 1000 µs 周期，1 µs 就是它的 0.1% ✓；
> 保留 DWT 的理由是"零外设占用 + 不用配置就能用"，**不是** ns ✗。

## 4. API

| 接口 | 说明 |
|---|---|
| `Timestamp_Init(&htim5)` | 启动（`NULL` = 用默认 `&htim5`）；内部 `HAL_TIM_Base_Start`，**不开中断** |
| `Timestamp_Is_Inited()` | 是否已启动 |
| **`Timestamp_Get_Us32()`** | **主力**：直接读 CNT（µs）；71.6 分钟数值回绕 ⇒ 判超时**必须用差值** |
| `Timestamp_Elapsed_Us32(start)` | 从起点到现在的 µs 数（无符号减法天然跨回绕，间隔 < 71.6 分钟绝对正确）|
| `Timestamp_Get_Us()` | 64 位 µs（软件计回绕，量程约 49 万年）|
| `Timestamp_Get_Ms()` | 64 位 ms |
| `Timestamp_Update()` | 软件计回绕：周期任务里每拍调一次（1 kHz 下最多 1 小时后才用到）|

## 5. 用法

```cpp
/* Robot_Init()(已接好 ✓) */
Timestamp_Init(&htim5);

/* 任何要判超时的地方 */
uint32_t t0 = Timestamp_Get_Us32();
if (Timestamp_Elapsed_Us32(t0) > 500u) { /* 超时 500µs */ }

/* 要长时间戳(日志/绝对时间) */
uint64_t us = Timestamp_Get_Us();

/* Robot_Task()(已接好 ✓，500 Hz) */
Timestamp_Update();
```

## 6. 两个实现细节（都处理了）

1. **"跨回绕读"毛刺** ✓：`Get_Us()` 里先读 CNT、**立刻**判断是否刚回绕（`now < s_last_cnt`），
   然后拼接 `wrap × 3.6e9 + now` ⇒ 不会出现"先读到旧 wrap、再读到新 CNT"而**凭空少 1 小时**的毛刺 ✗。
   （`dm02_test` 那版没有这层保护，理论上每小时有一个 µs 级窗口会出错 ✓）
2. **临界区极短且可嵌套** ✓：用 `__get_PRIMASK()/__set_PRIMASK()` 保存恢复，不破坏调用者的中断状态。
   `Get_Us32()` 只是**一条寄存器读**，完全不需要临界区 ✓。

## 7. 验证

```
build      exit 0
符号       bsp_timestamp.cpp.obj 依赖 HAL_TIM_Base_Start（不是 _Start_IT）✓ ⇒ 确认无中断
自检       tools/cubemx_post_export.ps1 已把 User/Bsp/timestamp 纳入目录检查 ✓
```

上板验证：把 `Timestamp_Get_Us32()` 读两次、中间 `DWT_Delay(1.0f)`，两者差值应约等于 **1,000,000** ✓。

## 8. 相关

* `User/Bsp/dwt/bsp_dwt.md`（短延时 / 测周期，分工见上）
* `User/Device/w25q64/`、`bmi088/`、`buzzer/`、`ws2812/`（目前的超时/节流仍用 `DWT_Get_Ms()`，
  按需逐个换成 `Timestamp_Get_Us32()` —— 见下一步 ✓）
