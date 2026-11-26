# dev_buzzer — 板载蜂鸣器（PWM）

文件：`dev_buzzer.h` / `dev_buzzer.cpp`。参照 `dm02_test/2_Device/BSP/Buzzer` +
`basic_framework/modules/alarm/buzzer`，按 RTOS 形态加了**非阻塞响一声**。

## 频率与响度怎么来

* **频率**：`bsp_pwm` 把计数器定在 1 MHz ⇒ `ARR = 1000000/freq`（`Buzzer_Set_Sound` 内部完成）；
* **响度**：比较值 = `ARR × loudness × 0.5`（半占空比最大声，和参考工程一致）。

## 接口

| 接口 | 说明 |
| --- | --- |
| `Buzzer_Init(&htim, channel, timer_clock_hz)` | 初始化（内部就是 `BSP_PWM_Init` + 静音）|
| `Buzzer_Set_Sound(freq_hz, loudness)` | 立即设声音（0 = 静音）|
| `Buzzer_Stop()` | 立即静音 |
| **`Buzzer_Beep(freq_hz, loudness, ms)`** | **非阻塞**响一声，到点由 `Buzzer_Task()` 自动停 |
| `Buzzer_Task()` | ★ 跑一拍（挂在周期任务里，1kHz/500Hz 都行）|
| `Buzzer_Is_Sounding()` | 当前是否在响 |

音名宏：`BUZZER_FREQ_C4/D4/E4/F4/G4/A4/B4/C5/D5/E5/G5/A5/C6`。

```cpp
/* 初始化(写在 Robot_Init 里) */
Buzzer_Init(&htim12, TIM_CHANNEL_2, 160000000u);

/* 报警:响 200ms,不阻塞调用处 —— 下一拍 Buzzer_Task 会把它停掉 */
if (motor.Has_Fault()) { Buzzer_Beep(BUZZER_FREQ_A5, 0.8f, 200u); }

/* 放音(阻塞式演奏就自己按节拍调 Set_Sound) */
Buzzer_Set_Sound(BUZZER_FREQ_C5, 0.7f);
```

## ⚠️ 用之前必须在 CubeMX 里做两件事

我们的 `.ioc` **目前没有配蜂鸣器引脚**。同板工程 `dm02_test` 用的是
**PB15 = TIM12_CH2**（标签 `BUZZER__PWM`），但**我们板子的引脚分配和它不完全一样**
（例如 PA7 在我们这儿被 OCTOSPI2 的 IO2 占了），所以**实际引脚请看你们自己的原理图/位号图**。

| 步骤 | CubeMX 里怎么做 |
| --- | --- |
| 1 | 找到蜂鸣器那个脚 → 设为 **TIMx_CHy 的 PWM Generation** |
| 2 | 该 TIM 的 **Prescaler/Period 随意**（`Buzzer_Init` 会重算 PSC/ARR）|
| 3 | 生成代码，然后在 `User/App/robot/robot.cpp::Robot_Init()` 里加一行 `Buzzer_Init(&htimX, TIM_CHANNEL_Y, 160000000u);` |
| 4 | 在 `Robot_Task()` 里加一行 `Buzzer_Task();`（已有 `BMI088_Task()` / `W25Q64_Task()` 的位置旁边）|

> 定时器时钟：本工程 APB1/APB2 的定时器都是 **160 MHz** ⇒ 第三个参数传 `160000000u`
> （不确定就传这个，`BSP_PWM_Init` 会用 `PSC = 160-1` 得到 1 MHz 计数）。

## 顺带：同板工程上还有的其它外设（我们暂未配）

| 外设 | 同板工程的引脚 | 我们 `.ioc` 状态 |
| --- | --- | --- |
| 蜂鸣器 | PB15 (TIM12_CH2, `BUZZER__PWM`) | ✗ 未配 |
| 用户按键 | PA15 (`KEY__INPUT`) | ✗ 未配（要配成 GPIO 输入 + 上拉）|
| WS2812 | PA7 (SPI1_MOSI, `WS2812_RX__SPI_MOSI`) | ✗ 未配 —— **我们这里 PA7 被 OSPI IO2 占了**，要用得换脚 |
| 加热 | PB1 (TIM3_CH4, `HEATER__PWM`) | ✓ **已配**（直接用 `bsp_pwm` 驱动即可）|

## 相关

* `User/Bsp/pwm/bsp_pwm.md` —— PWM 接口（1 MHz 计数器约定）
* `User/App/robot/robot.md` —— 接线位置（`Robot_Init()` / `Robot_Task()`）
