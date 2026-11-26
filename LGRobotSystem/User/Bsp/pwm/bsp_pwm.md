# bsp_pwm — PWM 通用接口

文件：`bsp_pwm.h` / `bsp_pwm.cpp`。定位参考 `cod-h7-template-main/BSP/bsp_pwm` 与 `basic_framework/bsp/pwm`。

只管「让某个 TIM 通道输出 PWM、频率多少、占空比多少」，不关心接的是蜂鸣器/加热片/舵机 —— 那些在器件层。

## ★ 频率怎么算（本层统一约定，和参考工程一致）

**让计数器时钟固定 = 1 MHz**，于是：

```
ARR = 1000000 / freq  - 1          ⇒ 改频率只改 ARR，不用重 init
PSC = 定时器时钟 / 1MHz - 1        ⇒ 由 BSP_PWM_Init(htim, ch, timer_clock_hz) 算好
```

* 本工程 APB1/APB2 的定时器时钟都是 **160 MHz**（APB 预分频 ≠ 1 时定时器时钟 = 2×PCLK）
  ⇒ `PSC = 160 - 1 = 159`；
* 参考工程 `dm02_test` 的 `dev_buzzer.h` 注释写的「主频 240M、psc 240、arr 250 ⇒ 4000Hz」
  就是同一套算法（它 `1MHz/250 = 4000Hz`）。

限幅 **20 Hz ~ 20 kHz**（低于 20 Hz 听不见；蜂鸣器有效音域大约 G3~A7）。

## 接口

| 接口 | 说明 |
| --- | --- |
| `BSP_PWM_Init(htim, channel, timer_clock_hz)` | 设 PSC/ARR(默认 1kHz) → `HAL_TIM_PWM_Init` → `ConfigChannel` → `Start` |
| `BSP_PWM_Start / Stop(htim, channel)` | 起停输出 |
| `BSP_PWM_Set_Freq(htim, freq_hz)` | 改频率（只改 ARR；传 0 = 停）|
| `BSP_PWM_Set_Duty(htim, channel, duty)` | 设占空比 0~1 |
| `BSP_PWM_Set_Compare(htim, channel, compare)` | 直接设比较值（WS2812 这类位编码需要）|
| `BSP_PWM_Get_Autoreload(htim)` | 取 ARR |

```cpp
BSP_PWM_Init(&htim3, TIM_CHANNEL_4, 160000000u);   // 计数器 = 1MHz
BSP_PWM_Set_Freq(&htim3, 1000);                    // 1kHz
BSP_PWM_Set_Duty(&htim3, TIM_CHANNEL_4, 0.5f);     // 50%
```

## 注意

* **改频率会改周期**，占空比（比较值不变时）会跟着变 ⇒ 需要保持响度就重设一次 compare
  （`Buzzer_Set_Sound()` 里已经按「先频率、再比较值」的顺序做了 ✓）；
* 同一 TIM 的**不同通道共享 ARR**：如果 A 通道要 1kHz、B 通道要 20kHz，得用**两个 TIM**；
* 前置条件：TIM 已在 CubeMX 里配成 **PWM Generation**，且 `MX_TIMx_Init()` 跑过。

## 本工程已配好的 PWM 通道

| 引脚 | 通道 | 备注 |
| --- | --- | --- |
| PE9 / PE13 | TIM1_CH1 / CH3 | —— |
| PA0 / PA2 | TIM2_CH1 / CH3 | —— |
| **PB1** | **TIM3_CH4** | 和同板工程 `dm02_test` 里 `HEATER__PWM` 是同一根网络 ⇒ **本板多半是加热** |
| PB15 | **TIM12_CH2** | **本工程未配**；同板工程里这是 `BUZZER__PWM` ⇒ 要用蜂鸣器请在 CubeMX 里开它 |

## 相关

* `User/Device/buzzer/dev_buzzer.md` —— 蜂鸣器（用它 + 音名宏 + 非阻塞响一声）
* `User/Bsp/gpio/bsp_gpio.md` —— GPIO 接口
