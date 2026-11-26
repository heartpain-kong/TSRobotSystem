# bsp_gpio — GPIO 通用接口

文件：`bsp_gpio.h` / `bsp_gpio.cpp`。定位参考 `cod-h7-template-main/BSP/bsp_gpio` 与 `basic_framework/bsp/gpio`：
**薄封装**，只提供写/读/翻转/批量写。

## ⚠️ 为什么没有 `BSP_GPIO_Init()`

引脚的模式与初值**统一由 CubeMX 的 `MX_GPIO_Init()` 负责**（在 `Core/Src/gpio.c`，
并且已经和我们其它配置一样被 `tools/cubemx_post_export.ps1` 照看）。
两处都配容易互相踩（谁后跑谁生效），所以这一层**只做读写**。

要改某个引脚的作用 → **去 CubeMX 改**，不要在这里加初始化。

## 接口

| 接口 | 说明 |
| --- | --- |
| `BSP_GPIO_Write(gpiox, pin, state)` | 写引脚 |
| `BSP_GPIO_Read(gpiox, pin)` | 读引脚 |
| `BSP_GPIO_Toggle(gpiox, pin)` | 翻转（指示灯闪一下）|
| `BSP_GPIO_Write_Multi(gpiox, pins, state)` | 同端口一次写多个引脚 |
| `BSP_GPIO_Write_Active(gpiox, pin, active_level, active)` | 按"低有效/高有效"语义写，省得业务代码里到处写三目运算 |

```cpp
BSP_GPIO_Write(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);                     // 点灯
BSP_GPIO_Write_Active(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET, true);      // 低有效器件:拉低 = 有效
```

## 已知的本工程引脚（来自 `main.h`，由 CubeMX 生成）

| 引脚 | 标签 | 用途 |
| --- | --- | --- |
| PC15 | `POWER_5V` | 5V 输出使能（**当前初始化为 RESET = 关闭**）|
| PC0 / PC3 | `ACCEL_CS` / `GYRO_CS` | BMI088 两个片选（软件控制，见 `bsp_spi.md`）|
| PC13 / PC14 | （无标签）| 输出，CubeMX 里初值 RESET |
| PE10 / PE12 | `ACCEL_INT` / `GYRO_INT` | IMU 中断（上升沿）|
| PA1/PA3/PA7/PD11/PE11 | `W25Q64_*` | OSPI Flash（见 `bsp_ospi.md`）|

**没有**配按键/LED/蜂鸣器引脚 —— 需要的话在 CubeMX 里加（见 `dev_buzzer.md` 的清单）。

## 相关

* `User/Bsp/pwm/bsp_pwm.md` —— PWM 接口（蜂鸣器/加热/舵机都用它）
* `User/Device/buzzer/dev_buzzer.md` —— 蜂鸣器
