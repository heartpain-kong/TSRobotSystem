# dev_ws2812 — 板载 WS2812 灯带（SPI6 单线发送）

文件：`dev_ws2812.h` / `dev_ws2812.cpp`

## 1. 为什么用 SPI 驱动 WS2812

WS2812 用 800kHz 单线归零码：`T0H≈0.4µs` / `T1H≈0.8µs` / 周期 1.25µs。
把 **SPI 时钟设成 2.5MHz**，则 **3 个 SPI 位 ≈ 1.2µs ≈ 一个 LED 位**：

| LED 位 | SPI 位 | 高电平时间 |
|---|---|---|
| `0` | `100` | 0.4 µs |
| `1` | `110` | 0.8 µs |

⇒ 1 个颜色字节(8 位) → **3 个字节**，1 颗灯(GRB) → **9 字节** ⇒ 好算，也不用 PWM+DMA。

## 2. 硬件 / CubeMX 前提

| 项 | 值 |
|---|---|
| 数据线 | **PA7 = SPI6_MOSI**（`TX_Only_Simplex_Unidirect_Master`，不接 MISO）|
| SPI | SPI6 主机、2LINES_TXONLY、8bit、MSB first、Mode 0 |
| **★ 分频** | **必须由 CubeMX 把 Prescaler 设成 32** ⇒ 80MHz/32 = **2.5Mbit/s** |
| 灯数 | `WS2812_LED_NUM`（默认 8，按实际硬件改）|
| 缓冲 | `WS2812_LED_NUM*9 + 24` 字节 |

> ⚠️ **驱动不会硬改 CubeMX 的分频**（本工程纪律：CubeMX 是引脚/外设配置的唯一来源）。
> 它只把分频**读回来校验**：`WS2812_Is_Clock_Ok()` 返回 false 就说明 CubeMX 里没设成 32
> ⇒ **灯会显示乱码（但不会死机）**。默认是 prescaler 8 = 10Mbit/s，太快了。

## 3. ★★ 缓冲必须放在 D3 SRAM（否则灯不亮）

SPI6 的 DMA 请求在 H7 上**只能走 BDMA**（`spi.c`: `hdma_spi6_tx.Instance = BDMA_Channel1`），
而 **BDMA 只能访问 D3 SRAM（SRAM4 = `0x38000000`）**：

* 放在 RAM_D1（AXI SRAM `0x24000000`）⇒ BDMA **读不到** ⇒ 发出去是空数据 ⇒ **灯不亮**
  （表现为"不亮但不死机"：`WS2812_Task()` 20ms 超时后把忙标志掰回来）；
* 所以缓冲带 `__attribute__((section(".RAM_D3"), aligned(32)))`，
  对应 `STM32H723VGTX_FLASH.ld` 里的 `.RAM_D3 (NOLOAD) >RAM_D3` 段；
* D3 SRAM 默认**是可缓存区** ⇒ 启动 DMA 前要 `SCB_CleanDCache_by_Addr()`
  把刚写进去的颜色刷到物理内存，否则 BDMA 可能读到旧值；
* ⚠️ **`.ld` 是 CubeMX 生成的** ⇒ 该段要由 `tools/cubemx_post_export.ps1` 在导出后补回 ✓

> 同理：**ADC3** 也挂 BDMA，所以 `bsp_adc` 里 ADC3 用"按需阻塞单次转换"而**不走 DMA** ✓
> 反例：SPI2（BMI088）、OCTOSPI2（W25Q64）走的是 DMA1/DMA2/MDMA，**能**访问 RAM_D1 ✓

### ⚠️ CubeMX 导出后要手工加回（脚本只报警，不自动改）

`STM32H723VGTX_FLASH.ld` 是 **CubeMX 生成的** ✗ —— 每次导出都会把 `.RAM_D3` 段冲掉。
导出后跑 `tools/cubemx_post_export.ps1` 会**提示**这一项（§2.6），但不自动改写 `.ld`
（自动改写脚本本身的风险更高，不值得）。请把下面整段贴到 `.ld` 里 **`.AXI_SRAM` 段之后**：

```ld
  /* ★ BDMA 专用缓冲(D3 SRAM / SRAM4 = 0x38000000):
     SPI6 的 DMA 请求只能走 BDMA,而 BDMA 只能访问 D3 SRAM。
     ws2812_buffer 就放在这个段里。 */
  .RAM_D3 (NOLOAD) :
  {
    . = ALIGN(4);
    *(.RAM_D3)
    *(.RAM_D3*)
    . = ALIGN(4);
  } >RAM_D3
```

**怎么确认加对了** ✓：编译后看符号地址，应该是 **`0x38xxxxxx`**：

```
arm-none-eabi-nm build\TSRobotSystem.elf | findstr ws2812_buffer
# ✓ 38000000 b _ZL13ws2812_buffer      ← 对
# ✗ 2400xxxx b _ZL13ws2812_buffer      ← 段没加进去,灯不会亮
```

## 4. API

| 接口 | 说明 |
|---|---|
| `WS2812_Init(led_num)` | 注册 SPI 回调 + 校验时钟 + 全灭；`led_num` 必须 ≤ `WS2812_LED_NUM` |
| `WS2812_Is_Clock_Ok()` | SPI6 时钟是否为 ≈2.5Mbit/s（false ⇒ CubeMX 分频没改）|
| `WS2812_Set_RGB(i, r, g, b)` | 设置第 i 颗（只改缓冲，**要 Show 才亮**）|
| `WS2812_Set_All(r, g, b)` | 全部设成同一颜色 |
| `WS2812_Clear()` | 全部熄灭（仍需 Show）|
| **`WS2812_Show()`** | **非阻塞 DMA 发送**（含尾部 reset 空白）；false = 正忙/未初始化 |
| `WS2812_Is_Busy()` | 是否正在发送 |
| `WS2812_Task()` | 跑一拍：发送超时保护（正常靠中断收尾）|
| `WS2812_Get_Buffer()` | 取原始缓冲（要发特殊数据时用）|

数据顺序：WS2812 要 **GRB**（不是 RGB），驱动内部已处理 ✓

## 5. 用法

```cpp
/* Robot_Init() */
ws2812_init_ok  = WS2812_Init(WS2812_LED_NUM);
ws2812_clock_ok = WS2812_Is_Clock_Ok();     // false ⇒ 去 CubeMX 把 SPI6 分频改成 32
WS2812_Clear();
WS2812_Show();

/* 任意时候改颜色 */
WS2812_Set_RGB(0, 40, 0, 0);    // 第 0 颗红
WS2812_Set_All(0, 0, 20);       // 全体暗蓝
WS2812_Show();                  // 发出去

/* Robot_Task() */
WS2812_Task();                  // 超时保护
```

## 6. 坑清单

1. **CubeMX 分频**：SPI6 Prescaler 必须 32（2.5Mbit/s）✗ 否则乱码；
2. **缓冲必须在 D3 SRAM**（见第 3 节）✗ 否则不亮；
3. **发完要有 reset 低电平**：≥50µs ⇒ 尾部补 `WS2812_RESET_BYTES = 24` 字节（2.5MHz 下 1 字节 3.2µs ⇒ ≈77µs）✓；
4. `led_num` 是**运行时**灯数，不能超过编译期的 `WS2812_LED_NUM`（缓冲按它分配）✗；
5. `WS2812_Show()` 发起前若已在发送会返回 false ⇒ 上层下次再试即可（灯带不需要背靠背刷）✓。

## 7. 上板自检

```
ws2812_init_ok  == true   （否则查 SPI6 是否配成 TX-only / PA7 是否 SPI6_MOSI）
ws2812_clock_ok == true   （否则把 CubeMX 里 SPI6 Prescaler 改成 32）
看到灯亮且颜色正确        （不亮 ⇒ 先看第 3 节的 D3 SRAM；颜色错乱 ⇒ 看分频）
```