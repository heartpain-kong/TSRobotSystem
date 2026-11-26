# bsp_adc — ADC 底层（ADC1 循环采样 / ADC3 内部温度）

文件：`bsp_adc.h` / `bsp_adc.cpp`。写法仿 `User/Bsp/fdcan/bsp_fdcan.*`（管理对象 + Init + 取值），
算法极简部分参考 `E:\rm\dm02_test\...\1_Middleware\Driver\ADC\drv_adc.*`、
`E:\rm\cod-h7-template-main\BSP\Src\bsp_adc.c`。

## 1. 我们 CubeMX 里的 ADC 配置（已核实）

| | ADC1 | ADC3 |
|---|---|---|
| 分辨率 | **16 位** ✓ | 12 位 |
| 规则通道 | **2 个 rank，都是 `ADC_CHANNEL_4`** ⇒ 引脚 **PC4（ADC1_INP4）被采两次/周期** | 1 个 = **内部温度传感器** |
| 转换模式 | `ScanConvMode = ENABLE` + **连续转换** | 单次（按需触发）|
| 数据搬运 | **DMA2_Stream6，CIRCULAR，两侧都是 HALFWORD** ✓ + `ADC_CONVERSIONDATA_DMA_CIRCULAR` | 挂 **BDMA_Channel0** —— **本驱动不用它** ✗（见 §4）|
| 校准 | `HAL_ADCEx_Calibration_Start(…, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)` ✓ 必须先做 | 同 |

## 2. API

| 接口 | 说明 |
|---|---|
| `BSP_ADC_Init(&hadc1, 2)` | **校准 + `HAL_ADC_Start_DMA`（循环）**；参数 = 一周期采样点数（= 规则通道数）|
| `BSP_ADC_Is_Inited(&hadc1)` | 是否已启动 |
| `BSP_ADC_Get_Raw(&hadc1, i)` | 第 i 个采样点原始值（DMA 一直在刷新，直接读即可 ✓）|
| `BSP_ADC_Get_Mv(&hadc1, i)` | 换算成 mV（按该 ADC 分辨率：16 位 → 65535、12 位 → 4095）|
| `BSP_ADC_Get_Mv_Average(&hadc1, n)` | 前 n 个点平均 mV —— 我们两个 rank 同一路，平均即降噪 ✓ |
| `BSP_ADC_Bind_ADC3(&hadc3)` | 绑定 ADC3（温度用），**只记句柄、不启动 DMA** |
| `BSP_ADC_Get_Temperature()` | MCU 温度（℃），ADC3 按需**阻塞单次转换** + H7 出厂两点校准 |

## 3. 用法（已接进 `Robot_Init()` / `Robot_Task()`）

```cpp
/* Robot_Init() */
adc1_init_ok = BSP_ADC_Init(&hadc1, 2);
BSP_ADC_Bind_ADC3(&hadc3);

/* Robot_Task()(500 Hz):每拍刷新调试量 */
adc1_mv    = BSP_ADC_Get_Mv_Average(&hadc1, 2);
mcu_temp_c = BSP_ADC_Get_Temperature();
```

## 4. ⚠️ 两个坑（都已在驱动里规避）

1. **ADC3 不能走 DMA** ✗：它在 H7 上挂的是 **BDMA**，而 BDMA **只能访问 D3 SRAM（0x38000000）**，
   我们的缓冲在 **RAM_D1（0x24000000）** ⇒ BDMA 写不进去 ✗（症状：温度读到 0/常数 ✗）。
   ⇒ 本驱动对 ADC3 用**按需阻塞单次转换** ✓（温度不需要高速）。
   若将来确实要 DMA，必须把缓冲放进 D3 SRAM 的专用链接段 —— 那要动 `.ld`，不要随手加 ✓。
2. **必须交换算** ✗：H7 的 ADC 不做校准的话精度很差，`BSP_ADC_Init` 里第一步就是
   `HAL_ADCEx_Calibration_Start` ✓（参考工程 `drv_adc.cpp` 同样如此 ✓）。

另：参考电压写死 `BSP_ADC_VREF_MV = 3300.0f`（VDDA = 3.3V）。要更准可以读 **VREFINT**
（`.ioc` 里已经有 `VP_ADC3_Vref_Input` ✓）来反推实际 VDDA —— 需要的话告诉我，加十来行即可 ✓。

## 5. 相关

* `User/Bsp/flash/bsp_flash.md` —— 片内 FLASH 读写（另一个空目录补齐的模块）
* `User/Bsp/fdcan/bsp_fdcan.md` —— 本模块模仿的写法模板
* `User/App/robot/robot.md` —— 接线位置
