# 每次 CubeMX 导出必做事情

> ⚠️ 本文里所有路径都相对 **工程根目录**(`E:\rm\开源\RM\TSRobotSystem`)✓；下面的命令请在**工程根目录**执行 ✓

> **背景**：CubeMX 重新生成代码时，会**覆盖**不少东西（`FreeRTOSConfig.h`、移植层、`CMakeLists.txt`、
> 链接脚本、`Core/` 下的 `.c`、以及所有不在 `USER CODE` 段里的内容）。
> 这个文档就是一份"导出后清单" ✓，配套脚本 **`tools/cubemx_post_export.ps1`** ✓。

---

## 0. 一句话：导出后**先跑脚本**

```powershell
powershell -ExecutionPolicy Bypass -File tools\cubemx_post_export.ps1
```
脚本是**幂等**的 ✓（跑几次结果一样 ✓），最后会自动编译一次（失败会**自动重试一次**，因为 MinGW 偶尔会报
`opening dependency file ... No such file` 这类假错误 ✓）。
**看到最后一行是 `build 成功` + `自检通过` 才算完** ✓。

> ⚠️ 脚本自己也是要被保留的工程文件 ✓ —— 改它之前先备份；它里面所有写文件的地方都**保留原 BOM 状态** ✓
> （`.ps1` 丢 BOM 会导致 PowerShell 按 ANSI 读、中文全乱、直接解析失败 ✗）。

---

## 1. 脚本**自动**帮你做的事

| # | 项目 | 为什么必须做 |
|---|---|---|
| 1 | `FreeRTOSConfig.h`：`configTOTAL_HEAP_SIZE = 32768` | CubeMX 默认 15360 ⇒ 任务建不出来（`osThreadNew` 只返回 NULL，**静默失败** ✗）|
| 2 | 两个错误钩子开启 | `configUSE_MALLOC_FAILED_HOOK=1`、`configCHECK_FOR_STACK_OVERFLOW=2` |
| 3 | `configASSERT` = 关中断死循环 | 否则 assert 失败被静默忽略 ✗ |
| 4 | FreeRTOS 移植层 = **M7 专用 `ARM_CM7/r0p1`**，且树里只有一个 `port.c` | 用错通用 ARM_CM3 版会随机挂 |
| 5 | `CMakeLists.txt` / `CMakeLists_template.txt` 重写 | CubeMX 会把你手写的 CMake 覆盖掉 |
| 6 | 两个 CMake 的 **User include 列表**：**按文件系统自动补全 + 去重**（含头文件的目录）| 少了目录 ⇒ C 侧找不到头 ⇒ 编译不过；重复行也会被清掉 |
| 7 | `octospi.c`：`hospi2.Init.ClockPrescaler = 3`、`FifoThreshold = 8` | 对齐同板可跑工程（160MHz/4 = **40MHz** SCK）|
| 8 | `it.c` 里有 `HAL_OSPI_IRQHandler(&hospi2)` | 少了它 ⇒ OSPI 自动轮询收不到中断 ⇒ Flash 操作卡死 |
| 9 | **`.ld` 里的 `.RAM_D3` 段**：内存区域与输出段都会**缺失则补、重复则去重** | `ws2812_buffer` 必须落在 D3 SRAM（0x38000000）；**BDMA 只能访问 D3** ✗ |
| 10 | `PC15` 初始化必须是 **RESET（5V 关）** | 上电就把 5V 打开有风险 |
| 11 | 关键 `USER CODE` 段确认（`main.c` 调 `Robot_Init()`、`freertos.c` 调 `OSTaskInit()` 等）| 导出会把 `USER CODE` **外**的东西清掉 ⇒ 确认它们还在 ✓ |
| 12 | 两个钩子**有且只有一份定义** | 重定义会编译不过；少了 hook 会静默失败 |
| 13 | 编译验证 + 假错误自动重试 | —— |

---

## 2. 必须在 **CubeMX GUI** 里人工确认的（脚本管不到 ✗）

> 这些改完**要重新导出一次**，然后回到第 0 步跑脚本 ✓。

| 项 | 应该是 | 不对的症状 |
|---|---|---|
| **UART5** | 100000 baud / 9bit(含偶校验) / 2 stop / **RX DMA = Circular** | 遥控器不掉线但杆量全 0、或只收到一帧 |
| **USART1** | 115200 / 8N1 / RX 缓冲足够（BSP 里按 136 B 开）| 裁判系统 `Reject_Count` 一直涨 |
| **SPI6 分频** | 让 SCK ≈ **2.5 Mbit/s**（时钟 32 分频）| WS2812 全不亮或颜色乱（`ws2812_clock_ok == false`）|
| **SPI2** | 给 BMI088，DMA 或阻塞按 BSP 需要 | IMU `Is_Online() == false` |
| **OCTOSPI2** | `SampleShifting = Half Cycle`、MemoryType/DeviceSize 按 W25Q64 | Flash 读回全 0xFF / JEDEC ID 不对 |
| **PC15** | `GPIO_Output`，初值 **Low**，标签 `POWER_5V` | 上电 5V 就通电 |
| **新增输出通道** | 标签写成 `RELAY_1` / `RELAY_2` / `RELAY_3` | `dev_electric` 找不到通道（代码**不用改**，靠标签自动接入 ✓）|
| **FreeRTOS** | CMSIS-RTOS2、tick 1 kHz、`TICK_RATE_HZ=1000` | 周期全乱 |
| **FreeRTOS 参数**<br>（Middleware → FREERTOS → **Config parameters**）| `TOTAL_HEAP_SIZE=32768`、`CHECK_FOR_STACK_OVERFLOW=Option2`、`USE_MALLOC_FAILED_HOOK=Enabled`、`MINIMAL_STACK_SIZE=128`、`ENABLE_FPU=Enabled`、`USE_NEWLIB_REENTRANT=Enabled`、`MAX_TASK_NAME_LEN=32`、`RECORD_STACK_HIGH_ADDRESS=Enabled`、**`USE_TRACE_FACILITY=Enabled`**、**`QUEUE_REGISTRY_SIZE=8`** | 任务建不出来（堆小）/ CLion 的 RTOS 视图里任务名空、没有 Objects 标签 |
| **中断优先级** | FreeRTOS 系统调用允许的优先级范围内（5~15）| `configASSERT` 里卡死 |
| **时钟树** | SYSCLK 640 / HCLK 160 / APB 按现设计 | ⚠️ **一变就要同步改**：`DWT_Init()` 的 CPU 频率、`Timestamp_Init()` 的 TIM5 预分频 |

---

## 3. 导出后**人工**还要做的（少量）

1. **看 diff**：`git status` / 对比 `Core/` 下被改动的地方（尤其是 `main.c` 的 USER CODE 段与 `stm32h7xx_it.c`）✓
2. **新加的模块**：**不用改脚本** ✓ —— 脚本第 5 节现在**按文件系统自动算** include 列表（含头文件的目录会自动补进两个 CMakeLists 并去重 ✓）。你只要照 `User/代码架构框架.md` 第 5 节把 `Robot_Init` / `OSTaskInit` / `msg_center` 话题接好 ✓
   —— 顺便照 `User/代码架构框架.md` 第 5 节把 CMake include / `Robot_Init` / `OSTaskInit` 接好 ✓
3. **按上板自检过一遍**：每个模块的 `.md` 最后一节 ✓（导出后最容易出问题的是外设参数类的东西 ✓）

---

## 4. "导出后翻车"速查表

| 现象 | 最可能原因 | 怎么修 |
|---|---|---|
| 某个任务完全不跑，其他正常 | 任务栈/RTOS 堆不够（`osThreadNew` 静默返回 NULL）| 看 `g_thread_create_failed`；加大 `configTOTAL_HEAP_SIZE` 或减栈 |
| WS2812 灯不亮 | ① 缓冲落回 RAM_D1（BDMA 读不到）② SPI6 分频不对 | 跑脚本（它会保证 `.RAM_D3`）✓；查 `ws2812_clock_ok` |
| 外部 Flash 读写全失败 | OSPI 参数/中断被导出改回 | 跑脚本 ✓；看 `flash_jedec_id` |
| 遥控器在线但不动 | UART5 参数或 RX DMA 模式被改 | 见第 2 节表格 |
| 编译报找不到某个用户头 | include 列表被覆盖 | 跑脚本 ✓（会自动补）|
| 编译报 `port.c` 重复/找不到 | 移植层被换 | 跑脚本 ✓ |
| 上电 5V 就通电 | PC15 初值被改成 High | `gpio.c` 改回 Low（脚本会检查并提示 ✓）|
| 时间戳/超时全乱 | 时钟树变了但 `DWT_Init`/TIM5 预分频没跟着改 | 见第 2 节最后一行 ⚠️ |

---

## 5. 标准操作顺序（背下来）

```
① CubeMX 里改配置(引脚/外设/时钟/FreeRTOS)
② Generate Code
③ 跑:tools\cubemx_post_export.ps1        ← 必须看到 build 成功 + 自检通过
④ 看 diff,确认 Core/ 里的改动符合预期
⑤ 新模块的话:补脚本清单 + 接线(CMake/Robot_Init/OSTaskInit)
⑥ 上板,按各模块 .md 的「上板自检」过一遍
```
> **CLion RTOS 视图专用（★ 别移出 USER CODE）**
> `configGENERATE_RUN_TIME_STATS` 与 `portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()`、`portGET_RUN_TIME_COUNTER_VALUE()`
> 这三样 **CubeMX 生成不出来** ✗（它只管 `GENERATE_RUN_TIME_STATS` 这个开关本身，port 宏得自己写）⇒
> 已写在 `Core/Inc/FreeRTOSConfig.h` 的 **`USER CODE BEGIN Defines` … `USER CODE END Defines`** 之间 ✓，
> 用 TIM5 的微秒计数当运行时间源 ✓。这段**导出后会保留** ✓；把它挪到生成区就会被 CubeMX 清掉 ✗。