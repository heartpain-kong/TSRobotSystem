# 外部 Flash(W25Q64)待办 —— 已定位的问题与下一步

> 现状：**`FLASH_ENABLE = 0`（关闭）**。板子功能正常，只是没有日志/上电链。
> 关闭开关在 `User/Bsp/flash/bsp_flash.h` 的 `FLASH_ENABLE`。

---

## 一、已确认的事实（有实测证据）

| 结论 | 证据 |
|---|---|
**芯片是好的、供电正常、单线通信正常** ✓ | 诊断探测（局部 buffer + 直调 `HAL_OSPI_Command`/`HAL_OSPI_Receive`）稳定读到 `raw=0xEF4017` ✓ |
**CubeMX 配置和官方例程完全一致** ✓ | `ClockPrescaler=3` / `SampleShifting=HALFCYCLE` / `FifoThreshold=8` / `DeviceSize=23` / `ClockMode=3` / `CSHighTime=1` / `MemoryType=MICRON` 等逐行相同；引脚 PA1/PA3/PB0/PB2/PD11/PE11 相同 |
**芯片应答正常、OSPI 初始化成功** ✓ | `hospi2.State/ErrorCode` 在探测时都是正常值，命令/接收都返回 `HAL_OK` |

## 二、已修掉的 3 个真 bug ✓

| # | 文件 | 原写法 | 修法 |
|---|---|---|---|
1 | `dev_w25q64.cpp` | `id = b[0] \| (b[1]<<8) \| (b[2]<<16)` ⇒ `0x1740EF` ✗ | 改成 **大端** `(b[0]<<16)\|(b[1]<<8)\|b[2]` ⇒ `0xEF4017` ✓（官方同）|
2 | 同上 | JEDEC 用 **DMA** 收 ✗（本板回调不来，永不完成）| 改成 **阻塞** `HAL_OSPI_Command`+`HAL_OSPI_Receive` ✓（官方同）|
3 | 同上 | 失败直接 `return false`，OSPI 停在"忙"⇒ 后续全 `INVALID_SEQUENCE` ✗ | 失败/超时都补 `HAL_OSPI_Abort()` ✓ |

## 三、还没解决的：打开 Flash 就会卡死/异常 ✗

现象（`FLASH_ENABLE=1` 时）：
* 一次：启动直接**无输出**，halt 后 PC 落在 `fault_decode.cpp`（进 HardFault）✗
* `CFSR = 0x00000400`（**IMPRECISERR**，不精确总线错误）✗
* 另一次：板子启动卡住、无遥测 ✗

疑点（按可能性）：
1. **DMA 通路**：本板 `HAL_OSPI_Command_Receive_DMA` / `Transmit_DMA` 都等不到回调 ⇒ 异步状态机（`W25Q64_Task`）永远等不到完成 ✗
2. **内存映射窗口**：诊断里出现过 `pc_psp=0x70408f1c`（`0x70000000` = OSPI 映射区）⇒ 有代码/异常帧指向"没做映射却去访问映射地址" ✗
3. 擦除/写入的**轮询**（`AutoPolling` 或自写 WIP 轮询）在阻塞改法下超时 ⇒ 卡在 `Log_Init` ✗

## 四、下一步（建议照这个顺序）

1. **直接移植官方驱动**（`例程\CtrBoard-H7_W25Q64\Devices\W25Q64\w25q64.c`，~700 行，**在这块板上验证过**）
   * 它的数据通路**全部是阻塞**：`HAL_OSPI_Command` + `Receive`/`Transmit` + `HAL_OSPI_AutoPolling`
   * 用它替换我们 `dev_w25q64.cpp` 的内部实现，**保持 `W25Q64_*` 对外 API 不变**（`Flash_Read/Write/Erase` 与 `dev_log` 都不用动）
2. 移植后**先不接日志**，用一个小测试跑三步：
   * 擦 1 个扇区 ⇒ 读回全 `0xFF`
   * 写 1 页（256 B 递增数据）⇒ 读回比对
   * 再擦 ⇒ 再读回全 `0xFF`
3. 三步都过，再把 `FLASH_ENABLE` 改回 **1**，观察 `JEDEC=0xEF4017` / `Flash=✓` / `日志=✓`
4. 最后验证上电链：连续上电 3 次，`boot` 应 1→2→3，`prev_start/prev_stop` 每次变成上一次的范围

## 五、调试时可以复用的东西

| 东西 | 位置 |
|---|---|
Flash 分层诊断（State/ErrorCode/Command/Receive/原始ID）| `bsp_flash.cpp` 的 `Flash_Diag_Probe()`，结果进遥测 `DEBUG_PL_FLASH` |
遥测里看这些 | 上位机「诊断① Flash/IMU」+「Flash 诊断」两行 |
关/开 Flash | `bsp_flash.h` 的 `FLASH_ENABLE` |
官方参考 | `例程\CtrBoard-H7_W25Q64\`（含 `.ioc` / `w25q64.c` / `main.c`）|

---

## 六、后续新增证据（第二轮）

| 现象 | 数据 |
|---|---|
打开 Flash 后 **HardFault**，`CFSR = 0x400`(IMPRECISERR，写) | `g_fault_code = 4`，`hfsr = 0x40000000` |
**坏地址就是 0** | `g_fault_bfar = 0x00000000` ⇒ 一次**写地址 0** |
自测连第 1 步都没到 | `flash_diag_state = 0`（`state=1` 在 `W25Q64_Init` 返回后才设）⇒ 崩在 `W25Q64_Init` 内 |
**探测函数（直调 HAL + 局部 buffer）能读到 `0xEF4017`** | 说明 `hospi2`、命令、阻塞收发**都没问题** |
⇒ 崩在 **`BSP_OSPI_*` 封装路径**或 `W25Q64_Init` 的某一步 | 已排除：`rx_buffer` 是结构体内合法数组；`BSP_OSPI_Get_Object()` 按 `hospi->Instance` 判断正确 |

**下一步（建议）**：在 `W25Q64_Init` 里**逐步**把结果写进 `flash_diag_*`（每 2~3 行一个编号），一次就能定位到具体语句；
或**直接绕开 `BSP_OSPI_*`**：让 `W25Q64_*` 全部用「局部 buffer + 直调 HAL」的写法（探测已证明这条路径可用 ✓），彻底不用驱动缓冲与回调。

> 当前：`FLASH_ENABLE = 0`、`FLASH_SELFTEST = 0` ⇒ 板子全功能正常（除日志）。
## 七、第三轮关键发现（最后一条，最有价值）

在 `Flash_Self_Test()` **一开始**（`W25Q64_Init` 之前）就把指针和 `hospi2.State` 记进 RAM，halt 后读出：

| 变量 | 值 | 判读 |
|---|---|---|
`BSP_OSPI_Get_Rx_Buffer(&hospi2)` | `0x2400F458` | **有效** ✓（不是空指针 ✗）|
`BSP_OSPI_Get_Tx_Buffer(&hospi2)` | `0x2400F258` | 有效 ✓ |
`hospi2.Instance` | `0x5200A000` | = OCTOSPI2，正确 ✓ |
**`hospi2.State`（自测开始前）** | **2 = BUSY_INDIRECT_WRITE** ✗ | ★ **驱动还没跑，外设就已经"忙"了** |

⇒ 结论：**不是空指针、不是配置** ✗。是**有别的代码在 `W25Q64_Init` 之前动过 OSPI，并把它留在 BUSY_INDIRECT_WRITE** ✗，
之后 `HAL_OSPI_Command` 在忙状态下操作 ⇒ IMPRECISERR/异常 ✗。

**下一步（唯一要查的）**：找出"谁先把 OSPI 弄忙的"：
1. 检查 `main.c` 里 `MX_OCTOSPI2_Init()` 之后、`Robot_Init()` 之前有没有别的 OSPI 操作
2. 检查 `HAL_OSPI_Init()` 之后 `hospi2.State` 是不是 `READY`（在 `Robot_Init` 第一行就读一次记下来）
3. 若发现是 `HAL_OSPI_Init` 自己留的状态 ⇒ 在 `Flash_Init` 开头加一次 `HAL_OSPI_Abort(&hospi2)` 把状态拉回 READY ✓

> 当前：`FLASH_ENABLE = 0`、`FLASH_SELFTEST = 0` ⇒ 板子全功能正常（仅日志不可用）。
## 八、第四轮：埋点定位结果（最精确）

在 `W25Q64_Init` 里插了 11 个检查点（写 `flash_diag_cmd`），自动循环跑完后 halt 读 RAM：

| 变量 | 值 | 判读 |
|---|---|---|
`flash_diag_cmd` | **61** | 51 号之前都过了；**崩在 `HAL_OSPI_Receive()` 这一行** |
`flash_diag_rcv` | **1** | ★ **`HAL_OSPI_Command()` 返回 `HAL_ERROR`** |
`flash_diag_err` | **4** | ★ 命令之后 `hospi2.State = BUSY_INDIRECT_READ` |
`flash_diag_state` | 1 | 自测还在"读 ID"阶段 |

### 结论（已闭环）

`HAL_OSPI_Command` 返回 `HAL_ERROR` 的**唯一原因**是进入时 `hospi2.State != READY`
（HAL 里就是 `if (State != READY) { ErrorCode = INVALID_SEQUENCE; return HAL_ERROR; }`）。

再结合第三轮的证据（`Flash_Self_Test` 第一步记录时就发现 `State = 2 BUSY_INDIRECT_WRITE`）：

> ★★ **OSPI 在 Flash 驱动执行之前就已经是"忙"状态了** ——
> 不是 W25Q64 驱动的问题、不是配置、不是芯片、不是指针。
> 是**启动早期有别的代码动过 OSPI 并把它留在了忙态**。

### 下一步（唯一要做的）

在 `Robot_Init()` 的**第一行**（任何 OSPI 相关操作之前）读一次 `hospi2.State` 并记进 `flash_diag_err`：

* 若那时**已经是忙** ⇒ 问题在 `main()` 里 `MX_OCTOSPI2_Init()` 之后、`Robot_Init()` 之前的那段初始化，
  逐个 `MX_*_Init()` 排查谁碰了 OSPI（**重点看有没有别的模块也用了 OSPI / 调了 `HAL_OSPI_*`**）
* 若那时是 `READY` ⇒ 说明是 `Robot_Init` 里 Flash 之前那段（GPIO/ADC/IMU…）弄忙的，同样逐个排查

> 当前：`FLASH_ENABLE = 0`、`FLASH_SELFTEST = 0` ⇒ 板子全功能正常（仅日志不可用）。
> 调试顺序：`FLASH_SELFTEST=1` 只看自测（不动日志）⇒ 全绿后再 `FLASH_ENABLE=1`。
## 九、第五轮：四段采样（把范围压到 HAL_OSPI_Init 里面）

在四个位置采样 `hospi2.State` 并写进 RAM，烧录后 halt 读：

| 采样点 | 变量 | 值 | 判读 |
|---|---|---|---|
`MX_OCTOSPI2_Init` 里、`HAL_OSPI_Init` **之前** | `flash_diag_cmd` | **0** | RESET（正常 ✓）|
`HAL_OSPI_Init` + `HAL_OSPIM_Config` **之后** | `flash_diag_rcv` | **2** | ★ 已经是忙 ✗ |
`MX_OCTOSPI2_Init` 之后（main.c） | `flash_diag_raw` | **2** | 忙 ✗ |
`Robot_Init` 入口 | `flash_diag_err` | **2** | 忙 ✗ |

⇒ 状态是在 **`HAL_OSPI_Init()` 内部**变成 2 的。

### HAL 源码里的关键逻辑（stm32h7xx_hal_ospi.c）

```
384: if (hospi->State == HAL_OSPI_STATE_RESET) {
405:   MspInitCallback(hospi);                  // 时钟/引脚
437:   status = OSPI_WaitFlagStateUntilTimeout(hospi, HAL_OSPI_FLAG_BUSY, RESET, ...);
439:   if (status == HAL_OK) {
468:     hospi->State = HAL_OSPI_STATE_READY;   // ← 只有这行才置 READY
470:   }
475:   return status;
```

* 若 437 行**超时** ⇒ `status != HAL_OK` ⇒ **468 不执行** ⇒ `State` 停在忙态 ✗，并且 `MX_OCTOSPI2_Init` 里的
  `if (HAL_OSPI_Init(...) != HAL_OK) Error_Handler();` **应该被触发** ✗
* 但板子**能正常启动** ⇒ 说明 `Error_Handler()` **没被调用** ⇒ `status` 是 `HAL_OK` ⇒ 与读到的 `2` 矛盾 ✗

### 结论 / 下一步（两个方向二选一）

1. **验证 `HAL_OSPI_Init` 的返回值**：在 `MX_OCTOSPI2_Init` 里把 `HAL_OSPI_Init(&hospi2)` 的返回值记进
   `flash_diag_state`（`0=OK 1=ERROR 2=BUSY 3=TIMEOUT`）。若**不是 0** ⇒ 就是 437 行超时，
   接着查：`HAL_OSPI_MspInit` 的 OSPI2 时钟/引脚是否真的生效、`BUSY` 标志为什么不清
2. 若返回值确实是 `0`（HAL_OK）⇒ 那 `State=2` 的读取可能有别的原因（符号/优化/LTO），
   改成在 `MX_OCTOSPI2_Init` 里**直接读 `hospi2.Instance->SR`** 和 `hospi2.State` 一起记下来对比。

> 当前：`FLASH_ENABLE = 0`、`FLASH_SELFTEST = 0` ⇒ 板子全功能正常（仅日志不可用）。
### 第五轮补充：照官方加了「清零句柄 + HAL_OSPI_DeInit」也没用 ✗

```c
/* 抄官方 CtrBoard-H7_W25Q64 的 OCTOSPI2_Init 1 */
memset(&hospi2, 0, sizeof(hospi2));
hospi2.Instance = OCTOSPI2;
(void)HAL_OSPI_DeInit(&hospi2);
```

⇒ 加完再烧：`FLASH_SELFTEST=1` 仍然 **0 帧**（启动卡死）✗
⇒ 结论：不是"句柄不干净"，也不是句柄没 DeInit。

### 最干净的下一步：把官方例程**当独立固件**烧到这块板上跑

1. 用 `例程\CtrBoard-H7_W25Q64` 的 MDK 工程（或把它移植成最小 CMake 工程）编译出 `.elf/.hex`
2. 直接烧到板上，看串口有没有打印 `W25Q64 OK,flash ID:EF4017` / `erase succeed` / `write succeed`
3. **它通** ⇒ 拿它的 `main.c` 初始化序列、`octospi.c`、`w25q64.c` 和我们逐文件 diff
   **它也不通** ⇒ 说明不是代码，而是**板子/接线/供电**（这颗 W25Q64 可能就是不通的）

> 当前：`FLASH_ENABLE = 0`、`FLASH_SELFTEST = 0` ⇒ 板子全功能正常（仅日志不可用）。
### 第六轮：初始化顺序也不是原因 ✗

把 `main.c` 里的 `MX_OCTOSPI2_Init()` 从第 19 个**挪到第 2 个**（紧跟 `MX_GPIO_Init`，与官方例程一致），
`FLASH_SELFTEST=1` 烧录后仍然 **0 帧** ✗ ⇒ 顺序无关，已还原。

### CubeMX 里需要改的东西（结论）

| 项 | 是否需要改 | 说明 |
|---|---|---|
`OCTOSPI2.DeviceType` | **建议补成 `SPI_DEVICE_FLASH`**（可选）| 官方有这项，我们 `.ioc` 没有；只影响生成参数，功能上暂未发现差异 |
`MX_OCTOSPI2_Init` 的调用顺序 | **不需要** ✗ | 已实测：挪到第 2 个也无效 |
引脚 PA1/PA3/PB0/PB2/PD11/PE11 | **不需要** ✓ | 与官方完全一致 |
`ClockPrescaler` / `SampleShifting` / `FifoThreshold` / `DeviceSize` / `ClockMode` / `CSHighTime` / `MemoryType` | **不需要** ✓ | 与官方逐项相同 |
`QSPIFreq_Value` | 可选 | 官方是 240 MHz（40→60 MHz），我们是 160 MHz（40 MHz）；两者都远低于 W25Q64 上限，不是原因 |

⇒ **CubeMX 侧没有必须修改项** ✗。真正的下一步是「官方例程独立固件」对比，见下。
### 第七轮：原始寄存器/结构体快照（供下次直接用）

```
hospi2 @ 0x24000F1C, 前 16 个字:
  +0x00 00000000 -> 5200A000   Instance = OCTOSPI2 ✓
  +0x04 00000008   FifoThreshold = 8 ✓
  +0x08 00000000   DualQuad = 0 (disable) ✓
  +0x0C 00000000   MemoryType = 0 = MICRON ✓
  +0x10 00000017   DeviceSize  = 23 ✓
  +0x14 00000001   ChipSelectHighTime = 1 ✓
  +0x18 00000000
  +0x1C 00000001
  +0x20 00000000
  +0x24 00000003   ClockPrescaler = 3 ✓
  ...(其余见 bsp_flash 的采样)

OSPI2 寄存器 @ 0x5200A000(注意:本工程 dump 出来是):
  0x5200A000: 00000701    (CR: EN=1, FTHRES=7 即 8)
  0x5200A004: 00000000
  0x5200A008: 00160009
  0x5200A00C: 00000002
  0x5200A010/14/18: 0
```

**关键矛盾（仍未解）**：
`hospi2.Init.MemoryType = 0 (MICRON)` ✓ ⇒ `HAL_OSPI_Init()` 里
`if (MemoryType == HYPERBUS) State = HYPERBUS_INIT; else State = READY;`
应该走 **else ⇒ State = READY(1)** ✓，但我们读到的 `hospi2.State` 是 **2** ✗。

⇒ 下次**第一步**就做这件事（5 分钟）：
在 `MX_OCTOSPI2_Init()` 里紧跟 `HAL_OSPI_Init(&hospi2)` **之后**读一次 `hospi2.State` 存到 RAM：
* 若是 **1** ⇒ 说明 `State` 是在**后面**被改的 ⇒ 继续往后二分（`HAL_OSPIM_Config` 之后、`Robot_Init` 各步）
* 若是 **2** ⇒ 说明 `HAL_OSPI_Init` 内部就是 2 ⇒ 打印它的**返回值**（`HAL_OK/ERROR/BUSY/TIMEOUT`）
  + 在 `MX_OCTOSPI2_Init()` 里 `printf` 一样记进 RAM ⇒ 就能判定是不是 437 行 `BUSY` 等待超时

> 板子当前：`FLASH_ENABLE=0`、`FLASH_SELFTEST=0` ⇒ 全功能正常（仅日志不可用）✓
## 十、第八轮：**重大突破** —— ID 读通了 ✓（还剩擦除路径）

### 这一轮又修掉的 3 个真 bug ✓

| # | 文件 | 问题 | 修法 |
|---|---|---|---|
1 | `dev_w25q64.cpp` | `W25Q64_Init` 开头做 `0x66`+`0x99` 软复位 ✗ | **删掉**（官方 `Init` 只做 ReadID，不做复位）；实测加了复位读不到 ID ✓ |
2 | `dev_w25q64.h` | `#define W25Q64_JEDEC_ID 0x001740EFu` ✗（字节序反了）| 改成 **`0x00EF4017u`** ✓ ⇒ ID 校验终于通过 ✓ |
3 | `dev_w25q64.cpp` | 手写「读状态寄存器轮询」把外设留在 `BUSY_INDIRECT_READ` ✗ | 改用官方写法 **`HAL_OSPI_AutoPolling`** ✓ |

### 现在的实测结果

```
flash_cmd  = 11     ← 检查点 11 = W25Q64_Init 全部走完 ✓✓
flash_raw  = 0xEF4017 ✓   ← 芯片 ID 读到了 ✓
flash_state = 2     ← 自测进到"擦除"阶段 ✗
flash_cmd  = 22     ← 停在 Blk_WEL 里"读状态寄存器命令失败"
flash_rcv  = 4      ← 发命令前 State = BUSY_INDIRECT_READ ✗
flash_err  = 2      ← HAL_OSPI_Command 返回 HAL_BUSY
flash_raw  = 16     ← ErrorCode = 0x10 INVALID_SEQUENCE
```

### 结论 / 下一步（已经很近了）

`0x06` 写使能发出去之后，外设被留在 **`BUSY_INDIRECT_READ`** ✗ ⇒ 下一条命令必然 `HAL_BUSY` ✗。

**下一次只需要试这一件事**：`0x06` 之后**不要立刻发别的命令**，改为
```c
/* 0x06 是纯命令(无数据),发完立刻把状态确认成 READY;或干脆不做 0x06 状态回读,
   直接信任 WEL,然后发 0x20 擦除,再用 AutoPolling 等 WIP */
```
两种写法二选一：
1. **去掉 `Blk_WEL` 里的状态回读**（只发 `0x06` 就返回），直接发 `0x20`
2. 或 `0x06` 之后调 `HAL_OSPI_Abort(&hospi2)` 强制拉回 READY 再发下一条

> 当前：`FLASH_ENABLE=0`、`FLASH_SELFTEST=1`（自测失败但**不会再卡死板子** ✓，遥测正常 ✓）
## 十一、第九轮：擦除命令能发了，卡在 WIP 等待

修掉 `Blk_WEL` 的状态回读（那是让外设变忙的元凶 ✗），改成「只发 `0x06` + Abort 拉回 READY」后：

```
flash_cmd  = 28   ← 0x20 擦除命令**已经发出**(27)✓，卡在 Wait_WIP(28)
flash_rcv  = 2    ← Abort 之后 State 仍是 2 ✗（HAL_OSPI_Abort 不能把它救回 READY）
flash_raw  = 0xEF4017 ✓
```

### 已经全线打通的（重要 ✓）

| 步骤 | 状态 |
|---|---|
读 JEDEC ID | **✓ 通**（`0xEF4017` 稳定读到）|
`0x06` 写使能 | **✓ 发出** |
`0x20` 扇区擦除命令 | **✓ 发出** |
WIP 等待 / 状态轮询 | ✗ 不通（`HAL_OSPI_AutoPolling` 也失败）|
`HAL_OSPI_Abort` | ✗ 不能把 State 拉回 READY（实测仍是 2）|

### 下一步建议（按可能性排序）

1. **检查 `HAL_OSPI_Abort` 的实现**：它内部 `if (State == READY/BUSY_INDIRECT_READ ...)` 的分支，
   看为什么 Abort 后 State 还是 2（可能我们的 HAL 版本对 `BUSY_INDIRECT_READ` 不走复位路径）
2. **对比官方的 `OSPI_W25Qxx_WriteEnable()`**（`例程\...\w25q64.c` 第 189 行起）：
   看它在 `0x06` 之后到底做了什么（我们猜"等 TC"，但 `AutoPolling` 也失败，说明不是普通等待）
3. **试 `HAL_OSPI_Command` 的超时**：我们给的是 100 ms，官方给的是 `HAL_OSPI_TIMEOUT_DEFAULT_VALUE`(65535) ✗
   —— 也许 `0x06`/`0x20` 这类命令需要更长等待，之前 `0xFFFF` 卡 2 分钟就是因为它真的是"在等"
4. 若上面都不行：把官方 `w25q64.c` 的 `OSPI_W25Qxx_WriteEnable/SectorErase/AutoPollingMemReady`
   **三个函数整体拷过来**（它们的等待方式和我们的不同）

> 当前：`FLASH_ENABLE=0`、`FLASH_SELFTEST=1`（自测失败但板子正常 ✓，遥测 ✓）
## 十二、✅ **已修复！Flash 全部打通**

### 最终根因（一句话）

**无数据命令（`0x06` 写使能 / `0x20` 擦除 / `0x66` `0x99` 复位）必须 `DataMode = HAL_OSPI_DATA_NONE`。**
我们用了 `COMMAND_DEFAULT`（`DATA_1_LINE`）✗ ⇒ HAL 以为还要传数据 ⇒ 状态卡在 `BUSY_INDIRECT_WRITE`
⇒ 下一条命令必 `HAL_BUSY` / `INVALID_SEQUENCE`。官方 `OSPI_W25Qxx_WriteEnable()` 里就是 `DATA_NONE` ✓。

### 本轮还修掉的两个连带 bug

| 问题 | 后果 | 修法 |
|---|---|---|
`W25Q64_Read` 改阻塞后**没置 `rx_done_flag`** ✗ | `Flash_Read()` 永远等不到 ⇒ 日志扫描成功却返回失败 ⇒ `0x0006` | 读成功后置 `rx_done_flag = true;` ✓ |
`Log_Init` 里同样的等待链 | — | 同上，一并解决 ✓ |

### 实测证据

```
Flash 自测:  selftest = 4 ✓   （擦→写→读回比对 全过；readback = 0x04030201 ✓）
JEDEC     :  0xEF4017 ✓
日志      :  log_ok = 1 ✓   err_code 从 0x0006 变回 0x0014（只剩没接电机）
上电链    :  log_boot=1 / log_cur=8192 / prev=(无) / wrap=0 ✓
```

### 唯一未完项：**上电链的"跨上电记忆"**（`log_boot` 每次复位都回到 1）

现象：连续复位 3 次，`log_boot` 恒为 **1**，`prev=(无)` ⇒ `Log_Init` 每次都走"全新"分支 ✗
⇒ 检查点**写了但没被扫到**。下一步只查这一处：

1. `Log_Write_Checkpoint()` 之后立刻 `Flash_Read(LOG_HEADER_ADDR, slot, 32)` 回读，确认
   `magic == 0x4C4F4732` && CRC 对 —— 若回读为空 ⇒ 写没落盘（`Flash_Write` 的 `W25Q64_Is_Busy()` 等待链）
2. 若回读 OK 但下次扫描找不到 ⇒ 查 `Log_Init` 扫描里的 `offset % LOG_RECORD_SIZE` 过滤
   （新写 `s_boot_start`/`s_prev_*` 字段后，`offset` 语义有没有被写错成非 256 对齐）

> 现状：`FLASH_ENABLE=1`、`FLASH_SELFTEST=0` ⇒ **Flash / 日志都正常工作**，只有"上电编号递增"这一个小功能待收尾。