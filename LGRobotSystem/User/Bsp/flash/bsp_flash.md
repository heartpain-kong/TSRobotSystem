# bsp_flash — 外部 8 MB QSPI Flash(W25Q64)存储层

文件：`bsp_flash.h` / `bsp_flash.cpp`。
**手册依据**：《达妙科技 DM-MC-Board02 电机开发板使用说明书 V1.0》**P12** 原话 ——
> "为方便用户存储数据，开发板上集成了一颗 **8Mbytes 的 QSPI Flash** 芯片，用户可以存储
> 调试数据或者 LCD 显示图片等。"

⇒ 这里说的是**板上那颗外部 QSPI Flash**（W25Q64）✓ —— **不是** MCU 芯片内部的 FLASH ✗。

## 1. 分层（本模块在哪一层）

```
User/Bsp/ospi/bsp_ospi.*         OCTOSPI2 总线驱动(命令 / DMA / 自动轮询)
        ↓
User/Device/w25q64/dev_w25q64.*  W25Q64 芯片驱动(读/页写/4K·32K·64K 擦/整片擦)
        ↓  非阻塞状态机:发起 → 靠 W25Q64_Task() 推进 → Is_Busy()/Is_Rx_Done() 收尾
User/Bsp/flash/bsp_flash.*   ★本层:包成"调用即完成"的阻塞接口 + 自动分段 + 分区约定
```

**本层解决的三个麻烦** ✓：
| 底层限制 | 本层怎么处理 |
|---|---|
| 一次读最多 512 B（SPI/OSPI 缓冲上限）| `Flash_Read()` **自动分段** ✓ |
| 一次写最多 256 B 且**不能跨页** | `Flash_Write()` **自动按页切分** ✓ |
| 擦除最小单位 4 KB | `Flash_Erase()` **自动按扇区对齐并逐个擦** ✓ |

## 2. API

| 接口 | 说明 |
|---|---|
| `Flash_Init()` | 注册 OSPI 回调 + `W25Q64_Init()`（读 JEDEC ID 自检 `EF 40 17`）|
| `Flash_Is_Ready()` | 是否可用 |
| `Flash_Get_Jedec_Id()` | 读回 ID（应为 **`0xEF4017`**）|
| `Flash_Get_Capacity()` | 8 MB |
| `Flash_Read(addr, buf, len)` | 读（自动分段，**阻塞**）|
| `Flash_Write(addr, buf, len)` | 写（自动按 256B 页，**阻塞**；**目标必须先擦**）|
| `Flash_Erase(addr, len)` | 擦（自动按 4KB 扇区，**阻塞**；4KB 约 45ms）|
| `Flash_Is_Blank(addr, len)` | 是否全 `0xFF`（已擦除 ⇒ 不必再擦）|

## 3. 分区约定（8 MB 随便用，改四个宏即可）

| 区 | 偏移 | 大小 | 用途（对应手册 P12）|
|---|---|---|---|
| `FLASH_PARAM_ADDR` | `0x000000` | 4 KB | 参数区（零位/PID/标定，读写频繁）|
| `FLASH_LOG_ADDR` | `0x001000` | ~1 MB | **调试数据登记区** ✓ |
| `FLASH_IMAGE_ADDR` | `0x100000` | 7 MB | **LCD 图片 / 大块数据** ✓ |

## 4. ⚠️ 三条使用纪律

1. **本层是阻塞的** ✗：一个 4KB 扇区擦除约 **45ms**、页写约 0.4ms。
   ⇒ 别在 **1 kHz 控制任务**里调 ✗；请在初始化阶段、或低优先级任务里做 ✓。
2. **写入前必须先擦** ✗：FLASH 只能把 1 写成 0 ⇒ 往没擦的地方写 = **按位与** ✗。
   ⇒ 先 `Flash_Is_Blank()` 查 ✓，或先 `Flash_Erase()` ✓。
3. **本层只能在一个任务里用** ✗：它在等待时会**自己泵 `W25Q64_Task()`** ✓
   （否则调用方也在泵同一个状态机 ⇒ 在这里死等会让回调永远不来 ✗）。
   现在挂在 `Robot_Init()`（初始化）与 `Robot_Task()`（若以后要用）✓ —— 两边是同一个任务 ✓ 安全 ✓。

## 5. 用法

```cpp
/* Robot_Init()（已接好 ✓） */
flash_init_ok  = Flash_Init();            // 内部已做 JEDEC 自检
flash_jedec_id = Flash_Get_Jedec_Id();    // 调试器看这个,应为 0xEF4017

/* 存一段调试数据（低优先级任务 / 空闲时做） */
if (Flash_Is_Blank(FLASH_LOG_ADDR, len) || Flash_Erase(FLASH_LOG_ADDR, len)) {
    Flash_Write(FLASH_LOG_ADDR, data, len);
}

/* 读回来 */
Flash_Read(FLASH_LOG_ADDR, data, len);
```

## 6. 验证

```
build   exit 0
符号    Flash_Init / Flash_Get_Jedec_Id ✓ ; 调试量 flash_init_ok / flash_jedec_id ✓
上板    flash_init_ok == true 且 flash_jedec_id == 0xEF4017  ⇒ 外部 8MB Flash 通了 ✓
```
（若 `flash_init_ok == false`：查 OSPI 配置 `ClockPrescaler=3` / `SampleShifting=HALFCYCLE` /
`FifoThreshold=8`、以及 `it.c` 里的 `HAL_OSPI_IRQHandler(&hospi2)` ✓ —— 见 `bsp_ospi.md`。）

## 7. 相关

* `User/Bsp/ospi/bsp_ospi.md` —— 总线驱动（含必须对齐的三个 OSPI 参数）
* `User/Device/w25q64/dev_w25q64.md` —— 芯片驱动（命令表 / 超时 / 内存映射模式）
* 厂家例程：`E:\rm\例程\CtrBoard-H7_W25Q64\`（同板官方例程，我们的配置与它逐项对齐过 ✓）