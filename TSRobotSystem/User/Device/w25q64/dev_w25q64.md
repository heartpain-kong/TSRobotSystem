# dev_w25q64 — W25Q64JV(8MB SPI NOR Flash,挂 OCTOSPI2)

文件：`dev_w25q64.h` / `dev_w25q64.cpp`。参照 `E:\rm\dm02_test\...\2_Device\BSP\W25Q64JV`
（同板：引脚标签就是 `W25Q64_WP/HOLD/IO0-3`），按本工程 RTOS 形态改写。

## 怎么用

初始化（`User/App/robot/robot.cpp::Robot_Init()` 里已经接好）：

```cpp
W25Q64_Register_Callbacks();                     // 注册 OSPI 三个回调
if (!W25Q64_Init(W25Q64_MODE_NORMAL)) { ... }    // 读 JEDEC ID 自检;结果看 w25q64_init_ok
```

> `w25q64_init_ok` 是 `robot.cpp` 里的全局量：**调试器直接看它** —— false 说明没读到芯片
> （查：OSPI 配置、SCK 过高、焊接、`OCTOSPI2_IRQHandler` 里有没有 `HAL_OSPI_IRQHandler`）。

读写（**非阻塞**，由 `W25Q64_Task()` 推进）：

```cpp
W25Q64_Read(addr, len);                          // 发起
while (W25Q64_Is_Busy()) { W25Q64_Task(); osDelay(1); }   // 或在周期任务里自动推进
const uint8_t *p = W25Q64_Get_Buffer();          // 读到的数据

W25Q64_Write_Page(buf, addr, len);               // len 不能跨 256B 页
W25Q64_Erase_Sector(addr);                       // 4KB 对齐
W25Q64_Erase_Block_64K(addr);                    // 64KB 对齐
W25Q64_Erase_Chip();                             // 几十秒
```

`W25Q64_Task()` 已经挂在 `Robot_Task()`（500 Hz）里 ✓ —— 所以正常使用时**不用自己轮询**，
发起后用 `W25Q64_Is_Busy()` / `W25Q64_Is_Rx_Done()` 查状态即可。

## 状态机（都在 `W25Q64_Task()` 里推进，中断只置标志）

```
IDLE ─Read──► READ ──────────收完回调──────► DONE(Is_Rx_Done=true)
     │
     ├Write─► WRITE_ENABLE(0x06 + 轮询 WIP=0) ─轮询回调─► PROGRAM(0x32 + DMA)
     │                                                      └─发完回调─► WAIT_WIP ─轮询回调─► DONE
     └Erase─► WRITE_ENABLE ─轮询回调─► ERASE(0x20/0xD8/0xC7) ─► WAIT_WIP ─轮询回调─► DONE
```

* **等 WIP**：先发 `0x05`(读状态寄存器1)，再用 `HAL_OSPI_AutoPolling_IT` 轮询
  `Mask=0x01 / Match=0x00`（WIP 清零）—— 这是 W25Q64 擦写必须的等待；
* **超时**：各自独立（写/读 50ms、扇区 500ms、64K 2s、整片 120s），超了进 `ERROR`
  并累加 `W25Q64_Get_Error_Count()`，由 `W25Q64_Clear_Error()` 复位 —— 防止 Flash 卡死拖死任务。

## 指令表

| 操作 | 指令 | 线数 |
| --- | --- | --- |
| 读 JEDEC ID | `0x9F` | 1 线，3 字节 → `EF 40 17` |
| 读 | `0xEB` FAST_READ_QUAD_IO | 地址 4 线 + 数据 4 线 + **6 dummy** |
| 写 | `0x06` 写使能 → `0x32` QUAD_PAGE_PROGRAM | 数据 4 线；**≤256B 且不跨页** |
| 擦 | `0x20` 扇区4K / `0xD8` 块64K / `0xC7` 整片 | 地址 1 线 |
| 等 | `0x05` 读状态寄存器1 | WIP = bit0 |

## ⚠️ 内存映射模式（可选）的坑

`W25Q64_Init(W25Q64_MODE_MEMORY_MAPPED)` 会让 OCTOSPI2 进入内存映射模式：
之后读数据可以直接 `*(volatile uint8_t *)(0x70000000u + addr)`（**不能再发命令**，要退出得重新 `HAL_OSPI_Init`）。

**必须同时给 `0x70000000` 加一条非缓存 MPU 区**，否则 D-Cache 会返回旧数据：

```c
/* main.c::MPU_Config() 里追加(region 1 空着) */
MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
MPU_InitStruct.BaseAddress      = 0x70000000;
MPU_InitStruct.Size             = MPU_REGION_SIZE_256MB;
MPU_InitStruct.SubRegionDisable = 0x0;
MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
HAL_MPU_ConfigRegion(&MPU_InitStruct);
```

> 只走**间接模式**（默认）时不需要这条 —— 那时读数据走 DMA 到 RAM_D1(非缓存)，天然安全。

## 相关

* `User/Bsp/ospi/bsp_ospi.md` —— 总线驱动（含两个必须对齐的 OSPI 参数）
* `User/App/robot/robot.md` —— 接线位置（`Robot_Init()` / `Robot_Task()`）
