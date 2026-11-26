# bsp_spi — SPI 总线驱动

文件：`bsp_spi.h` / `bsp_spi.cpp`。参照 `E:\rm\dm02_test\...\1_Middleware\Driver\SPI\drv_spi.cpp`。

## 它提供什么

| 接口 | 说明 |
| --- | --- |
| `BSP_SPI_Init(&hspi2, 回调)` | 绑定句柄 + 注册完成回调（可传 `NULL`）|
| `BSP_SPI_Get_Tx_Buffer / Get_Rx_Buffer` | 取 512B 收发缓冲（器件驱动往里填/往外读）|
| `BSP_SPI_Transmit_Receive(hspi, CS_GPIOx, CS_Pin, CS_有效电平, tx_len, rx_len, timeout_ms)` | **阻塞**全双工收发，**片选由它包住** |
| `BSP_SPI_Transmit(...)` | 只发（内部就是上面 `rx_len=0`）|
| `BSP_SPI_Transmit_Receive_IT(...)` | 中断版，立即返回；收完进回调（回调里顺带把 CS 拉回）|

## ★ 为什么片选要传进来

本工程 **SPI2 没有硬件 NSS**（`SPI2.NSSPMode = SPI_NSS_PULSE_DISABLE`），CS 必须是软件 GPIO：
* 加速度计 `ACCEL_CS = PC0`、陀螺仪 `GYRO_CS = PC3`，**低电平有效**；
* 每个收发函数自己保证「CS 有效 → 收发 → CS 回空闲」，中途出错也会拉回（`HAL_SPI_ErrorCallback`）。

```cpp
/* 例:读寄存器(把 CS/引脚/有效电平传进来) */
tx[0] = reg | 0x80;
BSP_SPI_Transmit_Receive(&hspi2, GPIOC, GPIO_PIN_0, GPIO_PIN_RESET, 2, 2, 5);
uint8_t value = BSP_SPI_Get_Rx_Buffer(&hspi2)[1];
```

## 为什么用阻塞版

BMI088 读一次几个字节，10 MHz 下 **≈ 5 µs** —— 阻塞完全无所谓，而且不涉及"ISR 里调 RTOS API"的风险。
需要非阻塞（比如以后接显示屏/大量数据）时再换 `_IT` 版（回调已实现 ✓）。

## 缓冲与 DMA/Cache

缓冲在 `.bss` → **RAM_D1**（MPU 已设 Non-cacheable）⇒ 换 DMA 时也不用 cache 维护 ✓。

## HAL 回调覆盖（已用 nm 验证）

`HAL_SPI_TxRxCpltCallback` / `HAL_SPI_TxCpltCallback` / `HAL_SPI_ErrorCallback`
在本文件里重定义（覆盖 HAL 的 `__weak`），在 ELF 里是 **`T`（强符号，来自 `bsp_spi.cpp.obj`）** ✓。

## 相关

* `User/Device/bmi088/dev_bmi088.md` —— 用它的 IMU 驱动
* `User/Bsp/ospi/bsp_ospi.md` —— 另一条总线（外部 Flash）
