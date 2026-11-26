# dev_bmi088 — BMI088 六轴 IMU（走 SPI2）

文件：`dev_bmi088.h` / `dev_bmi088.cpp`。参照 `E:\rm\dm02_test\...\2_Device\BSP\BMI088`（同板驱动），
本工程只保留「自检 + 配置 + 读原始数据 + 换算」；**姿态解算不在这里**（EKF/卡尔曼/四元数属于算法层）。

## 硬件与总线

| | 值 | 说明 |
| --- | --- | --- |
| SPI | **SPI2**，主机 / 8bit / **Mode3**(CPOL=1,CPHA=1) / 10 Mbit/s | BMI088 支持 Mode0 与 Mode3；10 MHz 是其上限 |
| 片选 | 加速度计 **PC0**(`ACCEL_CS`)、陀螺仪 **PC3**(`GYRO_CS`)，低电平有效 | 两颗 die **独立片选、两套寄存器** |
| 软件驱动 | `User/Bsp/spi/bsp_spi.*` | CS 由总线驱动包住整帧 |

## ★ 两个必须知道的坑

1. **加速度计的 SPI 读要在地址后多读 1 个 dummy 字节**（写**不用**）：
   `[reg|0x80] [dummy] [data0] [data1] …` ⇒ 数据从第 3 个字节开始（`dev_bmi088.cpp` 内部已处理）。
2. **复位后第一次读 CHIP_ID 可能是垃圾值** ⇒ 驱动里读两次、用第二次判断。

## 初始化顺序（驱动内部）

```
陀螺仪:软复位(0xB6) → 等 30ms → 读 CHIP_ID(应 0x0F) → GYR_RANGE → GYR_BANDWIDTH → GYR_LPM1=0(正常模式)
加速度计:软复位(0xB6) → 等 50ms → ACC_PWR_CTRL=0x04(打开) → 等 50ms
        → ACC_PWR_CONF=0x00(active,只有 active 才能改配置) → 等 5ms
        → 读两次 CHIP_ID(应 0x1E) → ACC_CONF=0xA8(400Hz/正常带宽) → ACC_RANGE
```

## 怎么用

```cpp
/* Robot_Init() 里(已经接好) */
BSP_SPI_Init(&hspi2, nullptr);
bmi088_init_ok = BMI088_Init();     // 调试器看这个变量:true = 两颗 die 都在

/* Robot_Task()(500Hz,已经接好) —— BMI088_Task() 内部按 1ms 自己限速 */
BMI088_Task();

/* 取数据 */
float ax, ay, az;  BMI088_Get_Accel(&ax, &ay, &az);   // m/s²
float gx, gy, gz;  BMI088_Get_Gyro(&gx, &gy, &gz);    // rad/s
float t = BMI088_Get_Temperature();                    // ℃(温度在加速度计 die 里)
bool ok = BMI088_Is_Online();
```

## 换算与量程（改量程要同步改系数）

| | 默认 | 系数 |
| --- | --- | --- |
| 加速度 | `BMI088_ACCEL_RANGE_REG = 0x01`(**±6g**) | `BMI088_ACCEL_LSB_PER_G = 5460`（±3g→10920、±12g→2730、±24g→1365）|
| 陀螺 | `BMI088_GYRO_RANGE_REG = 0x00`(**±2000dps**) | `BMI088_GYRO_LSB_PER_DPS = 16.384` |
| 温度 | —— | `raw11 × 0.125 + 23` ℃ |

## 上板自检清单

| 现象 | 查什么 |
| --- | --- |
| `bmi088_init_ok == false` | SPI2 是否初始化（`MX_SPI2_Init`）；CS 是否接对（PC0/PC3）；SPI 模式是否 Mode3；10 MHz 是否太快（先降到 5 MHz 试）|
| ID 读出来是 `0x00`/`0xFF` | MISO/MOSI/CLK 接反或没接；CS 电平反了（应低有效）|
| 陀螺对、加速度计不对 | 加速度计的 **dummy 字节** 处理（本驱动已做，别自己另写一套读函数）|
| 数据抖动大 | 量程/带宽；或者根本在读旧数据（检查是否误开了内存映射）|

## 相关

* `User/Bsp/spi/bsp_spi.md` —— SPI 总线驱动（软件片选）
* `User/App/robot/robot.md` —— 接线位置
