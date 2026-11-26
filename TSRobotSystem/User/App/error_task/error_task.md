# error_task — 错误处理（蜂鸣器 + RGB 逐位量化） + 关键数据记录

文件：`error_task.h` / `error_task.cpp`
任务：**100 Hz（10 ms）**、优先级 **`osPriorityLow`（最低）**、栈 3072 B
落盘：**10 Hz**（`ERROR_LOG_PERIOD_MS = 100`，改成别的值即可）

---

## 1. 它干两件事

```
Error_Task() 每 10ms ─┬─► 算错误码 → 蜂鸣器(N 声) + WS2812(LED0 类别 + LED1~7 二进制码)
                      └─► 码/级别变化时写一条**带可读名**的文本日志(能看出是哪台电机)
                      每 100ms ─► Log_Record(二进制帧:底盘指令/运行 + 欧拉角 + 云台指令/运行)
```
**只往 `dev_log` 的内存环形里塞**（µs 级，不阻塞）✓ —— 真正落盘由 `Robot_Task` 的 `Log_Task()` 做 ✓

---

## 2. ★ 错误码表（全部量化）

| 码 | 含义 |
|---|---|
| **0** | 无错 |
| **1** | RTOS 堆分配失败（`g_fault_code = 1`）|
| **2** | 任务栈溢出（`g_fault_code = 2`，出事任务名在 `g_fault_task`）|
| **3** | `Error_Handler`（HAL 初始化失败等）|
| **4** | HardFault |
| **5** | 外部 Flash 初始化失败 |
| **6** | 日志初始化失败 |
| **7** | IMU（BMI088）初始化失败 |
| **8** | ADC 初始化失败 |
| **9** | WS2812 初始化失败 |
| **10** | WS2812 时钟不对（CubeMX 里 SPI6 分频不是 32）|
| **11** | 蜂鸣器初始化失败 |
| **12** | 继电器（`dev_electric`）初始化失败 |
| **13** | 电压检测没就绪 / 欠压 / 过压 |
| **14** | 日志记录区满 |
| **15** | 日志丢过数据 |
| **20 + n** | **第 n+1 台电机初始化失败**（n 从 0 起 ⇒ 20=第1台，21=第2台…）|
| **40 + n** | **第 n+1 台电机离线**（100 ms 没收到反馈）|
| **60 + n** | **第 n+1 台电机报错**（反馈里的 `MError != 0`）|
| **80 + b** | 自定义标志位第 b 位（`Error_Set_Flag`）|
| **120+** | 你自己 `Error_Report()` 报的码 |

> 「哪一台」由 `motor_task_get_name(n)` 给出（如 `CAN1-DM-1`）✓
> 电机相关码全是 **20/40/60 + 下标**，所以加减就能反推是哪台 ✓

---

## 3. ★ 怎么"读"显示（三处一起对，最快）

**RGB（8 颗）**

```
LED0      = 类别色     红=FATAL  橙=ERROR  黄=WARNING  绿=OK(呼吸)
LED1~LED7 = 错误码的 bit0~bit6（常亮，半个亮度）
```

| 例子 | 读法 |
|---|---|
| LED0 红闪 + LED1、LED2 亮 | 码 = 1+2 = **3** ⇒ `Error_Handler` |
| LED0 橙闪 + LED1、LED3 亮 | 码 = 1+4 = **5** ⇒ 外部 Flash 初始化失败 |
| LED0 橙闪 + LED2、LED5 亮 | 码 = 2+16 = **18**? → 但表里没有 18 ⇒ 实际是 **20+**? 用蜂鸣器/日志确认（20+ 的码 LED 上就是二进制，20=LED3+LED5）|
| LED0 橙闪 + LED3、LED5 亮 | 码 = 4+16 = **20** ⇒ **第 1 台电机初始化失败**（看日志里的名字）|
| 全绿色心跳(暗绿底 + 每 1.2s 双闪) | 一切正常 |

**蜂鸣器**

```
音高 = 类别(FATAL 3000Hz / ERROR 2500Hz / WARNING 1500Hz)
响 N 声(N = 码,超过 10 按 10 算) → 长静音 → 循环
```
⇒ 「橙灯 + 嘀 20 声?」不可能（上限 10 声）⇒ **码 > 10 时精确值看 LED 二进制 + 日志** ✓
日志里那条文本是**最完整**的：`code 20(motor init fail: CAN1-DM-1) level=ERROR fault=0 ...` ✓

---

## 4. 各 App 层怎么用

### ① 关键数据（零拷贝，写进去就会被 10 Hz 记录）

```cpp
Error_Snapshot_t *s = Error_Get_Snapshot();
s->Chassis_Cmd[0..2] = vx, vy, w;         // 底盘指令
s->Chassis_Now[0..2] = 里程计;             // 底盘运行
s->Imu_Euler[0..2]   = roll, pitch, yaw;  // 欧拉角(连续 yaw, rad)
s->Gimbal_Cmd[0..1]  = yaw, pitch;        // 云台指令
s->Gimbal_Now[0..1]  = 云台实际;
s->Mode              = mode;
```
（这些 App 层还没建 ⇒ 快照里暂时是 0 ✓，填上自动生效 ✓）

### ② 运行期报错

```cpp
/* 用标志位(推荐,会自动折算成码 80+n) */
Error_Set_Flag(ERROR_FLAG_MASK(ERROR_FLAG_MOTOR), true);    // 出错
Error_Set_Flag(ERROR_FLAG_MASK(ERROR_FLAG_MOTOR), false);   // 恢复

/* 或者直接报一个自定义码(建议 120 起) */
Error_Report(120u, true);      // 报
Error_Report(120u, false);     // 撤
```
可用位：`MOTOR / IMU / REMOTE / FLASH / LOG / LED / ADC / VOLTAGE / SENSOR / TASK / CUSTOM`

> **设备初始化失败**和**电机状态**本任务自己查（`*_init_ok` + `motor_task_is_*`）✓ 不用手报 ✓

---

## 5. 落盘帧格式（上位机 dump）

```c
typedef struct {
    uint16_t Magic;      // 0x4552 = 'R''E'
    uint8_t  Type;       // 0 = 关键数据快照
    uint8_t  Level;      // 0=OK 1=WARN 2=ERROR 3=FATAL
    uint16_t Code;       // ★ 错误码(见第 2 节表)
    uint16_t Reserved;
    uint32_t Ms;         // 时间戳(ms)
    uint32_t Flags;      // 自定义标志位
    Error_Snapshot_t Snapshot;   // 底盘/欧拉角/云台
} Error_Frame_t;         // 70 字节 ⇒ 一条 256B Log 记录放得下
```

---

## 6. 排错时先看什么

| 灯/声 | 码 | 先查 |
|---|---|---|
| 红闪 + 长鸣式 | 1~4 | `g_fault_code` / `g_fault_task`（FreeRTOS 堆、任务栈、HardFault）|
| 橙闪 | 5~13 | 对应设备的 `*_init_ok`；10 号 ⇒ 去 CubeMX 把 SPI6 分频改 32 |
| 橙闪 | 20+n | 第 n+1 台电机使能后没回反馈 ⇒ 查 CAN 接线 / 电机 id / 波特率 |
| 橙闪 | 40+n | 第 n+1 台运行中掉线 ⇒ 查线 / 电源 / 该台是否报错 |
| 橙闪 | 60+n | 第 n+1 台报 `MError != 0` ⇒ 看电机自身故障（过温/过压/过流）|
| 橙闪 | 80+b | `Error_Get_Flags()` 第 b 位被置 ⇒ 看是哪层报的 |
| 黄闪 + 慢"嘀" | 13~15 | 电压（`Power_Get_Voltage()`）/ 日志满 / 日志丢数据 |
| **全绿色心跳(暗绿底 + 每 1.2s 双闪)** | 0 | 正常 |
| 绿但**不呼吸** | — | 任务没跑起来 ⇒ 看 `g_thread_create_failed`（>0 = FreeRTOS 堆不够）|

---

## 7. 上板自检

```
① 正常上电:8 颗灯绿色心跳(暗绿底 + 每 1.2s 双闪)、无声音
② 拔掉一台电机再上电 ⇒ 橙灯 + 码 20/21(哪台看 LED 二进制 + 日志里的名字)
③ 运行中拔 CAN 线 ⇒ 先 40+n(离线),插回 ⇒ 自动恢复
④ Error_Report(120, true) ⇒ 码 120:LED1~7 显示 120 的二进制 + 橙灯 + 响 10 声
⑤ 跑一会儿断电,用 Log_Read/dump 看:一串 'R''E' 帧,每帧带 Code/Level/关键数据
```
