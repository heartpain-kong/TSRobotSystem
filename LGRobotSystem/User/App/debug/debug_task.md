# debug_task — 调试遥测（UART7 + USART10 双路输出）

文件：`debug_task.h` / `debug_task.cpp`　层：**App**
任务：独立任务 `debug`（10 ms = 100 Hz 调 `Debug_Task()`）✓

## 1. 它是干什么的

把机器人内部状态**串口输出**给上位机（`E:\rm\上位机接收器\` ✓），用于：
* 实时看电机/姿态/底盘/云台 ✓
* **故障排查**：帧里带 `g_fault_code` / `CFSR` / 出错 PC ✓
* 上位机能导出「带时间 + 串口号的文档」✓

## 2. 走哪两个口（★ 别搞混）

| 端口 | 引脚 | 参数 | 用途 |
|---|---|---|---|
**UART7** | `PE7 = RX` `PE8 = TX` | **921600 8N1** | ★ 调试遥测输出 + 0x5C 调参输入 |
**USART10** | `PE2 = RX` `PE3 = TX` | **921600 8N1** | ★ 同上（与 UART7 **同时发**，内容一致）|
**USART3** | `PD8 = TX` `PD9 = RX`，`PB14 = DE` | **4000000 8N1**（`OverSampling=8`）| **RS485 通信**（不是调试 ✗）|

* 两路是**同一份帧、同一时刻**发出的 ✓ —— 接哪个口都能看 ✓
* 关闭某一路：`debug_task.h` 里把 `DEBUG_USE_UART7` / `DEBUG_USE_USART10` 置 0 ✓
* 发送用 **DMA** ✓，且两路**各有独立缓冲** ✓（`.AXI_SRAM` ✓，DMA 安全 ✓）；某一路忙就**只丢那一路** ✓ 不影响另一路 ✓

## 3. 帧格式（定长 230 字节，`0x5A` 开头）

```
[0]    0x5A            帧头(与裁判/上位机的 0xA5 区分)
[1]    版本 = 1
[2..3] 帧总长 = 230    小端
[4..5] cmd_id = 0x0101
[6..7] 序号            每发出一次 +1
[8..227] payload       220 B
[228..229] CRC16(反射版 0x8408 / init 0)覆盖 [0..227]
```

payload 偏移见 `debug_task.h` 的 `DEBUG_PL_*` 宏 ✓（错误级别/电机块/诊断/加速度/栈水位/日志上电链/温度/任务心跳/Flash 诊断/外接 IMU 预留）。

### 3.1 极简电机帧（`DEBUG_MOTOR_PID_ONLY=1`，当前默认）

```
[0]    0x5A
[1]    2
[2..3] 158         小端
[4..5] 0x0102
[6..7] 序号
[8..155] payload 148B
[156..157] CRC16
```

payload：

```text
[0]     电机 ID
[1]     flags
[2..37] 9 × f32:关节 rpm / 电机 rpm / 目标 rpm / 反馈 rpm / 误差 rpm
                  / 力矩 N·m / 位置 rad / 目标位置 rad / 位置误差 rad
[38]    PE2 原始电平
[39]    PE3 原始电平
[40..59] 故障块(错误级别/错误码/致命码/CFSR/PC/标志)
[60..83] IMU: roll/pitch/yaw + gx/gy/gz(各 f32,rad 或 rad/s)
[84..115] 遥控器/键鼠块(32B):
          [84] online
          [85] 左拨杆  [86] 右拨杆
          [87] 鼠标左键 [88] 鼠标右键
          [90..91] u16 键盘位图
          [92..101] 5 × i16 原始杆量(中心 0,约 ±660)
          [102..107] 3 × i16 鼠标 x/y/z
          [108..109] u16 掉线次数
          [112..115] u32 累计收帧数
[116..147] IM6014(UART2 ID1,0.5 N·m)块(32B):
          [116] online  [117] status
          [120..123] u32 MError32
          [124..127] f32 pos(rad)
          [128..131] f32 w(rad/s)
          [132..135] f32 t(N·m)
          [136..139] f32 temp(℃)
          [140..143] f32 voltage(V)
          [144..145] u16 recv_crc_err
          [146..147] u16 mwarn
```

IMU / 遥控器 / IM6014 块都是新增的；旧 70 B、94 B、126 B 帧仍按长度兼容。

## 4. 每拍做什么

```
① 限速:距上次不足 10ms 直接返回
② 组帧(帧头 + payload + CRC)—— 组一次,两路共用
③ 分别往 UART7 / USART10 发 DMA;哪路 UART 不 READY 就丢哪路(绝不阻塞)
```

## 4.1 每秒一行可读文本（1 Hz，两个口都发）

```
[LOG] boot=3 cur=0x008000 prev=0x002000 stop=0x007F00 wrap=1
```

串口助手直接能读 ✓；上位机会把它单独显示 ✓；`DEBUG_TEXT_ENABLE=0` 可关 ✓

## 4.2 0x5C 调试命令（上位机“电机调试”窗口）

调试口现在**也收**命令（UART7/USART10，921600 8N1）：

```
[0x5C][cmd][seq_h][seq_l][payload]
```

| cmd | payload | 作用 |
|---|---|---|
| `0x01` | int16 | 速度目标 [rpm] |
| `0x02` | int32 | 位置目标 [0.001°] |
| `0x03` | 6×f32 | 速度 Kp/Ki/Kd + 位置 Kp/Ki/Kd；写完后回 `[LOG] PID set ...` |
| `0x04` | — | 查询当前 PID；回 `[LOG] PID now ...` |
| `0x05` | 1 B | 停止（目标 0 + lose）|

- 命令在 `Debug_Task()`（任务上下文）里处理，ISR 只搬运；
- PID 回读行用 `[LOG]` 前缀，APP 的文本解析器会直接显示在日志里；
  `MOTOR_POS_LOOP_ENABLE=0` 时位置环不参与控制，但位置 PID 参数仍会保存/回读；
- 第一条命令生效后 `s_manual_override=true`，底盘对 0 号电机的写入会被忽略，直到复位；
- 诊断量：`g_debug_rx_bytes`（收到的原始字节）、`g_debug_cmd_count`（执行成功的命令数）。
  - `g_debug_rx_bytes == 0` ⇒ CH340 的 **TX 没接到板子 RX**（PE8 或 PE3）；
  - 有字节但命令数不涨 ⇒ 帧格式/长度不对。

> RX 接线：UART7 `PE7`，USART10 `PE2`；CH340 的 TX → 板子 RX，GND 共地。
> 只接 TX（板子→CH340）只能看遥测，不能调参。

## 5. 坑

1. **别把 USART3 拿来当调试口** ✗ —— 它是 RS485（带 DE 引脚 ✓），接调试器会乱 ✓
2. **两个口的 TX DMA 必须都在 CubeMX 里配好** ✗（`usart.c` 里要能看到 `__HAL_LINKDMA(..., hdmatx, hdma_uart7_tx / hdma_usart10_tx)` ✓）
3. **缓冲必须在 `.AXI_SRAM`** ✓（DMA 可见 ✓ 且该段 non-cacheable ✓ 不用手工清 cache ✓）
4. 当前帧长 **230** ✓ —— 改 `DEBUG_MAX_MOTORS`/尾部块会改帧长 ✗，上位机解析器要同步改 ✓
5. **TX 遥测 + RX 调参**：遥测只发；`0x5C` 命令走 RX。只接 TX 时能看不能调 ✓

## 6. 上板自检

```
① 串口助手开 921600 接 PE8(UART7 TX):应看到周期 10ms、帧头 0x5A（当前固件整帧 **230** 字节）
② 换接 PE3(USART10 TX):内容应与 PE8 完全一致
③ 用上位机(自动匹配端口):应把该口识别为"调试遥测"并显示电机/姿态
④ 拔掉遥控器 → 帧里的 err_level/err_code 变化,上位机"关键事件"里能看到时间点
⑤ 人为触发故障 → 帧里 fault_code=4、CFSR/PC 有值 ⇒ 拿 PC 跑 addr2line 定位
```
