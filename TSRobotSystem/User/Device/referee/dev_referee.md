# dev_referee — 裁判系统（USART1: 115200 8N1）

文件：`dev_referee.h` / `dev_referee.cpp`

## 1. 硬件

| 项 | 值 |
|---|---|
| 串口 | **USART1**（`huart1`）|
| 引脚 | **PA9 = TX，PA10 = RX** |
| 参数 | **115200 / 8 数据位 / 无校验 / 1 停止位**（CubeMX 里就是这个 ✓）|
| 缓冲 | `bsp_uart.h` 的 `USART1_recv_Data_N = 136`（= 裁判系统最大帧 ✓ 早就预留好了 ✓）|
| 接收 | DBM + IDLE 引擎 ⇒ 回调一次拿到**一整帧** ✓ |

## 2. 帧格式

```
SOF(0xA5) | data_length(2) | seq(1) | CRC8(1) | cmd_id(2) | data(n) | CRC16(2)
                ↑ data_length = 从 seq 到 data 结束 = n + 4
                  帧总长 = data_length + 5
```

| 校验 | 范围 | 算法 |
|---|---|---|
| **CRC8** | 字节 `0~3`（SOF + data_length + seq），存在字节 4 | poly **0x31**、init **0xFF** |
| **CRC16** | 字节 `5~size-3`（cmd_id + data），存在最后 2 字节 | CRC-CCITT（工程里 `crc_ccitt.h` 现成 ✓）|

> ⚠️ 工程里的 `User/algorithm/crc` **只有 CRC16/CRC32** ✗ ⇒ CRC8 的表在本模块内部生成（首次调用时算一次，之后查表 ✓）

## 3. 数据流（对应流程图「裁判系统 → cmd / robot」）

```
USART1 ISR ─► Referee_Uart_Callback():校验(帧长→CRC8→CRC16) → 按 cmd_id 解包到 s_ref
Referee_Task() ─► 掉线检测(>1s 没帧) + 有新帧就 Msg_Publish(MSG_TOPIC_REFEREE)
cmd 任务   ─► Msg_Latest(MSG_TOPIC_REFEREE):比赛阶段/剩余时间 → 决定"能不能动、能动多快"
robot 任务 ─► Msg_Latest(MSG_TOPIC_REFEREE):血量/等级 → 上层决策
```
**消费者是 `cmd` 和 `robot` 两个** ✓（同一话题多个消费者用 `Msg_Latest` + `Msg_Is_New(seq)` ✓ 互不影响 ✓）

## 4. 已解析的 cmd_id

| cmd_id | 内容 | 解出的字段 |
|---|---|---|
| `0x0001` | 比赛状态 | `Game_Type`、`Game_Progress`（4 = 比赛中）、`Stage_Remain_S` |
| `0x0201` | 机器人状态 | `Robot_Id/Level`、`Hp/Max_Hp`、`Heat/Heat_Limit/Barrel_Cooling`、**`Chassis_Power_Limit`**、三个功率管理输出 |
| `0x0003` | 机器人血量 | `Hp` |

要加别的 ID（飞镖/雷达/伤害事件…）⇒ 在 `Referee_Dispatch()` 里加一个 `case` 即可 ✓

## 5. API

| 接口 | 说明 |
|---|---|
| `Referee_Init(&huart1)` | 注册接收回调（传 NULL 用默认 `huart1`）|
| `Referee_Task()` | 周期跑（CMD 任务 500 Hz 调）：掉线检测 + 发布 |
| `Referee_Get()` | 读数据（`Referee_Data_t*`）|
| `Referee_Is_Online()` | 是否在线 |
| `Referee_Is_In_Match()` | 是否比赛进行中（`Game_Progress == 4`）|
| `Referee_Get_Stage_Remain()` | 当前阶段剩余时间（s）|

## 6. 坑

1. **CRC8 的 init 是 0xFF** ✗（不是 0x00）—— 写错会"全部帧都校验不过"
2. **`data_length` 从 `seq` 开始算** ✗（不是从 `cmd_id`）⇒ 帧总长 = `data_length + 5`
3. **回调在 ISR 里** ✗ ⇒ 只做校验 + 解包，不发消息（发布在 `Referee_Task()` 里 ✓）
4. **裁判系统 10 Hz 左右** ✓ ⇒ 掉线超时给到 **1 s**（给 100 ms 会一直误判掉线 ✗）
5. `Chassis_Power_Limit` **只有 0x0201 里有** ✓ ⇒ 底盘限功率要用它，别用 `0x0001`

## 7. 上板自检

```
① 不接裁判系统:referee_init_ok == true,Is_Online() == false,Rx_Frames == 0
② 接上裁判系统 ⇒ Rx_Frames 开始涨(~10Hz),Online == true
③ Crc8_Errors / Crc16_Errors 应一直为 0(不为 0 ⇒ 波特率/线序/CRC 实现有问题)
④ Reject_Count 一直涨 ⇒ 帧长不对(多半是 bsp_uart 那个口的 recv 缓冲长度没配够)
⑤ Game_Progress 在比赛中应变成 4;Stage_Remain_S 大约每秒 -1
⑥ Msg_Get_Published(MSG_TOPIC_REFEREE) 跟着 Rx_Frames 涨
```
