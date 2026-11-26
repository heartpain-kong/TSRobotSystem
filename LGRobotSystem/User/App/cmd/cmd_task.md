# cmd_task — 指令任务（遥控器/键鼠 + 裁判系统 + 上位机 → 统一控制指令）

文件：`cmd_task.h` / `cmd_task.cpp`
任务：**500 Hz（2 ms）**、优先级 `osPriorityNormal`、栈 3072 B（`robot_def.h` 里改）

---

## 1. 它在流程图里的位置

```
遥控器/键鼠 ─ dev_rc ────────┐
上位机(USART/USB,未做)───────┼─► MSG_TOPIC_RC / HOST / REFEREE
裁判系统 ─ dev_referee ──────┘            │
                                          ▼
                                     cmd_task  ◄── 本模块
                                          │  Msg_Publish(MSG_TOPIC_CMD)
                                          ▼
                          robot / chassis / gimbal / highRise 各自 Msg_Latest
```

**它同时负责推进两个输入设备**：`Rc_Task()` 与 `Referee_Task()` 都在 `Cmd_Task()` 里调用 ✓
（所以别在别的任务里再调一次 ✗）

## 2. 输入 → 输出

| 输入 | 来源 | 用途 |
|---|---|---|
| `MSG_TOPIC_RC` | `dev_rc`（遥控器/键鼠）| 模式、底盘速度、云台、发射 |
| `MSG_TOPIC_REFEREE` | `dev_referee`（裁判系统）| 比赛门控 + 底盘功率上限 |
| `MSG_TOPIC_MINIPC` | 上位机（**还没做** ✗）| 自动模式指令（占位：收到就置 `CMD_SRC_HOST`）|

输出 `MSG_TOPIC_CMD` = `Cmd_Data_t`（见 `cmd_task.h`）✓

## 3. 模式（右拨杆 S1）

| S1 | 模式 | 说明 |
|---|---|---|
| **上(1)** | `CMD_MODE_REMOTE` | 遥控 |
| **中(2)** | `CMD_MODE_AUTO` | 自动（等上位机接入 ✗）|
| **下(3)** | `CMD_MODE_IDLE` | **空闲：全零，机器人不动**（安全态）|
| 掉线 / `rc.Lost` | `CMD_MODE_IDLE` | 遥控器没了 ⇒ 立刻全零 ✓ |

## 4. 操作映射

| 输入 | 作用 |
|---|---|
| 右杆 上下(ch1) | `Chassis_Vx` 前后（±3 m/s）|
| 右杆 左右(ch0) | `Chassis_Vy` 左右（±3 m/s）|
| 左杆 左右(ch2) | `Chassis_W` 旋转（±8 rad/s）|
| **左拨杆 S2 上(1)** | **小陀螺** `Chassis_Spin = true` |
| **左拨杆 S2 下(3)** | **云台不使能** `Gimbal_Enable = false`（搬运/调试用）|
| 鼠标 x / y | 云台 yaw / pitch（**增量式**累加，±0.0016 rad/计数）|
| 键 **R** | 切小陀螺（和 S2 上等效）|
| 键 **F** | 云台归中（yaw=pitch=0）|
| 键 **Shift** | 摩擦轮（`Shoot_Enable`）|
| 键 **Q** / **E** | 单发 / 连发（`Shoot_Rate` = 1 / 2）|

摇杆有 **0.05 死区** ✓；pitch 限幅 **−0.60 ~ +0.50 rad** ✓

## 5. ★ 裁判系统门控（本轮新增的重点）

| 情况 | 行为 |
|---|---|
| 裁判系统**在线**且 `Game_Progress == 4`（比赛中）| 允许发弹 ✓ |
| 裁判系统**在线**但不在比赛中 | **禁发弹**（`Shoot_Enable=false`、`Shoot_Rate=0`）✗ |
| 裁判系统**离线** | 按"允许"处理 ✓ —— **练习/调试时没有裁判系统也能打** ✓ |
| `Chassis_Power_Limit` | 从 `0x0201` 取，放进 `Cmd_Data_t` 给底盘任务限功率 ✓（0 = 没拿到）|

门控结果也在 `Cmd_Data_t` 里：`Referee_Online` / `Referee_Shoot_Allowed` ✓

## 6. API

| 接口 | 说明 |
|---|---|
| `Cmd_Init()` | 清零（`Robot_Init()` 里调）|
| `Cmd_Task()` | 跑一拍（CMD 任务 500 Hz 调）|
| `Cmd_Get()` | 读当前指令 `const Cmd_Data_t*` |
| `Cmd_Get_Mode()` / `Cmd_Get_Mode_Name()` | 模式 / 模式名 |
| `Cmd_Get_Seq()` | 指令序号（每拍 +1）|
| `CMD_SRC_xxx` | 本拍指令来源位（RC / KEYBOARD / HOST / REFEREE）|

**单位约定**：`Chassis_Vx/Vy` = m/s，`Chassis_W` = rad/s，`Gimbal_Yaw/Pitch` = rad（相对角）✓
底盘任务自己换算成四轮转速 ✓

## 7. 坑

1. **`Cmd_Task()` 拥有 `Rc_Task()`/`Referee_Task()`** ✗ ⇒ 别在 `Robot_Task()` 或别处再调（会重复推进 / 重复发布 ✓）
2. **云台是增量式**（鼠标每次读数是一个增量）✗ ⇒ 不推鼠标时角度**保持不变** ✓（这是对的 ✓ 不是 bug ✓）
3. **IDLE = 全零** ✓ ⇒ 想"保持不动"就是它 ✓；拨杆打到下、或遥控器掉线都会进 IDLE ✓
4. **`Chassis_Power_Limit` 只在 `0x0201` 里** ✓ ⇒ 裁判系统离线时它是 0 ⇒ 底盘别拿它当 0 瓦 ✗（0 = 没限制 ✓）
5. 摇杆**通道顺序**：ch0=右左右，ch1=右上下，ch2=左左右，ch3=左上下 ✓（`dev_rc.md` 第 5 节 ✓）

## 8. 上板自检

```
① 上电不接遥控器:Cmd_Get_Mode() == IDLE,Chassis_Vx/Vy/W 全 0
② 开遥控器、S1 打到上 ⇒ 模式变 REMOTE;推右杆上下 ⇒ Chassis_Vx 跟着变(±3 内)
③ S1 打到下 ⇒ 立刻回 IDLE(全零)
④ S2 打到上 ⇒ Chassis_Spin == true;打到下 ⇒ Gimbal_Enable == false
⑤ 鼠标左右 ⇒ Gimbal_Yaw 累加;按 F ⇒ 归零
⑥ 接裁判系统、不在比赛 ⇒ Shoot_Enable 一直 false;比赛开始(Game_Progress==4)后可置 true
⑦ Msg_Get_Published(MSG_TOPIC_CMD) 应稳定 ~100/s
```
