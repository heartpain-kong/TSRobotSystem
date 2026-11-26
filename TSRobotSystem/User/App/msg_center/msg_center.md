# msg_center — 任务间通信：环形数组

文件：`msg_center.h` / `msg_center.cpp`

## 1. 为什么用环形数组

流程图里每个任务都要**读好几个别的任务的数据**（cmd → robot/chassis/gimbal/highRise，
chassis → gimbal/highRise …）。直接互相持有对方的全局变量会变成一团乱麻，所以：

```
生产者任务 ──Msg_Publish()──► 该话题的环形数组 ──Msg_Latest()──► 任意多个消费者任务
                              (静态分配,4 条,每条 ≤64B)          (各自记 seq,互不影响)
```

* **静态、无 malloc** ✗（参考工程 `basic_framework` 的 `message_center` 用链表 + `malloc` ✗，
  在机器人上跑久了会碎堆；我们改成编译期定长的环形数组 ✓）
* **不阻塞**：满了就**覆盖最旧的** ✓（控制任务永远只关心最新一帧 ✓）
* **短临界区**：只有 `__disable_irq()` 保护下标 + memcpy（µs 级）✓

## 2. 话题表（与 `E:\rm\流程图.jpg` 一一对应）

| 话题 | 谁发 | 谁收（对应流程图的线）|
|---|---|---|
| `MSG_TOPIC_RC` | `dev_rc`（遥控器/键鼠）| **cmd** |
| `MSG_TOPIC_MINIPC` | 上位机（USB/UART）| **cmd** |
| `MSG_TOPIC_REFEREE` | `dev_referee`（裁判系统）| **cmd** + robot |
| `MSG_TOPIC_CMD` | **cmd** | robot / chassis / gimbal / highRise |
| `MSG_TOPIC_IMU_CHASSIS` | 底盘 imu | chassis / gimbal |
| `MSG_TOPIC_IMU_GIMBAL` | 云台 imu | gimbal |
| `MSG_TOPIC_CHASSIS` | chassis | gimbal / highRise / error |
| `MSG_TOPIC_GIMBAL` | gimbal | highRise / error |
| `MSG_TOPIC_HIGHRISE` | highRise | error |
| `MSG_TOPIC_MOTOR` | 电机/设备 | error |
| `MSG_TOPIC_ROBOT` | **robot**（决策层）| chassis / gimbal / highRise |

## 3. 两类消费者（★ 这是最容易用错的地方）

**① 广播型**（一个话题多个消费者，例如 `cmd` 要给 4 个任务都看）
每个消费者**自己**记一个 `seq`，互不干扰：

```cpp
static uint32_t my_seq = 0;
Cmd_Data_t cmd;

if (Msg_Is_New(MSG_TOPIC_CMD, my_seq)) {
    if (Msg_Latest(MSG_TOPIC_CMD, &cmd, sizeof(cmd), &my_seq)) {
        /* 用 cmd 里的新指令 */
    }
}
```

**② 独占型**（一条数据只该被消费一次，例如记录/日志）

```cpp
My_Data_t d;
while (Msg_Pop(MSG_TOPIC_MOTOR, &d, sizeof(d))) { /* 逐条处理 */ }
```

> ⚠️ **广播型话题绝不能用 `Msg_Pop`** ✗ —— pop 会把它从队列里拿走，别的任务就读不到了 ✗

## 4. API

| 接口 | 说明 |
|---|---|
| `Msg_Init()` | 清空所有话题（`Robot_Init()` 里最早调用）|
| `Msg_Publish(topic, data, len)` | 发布（`len ≤ 96`，超了拒收并计数）|
| `Msg_Latest(topic, out, size, &seq)` | 取**最新**一条（不弹出）|
| `Msg_Is_New(topic, seq)` | 有没有比 `seq` 更新的数据 |
| `Msg_Pop(topic, out, size)` | 取**最旧**一条并弹出（独占型）|
| `Msg_Get_Seq / Published / Overwritten` | 诊断：序号 / 累计发布 / 被覆盖条数 |
| `Msg_Get_Topic_Name(topic)` | 话题名（日志用）|

## 5. 配置（改 `msg_center.h`）

```c
#define MSG_RING_DEPTH   4u     // 每话题留几条(改小省 RAM:每条 = 64B)
#define MSG_SLOT_SIZE    96u    // 单条最大长度(IMU 帧 ~92B)
```
> RAM 占用 = `话题数(10) × MSG_RING_DEPTH × MSG_SLOT_SIZE` = 现在 **3.8 KB** ✓
> 紧张就把它降到 `2`（省 1.25 KB）✓

## 6. 坑

1. **长度必须 ≤ `MSG_SLOT_SIZE`** ✗ —— 超了直接拒收（`Overwritten` 会 +1，便于发现协议写错）✓
2. **`Overwritten` 增长是正常的** ✓ —— 广播话题本来就一直在被新数据覆盖；它不是"丢数据" ✗
3. 结构体要**定长、无指针** ✗（跨任务传指针没有意义 ✓）
4. ISR 里也能 `Publish`（临界区很短 ✓），但**建议只在任务里发** ✓，ISR 只把数据存下来 ✓

## 7. 上板自检

```
① Msg_Init() 之后 Msg_Get_Published(MSG_TOPIC_CMD) == 0
② 发一次:Msg_Publish(MSG_TOPIC_CMD, &d, sizeof(d)) ⇒ Published==1, Seq==1
③ 两个消费者各自 Latest 都能拿到同一份数据(互不影响)✓
④ Msg_Is_New(MSG_TOPIC_CMD, seq) 在没新数据时返回 false ✓
⑤ 连发 10 条 ⇒ Published==10,Overwritten==6(深度 4)✓,Latest 拿到的是第 10 条 ✓
```
