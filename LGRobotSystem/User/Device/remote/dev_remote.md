# dev_remote — 遥控器/键鼠设备（语义层）

文件：`dev_remote.h` / `dev_remote.cpp`　层：**Device**
任务：无独立任务 ✓ —— 由 **CMD 任务**（500 Hz）调 `Remote_Task()` 推进 ✓

## 1. 它在数据流里的位置

```
UART5 ─► bsp_rc(协议层:解析成原始帧) ─► dev_remote(本模块:在线判定 + 对外接口)
                                              │
                                              └─► Msg_Publish(MSG_TOPIC_RC) ─► cmd 任务
```
**分工**：`bsp_rc` 管"帧怎么解" ✓；本模块管"设备在不在线、上层怎么用" ✓

## 2. 每拍做什么

```
① 从 BSP 层取原始帧 ⇒ 一次 memcpy 搬进自己的结构(前缀完全一致,见 .h 注释)
② 掉线判定:超过 REMOTE_LOST_TIMEOUT_MS(默认 100ms) 没帧 ⇒ Lost = true(进一次 Lost_Count+1)
③ 只有收到**新帧**才 Msg_Publish(MSG_TOPIC_RC) ⇒ 不会白刷环形数组
```

## 3. 发布的数据 `Remote_Data_t`

| 字段 | 单位 | 说明 |
|---|---|---|
| `Ch[5]` | — | 杆量,已减 1024(−660~+660);0=右左右 1=右上下 2=左左右 3=左上下 4=拨轮 |
| `Switch[2]` | — | 左右拨杆 **1/2/3**（不是 0/1/2 ✗）|
| `Mouse_X/Y/Z` | 计数 | 鼠标增量/滚轮 |
| `Mouse_Press_L/R` | — | 左右键 |
| `Key` | 位图 | 键盘（`RC_KEY_xxx`）|
| `Last_Rx_Ms` / `Rx_Count` | ms / 次 | 最近一帧时刻 / 累计帧数 |
| `Lost` / `Lost_Count` | — | 当前是否掉线 / 掉过几次 |
| `Seq` | — | 每收到一帧 +1（消费者用它判断"有没有新数据" ✓）|

## 4. API

| 接口 | 说明 |
|---|---|
| `Remote_Init(&huart5)` | 清状态 + 让 BSP 层开始收帧（传 NULL 用默认 `huart5`）|
| `Remote_Task()` | 周期跑：掉线判定 + 发布 |
| `Remote_Get()` | 读数据 `const Remote_Data_t*` |
| `Remote_Is_Online()` / `Remote_Get_Rx_Count()` | 在线 / 累计收帧数 |
| `Remote_Get_Ch_Norm(i)` | 杆量归一化 **−1.0 ~ +1.0**（中位 0）|
| `Remote_Get_Switch(i)` | 拨杆 1/2/3（**0 = 不在线** ✓ 用它区分"没数据"）|
| `Remote_Key_Pressed(RC_KEY_W)` | 键盘某键是否按下 |

消费端（`cmd`）标准写法：
```cpp
static uint32_t seq = 0;
Remote_Data_t rc;
if (Msg_Is_New(MSG_TOPIC_RC, seq) && Msg_Latest(MSG_TOPIC_RC, &rc, sizeof(rc), &seq)) { /* 用 rc */ }
```

## 5. 坑

1. **不要再调 `USART_init(&huart5, ...)`** ✗ —— UART5 归 `bsp_rc` 管 ✓（一个口只能一个回调 ✓）
2. **`Remote_Data_t` 必须以 `Bsp_Rc_Frame_t` 为前缀** ✗ —— `.cpp` 里靠一次 `memcpy` 搬运 ✓
   （已加 `static_assert` 兜底 ✓；改结构体顺序会编译不过，这是故意的 ✓）
3. **`Lost` 的初值是 true** ✓ —— 收到第一帧之前都算掉线 ✓（上层据此走安全态 ✓）
4. **`Switch` 读到 0 表示"不在线"** ✓，不是"拨杆在 0 位" ✓
5. **掉线超时 100ms** ✓ —— 遥控器关机时**本来就不来帧** ✓，靠超时足够 ✓（别去读什么 failsafe 位 ✗）

## 6. 上板自检

```
① 不接遥控器:Remote_Init 返回 true;Remote_Is_Online() == false;Rx_Count == 0
② 开遥控器:Rx_Count 开始涨 → Online == true
③ 推杆:Remote_Get_Ch_Norm(0) 在 -1.0~+1.0、回中 ≈ 0
④ 拨杆:Remote_Get_Switch(0)/(1) 随拨动在 1/2/3 之间变
⑤ 关遥控器:100ms 内 Online 变 false、Lost_Count +1;开回来自动恢复
⑥ Msg_Get_Published(MSG_TOPIC_RC) 跟着 Rx_Count 涨(不是每拍都发 ✓)
```
