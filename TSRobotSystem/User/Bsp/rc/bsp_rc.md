# bsp_rc — DBUS 遥控帧接收（协议层）

文件：`bsp_rc.h` / `bsp_rc.cpp`　层：**Bsp**（协议级，不含业务语义）

## 1. 它是什么 / 为什么在 BSP 层

DBUS 就是"**UART5 上的一个协议**"：固定 18 字节帧、11 位打包的通道、鼠标/键盘位图。
这些和 `bsp_uart` / `bsp_fdcan` 是同一类东西 —— **协议/外设级**，所以放 BSP ✓
好处：别的工程要用遥控器，把这个目录 + `bsp_uart` 拿走就行 ✓（不需要 Device/App ✓）

> **它不管**：在线/掉线、归一化、按键语义、发布消息 ✓（那些是 `dev_remote` 的事 ✓）

## 2. 硬件

| 项 | 值 |
|---|---|
| 串口 | **UART5**（`huart5`）|
| 引脚 | **PD2 = RX**（PC12 = TX，不用）|
| 参数 | **100000 baud / 8 数据位 / 偶校验 / 2 停止位**（CubeMX 里就是这个 ✓）|
| 接收 | `bsp_uart` 的 **DBM + IDLE 引擎**（`SBUS_RX_BUF_NUM = 18` 已预留 ✓）|

## 3. 帧格式（18 字节）

```
[0..1]  ch0(11b)   [1..2] ch1   [2..4] ch2   [4..5] ch3   [16..17] ch4
[5]     高 4 位 = 拨杆 S1(bit4-5) / S2(bit6-7)
[6..13] 鼠标 x,y,z(各 2B) + 左/右键
[14..15] 键盘位图(16 键,见 RC_KEY_xxx)
```
通道值 **0~2047，中位 1024** ⇒ 本层已**减去 1024**（约 −660 ~ +660）✓

> ⚠️ **第 16/17 字节属于 ch4** ✗ —— 它 **不是** SBUS 那种 failsafe 标志字节 ✓
> （照抄别人的代码时最容易在这里误判，现象是"ch4 稍大就报掉线" ✓）

## 4. API

| 接口 | 说明 |
|---|---|
| `BSP_RC_Init(&huart5)` | 注册接收回调（传 NULL 用默认 `huart5`）|
| `BSP_RC_Get()` | 读最近的**原始帧** `const Bsp_Rc_Frame_t*` |
| `BSP_RC_Get_Rx_Count()` | 累计收帧数 |

`Bsp_Rc_Frame_t` = `Ch[5]`、`Switch[2]`、`Mouse_X/Y/Z`、`Mouse_Press_L/R`、`Key`、`Last_Rx_Ms`、`Rx_Count` ✓

## 5. 坑

1. **回调在 ISR 里** ✗ ⇒ 只解析这一帧，**不要**在这里判在线/发消息/打印 ✓
2. **一个串口只能注册一种回调** ✗ ⇒ 上层（`dev_remote`）不要再调 `USART_init(&huart5, ...)` ✓
3. **`Bsp_Rc_Frame_t` 的字段顺序被 `dev_remote` 当"前缀"用** ✗ ⇒ 改字段时要同步看 `dev_remote.cpp` 的 `memcpy` ✓
4. 掉线判定**不在这里** ✗ —— 本层只提供 `Last_Rx_Ms` 与 `Rx_Count`，语义由上层决定 ✓

## 6. 上板自检

```
① 不接遥控器:BSP_RC_Get_Rx_Count() 一直是 0
② 开遥控器:Rx_Count 以 ~14ms 一帧的速度增长
③ 推右杆上下:BSP_RC_Get()->Ch[1] 在 -660~+660 之间变化,回中 ≈ 0
④ 拨杆:Switch[0]/[1] 随拨动变 1/2/3
⑤ 按 W/Shift:Key 的对应位变 1
```
