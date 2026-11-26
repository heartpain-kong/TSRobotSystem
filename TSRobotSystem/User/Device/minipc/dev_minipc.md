# dev_minipc — miniPC 通信（USB CDC 虚拟串口）

**下位机侧链路：** `USB` → **`minipc`** → **`cmd`**

> 完整链路：上位机（操作手/视觉主机）`──USB──►` **miniPC** `──USB(CDC 虚拟串口)──►` **STM32**
> STM32 侧:`CDC_Receive_HS` → `MiniPC_Rx_Callback()`(只搬运) → `MiniPC_Task()`(组帧/校验) → `MSG_TOPIC_MINIPC` → **cmd**
> 因此本模块的**对端是 miniPC** ✓（它把上位机/视觉的指令按第 2 节帧格式转发过来 ✓）

文件：`dev_minipc.h` / `dev_minipc.cpp`
任务：无独立任务 ✓ —— 由 **CMD 任务**（500 Hz）调 `MiniPC_Task()` 推进 ✓

---

## 1. 链路

```
上位机 ──USB──► STM32 OTG_HS ──► USB CDC 类驱动
                                      │ CDC_Receive_HS()(中断)  ← 只搬运
                                      ▼
                              dev_minipc: 256B 接收缓存(ISR 追加)
                                      │ MiniPC_Task()(500Hz) ← 组帧 + 校验 + 掉线判定
                                      ▼
                       Msg_Publish(MSG_TOPIC_MINIPC) ──► cmd 任务(自动模式用它的指令)
```
* USB 本身由 CubeMX 的 `MX_USB_DEVICE_Init()` 初始化 ✓（在 `freertos.c` 的 `StartDefaultTask` 里 ✓）
* 接收入口就一行，写在 `USB_DEVICE/App/usbd_cdc_if.c` 的 `USER CODE BEGIN 11` 段里 ✓：
  ```c
  MiniPC_Rx_Callback(Buf, (uint16_t)(*Len));   /* 中断里:只搬运 */
  ```
  > ⚠️ 该文件里 `USBD_CDC_ReceivePacket(&hUsbDeviceHS);` **必须是活的** ✗ —— 被注释掉的话只能收到**第一包** ✓（本工程已确认是活的 ✓）

## 2. 帧格式（和裁判系统同一框架 ⇒ 上位机/操作手端可共用工具链）

```
SOF(0xA5) | data_length(2) | seq(1) | CRC8(1) | cmd_id(2) | data(n) | CRC16(2)
                data_length = n + 4 ;帧总长 = data_length + 5
CRC8 : 字节 0~3    (poly 0x31, init 0xFF)  ← User/algorithm/crc/crc8.h(与 dev_referee 共用 ✓)
CRC16: 字节 5~末尾-2(CRC-CCITT)            ← User/algorithm/crc/crc_ccitt.h
```

| cmd_id | 方向 | 内容 |
|---|---|---|
| `0x0001` | 上位机 → 机器人 | 控制指令（`MiniPC_Cmd_Payload_t`，24 B）|
| `0x0002` | 上位机 → 机器人 | 视觉目标（预留）|
| — | 机器人 → 上位机 | 用 `MiniPC_Send(buf, len)` 自己发 ✓ |

`MiniPC_Cmd_Payload_t`（小端，和 `Cmd_Data_t` 字段对齐）：

| 字段 | 类型 | 单位 |
|---|---|---|
| `Mode` | u8 | `Enum_Cmd_Mode` |
| `Shoot_Enable` / `Shoot_Rate` | u8 | — |
| `Reserved` | u8 | 填充（对齐用）|
| `Chassis_Vx` / `Chassis_Vy` | f32 | m/s |
| `Chassis_W` | f32 | rad/s |
| `Gimbal_Yaw` / `Gimbal_Pitch` | f32 | rad（相对关节角）|

## 3. 在 cmd 里怎么用

```c
MiniPC_Task();                                     // 推进(组帧/校验/掉线)
if (Msg_Is_New(MSG_TOPIC_MINIPC, s_minipc_seq) &&
    Msg_Latest(MSG_TOPIC_MINIPC, &host, sizeof(host), &s_minipc_seq))
{
    s_cmd.Src |= CMD_SRC_HOST;
    if ((s_cmd.Mode == CMD_MODE_AUTO) && host.Online) { /* 覆盖底盘/云台/发射目标 */ }
}
```
> ★ **只有遥控器拨杆打到 AUTO（右拨杆 S1 = 中）时，上位机指令才生效** ✓
> 这是**有意设计** ✓：调试/比赛时随时能一巴掌拨回遥控 ✓（不会被上位机抢走控制权 ✗）

## 4. API

| 接口 | 说明 |
|---|---|
| `MiniPC_Init()` | 清状态（`Robot_Init()` 里调）|
| `MiniPC_Task()` | 周期跑（CMD 任务 100 Hz 调）|
| `MiniPC_Rx_Callback(buf, len)` | **USB 中断入口**（只搬运，别在这里做别的 ✗）|
| `MiniPC_Get()` | 读解析结果 `const MiniPC_Data_t*` |
| `MiniPC_Is_Online()` | 在线（200 ms 没帧 = 掉线）|
| `MiniPC_Send(data, len)` | 回传给上位机（USB 忙时返回 false）|

## 5. 坑

1. **USB CDC 是字节流** ✗ —— 一次回调可能是半帧，也可能是好几帧粘在一起 ✓ ⇒ 必须**缓存 + 组帧** ✓（本模块就是这么做的 ✓），不能"一次回调解一帧" ✗
2. **`MiniPC_Rx_Callback` 在中断上下文** ✗ ⇒ 里面只做 memcpy + 长度更新（带 PRIMASK 临界区 ✓）
3. **环形缓存是 256 B** ✓ ⇒ 单帧 ≤ 127 B + 4 ✓；上位机发太快/发垃圾时满了会**清空重来**并计 `s_rx_overflow` ✓
4. **掉线判定 200 ms** ✓（比遥控器的 100 ms 宽松，因为上位机是"按需发指令"✓）
5. **`MiniPC_Data_t` 必须 ≤ `MSG_SLOT_SIZE`(96)** ✗ ⇒ 已加 `static_assert` ✓
6. **USB 没插/没枚举时 `CDC_Transmit_HS` 返回 BUSY/FAIL** ✓ ⇒ `MiniPC_Send` 返回 false，**绝不阻塞** ✓

## 6. 上板自检

```
① 插上 USB:电脑应出现一个虚拟串口(VID/PID 见 usbd_desc.c);minipc_init_ok == true
② 用串口助手按第 2 节格式发一帧(0x0001):minipc_rx 开始涨,MiniPC_Is_Online() == true
③ Crc8_Errors / Crc16_Errors 应为 0(不为 0 ⇒ 校验范围写错 ✓)
④ Reject_Count 涨 ⇒ 帧长/帧头不对(多半是 data_length 算错:它**从 seq 开始算** ✓)
⑤ 遥控器拨到 AUTO + 上位机发速度指令 ⇒ 底盘跟着走;拨回遥控 ⇒ 立刻听遥控器的 ✓
⑥ 拔掉 USB ⇒ 200 ms 内 MiniPC_Is_Online() 变 false,Lost_Count +1;插回自动恢复 ✓
⑦ Msg_Get_Published(MSG_TOPIC_MINIPC) 跟着 minipc_rx 涨(不是每拍都发 ✓)
```
