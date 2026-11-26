# RobotSlutionCalculation — 双 MCU 运动学/动力学与 CAN3 协议

本目录不依赖 HAL/RTOS，上 MCU 和下 MCU 都可以直接编译。

```text
robot_solution.h/.cpp   D2Q2 逆运动学、正运动学、功率估算
can3_protocol.h/.cpp    CAN3 8 字节帧编解码
```

---

# 一、双 MCU 分工

## 上 MCU

```text
遥控器 DBUS
上位机 USB CDC
裁判系统 USART1
4310 pitch（CAN ID 1）
两个摩擦轮
CAN3 与下 MCU 通信
自带 IMU
```

负责：

```text
遥控/上位机/裁判输入
pitch 控制
摩擦轮控制
整车 yaw 目标
CAN3 下发底盘速度
```

## 下 MCU

```text
底盘 4 个 D 轮（2 舵向 + 2 驱动）
yaw 6020（0x205）
拨盘 3508（0x206）
CAN3 与上 MCU 通信
自带 IMU
```

负责：

```text
底盘运动学
舵向/驱动闭环
yaw 6020 位置环
拨盘速度环
CAN3 回传状态
```

下 MCU **不接遥控器**，所有驾驶输入通过 CAN3 从下 MCU 拿。

---

# 二、CAN3 物理层

```text
FDCAN3
经典 CAN
1 Mbps
标准帧
8 字节
```

上 MCU 和 CAN3 收发器共地，终端电阻 120Ω。

---

# 三、CAN3 帧

所有多字节字段小端。

## 0x300 上 MCU → 下 MCU：底盘速度

| 字节 | 内容 |
|---|---|
| 0 | seq |
| 1 | mode：bit0 enable，bit1 spin，bit2 trigger |
| 2~3 | int16 vx [mm/s] |
| 4~5 | int16 vy [mm/s] |
| 6~7 | int16 w [mrad/s] |

周期：

```text
2ms（500Hz）
```

CAN3 总线任务按 1kHz 运行，0x300 每 2ms 发一帧。

下 MCU 超过：

```text
100ms
```

没收到 0x300：

```text
vx=vy=w=0
进入安全停止
```

## 0x301 下 MCU → 上 MCU：底盘反馈（500Hz）

用途：上 MCU 做速度闭环、里程计、打滑判断。

| 字节 | 内容 |
|---|---|
| 0 | seq |
| 1 | status：bit0 online，bit1 fault，bit2 imu |
| 2~3 | int16 vx [mm/s] |
| 4~5 | int16 vy [mm/s] |
| 6~7 | int16 w [mrad/s] |

## 0x302 上 MCU → 下 MCU：yaw / 拨盘

| 字节 | 内容 |
|---|---|
| 0 | seq |
| 1 | flags |
| 2~3 | int16 yaw 目标 [0.01°] |
| 4~5 | int16 拨盘 rpm |
| 6~7 | 预留 |

## 0x303 下 MCU → 上 MCU：状态（100Hz）

| 字节 | 内容 |
|---|---|
| 0 | seq |
| 1 | status |
| 2~3 | int16 yaw 实际 [0.01°] |
| 4~5 | int16 功率 [W] |
| 6~7 | int16 fault |

用途：

```text
yaw 实际值 -> 上 MCU 航向闭环
功率       -> 上 MCU 120W 限幅
fault      -> 上 MCU 显示/降级
```

status 位：

```text
bit0 下 MCU 在线
bit1 底盘故障
bit2 下 MCU IMU 有效
bit3 四台底盘电机在线
bit4 拨盘正在转
bit5 6020 在线
```

---

# 四、运动学

## 逆运动学

输入：

```text
vx, vy, w
```

每个轮：

```text
vx_i = vx - w*y_i
vy_i = vy + w*x_i
```

输出：

```text
steer_i = atan2(vy_i, vx_i)
rpm_i   = hypot(vx_i, vy_i) / R * 30/π
```

实际工程里还要：

```text
零位偏置
90° 翻转
轮速上限
舵向到位后才给驱动
```

## 正运动学

由两个 D 轮的实测：

```text
steer_rad_i
wheel_rpm_i
```

反推：

```text
vx
vy
w
```

下 MCU 用这个回传 0x301。

## 功率估算

```text
P = Σ |ω_i * T_i| / η + P_idle
```

用于 120W 限幅。

---

# 五、上 MCU 驾驶逻辑

```text
遥控器 xy
→ 期望车体 vx/vy
→ 期望 yaw
→ yaw 闭环修正 w
→ CAN3 0x300 下发
```

yaw=0：

```text
上电时 IMU EKF yaw 从 0 开始
```

没有磁力计，所以是相对航向。

---

# 六、下 MCU 执行逻辑

```text
CAN3 0x300
→ 超时判断
→ D2Q2 逆运动学
→ 舵向 202/204
→ 驱动 201/203
```

同时：

```text
0x302 yaw → 6020 0x205
0x302 trigger → 206 拨盘
```

---

# 七、协议代码用法

上 MCU 发送：

```cpp
Can3_Cmd cmd = {};
cmd.seq = seq++;
cmd.mode = 0x01;
cmd.vx_mm_s = (int16_t)(vx * 1000.0f);
cmd.vy_mm_s = (int16_t)(vy * 1000.0f);
cmd.w_mrad_s = (int16_t)(w * 1000.0f);

uint8_t data[8];
Can3_EncodeCmd(&cmd, data);
fdcan_send_data_stand(&hfdcan3, CAN3_ID_UPPER_CMD, data, 8);
```

下 MCU 接收：

```cpp
Can3_Cmd cmd;
if (Header.Identifier == CAN3_ID_UPPER_CMD)
{
    Can3_DecodeCmd(Buffer, &cmd);
}
```

---

# 八、CAN3 回调划分

上 MCU：

```text
0x301 -> 底盘反馈
0x303 -> 下 MCU 状态
```

下 MCU：

```text
0x300 -> 底盘速度
0x302 -> yaw / 拨盘
```

两边各自调用：

```cpp
bsp_can_init(&hfdcan3, Can3_Callback);
```

---

# 九、后续要补

```text
1. 4310 pitch 型号/反馈解析
2. 两个摩擦轮 CAN ID 与方向
3. 上 MCU 的 yaw 闭环与下 MCU 底盘反馈联动
4. 下 MCU 6020 yaw 零位偏置
5. 断链/超时安全停车
```
