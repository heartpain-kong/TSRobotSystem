# motor_task — 电机应用层(C++)

**位置**：`User/App/motor_task/`
**一句话**：持有电机实例、注册 CAN 接收、每拍写目标 → 算 PID → 按各自频率发帧；对 C 侧只暴露 `motor_task_init()` / `motor_task()`。

| 文件 | 说明 |
|---|---|
| `motor_task.h` | 纯 C 声明（`extern "C"`），C 文件可安全 include |
| `motor_task.cpp` | C++ 实现：实例 + 初始化 + 接收分发 + 跑一拍 |
| `motor_task.md` | 本文件 |

---

## 1. 当前接的电机

| 电机 | 总线 | 反馈 ID | 型号 | 关节侧减速比 | 当前模式 |
|---|---|---|---|---|---|
| 大疆 3508 舵向 | FDCAN1 | `0x201` | `MOTOR_DJ_M3508_BARE` | **4.0**（屁股 1:1 × 外部 1:4） | 速度 / 位置 |

- 达妙：`DM_MOTOR_N = 0`，代码用 `#if` 屏蔽，不占数组、不进回调。
- 宇树 485：`MOTOR_485_ENABLE = 0`（在 `robot_def.h`），代码整段编译掉。
- 所以 `motor_task_get_count()` 当前返回 **1**。

### 电机总开关

```cpp
#define MOTOR_TASK_ENABLE 1   // 0 = 系统里没有电机:不初始化、不驱动、不报电机错误码
```

为 0 时：`motor_task_init()` 不碰 DJI、`motor_task()` 不写目标/不发帧、`motor_task_get_count()` 返回 0，`error_task` 不会报 `20+/40+/60+`。

---

## 2. 数据流

```
3508 电调 ──(1kHz 自动反馈 0x201)──► FDCAN1 RX FIFO0
                                        │ RX FIFO0 中断
                                        ▼
                         bsp_fdcan: HAL_FDCAN_RxFifo0Callback
                                        ▼
                         CAN1_Callback()  (ISR,只解析)
                                        ▼
                         dj_motor.recv() → Now_Pos/W/T
                         dj_rx_us = 收到时刻

MOTOR 任务 1kHz
  motor_task()
    ├─ 写目标:速度 Set_W(rpm×π/30) / 位置 Set_Pos(rad)
    ├─ 到点:dj_motor.Calculate()  → out(电流刻度)
    └─      dj_motor.send()       → 0x200 广播帧
```

> ⚠️ **不要在 `motor_task()` 里轮询 `HAL_FDCAN_GetRxMessage()`**：接收已经由
> `bsp_fdcan` 的 FIFO0 中断分发；任务里再读会跟中断抢同一个 FIFO。

---

## 3. 配置（`motor_task.cpp` 顶部）

| 宏 | 当前值 | 含义 |
|---|---|---|
| `MOTOR_TASK_ENABLE` | `1` | 电机总开关 |
| `DM_MOTOR_N` | `0` | 达妙台数（0 = 不编译 DM 相关代码） |
| `MOTOR_ONLINE_TIMEOUT_US` | `100000` | 100ms 没反馈 = 离线 |
| `DJ_GEAR_RATIO` | `4.0` | 3508 屁股 1:1 × 外部 1:4 |
| `DJ_KP` / `DJ_KI` | `1.0` / `0.1` | 速度环，输出量纲是**关节力矩** |
| `DJ_STEER_KP` / `DJ_STEER_KD` | `5.0` / `0.2` | 位置环（输出 = 速度目标） |
| `DJ_STEER_MAX_W` | `10.0` | 位置环输出的速度上限 [rad/s] |

> 上电默认目标 `g_dj_target_rpm = 0`，不会自己转。台架试转显式调用：
> `motor_task_set_target_rpm(100)`（当前是速度模式）。
> `DJ_KP=1.0` 偏激进，第一次上电建议先降到 `0.05~0.2`。

---

## 4. `motor_task_init()`

```
① dj_motor.Init(&hfdcan1, 0x201, M3508_BARE, motor_speed, 1000Hz)
② Set_Gearbox_Rate(4.0)
③ PID_speed.Init(DJ_KP, DJ_KI, …)
④ PID_Angle.Init_PD(DJ_STEER_KP, DJ_STEER_KD, DJ_STEER_MAX_W) + 角度模式
⑤ bsp_can_init(&hfdcan1, CAN1_Callback)   ← 注册回调 + Start + 开 RX FIFO0 中断
⑥ dj_motor.enable()                        ← 只置命令位；之后 send() 持续发帧
```

顺序原因：**先 Init 对象，再启动总线/开中断**，避免第一帧反馈在对象配置好之前进回调。

---

## 5. `motor_task()` 每拍

```
① 速度模式：Set_W(target_rpm × π/30)
   位置模式：Set_Pos(target_rad)
② if (Is_Send_Due()) { Calculate(); send(); }
```

- `Calculate()`：大疆是“电调只吃电流”，主机算速度/位置环，把结果写进 `out`（电流刻度）。
- `send()`：把 `out` 打进 `CAN_Tx_Data[总线][组][槽位]`，发 `0x200`；
  - 命令位 DISABLE 时**仍继续发**，但强制 `out=0`（大疆没有失能帧，停发会让电调保持上一拍电流）；
  - TX FIFO 满时不 `Mark_Sent()`，下一拍重试，并累加 `Tx_Fail`。

---

## 6. API

| 接口 | 说明 |
|---|---|
| `motor_task_init()` | 初始化 + 注册接收 + 使能（不建任务） |
| `motor_task()` | 跑一拍；MOTOR 任务 1kHz 调 |
| `motor_task_get_count()` | 台数（当前 1；总开关为 0 时 0） |
| `motor_task_is_init_ok(i)` | 是否收到过反馈（`dj_rx_us != 0`） |
| `motor_task_is_online(i)` | 100ms 内是否收到过反馈 |
| `motor_task_get_m_error(i)` | 故障码（DJI 反馈里没有，恒 0） |
| `motor_task_get_name(i)` | `"CAN1-DJ-3508"` |
| `motor_task_set_wheel_rpm(i, rpm, kd)` | 切速度模式 + 写目标（kd 保留不用） |
| `motor_task_get_wheel_rpm(i)` | 实测转速，换算成 rpm |
| `motor_task_get_now_w(i)` | 实测转速 [rad/s，关节侧] |
| `motor_task_get_now_pos(i)` | 实测位置 [rad，关节侧] |
| `motor_task_get_now_torque(i)` | 实测力矩 [N·m，关节侧] |
| `motor_task_set_steer_rad(rad)` | 切位置模式 + 写角度目标 |
| `motor_task_get_steer_rad()` | 实测关节角 [rad] |
| `motor_task_set_mit(i, pos, w, t, kp, kd)` | 只缓存位置/速度目标；`t/kp/kd` 忽略 |
| `motor_task_set_target_rpm(rpm)` | 只改速度目标，不切模式 |
| `motor_task_get_target_rpm()` | 读速度目标 |

### 单位

- 对外底盘/调试：**rpm**（`set_wheel_rpm` / `get_wheel_rpm`）。
- 电机接口：**关节侧 rad、rad/s、N·m**。
- 换算只在 `motor_task.cpp` 做一次：`rpm × MATH_RPM_TO_RADPS`、`rad/s ÷ MATH_RPM_TO_RADPS`。

---

## 7. 坑

1. **`Motor_Status` 不能当反馈标志**  
   大疆没有状态回读，`enable()/lose()` 会把命令值镜像进 `Motor_Status`；  
   判断“是否收到反馈”用 `dj_rx_us`（`is_init_ok` / `is_online` 已经这么做）。

2. **底盘还在写 0 号电机**  
   `chassis_task` 每 2ms 调 `motor_task_set_wheel_rpm(0, …)`，会覆盖速度目标；  
   台架单独试舵向时，先把 `CHASSIS_MOTOR_0` 改成 `0xFF` 或停掉底盘任务。

3. **位置模式要重新调参**  
   `DJ_STEER_KP/KD` 只是默认值，实车必须按“转向到位、不抖、不甩轮”重新调。

4. **速度环输出是力矩量纲**  
   `DJ_KP=1.0` 时 1 rad/s 误差 ≈ 1 N·m 关节力矩，100 rpm 起步会直接顶限幅。  
   第一次上电先用小目标（如 10~20 rpm）+ 小 Kp。

5. **TX 失败要可诊断**  
   `dj_motor.Get_Tx_Fail_Count()` 持续增长 ⇒ TX FIFO 满/总线没起来；  
   当前没接进遥测，需要时自己加一列。

6. **达妙/485 是“预留路径”**  
   `DM_MOTOR_N > 0` 时才会编译 DM 代码，但 `Initialize` 里还没有逐台 `Init`（接实物时要补）；
   485 需要 `MOTOR_485_ENABLE=1` 并确认串口/帧长。

---

## 8. 上板自检

```
① 不接电机:error_task 报 20(未收到反馈)，LED/蜂鸣器有对应码；电机不转
② 接电机、上电:dj_rx_us 开始涨，is_init_ok/online 变 true，20/40 消失
③ motor_task_set_target_rpm(20) ⇒ 电机低速转；get_wheel_rpm 跟随
④ 拔 CAN:100ms 内 is_online 变 false ⇒ 报 40
⑤ 位置模式:motor_task_set_steer_rad(0) / (1.57) ⇒ 轮子转向、无来回抽动
⑥ Tx_Fail 不持续增长；增长 ⇒ 查总线/TX FIFO
```
