# chassis_task — 底盘任务（麦轮：指令 + IMU → 四轮目标 + 里程计）

文件：`chassis_task.h` / `chassis_task.cpp`
任务：**500 Hz（2 ms）**、优先级 `osPriorityNormal`、栈 2048 B（`robot_def.h` 里改）

---

## 1. 它在流程图里的位置

```
MSG_TOPIC_CMD          ─┐
MSG_TOPIC_IMU_CHASSIS  ─┴─► Chassis_Task() ─┬─► 写电机目标(motor_task_set_wheel_rpm)
                            Class_chassis_M4  ├─► MSG_TOPIC_CHASSIS ─► gimbal / highRise / error
                            (麦轮解算+里程计) └─► error_task 关键数据快照(底盘指令 + 实际)
```

## 2. 每拍做什么（500 Hz）

```
① 收 CMD(广播:Msg_Is_New + Msg_Latest)       ← 遥控器/裁判系统给的指令
② 收 IMU_CHASSIS                              ← 连续 yaw
③ 小陀螺"上升沿" ⇒ s_m4.Set_Top_Yaw(yaw)      ★ 记下启动那一刻的 yaw 作 0 位
④ 反解:x/y/w(m/s,m/s,rad/s) → 四轮 rpm
     小陀螺 ⇒ count_run2(xyv, yaw)(内部按 yaw-top_yaw 旋到机体系)
     普通   ⇒ count_run1(xyv)
⑤ 读实测轮速(电机反馈 rad/s → rpm)+ forward_count() 正解里程计
⑥ 限功率:Power_Est = Σ|实测力矩|×|实测转速|,超裁判系统上限就按比例回退速度指令
⑦ 写电机:motor_task_set_wheel_rpm(i, rpm*scale, kd)   ← 内部做 rpm → rad/s
⑧ 发布 CHASSIS + 写 error 快照(Chassis_Cmd / Chassis_Now)
```

## 3. ★ 单位约定（统一在一处换算）

| 环节 | 单位 |
|---|---|
| `Cmd_Data_t.Chassis_Vx/Vy/W` | **m/s、m/s、rad/s** |
| `Class_chassis_M4` 输入 `X_Y_W[3]` | 同上 ✓ |
| `Class_chassis_M4` 输出 `Get_Speed(i)` | **rpm**（`|V|·cos/r × 30/π`）|
| `motor_task_set_wheel_rpm(i, rpm, kd)` | 收 **rpm** ⇒ 内部 ×`MATH_RPM_TO_RADPS` ⇒ 关节侧 rad/s |
| 电机接口(`Set_W` / `Get_Now_W`) | **关节侧 rad/s**（按《电机使用说明.md》✓）|

⇒ **换算只在 `motor_task` 里做一次** ✓，底盘任务全程用 rpm ✓

## 4. 参数（★ 按实车改，都在 `chassis_task.cpp` 顶部）

```c
#define CHASSIS_HALF_A      0.30f   // 半轴距 a/2 [m]
#define CHASSIS_HALF_B      0.30f   // 半轮距 b/2 [m]
#define CHASSIS_WHEEL_R     0.06f   // 轮半径 [m]
#define CHASSIS_WHEEL_KD    1.0f    // MIT 速度控制阻尼(轮子用 kp=0 + kd)
#define CHASSIS_POWER_KP    0.5f    // 限功率回退快慢
static chassis_M_wheel_init s_m4_init[4] = { {CHASSIS_A,{+1,+1}}, ... };  // 辊子方向 + 方位
static const uint8_t s_motor_index[4] = {0,1,2,3};                        // 轮 → 电机下标
```

> ⚠️ 宏名注意：轮子方位用的是枚举 `CHASSIS_A` / `CHASSIS_B`（辊子方向），
> 几何参数叫 `CHASSIS_HALF_A/B` —— **别撞名** ✗（我一开始就撞了 ✗）

## 5. 现在的接线现状（重要）

* 当前实车：**0 号大疆 3508 舵向**，`motor_task_get_count() == 1`；
  本任务仍用 `Class_chassis_M4` 解算，所以 0 号电机被当作“麦轮 0 号轮”在写。
* 台架单独试舵向时，要先把 `CHASSIS_MOTOR_0` 改成 `0xFF`（或停掉底盘任务），
  否则底盘每 2ms 会覆盖 `motor_task_set_wheel_rpm(0, …)`。

### 2 舵轮 + 2 全向轮（D Q / Q D）

* 解算类 `Class_chassis_D2Q2` 已加在 `alg_chassis_control.h/.cpp`；
* `chassis_task.cpp` 里已放好 `s_d2q2` / `s_d2q2_init` 的预留配置，并在 `Chassis_Init()` 里初始化；
* **但 `Chassis_Task()` 目前仍走 `s_m4`**，D2Q2 还没有接入控制回路。
* 启用前需要：
  1. 2 个转向 + 2 个驱动电机（共 4 台）注册进 `motor_task`；
  2. 每拍把两个 D 轮的实测转向角喂给 `s_d2q2.Set_Seat_Now()`；
  3. 用 `s_d2q2.count_run1/count_run2` 替换 `s_m4` 的反解；
  4. 用 `s_d2q2.forward_count()` 替换 `s_m4.forward_count()`。
* Q 轮只承重，**不需要电机/编码器**，也不参与解算。

## 6. 发布的数据 `Chassis_Data_t`

| 字段 | 单位 | 说明 |
|---|---|---|
| `Vx` / `Vy` / `W` | m/s, m/s, rad/s | **里程计正解**的车体速度 |
| `Wheel_Speed[4]` | rpm | 各轮目标（已含限功率回退）|
| `Wheel_Now_Speed[4]` | rpm | 各轮实测 |
| `Wheel_Seat[4]` | rad | 舵轮转向目标（麦轮恒 0）|
| `Yaw_Total` | rad | 本拍用的连续 yaw |
| `Top_Yaw` | rad | 小陀螺零位 |
| `Power_Est` / `Power_Limit` | W | 估计功率 / 裁判系统上限 |
| `Mode` / `Spin` / `Unit_Count` / `Online` / `Seq` | — | 状态 |

## 7. API

| 接口 | 说明 |
|---|---|
| `Chassis_Init()` | 初始化解算层（`Robot_Init()` 里调）|
| `Chassis_Task()` | 跑一拍（CHASSIS 任务 500 Hz 调）|
| `Chassis_Get()` | 读 `const Chassis_Data_t*` |
| `Chassis_Is_Online()` | IMU 在线 + 至少一路电机可用 |
| `Chassis_Get_Wheel_Rpm(i)` | 某路目标轮速 |
| `Chassis_Set_Top_Yaw_Now()` | 手动对齐小陀螺零位（调试）|

## 8. 坑

1. **小陀螺零位只在"上升沿"对齐一次** ✗ ⇒ 一直按着不会反复重置 ✓；要重新对齐就用 `Chassis_Set_Top_Yaw_Now()` ✓
2. **速度符号靠 `s_m4_init` 的 `wheel_xy`** ✗ ⇒ 某个轮子反转，先查这张表（不是改电机线序）✓
3. **限功率是"按实测力矩估计"的近似** ✗ —— 不是裁判系统的真实功率；要更准得用裁判系统的功率缓冲/电流反馈 ✓
4. **`Msg_Latest` 是广播式** ✓ ⇒ 别用 `Msg_Pop`（会把指令从环形里拿走，别的任务就没了 ✗）
5. 电机没初始化成功（`is_init_ok == false`）时本任务**不写那一路** ✓ ⇒ 现象是"某个轮子不动"，看 `Unit_Count` 和错误码 20+n ✓

## 9. 上板自检

```
① 上电:Chassis_Get()->Unit_Count 应等于已初始化成功的电机数;Online == true
② 遥控器推到"非 IDLE"模式 + 推杆 ⇒ Wheel_Speed 随杆量变(符号正确 = 车往对应方向走)
③ 小陀螺(S2 上):Top_Yaw 变成当前 yaw;车开始自转但平移方向仍按遥控器 ✓
④ 手动转车:Vx/Vy/W 应能读出来(推着车走,符号与方向一致)
⑤ 某轮堵转:Power_Est 上升 ⇒ 超过 Power_Limit 时 Wheel_Speed 会被回退
⑥ Msg_Get_Published(MSG_TOPIC_CHASSIS) ≈ 500/s
```
