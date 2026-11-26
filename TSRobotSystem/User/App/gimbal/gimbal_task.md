# gimbal_task — 云台任务（yaw/pitch 关节角控制 + 小陀螺前馈）

文件：`gimbal_task.h` / `gimbal_task.cpp`
任务：**500 Hz（2 ms）**、优先级 `osPriorityNormal`、栈 2048 B（`robot_def.h` 里改）

---

## 1. 它在流程图里的位置

```
MSG_TOPIC_CMD          ─┐
MSG_TOPIC_IMU_GIMBAL   ─┼─► Gimbal_Task() ─┬─► 写电机(MIT 位置:关节侧 rad,不用换算)
MSG_TOPIC_CHASSIS      ─┘                  ├─► MSG_TOPIC_GIMBAL ─► highRise / error
                                           └─► error_task 关键数据快照(Gimbal_Cmd/Gimbal_Now)
```
> 流程图里 `chassis → gimbal` 这条线，就是用来做**小陀螺前馈**的 ✓（见第 4 节）

## 2. 每拍做什么（500 Hz）

```
① 收 CMD / IMU_GIMBAL / CHASSIS(广播式:Msg_Is_New + Msg_Latest)
② dt ← TIM5(小陀螺前馈要)
③ 小陀螺前馈:底盘在自转 ⇒ 关节目标反向补偿,尽量保持世界朝向
④ 目标角 = 指令 + 前馈,再按限位夹紧
⑤ 写电机:motor_task_set_mit(idx, 目标角, 0, 0, Kp, Kd)
     ★ 云台是**关节角**控制,电机接口就是关节侧 rad ⇒ **不需要任何单位换算** ✓
     不使能(Gimbal_Enable == false)⇒ 目标 = 当前角(锁住,别让云台自由倒)
⑥ 发布 GIMBAL + 写 error 快照
```

## 3. ★ 电机分配（集中在 `robot_def.h`）

```c
#define CHASSIS_MOTOR_0     0u      // 底盘 4 轮
...
#define GIMBAL_MOTOR_YAW    0xFFu   // 0xFF = 未分配
#define GIMBAL_MOTOR_PITCH  0xFFu
```
* 现在注册表只有 **2 台达妙**（`motor_task.cpp` 的 `DM_MOTOR_N = 2`）⇒ 底盘占 0/1、**云台默认未分配** ✓
* 云台未分配时：本任务照常发布状态（`Axis_Count = 0`），但**不写任何电机** ✓ ⇒ 不会和底盘抢同一路 ✓
* 接上云台电机后：把 `GIMBAL_MOTOR_YAW/PITCH` 改成实际下标 + 把 `DM_MOTOR_N` 加大 + 在 `motor_task.cpp` 注册对应电机 ✓

## 4. 小陀螺前馈（为什么用底盘里程计而不是云台 IMU）

小陀螺时底盘在自转，云台想**保持世界朝向**就得反向转 ✓。
补偿量用 **底盘的里程计角速度 `Chassis_Data_t.W`** 积分得到：

```c
s_data.Spin_Comp -= GIMBAL_SPIN_COMP_GAIN * chas.W * dt;
```
> ⚠️ **不能**用云台 IMU 的 yaw 变化量 ✗ —— 那里面同时含"底盘带着转"和"操作手自己推鼠标转"两部分，
> 会把操作手的输入也抵消掉 ✓（我第一版就想这么写，已改掉 ✓）

`GIMBAL_SPIN_COMP_GAIN`：1 = 完全反向补偿，0 = 关闭 ✓

## 5. 参数（★ 按实车改）

| 宏 | 值 | 说明 |
|---|---|---|
| `GIMBAL_YAW_MIN/MAX` | ∓2.967 rad | yaw 限位（±170°）|
| `GIMBAL_PITCH_MIN/MAX` | −0.60 / +0.50 rad | pitch 限位 |
| `GIMBAL_KP` / `GIMBAL_KD` | 30.0 / 1.0 | MIT 刚度/阻尼（位置误差 → 力矩）|
| `GIMBAL_SPIN_COMP_GAIN` | 1.0 | 小陀螺前馈增益 |

## 6. 发布的数据 `Gimbal_Data_t`

| 字段 | 单位 | 说明 |
|---|---|---|
| `Target[2]` | rad | 关节目标 [yaw, pitch] |
| `Now[2]` | rad | 关节实测（电机反馈）|
| `Imu_Yaw_Total` | rad | 云台 IMU 连续 yaw |
| `Imu_Pitch` | rad | 云台 IMU pitch |
| `Spin_Comp` | rad | 小陀螺前馈量 |
| `Kp` / `Kd` | — | 当前用的刚度/阻尼 |
| `Axis_Count` / `Enable` / `Online` / `Seq` | — | 状态 |

## 7. API

| 接口 | 说明 |
|---|---|
| `Gimbal_Init()` | 清零（`Robot_Init()` 里调）|
| `Gimbal_Task()` | 跑一拍（GIMBAL 任务 500 Hz 调）|
| `Gimbal_Get()` | 读 `const Gimbal_Data_t*` |
| `Gimbal_Is_Online()` | IMU 在线 |
| `Gimbal_Get_Target(axis)` | 某轴目标角 |

## 8. 坑

1. **云台用关节侧 rad，不需要 rpm 换算** ✗ ⇒ 别照抄底盘的 `set_wheel_rpm` ✓（那是给轮子的）
2. **指令里的 `Gimbal_Yaw/Pitch` 是"相对关节角"** ✓ ⇒ 直接当关节目标用 ✓；限位在本任务里夹 ✓
3. **不使能时锁在当前角** ✗ ⇒ 别改成"零力矩"（云台会自由倒下 ✓）
4. **小陀螺前馈必须退出时归零** ✓（本任务在 `chas.Spin == false` 时把 `Spin_Comp` 清零 ✓）
5. **`0xFF` = 未分配** ✓ ⇒ 用 `Axis_Motor_Ok()` 判断，别直接拿它当下标 ✗

## 9. 上板自检

```
① 接上云台电机前:Axis_Count == 0,发布照常,底盘不受影响 ✓
② 接上后:Axis_Count == 2;遥控器使能 ⇒ Target 跟着指令变,Now 跟上(无明显静差)
③ 不使能 ⇒ Target == Now(锁住,推不动但也不会倒)
④ 底盘小陀螺开:云台 yaw 应反向补偿(世界朝向基本不变);关掉 ⇒ Spin_Comp 立刻归零
⑤ pitch 推到极限 ⇒ 被限位挡住(−0.60 / +0.50 rad)
⑥ Msg_Get_Published(MSG_TOPIC_GIMBAL) ≈ 500/s
```
