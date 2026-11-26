# highRise_task — 末端机构任务（默认：发射/摩擦轮+拨弹；预留：升降/抬升）

文件：`highRise_task.h` / `highRise_task.cpp`
任务：**500 Hz（2 ms）**、优先级 `osPriorityNormal`、栈 2048 B（`robot_def.h` 里改）

---

## 1. ★ 这个任务是 RC / RM 的"app 层差异点"

整套系统 **ROBOCON(RC) 与 RoboMaster(RM) 共用** ✓，末端机构是两边最不一样的地方，
所以把差异收成 **`robot_def.h` 里的一个宏** ✓：

```c
#define HIGHRISE_MECH_SHOOT     0u      // 发射:2 摩擦轮 + 1 拨弹(默认,RM)
#define HIGHRISE_MECH_LIFT      1u      // 升降/抬升(接口已预留,RC 用)
#define HIGHRISE_MECH           HIGHRISE_MECH_SHOOT
```
* **RM**：不用改 ✓（默认就是发射）
* **RC**：把宏改成 `HIGHRISE_MECH_LIFT` ✓ + 按需改 `#elif` 那段分支 ✓ —— 其余任务/设备/通信层**一行不动** ✓

## 2. 它在流程图里的位置

```
MSG_TOPIC_CMD      ─┐
MSG_TOPIC_CHASSIS  ─┼─► HighRise_Task() ─┬─► 写电机(摩擦轮:速度 / 拨弹:位置 / 升降:位置)
MSG_TOPIC_GIMBAL   ─┘                    └─► MSG_TOPIC_HIGHRISE ─► error
```
> 流程图里 `chassis → highRise`、`gimbal → highRise` 两条线，本任务已经**订阅好**了 ✓
> （现在只做联锁占位，等你有"底盘在动时不许发弹/云台异常不许发弹"的规则，直接在这里加判断 ✓）

## 3. 每拍做什么（发射模式）

```
① 收 CMD + CHASSIS + GIMBAL(广播式)
② 热量门控:Heat_Blocked = !使能 || !裁判允许 || (热量 + 余量 ≥ 上限)
      ⚠️ Heat_Limit == 0 表示"没拿到裁判数据" ⇒ **不拦**(练习/调试能打)✓
③ 摩擦轮(左/右):纯速度控制,使能就转到 HIGHRISE_FRICTION_RPM(默认 6000rpm)
④ 拨弹盘:0 停(锁在当前格)/ 1 单发(每次上升沿走一格 + 卡弹保护)/ 2 连发(定速)
⑤ 发布 HIGHRISE
```

## 4. 升降模式（预留，`HIGHRISE_MECH_LIFT`）

```
Lift_Target 斜坡限制地加/减(每拍 ≤ HIGHRISE_LIFT_RATE)⇒ 位置控制(丝杆/升降电机)
目标暂时借用指令的发弹档位字段(1 上 / 2 下)—— 以后换成真正的升降指令即可 ✓
限位:HIGHRISE_LIFT_MIN ~ HIGHRISE_LIFT_MAX
```
> 需要加减速限幅就自己乘一个斜坡因子（原先的 `User/algorithm/Slope` 没被用到，已删除）

## 5. 电机分配（`robot_def.h`，0xFF = 未分配）

```c
#define HIGHRISE_MOTOR_FRICTION_L   0xFFu
#define HIGHRISE_MOTOR_FRICTION_R   0xFFu
#define HIGHRISE_MOTOR_TRIGGER      0xFFu
#define HIGHRISE_MOTOR_LIFT         0xFFu
```
* 默认全未分配 ⇒ 本任务照常发布状态（`Friction_Count = 0`）但**不写任何电机** ✓（不会和底盘/云台抢 ✓）
* 接上电机后：改这几个宏 + `DM_MOTOR_N` + 在 `motor_task.cpp` 注册 ✓

## 6. 发布的数据 `HighRise_Data_t`

| 字段 | 单位 | 说明 |
|---|---|---|
| `Mech` | — | 机构类型 |
| `Friction_Target/Now[2]` | rpm | 摩擦轮左/右 目标/实测 |
| `Trigger_Pos/Target` | rad | 拨弹盘实测/目标角 |
| `Trigger_Rate` | — | 0 停 / 1 单发 / 2 连发 |
| `Shot_Count` | 格 | 累计发弹数 |
| `Lift_Target/Now` | rad | 升降目标/实测（预留）|
| `Heat` / `Heat_Limit` / `Heat_Blocked` | ℃/— | 热量与门控结果 |
| `Enable` / `Online` / `Seq` | — | 状态 |

## 7. API

| 接口 | 说明 |
|---|---|
| `HighRise_Init()` | 初始化（`Robot_Init()` 里调；会记录拨弹盘初始角当零点）|
| `HighRise_Task()` | 跑一拍（HIGHRISE 任务 500 Hz 调）|
| `HighRise_Get()` | 读 `const HighRise_Data_t*` |
| `HighRise_Is_Online()` | 是否就绪 |
| `HighRise_Get_Shot_Count()` | 累计发弹数 |

## 8. 坑

1. **`Heat_Limit == 0` 不能当成"热量上限为 0"** ✗ ⇒ 那是"裁判系统没数据"的意思，要**放过**（否则练习时打不了 ✓）
2. **摩擦轮走 `set_wheel_rpm`** ✓（它就是个轮子 ✓），**拨弹盘走 `set_mit`** ✓（要走格 = 位置控制 ✓）—— 别混用 ✗
3. **单发用"上升沿"** ✓ ⇒ 一直按着不会连发 ✓（要连发就用档位 2 ✓）
4. **卡弹保护**：目标与实测差 > `HIGHRISE_ANTI_JAM_RAD` 就不再往前走 ✓（防止把弹盘拧坏 ✓）
5. **发弹门控链是"与"关系** ✓：指令使能 && 裁判允许 && 热量有余量 ✓ —— 想知道被谁拦了看 `Heat_Blocked` + `Enable` + `Cmd.Get()->Referee_Shoot_Allowed` ✓

## 9. 上板自检

```
① 不接电机:Friction_Count == 0,发布照常,其他任务不受影响 ✓
② 接上后:按 Shift ⇒ 摩擦轮 Friction_Now 升到 ~6000rpm;松开 ⇒ 掉到 0
③ 按 Q(单发)⇒ Trigger_Target 每次 +1 格(1.047 rad),Shot_Count +1;按住不重复发
④ 按 E(连发)⇒ 拨弹盘定速转,Shot_Count 按格数增长
⑤ 手动堵住拨弹盘 ⇒ Trigger_Target 最多领先 ANTI_JAM_RAD(不继续加)
⑥ 接裁判系统、不在比赛 ⇒ Heat_Blocked == true,弹盘不动;比赛中热量满 ⇒ 同样被拦
⑦ Msg_Get_Published(MSG_TOPIC_HIGHRISE) ≈ 500/s
```
