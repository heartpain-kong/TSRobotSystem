# motor_DJ — 大疆（DJI）M2006 / M3508 / GM6020 的 `motor_mit` 实现（含“一拖四”组封装）

把大疆的 CAN 电流环电机接进统一电机层。关键约束是大疆**一条广播帧管 4 台**：
同一总线上 `0x201~0x204` 共 `0x200`、`0x205~0x208` 共 `0x1FF`、`0x209~0x20B` 共 `0x2FF`，
所以单台对象不能各发各的：发送缓冲按 `[总线][组][字节]` 组织（`motor_DJ.cpp` 里的
`CAN_Tx_Data[3][3][8]`），再由组对象保证“4 台的值装进同一帧、只发一次”。

## 怎么用

### ① 单台（调试 / 一台一组）

```cpp
static Class_Motor_DJ m;                 // ⚠️ 别放栈上
m.Init(&hfdcan1, CAN_M1_ID, MOTOR_DJ_M3508, motor_speed, Motor_Control_Freq_1000HZ);
m.enable();
m.Set_W(10.0f);
m.Calculate();                           // 算 PID，并把 out 写进本组帧缓冲
m.send();                                // 本组广播帧发出去（含同组其它台当前值）
```

### ② 成组（推荐）：一个对象初始化同组 4 台，一帧发出

```cpp
static Class_Motor_DJ_Group chassis;     // ⚠️ ≈1.5KB（4 个 Class_Motor_DJ），别建在栈上
const uint16_t ids[4] = {CAN_M1_ID, CAN_M2_ID, CAN_M3_ID, CAN_M4_ID};
chassis.Init(&hfdcan1, ids, MOTOR_DJ_M3508, motor_speed, Motor_Control_Freq_1000HZ);

chassis.PID_Calculate_All();                 // 4 台一起算，各自写进本组帧
if (chassis.Is_Send_Due()) chassis.Send();   // 到点 → 整组同一帧发一次
```

## 型号与减速比（含“3508 屁股”）

| 型号 | 用途 | `Gearbox_Rate` |
|---|---|---|
`MOTOR_DJ_M2006` | M2006 + C610 | 36.0 |
`MOTOR_DJ_M3508` | 原装（带 19.2 减速箱） | 19.2 |
`MOTOR_DJ_M6020` | GM6020 直驱 | 1.0 |
**`MOTOR_DJ_M3508_BARE`** | **3508 屁股**（拆掉减速箱，1:1） | **1.0** |

“屁股”= 去掉 19.2 减速箱、转子直接输出。**电调 / 编码器 / 电流常数完全不变**，只是输出轴不再减速。

齿比支持运行时覆盖（屁股再挂外部减速时用）：

```cpp
m.Init(&hfdcan1, CAN_M1_ID, MOTOR_DJ_M3508_BARE, motor_speed, Motor_Control_Freq_1000HZ);
m.Set_Gearbox_Rate(4.0f);      // 屁股 1:1 × 外部 1:4 = 4.0
```

### 齿比只在电机层生效一次

```c
Now_Pos = Total_Encoder / 8192 * 2π / Gearbox_Rate     // ÷
Now_W   = rpm * π/30 / Gearbox_Rate                     // ÷
Now_T   = 电流 / CURRENT_TO_OUT * CURRENT_TO_TORQUE * Gearbox_Rate   // ×
out     = T / CURRENT_TO_TORQUE / Gearbox_Rate * CURRENT_TO_OUT      // 指令反算
```

⇒ **对外返回的已经是关节侧值**，上层（chassis / 算法）**不要再除或乘一次** ✗

编码器 **8192 计数/圈（转子侧）**：转子 4 圈 = `Total_Encoder` 32768 ⇒ 关节 2π ✓

### 扭矩常数

```c
CURRENT_TO_TORQUE = 0.3f / (3591.0f / 187.0f);   // = 0.01562 N·m/A（转子侧）
CURRENT_TO_OUT    = 16384.0f / 20.0f;            // = 819.2 raw/A
```

`3591/187 = 19.2032` 是 M3508 精确齿比；`0.3 N·m/A` 是**原装减速后**的常数，除齿比得转子侧常数。
齿比统一由 `Gearbox_Rate` 表达，所以同一套常数可覆盖三种用法：

| 用法 | 关节侧扭矩常数 | 20 A 时 |
|---|---|---|
原装 3508 | 0.01562 × 19.2 = **0.30 N·m/A** | 6 N·m |
3508 屁股 | 0.01562 × 1 = **0.0156 N·m/A** | 0.31 N·m |
屁股 + 外部 1:4 | 0.01562 × 4 = **0.0625 N·m/A** | 1.25 N·m ⚠️ |

> ⚠️ 屁股做舵向时力矩很小（比 6020 直驱低约 6 倍）⇒ 重点验证**静止能否把轮子拧到位**；
> 不够就加大外部减速比，或提高电流限幅。

### 核对方法

手转轮子一圈，`motor_task_get_now_pos()` 应变化 **6.283 rad**（转子实际转 4 圈）。
只变 1.57 ⇒ 多除了一次；变 25.1 ⇒ 少除了一次。

## 注意

* **失能 = 槽位写 0，绝不能靠“停发”来停机**：大疆没有失能指令帧，收不到帧它会保持上一次
  电流。所以 `lose()` / `Group::Lose()` 都是“把 `out` 清零 → 主动发一帧 0”。
* **一帧一帧地发**：逐台调 `obj.send()` 会把同组其它台的槽位按各自相位重复发（一帧变最多
  4 帧）；成组必须走 `Group::Send()` 只发一帧。
* 大疆反馈里**没有模式字段**，`Motor_Status` 只是命令值的镜像；`send()` 不看 `Motor_Status`，
  命令位 `Send_Status_Cmd=DISABLE` 时**继续发但强制 `out=0`**（不能停发）。
* **单位**：对外是关节侧 `rad` / `rad·s⁻¹` / `N·m`；`out` 是**转子侧电流刻度**（档位：
  2006 → ±10000@0.18N·m、3508 → ±16384@0.3N·m、6020 → ±16384@0.741N·m），
  换算在 `PID_Calculate_Data()` 里做。
* 编码器 8192/圈，圈数由 delta 过零累加得到；**`zero()` 只清偏移，不清 `Total_Round`**。
* `Init` 的 4 个 ID 必须同总线、同组，否则 `Group::Init()` 整体失败（不会初始化一半）。

## 类型与常量

| 名称 | 值 / 内容 |
|---|---|
| `MOTOR_DJ_UNIT_MAX` | `4u`（组内最多 4 台） |
| `motor_DJ_Model` | `MOTOR_DJ_M2006` / `MOTOR_DJ_M3508` / `MOTOR_DJ_M6020`（GM6020） |
| 组广播帧 id | `CAN_M1_M4_ID = 0x200`、`CAN_M5_M8_ID = 0x1FF`、`CAN_M9_MA_ID = 0x2FF` |
| 单台反馈 id | `CAN_M1_ID = 0x201` … `CAN_M4_ID = 0x204`、`CAN_M5_ID = 0x205` … `CAN_M8_ID = 0x208`、`CAN_M9_ID = 0x209` … `CAN_MB_ID = 0x20B` |
| `Struct_recv_motor_DJ` | `Pre_Encoder`（上次编码器值）/ `Total_Encoder`（累计编码器值）/ `Total_Round`（累计圈数） |
| 发送缓冲（`motor_DJ.cpp`，文件级 static） | `CAN_Tx_Data[3][3][8]`，索引 = `[总线][组][字节]`；总线 FDCAN1/2/3 → 0/1/2 |

## 接口一览

### `Class_Motor_DJ`（单台）

| 接口 | 作用 | 备注 |
|---|---|---|
| `Init(const Struct_motor_init &, motor_DJ_Model)` | 统一入口初始化 | 频率取 `_motor_init.Control_Freq`（传 0 时基类周期保持默认 1000us） |
| `Init(hfdcan, __id, __model, __control_mode, __freq)` | 品牌版初始化 | 定好型号 / 模式 / 频率 |
| `send()` | 发一帧控制量 | **每拍都发**；命令位 DISABLE 时强制 `out=0`；TX 成功才 `Mark_Sent()` |
| `Calculate()` | 单台闭环：算 PID + 写本组帧缓冲 | 单台 App 在 `send()` 前调；组用 `PID_Calculate_All()` |
| `recv(_id, _data, _len)` | 解析一帧反馈 | `_id == 0` 表示不校验 id；`_len < 7` 直接丢 |
| `can_recv(const uint8_t *data)` | 反馈原始解析 | 编码器 / 转速 / 电流 / 温度；**不改 `Motor_Status`** |
| `can_send_Data()` | 把 `out` 写入本组帧缓冲 | **只写缓冲，不发总线** |
| `send_group_frame()` | 发本组广播帧 | 返回 bool；失败计入 `Tx_Fail` |
| `PID_Calculate_Data()` | 按控制模式算 PID | 命令位 DISABLE 时只写 0；内含 `can_send_Data()` |
| `zero()` / `enable()` / `lose()` | 置零 / 允许发帧 / 停输出 | `lose()` 先发一帧 0，并清 PID/目标 |
| `Get_CAN_Id()` | 本台 CAN 标准帧 id | 接收分发时用 |
| `Get_Tx_Fail_Count()` | 发送失败累计 | TX FIFO 满/总线没起来时增长 |
| `Get_Now_pos()` / `Get_Temperature()` | 读位置 / 温度 | 等价于基类的 `Get_Now_Pos()` / `Get_Motor_Temperature()` |
| `PID_Angle` / `PID_speed` | 位置环 / 速度环 | `Class_PID`，来自 `alg_pid.h` |

### `Class_Motor_DJ_Group`（一组，最多 4 台）

| 接口 | 作用 | 备注 |
|---|---|---|
| `Init(hfdcan, ids[4], model, mode, freq)` | 一次初始化 4 台 | 同一总线、同一组、型号 / 模式相同；`ids[i] == 0` 视为空位 |
| `Init(hfdcan, idx, id, model, mode, freq)` | 逐台初始化 | 想混型号 / 混模式时用；频率以**第一台**为准 |
| `operator[](idx)` | 取第 idx 台（0~3） | 单台的 set/get/PID 都从这儿走 |
| `Get_Unit_Count()` / `Is_Used(idx)` | 挂了几台 / 第 idx 台是否已初始化 | — |
| `PID_Calculate_All()` | 4 台一起算 PID | 各自的控制模式可以不同 |
| `Send()` | **一帧发出** | 组内每台的值都在同一帧里，只发一次，并给 4 台都 `Mark_Sent()` |
| `Is_Send_Due()` | 到点才发（只读） | 用组内第一台已初始化的电机代表整组；`Send()` 会给每台都记时刻 |
| `Set_Control_Freq(freq)` | 设置整组发送频率 | 转发给每台，保持各台一致 |
| `Enable()` / `Lose()` / `Zero()` | 整组使能 / 失能 / 标零 | 失能只发**一帧** 0 电流 |
| `Get_Group_Id()` / `Get_Group_Index()` / `Get_FDCAN()` / `Get_Tx_Frame()` | 本组帧 id / 组号 / 总线 / 这一帧的 8 个字节 | `Get_Tx_Frame()` 诊断用 |

## 相关

* 依赖：`motor_mit.h`（基类）、`alg_pid.h`（`PID_Angle` / `PID_speed`）、`bsp_fdcan.h`
  （`fdcan_send_data_stand`）、`math_support.h`（`MATH_RPM_TO_RADPS`）。
* 被 App 的电机注册 / 运控任务使用。
* 协议与坑的逐条细节（帧 id、失能 = 写 0、组对象尺寸等）见 `motor_DJ.h` 里的
  `Class_Motor_DJ_Group` 类注释与 `motor_DJ.cpp` 内的注释，
  以及 `motor_mit.h` 的“命令状态 vs 实际状态”一节。
