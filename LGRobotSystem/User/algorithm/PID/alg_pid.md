# alg_pid — `Class_PID`：位置式 PID（一个类覆盖 PD / PI / PID）

## 1. 定位

电机控制环的通用 PID。

* 大疆 / 灵足的**主机闭环**、以及所有"主机侧位置环 / 速度环"都用它。
* 达妙 / 宇树 / 灵足走 **MIT 透传**时**不需要它** —— 那种情况下 PD 在电机固件里，主机只下发 Kp/Kd。

| 文件 | 内容 |
|---|---|
| `alg_pid.h` | 接口：`Enum_PID_Improve`、`Class_PID` 声明 + 文件底部的「inline 实现」一节（一行式 setter/getter）|
| `alg_pid.cpp` | 算法：`Init()`、`Init_PD()`、`Cout()`、`Clear()` —— **想看懂 PID 只读这一个文件** |

## 2. 三种用法

| 用法 | 初始化 | 说明 |
|---|---|---|
| 位置环 **PD** | `Init_PD(kp, kd, max_out, d_alpha)` | Ki = 0，自动打开「微分先行 + D 项低通」，位置环标准配置 |
| 速度环 **PI** | `Init(kp, ki, 0.0f, 0.0f, max_out, max_iout)` | 无微分、无前馈；建议再 `Add_Improve(PID_Improve_Anti_Windup)` |
| 完整 **PID** | `Init(kp, ki, kd, 0.0f, max_out, max_iout)` | 优化环节**默认全开**（微分先行 + D 低通 + 抗饱和）；想要最朴素的用 `Set_Improve(PID_Improve_NONE)` |

```cpp
p.Init_PD(kp, kd, max_out, 0.3f);
p.Set_Target(ref);
p.Set_Now(fdb);
p.Cout();
float u = p.Get_Out();
// 调参时看分量： Get_Pout() / Get_Iout() / Get_Dout() / Get_Fout()
```

## 3. 计算方法（位置式）

```
out = Pout + Iout + Dout + Fout
Pout  = Kp * e                  e = Target - Now
Iout += Ki * e                  （每拍累加，受 max_iout 限幅；抗饱和打开时饱和拍不积）
Dout  = Kd * d                  d 的来源见下面两个开关
Fout  = KF * Target + FF        KF = 设定值前馈，FF = 外部每拍注入的前馈
```

**Ki / Kd 是"每拍增益"，不看 dt**（和 RM 的写法一致）：调用周期固定时直接调即可
（1 kHz 任务里，`Ki` 就是每秒积 1000 次）。要按"每秒"给，自己乘上周期。

各项都能单独读出来（`Get_Pout` / `Get_Iout` / `Get_Dout` / `Get_Fout`），方便调参时看谁在发力。

## 4. ★ 优化环节位掩码 `Enum_PID_Improve`（已精简为 3 个）

| 宏 | 值 | 作用 |
|---|---|---|
| `PID_Improve_NONE` | `0x00` | 不加任何额外环节（默认）|
| `PID_Improve_Derivative_On_Measurement` | `0x01` | **微分先行**：只对反馈微分，目标阶跃时 D 不突变（`Init_PD` 自动开）|
| `PID_Improve_Derivative_Filter` | `0x02` | **D 项一阶低通**：压反馈噪声（`Init_PD` 自动开）|
| `PID_Improve_Anti_Windup` | `0x04` | **抗积分饱和**：上一拍输出被限幅过 → 这一拍不积分 |
| **`PID_Improve_DEFAULT`** | **`0x07`** | **默认值 = 三个全开**（`Improve` 成员的初值）。Ki = 0 时抗饱和是空操作，不影响 PD；加了 Ki 正好防"冲出去回不来" |

用法：`Set_Improve(位或)` / `Add_Improve(位)` / `Del_Improve(位)` / `Get_Improve()`。

### 之前删掉了哪 5 个环节，为什么

原来堆了 9 个开关：输出低通、误差死区、积分分离、变速积分、梯形积分。
**全工程没有任何调用者**（只有 `Init_PD` 开了"微分先行 + D 项低通"）✗，
却让 `Cout()` 变成 8 个条件分支、类里多出 6 个只服务于它们的成员（`Outfilter`、
`Out_Alpha`、`DeadBand`、`Separation_Threshold`、`CI_A/CI_B`、`Dbuf[3]` …），
还顺带把 `Dbuf[3]`（只写不读）和 `error[2]`（只挪不用）留下了。

⇒ 已删除，代码规模 **519 行 → 260 行**（`h` 321→150、`cpp` 198→110）。

### 想加回某个环节（以"误差死区"为例）

1. `alg_pid.h` 的枚举加一项：`PID_Improve_DeadBand = 0x08,`
2. 类里加字段 + 接口：`fp32 DeadBand = 0.0f;` 与 `Set_DeadBand()/Get_DeadBand()`
3. `alg_pid.cpp::Cout()` 里在误差后面加两行：
   ```cpp
   fp32 err = error[0];
   if ((Improve & PID_Improve_DeadBand) && (DeadBand > 0.0f) && (fabsf(err) < DeadBand))
       err = 0.0f;
   ```
   然后把后面用到的 `error[0]` 换成 `err` 即可（P 和 I 都用它）。

"输出低通"照此在求和之后、"积分分离/变速积分"在 I 那一步、"梯形积分"把 `Ki * error[0]`
换成 `Ki * (error[0] + error[1]) * 0.5f`。

## 5. 关键函数

### 初始化与计算

| 函数 | 作用 | 注意 |
|---|---|---|
| `Init(__KP, __KI, __KD, __KF, __max_out, __max_iout)` | 完整初始化 | `max_out` / `max_iout <= 0` = **不限制**；只清运行状态，**不动** `Improve`（所以 `Set_Improve` 可以先调）|
| `Init_PD(__KP, __KD, __max_out, __d_alpha = 0.2f)` | 按 PD 初始化（位置环最常用）| 内部 = `Init(kp, 0, kd, 0, max_out, 0)` + `Add_Improve(微分先行 | D 低通)` + 设 `D_Alpha`；**只开不关**，默认开的抗饱和会保留；想兜静差之后 `Set_KI(小值)` 即可 |
| `Cout()` | 跑一拍 | 先 `Set_Target` / `Set_Now`（要前馈再 `Set_Feedforward`）再调它 |
| `Clear()` | 清积分 / 误差历史 / D 低通 / 饱和标志，并让**微分基准重新对齐** | 使能、切模式、目标大跳变时调一次 |

### 调参与开关

| 函数 | 说明 |
|---|---|
| `Set_Target` / `Get_Target`、`Set_Now` / `Get_Now` | 目标 / 反馈 |
| `Set_KP/KI/KD/KF` + `Get_*` | 四个系数（`Ki = 0` 即 PD）|
| `Set_Max_out` / `Get_Max_out` | 对称输出限幅（`<= 0` = 不限制）；设置时同时复位不对称限幅为 ±它 |
| `Set_Max_iout` / `Get_Max_iout` | 积分项限幅（`<= 0` = 不限制）|
| `Set_Out_Limit(__min, __max)` | 不对称限幅（"正转多、反转少"时用）|
| `Set_Improve` / `Get_Improve` / `Add_Improve` / `Del_Improve` | 优化环节位掩码 |
| `Set_D_Filter_Alpha` / `Get_D_Filter_Alpha` | D 项低通系数（0~1，推荐 0.1~0.3）|
| `Set_Feedforward` / `Add_Feedforward` / `Get_Feedforward` | 外部前馈，**每拍注入一次、不会自动清零** |

### 读各项分量

| 函数 | 返回值 | 说明 |
|---|---|---|
| `Get_Out()` | `fp32` | 本拍总输出（已过限幅）|
| `Get_Pout()` / `Get_Iout()` / `Get_Dout()` / `Get_Fout()` | `fp32` | 各项分量 |
| `Get_Error()` | `fp32` | 最近一拍的误差（`Target - Now`）|
| `Is_Saturated()` | `bool` | 上一拍输出是否被限幅削掉过（抗积分饱和的依据）|

## 6. 注意事项

* **单位必须与反馈一致**：本工程统一用关节侧 `rad` / `rad·s⁻¹` / `N·m`。
* 使能 / 切模式时先 `Clear()` 清积分与滤波残值 —— 它同时会把微分基准重新对齐，
  避免"电机当前不在零位"时第一次微分造出一个假速度、使能瞬间弹一下。
* `max_out` / `max_iout <= 0` 表示**不限制**。
* 抗积分饱和**默认就是开的**（`PID_Improve_DEFAULT`）—— 加 `Ki` 时不用再手动 `Add_Improve`；
  想要"和最早版本完全一致、不带任何额外环节"的 PID：`Set_Improve(PID_Improve_NONE)`。
* 位置环 PD 的标准配置是「微分先行 + D 项低通」：前者避免目标阶跃在 D 项上踢出巨大尖峰
  （电机瞬间给大电流），后者滤掉编码器 / 转速反馈的量化噪声。

## 7. 相关

* lg_pid.cpp（算法实现，**推荐先读它**）
* `math_support.h`（`LimitMax`、`LimitMinMax`）
* `User/Device/motor/motor_DJ/`（目前唯一的调用者：`PID_Angle` / `PID_speed`）
* `User/Device/motor/电机使用说明.md`（电机层怎么用这些环）
