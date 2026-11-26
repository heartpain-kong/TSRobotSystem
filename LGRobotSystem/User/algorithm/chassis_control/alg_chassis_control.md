# alg_chassis_control — 底盘运动学（麦轮 M4 / 舵轮 D4）

文件：`alg_chassis_control.h` / `alg_chassis_control.cpp`
- `Class_chassis_M4` —— **麦轮**（M = mecanum）：速度符号由 `chassis_M_wheel_init.wheel_xy` 决定 ✓
- `Class_chassis_D4` —— **舵轮/独立转向**（本次新增，D = 舵；参考 `E:\rm\chassis_task.c`）

## 1. 两个类的对照

| | `Class_chassis_M4`（麦轮）| `Class_chassis_D4`（舵轮）|
|---|---|---|
| 每个轮子 | 1 个驱动电机 | **1 个转向 + 1 个驱动** |
| 反解输出 | 轮速 `speed`（rpm）| **转向角 `seat`（rad）+ 轮速 `speed`（rpm）** |
| 速度符号 | **由 `wheel_xy` 的 X 方位自动定**（X 正半轴为正、负半轴为负）| 不需要（舵向角说了算）|
| 输入 | `{vx[m/s], vy[m/s], ω[rad/s]}` | `{vx[m/s], vy[m/s], ω[rad/s]}`（ω 逆时针为正）|
| 小陀螺 | `count_run2({vx,vy,ω}, yaw)` | 同（先按 yaw 旋到机体系）|
| 小陀螺 0 位 | `Set_Top_Yaw(yaw)` / `Get_Top_Yaw()` ★ 见第 3 节 | 同 |
| 里程计 | `forward_count(speed[4])` | `forward_count(seat_now[4], speed_now[4])` |

> 单位约定：`speed` 与 M 版一致用 **rpm**（若电机层要 rad/s，乘 `π/30`）；`seat` 用 **rad ∈ [0, 2π)**。
>
> ⚠️ `forward_count()` 的 `speed[]` 必须是**关节侧 rpm**（电机类内部已经除过 `Gearbox_Rate`）。
> 它内部按 `v = rpm × π/30 × r` 换算，**不要再乘/除减速比**，否则会把减速比算两次。

## 2. API

```cpp
Class_chassis_D4 chassis_d;

/* 初始化：a = 半轴距, b = 半轮距, r = 轮半径(单位都用 m) */
chassis_D_wheel_init init[4] = {
    /*  x      y    seat_offset */
    {+a, +b,   0.0f},              // 左前
    {+a, -b,   0.0f},              // 右前
    {-a, +b,   0.0f},              // 左后
    {-a, -b,   0.0f},              // 右后
};
chassis_d.Init(a, b, r, init);

/* 每个控制周期 */
for (int i = 0; i < 4; ++i) {
    chassis_d.Set_Seat_Now(i, 实际转向角_rad);   // ★ 必须喂：就近翻转/到位判断都靠它
}
chassis_d.count_run1({vx, vy, ω});               // 或 count_run2({...}, yaw) 小陀螺

for (int i = 0; i < 4; ++i) {
    转向电机.Set_Pos(chassis_d.Get_Seat(i));     // rad
    驱动电机.Set_W (chassis_d.Get_Speed(i) * PI/30.0f);   // rpm → rad/s
}
```

## 2.5 ★ 小陀螺的 yaw 零位（`top_yaw`）—— 别删，很重要

```cpp
/* 按小陀螺键的那一刻，对齐一次： */
chassis.Set_Top_Yaw(当前yaw);      // 把"此刻的 yaw"记成小陀螺模式的 0 位

/* 之后每个周期： */
chassis.count_run2({vx, vy, ω}, 当前yaw);   // 内部自动用 (当前yaw - top_yaw)
```

**为什么不能直接用 yaw** ✗：
- 上电时 IMU 的 yaw = 0 只是**当时那个随机朝向** ⇒ 拿它当基准，**小陀螺一开，车会朝一个没意义的方向跑** ✗；
- `top_yaw` 记录的是 **"启动小陀螺那一刻"** 的 yaw ⇒ 无论上电时车头朝哪，小陀螺一开都是以"**按下那一刻的朝向**"为基准 ✓。

### ★ 为什么内部要取"最短夹角"（抗 ±π 跳变）

陀螺仪的 yaw 在 **±π 处会瞬间从 π 跳到 -π**（不连续）⇒ 直接相减：

```
yaw = +3.13,  top_yaw = -0.01   ⇒  yaw - top_yaw = 3.14   ✓
yaw = -3.13,  top_yaw = -0.01   ⇒  yaw - top_yaw = -3.12  ✗ 差了一圈!
```

⇒ 本实现把 `rel` 用 **`chassis_angle_diff_pi()` 取最短带符号夹角**（结果落在 (-π, π]）后再送 `arm_cos_f32/arm_sin_f32` ✓：

```cpp
const fp32 rel = chassis_angle_diff_pi(__yaw, top_yaw);   // 内部已归一,调用方无需处理
```

**因此 App 层不需要**再写这种归一化 ✗：

```cpp
/* 以前的做法(只归一了一边,top_yaw 仍是原始值 ⇒ 仍可能差一整圈): */
if (__yaw < 0) chassis.count_run2(__X_Y_W, 2*PI + __yaw);
else           chassis.count_run2(__X_Y_W, __yaw);

/* 现在直接传绝对 yaw 即可 ✓ */
chassis.count_run2(__X_Y_W, __yaw);
```

**要点** ✓：
- `count_run2` 的 `__yaw` 参数传的是**当前绝对 yaw** ✓，`top_yaw` 的减法**在函数内部做** ✓ ⇒ 调用方不用自己减 ✓；
- `Init()` 里把 `top_yaw` 置 0 ✓（即"上电朝向"），所以**没按小陀螺时行为不变** ✓；
- 想改基准（比如"重新以当前朝向为前"）⇒ 再调一次 `Set_Top_Yaw(当前yaw)` 即可 ✓；
- **`Set_Top_Yaw()` 内部会预处理**：把传进来的 yaw 归一到 **[0, 2π)** 再存 ⇒ 存下来的 `top_yaw` 始终是规范值，`Get_Top_Yaw()` 读出来也不会是 `-π` 那种别扭值 ✓；
- 标定/调试可以用 `Get_Top_Yaw()` 看当前基准 ✓。
## 3. ★ 舵轮特有的三条逻辑（抄自 `E:\rm\chassis_task.c`）

| # | 参考代码 | 本实现 | 为什么需要 |
|---|---|---|---|
| ① | `if (V_X==0 && V_Y==0) angle = angle_last;` | `mold < 1e-4 ⇒ seat = seat_last, speed = 0` | 指令为 0 时**保持上次转向角**，否则轮子会乱甩 |
| ② | `if (angle_set_ref > π/2) { angle -= π; speed = -speed_in*19/0.061; }` | 与 `seat_now` 差 > 90° ⇒ **翻 180° + 轮速取反** | 转向电机**少转半圈**（否则可能白转一整圈）|
| ③ | `if (angle_set_ref >= π/4) speed = 0;` | 残差 ≥ 45° ⇒ **先不给驱动速度** | 防轮胎"搓地"（转向没到位就驱动会撕胎/打滑）|

参考里另外两处工程细节也保留/等价实现了：
- `seat = angle*8191/(2π) + original_seat` ⇒ 本层输出 **rad**，编码器零位由 `chassis_D_wheel_init.seat_offset` 交给上层换算（不同电机编码器位数不同，写死 8191 不通用）；
- `19/0.061`（减速比 / 轮半径）⇒ 本层用 `speed = |v| / r * 30/π`，若你的舵轮带减速箱，把 `r` 传成"等效轮半径 = 真实半径 / 减速比"即可，或在上层再乘减速比。

## 4. 反解 / 正解公式

**反解（机体系）**：轮心速度 = 车体平动 + 绕中心转动
```
V_X = vx - ω·y_i
V_Y = vy + ω·x_i
seat  = atan2(V_Y, V_X)        → 归一到 [0, 2π)
speed = |V| / r * 30/π         （翻 180° 时取负）
```

**正解（里程计）**：由 4 轮实测反推车体
```
v_i  = rpm·π/30·r
vx_i = v_i·cos(θ_i),  vy_i = v_i·sin(θ_i)
vx = ¼Σvx_i ,  vy = ¼Σvy_i
ω  = Σ(-vx_i·y_i + vy_i·x_i) / (4·(a²+b²))
```
（最后一条是 4 轮 |r_i| 相等时的最小二乘解，与麦轮那套 `forward_count` 精神一致。）

## 5. 注意

1. **必须每拍喂 `Set_Seat_Now()`** ✗：不喂的话"就近翻转"和"到位判断"会失效（`seat_now` 一直是 0）；
2. **ω 正方向**：本实现取**逆时针为正**（右手系）。参考工程里 `V_X = x + ω·xy[0]` 的符号与它相反，那只说明它的 ω 定义是顺时针 —— 接线时把遥控的 ω 按你的实际方向取反即可，不要改这里；
4. 角度用的 8191（13 位编码器）在参考里是写死的，本层**不写死**，见第 3 节最后一条。

---

## 6. 2 舵轮 + 2 全向轮（`Class_chassis_D2Q2`）

构造（俯视，上 = 车头）：

```
D   Q
Q   D
```

- **D = 舵轮**：有转向 + 驱动，参与运动学；
- **Q = 全向轮**：被动支撑，**无动力、无编码器**，不参与解算。

### 反解

每个 D 轮：

```
V_X = vx - ω·y_i
V_Y = vy + ω·x_i
正向候选：(atan2(V_Y, V_X), +|V| / r × 30/π)
反向候选：(atan2(V_Y, V_X) + π, -|V| / r × 30/π)
```

两个候选的舵角归一到 [0, 2π)，以最后一次有效实测舵角为起点选更短的弧。两条弧在浮点精度内等长时，先比较上一目标舵角，仍等长时沿用上次驱动方向。轮心速度低于 0.02 m/s 时保留上一目标舵角和带符号转速记录，实际驱动指令归零。首次反馈前或所选舵角残差达到 30° 时停止驱动。

### 正解

每个 D 轮的实际速度矢量：

```
v_i = speed[rpm]·π/30·r，方向 = seat_now[i]
v_ix = v_i·cos(seat_i),  v_iy = v_i·sin(seat_i)
```

刚体速度场：

```
v_ix = vx - ω·y_i
v_iy = vy + ω·x_i
```

两个轮子联立：

```
ω = (v0x - v1x)/(y1 - y0) = (v0y - v1y)/(x0 - x1)
vx = ½[(v0x + ω·y0) + (v1x + ω·y1)]
vy = ½[(v0y - ω·x0) + (v1y - ω·x1)]
```

两个 ω 估计都有效时取平均；若两轮 x 或 y 相同，只有一条有效，代码自动用另一条。

### API

| 接口 | 说明 |
|---|---|
| `Init(a, b, r, init[2])` | 初始化；`init` 给两个 D 轮的 `x/y/seat_offset`，Q 轮位置不影响解算 |
| `count_run1({vx,vy,ω})` | 反解出 2 个 D 轮的 `seat/speed` |
| `count_run2({vx,vy,ω}, yaw)` | 小陀螺：先按 `yaw-top_yaw` 旋到机体系 |
| `forward_count(seat_now[2], speed_now[2])` | 正解出 `vx/vy/ω` |
| `Set_Seat_Now(i, rad)` | 每拍喂实际转向角（就近翻转/到位判断用） |
| `Set_Steer_Def(i, def)` / `Motor_To_Seat` / `Seat_To_Motor_Delta` | 与 D4 同一套舵向定义/零位/最短弧 |
| `Get_Seat(i)` / `Get_Speed(i)` / `Get_Vx/Vy/Vw` | 读目标/反馈/里程计 |

### 配置示例

```cpp
static Class_chassis_D2Q2 s_d2q2;
static const chassis_D_wheel_init s_d2q2_init[CHASSIS_D2Q2_UNIT_N] = {
    { +0.30f, +0.30f, 0.0f },   /* 前左 D */
    { -0.30f, -0.30f, 0.0f },   /* 后右 D */
};
s_d2q2.Init(0.30f, 0.30f, 0.06f, s_d2q2_init);

/* 两个 D 轮各配:转向电机下标 / 减速比 / 方向 / 零位 */
s_d2q2.Set_Steer_Def(0, {0u, CHASSIS_STEER_RATIO_3508BARE_1TO4, +1.0f, 0.0f});
s_d2q2.Set_Steer_Def(1, {2u, CHASSIS_STEER_RATIO_3508BARE_1TO4, +1.0f, 0.0f});
```

> Q 轮在另外两个角 `(+a, -b)`、`(-a, +b)`，只承重，不需要电机。
>
> ⚠️ **App 接线**：D2Q2 需要 4 个电机（2 转向 + 2 驱动）。当前 `motor_task` 只支持单台大疆
> 舵向（0 号），启用 D2Q2 前要先扩展电机注册表与转向/驱动目标写入。

### 注意

1. **Q 轮不能提供里程计**：正解只靠 2 个 D 轮的速度矢量；Q 没有编码器。
2. 每拍有有效反馈时调用 `Set_Seat_Now` 更新实测舵角。重新标定后，第一份新反馈会重新设定舵角起点。
3. **两个 D 轮不要放在同一 x 或同一 y**：否则会丢掉一条 ω 估计；对角线布置正好两条都有。
4. 反解输出 `speed` 是**关节侧 rpm**，驱动电机层再按 `rpm × π/30` 转 rad/s。