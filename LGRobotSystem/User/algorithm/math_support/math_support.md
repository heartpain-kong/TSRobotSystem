# math_support — 数学基础设施（**本目录唯一的一份文档**）

> 本目录**只保留这一份 md**（不再给每个模块各配一个）。下面按模块分节说明。

## 目录结构

| 文件 | 职责 | 主要使用方 |
|---|---|---|
| `math_support.h` | **汇总入口**：常量 + 转发下面 4 个模块 | 全工程 `#include "math_support.h"` |
| `alg_math_angle.{h,cpp}` | 角度：归一 / 最短夹角 | 底盘(麦轮+舵轮)、PID |
| `alg_math_limit.{h,cpp}` | 限幅：`LimitMax` / `LimitMinMax` | `alg_pid`、各电机 |
| `alg_math_convert.{h,cpp}` | 定点↔浮点换算 + 大小端 | DM / LZ / MIT 电机 |
| `alg_math_matrix.{h,cpp}` | 4×4 矩阵 + D-H | （暂无调用，保留）|

> `math_support.h` 是**纯头文件**（常量用 C++17 `inline constexpr`）⇒ 不需要 `math_support.cpp`。
> **老代码继续 `#include "math_support.h"` 即可**，拆分对调用方零影响 ✓
> ⚠️ `motor_max_min()` 的**定义在 `motor_mit.cpp`**（它的家），这里不声明。

---

## 1. 通用常量 + `Basic_Math_Abs`

| 常量 | 值 | 含义 |
|---|---|---|
| `PI2` | 6.2831853084 | 2π |
| `MATH_RPM_TO_RADPS` | 2π/60 = 0.10472 | rpm → rad/s |
| `MATH_DEG_TO_RAD` | π/180 | 度 → 弧度 |
| `MATH_CELSIUS_TO_KELVIN` | 273.15 | ℃ → K |

> ⚠️ `MATH_RPM_TO_RADPS` 曾经写成 `2π/360`（那是"度→弧度"，与 `MATH_DEG_TO_RAD` 同值），
> 会让所有用它的转速反馈**小 6 倍** —— 已修正为 `2π/60` ✓

```cpp
template<typename Type> Type Basic_Math_Abs(Type x);
```
原来是 `(x > 0) ? x : -x` ⇒ `x == 0` 时返回 `-0.0f`；已改成先判负 ✓（`alg_slope` 在用，6 处）

---

## 2. `alg_math_angle` — 角度工具（弧度）

| 接口 | 语义 | 值域 |
|---|---|---|
| `Math_Angle_Wrap_2Pi(a)` | 归一到 [0, 2π) | `[0, 2π)` |
| `Math_Angle_Wrap_Pi(a)` | 归一到 (-π, π] | `(-π, π]` |
| **`Math_Angle_Diff_Pi(target, now)`** | 最短**带符号**夹角 | `(-π, π]` |
| `Math_Angle_Abs_Diff(ref, set)` | 最短夹角**绝对值**（原 `chassis_math_angle_set_ref`）| `[0, π]` |

### ★ 为什么必须有 `Math_Angle_Diff_Pi`

陀螺仪 yaw 在 **±π 处会瞬间从 π 跳到 -π**（不连续）⇒ 直接相减会凭空差 2π：

```
yaw = +3.13, top = -0.01  ⇒ yaw - top =  3.14  ✓
yaw = -3.13, top = -0.01  ⇒ yaw - top = -3.12  ✗ 差了一整圈!
```

⇒ **送 `arm_cos_f32` / `arm_sin_f32` 之前必须先取最短带符号夹角** ✓：

```cpp
const fp32 rel = Math_Angle_Diff_Pi(__yaw, top_yaw);   // 内部已归一，调用方无需处理
```

它**先把两个角各自归一再相减并折到 (-π, π]** ⇒ 对未归一的输入也安全 ⇒ App 层
**不需要**再写 `if (yaw < 0) yaw += 2π;`（那种归一只归一了一边，另一边还是原始值，反而可能差一整圈）。

### 为什么单独拆出来
原来"角度"只有一个 `chassis_math_angle_set_ref`，而底盘、舵轮各自在本地重写了
"归一到 [0,2π) / 取最短夹角" ⇒ 同一段逻辑散在三处。现在统一一份 ✓
（`alg_chassis_control.cpp` 已删掉本地 23 行实现，`angle_pi()` 变成 3 行薄封装 ✓）

---

## 3. `alg_math_limit` — 限幅

**函数名与签名保持不变** ⇒ 调用方零改动 ✓

| 接口 | 说明 |
|---|---|
| `LimitMax(x, max)` | 对称限幅 `±max`；`max <= 0` 表示**不限制**；**形参是引用 ⇒ 直接改 x** |
| `LimitMinMax(x, max, min)` | 不对称限幅 `[min, max]`；`max <= min` 表示不限制；同样是引用 |

### ★ 必须知道的历史坑（已修）

这两个函数以前是**按值传递** `void LimitMax(fp32 input, fp32 max);` ✗ ⇒ 改的是局部副本
⇒ `Class_PID::Cout()` 里的积分限幅与输出限幅

```cpp
LimitMax(Iout, max_iout);   // 积分限幅
LimitMax(out,  max_out);    // 输出限幅
```

**两条从来没生效过** ✗（积分无限累积 ⇒ 积分饱和；输出不限幅）。改成 `fp32 &input` 之后才是真的限幅 ✓

---

## 4. `alg_math_convert` — 定点/浮点换算 + 大小端

| 接口 | 说明 |
|---|---|
| `uint_to_float(raw, min, max, bits)` | 定点 → 浮点：`raw·span/(2^bits−1) + min` |
| `float_to_uint_f(x, min, max, bits)` | 浮点 → 定点（min/max 是浮点）|
| `float_to_uint_i(x, min, max, bits)` | 浮点 → 定点（min/max 是整数）|
| `Math_Endian_Reverse_16(src, dst)` | 16 位大小端翻转；`dst = nullptr` 则只返回不写回 |

> 公式与原来**一字不差**（原样搬移）。注意 `float_to_uint_*` **不做饱和**，超范围会回绕 ⇒ 调用方先限幅 ✓

### 为什么收拢到这里
这三个换算函数的**实现原来写在 `motor_mit.cpp`**（某个品牌电机的基类文件）里，却在
`math_support.h` **和** `motor_mit.h` 里**各声明了一遍** ✗ —— 跨模块重复声明，而且"算法层通用的
换算"挂在某个电机文件里，达妙/瓴控要用都得靠间接包含。
⇒ 实现搬到算法层 ✓，`motor_mit.h` 改为 `#include "alg_math_convert.h"` ✓ ⇒ 三家电机照常调用 ✓

---

## 5. `alg_math_matrix` — 4×4 矩阵与 D-H

**现状：全工程没有调用点** —— 整套原样搬过来保留（以后做机械臂正/逆解会用到）。

| 接口 | 说明 |
|---|---|
| `math_matrix_equal4x4(set, ret)` | 4×4 复制 |
| `math_matrix_contrastl4x4(a1, a2, err)` | 逐元素比较，误差 `err` 内一致返回 true |
| `math_matrix_multiplication4x4(a1, a2, ret)` | `ret = a1 × a2` |
| `math_matrix_DH(ret, θ, α, a, d)` | D-H 变换矩阵 |
| `math_matrix_DH3x3(ret, θ, α)` | D-H 旋转部分 |

### 搬移时修的笔误
两个 DH 函数的 `[0][1]` 原来写成 `cos(theta)` ✗，但函数下方注释给出的公式是：

```
T = [cos(theta), -sin(theta),          0,          a;
     sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
     sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
     0, 0, 0, 1];
```

⇒ 已按公式改为 **`-sin(theta)`** ✓（因为无人调用，改动无影响）

---

## 6. 谁在用（拆分后调用方一行没改 ✓）

| 调用方 | 用了什么 |
|---|---|
| `alg_chassis_control.cpp` | `Math_Angle_Wrap_2Pi` / `Math_Angle_Diff_Pi`（麦轮 + 舵轮）|
| `alg_pid.cpp` | `LimitMax` / `LimitMinMax` |
| `motor_DM` / `motor_LZ` / `motor_YS` / `motor_DJ` / `motor_mit` | 定点换算、`motor_max_min` |
| `alg_slope.cpp` | `Basic_Math_Abs` |