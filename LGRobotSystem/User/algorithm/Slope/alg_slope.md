# alg_slope — 斜坡函数，用于速度规划等

原作者（yssickjgd）的 Doxygen 文件头，照原文保留如下（`alg_slope.h` / `alg_slope.cpp` 顶部原本各有一份，
内容一致）：

```c
/**
 * @file alg_slope.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 斜坡函数, 用于速度规划等
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-06-03 1.1 规划引入优先级方式
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */
```

```c
/**
 * @file alg_slope.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 斜坡函数, 用于速度规划等
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-06-03 1.1 规划引入优先级方式
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */
```

---

## 1. 定位

斜坡函数（Slope）本体，把目标值按每个计算周期最多 `Increase_Value` / `Decrease_Value` 的步长
逼近，用于速度规划等场合。

* `alg_slope.h`：`Enum_Slope_First`、`Class_Slope` 声明 + 全部 inline 实现
* `alg_slope.cpp`：`Class_Slope::Init()` 与 `Class_Slope::TIM_Calculate_PeriodElapsedCallback()`

## 2. 规划优先类型 `Enum_Slope_First`

| 枚举 | 值 | 含义 |
|---|---|---|
| `Slope_First_REAL` | `0` | 真实值优先（硬规划的软化）：当前真实值夹在当前规划值和目标值之间时，当前规划值转为当前真实值 |
| `Slope_First_TARGET` | `1` | 目标值优先，即硬规划 |

## 3. 关键函数

| 函数 | 作用 | 参数 | 返回值 | 注意 |
|---|---|---|---|---|
| `Init(__Increase_Value, __Decrease_Value, __Slope_First = Slope_First_REAL)` | 初始化斜坡参数 | `__Increase_Value` 增长最大幅度；`__Decrease_Value` 降低最大幅度；`__Slope_First` 规划优先类型 | `void` | 默认按 `Slope_First_REAL` |
| `Set_Now_Planning(__Now_Planning)` | 设定当前规划值（内部 `Out` / `Now_Planning`） | `__Now_Planning` 当前规划值 | `void` | **初始化后必须调用**，确保斜坡从当前位置开始步进（初次必须设为当前位置，避免从 0 跳变）；同时写 `Out` 与 `Now_Planning` |
| `Set_Now_Real(__Now_Real)` | 设定当前真实值 | `__Now_Real` | `void` | `inline` |
| `Set_Increase_Value(__Increase_Value)` | 设定绝对值增量（一次计算周期改变值） | `__Increase_Value` | `void` | `inline` |
| `Set_Decrease_Value(__Decrease_Value)` | 设定绝对值减量（一次计算周期改变值） | `__Decrease_Value` | `void` | `inline` |
| `Set_Target(__Target)` | 设定目标值 | `__Target` | `void` | `inline` |
| `Get_Out()` | 获取输出值 | 无 | `float` 输出值 | `inline` |
| `TIM_Calculate_PeriodElapsedCallback(__Now_Real, __Target)` | 斜坡函数调整值，**计算周期取决于调用者** | `__Now_Real` 当前真实值；`__Target` 目标值 | `const fp32` 本拍输出 | 末尾把 `Out` 回写进 `Now_Planning`；返回 `Out` |

## 4. 计算逻辑（`TIM_Calculate_PeriodElapsedCallback`）

1. **真实值优先的额外逻辑**（`Slope_First == Slope_First_REAL`）：
   若 `(__Target >= __Now_Real && __Now_Real >= Now_Planning)` 或
   `(__Target <= __Now_Real && __Now_Real <= Now_Planning)`，则 `Out = __Now_Real`。
2. 再按 `Now_Planning` 的符号（正值 / 负值 / 0）分三支，各自比较 `__Target` 与 `Now_Planning`：

   * 差值绝对值 `> Increase_Value`（加速方向）→ `Out += / -= Increase_Value`
   * 差值绝对值 `> Decrease_Value`（减速方向）→ `Out -= / += Decrease_Value`
   * 否则 `Out = __Target`（直接到位）

   三支里的注释分别为"正值加速 / 正值减速 / 负值加速 / 负值减速 / 0 值正加速 / 0 值负加速"。
3. **善后**：`Now_Planning = Out;` 然后 `return Out;`。

## 5. 内部变量

| 变量 | 类型 / 初值 | 含义 |
|---|---|---|
| `Out` | `float = 0.0f` | 输出值 |
| `Slope_First` | `Enum_Slope_First = Slope_First_REAL` | 规划优先类型 |
| `Now_Planning` | `float = 0.0f` | 当前规划值 |
| `Now_Real` | `float = 0.0f` | 当前真实值 |
| `Increase_Value` | `float = 0.0f` | 绝对值增量，一次计算周期改变值 |
| `Decrease_Value` | `float = 0.0f` | 绝对值减量，一次计算周期改变值 |
| `Target` | `float = 0.0f` | 目标值 |

## 6. 相关

* `math_support.h` —— 用到 `Basic_Math_Abs`
* 版权：`/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/`
  （`alg_slope.h` 与 `alg_slope.cpp` 文件末尾各有一行，保留在源码中）
