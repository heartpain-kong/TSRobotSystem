# RobotSlutionCalculation — 底盘解算（纯数学层）

> 目录名是历史拼写（`Slution`，App 里也这么引用），为不动 `#include` 路径保留原名。
> **本目录不依赖 HAL / FreeRTOS / CAN**，上板、下位机、PC 端都可以直接编译。

---

## 1. 这个目录是干什么的

一句话：**把「车体速度」和「每个舵轮的转角/转速」互相换算**，再给一个功率粗估。
纯数学，不碰任何硬件；几何参数、轮半径都由调用方传进来。

---

## 2. 文件作用

| 文件 | 作用 | 现在谁在用 |
|---|---|---|
| `robot_solution.h` | 接口 + 数据结构声明 | — |
| `robot_solution.cpp` | 三个纯函数的实现（逆解 / 正解 / 功率估算）| **目前没有 App 调用**，见第 4 节 |
| ~~`can3_protocol.h/.cpp`~~ | 旧的 CAN3 8 字节帧编解码 | **已删除（2026-11-27）**：上下 MCU 链路改成 USART2 RS485（`Device/comMCU/cmd_485`）后没有任何代码引用 |

---

## 3. 接口与单位

```c
typedef struct { float x, y;        /* 轮心位置 [m] */
                 float zero_rad;    /* 机械零位偏置 [rad] */
                 float dir;         /* +1 / -1(装反) */ } RobotWheelGeometry;

typedef struct { float vx, vy;      /* 车体速度 [m/s] */
                 float w; }         /* 车体 yaw 角速度 [rad/s] */ RobotVelocity;

typedef struct { float steer_rad;   /* 舵向目标 [rad] */
                 float wheel_rpm;   /* 驱动轮目标 [rpm] */
                 float vx, vy;      /* 该轮心速度 [m/s] */
                 uint8_t flipped; } /* 1 = 走了 180° 翻转 */ RobotWheelCommand;
```

| 函数 | 做什么 | 公式 |
|---|---|---|
| `RobotSolution_Inverse2` | 车体 (vx,vy,w) → 每个舵轮 (steer, rpm) | 刚体速度场 `vx_i = vx − w·y_i`、`vy_i = vy + w·x_i`；`steer = atan2(vy_i, vx_i)`、`rpm = |v_i| / r` |
| `RobotSolution_Forward2` | 每个舵轮 (steer, rpm) → 车体 (vx,vy,w) | 两点速度联立，最小二乘（下位机回传实测里程计用）|
| `RobotSolution_PowerEstimate` | 功率粗估 | `P = Σ|v_i|·|F_i| / η + P_idle`，只用于 120 W 限幅 |

调用方负责：轮半径、`max_wheel_rpm` 限幅（`<= 0` = 不限）、几何表。

---

## 4. 和 App 里真正在跑的解算是什么关系（重要）

App 实际用的是 **`User/algorithm/chassis_control/alg_chassis_control.cpp` 的 `Class_chassis_D2Q2`**：

| | 本目录 `robot_solution` | App 实际用的 `Class_chassis_D2Q2` |
|---|---|---|
| 反解 | `Inverse2` | `count_run1`：同一套速度场 + **优劣弧**（>90° 翻 180° 同时轮速取反）+ 零速保持上一舵角 |
| 正解 | `Forward2` | `forward_count`：两点速度联立，两个 ω 估计取平均 |
| 舵角存储 | 返回 `steer_rad`（无归一） | `seat ∈ [0, 2π)`，配 `Motor_To_Seat / Seat_To_Motor_Delta` 走最短弧 |
| 功率 | `PowerEstimate` | `Chassis_Task` 里按实测力矩×转速估，见 `chassis_task.cpp` |

结论：两者**公式一致**，但 App 走的是 `Class_chassis_D2Q2`（多了优劣弧/零位映射/最短弧这些量产细节）。
`robot_solution` 是同一套东西的纯 C 版（带功率估算），留给上板或其它工程复用；
**改底盘参数只改 `chassis_task.cpp`（几何在 `robot_def.h` 的 `CFG_CHASSIS_*`），不要以为改这里会影响整车。**

---

## 5. 想直接用它时

```c
#include "robot_solution.h"

RobotWheelGeometry geom[2] = {
    { +0.1625f, -0.1625f, 0.0f, +1.0f },
    { -0.1625f, +0.1625f, 0.0f, -1.0f },
};
RobotVelocity vel = { .vx = 3.0f, .vy = 0.0f, .w = 0.0f };
RobotWheelCommand out[2];

if (RobotSolution_Inverse2(&vel, geom, 0.05f, 600.0f, out) == 0)
{
    /* out[0].steer_rad / wheel_rpm 就是 202/201 的目标 */
}
```

编译期需要 `User/algorithm/RobotSlutionCalculation` 在 `include_directories` 里（`CMakeLists.txt` 已有）。
