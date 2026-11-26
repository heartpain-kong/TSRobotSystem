# Kalman — 通用卡尔曼框架 + 四元数 EKF

移植来源：`E:\rm\chaiss\dm02\User_File\algorithm\Kalman\{kalman_filter,quaternion_ekf}.{c,h}`
（同一套算法在 `wheel-legged-master\...\User\Algorithm\EKF\QuaternionEKF.c` 里也有一份，两份代码逐行一致）

| 文件 | 作用 |
|---|---|
| `alg_kalman.{h,cpp}` | **通用卡尔曼框架**（可复用，任何线性 KF 都能搭）|
| `alg_quaternion_ekf.{h,cpp}` | **四元数 EKF 姿态解算**（6 状态 / 3 量测，含卡方门控与自适应增益）|

依赖：`Class_QuaternionEKF` **继承** `Class_Kalman`；矩阵运算走 CMSIS-DSP
（`Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a`，CMakeLists 里已经链好了）。

---

## 1. 通用框架 `Class_Kalman`

```
系统模型   x_k = F·x_{k-1} + B·u_k + w_k        w ~ N(0, Q)
           z_k = H·x_k + v_k                    v ~ N(0, R)
```

### 两种用法

**① 定常线性 KF**：填好 F/H/Q/R，每拍调一次 `Update()` 就完了

```cpp
Class_Kalman kf;
kf.Init(2, 1, 1);                       // 2 状态 / 1 输入 / 1 量测
// 填矩阵：Matrix_* 的 pData 就是内部数据区，直接写
kf.Matrix_F.pData[0] = 1.0f;  kf.Matrix_F.pData[1] = dt;
kf.Matrix_F.pData[2] = 0.0f;  kf.Matrix_F.pData[3] = 1.0f;
kf.Matrix_H.pData[0] = 1.0f;  kf.Matrix_H.pData[1] = 0.0f;
kf.Matrix_Q.pData[0] = 0.001f;  kf.Matrix_Q.pData[3] = 0.003f;
kf.Matrix_R.pData[0] = 0.5f;
kf.Matrix_P.pData[0] = 1.0f;    kf.Matrix_P.pData[3] = 1.0f;
// 每拍：
kf.MeasuredVector[0] = 传感器值;         // ★ 每拍都要重填(Measure 里会清零)
kf.ControlVector[0]  = 控制量;
float *x = kf.Update();
float v  = kf.Get_State(0);
```

**② 时变 / 非线性**（EKF 就是这种）：**别用 `Update()`**，按参考的顺序自己调各步，
在中间插自己的线性化与量测模型 —— 原实现就是靠 7 个函数指针钩子插在这 7 个位置上：

```cpp
Measure();                // z ← 量测   (钩子0 在参考里是"备份给调试器看")
xhatMinusUpdate();        // x⁻ = F·x
/* ← 这里插"F 线性化 / P 渐消"(钩子1) */
PminusUpdate();           // P⁻ = F·P·Fᵀ + Q
/* ← 这里插"量测矩阵 H"(钩子2) */
SetK();  xhatUpdate();  P_Update();       // 通用版本;自己算 K 时用 SkipEq3/4/5 跳过
Finalize();               // P 对角元下限 + FilteredValue ← x
```

### API

| 接口 | 说明 |
|---|---|
| `Init(xhatSize, uSize, zSize)` | 绑定内部矩阵(定长存储,上限见下) |
| `Measure()` | `MeasuredVector → z_data`、`ControlVector → u_data`，并把前者**清零** |
| `xhatMinusUpdate()` / `PminusUpdate()` | 先验状态 / 先验协方差 |
| `SetK()` / `xhatUpdate()` / `P_Update()` | 通用增益 / 状态更新 / 协方差更新 |
| `Finalize()` | P 对角元下限保护 + 输出到 `FilteredValue` |
| `Update()` | 定常线性 KF 的一条龙 |
| `Get_FilteredValue()` / `Get_State(i)` | 取滤波结果 |
| `Get_MeasurementValidNum()` | 本拍有效量测数（自适应量测才有意义）|
| `Get_MatStatus()` | 最近一次矩阵运算返回码 |
| `SkipEq1..5` | 跳过对应步骤（`SkipEq3/4/5` 是"自己算 K/xhat/P"用的）|
| `UseAutoAdjustment` + `MeasurementMap/Degree/MatR_DiagonalElements` | 自适应量测：**量测里等于 0 的项视为无效**，H/R/K/z 自动缩小 |

容量上限（`alg_kalman.h` 顶部宏，按需改大）：`KALMAN_MAX_XHAT_SIZE = 8`、
`KALMAN_MAX_U_SIZE = 4`、`KALMAN_MAX_Z_SIZE = 6`。

---

## 2. 四元数 EKF `Class_QuaternionEKF`

```
状态(6)  x = [q0 q1 q2 q3 | b_x b_y]ᵀ      q:机体→大地四元数;b:陀螺 x/y 零偏
量测(3)  z = 归一化加速度计向量(重力方向)
输入     无(uSize = 0):角速度直接从 F 矩阵进,不是走 B·u
```

### 用法

```cpp
Class_QuaternionEKF ekf;

/* Robot_Init():参数用参考工程实际在跑的那组 */
ekf.Init(10.0f, 0.001f, 1000000.0f, 1.0f, 0.0f);
/*        Q1    Q2     R           λ    lpf  */

/* 1kHz 任务里(gyro: rad/s , accel: m/s² , dt: s) */
ekf.Update(gx, gy, gz, ax, ay, az, dt);

float yaw_rad  = ekf.Get_Yaw_Rad();            // [-π, π],会跳
float yaw_cont = ekf.Get_YawTotalAngle_Rad();  // ★ 连续 Yaw,不跳 —— 底盘小陀螺/舵轮用这个
float roll_rad = ekf.Get_Roll_Rad();
```

### 参数

| 参数 | 含义 | 参考工程的值 |
|---|---|---|
| `Q1` | 四元数过程噪声（乘 dt 进 Q） | `10` |
| `Q2` | 陀螺零偏过程噪声 | `0.001` |
| `R` | 加速度计量测噪声 | `1000000` |
| `λ` | 渐消记忆因子（>1 会被夹到 1）| `1` |
| `lpf` | 加速度计一阶低通**时间常数(s)**；**`0` = 关闭低通** | `0`（basic_framework）/ `0.0089`（wheel-legged 的 INS 任务里用过）|

> `Q1/Q2` 越大越信陀螺、越小越信加速度计；`R` 越大越不信加速度计（动态时别被加速度骗）。

### API

| 接口 | 说明 |
|---|---|
| `Init(Q1, Q2, R, λ, lpf)` | 初始化（**必须先调**）|
| `Update(gx..az, dt)` | 跑一拍 |
| `Get_Roll() / Get_Pitch() / Get_Yaw() / Get_YawTotalAngle()` | **度**（与参考一致，方便看数）|
| `Get_Roll_Rad() / ..._Rad()` | **弧度**（本工程转换，直接喂底盘/PID）|
| `Get_Quaternion(q[4])` | 四元数 |
| `Get_GyroBias(b[3])` | 零偏估计（z 恒为 0）|
| `Is_Converged()` / `Is_Stable()` | 是否已收敛 / 本拍是否"静止可信" |
| `Get_ChiSquare()` / `Get_AdaptiveGainScale()` / `Get_ErrorCount()` / `Get_UpdateCount()` | 调试量 |
| `Matrix_P` / `Matrix_H` / `Get_Matrix_K()` | 看协方差 / 量测矩阵 / 增益 |

---

## 3. 单位约定（别搞混）

| 量 | 单位 |
|---|---|
| 陀螺输入 | **rad/s**（BMI088 驱动给的就是 rad/s ✓）|
| 加速度输入 | **m/s²**（BMI088 驱动给的就是 m/s² ✓）|
| `dt` | **s**（用实测周期，别用常数 0.001 凑）|
| EKF 的 Roll/Pitch/Yaw | 内部**按度存**（参考如此）；要弧度用 `Get_*_Rad()` |
| Mahony 的 Roll/Pitch/Yaw | **弧度**（参考如此）；要度用 `Get_*_Deg()` |

---

## 4. ★ 三个必须知道的点

### ① 原作者有意"不修正 yaw 轴数据" —— 别当 bug 删掉

`alg_quaternion_ekf.cpp` 里这一行：

```cpp
/* ★ 原作者的注释:「不修正 yaw 轴数据」 */
temp_vector.pData[3] = 0.0f;
```

**RM 原版（wheel-legged 的 `QuaternionEKF.c`）里是同一行、同一句注释**，两份代码完全一致
⇒ 是作者有意为之（加速度计本来就观测不到偏航 ⇒ 干脆不让卡尔曼去动它），本移植**照搬**。
想验证影响：把这行注释掉跑一次，对比静止/慢转时的 yaw 漂移即可。

### ② 静止时 yaw 只能靠陀螺积分；要有动作才收敛

加速度计只提供 roll/pitch 两个约束 ⇒ **yaw 在静止时不可观测**（`ConvergeFlag` 要等
`χ²` 连续变小才会置位）。所以：

* 上电后**先静置几百毫秒**（`Is_Converged()` 变 true），再开始用 yaw；
* 长时间静止时 yaw 会有缓慢漂移（陀螺零偏在估，但不是瞬间收敛）；
* 要"绝对朝向"就得加磁力计或视觉/激光（本模块没有）。

### ③ 必须每拍重填量测

`Measure()` 会把 `MeasuredVector` **清零**（参考如此）⇒ 填一次只生效一拍。
EKF 的 `Update()` 内部已经帮你填好了 ✓；但如果你直接用通用框架，记得每拍重填。

---

## 5. 本工程加的保护（都不改数学，仅防 NaN/首拍毛刺）

| # | 位置 | 参考的问题 | 本移植 |
|---|---|---|---|
| ① | `Inv_Sqrt` | 输入 ≤ 0 会算出 NaN/Inf | 返回 0 |
| ② | `Update()` 里算模长 | `1/0` 得到 Inf 流进 `StableFlag` 判断 | 先判 0，取不到就置 0 |
| ③ | `Linearization_And_P_Fading` | 四元数模长可能为 0 | 为 0 就不归一 |
| ④ | `Update()` 入口 | 参考若没 `Init` 会**偷偷用一套默认噪声自动 Init**（掩盖调参错误）| 没 Init 就**直接返回**，不猜 |

另外：参考里的局部量是 `static`（不可重入），本移植改成普通局部量；
`invSqrt` 里的位转换用 `int32_t`（参考用 `long`，在 PC 上是 8 字节 —— 会算错）。

---

## 6. Mahony 还是 EKF？

| | **Mahony**（`User/algorithm/Mahony`）| **四元数 EKF**（本目录）|
|---|---|---|
| 算力 | 极小（几次乘法 + 一次 sqrt）| 大（一堆 6×6/6×3 矩阵乘 + 3×3 求逆）|
| 调参 | 只有 `Kp/Ki` | `Q1/Q2/R/λ/lpf` |
| 陀螺零偏 | **不估计** ⇒ 长时间缓慢漂移 | 估计（状态里带 b_x/b_y）⇒ 更稳 |
| 异常量测 | 只靠 Kp 拉回 | 卡方门控 + 自适应增益（被撞/被抬起时自动不采信）|
| 建议 | 先跑通用它验证硬件通路；或算力紧张时用 | 正式上车用 |

---

## 7. 上板自检

```
① 静止放平:Roll ≈ 0°, Pitch ≈ 0°,几秒内 Is_Converged() 变 true
② 绕各轴慢慢转 90°:角度符号与大小对不对(先确认 BMI088 三轴符号/安装方向)
③ 快速晃动:Is_Stable() 应变成 false(陀螺快/加速度模长偏离 1g)
④ 静置 5 分钟:yaw 漂移应小于几度(零偏在收敛);若漂很多 ⇒ 调小 R 或调大 Q2
⑤ 对比 Mahony:两者 roll/pitch 应基本一致 ⇒ 对不上说明有一路输入单位搞错了(deg vs rad)
```
