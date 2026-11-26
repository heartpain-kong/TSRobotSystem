# alg_mahony — Mahony 互补滤波姿态解算

移植来源：`E:\rm\chaiss\dm02\User_File\algorithm\mahony\mahony_filter.{c,h}`
（同一套算法也在 `wheel-legged-master` 的 `INS_task` 里用过，那时的调用是
`mahony_init(&mahony, 1.0f, 0.0f, 0.001f)`）

## 1. 原理（就 5 步，代码与参考逐行一致）

1. **加速度计归一化** ⇒ 得到"重力方向"的观测向量；
2. 观测重力方向 **×** 旋转矩阵里的重力方向（`Rotation_Matrix[2][*]`）⇒ 得到姿态误差 `e`；
3. `e` 积分（`Ki`）+ 比例（`Kp`）⇒ **修正陀螺**（这是"互补"的来源：加速度计管长时、陀螺管短时）；
4. 修正后的陀螺用**一阶龙格-库塔**积分四元数，再归一化；
5. 由四元数刷新**旋转矩阵** ⇒ 反解 `roll / pitch / yaw`。

## 2. API

| 接口 | 对应参考 | 说明 |
|---|---|---|
| `Init(Kp, Ki, dt)` | `mahony_init` | `dt` 是**固定**调用周期(s) |
| `Set_Input(gx..az)` | `mahony_input` | 只喂数据（gyro: rad/s，accel: m/s²）|
| `Calculate()` | `mahony_update` | 互补滤波 + 四元数积分 + 刷新旋转矩阵 |
| `Calculate_Attitude()` | `mahony_output` | 由旋转矩阵反解姿态角 |
| **`Update(gx..az)`** | 上面三步 | **一条龙**（本工程加的便利接口，平时用这个）|
| `Get_Roll() / Get_Pitch() / Get_Yaw()` | — | **弧度**（与参考一致）|
| `Get_Roll_Deg() / ...` | — | 度 |
| `Get_Quaternion(q[4])` / `Get_Rotation_Matrix(i,j)` | — | 四元数 / 旋转矩阵元素 |
| `Set_Kp / Set_Ki / Set_Dt` | — | 在线调参 |

### 用法

```cpp
Class_Mahony mahony;

/* 初始化 */
mahony.Init(1.0f, 0.0f, 0.001f);        // Kp, Ki, dt(s)

/* 1kHz 任务里 */
mahony.Update(gx, gy, gz, ax, ay, az); // gyro: rad/s   accel: m/s²
float yaw_rad = mahony.Get_Yaw();      // 弧度(注意:和 EKF 的"度"不一样 ✗)
```

**调参**：`Kp` 越大越信加速度计（跟得快但易被运动加速度干扰）；`Ki` 用来慢速消除陀螺零偏
（参考默认给 0 —— 想要零偏自消就加一点，比如 0.005，但别大）。

## 3. 单位（最容易搞错的地方）

| 量 | 单位 |
|---|---|
| 陀螺输入 | **rad/s**（若你手上是 deg/s，先 ×0.0174533）|
| 加速度输入 | **m/s²** |
| `dt` | **s**（`Init` 里给固定值；周期变了要重新 Init 或 `Set_Dt`）|
| 输出角度 | **弧度**（`Get_*_Deg()` 才是度）|

## 4. 本工程加的保护（不改数学）

| # | 参考的问题 | 本移植 |
|---|---|---|
| ① | `mahony_update` 直接 `acc/模长`，**没判 0** ⇒ 加速度全 0 时四元数被 NaN 污染 | 模长 ≤ 0 直接返回 |
| ② | 四元数归一化同样没判 0 | 模长 ≤ 0 不归一 |
| ③ | 结构体是全局变量 ⇒ 首拍 `update` 前旋转矩阵是**全 0**，第一拍误差是假的 | `Init` 里先按单位四元数刷一次矩阵 |
| ④ | `asinf` 输入没夹范围，浮点误差可能略超 ±1 ⇒ NaN | 先夹到 `[-1, 1]` |

## 5. 和 EKF 怎么选

| | **Mahony（本模块）** | **四元数 EKF**（`User/algorithm/Kalman`）|
|---|---|---|
| 算力 | 极小 | 大（矩阵运算）|
| 陀螺零偏 | 不估计（`Ki` 只能粗略压一压）⇒ **长时间缓慢漂移** | 状态里带 `b_x/b_y`，会估 |
| 异常量测 | Kp 硬拉 | 卡方门控 + 自适应增益 |
| 适合 | 先验证硬件通路 / 算力紧张 / 兜底 | 正式上车 |

> 建议：**先用 Mahony 把 BMI088 的符号、单位、安装方向验对**（它简单，出错容易定位），
> 再切到 EKF 看效果。两者 roll/pitch 应当基本一致；不一致 ⇒ 一定是某一路的单位
> （deg vs rad）或轴向符号错了。

## 6. 上板自检

```
① 静止放平:Roll/Pitch ≈ 0(rad);若差了固定一个角 ⇒ BMI088 安装/零偏没标定
② 绕某一轴转 90°:看是哪个角的绝对值变大、正负对不对 ⇒ 验轴向
③ 快速晃:角度应该跟得住而不是乱跳(乱跳 ⇒ Kp 太大或输入单位错了)
④ 静置 5 分钟:缓慢漂移是正常的(Mahony 不估零偏) ⇒ 要稳就上 EKF
```
