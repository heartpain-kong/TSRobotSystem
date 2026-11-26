# imu_task — 姿态解算任务（BMI088 + 四元数 EKF）

文件：`imu_task.h` / `imu_task.cpp`
任务：**1 kHz（1 ms）**、优先级 `osPriorityHigh`（与电机任务同级 ✓）、栈 2048 B

---

## 1. 它在流程图里的位置

```
BMI088(SPI2) ─► dev_bmi088 ─► Imu_Task() ─┬─► MSG_TOPIC_IMU_CHASSIS ─► chassis / gimbal
                          (四元数 EKF)     ├─► MSG_TOPIC_IMU_GIMBAL  ─► gimbal
                                           └─► error_task 的关键数据快照(10Hz 落盘)
```

## 2. 任务里每拍做什么（1 kHz）

```
① BMI088_Task()            推进设备状态机(超时/在线判定)
② dt ← TIM5 微秒时间戳      限幅 100µs ~ 20ms(卡顿/重复调用保护)
③ BMI088_Read() + Get_Gyro/Get_Accel   阻塞 SPI,几十 µs;读失败不喂 EKF
④ s_ekf.Update(gx,gy,gz,ax,ay,az,dt)   四元数 EKF 融合
⑤ 发布 IMU_CHASSIS + IMU_GIMBAL 两个话题
⑥ 写 Error_Get_Snapshot()->Imu_Euler[]  (记的是**连续 yaw** ✓)
⑦ IMU 异常 → Error_Set_Flag(ERROR_FLAG_IMU)
```

> 设备初始化（`BMI088_Init()`）在 `Robot_Init()` 里 ✓；本任务只负责**跑** ✓
> `BMI088_Task()` **由本任务调用** ✗ ⇒ 别在 `Robot_Task()` 里再调 ✓

## 3. EKF 参数（照 chaiss/官方调好的一组）

| 参数 | 值 | 说明 |
|---|---|---|
| `Q1` | 10.0 | 四元数过程噪声 |
| `Q2` | 0.001 | 陀螺零偏过程噪声 |
| `R` | 1000000.0 | 加速度计量测噪声（越大越信陀螺）|
| `λ` | 1.0 | 渐消因子（1 = 不渐消）|
| `lpf` | 0.0 | 加速度低通（0 = 不滤）|

dt 限幅：`IMU_DT_MIN_US 100`、`IMU_DT_MAX_US 20000` ✓

## 4. 发布的数据 `Imu_Data_t`（单位一律 SI）

| 字段 | 单位 | 用途 |
|---|---|---|
| `Euler[3]` | rad | roll / pitch / **yaw（−π~π，会跳变）** |
| **`Yaw_Total`** | rad | **连续 yaw，不跳变 ⇒ 小陀螺 / 云台用这个** ✓ |
| `Gyro[3]` | rad/s | 原始角速度（未去零偏）|
| `Accel[3]` | m/s² | 原始加速度 |
| `Quat[4]` | — | 四元数 w,x,y,z |
| `Gyro_Bias[3]` | rad/s | EKF 估的陀螺零偏 |
| `Temp_C` | ℃ | 温度 |
| `ChiSquare` | — | 卡方值：越大说明模型/测量越不一致 |
| `Update_Count` / `Error_Count` | — | EKF 跑了几拍 / 卡方门控拒了几次 |
| `Online` / `Converged` | — | BMI088 在线 / EKF 已收敛 |
| `Seq` | — | 每拍 +1 |

## 5. 两个通道（本板只有一颗 IMU）

* `IMU_CHANNEL_CHASSIS` = 0 ✓（真实数据）
* `IMU_CHANNEL_GIMBAL` = 1 ✓ —— **目前镜像通道 0** ✓（板上只有一颗 BMI088，SPI2 的 PC0/PC3 是同一颗的加速度/陀螺片选）
  以后加了第二颗，就把 `imu_task.cpp` 里"镜像"那两行换成对第二颗的读+EKF ✓

## 6. API

| 接口 | 说明 |
|---|---|
| `Imu_Init()` | 初始化 EKF（`Robot_Init()` 里调）|
| `Imu_Task()` | 跑一拍（IMU 任务 1 kHz 调）|
| `Imu_Get(ch)` | 读姿态数据 `const Imu_Data_t*` |
| `Imu_Is_Online(ch)` / `Imu_Is_Converged(ch)` | 在线 / 收敛 |
| `Imu_Get_Yaw_Total(ch)` | 连续 yaw（rad）|

订阅方（chassis/gimbal）请用**广播式**读法：
```cpp
static uint32_t my_seq = 0;
Imu_Data_t imu;
if (Msg_Is_New(MSG_TOPIC_IMU_CHASSIS, my_seq) &&
    Msg_Latest(MSG_TOPIC_IMU_CHASSIS, &imu, sizeof(imu), &my_seq)) { /* 用 imu */ }
```

## 7. 坑

1. **`sizeof(Imu_Data_t)` 必须 ≤ `MSG_SLOT_SIZE`** ✗（现在 ~92 B / 上限 96 B）⇒ 已在 `imu_task.cpp` 里加了 `static_assert` ✓，改结构体时编译期就会报 ✓
2. **1 kHz 跑 EKF 偏重** ✗ ⇒ 看 `imu_task_overrun`：一直涨就把 `IMU_TASK_PERIOD_MS` 改成 2（500 Hz）✓
3. **`Yaw_Total` 才是连续的** ✗ ⇒ `Euler[2]` 在 ±π 处会跳 ✓（这正是 `Class_chassis_*::Set_Top_Yaw` 要处理的东西 ✓）
4. **EKF 上电需要几秒收敛** ✗ ⇒ 静止放着等 `Converged == true` 再起飞 ✓（收敛前 `Error_Flag_IMU` 不会报，避免上电一直橙灯 ✓）
5. **`BMI088_Read()` 是阻塞 SPI** ✗ ⇒ 不放在 1 kHz 以上的任务里 ✓
6. **不要两个任务都调 `Imu_Task()`** ✗（会 dt 错乱 + 重复发布 ✓）

## 8. 上板自检

```
① Imu_Init() 后:Imu_Is_Online() 应为 true(BMI088 已初始化);Update_Count 开始涨
② 静止几秒 ⇒ Converged == true,此时 Euler 的 roll/pitch ≈ 0(与板子实际水平度一致)
③ 绕 z 轴慢慢转:Yaw_Total 连续变化(转过 ±π 不跳);Euler[2] 会在 ±π 处跳(正常 ✓)
④ 倾斜板子:Euler[0]/[1] 跟着变,符号与方向一致
⑤ 拔掉 IMU:Online 变 false ⇒ error 任务报码 7 或 81(IMU 位),灯变橙
⑥ imu_task_overrun 应保持 0;一直涨 ⇒ 把任务周期改成 2ms
⑦ Msg_Get_Published(MSG_TOPIC_IMU_CHASSIS) ≈ 1000/s(按 1kHz)
```
