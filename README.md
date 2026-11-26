<p align="center">
  <img src="assets/logo.png" alt="天枢战队" width="210">
</p>

# 天枢战队-机器人控制系统开源

> 舵轮底盘 · 双 MCU（下位机 + 上板）· 一套代码 `robot_def.h` 一行切板 · 配套 PC/Ubuntu/Android 调试助手

STM32H723 舵轮机器人：**同一份代码烧两块板**，`robot_def.h` 里一行开关决定编哪块：

```c
#define ROBOT_BOARD_SEL   ROBOT_BOARD_LOWER   // 下位机(底盘)  ← 默认
#define ROBOT_BOARD_SEL   ROBOT_BOARD_UPPER   // 上板(遥控/云台pitch/摩擦轮)
```

| | 职责 | 这块板上挂的东西 |
|---|---|---|
| **下位机**（底盘 MCU） | 底盘解算、舵向/驱动闭环、云台 **yaw** 保持、拨弹、找零、日志 | 舵向 **202/204**、驱动 **201/203**、云台 yaw **6020(0x205)**、拨弹盘 **0x206**、底盘 IMU（**单颗** BMI088）|
| **上板**（云台 MCU） | 遥控接收、**裁判系统**、云台 **pitch**、**摩擦轮**、上下链路中继、调试分片转发 | **裁判系统 USART1(115200, PA9/PA10)**、遥控接收机、云台 pitch **DM4310**（CAN2, id1）、摩擦轮 **0x207/0x208**、云台 IMU（**单颗** BMI088）|

两块板各挂**一颗** BMI088（没有"双 IMU"的板子；`IMU_CHANNEL_GIMBAL` 在单 IMU 板上是通道 0 的镜像），
靠 **RS485 4 Mbps / 500 Hz 一发一回**通信；PC/Ubuntu/Android **调试助手**用 `0x5A` 遥测看波形、
`0x5C` 在线改参数。

> **裁判系统只在上板接**：下板没有裁判接收机，`Referee_Init()` 在下板被 `#if ROBOT_UPPER_ONLY` 跳过。
> 下板需要的裁判量（底盘功率上限）由命令帧 `MiniPC_Cmd_Payload_t.Reserved` 这一个字节带下来
> （`Cmd_485_Get_Power_Limit()`，0/超时 = 无裁判 ⇒ 用练习模式保守值）。
> 热量、发弹许可这些只影响发射的判定在上板完成，随 `Shoot_Enable` 一起下发。

```text
                       上板 MCU                                    下位机 MCU
 遥控器(SBUS) ───────►│ 裁判系统   USART1 115200(PA9/PA10)     ┌──► 舵向 202/204(3508+1:8)
                      │ 云台 pitch  DM4310(CAN2,id1)         ├──► 驱动 201/203(3508)
                      │ 摩擦轮      0x207 / 0x208(3508)      ├──► 云台 yaw 6020(0x205,世界朝向保持)
                      │ 云台 IMU    单颗 BMI088              ├──► 拨弹盘 0x206(3508)
                      │ 遥控解析 / 链路中继 / 调试分片        ├──► 底盘 IMU 单颗 BMI088
                      └──── 485 4 Mbps · 500 Hz 一发一回 ───┘    ├──► 底盘功率上限(Reserved 字节)
                                                                 └──► PE2/PE3 光电找零

  PC 调试助手 ◄──── 上板 UART7 / USART10 (921600)   ← 下位机遥测经 485 分片上送、由上板转出
```

---

## 1. 目录

```text
开源/
├─ LGRobotSystem/      固件工程(下位机 / 上板 同一份,robot_def.h 切换)
│  ├─ User/App/robot/robot_def.h   ← ★ 换车 / 换板 / 调参只改这里
│  ├─ User/algorithm/RobotSlutionCalculation/  底盘解算(独立模块)
│  └─ tools/                      串口抓数据 / 阶跃测试 / 波形解析脚本
├─ 调试助手/            PC / Ubuntu / Android 调试工具(单源码)
└─ 文档/                标定与联调记录
```

---

## 2. 快速开始

### 2.1 编译

需要 `arm-none-eabi-gcc` 和 **CMake ≥ 4.3**（STM32CubeCLT 1.21 / CLion 自带；系统里那个 3.28 会直接报
`CMake 4.3 or higher is required`）。

```powershell
cd 开源\LGRobotSystem
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 8
# 产物: build/LGRobotSystem.elf / .hex / .bin
```

**编哪块板**：打开 `User/App/robot/robot_def.h`，改一行

```c
#define ROBOT_BOARD_SEL  ROBOT_BOARD_LOWER   /* 下位机(默认) */
#define ROBOT_BOARD_SEL  ROBOT_BOARD_UPPER   /* 上板 */
```

两个配置都是编过的（Release：下位机 **186 972 B** / 上板 **189 360 B**），`Core/`、`Drivers/`、
`Middlewares/`（含 CMSIS-DSP 预编译库 `libarm_cortexM7lfsp_math.a`）全在树内，无外部依赖、无绝对路径。

> ⚠️ **必须放在纯 ASCII 路径下构建**（如 `D:\opensource\`）。路径带中文时 `mingw32-make` 会报
> `No rule to make target 'E:/.../开源/.../Core/Src/adc.c'`（make 的非 ASCII 路径限制），
> 换 `-G Ninja` 会直接卡住不报错。源码本身没问题（同一棵树拷到 ASCII 路径下编译通过）。

### 2.2 烧录

```powershell
openocd -f daplink.cfg -c "program build/LGRobotSystem.elf verify reset exit"
```

* `daplink.cfg` = CMSIS-DAP（`adapter driver cmsis-dap`），**推荐**；
* `stlink.cfg` = ST-LINK 的 `dapdirect` 模式。**别用** `interface/stlink-v2.cfg` + `hla_swd`：
  hla 适配器不支持 STM32H7，会卡在 `Info : clock speed ...` 不动。

### 2.3 调试助手

```powershell
python 开源\调试助手\调试助手.py        # 需要 pyserial；Windows 有 PyInstaller 脚本
```

详细用法见 [调试助手/README.md](调试助手/README.md)：串口/波特率自动匹配、字段表、
VOFA 风格波形、电机/舵轮/底盘子窗口、CSV/HEX/Markdown 导出。

> 仓库里只放**源码**：`调试助手/windows/调试助手.exe`（PyInstaller 打包）和
> `ubuntu/*.tar.gz` 这类二进制没打包进来，需要时按 `调试助手/windows/build.ps1` 自己生成。

---

## 3. ★ 本机配置总表（`User/App/robot/robot_def.h`）

换车、换板、调射速都只改这一段；各模块用**自己的老名字**引用这些值，内部代码不用动。

### 3.1 板卡

| 宏 | 值 | 说明 |
|---|---|---|
| `ROBOT_BOARD_SEL` | `ROBOT_BOARD_LOWER` | 一行切换下位机 / 上板 |
| `ROBOT_UPPER_ONLY` | `(SEL == UPPER)` | 派生量，源码里的 `#if ROBOT_UPPER_ONLY` 全靠它 |

### 3.2 底盘几何 [m]

| 宏 | 值 | 说明 |
|---|---|---|
| `CFG_CHASSIS_HALF_A` | 0.1625 | 舵轮前后半轴距（325 mm / 2）|
| `CFG_CHASSIS_HALF_B` | 0.1625 | 舵轮左右半轴距 |
| `CFG_CHASSIS_HALF_OMNI` | 0.183345 | 被动全向轮半距（不参与解算）|
| `CFG_CHASSIS_WHEEL_R` | 0.05 | 驱动轮半径 |
| `CFG_CHASSIS_HALF_DIAG` | 0.2298 | `sqrt(A²+B²)`，**改 A/B 后要跟着改** |

### 3.3 舵向 / 云台标定

| 宏 | 值 | 说明 |
|---|---|---|
| `CFG_STICK_TO_BODY_YAW_DEG` | 303 | 摇杆 y+ 在车体系的角度 |
| `CFG_GIMBAL_POS_SIGN` | +1 | 云台与舵向同向；装反才改 −1 |
| `CFG_SEAT_TRIM_DEG_202` | −10 | 202 舵向微调（叠加在光电找零结果上）|
| `CFG_SEAT_TRIM_DEG_204` | 0 | 204 同上 |
| `CFG_6020_ZERO_ENC` | 550 | 云台零点：raw 550 = 指向 +Y（车头正前）|

### 3.4 速度 / 功率

| 宏 | 值 | 说明 |
|---|---|---|
| `CFG_MAX_VX` / `CFG_MAX_VY` | 3.0 / 3.0 | 前后 / 左右最大速度 [m/s] |
| `CFG_MAX_W` | 8.0 | 最大旋转角速度 [rad/s] |
| `CFG_POWER_LIMIT` | 120 | 底盘功率上限 [W] |

### 3.5 发射（射速）

| 宏 | 值 | 说明 |
|---|---|---|
| `CFG_FRICTION_RPM` | 3000 | 摩擦轮目标转速 [rpm] |
| `CFG_TRIGGER_RPM` | 100 | 连发时拨弹盘转速 [rpm] |
| `CFG_TRIGGER_SLOTS` | 6 | 拨弹盘一圈几格（单发走一格）|

```text
射速 = CFG_TRIGGER_RPM / 60 × CFG_TRIGGER_SLOTS = 100/60×6 ≈ 10.0 发/s
```

> 控制环参数（PID、航向修正权、放行门限）仍留在各模块里，改动位置在
> [App/chassis/chassis_task.md](LGRobotSystem/User/App/chassis/chassis_task.md) 等文档中标注。

---

## 4. 关键链路参数

| 链路 | 参数 |
|---|---|
| 上↔下 | **USART2**（PD5=TX / PD6=RX / PD4=DE），**4 Mbps** 8N1，RS485 半双工，**54 B 固定帧，500 Hz 一发一回** |
| 调试输出 | 只有**上板**直发 UART（`DEBUG_UART_TX_ENABLE = ROBOT_UPPER_ONLY`）：**UART7**（PE8/PE7）+ **USART10**（PE3/PE2，与光电找零互斥），**921600** 8N1；下位机的遥测经 485 分片上送、由上板转出 |
| 调试遥测 | `0x5A`：`0x0102` 电机 PID 帧（100 Hz）/ `0x0104` 舵向帧（50 Hz，含 `heading_error`、实际 ω）/ JSON（20 Hz） |
| 在线调参 | `0x5C`（CRC-CCITT 反射版 `0x8408`，初值 0） |
| 裁判系统 | **USART1**（PA9/PA10）**115200** 8N1，CRC8(0x31)/CRC16(CCITT) 解析；**只在上板接**，下板靠 485 的 `Reserved` 字节拿功率上限 |
| 上位机/裁判/miniPC | `0xA5`（与 485 链路同帧格式） |

帧结构、字段偏移见 [User/通信协议.md](LGRobotSystem/User/通信协议.md)；
485 帧的 channel/mode 位定义见 [Device/comMCU/cmd_485.md](LGRobotSystem/User/Device/comMCU/cmd_485.md)。

---

## 5. 代码结构

```text
User/
├─ App/        robot(决策/模式/门控) chassis(D2Q2 解算) gimbal cmd(遥控+485) motor_task
│              debug(遥测/调参) imu error_task(日志) msg_center(话题) highRise
├─ Algorithm/  chassis_control(舵轮解算) RobotSlutionCalculation PID Kalman mahony Slope math_support
├─ Bsp/        fdcan uart spi ospi adc pwm rc power dwt timestamp flash fault
└─ Device/     motor(DJ/DM/LZ/YS + MIT) bmi088 remote referee minipc log ws2812 buzzer
```

分层铁律（见 [代码架构框架.md](LGRobotSystem/User/代码架构框架.md)）：
`Bsp` 不许出现"底盘/云台"这类业务词；`Device` 只管协议/寄存器；`App` 只管业务；
电机单位统一关节侧 `rad / rad·s⁻¹ / N·m`，rpm 换算只在一个地方做。

### 一块代码两块板

`#if ROBOT_UPPER_ONLY` / `#ifndef ROBOT_UPPER_ONLY` 共 19 处（另有 2 处只在文档里），集中在：

```text
App/cmd/cmd_task.cpp        命令来源(下板: 从 485 来)
Device/comMCU/cmd_485.cpp   上板发命令 / 下板收命令 + 回反馈(设备层)
App/debug/debug_task.cpp    UART 直发只在上板开(DEBUG_UART_TX_ENABLE)
App/motor_task/motor_task.cpp  DM_MOTOR_N(上板 1 / 下板 0) ⇒ 4310 只在 CAN2 上建对象
App/robot/robot.cpp         上板跳过 Chassis_Init() / 只在上板初始化裁判
App/robot/robot_def.h        ROBOT_UPPER_ONLY 派生 + 电机分配表
```

其余模块两块板共用同一套框架（`Bsp`/`Device`/`Algorithm`/`msg_center` 完全一样），
**但两块板各自挂的电机不同**，靠 `robot_def.h` 里的电机分配表区分：

| 项 | 下位机 | 上板 |
|---|---|---|
| 遥控接收机(SBUS) | 不接 | **接**（下板命令全从 485 来）|
| 裁判系统(USART1 115200) | 不接（`Referee_Init` 被 `#if ROBOT_UPPER_ONLY` 跳过）| **接** |
| 云台 yaw | **6020**（`aux_group[1]`, 0x205, CAN1）| 不驱动（`GIMBAL_MOTOR_YAW = 0xFF`）|
| 云台 pitch | 不驱动 | **DM4310**（`MOTOR_DM_J4310`, CAN2, id1, MIT 位置 + 重力前馈）|
| 摩擦轮 | 同框架（物理上没接）| **0x207 / 0x208**（`aux_group[2]/[3]`，`motor_task_friction_*` API）|
| 拨弹盘 | **0x206**（`aux_group[0]`）| 不驱动 |
| 舵向/驱动 | 202/204 + 201/203 | 不驱动（`Chassis_Init()` 在上板被 `#if !ROBOT_UPPER_ONLY` 跳过）|
| IMU | 底盘 IMU（单颗 BMI088）| 云台 IMU（单颗 BMI088，含 `Imu_Zero_Pitch_Now()` 关节零偏）|
| 裁判数据下发 | 收：功率上限（`Cmd_485_Get_Power_Limit`）| 发：`Reserved` 字节 = 功率上限 [W] |

板级差异全部用 `#if ROBOT_UPPER_ONLY` 收口，共 4 类：

```text
App/robot/robot_def.h        ROBOT_BOARD_SEL / GIMBAL_MOTOR_PITCH(0xFE) / 板卡派生宏
App/robot/robot.cpp          上板跳过 Chassis_Init()(不等舵向找零)
App/motor_task/motor_task.cpp DM_MOTOR_N(上板 1/下板 0) + CAN2_Callback + motor_task_4310_*
App/cmd/cmd_task.cpp、Device/comMCU/cmd_485.cpp、App/debug/debug_task.cpp   命令来源 / 485 收发 / UART 直发
```

### 上板第一次烧录前的检查清单（重要）

上板代码是**从 11-20 版旧上板工程逐字节搬过来**的（`gimbal_task.cpp` 与旧工程完全相同），
但换了新框架，第一次上电请按这个顺序验：

1. **架空**：车体垫起、云台手扶住，再上电
2. 看 `m6020.online` / `4310 online`（调试助手 `0x0102` 帧）——没有 4310 反馈 = CAN2/ID 不对，先别放云台
3. 推 pitch：云台应平滑跟随、松手不回落（重力前馈生效）；`MOTOR_DM_ENABLE_ON_INIT = 0`，
   使能由 `gimbal_task` 第一次发目标时触发，所以**上电后不动摇杆时 DM4310 是松的**，属正常
4. 推 shoot：摩擦轮 0x207/0x208 起转；拨弹盘在下位机（0x206），要两块板都在线
5. 上板 `pitch` 限位沿用旧实测值 `-0.53 / +0.53 rad`、`GIMBAL_KP = 25`；要改就改
   `gimbal_task.cpp` 顶部那一段（下位机不驱动 pitch，不受影响）

---

## 6. 文档索引

| 文档 | 内容 |
|---|---|
| [User/通信协议.md](LGRobotSystem/User/通信协议.md) | `0x5A`/`0xA5`/`0x5C` 全部帧格式与字段偏移 |
| [User/代码架构框架.md](LGRobotSystem/User/代码架构框架.md) | 分层、任务、数据流 |
| [User/写代码的思想.md](LGRobotSystem/User/写代码的思想.md) | 踩过的坑、模板 |
| [algorithm/chassis_control/舵轮_舵向定义.md](LGRobotSystem/User/algorithm/chassis_control/舵轮_舵向定义.md) | seat 定义、2π 归一、优劣弧、减速比 |
| [App/chassis/chassis_task.md](LGRobotSystem/User/App/chassis/chassis_task.md) | 底盘任务：解算、航向环、功率 |
| [Device/comMCU/cmd_485.md](LGRobotSystem/User/Device/comMCU/cmd_485.md) | 上下链路 500 Hz 一发一回 |
| [App/debug/debug_task.md](LGRobotSystem/User/App/debug/debug_task.md) | 遥测帧与调参命令 |
| [Device/motor/电机使用说明.md](LGRobotSystem/User/Device/motor/电机使用说明.md) | DJ/DM/LZ/YS 四品牌单位与命令位 |
| [algorithm/RobotSlutionCalculation/RobotSlutionCalculation.md](LGRobotSystem/User/algorithm/RobotSlutionCalculation/RobotSlutionCalculation.md) | 底盘解算独立模块（纯数学层，与 App 实际解算的关系）|
| [文档/标定与联调记录.md](文档/标定与联调记录.md) | 6020 零点、toe-in、标定的实测数据 |
| [调试助手/README.md](调试助手/README.md) | 调试助手全功能说明 |

---

## 7. 许可

MIT，见 [LICENSE](LICENSE)（随工程自带）。
