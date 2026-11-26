# motor_mit — 四类电机（DJ / DM / YS / LZ）的统一抽象基类

`motor_mit` 是统一电机层的基类：只把五个纯虚接口 `send()` / `recv()` / `zero()` /
`enable()` / `lose()` 留给各品牌按自己的协议实现，其余（设定值、反馈读取、使能状态、
发送周期调度）全在基类用**非虚成员 + inline 实现**给好。

于是上层 App 只认这一个类型：写好 `Set_Pos/W/T/Kp/Kd` 与 `Get_Now_Pos/W/T` 就能换任意
品牌电机，不碰 FDCAN / UART 协议。（MIT 协议是达妙电机的实现，在 `motor_DM` 里，不在基类。）

## 怎么用

App 侧只剩三件事：**设定值 → 发帧 → 读反馈**。

```cpp
motor->Set_Pos(1.0f); motor->Set_W(0.0f); motor->Set_T(0.0f);
motor->Set_Kp(30.0f); motor->Set_Kd(1.0f);

if (motor->Is_Send_Due()) motor->send();   // 调度分工：状态在电机里，判断在 App 里
fp32 pos = motor->Get_Now_Pos();           // 反馈只读 Get_Now_*，不用自己解协议
```

* 初始化 / 频率：`motor->Init(...)` 内部调 `Set_Control_Freq()`；运行中改频率直接再调它。
* 状态：`enable()` / `lose()` 改的是**命令位** `Send_Status_Cmd`（可读 `Get_Send_Status()`）；
  电机**实际**状态看 `Get_Motor_Status()`。

## 接口一览

### 纯虚接口（只声明不实现，各品牌各自重写）

| 接口 | 作用 | 备注 |
|---|---|---|
| `send()` | 按当前设定值组帧并发送一帧控制量 | 内部负责 `Mark_Sent()` 记发送时刻 |
| `recv(uint32_t _id, const uint8_t *_data, uint8_t _len)` | 解析一帧反馈数据 | `_id` = 收到帧的 id（UART 电机可传 0） |
| `zero()` | 设置机械零点 | — |
| `enable()` | 电机使能 | 同时置命令位 |
| `lose()` | 电机失能 | 同时置命令位 |

### 设定值（关节侧单位）

| 接口 | 作用 |
|---|---|
| `Set_Pos()` / `Get_Pos()` | 位置设定值 |
| `Set_W()` / `Get_W()` | 角速度设定值 |
| `Set_T()` / `Get_T()` | 力矩设定值 |
| `Set_Kp()` / `Get_Kp()` | 刚度（MIT 类电机随帧下发） |
| `Set_Kd()` / `Get_Kd()` | 阻尼（MIT 类电机随帧下发） |
| `Set_Send_Data()` / `Get_Send_Data()` | 整包读写统一发送量 `Struct_send_motor` |

### 反馈读取（默认实现直接读公共的 `recv_data`）

| 接口 | 作用 | 备注 |
|---|---|---|
| `Get_Now_Pos()` / `Get_Now_W()` / `Get_Now_T()` | 读关节侧位置 / 角速度 / 力矩 | 带品牌私有反馈结构的电机可重写 |
| `Get_Motor_Temperature()` | 读温度 | — |
| `Get_MError()` | 读故障码 | — |
| `Get_Feedback_Mode()` | 读反馈里的模式字段 | — |
| `Set_Recv_Data()` / `Get_Recv_Data()` | 整包读写统一反馈量 `Struct_recv_motor` | — |

### 状态与控制模式

| 接口 | 作用 | 备注 |
|---|---|---|
| `Get_Send_Status()` | 读**命令位** `Send_Status_Cmd` | `send()` 的闸门看它 |
| `Set_Motor_Status()` / `Get_Motor_Status()` | 写 / 读**实际**状态 `Motor_Status` | 无模式回读的品牌（达妙 / 大疆）只能镜像命令值 |
| `Set_Motor_Control_mode()` / `Get_Motor_Control_mode()` | 读写控制模式 | `motor_pos` / `motor_speed` / `motor_T` |

### 发送周期调度

| 接口 | 作用 | 备注 |
|---|---|---|
| `Set_Control_Freq()` / `Get_Control_Freq()` | 设置 / 读发送频率 | 顺便算出周期，并把首帧置为立即到期 |
| `Get_Control_Period_Us()` | 当前发送周期 `[us]` = 1000000 / 频率 | — |
| `Get_Send_Time()` | 上一次发送的时刻 `[us]` | `DWT_Get_Us32()` 的值域，会回绕 |
| `Get_Send_Interval_Us()` | 距上次发送过了多久 `[us]` | 只读；无符号相减，回绕安全 |
| `Is_Send_Due()` | 这一拍该不该发 | **只读、不改任何状态** |
| `Mark_Sent()`（`protected`） | 记录本拍时刻 | 各品牌 `send()` 把帧真正发出去之后调用 |

## 统一数据契约

| 类型 | 内容 |
|---|---|
| `Struct_send_motor` | `Pos` / `T` / `W` / `Kp` / `Kd` |
| `Struct_recv_motor` | `MError` / `mode` / `Now_Pos` / `Now_W` / `Now_T` / `Temperature` / `CurrentValue` |
| `Struct_motor_init` | 通信方式、`id` / `master_id`、控制模式、Kp/Kd/T/Pos/W 的上下限、`FDcan` / `UART`、`Control_Freq` |
| `Enum_Motor_Control_Freq` | 频率档位，**枚举值直接就是 Hz**：`Motor_Control_Freq_100HZ` / `200HZ` / `500HZ` / `1000HZ` |
| `Enum_Motor_Status` | `Motor_Status_DISABLE` / `Motor_Status_ENABLE` |
| `motor_Control_mode` | `motor_pos` / `motor_speed` / `motor_T` |
| `Enum_communication` | `Enum_UART` / `Enum_FD_CAN` |

初始化参数里的**限幅字段留 0** 表示“用该型号的默认限幅”（见各品牌 `Init` 实现）；
`Struct_motor_init` 里的 `Control_Freq` 只是给上层读的副本，真正决定“多久发一帧”的是基类的
`Control_Freq` / `Control_Period_Us`。

## 注意

* **单位统一在关节侧（输出端）**：`Pos [rad]`、`W [rad/s]`、`T [N·m]`；转子侧换算由各品牌实现
  内部完成，上层不要再除减速比。
* **调度分工**：`send()` 内部 `Mark_Sent()` 记录 `send_time`（`DWT_Get_Us32()`，us）；
  `Is_Send_Due()` 是**只读**判断，“先判断再 `send()`”是安全的，不会双重门控。
* **Recv 上下文**：`recv()` 通常由 CAN 接收回调调用，别在里面做耗时 / 浮点重活。
* **反馈不保证每帧都有效**：协议里没有模式回读的品牌（达妙 / 大疆）只能把 `Motor_Status`
  镜像成命令值，别拿它当“电机真的转起来了”的依据。
* **频率档位**只列能整除 1000 的值，1ms tick 下有效上限 1000Hz。

## 相关

* 基类依赖：`bsp_fdcan.h` / `bsp_uart.h`（通信）、`bsp_dwt.h`（`DWT_Get_Us32` 时基）、
  `struct_typedef.h`（`fp32` 等）。
* 派生类：`motor_DJ` / `motor_DM` / `motor_LZ` / `motor_YS`。
* 更细的说明见头文件内对应小节：“命令状态 vs 实际状态（必须分开！）”、
  “控制周期（调度用，不是协议的一部分）”、`Enum_Motor_Control_Freq` 的注释。

## 文件构成

* `motor_mit.h`：枚举 / 结构体 / 类声明 + 全部 inline 实现（纯接口的公共部分）。
* `motor_mit.cpp`：只有特殊帧数组的定义与 4 个换算 / 限幅工具函数 ——
  `uint_to_float()`、`float_to_uint_f()`、`float_to_uint_i()`、`motor_max_min()`
  （声明同时出现在 `math_support.h`），以及 4 个 8 字节特殊帧数组
  `motor_send_Data_lose` / `motor_send_Data_enable` / `motor_send_Data_zero` /
  `motor_send_Data_clear_error`（末字节 `0xFD` / `0xFC` / `0xFE` / `0xFB`）。
