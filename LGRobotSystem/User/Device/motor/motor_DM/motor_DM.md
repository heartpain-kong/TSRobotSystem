# motor_DM — 达妙（DM）J10010L / J4310 / G6220 的 MIT 协议实现

达妙用标准 8 字节 MIT 帧，而且 **PD 在电机固件里跑、主机不跑 PID** —— 主机每帧只送
(位置, 速度, 力矩, Kp, Kd) 五个设定，电机自己闭环。所以本类**没有 `Class_PID` 成员**：
App 把 Kp/Kd 当“刚度 / 阻尼”参数下发，`send_data.T` 就是前馈力矩。
一台一帧（标准帧 id = 电机 CAN_ID），反馈帧走 Master ID。

它是 `motor_mit` 统一电机层里的达妙分支，被 App 电机注册表 + 运控任务使用。

## 怎么用

初始化一次，然后任务里排程发帧：

```cpp
static Class_Motor_DM m;
m.Init(&hfdcan1, 0x01, MOTOR_DM_J4310, motor_pos, Motor_Control_Freq_500HZ);
m.enable();                        // 发 FF..FC 使能帧（同时打开 send() 的闸门）
m.Set_Pos(0.0f); m.Set_W(0.0f); m.Set_T(0.0f);
m.Set_Kp(30.0f); m.Set_Kd(1.0f);   // Kp/Kd 由电机侧执行
if (m.Is_Send_Due()) m.send();     // 只发；闸门判断在 send() 内部看 Send_Status_Cmd
m.zero();  m.lose();               // 置零 FF..FE，失能 FF..FD
// 接收回调：CAN RX 里 m.recv(id, data, len); 读值用 Get_Now_Pos/W/T()、Get_MError()
```

发帧路径：`send()` → `MIT_Pack()` → `Transmit()`，发完 `Mark_Sent()`。
收帧路径：`recv()` → `MIT_Parse()`，故障码读 `Get_MError()`。
底层两个函数也可单独用：`MIT_Pack(uint8_t _data[8])` / `MIT_Parse(const uint8_t *_data)`。

## 注意

* 单位关节侧 `rad` / `rad·s⁻¹` / `N·m`；组帧前按型号限幅，结构体限幅全 0 时套型号默认
  （J4310 ±12.5rad / ±30rad·s⁻¹ / ±10N·m，Kp 0~500、Kd 0~5）—— 这些 PMAX/VMAX/TMAX
  **必须与达妙上位机里设的一致**，否则定点换算整体偏比例。
* 特殊帧 = 8 个 `0xFF` + 末字节：`0xFC` 使能 / `0xFD` 失能 / `0xFE` 置零 / `0xFB` 清错；
  `enable()` / `lose()` 同时改命令位 `Send_Status_Cmd`（`send()` 的闸门看它）。
* 反馈帧 `_data[0] >> 4` 是**状态 / 故障码（ERR）**，不是错误标志位；只有 `ERR == 1` 才算
  “确实已使能”。别用 `Motor_Status` 去关发送闸门（某些故障要靠继续收帧才能恢复）。
* **发送前会自检"是否已使能"**（`send()` 里），因为达妙不发使能帧就不出力矩、
  而且它的**通信看门狗**停发一会儿就会自己失能（`ERR=0xD`）：

  | 情况 | `send()` 的动作 |
  | --- | --- |
  | 命令位 `Send_Status_Cmd != ENABLE`（`lose()`/急停过）| **不发控制帧、也不补使能** —— 尊重急停，绝不"自愈"回使能 |
  | 命令位是 ENABLE，但反馈 `ERR != 1`（掉线等），且**无真故障** | 补发一次 `0xFC` 再发控制帧（看门狗掉线自动恢复）|
  | 有**真故障**（`ERR` = `0x8`/`0x9`/`0xA`/`0xB`/`0xC`/`0xE`）| 不发、不使能，等上层 `Clear_Error()` + `enable()` |

  判定用 `Has_Fault()` / `Is_Enabled()`（`ERR=0xD` 通信丢失算可自愈，不算"真故障"）。
* 带宽：一台一帧（≈111bit）+ 电机自身反馈帧，单总线 1Mbps 下 4 台 @1kHz 控制 + 反馈 ≈88%，
  频率建议 ≤500Hz。

## 特殊帧一览

| 帧（data[0..7]） | 作用 | 接口 / 数组 |
|---|---|---|
| `FF..FC` | 使能 | `enable()` / `motor_send_Data_enable` |
| `FF..FD` | 失能 | `lose()` / `motor_send_Data_lose`（之后还会清空 `send_data`） |
| `FF..FE` | 置机械零点 | `zero()` / `motor_send_Data_zero` |
| `FF..FB` | 清除电机错误 | `Clear_Error()` / `motor_send_Data_clear_error` |

* 清错帧用于：电机报故障后（ERR = `0x8` 过压 / `0x9` 欠压 / `0xA` 过流 / `0xB` MOS 过温 /
  `0xC` 线圈过温 / `0xD` 通信丢失 / `0xE` 过载），即使故障源已经消失电机也**不会自己恢复**，
  必须发这一帧清错，再 `enable()` 才会重新接受控制。达妙自带通信超时保护，主机看门狗不能替代它。
* 失能 / 置零 / 清错都直接 `Transmit`，不经过 `send()`，所以**不受发送周期闸门限制**。

## 型号默认限幅（`Apply_Model_Limit()`）

只有在 `Struct_motor_init` 里没给限幅（`Pos_max == 0 && Pos_min == 0`）时才套用：

| 型号 | `Pos` [rad] | `W` [rad·s⁻¹] | `T` [N·m] | `Kp` | `Kd` |
|---|---|---|---|---|---|
| `MOTOR_DM_J10010L` | ±12.5 | ±25 | ±200 | 0 ~ 500 | 0 ~ 5 |
| `MOTOR_DM_J4310` | ±12.5 | ±30 | ±10 | 0 ~ 500 | 0 ~ 5 |
| `MOTOR_DM_G6220` | ±12.5 | ±45 | ±10 | 0 ~ 500 | 0 ~ 5 |
| 其它（`default`） | ±12.5 | ±30 | ±10 | 0 ~ 500 | 0 ~ 5 |

## 接口一览

| 接口 | 作用 | 备注 |
|---|---|---|
| `Init(hfdcan, __id, __model, __control_mode, __freq)` | 初始化（按型号） | `__freq` 必填；`Set_Control_Freq()` 算出周期并让首帧立即到期，同时 `Apply_Model_Limit()` 定量程 |
| `send()` | 发送一帧 MIT 控制量 | **发送前自检使能**：命令位 ENABLE 且 `ERR!=1` 且无真故障 ⇒ 先补 `0xFC`；之后 `MIT_Pack()` + `Transmit()` + `Mark_Sent()` |
| `recv(_id, _data, _len)` | 解析一帧 MIT 反馈 | 要求 `_len >= 8`；`_id` 当前未使用 |
| `zero()` / `enable()` / `lose()` | 置零（`FF..FE`）/ 使能（`FF..FC`）/ 失能（`FF..FD`） | 都直接发特殊帧 |
| `Clear_Error()` | 清除电机错误（`FF..FB`） | 故障排除后要清错 + 重新 `enable()`（否则 `send()` 一直判为有故障、不发帧）|
| `Has_Fault()` / `Is_Enabled()` | 判定"有真故障" / "反馈说已使能" | 供 `send()` 自检;`0xD` 通信丢失算可自愈、不算真故障 |
| `Get_CAN_Id()` | 本电机的 CAN id（发送用） | 读 `motor_init.id` |
| `MIT_Pack(uint8_t *)`（`protected`） | MIT 组帧 | 先限幅再定点量化：位置16 / 速度12 / Kp12 / Kd12 / 力矩12 |
| `MIT_Parse(const uint8_t *)`（`protected`） | MIT 解帧 | 反馈帧排布与控制帧**不同**：力矩紧跟速度，没有 Kp/Kd |
| `Transmit(_id, _data, _len)`（`protected`） | 按通信方式发一帧 | 走 `fdcan_send_data_stand()` 或 `UART_Transmit_Data()` |
| `Apply_Model_Limit(model)`（`protected`） | 套用型号默认限幅 | 见上表 |

## 相关

* 依赖：`motor_mit.h`（基类 / 调度）、`bsp_fdcan.h`、`bsp_dwt.h`、`math_support.h`、
  `struct_typedef.h`。
* **MIT 帧字段表、ERR 码表**、频率与闸门的完整说明见 `motor_DM.h` 内的协议注释
  （本文件不重复），以及 `motor_mit.h` 的“命令状态 vs 实际状态（必须分开！）”一节。
* 反馈帧解析要点：`data[1..2]` 位置16、`data[3] + data[4]>>4` 速度12、
  `data[4]&0xF + data[5]` 力矩12；温度取 `max(data[6], data[7])`（MOS / 线圈取较大者）。
* `recv()` 里 `Motor_Status = (MError == 1)` 只是**约定**：反馈帧本身没有“模式”字段。
