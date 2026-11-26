# motor_LZ — 灵足(LZ)RS 系列电机驱动:CAN **扩展帧**、一帧一台,接 `motor_mit`

灵足电机的协议层:把统一的关节点位/速度/力矩(rad、rad·s⁻¹、N·m)编成 16 bit 定点的**扩展帧**发出去,并把反馈帧解回 `recv_data`。运控帧自带 (Pos, W, T, Kp, Kd),所以两种用法都支持:

1. 只给 T(Kp = Kd = 0)= 主机闭环;
2. 给 Pos + Kp/Kd = 电机内部 PD。

另外实现官方参数操作(读 / 写 / 保存 / 运行模式 / 置零),用于标定与掉电保存。

| 文件 | 内容 |
| --- | --- |
| `motor_LZ.h` | 类声明、扩展帧类型码 `enum canComMode`、量程/运行模式/pattern/温度窗口等宏 |
| `motor_LZ.cpp` | `Class_Motor_LZ` 实现:组帧 / 解帧、命令载荷、参数操作 |
| `motor_mit.h` | 基类、统一单位与调度接口(`send_data` / `recv_data`、`Is_Send_Due()`、`Mark_Sent()`) |

## 一、型号分派

`Init()` 传入 `motor_LZ_Model`(`MOTOR_LZ_00` … `MOTOR_LZ_06`,对应官方的 `rs-00` … `rs-06`),
`Apply_Model_Limit()` 按型号补 16 bit 编码量程 + Kp/Kd 上限。

**全部数值来自官方 Python 驱动 `DM_RS-main/constants.py`**(`MODEL_MIT_*_TABLE` / `KP_TABLE` / `KD_TABLE`),
已与源码逐项核对一致:

| 型号 | Pos | W | T | Kp / Kd |
| --- | --- | --- | --- | --- |
| `MOTOR_LZ_00`(rs-00) | ±12.57(≈4π) | ±50.0 | ±17.0 | 0 ~ 500 / 0 ~ 5 |
| `MOTOR_LZ_01`(rs-01) | ±4π | ±44.0 | ±17.0 | 0 ~ 500 / 0 ~ 5 |
| `MOTOR_LZ_02`(rs-02,**默认型号**) | ±`MOTOR_LZ_POS_MAX_DEFAULT` | ±`MOTOR_LZ_W_MAX_DEFAULT` = ±44.0 | ±`MOTOR_LZ_T_MAX_DEFAULT` = ±17.0 | 0 ~ `MOTOR_LZ_KP_MAX`(500)/ 0 ~ `MOTOR_LZ_KD_MAX`(5) |
| `MOTOR_LZ_03`(rs-03) | ±4π | ±50.0 | ±60.0 | 0 ~ **5000** / 0 ~ **100** |
| `MOTOR_LZ_04`(rs-04) | ±4π | ±15.0 | ±120.0 | 0 ~ **5000** / 0 ~ **100** |
| `MOTOR_LZ_05`(rs-05) | ±4π | ±33.0 | ±17.0 | 0 ~ 500 / 0 ~ 5 |
| `MOTOR_LZ_06`(rs-06) | ±4π | ±20.0 | ±60.0 | 0 ~ **5000** / 0 ~ **100** |
| `default`(型号不明) | 同 rs-02 | 同左 | 同左 | 同左 |

> ⚠️ `MOTOR_LZ_05` 原来写的是 `W=50 / T=5.5`,与官方表(`33 / 17`)**不符**,已按官方修正。
> ⚠️ Kp/Kd 上限**按型号不同**:小电机(00/01/02/05)是 500/5,大电机(03/04/06)是 5000/100 ——
> 给大电机填 5000 的 Kp 却被夹到 500,位置环会明显偏软。

* ⚠️ 这里的 PMAX / VMAX / TMAX 是 **16 bit 定点编码所必需的协议量程,不是关节软限位**;必须与上位机里设置的参数一致,否则组帧换算出来的位置/速度/力矩会整体偏比例。
* 若结构体里已经给过 `Pos_max` / `Pos_min`(非 0),`Apply_Model_Limit()` 直接返回,不覆盖。
* 相关宏:`MOTOR_LZ_POS_MAX_DEFAULT 12.57f`(≈4π)、`MOTOR_LZ_W_MAX_DEFAULT 44.0f`、`MOTOR_LZ_T_MAX_DEFAULT 17.0f`、`MOTOR_LZ_KP_MAX 500.0f`、`MOTOR_LZ_KD_MAX 5.0f`。
* 量程夹紧(`motor_max_min()`)只作用于局部量,不改动 `send_data`。

## 二、帧格式(扩展帧)

`| 类型(5bit) | 数据段(16bit) | 电机ID(8bit) |` = bits 28-24 / bits 23-8 / bits 7-0,组装见 `make_frame_id()`:

```c
((uint32_t)mode << 24) | ((Id_data & 0xFFFFu) << 8) | (uint32_t)(motor_init.id & 0xFFu)
```

| 帧 | 类型 | 数据段(bits 23-8) | 数据区(8 B) |
| --- | --- | --- | --- |
| 运控帧 | `CANCOM_MOTOR_CTRL` = 0x01 | 力矩 | `[0..1]` 位置、`[2..3]` 速度、`[4..5]` Kp、`[6..7]` Kd |
| 命令帧 | 0x03 / 0x04 / 0x06 / 0x11 / 0x12 / 0x16 / 0x18 … | 主机(上位机)CAN ID | 见各命令载荷 |
| 反馈帧 | 0x02 指令应答 / 0x18 主动上报 | 电机 ID | 见「反馈解析」 |

* 主机(上位机)CAN ID = `motor_LZ_user_id`(本工程 0x00;官方样例用过 0xFD / 0x1F),`Init()` 写进 `motor_init.master_id`,由 `Get_Host_Id()` / `Set_Host_Id()` 读写。
* ⚠️ 命令帧与反馈帧的 bits 15-8 含义**不一样**(一个放主机 ID、一个放电机 ID),接收分发请用 `Is_My_Feedback(_id)`(`((_id >> 8) & 0xFF) == motor_init.id`)。

### 类型码 `enum canComMode`(扩展帧 ID 的 bits 28-24)

| 值 | 名称 | 说明 |
| --- | --- | --- |
| 0x00 | `CANCOM_ANNOUNCE_DEVID` | 通告设备 ID / 获取 ID |
| 0x01 | `CANCOM_MOTOR_CTRL` | 电机控制 = 运控模式指令(唯一在用的控制帧) |
| 0x02 | `CANCOM_MOTOR_FEEDBACK` | 电机反馈(方向是电机 → 主机) |
| 0x03 | `CANCOM_MOTOR_IN` | 电机使能 |
| 0x04 | `CANCOM_MOTOR_RESET` | 电机失能 / 停止(`data[0]=1` 附带清故障) |
| 0x05 | `CANCOM_MOTOR_CALI` | 官方未定义,勿用(旧代码遗留) |
| 0x06 | `CANCOM_MOTOR_ZERO` | 设置机械零位(`data[0]=1`) |
| 0x07 | `CANCOM_MOTOR_ID` | 设置 CAN ID |
| 0x11 | `CANCOM_PARA_READ` | 参数-读取 |
| 0x12 | `CANCOM_PARA_WRITE` | 参数-写入 / 设置控制模式(掉电丢失) |
| 0x15 | `CANCOM_FAULT_WARN` | 故障与警告反馈 |
| 0x16 | `CANCOM_MODE_TOTAL` | 电机数据保存到 Flash(数据区必须 01..08) |
| 0x17 | `CANCOM_MODE_Bd` | 波特率修改(重新上电生效) |
| 0x18 | `CANCOM_MODE_ACTIVE_RECV` | 主动上报设置(默认周期 10 ms) |
| 0x19 | `CANCOM_MODE_AGREEMENT` | 协议修改(0 私有 / 1 Canopen / 2 MIT,重新上电生效) |
| 0x7D / 0x7E / 0x7F | `CANCOM_CALI_ING` / `CANCOM_CALI_RST` / `CANCOM_PARA_STR_INFO` | 官方手册里没有,本工程也没人用 —— 留着只为兼容旧引用,**勿发帧** |

⚠️ 必须按官方取值:按 enum 顺序排下来的话 `PARA_READ` / `PARA_WRITE` / `MODE_TOTAL` / `MODE_Bd` / `FAULT_WARN` 全错,那些帧发出去就是废帧。

### 运行模式:参数 `0x7005`

| 值 | 宏 | 含义 |
| --- | --- | --- |
| 0 | `MOTOR_LZ_RUNMODE_MOTION` | **运控模式**(本工程必须用它) |
| 1 | `MOTOR_LZ_RUNMODE_POS_PP` | PP 位置 |
| 2 | `MOTOR_LZ_RUNMODE_SPEED` | 速度 |
| 3 | `MOTOR_LZ_RUNMODE_CURRENT` | 电流 |
| 4 | `MOTOR_LZ_RUNMODE_ZERO` | 置零 |
| 5 | `MOTOR_LZ_RUNMODE_POS_CSP` | CSP 位置 |

运控帧(类型 0x01)**只在【运行模式 = 运控模式】时才被执行** —— 被别的上位机改成 PP 位置 / 速度 / 电流 / CSP 后,板子发的运控帧会被静默忽略(现象:电机不使能、不动)。用 `set_run_mode()` 写 0x7005,再用 `Get_RunMode()` / `Get_RunModeValid()` 回读确认。

### 电机状态 pattern(反馈帧 ID 的 bits 22-23)

| 值 | 宏 | 含义 |
| --- | --- | --- |
| 0 | `MOTOR_LZ_PATTERN_RESET` | 复位 |
| 1 | `MOTOR_LZ_PATTERN_CALI` | 标定中 |
| 2 | `MOTOR_LZ_PATTERN_RUN` | 运行中 |

⚠️ 它不是「运行模式」:电机上电要跑自己的复位 / 编码器标定,这期间它不接受使能。读法见 `Get_Pattern()`(即 `recv_data.mode`)。

### 反馈解析(`can_recv()`,8 字节数据区)

| 字节 | 字段 | 换算 |
| --- | --- | --- |
| `data[0..1]` | 位置 | 大端 u16 → `uint_to_float(raw, Pos_min, Pos_max, 16)` |
| `data[2..3]` | 速度 | 大端 u16 → `uint_to_float(raw, W_min, W_max, 16)` |
| `data[4..5]` | 力矩 | 大端 u16 → `uint_to_float(raw, T_min, T_max, 16)` |
| `data[6..7]` | 温度 | 大端 u16 / 10 = ℃ |

* 反馈 ID:bits 21-16 = 故障码 → `recv_data.MError`;bits 23-22 = 电机状态 pattern → `recv_data.mode`。
* 温度合理性窗口:`MOTOR_LZ_TEMP_VALID_MAX` = 150.0 ℃、`MOTOR_LZ_TEMP_VALID_MIN` = −40.0 ℃;超出窗口置 −1.0(约定 < 0 = 无效)。实测遇到过恒报 6533.6 ℃(原始 u16 = 0xFF38)的一路哨兵值。
* `can_recv()` 同时记 `LastRxTick = HAL_GetTick()`(判「还活着」用),并把实际状态回读进 `Motor_Status`:`pattern == 2` → `Motor_Status_ENABLE`,否则 `Motor_Status_DISABLE`。它表示的是「电机状态机跑到哪一步」,**不完全等于**「我们使能了没有」。

### 设置 CAN ID 帧(类型 0x07)的位域

核对来源:**官方示例工程 `F446_CAN_Lingzu`**(`Core/Src/can.c`)与
**官方 Python 驱动 `DM_RS-main/robstride_driver.py::set_can_id()`** —— 两者一致:

```
bit 28-24 : 类型 = 7 (SET_CAN_ID)
bit 23-16 : 新 ID            ← 0x07 帧独有
bit 15-8  : 主机 ID          ← 与使能/失能/置零帧同一个字段
bit  7-0  : 当前(旧)电机 ID ← 必须用旧 ID 寻址,否则电机收不到这一帧
数据区    : 全 0(官方驱动不带数据)
```

* Python 驱动原文:`extra_data = (new_id << 8) | self.host_id` →
  `ExtId = (7 << 24) | (extra_data << 8) | current_id`
* C 例程(使能帧)原文:`exdata = 0xfd; //主机ID` →
  `ExtId = (id & 0xff) | ((exdata & 0xffff) << 8) | ((mode & 0x1f) << 24)`
  ⇒ **数据段(15-8)放主机 ID** 这一点两处一致;`23-16` 放新 ID 是 0x07 帧独有的
* 本驱动实现:`motor_LZ.cpp::motor_set_CAN_ID()` 按上表显式组帧(`lose()` → 发帧 → 更新
  `motor_init.id`)。**顺序不能反**:先更新 id 就会把这一帧发给新 ID,电机收不到。
* ID 是否掉电保持官方例程里没体现,需要的话再发一次"保存参数"(类型 0x16)。

## 三、收发路径

```text
App:  if (Is_Send_Due()) { Set_Pos/Set_T/Set_Kp/Set_Kd;  send(); }
      send()  → can_send()  → Transmit() → fdcan_send_data_Exten()          // 不等 FIFO
      zero()/write_param()/save_param() → Transmit_Wait() → fdcan_send_data_Exten_wait()  // 等 FIFO 空位
CAN 回调: recv(_id, _data, _len) → 类型 0x11 → param_answer();其余 → can_recv() → recv_data
```

* `can_send()` 只做「命令位闸门 + 量程夹紧 + 定点编码 + 发送」,**不管发送频率**;节奏由 App 用基类 `Is_Send_Due()` 判,发送时刻由 `send()` 内的 `Mark_Sent()` 记。
* `recv()` 要求 `_len >= 8`,按 `_id` 的 bits 28-24 分发:`0x11` → `param_answer()`,其余 → `can_recv()`。

## 四、接口表

### 4.1 `motor_mit` 的五个纯虚接口

| 接口 | 作用 | 备注 |
| --- | --- | --- |
| `Init(hfdcan, __id, __model, __control_mode, __freq)` | 初始化(按型号) | `__freq` 在这里定好(内部 `Set_Control_Freq()` 算出 `Control_Period_Us` 并把首帧置为立即到期);运行中想改再调 `Set_Control_Freq()` |
| `send()` | 发送一帧运控帧 | 16 bit 编码,力矩走 ID 的数据段;内部 `can_send()` |
| `recv(_id, _data, _len)` | 解析一帧数据 | 0x11 参数应答 → `param_answer()`;0x02 / 0x18 反馈 → `can_recv()`;`_len < 8` 直接丢 |
| `zero()` | 设置机械零位 | 类型 0x06;结果见 `Get_Zero_Result()` |
| `enable()` | 电机使能 | 类型 0x03;同时把命令位 `Send_Status_Cmd` 置 ENABLE |
| `lose()` | 电机失能 / 停止 | 类型 0x04(`data[0]=1` 顺带清故障);同时把命令位置 DISABLE |

### 4.2 官方参数操作

| 接口 | 作用 | 备注 |
| --- | --- | --- |
| `read_param(uint16_t index)` | 读一个参数(类型 0x11) | 发 `[index_lo, index_hi, 0…0]`;应答值在 `data[4..7]` |
| `write_param(index, value, width)` | 通用写参数(类型 0x12) | `Byte0~1 = Index` 小端、`Byte2~3 = 位宽`、`Byte4~7 = 值`;走 `Transmit_Wait`;⚠️ 掉电丢失,写完要 `save_param()` |
| `write_param_u8(index, value)` | 写一个 u8 参数 | 等价于 `write_param(index, &value, 1)` |
| `set_run_mode(uint8_t run_mode)` | 写 0x7005 并立刻回读确认 | 结果见 `Get_RunMode()` / `Get_RunModeValid()` |
| `save_param()` | 保存到电机 Flash(类型 0x16) | 数据区固定 `01..08`;返回 0 = 已进 TX FIFO,1 = 超时 |
| `set_zero_sta(uint8_t v)` | 0x7029 `zero_sta` | 写 1 标零;csp / 运控模式下可标零,PP 模式会被屏蔽 |
| `set_add_offset(float v)` | 0x2024 `add_offset` | 把电机零点再挪 v 弧度(可就地标零,不用把机器人摆到零点姿态) |
| `set_report_time(uint16_t v)` | 0x7026 `EPSCAN_time` | 主动上报间隔 |
| `active_recv(uint8_t F_CMD)` | 主动上报开关(类型 0x18) | 1 开 / 0 关 |
| `param_answer(const uint8_t *data)` | 参数应答帧处理 | 布局:`bits 15-8` = 电机 ID、`data[0..1]` = 参数地址(小端)、`data[4..7]` = 值;本类只关心 0x7005 |
| `motor_set_CAN_ID(uint8_t set_id)` | 修改 CAN ID(类型 0x07) | 位域见「设置 CAN ID 帧」;**必须用旧 ID 寻址**,发完才更新本对象 id |

### 4.3 状态 / 诊断

| 接口 | 作用 | 备注 |
| --- | --- | --- |
| `Get_CAN_Id()` | 本电机 CAN ID | 反馈帧 bits 15-8,接收分发时用 |
| `Get_Host_Id()` / `Set_Host_Id(__user_id)` | 主机(上位机)CAN ID | 命令帧 bits 15-8 |
| `Is_My_Feedback(_id)` | 这一帧是不是本电机的反馈帧 | 分发用 |
| `Get_RunMode()` | 回读到的运行模式 | 0 = 运控模式 |
| `Get_RunModeValid()` / `Clear_RunModeValid()` | 是否已回读到 / 清标志 | 重新探测前先清 |
| `Get_Pattern()` | 电机状态 pattern | 0 复位 / 1 标定中 / 2 运行中 |
| `Get_LastRxTick()` | 最近一次收到本电机反馈帧的 `HAL_GetTick()` | 判「还活着」用 |
| `Get_Zero_Result()` | 最近一次 `zero()` 的结果 | 0 = 全部帧已进 TX FIFO;1 = 有帧超时没发出去 |

### 4.4 内部(protected)

| 接口 | 作用 |
| --- | --- |
| `Transmit(_id, _data, _len)` | 按通信方式发一帧(灵足用扩展帧),不等 FIFO |
| `Transmit_Wait(_id, _data, _len, _timeout_ms)` | 发送并等 TX FIFO 空位(0 = 成功) |
| `Apply_Model_Limit(__model)` | 按型号补齐 16 bit 编码量程(结构体未指定时使用) |
| `make_frame_id(Id_data)` | 组装扩展帧 ID |
| `can_send()` / `can_recv(Data, data)` | 灵足电机扩展帧收发 |

## 五、典型用法

```c
static Class_Motor_LZ lz;
lz.Init(&hfdcan1, 0x02, MOTOR_LZ_02, motor_pos, Motor_Control_Freq_500HZ);  // 型号 + 模式 + 频率
lz.enable();                       // 还要确认参数 0x7005 = 运控模式(见 set_run_mode)
if (lz.Is_Send_Due()) {            // 该不该发在 App 里判;时刻由 send() 内部 Mark_Sent()
    lz.Set_Pos(p); lz.Set_Kp(kp); lz.Set_Kd(kd);   // 内部 PD;主机闭环改成 lz.Set_T(t);
    lz.send();
}
// CAN 回调里:lz.recv(id, buf, 8u);  反馈帧用 Is_My_Feedback(id) 分发到本台
```

## 六、坑与注意

* **闸门看命令位**。`can_send()` 的闸门是 `Send_Status_Cmd`(`enable()` / `lose()` 改);反馈的 pattern 只回读成 `Get_Pattern()` / `Motor_Status` —— 「命令了没有」和「电机现在怎样」是两件事。电机还在复位 / 标定阶段时 `Motor_Status` 是 DISABLE,拿它当闸门就会在最需要持续发帧的那段时间把帧停掉。
* **`Transmit_Wait()` 会阻塞**。`zero()` / `write_param()` / `save_param()` 内部走 `Transmit_Wait()` 等 TX FIFO 空位(超时 `MOTOR_LZ_TX_WAIT_TIMEOUT_MS` = 5 ms)⇒ **不能在中断 / 硬实时上下文里调**。
* **FIFO 只有 8 槽**。`Transmit()` 不等 FIFO(满了静默丢帧);一条总线上多台电机连续置零时不等空位会把后面的帧静默丢掉(真机现象:「每条总线只生效前 3 个电机」),所以置零用 `Transmit_Wait()` 并把结果存进 `zero_result`。
* **`zero()` 的顺序是** `lose()` → 类型 0x06 →(`MOTOR_LZ_ZERO_SEND_SAVE` 为真时补发类型 0x16 保存)→ `lose()`:置零只允许在未使能时执行,所以结尾是**失能**而不是使能。
* **同一实例不可重入**。组帧读的是成员变量 `mode`(`make_frame_id()`),多任务并发调会串帧;所以每个发帧函数都先给 `mode` 赋值。
* **写参数掉电丢失**。写参数(0x12)写完要 `save_param()`(0x16,数据区固定 `01..08`);官方注明「搭配类型 0x16 可保存」。
* **量程必须与上位机一致**,否则整条换算比例都是错的;它不是关节软限位。
* **`motor_set_CAN_ID()` 的帧格式已核对**(官方 C 例程 `F446_CAN_Lingzu/Core/Src/can.c` + 官方 Python 驱动
  `robstride_driver.set_can_id()`),位域见「设置 CAN ID 帧」小节;本工程未使用该接口,现场改 ID 时注意**先失能**。
* `Init()` 末尾会把 `mode` 置成 `CANCOM_ANNOUNCE_DEVID`;实际发帧时各发帧函数都会先重设 `mode`,所以不影响使用。

## 七、相关文件

* 基类、统一单位与调度接口:`User/Device/motor/motor_mit.h`
* 底层 CAN 发送:`User/Bsp/fdcan/bsp_fdcan.h`
* 定点换算 `uint_to_float()` / `float_to_uint_f()` / `motor_max_min()`:`User/Device/motor/motor_mit.h`、`User/algorithm/math_support/math_support.h`
* 整机用法示例(不在此重复):`User/Device/motor/电机使用说明.md`
