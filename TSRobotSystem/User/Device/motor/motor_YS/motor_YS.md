# motor_YS — 宇树(YS)电机驱动:**RS-485/UART**、一个类覆盖两代协议,接 `motor_mit`

`Class_Motor_YS` 按 `Init()` 传入的型号分派两代协议:GO-M8010-6 与 IM6014(J288/S288)。485 是**半双工一问一答**,一台一路;对外默认 **关节侧**单位(rad、rad·s⁻¹、N·m),内部按减速比自动折算成协议里的转子侧量。

| 文件 | 内容 |
| --- | --- |
| `motor_YS.h` | 类声明、两代协议的宏与字节表、型号枚举 `motor_YS_Model` |
| `motor_YS.cpp` | `pack_send_8010/6014`、`parse_recv_8010/6014`、分派与收发实现 |
| `motor_mit.h` | 基类、统一单位与调度接口(`send_data` / `recv_data`、`Is_Send_Due()`、`Mark_Sent()`) |

## 一、型号分派

| 型号 | 电机 | 发 / 收帧长 | 校验 | 减速比 | 依据 |
| --- | --- | --- | --- | --- | --- |
| `MOTOR_YS_M8010` = 0 | GO-M8010-6 | 17 B / 16 B | CRC16(`crc_ccitt()`,CRC16-CCITT) | `MOTOR_YS_GEARBOX_M8010` = 6.33 | 《GO-M8010-6 Motor 使用手册 V1.0》表 1 / 表 2 |
| `MOTOR_YS_IM6014` = 1 | IM6014(J288 / S288) | 20 B / 26 B | CRC32(`crc32_mpeg2()`) | `MOTOR_6014_RATIO` = 38/3 | 官方 `N6014B_HALDemo/App/protocol.h` + `protocol.c` |

`Apply_Model_Limit()` 按型号填这一组协议参数,组帧 / 解帧全部按这些成员走:

| 成员 | 8010 | 6014 |
| --- | --- | --- |
| `Gearbox_Rate` | `MOTOR_YS_GEARBOX_M8010` 6.33 | `MOTOR_6014_RATIO` 38/3 |
| `Send_Len` / `Recv_Len` | `MOTOR_YS_SEND_LEN` 17 / `MOTOR_YS_RECV_LEN` 16 | `MOTOR_6014_SEND_LEN` 20 / `MOTOR_6014_RECV_LEN` 26 |
| `Crc_Send_Len` / `Crc_Recv_Len` | 15(CRC16 覆盖前 15 B)/ 14(CRC16 覆盖前 14 B) | 16(CRC32 覆盖前 16 B,含包头)/ 20(CRC32 覆盖 Byte2..21,不含包头) |
| `Recv_Head_0` | `MOTOR_YS_RECV_HEAD_0` 0xFD | `MOTOR_6014_RECV_HEAD_0` 0xFC |

* 未知型号按 `MOTOR_YS_M8010` 处理(`default` 分支),总比留一组未初始化的系数好。
* 若结构体里已经给过 `Pos_max` / `Pos_min`(非 0),`Apply_Model_Limit()` 直接返回,不覆盖。
* `MOTOR_YS_CHECK_RECV_CRC`(默认 1)控制反馈侧 CRC 校验:8010 校 CRC16、6014 校 CRC32;置 0 关闭。

### 量程

| 量 | 8010(表 1,★**转子侧**) | 6014(官方 SATURATE,★本来就是**输出端**) |
| --- | --- | --- |
| 力矩 T | `MOTOR_YS_T_MAX_ROTOR` 127.99 N·m | `MOTOR_6014_TOR_MAX` 162.128385 / `TOR_MIN` −162.133333 N·m |
| 速度 W | `MOTOR_YS_W_MAX_ROTOR` 804.0 rad/s | `MOTOR_6014_SPD_MAX` 253.965213 / `SPD_MIN` −253.972964 rad/s |
| 位置 Pos | 表 1 无关节限位(θ 是多圈累加 int32),只按「编码不溢出」反推:`2147483647.0f / (MOTOR_YS_P_SCALE * Kin_Out2Raw())` | `MOTOR_6014_POS_MAX` 32508.539376 / `POS_MIN` −32508.539391 rad |
| Kp / Kd | `MOTOR_YS_KP_MAX` / `MOTOR_YS_KD_MAX` = 25.599(与减速比无关) | `MOTOR_6014_KP_MAX` 410.725243 / `MOTOR_6014_KD_MAX` 102.681311 |

6014 的上下限不对称,是因为 raw 的 ±32768 / ±32767 本身不对称。

### 量化系数

| 量 | 8010(宏) | 6014(宏) |
| --- | --- | --- |
| 力矩 | `MOTOR_YS_T_SCALE` 256.0f | `MOTOR_6014_TOR_SCALE` 2560.0f |
| 速度 | `MOTOR_YS_W_SCALE` 40.743665f(= 256/(2π)) | `MOTOR_6014_SPD_SCALE` 64.0f |
| 位置 | `MOTOR_YS_P_SCALE` 5215.18917f(= 32768/(2π)) | `MOTOR_6014_POS_SCALE` 32768.0f |
| Kp | `MOTOR_YS_KP_SCALE` 1280.0f | `MOTOR_6014_KP_SCALE` 12800.0f |
| Kd | `MOTOR_YS_KD_SCALE` 1280.0f | `MOTOR_6014_KD_SCALE` 51200.0f |
| 反馈除数 | `MOTOR_YS_W_FBK_DIV` 256.0f、`MOTOR_YS_P_FBK_DIV` 32768.0f | 同左(与 `_SCALE` 互为逆运算) |

⚠️ 8010 的发送系数必须与表 1 严格一致,并且**与表 2 的反馈除数互为逆运算**:

* 表 1(发送):`ω_set = ω/(2π)×256` → 256/(2π) = 40.743665;`θ_set = θ/(2π)×32768` → 32768/(2π) = 5215.18917
* 表 2(接收):`ω = ω_fbk/256×2π`、`θ = θ_fbk/32768×2π` ✓ 互为逆

历史错误:这两个值曾写成 `128.0f` / `16384.0f`(= ÷2 而不是 ÷2π,**少了 π**),结果「发 1 rad/s、回读 π rad/s」,电机实际走 3.14 倍。改这两个值后务必验一遍「发出去再读回来等于原值」(1.0 rad/s → raw 41 → 回读 ≈ 1.0)。

## 二、协议 A:GO-M8010-6(17 B 发 / 16 B 收,CRC16,减速比 6.33)

### 发送帧(表 1,主机侧控制协议)

| Byte | 字段 | 公式 / 取值 |
| --- | --- | --- |
| 0-1 | HEAD | `0xFE 0xEE` |
| 2 | `[4bit] ID \| [3bit] STATUS<<4 \| [1bit] 保留` | STATUS:0 = 锁定(Default)、1 = FOC 闭环、2 = 编码器校准、3~7 保留 |
| 3-4 | τ_set | `τ_ff*256`,−127.99 ≤ τ_ff ≤ 127.99 N·m |
| 5-6 | ω_set | `ω_des/(2π)*256`,−804.0 ≤ ω_des ≤ 804.0 rad/s |
| 7-10 | θ_set | `θ_des/(2π)*32768`(int32,多圈累加) |
| 11-12 | K_pos | `Kp*1280`,0 ≤ Kp ≤ 25.599 |
| 13-14 | K_spd | `Kd*1280`,0 ≤ Kd ≤ 25.599 |
| 15-16 | CRC16 | CRC16-CCITT(前 15 B),小端写入 |

### 反馈帧(表 2,16 B)

| Byte | 字段 | 解析 |
| --- | --- | --- |
| 0-1 | HEAD | **`0xFD 0xEE`**(与主机方向的 0xFE 不同) |
| 2 | ID + STATUS | STATUS = `(data[2] >> 4) & 0x07`(bit7 是保留位,不能一起取) |
| 3-4 | τ | `raw/256`,再 `Tq_Raw2Out()` 到对外侧 |
| 5-6 | ω | `raw/256*2π`,再 `Kin_Raw2Out()` |
| 7-10 | θ | `raw/32768*2π`,再 `Kin_Raw2Out()` |
| 11 | 温度 | int8,℃;≥ `MOTOR_YS_TEMP_PROTECT`(90.0 ℃)= 过温(电机自保护,需重新上电) |
| 12 | MERROR / FORCE | 低 3 位 MERROR;高 5 位是 FORCE 的高位 |
| 13 | FORCE | `bits 99-110` 12 bit 足端力原始值(0~4095;物理单位手册注明「待确认」) |
| 14-15 | CRC16 | CRC16-CCITT 覆盖前 14 B |

## 三、协议 B:IM6014(J288 / S288,20 B 发 / 26 B 收,CRC32,减速比 38/3)

### 发送帧(官方 `ControlData_t`)

| Byte | 字段 | 公式 |
| --- | --- | --- |
| 0-1 | HEAD | `0xFE 0xEE` ★参与 CRC32 |
| 2 | `[4bit]ID \| [3bit]STATUS<<4 \| [1bit]TIMEOUT<<7` | TIMEOUT(主机→电机):0 = 禁用超时保护、1 = 开启(电机默认 1 s 超时) |
| 3 | res | 保留(= 0) |
| 4-5 | tor_des int16 | `outputTor / RATIO * 2560` |
| 6-7 | spd_des int16 | `outputSpd * RATIO * 64 / (2π)` |
| 8-11 | pos_des int32 | `outputPos * RATIO * 32768 / (2π)` |
| 12-13 | k_pos int16 | `Kp / RATIO² * 12800` |
| 14-15 | k_spd int16 | `Kd / RATIO² * 51200` |
| 16-19 | CRC32 | `crc32_mpeg2(前 16 B,含包头)`,小端写入 |

### 反馈帧(官方 `extract_data()`,26 B)

| Byte | 字段 | 解析 |
| --- | --- | --- |
| 0-1 | HEAD | **`0xFC 0xEE`**(不参与 CRC32;与 8010 的 0xFD、主机方向的 0xFE 都不同) |
| 2 | mode | STATUS = `(data[2] >> 4) & 0x07`;bit7 = 电机报超时保护 |
| 3 | 壳体温度 | int8,℃ |
| 4 | 绕组温度 | u8,℃ |
| 5 | 电压 | u8,0.5 V/格(`raw / 2` = V) |
| 6-7 / 8-9 / 10-13 | τ / ω / θ | 与 8010 同法反算:`torque/2560*RATIO`、`speed/64*2π/RATIO`、`2π*pos/32768/RATIO` |
| 14-17 | MError | u32;`recv_data.MError` 只放低 8 位,完整值用 `Get_MError32()` |
| 18-19 | `[Res:13 \| ExFlag:3]` | 警告码 `MWarn = (rd_u16(18) >> 13) & 0x07` |
| 20-21 | ExSensor2 / ExCom | 在 CRC32 覆盖范围内 |
| 22-25 | CRC32 | `crc32_mpeg2(Byte2..21 共 20 B)`,小端 |

* 6014 的 CRC32 见 `crc32.h`(poly 0x04C11DB7、初值 0xFFFFFFFF、无末尾异或、按 4 字节一组)。
* 8010 的 CRC 算法手册 / 官方都指明参考 Linux 内核 `crc_ccitt` —— 见 `crc_ccitt.h`。

### 两代共用的 mode 字节位布局

| 宏 | 值 | 含义 |
| --- | --- | --- |
| `MOTOR_YS_ID_MASK` | 0x0F | ID 低 4 位 |
| `MOTOR_YS_ID_MAX` | 14 | 0~14 可用 |
| `MOTOR_YS_ID_BROADCAST` | 15 | 广播(总线无返回值) |
| `MOTOR_YS_STATUS_SHIFT` / `MOTOR_YS_STATUS_MASK` | 4 / 0x07 | `[3bit]` STATUS;bit7 在 8010 是保留位、在 6014 是 TIMEOUT |
| `MOTOR_YS_STATUS_LOCK` / `_FOC` / `_CALI` | 0 / 1 / 2 | 锁定(Default)/ FOC 闭环 / 编码器校准 |
| `MOTOR_6014_TIMEOUT_BIT` | 0x80 | 6014:主机→电机 1 = 开启超时保护;电机→主机 1 = 已触发 |

两代的 mode 字节位布局相同(id:4 / status:3 / 最高位),只是最高位在 8010 是保留位、在 6014 是 TIMEOUT。

## 四、对外单位约定:关节侧(输出轴)

见 `MOTOR_YS_JOINT_SIDE_UNITS`:**1 = 关节侧**(输出轴,与 DM / LZ / DJ 一致,推荐)、**0 = 转子侧**(与手册 / 官方字段字面一致,上层需自己乘减速比)。

| 量 | 换算 |
| --- | --- |
| 角度、角速度 | 转子量 = 对外量 × `Gearbox_Rate`(`Kin_Out2Raw()`) |
| 力矩 | 转子量 = 对外量 ÷ `Gearbox_Rate`(`Tq_Out2Raw()`) |

两代协议的线上字段都是**转子端**量(8010 手册要求「用户自己乘 6.33」;6014 官方 `RIS_Comd_t` 注释写明「电机转子端」),而本工程其它品牌(DM / LZ / DJ)都用关节侧量,所以本类默认对外就是关节侧。

⇒ 上层直接 `Set_Pos(关节弧度)`,**不要**再自己乘减速比。想按手册字面(转子侧)用,把 `MOTOR_YS_JOINT_SIDE_UNITS` 置 0 即可(此时四个换算函数全部退化成恒等 1.0)。

## 五、收发路径

```text
组帧:  Init() → Apply_Model_Limit() 定协议参数 + Set_Control_Freq() 定周期(首帧立即到期)
       send()  → 闸门(命令位) → pack_send_frame() → pack_send_8010/6014() → Transmit() → UART_Transmit_Data()
       enable() / lose() 直接 pack_send_frame() + Transmit(),不走闸门
解帧:  485 回调 → recv(0u, buf, len) → memcpy 到 motor_YS_recv_buf → unpack_recv_frame()
       → parse_recv_8010/6014() → 校验包头 + CRC → 写 recv_data 与各专有量
```

* `pack_send_frame()` / `unpack_recv_frame()` 按 `model` 分派到 8010 / 6014 两对函数。
* 收发缓冲 `motor_YS_send_buf` / `motor_YS_recv_buf` 固定开在 `.AXI_SRAM`(要走 DMA);发送缓冲按最长协议开 `MOTOR_6014_SEND_LEN`(20),接收缓冲 `MOTOR_6014_RECV_LEN`(26)。
* `send()` 不做门控判定:「该不该发」由 App 用 `Is_Send_Due()` 判断;发完 `Mark_Sent()` 把时刻记进 `send_time`。

## 六、接口表

### 6.1 `motor_mit` 的纯虚接口

| 接口 | 作用 | 备注 |
| --- | --- | --- |
| `Init(huart, __id, __model, __control_mode, __freq, __timeout_enable = true)` | 初始化(按型号选协议) | ⚠️ 波特率 4 Mbps;`__id` 0~14(15 = 广播,总线无返回值);`__timeout_enable` **仅 6014 有意义**(是否让电机开启 1 s 超时保护,默认开) |
| `send()` | 发送一帧控制量 | 8010 = 17 B / 6014 = 20 B(含校验);闸门看命令位 |
| `recv(_id, _data, _len)` | 解析一帧反馈 | 8010 = 16 B / 6014 = 26 B;校验包头 + CRC |
| `zero()` | 设置机械零点 | 两代都无此命令,用软件偏置:把此刻位置记为 0 |
| `enable()` | 电机使能 | 发一帧,STATUS = 1 FOC 闭环 |
| `lose()` | 电机失能 | 发一帧,STATUS = 0 锁定;顺带清空 `send_data` |

统一反馈读取(`Get_Now_Pos/W/T`、`Get_Motor_Temperature`、`Get_MError`、`Get_Feedback_Mode`)由基类提供:非虚函数,直接读基类的 `recv_data`。

### 6.2 品牌自有接口

| 接口 | 作用 | 备注 |
| --- | --- | --- |
| `Get_Motor_Id()` | 本电机的 485 id | 反馈 mode 字节低 4 位,接收分发时用 |
| `Get_Model()` | 型号(当前用哪一代协议) | — |
| `Get_Send_Len()` / `Get_Recv_Len()` | 本型号的收发帧长 | 注册 BSP 时用:`USART_RX485_init_Ex(huart, motor.Get_Recv_Len(), cb)`;`Transmit` 用 `Get_Send_Len()` |
| `Get_Gearbox_Rate()` | 减速比 | 8010 = 6.33,6014 = 38/3 |
| `Set_Init_Pos(__init_Pos)` / `Get_Init_Pos()` | 零位偏置(关节侧弧度) | 实际发送位置 = `Pos + init_Pos` |
| `Get_Force_Raw()` | **8010**:足端力原始值 | 表 2 FORCE 12 bit,0~4095 |
| `Is_Over_Temperature()` | **8010**:上次反馈是否报过温 | 表 2 TEMP ≥ 90 ℃ |
| `Get_Winding_Temperature()` | **6014**:绕组温度 ℃ | — |
| `Get_Voltage()` | **6014**:电机端电压 V | 0.5 V 分辨率 |
| `Get_MError32()` | **6014**:32 位错误码 | 完整值(基类 `Get_MError()` 只有低 8 位) |
| `Get_MWarn()` | **6014**:警告码 | ExFlag 3 bit |
| `Is_Motor_Timeout()` | **6014**:电机是否报超时保护 | 反馈 mode 的 TIMEOUT 位;发一帧 `timeout = 0` 可清除 |
| `Set_Timeout_Protect(__enable)` / `Get_Timeout_Protect()` | **6014**:超时保护开关 | 写进每一帧 mode 字节的 bit7 |
| `Get_Recv_Crc_Error_Count()` | 反馈校验错误累计次数 | 诊断;8010 = CRC16 错,6014 = CRC32 错 |

### 6.3 内部(protected)

| 接口 | 作用 |
| --- | --- |
| `Transmit(_data, _len)` | 按通信方式发一帧(走 485 / UART) |
| `Apply_Model_Limit(__model)` | 按型号填:减速比、收发帧长、校验长度、反馈包头、量程 |
| `pack_send_frame()` / `unpack_recv_frame(_data)` | 组帧 / 解帧(按型号分派) |
| `pack_send_8010()` / `parse_recv_8010(_data)` | 协议 A:GO-M8010-6 |
| `pack_send_6014()` / `parse_recv_6014(_data)` | 协议 B:IM6014 |
| `Kin_Out2Raw()` / `Kin_Raw2Out()` / `Tq_Out2Raw()` / `Tq_Raw2Out()` | 对外量 ↔ 协议(转子侧)量的换算 |

## 七、典型用法

```c
static Class_Motor_YS ys;
ys.Init(&huart2, 0, MOTOR_YS_M8010, motor_pos, Motor_Control_Freq_200HZ); // 6014 多一个 timeout_enable
USART_RX485_init_Ex(&huart2, ys.Get_Recv_Len(), USART2_Callback);  // 帧长按型号:16 B / 26 B
// 回调里:ys.recv(0u, buf, (uint8_t)size);   // 485 无 id,recv 也不校验 id
ys.enable(); ys.Set_Pos(p); ys.Set_Kp(kp); ys.Set_Kd(kd);
if (ys.Is_Send_Due()) ys.send();            // 半双工:节奏由 App 轮询控,别背靠背连发
```

## 八、坑与注意

* **两代协议不能挂同一条总线**:帧长、校验算法、包头、量化系数全不同 —— 主机发 `0xFE 0xEE`,8010 收 `0xFD 0xEE`、6014 收 `0xFC 0xEE`,所以各占一路 485(本工程 huart2 / huart3),两路波特率都可用 4 Mbps(6014 也支持 6 Mbps,但 PCLK1 = 80 MHz 下 4 Mbps 的 USARTDIV 正好 2.5、误差 0%,6 Mbps 只能取到 1.625 → 误差 −2.5%,所以选 4 Mbps)。
* **半双工一问一答**:8010 手册 §5 / §8.2「不发命令电机就不返回状态」;6014 官方 demo 是「拉高 DE → 阻塞发一帧 → 拉低 DE → ReceiveToIdle 收一帧」。⇒ 主机必须控制节奏,不要对多台背靠背连发(RS-485 半双工会撞总线),App 建议做**轮询**:每拍只发一台,收到应答再推进。带宽(4 Mbps,8N1):8010 `17B≈42.5µs + 16B≈40µs ≈ 83µs/台`;6014 `20B≈50µs + 26B≈65µs ≈ 115µs/台`。
* **`recv()` 必须先 memcpy 再解析**:接收侧是 DMA(BSP 的 485 引擎 = DMA 双缓冲 DBM + IDLE 帧结束),而且 `HAL_UARTEx_RxEventCallback` 是**先回调、后手动翻转 CT** —— 回调执行期间 DMA 仍指向它正在收的那个缓冲,下一帧字节一到就会覆盖当前帧。所以 `recv()` 先把整帧 `memcpy` 到 `motor_YS_recv_buf` 再交给 `unpack_recv_frame()`。`_len < Recv_Len` 直接丢弃;校验不过**不更新** `recv_data`,只把 `Recv_Crc_Error_Count` 加一。
* **`Get_Recv_Len()` 要传给 BSP**:485 引擎按帧长收帧,注册时要用本类的 `Get_Recv_Len()`(8010 = 16、6014 = 26);头文件里的写法是 `8010: USART_RX485_init(&huart2, cb)`(16 B 默认)、`6014: USART_RX485_init_Ex(&huart3, 26, cb)`。缓冲容量已放开到 `RS485_recv_Data_N_MAX = 32`。
* **多台共用串口有坑**:BSP 的 `UART_Transmit_Data()` 内部是一个**全局 static 发送缓冲**,上一帧 DMA 没发完(返回 `HAL_BUSY`)时 `memcpy` 已经把在途帧踩了。
* **发送缓冲是文件级 static**:`motor_YS_send_buf` / `motor_YS_recv_buf` 所有实例共用 ⇒ 多实例或多任务同时发会互相踩,一条 485 上必须串行一问一答。
* **两代都没有独立的使能命令帧**:使能位就在**每一帧** mode 字节的 STATUS 里(`enable()` = 1 FOC,`lose()` = 0 锁定),要实打实发一帧才生效。
* **两代也都没有「设置机械零点」命令**:`zero()` 是软件偏置(`Set_Init_Pos(-当前反馈位置)`),只加在**发送侧**(发送位置 = `Set_Pos + init_Pos`),反馈仍是电机自己的绝对位置 —— 注意 `zero()` 的注释说反馈侧保持绝对位置不动,便于上层做标定 / 对齐。
* **闸门只看命令位**:`send()` 的闸门是 `Send_Status_Cmd`;反馈的 STATUS 只写 `Motor_Status`(实际状态),不会反向把命令位打开(否则 `lose()` 后收到旧反馈就被「重新使能」了)。`enable()` / `lose()` 直接发帧、不走闸门;`lose()` 还会清空 `send_data` 防止下次使能冲出。
* **`sat_i16()` 不可省**:两代协议量程上限正好压在 raw = 32767 边上(6014 官方 SATURATE 的 τ/ω/Kp/Kd 四个上限算出来就是 32767.002 这种值,余量不到 1 LSB),浮点舍入往上偏一点就会翻成 −32768(指令极性反转);所有写进 2 字节字段的量都过一遍 `sat_i16()`。
* **8010 的位置是多圈累加 int32、没有关节限位**;发送侧位置要叠加 `init_Pos` 偏置(`zero()` 就是拿它实现的)。

## 九、相关文件

* 基类、统一单位与调度接口:`User/Device/motor/motor_mit.h`
* CRC:`User/algorithm/crc/crc_ccitt.h`、`User/algorithm/crc/crc32.h`
* 485 收发与 DMA 引擎(`USART_RX485_init` / `_Ex`、`UART_Transmit_Data()`):`User/Bsp/uart/bsp_uart.h`(原 `bsp_RS485.h` 已并入)
* 整机用法示例(不在此重复):`User/Device/motor/电机使用说明.md`
