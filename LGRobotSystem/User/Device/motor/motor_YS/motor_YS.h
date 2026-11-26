//
// Created by 20159 on 2026/3/13.
//
// Class_Motor_YS — 宇树(YS)电机,一个类覆盖两代 RS-485 协议,按 Init 的型号选:
//
//   MOTOR_YS_M8010  : GO-M8010-6        17 B 发 / 16 B 收,CRC16,减速比 6.33
//                     依据《GO-M8010-6 Motor 使用手册 V1.0》表1/表2
//   MOTOR_YS_IM6014 : IM6014(J288/S288) 20 B 发 / 26 B 收,CRC32,减速比 38/3
//                     依据官方 N6014B_HALDemo/App/protocol.h + protocol.c
//
// ⚠️ 两代协议**不能挂同一条总线**(帧长、校验算法、包头、量化系数全不同):
//     8010:包头 主机 0xFE 0xEE → 电机 0xFD 0xEE
//     6014:包头 主机 0xFE 0xEE → 电机 0xFC 0xEE
//    所以各占一路 485(本工程 huart2 / huart3),两路波特率都可用 4 Mbps
//    (6014 也支持 6 Mbps,但 PCLK1=80 MHz 下 4 Mbps 的 USARTDIV 正好 2.5、误差 0%,
//     6 Mbps 只能取到 1.625 → 误差 −2.5%,所以选 4 Mbps)。
//
// ══════════════════════════════════════════════════════════════════════════
// 【协议 A:GO-M8010-6(表1 主机侧控制协议,17 Byte)】
//   Byte 0-1   HEAD      = 0xFE 0xEE
//   Byte 2     [4bit] ID | [3bit] STATUS<<4 | [1bit] 保留
//                STATUS: 0=锁定(Default) 1=FOC闭环 2=编码器校准 3~7保留
//   Byte 3-4   τ_set  = τ_ff*256                 −127.99 ≤ τ_ff ≤ 127.99 N·m
//   Byte 5-6   ω_set  = ω_des/(2π)*256           −804.0 ≤ ω_des ≤ 804.0 rad/s
//   Byte 7-10  θ_set  = θ_des/(2π)*32768         (int32,多圈累加)
//   Byte 11-12 K_pos  = Kp*1280                  0 ≤ Kp ≤ 25.599
//   Byte 13-14 K_spd  = Kd*1280                  0 ≤ Kd ≤ 25.599
//   Byte 15-16 CRC16  = CRC16-CCITT(前 15 B)
//   (表2 反馈 16 B:HEAD **0xFD 0xEE**;τ=raw/256;ω=raw/256*2π;θ=raw/32768*2π;
//    温度 int8;MERROR 低 3 位;FORCE 12 bit;CRC16 覆盖前 14 B)
//   量化系数:×256 / ×128 / ×16384 / ×1280;减速比 6.33;CRC 用 crc_ccitt()
//
// 【协议 B:IM6014(J288/S288,20 Byte 发送)】
//   Byte 0-1   HEAD = 0xFE 0xEE                       ★参与 CRC32
//   Byte 2     [4bit]ID | [3bit]STATUS<<4 | [1bit]TIMEOUT<<7
//                TIMEOUT(主机→电机):0=禁用超时保护 1=开启(电机默认 1 s 超时)
//   Byte 3     res 保留(=0)
//   Byte 4-5   tor_des int16 = outputTor / RATIO * 2560
//   Byte 6-7   spd_des int16 = outputSpd * RATIO * 64 / (2π)
//   Byte 8-11  pos_des int32 = outputPos * RATIO * 32768 / (2π)
//   Byte 12-13 k_pos   int16 = Kp / RATIO² * 12800
//   Byte 14-15 k_spd   int16 = Kd / RATIO² * 51200
//   Byte 16-19 CRC32   = crc32_mpeg2(前 16 B,含包头)
//   (反馈 26 B:HEAD **0xFC 0xEE**(不参与 CRC32);mode;壳体温度 int8;绕组温度 u8;
//    电压 u8(0.5 V/格);τ/ω/θ 同上反算;MError u32;[Res:13|ExFlag:3];ExSensor2;ExCom;
//    CRC32 覆盖 Byte2..21 共 20 B)
//   饱和值(官方 SATURATE,输出端):τ ±162.13 N·m、ω ±253.97 rad/s、θ ±32508.54 rad、
//                                  Kp 0~410.725、Kd 0~102.681
//
// 两代的 mode 字节**位布局相同**(id:4 / status:3 / 最高位),只是最高位在 8010 是保留位、
// 在 6014 是 TIMEOUT。CRC 算法手册/官方都指明参考 Linux 内核 crc_ccitt —— 见 crc_ccitt.h;
// 6014 的 CRC32 见 crc32.h(poly 0x04C11DB7、初值 0xFFFFFFFF、无末尾异或、按 4 字节一组)。
//
// ══════════════════════════════════════════════════════════════════════════
// 【对外单位约定:关节侧(输出轴),见 MOTOR_YS_JOINT_SIDE_UNITS】
//   两代协议的线上字段都是**转子端**量(8010 手册要求"用户自己乘 6.33";
//   6014 官方 RIS_Comd_t 注释写明"电机转子端"),而本工程其它品牌(DM/LZ/DJ)都用关节侧量。
//   所以本类默认**对外 = 关节侧(输出轴)弧度 / 弧度每秒 / 牛米**,内部自动折算:
//       角度、角速度:转子量 = 对外量 × Gearbox_Rate
//       力矩        :转子量 = 对外量 ÷ Gearbox_Rate
//     ⇒ 上层直接 Set_Pos(关节弧度),**不要**再自己乘减速比。
//   想按手册字面(转子侧)用,把 MOTOR_YS_JOINT_SIDE_UNITS 置 0 即可。
//
// 【半双工一问一答】
//   8010 手册 5/8.2:"不发命令电机就不返回状态";6014 官方 demo 是
//   "拉高 DE → 阻塞发一帧 → 拉低 DE → ReceiveToIdle 收一帧"。
//   ⇒ 主机必须控制节奏,不要对多台背靠背连发(RS-485 半双工会撞总线),
//     App 建议做**轮询**:每拍只发一台,收到应答再推进。
//   带宽(4 Mbps,8N1):8010 17B≈42.5µs + 16B≈40µs ≈ 83µs/台;
//                     6014 20B≈50µs   + 26B≈65µs ≈ 115µs/台
//
// 【使能/失能/置零】
//   两代都没有独立的使能命令帧:使能位就在**每一帧**的 mode 字节 STATUS 里
//   (enable() → 1 FOC;lose() → 0 锁定)。也都没有"设置机械零点"命令
//   (status=2 是编码器校准,不是零点),所以 zero() 用软件偏置实现。
//
// 【接收 DMA 前提】
//   BSP 的 485 引擎按帧长收帧,注册时要用本类的 Get_Recv_Len():
//     8010: USART_RX485_init   (&huart2, cb)               // 16 B
//     6014: USART_RX485_init_Ex(&huart3, 26, cb)            // 26 B
//   (缓冲容量已放开到 RS485_recv_Data_N_MAX = 32)
//
// ⚠️ 多台共用串口时:BSP 的 UART_Transmit_Data() 内部是一个**全局 static 发送缓冲**,
//    上一帧 DMA 没发完(返回 HAL_BUSY)时 memcpy 已经把在途帧踩了。
//

#ifndef MOTOR_YS_H
#define MOTOR_YS_H


#include "struct_typedef.h"
#include "bsp_uart.h"   // 原bsp_RS485.h已并入bsp_uart.h(USART_RX485_init / _Ex)
#include "motor_mit.h"

/* ===== 对外单位约定(见文件头说明)=====
   1 = 关节侧(输出轴,与 DM/LZ/DJ 一致,推荐)
   0 = 转子侧(与手册/官方字段字面一致,上层需自己乘减速比) */
#define MOTOR_YS_JOINT_SIDE_UNITS  1

/* ═════════ 协议 A:GO-M8010-6 ═════════ */

//帧头 / 帧长
#define MOTOR_YS_SEND_HEAD_0       0xFE   //主机→电机
#define MOTOR_YS_SEND_HEAD_1       0xEE
#define MOTOR_YS_RECV_HEAD_0       0xFD   //电机→主机 ★与 6014 的 0xFC 不同
#define MOTOR_YS_RECV_HEAD_1       0xEE
#define MOTOR_YS_SEND_LEN          17u
#define MOTOR_YS_RECV_LEN          16u
#define MOTOR_YS_CRC_SEND_LEN      15u    //CRC16 覆盖前 15 B
#define MOTOR_YS_CRC_RECV_LEN      14u    //CRC16 覆盖前 14 B

//量化系数:τ×256 ; ω = ω/(2π)×256 ; θ = θ/(2π)×32768 ; K×1280
//  ⚠️ 发送系数必须与"表1"严格一致,并且**与下面表2的反馈除数互为逆运算**:
//      表1(发送):ω_set = ω/(2π)×256   → 256/(2π)   = 40.743665
//                θ_set = θ/(2π)×32768 → 32768/(2π) = 5215.18917
//      表2(接收):ω = ω_fbk/256×2π , θ = θ_fbk/32768×2π   ✓ 互为逆
//  历史错误:这里曾写成 128.0f / 16384.0f(= ÷2 而不是 ÷2π,**少了 π**),
//  结果"发 1 rad/s、回读 π rad/s",电机实际走 3.14 倍。改这两个值后务必验一遍
//  "发出去再读回来等于原值"(1.0 rad/s → raw 41 → 回读 ≈1.0)。
#define MOTOR_YS_T_SCALE           256.0f
#define MOTOR_YS_W_SCALE           40.743665f    //= 256/(2π)   表1:ω_set = ω/(2π)×256
#define MOTOR_YS_P_SCALE           5215.18917f   //= 32768/(2π) 表1:θ_set = θ/(2π)×32768
#define MOTOR_YS_KP_SCALE          1280.0f
#define MOTOR_YS_KD_SCALE          1280.0f
//反馈侧除数(表2 原公式:ω = ω_fbk/256*2π,θ = θ_fbk/32768*2π)
#define MOTOR_YS_W_FBK_DIV         256.0f
#define MOTOR_YS_P_FBK_DIV         32768.0f

//协议量程(表1,★都是**转子侧**的量)
#define MOTOR_YS_T_MAX_ROTOR       127.99f   //N·m
#define MOTOR_YS_W_MAX_ROTOR       804.0f    //rad/s
#define MOTOR_YS_KP_MAX            25.599f
#define MOTOR_YS_KD_MAX            25.599f
//位置:表1 没有关节限位(θ 是多圈累加 int32),只保证编码不溢出(见 Apply_Model_Limit)

//温度保护(表2:≥90℃ 电机自保护,需重新上电)
#define MOTOR_YS_TEMP_PROTECT      90.0f
//减速比(手册 7.1)
#define MOTOR_YS_GEARBOX_M8010     6.33f

/* ═════════ 协议 B:IM6014(J288/S288)═════════ */

//帧头 / 帧长
#define MOTOR_6014_SEND_HEAD_0     0xFE
#define MOTOR_6014_SEND_HEAD_1     0xEE
#define MOTOR_6014_RECV_HEAD_0     0xFC   //★与 8010 的 0xFD、主机方向的 0xFE 都不同
#define MOTOR_6014_RECV_HEAD_1     0xEE
#define MOTOR_6014_SEND_LEN        20u
#define MOTOR_6014_RECV_LEN        26u
#define MOTOR_6014_CRC_SEND_LEN    16u    //CRC32 覆盖前 16 B(含包头)
#define MOTOR_6014_CRC_RECV_LEN    20u    //CRC32 覆盖 Byte2..21(不含包头)

//量化系数(官方 protocol.c)
#define MOTOR_6014_TOR_SCALE       2560.0f
#define MOTOR_6014_SPD_SCALE       64.0f
#define MOTOR_6014_POS_SCALE       32768.0f
#define MOTOR_6014_KP_SCALE        12800.0f
#define MOTOR_6014_KD_SCALE        51200.0f
//饱和值(官方 SATURATE,输出端;上下限不对称是因为 raw 的 ±32768/32767 不对称)
#define MOTOR_6014_TOR_MAX         162.128385f
#define MOTOR_6014_TOR_MIN         (-162.133333f)
#define MOTOR_6014_SPD_MAX         253.965213f
#define MOTOR_6014_SPD_MIN         (-253.972964f)
#define MOTOR_6014_POS_MAX         32508.539376f
#define MOTOR_6014_POS_MIN         (-32508.539391f)
#define MOTOR_6014_KP_MAX          410.725243f
#define MOTOR_6014_KD_MAX          102.681311f
//减速比(官方 RATIO)
#define MOTOR_6014_RATIO           (38.0f / 3.0f)

/* ═════════ 两代共用的 mode 字节位布局 ═════════ */

#define MOTOR_YS_ID_MASK           0x0Fu
#define MOTOR_YS_ID_MAX            14u    //0~14 可用
#define MOTOR_YS_ID_BROADCAST      15u    //15 = 广播(总线无返回值)
#define MOTOR_YS_STATUS_SHIFT      4u
#define MOTOR_YS_STATUS_MASK       0x07u  //[3bit],bit7 在 8010 是保留位、在 6014 是 TIMEOUT
#define MOTOR_YS_STATUS_LOCK       0u     //锁定(Default)
#define MOTOR_YS_STATUS_FOC        1u     //FOC 闭环
#define MOTOR_YS_STATUS_CALI       2u     //编码器校准
#define MOTOR_6014_TIMEOUT_BIT     0x80u  //6014:主机→电机 1=开启超时保护;电机→主机 1=已触发

//反馈 CRC 校验开关(8010 校验 CRC16、6014 校验 CRC32;置 0 关闭)
#define MOTOR_YS_CHECK_RECV_CRC    1

//电机型号(= 用哪一代协议)
enum motor_YS_Model{
	MOTOR_YS_M8010  = 0,   //GO-M8010-6:17/16 B,CRC16,减速比 6.33
	MOTOR_YS_IM6014 = 1,   //IM6014(J288/S288):20/26 B,CRC32,减速比 38/3
};


//创建宇树电机类
class Class_Motor_YS : public motor_mit {

	//公共
	public:

	/*========== 实现 motor_mit 的纯虚接口 ==========*/

	/**
	 * @brief 初始化(按型号选协议)
	 * @param huart 串口编号(485)⚠️ 波特率 4 Mbps
	 * @param __id  电机ID(0~14;15 = 广播,总线无返回值)
	 * @param __model 型号:8010 或 IM6014(两代协议不通用)
	 * @param __control_mode 控制模式(motor_pos / motor_speed / motor_T)
	 * @param __freq 发送频率:在这里定好(内部 Set_Control_Freq 算出 Control_Period_Us,
	 *        并把首帧置为立即到期);运行中想改再调 Set_Control_Freq()。
	 *        485 是半双工"一问一答",频率不要超过 App 轮询这一路的节拍。
	 * @param __timeout_enable 仅 6014 有意义:是否让电机开启 1 s 超时保护(默认开)
	 * @return void
	 */
	void Init(UART_HandleTypeDef *huart,uint16_t __id,const motor_YS_Model &__model,
	          const motor_Control_mode &__control_mode, const Enum_Motor_Control_Freq &__freq,
	          const bool __timeout_enable = true);

	//发送一帧控制量(8010 = 17 B / 6014 = 20 B,含校验)
	//  ★ 发送前会自检:反馈 STATUS 不在 FOC 且无故障 ⇒ 补一帧使能命令(STATUS=FOC)再发
	void send() override;

	/**
	 * @brief 反馈说"确实在 FOC 闭环"吗?(发送前自检用)
	 *        宇树反馈帧里的 STATUS 字段(0 锁定 / 1 FOC …),recv() 会把它写进 Motor_Status。
	 * @return true = STATUS == FOC
	 */
	bool Is_Enabled() const override { return Motor_Status == Motor_Status_ENABLE; }

	/**
	 * @brief 有没有故障?(发送前自检用)
	 *        8010:反馈包头 bits96-98 的 3bit MERROR;6014:MError32 低 8 位
	 *        —— 两者都汇到 recv_data.MError,0 = 无故障。
	 * @return true = 有故障(此时 send() 不补使能、也不发控制帧)
	 */
	bool Has_Fault() const override { return recv_data.MError != 0u; }

	//解析一帧反馈(8010 = 16 B / 6014 = 26 B,校验包头+CRC)
	void recv(uint32_t _id, const uint8_t *_data, uint8_t _len) override;

	//设置机械零点(两代都无此命令,用软件偏置:把此刻位置记为 0)
	void zero() override;

	//电机使能(发一帧,STATUS = 1 FOC 闭环)
	void enable() override;

	//电机失能(发一帧,STATUS = 0 锁定)
	void lose() override;

	//统一反馈读取(Get_Now_Pos/W/T、Get_Motor_Temperature、Get_MError、
	//Get_Feedback_Mode)由基类提供:非虚函数,直接读基类的 recv_data

	/*========== 品牌自有接口 ==========*/

	//本电机的 485 id(反馈帧 mode 字节低 4 位,接收分发时用)
	inline uint16_t Get_Motor_Id() const;

	//型号(当前用哪一代协议)
	inline motor_YS_Model Get_Model() const;

	//本型号的收发帧长 —— 给 App/BSP 用:
	//  USART_RX485_init_Ex(huart, motor.Get_Recv_Len(), cb)
	//  Transmit 时用 Get_Send_Len()
	inline uint8_t Get_Send_Len() const;
	inline uint8_t Get_Recv_Len() const;

	//减速比
	inline fp32 Get_Gearbox_Rate() const;

	//零位偏置(关节侧弧度):实际发送位置 = Pos + init_Pos
	inline void Set_Init_Pos(const fp32 &__init_Pos);
	inline fp32 Get_Init_Pos() const;

	/* --- 8010 专有 --- */
	//足端力原始值(表2 FORCE 12 bit,0~4095;物理单位手册注明"待确认")
	inline uint16_t Get_Force_Raw() const;
	//上次反馈里电机是否报过温(表2 TEMP ≥ 90℃)
	inline bool Is_Over_Temperature() const;

	/* --- 6014 专有 --- */
	//绕组温度 ℃ / 电机端电压 V / 32 位错误码 / 警告码(ExFlag 3 bit)
	inline uint8_t  Get_Winding_Temperature() const;
	inline fp32     Get_Voltage() const;
	inline uint32_t Get_MError32() const;
	inline uint16_t Get_MWarn() const;
	//电机是否报过超时保护(反馈 mode 的 TIMEOUT 位;发一帧 timeout=0 可清除)
	inline bool Is_Motor_Timeout() const;
	//超时保护开关(写进每一帧 mode 字节的 bit7;仅 6014 用)
	inline void Set_Timeout_Protect(const bool __enable);
	inline bool Get_Timeout_Protect() const;

	//反馈校验错误累计次数(诊断;8010 = CRC16 错,6014 = CRC32 错)
	inline uint32_t Get_Recv_Crc_Error_Count() const;

	//内部
	protected:

	//按通信方式发一帧(走 485/UART)
	void Transmit(uint8_t *_data, uint8_t _len);

	//按型号填:减速比、收发帧长、校验长度、反馈包头、量程
	void Apply_Model_Limit(const motor_YS_Model &__model);

	//组帧 / 解帧(按型号分派到下面两对函数)
	void pack_send_frame();
	bool unpack_recv_frame(const uint8_t *_data);

	//协议 A:GO-M8010-6
	void pack_send_8010();
	bool parse_recv_8010(const uint8_t *_data);

	//协议 B:IM6014
	void pack_send_6014();
	bool parse_recv_6014(const uint8_t *_data);

	/* ---- 对外量 ↔ 协议(转子侧)量的换算 ----
	   角度/角速度: 转子 = 对外 × Gearbox   力矩: 转子 = 对外 ÷ Gearbox
	   MOTOR_YS_JOINT_SIDE_UNITS = 0 时全部退化成恒等(对外量就是协议字段量) */
	inline fp32 Kin_Out2Raw() const;   //θ/ω:对外 → 转子
	inline fp32 Kin_Raw2Out() const;   //θ/ω:转子 → 对外
	inline fp32 Tq_Out2Raw()  const;   //τ  :对外 → 转子
	inline fp32 Tq_Raw2Out()  const;   //τ  :转子 → 对外

	//初始化参数
	Struct_motor_init motor_init{};

	//型号
	motor_YS_Model model = MOTOR_YS_M8010;

	//按型号决定的协议参数(由 Apply_Model_Limit 填)
	uint8_t Send_Len     = MOTOR_YS_SEND_LEN;
	uint8_t Recv_Len     = MOTOR_YS_RECV_LEN;
	uint8_t Crc_Send_Len = MOTOR_YS_CRC_SEND_LEN;
	uint8_t Crc_Recv_Len = MOTOR_YS_CRC_RECV_LEN;
	uint8_t Recv_Head_0  = MOTOR_YS_RECV_HEAD_0;

	//减速比
	fp32 Gearbox_Rate = MOTOR_YS_GEARBOX_M8010;

	//零位偏置(关节侧弧度)
	fp32 init_Pos = 0.0f;

	//6014:超时保护开关(每帧都带)
	bool Timeout_Protect = true;

	//反馈附加信息
	uint16_t Force_Raw = 0;             //8010:足端力原始值
	bool     Over_Temperature = false;  //8010:TEMP ≥ 90℃
	uint8_t  Winding_Temperature = 0;   //6014:绕组温度
	fp32     Voltage = 0.0f;            //6014:电机端电压
	uint32_t MError32 = 0;              //6014:32 位错误码
	uint16_t MWarn = 0;                 //6014:警告码
	bool     Motor_Timeout = false;     //6014:电机报超时
	uint32_t Recv_Crc_Error_Count = 0;  //公共:校验错误计数
};


inline uint16_t Class_Motor_YS::Get_Motor_Id()const{
	return (motor_init.id);
}

inline motor_YS_Model Class_Motor_YS::Get_Model()const{
	return (model);
}

inline uint8_t Class_Motor_YS::Get_Send_Len()const{
	return (Send_Len);
}

inline uint8_t Class_Motor_YS::Get_Recv_Len()const{
	return (Recv_Len);
}

inline fp32 Class_Motor_YS::Get_Gearbox_Rate()const{
	return (Gearbox_Rate);
}

inline void Class_Motor_YS::Set_Init_Pos(const fp32 &__init_Pos){
	init_Pos = __init_Pos;
}

inline fp32 Class_Motor_YS::Get_Init_Pos()const{
	return (init_Pos);
}

inline uint16_t Class_Motor_YS::Get_Force_Raw()const{
	return (Force_Raw);
}

inline bool Class_Motor_YS::Is_Over_Temperature()const{
	return (Over_Temperature);
}

inline uint8_t Class_Motor_YS::Get_Winding_Temperature()const{
	return (Winding_Temperature);
}

inline fp32 Class_Motor_YS::Get_Voltage()const{
	return (Voltage);
}

inline uint32_t Class_Motor_YS::Get_MError32()const{
	return (MError32);
}

inline uint16_t Class_Motor_YS::Get_MWarn()const{
	return (MWarn);
}

inline bool Class_Motor_YS::Is_Motor_Timeout()const{
	return (Motor_Timeout);
}

inline void Class_Motor_YS::Set_Timeout_Protect(const bool __enable){
	Timeout_Protect = __enable;
}

inline bool Class_Motor_YS::Get_Timeout_Protect()const{
	return (Timeout_Protect);
}

inline uint32_t Class_Motor_YS::Get_Recv_Crc_Error_Count()const{
	return (Recv_Crc_Error_Count);
}

/* ---- 单位换算 ---- */
#if (MOTOR_YS_JOINT_SIDE_UNITS)
inline fp32 Class_Motor_YS::Kin_Out2Raw() const { return (Gearbox_Rate); }
inline fp32 Class_Motor_YS::Kin_Raw2Out() const { return (1.0f / Gearbox_Rate); }
inline fp32 Class_Motor_YS::Tq_Out2Raw()  const { return (1.0f / Gearbox_Rate); }
inline fp32 Class_Motor_YS::Tq_Raw2Out()  const { return (Gearbox_Rate); }
#else
inline fp32 Class_Motor_YS::Kin_Out2Raw() const { return (1.0f); }
inline fp32 Class_Motor_YS::Kin_Raw2Out() const { return (1.0f); }
inline fp32 Class_Motor_YS::Tq_Out2Raw()  const { return (1.0f); }
inline fp32 Class_Motor_YS::Tq_Raw2Out()  const { return (1.0f); }
#endif


#endif //MOTOR_YS_H
