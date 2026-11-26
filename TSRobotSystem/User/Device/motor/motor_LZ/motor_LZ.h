//
// Created by 20159 on 2026/3/13.
//
// Class_Motor_LZ — 灵足(LZ)RS 系列电机,继承 motor_mit 并完整实现各接口
//
// 灵足 RS 系列协议(扩展帧):
//   | 类型(5bit) | 数据段(16bit) | 电机ID(8bit) |
//     bits 28-24    bits 23-8       bits 7-0
//   运控帧(类型 0x01):数据段 = 力矩;数据段 8 字节 = data[0..7](位置/W/Kp/Kd)
//   命令帧(类型 0x03/0x04/0x06/0x11/0x12/0x16/0x18…):数据段 = 主机 CAN ID
//   反馈帧(类型 0x02 指令应答 / 0x18 主动上报):数据段 = 电机 ID,
//          bits 23-22 = 电机状态 pattern,bits 21-16 = 故障码
//   使能/失能 = 类型 3 / 类型 4(data[0]=1 顺带清故障)
//   置零 = 类型 6,之后可选补发类型 0x16(保存,数据区 01..08)
//
// ⚠️ 命令帧与反馈帧的 bits 15-8 含义**不一样**(一个放主机 ID、一个放电机 ID),
//    接收分发请用 Is_My_Feedback(_id)。
//
// 设定值/反馈量全部走基类的 send_data / recv_data(弧度、弧度/秒、牛米)。
//

#ifndef MOTOR_LZ_H
#define MOTOR_LZ_H

#include "struct_typedef.h"
#include "bsp_fdcan.h"
#include "motor_mit.h"

//主机(上位机)CAN ID —— 命令帧 ID 的 bits 15-8(官方样例用过 0xFD/0x1F,本工程用 0)
#define motor_LZ_user_id               0x00

/* ===== 16bit 定点编码所必需的协议量程 =====
   ⚠️ 不是关节软限位!这里只负责把物理量线性映射进 16bit,
      必须与上位机设置的 PMAX/VMAX/TMAX 一致,否则整条换算比例都是错的。 */
#define MOTOR_LZ_POS_MAX_DEFAULT       12.57f      /* ≈4π */
#define MOTOR_LZ_W_MAX_DEFAULT         44.0f
#define MOTOR_LZ_T_MAX_DEFAULT         17.0f
#define MOTOR_LZ_KP_MAX                500.0f
#define MOTOR_LZ_KD_MAX                5.0f

/* ===== 参数 0x7005:运行模式 =====
   运控帧只在【运行模式 = 运控模式】时才被执行 —— 被别的上位机改成
   PP位置/速度/电流/CSP 后,板子发的运控帧会被静默忽略(现象:电机不使能、不动)。 */
#define MOTOR_LZ_RUNMODE_MOTION        0u
#define MOTOR_LZ_RUNMODE_POS_PP        1u
#define MOTOR_LZ_RUNMODE_SPEED         2u
#define MOTOR_LZ_RUNMODE_CURRENT       3u
#define MOTOR_LZ_RUNMODE_ZERO          4u
#define MOTOR_LZ_RUNMODE_POS_CSP       5u

/* ===== 电机状态 pattern(反馈帧 ID 的 bits 22-23)=====
   ⚠️ 不是"运行模式":电机上电要跑自己的复位/编码器标定,这期间它不接受使能。 */
#define MOTOR_LZ_PATTERN_RESET         0u
#define MOTOR_LZ_PATTERN_CALI          1u
#define MOTOR_LZ_PATTERN_RUN           2u

/* ===== 温度合理性窗口 =====
   反馈帧 data[6..7] 大端 /10 = ℃。超出窗口置 -1.0(约定 <0 = 无效)——
   实测遇到过恒报 6533.6 ℃(原始 u16 = 0xFF38)的一路哨兵值。 */
#define MOTOR_LZ_TEMP_VALID_MAX        150.0f
#define MOTOR_LZ_TEMP_VALID_MIN        (-40.0f)

/* ===== 等 TX FIFO 空位的超时 [ms](FIFO 只有 8 槽,不等会静默丢帧)===== */
#define MOTOR_LZ_TX_WAIT_TIMEOUT_MS    5u

/* ===== 置零后是否补发官方类型 0x16(数据保存)=====
   写参数(0x12)是"掉电丢失"的,官方注明"搭配类型 0x16 可保存"。 */
#define MOTOR_LZ_ZERO_SEND_SAVE        1u




//电机型号(MOTOR_LZ_0N ↔ 官方的 rs-0N;量程/Kp/Kd 上限见 motor_LZ.cpp 的 Apply_Model_Limit)
enum motor_LZ_Model{
	MOTOR_LZ_00=0,
	MOTOR_LZ_01,
	MOTOR_LZ_02,
	MOTOR_LZ_03,
	MOTOR_LZ_04,
	MOTOR_LZ_05,
	MOTOR_LZ_06,
};

/* 电机通信类型(扩展帧 ID 的 bits 28-24)
   ★ 必须按官方取值!按 enum 顺序排下来的话 PARA_READ/PARA_WRITE/MODE_TOTAL/
     MODE_Bd/FAULT_WARN 全错,那些帧发出去就是废帧。 */
enum canComMode{
	CANCOM_ANNOUNCE_DEVID      = 0x00, //通告设备 ID / 获取 ID
	CANCOM_MOTOR_CTRL          = 0x01, //电机控制 = 运控模式指令(唯一在用的控制帧)
	CANCOM_MOTOR_FEEDBACK      = 0x02, //电机反馈(方向是电机→主机)
	CANCOM_MOTOR_IN            = 0x03, //电机使能
	CANCOM_MOTOR_RESET         = 0x04, //电机失能/停止(data[0]=1 附带清故障)
	CANCOM_MOTOR_CALI          = 0x05, //官方未定义,勿用(旧代码遗留)
	CANCOM_MOTOR_ZERO          = 0x06, //设置机械零位(data[0]=1)
	CANCOM_MOTOR_ID            = 0x07, //设置 CAN ID
	CANCOM_PARA_READ           = 0x11, //参数-读取
	CANCOM_PARA_WRITE          = 0x12, //参数-写入 / 设置控制模式(掉电丢失)
	CANCOM_FAULT_WARN          = 0x15, //故障与警告反馈
	CANCOM_MODE_TOTAL          = 0x16, //电机数据保存到 Flash(数据区必须 01..08)
	CANCOM_MODE_Bd             = 0x17, //波特率修改(重新上电生效)
	CANCOM_MODE_ACTIVE_RECV    = 0x18, //主动上报设置(默认周期 10 ms)
	CANCOM_MODE_AGREEMENT      = 0x19, //协议修改(0私有/1Canopen/2MIT,重新上电生效)
	/* 下面三个官方手册里没有,本工程也没人用 —— 留着只为兼容旧引用,勿发帧 */
	CANCOM_CALI_ING            = 0x7D,
	CANCOM_CALI_RST            = 0x7E,
	CANCOM_PARA_STR_INFO       = 0x7F,
};


//创建灵足电机类
class Class_Motor_LZ : public motor_mit{

	//公共
	public:

	/*========== 实现 motor_mit 的五个纯虚接口 ==========*/

	//初始化(按型号)
	//  __freq 发送频率:在这里定好(内部 Set_Control_Freq 算出 Control_Period_Us,
	//         并把首帧置为立即到期);运行中想改再调 Set_Control_Freq()。
	void Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,const motor_LZ_Model &__model,
	          const motor_Control_mode &__control_mode, const Enum_Motor_Control_Freq &__freq);

	//发送一帧运控帧(16bit 编码,力矩走 ID 的数据段)
	//  ★ 发送前会自检:没在运行(pattern!=2)且无故障 ⇒ 补一次使能帧(0x03)再发
	void send() override;

	/**
	 * @brief 反馈说"确实在运行"吗?(发送前自检用)
	 *        灵足反馈帧 id 的 bits23-22 是电机状态 pattern:0 复位 / 1 标定 / 2 运行
	 *        —— 电机上电/复位后会先跑自己的复位与编码器标定,这期间它不接受使能,
	 *        所以"跑没跑起来"要看这个 pattern。
	 * @return true = pattern == 2(运行)
	 */
	bool Is_Enabled() const override { return Motor_Status == Motor_Status_ENABLE; }

	/**
	 * @brief 有没有故障?(发送前自检用)
	 *        灵足故障码在反馈帧 id 的 bits21-16(0 = 无故障)。
	 * @return true = 有故障(此时 send() 不补使能、也不发运控帧)
	 */
	bool Has_Fault() const override { return recv_data.MError != 0u; }

	//解析一帧数据(0x11 参数应答 → param_answer;0x02/0x18 反馈 → can_recv)
	void recv(uint32_t _id, const uint8_t *_data, uint8_t _len) override;

	//设置机械零位(类型 0x06;结果见 Get_Zero_Result())
	void zero() override;

	//电机使能(类型 0x03)
	void enable() override;

	//电机失能/停止(类型 0x04)
	void lose() override;

	/*========== 官方参数操作(类型 0x11 读 / 0x12 写 / 0x16 保存)==========*/

	//读一个参数(应答是类型 0x11 的帧,值在 data[4..7])
	void read_param(uint16_t index);

	//通用写参数(Byte0~1=Index 小端,Byte2~3=位宽,Byte4~7=值)。⚠️ 掉电丢失,写完要 save_param()
	void write_param(uint16_t index, const void *value, uint8_t width);

	//写一个 u8 参数
	void write_param_u8(uint16_t index, uint8_t value);

	//设置运行模式(0x7005)并立刻回读确认,结果见 Get_RunMode()/Get_RunModeValid()
	void set_run_mode(uint8_t run_mode);

	//保存到电机 Flash(类型 0x16,数据区固定 01..08)
	uint8_t save_param();

	//0x7029 zero_sta:写 1 标零(csp/运控模式下可标零,PP 模式会被屏蔽)
	void set_zero_sta(uint8_t v);

	//0x2024 add_offset:把电机零点再挪 v 弧度(可就地标零,不用把机器人摆到零点姿态)
	void set_add_offset(float v);

	//0x7026 EPSCAN_time:主动上报间隔
	void set_report_time(uint16_t v);

	//主动上报开关:1 开 / 0 关(类型 0x18)
	void active_recv(uint8_t F_CMD);

	//修改 CAN ID(官方类型 0x07;官方载荷约定未核对,启用前先验证)
	void motor_set_CAN_ID(uint8_t set_id);

	/*========== 状态 / 诊断 ==========*/

	//本电机的 CAN id(反馈帧 bits 15-8,接收分发时用)
	inline uint16_t Get_CAN_Id() const;

	//主机(上位机)CAN ID(命令帧 bits 15-8)
	inline uint16_t Get_Host_Id() const;
	inline void Set_Host_Id(uint16_t __user_id);

	//这一帧是不是本电机的反馈帧
	inline bool Is_My_Feedback(uint32_t _id) const;

	//回读到的运行模式(0 = 运控模式)
	inline uint8_t Get_RunMode() const;
	//运行模式是否已回读到
	inline uint8_t Get_RunModeValid() const;
	//清掉"运行模式已回读"标志(重新探测/重读前调用)
	inline void Clear_RunModeValid();
	//电机状态 pattern(0复位/1标定中/2运行中)
	inline uint8_t Get_Pattern() const;
	//最近一次收到本电机反馈帧的 HAL tick(判"还活着"用)
	inline uint32_t Get_LastRxTick() const;
	//最近一次 zero() 的结果:0 = 全部帧已进 TX FIFO;1 = 有帧超时没发出去
	inline uint8_t Get_Zero_Result() const;

	//内部
	protected:

	//参数应答帧处理(类型 0x11 的回帧)—— 只由基类虚接口 recv() 内部调用,不对外
	void param_answer(const uint8_t *data);

	//按通信方式发一帧(灵足用扩展帧,id = 类型/数据段/电机id 组装)
	void Transmit(uint32_t _id, uint8_t *_data, uint8_t _len);

	//发送并等 TX FIFO 空位(0 = 成功)
	uint8_t Transmit_Wait(uint32_t _id, uint8_t *_data, uint8_t _len, uint32_t _timeout_ms);

	//按型号补齐 16bit 编码量程(结构体未指定时使用)
	void Apply_Model_Limit(const motor_LZ_Model &__model);

	//组装扩展帧 id:| 类型(5) | 数据段(16) | 电机id(8) |
	uint32_t make_frame_id(uint32_t Id_data) const;

	//灵足电机扩展帧收发
	void can_send();
	void can_recv(uint32_t Data , const uint8_t *data);

	//初始化参数
	Struct_motor_init motor_init{};
	//型号
	motor_LZ_Model model = MOTOR_LZ_02;

	//当前在发的帧类型
	enum canComMode mode = CANCOM_MOTOR_FEEDBACK;

	//LZ 私有反馈量
	uint8_t  RunMode = 0;          //参数 0x7005 回读值
	uint8_t  RunModeValid = 0;     //RunMode 是否已回读到
	uint32_t LastRxTick = 0;       //最近一次反馈时刻
	uint8_t  zero_result = 0;      //最近一次 zero() 的结果
};


inline uint16_t Class_Motor_LZ::Get_CAN_Id() const
{
	return (motor_init.id);
}

inline uint16_t Class_Motor_LZ::Get_Host_Id() const
{
	return (motor_init.master_id);
}

inline void Class_Motor_LZ::Set_Host_Id(uint16_t __user_id){
	motor_init.master_id = __user_id;
}

inline bool Class_Motor_LZ::Is_My_Feedback(uint32_t _id) const
{
	return (((_id >> 8) & 0xFFu) == motor_init.id);
}

inline uint8_t Class_Motor_LZ::Get_RunMode() const
{
	return (RunMode);
}

inline uint8_t Class_Motor_LZ::Get_RunModeValid() const
{
	return (RunModeValid);
}

inline void Class_Motor_LZ::Clear_RunModeValid(){
	RunModeValid = 0u;
}

inline uint8_t Class_Motor_LZ::Get_Pattern() const
{
	return (recv_data.mode);
}

inline uint32_t Class_Motor_LZ::Get_LastRxTick() const
{
	return (LastRxTick);
}

inline uint8_t Class_Motor_LZ::Get_Zero_Result() const
{
	return (zero_result);
}

#endif //MOTOR_LZ_H
