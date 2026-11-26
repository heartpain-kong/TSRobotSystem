//
// Created by 20159 on 2026/3/13.
//
// Class_Motor_DM — 达妙(DM)电机,继承纯接口 motor_mit 并完整实现各接口
//
// 达妙 MIT 协议(8 字节标准帧):
//   控制帧  位置16 + 速度12 + Kp12 + Kd12 + 力矩12
//   反馈帧  data[0] = ID(低 4 位) | ERR(高 4 位),然后 位置16 + 速度12 + 力矩12
//           + MOS温度8 + 线圈温度8
//           ★ ERR(_data[0]>>4)是电机的**状态/故障码**,不是"错误标志位":
//               0x0 失能        0x1 使能
//               0x8 超压        0x9 欠压        0xA 过流
//               0xB MOS过温     0xC 线圈过温     0xD 通讯丢失   0xE 过载
//             (按达妙手册;若与你的手册版本有出入以手册为准)
//             ⇒ MIT_Parse 里 recv_data.MError = _data[0]>>4 拿到的就是这个码,
//               只有 ERR==1 才代表"电机确实已使能"。
//   使能/失能/置零 = FF..FC / FF..FD / FF..FE
//
//   ★ 反馈是"每收到一帧命令就回一帧"(发使能帧也会回一帧,带 ERR=1),
//     所以靠反馈判状态是可行的 —— 但要看清楚下面的闸门说明。
//
// 频率控制:**状态在电机里,判断在 App 里**
//   · Init 收频率 → Set_Control_Freq() 算出 Control_Period_Us,并让第一帧立即到期
//   · send() 只管发,发完 Mark_Sent() 把时刻记进基类的 send_time
//   · App 每拍:  if (motor_dm[i].Is_Send_Due()) motor_dm[i].send();
//     Is_Send_Due() 是**只读**判断(距上次发送 >= 一个周期),不改任何状态,
//     所以"先判断再发"是安全的,不存在"判过一次就永远不再到期"的坑。
//   ⚠️ 带宽:DM 是一台一帧(标准帧 ≈111bit),而且电机自己还在发反馈帧。
//      单总线 1Mbps 下 4 台 @1kHz 控制帧 ≈44%,反馈帧再来一份 ≈88% —— 别只看控制帧。
//
// 状态语义(与基类一致):Send_Status_Cmd = 我们命令它怎样(闸门);
//                      Motor_Status   = 电机反馈 ERR 报回来的实际状态
//

#ifndef MOTOR_DM_H
#define MOTOR_DM_H


#include "struct_typedef.h"
#include "bsp_fdcan.h"
#include "motor_mit.h"

//电机反馈帧的 CAN_id(达妙上位机里的 Master ID),按实际电机参数修改
#define motor_DM_master_id 0

/* 反馈帧要不要校验"这帧是不是我这台"?
 *   达妙反馈帧的 **CAN id 是共用的 Master ID**,没法用它区分电机;
 *   真正带电机身份的是**数据里 data[0] 的低 4 位 = 电机自己的 id**。
 *   ⇒ 总线上挂多台 DM 时必须校验(不校验会互相污染 recv_data),所以默认打开。
 *   如果你的电机 id 配置与上位机对不上、导致收不到反馈,把它置 0 先排查。
 */
#define MOTOR_DM_CHECK_FEEDBACK_ID   1

/* ---- 反馈帧 data[0] 高 4 位 = ERR(状态/故障码)----
 *   0x0 失能          0x1 使能
 *   0x8 超压  0x9 欠压  0xA 过流  0xB MOS过温  0xC 线圈过温  0xD 通信丢失  0xE 过载
 *
 * 其中 0xD(通信丢失)是**我们自己停发**造成的,属于可自愈的一类:
 * 继续发帧(必要时补一次使能帧)就能恢复 ⇒ send() 里的"发送前自检"允许对它自动重使能。
 * 其余 0x8~0xE 是**真故障**,故障源没消失前不要硬使能(要上层 Clear_Error() 后再说)。
 */
#define MOTOR_DM_ERR_DISABLE    0x0u    /* 失能 */
#define MOTOR_DM_ERR_ENABLE     0x1u    /* 使能 */
#define MOTOR_DM_ERR_LOST       0xDu    /* 通信丢失(看门狗) */


//电机型号
enum motor_DM_Model{
	MOTOR_DM_J10010L = 0,
	MOTOR_DM_J4310	=1,
	MOTOR_DM_G6220	=2,
};


//创建达妙电机类
class Class_Motor_DM : public motor_mit{
	//公共
	public:

	/*========== 实现 motor_mit 的五个纯虚接口 ==========*/

	//初始化(按型号)
	//  __freq:运控帧发送频率(必填)。建议 ≤500Hz —— DM 一台一帧,电机自身还有反馈帧,
	//         单总线 1kHz 时控制帧+反馈帧能吃到 ~88%。
	void Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,motor_DM_Model __model,
	const motor_Control_mode &__control_mode , const Enum_Motor_Control_Freq &__freq);

	//发送一帧 MIT 控制量
	//  ★ 发送前会自检"是否已使能":未使能且无(硬)故障 ⇒ 自动补一次 enable() 再发
	void send() override;

	/**
	 * @brief 有没有"需要人工处理"的故障?(发送前自检用)
	 *
	 * ERR = 0x8~0xE 里只有 **0xD(通信丢失)算可自愈** —— 那是我们自己停发造成的,
	 * 继续发帧/补使能就能恢复;其余(超压/欠压/过流/过温/过载)是真故障,
	 * 故障源没消失前不要自动使能。
	 * @return true = 有真故障(此时 send() 不使能也不发帧)
	 */
	bool Has_Fault() const override {
		const uint8_t err = recv_data.MError;
		return (err >= 0x8u) && (err != MOTOR_DM_ERR_LOST);
	}

	/**
	 * @brief 电机当前是否已被使能(按反馈里的 ERR 判断)
	 * @return true = ERR 报的是 0x1(使能)
	 */
	bool Is_Enabled() const override { return recv_data.MError == MOTOR_DM_ERR_ENABLE; }

	//解析一帧 MIT 反馈
	void recv(uint32_t _id, const uint8_t *_data, uint8_t _len) override;

	//设置机械零点(FF..FE)
	void zero() override;

	//电机使能(FF..FC)
	void enable() override;

	//电机失能(FF..FD)
	void lose() override;

	/**
	 * @brief 清除电机错误(FF..FB)—— 达妙独有的特殊帧
	 *
	 * 用途:电机报故障后(ERR = 0x8 过压 / 0x9 欠压 / 0xA 过流 / 0xB MOS 过温 /
	 *       0xC 线圈过温 / 0xD 通信丢失 / 0xE 过载),即使故障源已经消失,电机也**不会
	 *       自己恢复**;必须发这一帧清错,再 enable() 才会重新接受控制。
	 *       典型用法(放在 App 的 1kHz 任务或一个低速监护任务里):
	 *           if (dm.Get_MError() != 0x1 && dm.Get_MError() != 0x0) {
	 *               dm.Clear_Error();   // 清错
	 *               dm.enable();        // 重新使能
	 *           }
	 * @note 达妙自带通信超时保护,主机看门狗不能替代它;
	 *       ERR = 0xD(通信丢失)时清错 + 重使能同样适用。
	 */
	void Clear_Error();

	//本电机的 CAN id(发送用)
	inline uint16_t Get_CAN_Id() const;

	//内部
	protected:

	//按通信方式发一帧(达妙用标准帧,id = 电机 CAN_ID)
	void Transmit(uint32_t _id, uint8_t *_data,  uint8_t _len);

	//MIT 协议组帧 / 解帧
	void MIT_Pack(uint8_t *_data) const;
	void MIT_Parse(const uint8_t *_data);

	void Apply_Model_Limit(const motor_DM_Model &__model);

	//初始化参数
	Struct_motor_init motor_init{};
	//主机/反馈 id
	uint16_t mst_id = motor_DM_master_id;
	//型号
	motor_DM_Model model = MOTOR_DM_J4310;
	//注:这里原来还有一个 `uint32_t send_time;` —— 它**遮蔽**了基类 motor_mit 的
	//    send_time(Mark_Sent/Is_Send_Due 用的那个),而且从未赋值。已删除,
	//    需要读发送时刻请用基类的 Get_Send_Time()/Get_Send_Interval_Us()。
};

inline uint16_t Class_Motor_DM::Get_CAN_Id() const
{
	return (motor_init.id);
}


#endif //DM02_MOTOR_DM_H
