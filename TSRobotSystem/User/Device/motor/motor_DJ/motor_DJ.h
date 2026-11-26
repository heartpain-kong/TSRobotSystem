//
// Class_Motor_DJ — 大疆(DJI)电机,继承 motor_mit 并重写统一虚接口
//
// 说明:
//  - DJI 电机没有"使能/失能/置零"指令帧,enable/lose 只控制是否继续发送控制帧,
//    zero 通过把当前编码器值记为机械零点实现;
//  - DJI 采用一拖四广播帧:同一总线上 0x201~0x204 / 0x205~0x208 / 0x209~0x20B
//    四台电机共用一帧,发送缓冲按 [总线][组][字节] 组织。
//
// 发送的两个粒度:
//  · 单台一组(或调试):obj.send() / obj.send_group_frame()
//  · 成组使用:Class_Motor_DJ_Group —— 一个对象初始化同组 4 台,Send() 一帧发出
//  (原来那个"把三条总线九组全发一遍"的 motor_DJ_can_send() 已删除:
//   它绕过频率闸门、还直接引用 CubeMX 的全局 hfdcan1/2/3,已被上面两者取代)
//
#ifndef MOTOR_DJ_H
#define MOTOR_DJ_H

#include "struct_typedef.h"
#include "bsp_fdcan.h"
#include "alg_pid.h"
#include "motor_mit.h"

//组内最多 4 台(大疆一拖四)
#define MOTOR_DJ_UNIT_MAX   4u

//电机的型号
enum motor_DJ_Model{
    MOTOR_DJ_M2006=0,
	MOTOR_DJ_M3508,
	MOTOR_DJ_M6020,
	/* ★ 3508 屁股:拆掉 19.2 减速箱(输出轴即转子)⇒ 1:1
	   电机恒差/反馈换算和 M3508 完全一样,只是 Gearbox_Rate=1.0 ✓ */
	MOTOR_DJ_M3508_BARE,

};

//dji电机的CAN_ID
enum motorDJ_CAN_ID{
	CAN_M1_M4_ID = 0x200,
    CAN_M1_ID = 0x201,
    CAN_M2_ID = 0x202,
    CAN_M3_ID = 0x203,
    CAN_M4_ID = 0x204,
	
	CAN_M5_M8_ID = 0x1FF,
    CAN_M5_ID = 0x205,
    CAN_M6_ID = 0x206,
    CAN_M7_ID = 0x207,
	CAN_M8_ID = 0x208,
	
	CAN_M9_MA_ID = 0x2FF,
    CAN_M9_ID = 0x209,
    CAN_MA_ID = 0x20A,
    CAN_MB_ID = 0x20B,
};

//单台电机的接收累加信息(.cpp 里的 dj_recv 就是这个类型)
struct Struct_recv_motor_DJ
{
	uint16_t Pre_Encoder = 0;      //上一次的编码器值
	int32_t  Total_Encoder = 0;    //累计编码器值(含圈数与零点偏移)
	int32_t  Total_Round = 0;      //累计圈数(注意方向)
	bool     First_Frame = true;   //首帧只对齐,不能拿 0 当上一拍做圈数判断
	int32_t  Feedback_Offset = 0;  // 上一帧 CAN 位置使用的零偏
	fp32     Feedback_Position = 0.0f;  // 只由编码器反馈更新
};

//返回数据的结构体
struct Struct_motor_DJ_init
{
	uint16_t id;                   //电机自身 id(CAN 标准帧 id / 485 id)
	uint16_t master_id;            //主机 id(达妙反馈 id / 灵足 user id 等)
	motor_Control_mode mode;
	motor_DJ_Model model;
	Enum_Motor_Status status;

};

//组对象(一个对象初始化同组 4 台 + 一帧发送),定义在 Class_Motor_DJ 之后
class Class_Motor_DJ_Group;


//创建dji电机类
class Class_Motor_DJ : public motor_mit {

	//公共
	public:
    /** 覆盖减垃比:3508屁股(1:1)=1.0;舵向 3508+1:4=4.0 */
    void Set_Gearbox_Rate(const float __rate);

    /* ★ 位置环速度阻尼(可选,默认关闭 = 保持旧行为)
       PID_Angle 的 Kd 是"每拍"增益:1kHz 下 Kd*Δpos 只有 Kd*ω*0.001,
       等于没有阻尼 —— 位置阶跃必然冲过头。真正有效的阻尼要直接用
       反馈转速:  w_cmd = PID_Angle.out - Pos_Vel_Damp * ω反馈
       典型值 1.0~2.0; <=0 = 关闭。 */
    void Set_Pos_Vel_Damp(const float __k);
    /** 位置环速度前馈:直接加到位置环输出的速度目标上(小陀螺反向补偿用) */
    void Set_Pos_Vel_Feedforward(const float __ff);

    /* 位置环输出(速度目标)上限 [rad/s];<=0 = 不额外限制(只受 PID_Angle 自身限幅)。
       必须设成电机真能跑到的速度,否则位置环一直要求不可能的速度,
       速度环长期饱和 ⇒ 到点就是满速冲过去。6020 空载约 33 rad/s。 */
    void Set_Pos_Max_Speed(const float __w);

    /* ★ 刹车包线:位置环指令速度不超过"以该减速度在剩余角度内能停住"的速度
       w_max = sqrt(2 · a_dec · |e|)。远处不限制,接近目标自动减速。
       <=0 = 关闭。典型 30~80 rad/s²。 */
    void Set_Pos_Brake_Decel(const float __a);
    void Set_Pos_Deadband(const float __d);

	//位置环
    Class_PID PID_Angle;

	//速度环
    Class_PID PID_speed;

	/*========== 统一虚接口:重写 motor_mit ==========*/


	//发送一帧控制量(把 out 写入本组广播缓冲并发送本组帧)
	//  失能态也会继续发(强制 out=0),避免电调保持上一拍电流
	void send() override;

	//单台闭环:算 PID,把结果写进本组帧缓冲(之后由 send() 发出去)
	//  组对象用 PID_Calculate_All() 即可,不必逐台调这个
	void Calculate();

	//解析一帧反馈数据
	void recv(uint32_t _id, const uint8_t *_data, uint8_t _len) override;

	//把当前编码器值记为机械零点
	void zero() override;

	//把当前机械位置直接标记为 angle_rad(0 / π 等),用于光电找零后的坐标系重建
	void zero_to_angle(float angle_rad);
	void zero_to_encoder(uint16_t raw_encoder);

	//允许发送控制帧
	void enable() override;

	//停止输出(发一帧 0 电流后置为失能)
	void lose() override;

	//统一反馈接口(Get_Now_Pos/W/T、Get_Motor_Temperature、Get_MError、Get_Feedback_Mode)
	//由基类提供:它们是非虚函数,直接读基类的 recv_data —— can_recv 就写在那里

	/*========== 品牌原有接口(保留) ==========*/

	//初始化(Struct_motor_init 打包版;型号单独传)
	void Init(const Struct_motor_init &_motor_init, motor_DJ_Model __model = MOTOR_DJ_M3508);

	//初始化
	//  __freq 发送频率:直接在这里定好(内部调 Set_Control_Freq 算出 Control_Period_Us,
	//         并让第一帧立即到期);之后 App 用 Is_Send_Due()/Mark_Sent() 排程。
	//         运行中想改频率再调 Set_Control_Freq() 即可。
	void Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,motor_DJ_Model __model,
        const motor_Control_mode &__control_mode, const Enum_Motor_Control_Freq &__freq);

	/*========== 内部实现函数 ==========
	   can_send_Data / PID_Calculate_Data / can_recv / send_group_frame
	   这四个只有**本类内部**和**同组的 Class_Motor_DJ_Group** 会用,不对外暴露 ——
	   声明放在文件下方的 "内部" 段(protected),并在那里把组类声明为友元。
	   和 motor_LZ / motor_YS / motor_DM 的 Transmit / MIT_Pack 等保持同一风格。 */

	//使能/失能状态统一用基类的 Enum_Motor_Status:
	//    Motor_Status + Get_Motor_Status() / Set_Motor_Status()
	//  (原来这里的 Get_Status/Set_Status 是它的一份重复实现,已删)

	inline fp32 Get_Now_pos() const;
	bool Read_Position_Feedback(fp32 &position) const
	{
	    if (dj_recv.First_Frame || dj_recv.Feedback_Offset != Encoder_Offset)
	    {
	        return false;
	    }
	    position = dj_recv.Feedback_Position;
	    return true;
	}
	static constexpr uint16_t ENCODER_NUM_PER_ROUND = 8192u;

	inline fp32 Get_Temperature() const;
	inline fp32 Get_Now_Current_A() const;   /* C620 电流(A,电机侧) */

	//本电机的 CAN 标准帧 id(接收分发时用)
	inline uint16_t Get_CAN_Id() const;
	uint16_t Get_Raw_Encoder() const { return dj_recv.Pre_Encoder; }

	//发送失败次数(TX FIFO 满/总线没 Start);持续增长说明总线太忙或没起来
	inline uint32_t Get_Tx_Fail_Count() const;
	
	//内部
	protected:

	/* ---- 内部实现函数(不对外暴露)----
	   同组的 Class_Motor_DJ_Group 要"逐台"调用它们:一起算 PID、把 4 台的值写进同一帧、
	   整组发一帧 —— 所以把那个类声明为友元,而不是把这几个函数放 public。 */
	friend class Class_Motor_DJ_Group;

	//把 out 写进本组广播缓冲(组内 4 台共用一份 [总线][组] 帧缓冲)
	void can_send_Data();

	//运用 PID 把目标(位置/速度)算成 send_data(位置模式和速度模式各走一遍)
	void PID_Calculate_Data();

	//解析本机一帧反馈(由基类虚接口 recv() 调用)
	void can_recv(const uint8_t *data);

	//本组广播帧的发送(整组只发这一帧)
	//  @return true = 已进 TX FIFO;false = TX FIFO 满/总线没 Start
	bool send_group_frame();

	//CAN几
	FDCAN_HandleTypeDef *FDcan;

	//本电机的 CAN 反馈 id(0x201~0x20B)
	uint16_t CAN_id = 0;

	//总线序号(FDCAN1/2/3 -> 0/1/2)—— can_send_Data / send_group_frame 用
	uint8_t bus_index = 0;

	//组号(0/1/2)—— 由 CAN_id 决定
	uint8_t group_index = 0;

	//型号
	motor_DJ_Model model = MOTOR_DJ_M3508;

	//PID 控制模式直接用基类的成员 Motor_Control_mode(Init 里赋值),
	//  读取用 Get_Motor_Control_mode()

	//本电机的接收累加信息
	Struct_recv_motor_DJ dj_recv;

	//减速比
    float Gearbox_Rate;


    // 一圈编码器刻度

    // 扭矩电流常数, 减速前
    float CURRENT_TO_TORQUE = 0.0f;
    // 电流到输出的转化系数
    float CURRENT_TO_OUT = 0.0f;
    // 最大输出刻度
    float OUT_MAX = 0.0f;    

    // 编码器偏移
    int32_t Encoder_Offset = 0;

	//输入电流
    fp32 out = 0.0f;

    /* 位置环速度阻尼/限速(见 Set_Pos_Vel_Damp / Set_Pos_Max_Speed) */
    fp32 Pos_Vel_Damp   = 0.0f;
    fp32 Pos_Vel_FF     = 0.0f;
    fp32 Pos_Max_Speed  = 0.0f;
    fp32 Pos_Brake_Decel = 0.0f;  /* 刹车包线减速度 [rad/s²],<=0 关闭 */
    fp32 Pos_Deadband    = 0.0f;  /* 位置死区 [rad]:误差小于它就把速度目标置 0,防止定点来回抖 */
    fp32 W_Damp_Filter  = 0.0f;   /* 反馈转速一阶低通(压 CAN 1rpm 量化噪声) */
    bool W_Damp_Ready   = false;
    motor_Control_mode Last_Calc_Mode = motor_speed;   /* 上一拍算 PID 时的模式(检测切入位置环) */

	//发送失败计数(诊断用)
	uint32_t Tx_Fail = 0u;

	/* 组对象要做"整组只发一帧"(直接读写闸门/out/发送时刻),声明为友元。
	   这样 Class_Motor_DJ 本身不用为它加任何 getter/setter。 */
	friend class Class_Motor_DJ_Group;
};



inline fp32 Class_Motor_DJ::Get_Now_pos()const{
	return (recv_data.Now_Pos);
}

inline fp32 Class_Motor_DJ::Get_Temperature() const{
	return (recv_data.Temperature);
}

inline fp32 Class_Motor_DJ::Get_Now_Current_A() const{
	/* CURRENT_TO_OUT 是"计数/A"的倒数刻度:3508 = 16384/20 */
	return (CURRENT_TO_OUT > 0.0f) ? (recv_data.CurrentValue / CURRENT_TO_OUT) : 0.0f;
}

inline uint16_t Class_Motor_DJ::Get_CAN_Id() const{
	return (CAN_id);
}

inline uint32_t Class_Motor_DJ::Get_Tx_Fail_Count() const{
	return (Tx_Fail);
}

inline void Class_Motor_DJ::Set_Pos_Vel_Damp(const float __k){
	Pos_Vel_Damp = (__k > 0.0f) ? __k : 0.0f;
	W_Damp_Ready = false;          /* 下一拍用当前转速重新对齐,避免切换时踢一下 */
}

inline void Class_Motor_DJ::Set_Pos_Max_Speed(const float __w){
	Pos_Max_Speed = (__w > 0.0f) ? __w : 0.0f;
}

inline void Class_Motor_DJ::Set_Pos_Vel_Feedforward(const float __ff){
	Pos_Vel_FF = __ff;
}

inline void Class_Motor_DJ::Set_Pos_Brake_Decel(const float __a){
	Pos_Brake_Decel = (__a > 0.0f) ? __a : 0.0f;
}

inline void Class_Motor_DJ::Set_Pos_Deadband(const float __d){
	Pos_Deadband = (__d > 0.0f) ? __d : 0.0f;
}


/**
 * @brief 大疆"一组"(最多 4 台)的封装 —— 一个对象初始化同组 4 台,一帧发送
 *
 * 设计:**完全不动 Class_Motor_DJ**(它仍然是"一个对象 = 一台电机"),
 *       本类只是**持有 4 台**并负责"一次初始化 + 一帧发送"。
 *
 *   · 想单独操作某台:grp[0].Set_W(...) / grp[1].Get_Now_Pos() —— 就是原来的单台接口
 *   · 想整组一起算/发:grp.PID_Calculate_All() / grp.Send()
 *   · 同组可以混型号(减速比/电流常数本来就在各自对象里)
 *
 * 用法:
 *     static Class_Motor_DJ_Group chassis;              // ⚠️ 别建在栈上(实测 1564 B ≈ 1.5KB)
 *     const uint16_t ids[4] = {CAN_M1_ID, CAN_M2_ID, CAN_M3_ID, CAN_M4_ID};
 *     chassis.Init(&hfdcan1, ids, MOTOR_DJ_M3508, motor_speed, Motor_Control_Freq_1000HZ);
 *
 *     // 1kHz 任务
 *     chassis.PID_Calculate_All();                      // 4 台一起算(并写进本组帧)
 *     if (chassis.Is_Send_Due()) chassis.Send();        // 到点 → **同一帧**发出去
 */
class Class_Motor_DJ_Group
{
	public:

	

//一次初始化 4 台(同一总线、同一组、型号/模式相同;ids[i]==0 视为空位)
	//  __freq 就是这一组的发送频率(4 台共用一帧,所以整组一个频率)
	bool Init(FDCAN_HandleTypeDef *hfdcan, const uint16_t ids[MOTOR_DJ_UNIT_MAX],
	          const motor_DJ_Model __model = MOTOR_DJ_M3508,
	          const motor_Control_mode &__mode = motor_pos,
	          const Enum_Motor_Control_Freq &__freq = Motor_Control_Freq_1000HZ);

	//逐台初始化(想混型号/混模式时用);__freq 只有第一台落进整组频率
	bool Init(FDCAN_HandleTypeDef *hfdcan, uint8_t idx, uint16_t __id,
	          const motor_DJ_Model __model, const motor_Control_mode &__mode,
	          const Enum_Motor_Control_Freq &__freq = Motor_Control_Freq_1000HZ);

	//取第 idx 台(0~3) —— 单台的 set/get/PID 都从这儿走
	inline Class_Motor_DJ &operator[](uint8_t idx);
	inline const Class_Motor_DJ &operator[](uint8_t idx) const;

	//本组挂了几台 / 第 idx 台是否已初始化
	inline uint8_t Get_Unit_Count() const;
	inline bool Is_Used(uint8_t idx) const;

	//4 台一起算 PID(各自的控制模式可以不同)
	void PID_Calculate_All();

	//★ 一帧发出:组内每台的值都在同一帧里,只发一次
	void Send();

	//到点才发(只读):用组内第 0 台代表整组(发送时会给每台都记时刻)
	bool Is_Send_Due() const;

	//设置整组发送频率(转发给每台,保持各台 Is_Send_Due 一致)
	void Set_Control_Freq(const Enum_Motor_Control_Freq &__freq);

	//整组使能 / 整组失能(失能只发**一帧** 0 电流) / 整组标零
	void Enable();
	void Lose();
	void Zero();

	//本组发送帧 id(0x200/0x1FF/0x2FF) / 组号 / 所在总线 / 这一帧的 8 个字节
	inline uint32_t Get_Group_Id() const;
	inline uint8_t  Get_Group_Index() const;
	inline FDCAN_HandleTypeDef *Get_FDCAN() const;
	const uint8_t  *Get_Tx_Frame() const;

	private:

	//组内 4 台(每台就是原来的 Class_Motor_DJ,一行都没改)
	Class_Motor_DJ motors[MOTOR_DJ_UNIT_MAX];
	bool used[MOTOR_DJ_UNIT_MAX] = {false};

	FDCAN_HandleTypeDef *FDcan = nullptr;
	uint8_t group_index = 0xFF;      //0/1/2
	uint8_t unit_count  = 0;
};


inline Class_Motor_DJ &Class_Motor_DJ_Group::operator[](uint8_t idx)
{
	return motors[(idx < MOTOR_DJ_UNIT_MAX) ? idx : (MOTOR_DJ_UNIT_MAX - 1u)];
}

inline const Class_Motor_DJ &Class_Motor_DJ_Group::operator[](uint8_t idx) const
{
	return motors[(idx < MOTOR_DJ_UNIT_MAX) ? idx : (MOTOR_DJ_UNIT_MAX - 1u)];
}

inline uint8_t Class_Motor_DJ_Group::Get_Unit_Count() const
{
	return (unit_count);
}

inline bool Class_Motor_DJ_Group::Is_Used(uint8_t idx) const
{
	return (idx < MOTOR_DJ_UNIT_MAX) && used[idx];
}

inline uint8_t Class_Motor_DJ_Group::Get_Group_Index() const
{
	return (group_index);
}

inline FDCAN_HandleTypeDef *Class_Motor_DJ_Group::Get_FDCAN() const
{
	return (FDcan);
}

inline uint32_t Class_Motor_DJ_Group::Get_Group_Id() const
{
	switch (group_index)
	{
		case 0u: return CAN_M1_M4_ID;
		case 1u: return CAN_M5_M8_ID;
		case 2u: return CAN_M9_MA_ID;
		default: return 0u;
	}
}



#endif //DM02_MOTOR_DJ_H
