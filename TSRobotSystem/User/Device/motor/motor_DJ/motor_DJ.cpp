//
// Created by 20159 on 2026/3/13.
//

#include "motor_DJ.h"
#include "math_support.h"

#include <math.h>       /* fabsf / sqrtf(刹车包线) */

//dji的1拖4的发送数据 [总线][组][字节]
//  总线:FDCAN1=0 FDCAN2=1 FDCAN3=2
//  组  :0 -> 0x200(M1~M4) 1 -> 0x1FF(M5~M8) 2 -> 0x2FF(M9~MA)
static uint8_t CAN_Tx_Data[3][3][8];

//三组广播帧的 id
static const uint32_t CAN_Group_ID[3] = {CAN_M1_M4_ID, CAN_M5_M8_ID, CAN_M9_MA_ID};

/**
 * @brief dji电机的初始化(统一入口)
 * @param _motor_init 初始化参数
 * @param __model     型号(Struct_motor_init 里没有型号字段,单独传)
 * @return void
 */
void Class_Motor_DJ::Set_Gearbox_Rate(const float __rate)
{
    if (__rate > 0.0f)
    {
        Gearbox_Rate = __rate;
    }
}

void Class_Motor_DJ::Init(const Struct_motor_init &_motor_init, motor_DJ_Model __model)
{
    /* Struct_motor_init.control_mode 就是公共枚举 motor_Control_mode
       (motor_pos / motor_speed / motor_T),与 DJ 用的完全同一套,直接透传;
       发送频率取结构体里的 Control_Freq(没设就是默认值 1000Hz) */
    Init(_motor_init.FDcan, _motor_init.id, __model,
         _motor_init.control_mode, _motor_init.Control_Freq);
}

/**
 * @brief dji电机的初始化
 * @param hfdcan CAN编号
 * @param __id   CAN_ID
 * @param __model 电机的型号 MOTOR_DJ_M2006=0 & MOTOR_DJ_M3508 & MOTOR_DJ_M6020
 * @param __control_mode pid的控制模式  motor_pos & motor_speed & motor_T
 * @param __freq 发送频率(在这里就把周期定好,App 用 Is_Send_Due() 排程)
 * @return void
 */

void Class_Motor_DJ::Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,motor_DJ_Model __model,
        const motor_Control_mode &__control_mode, const Enum_Motor_Control_Freq &__freq)
{
    FDcan = hfdcan;
    CAN_id = __id;
    model = __model;
    Motor_Control_mode = __control_mode;   //基类成员

    /* ★ 发送频率:Set_Control_Freq 算出 Control_Period_Us(= 1000000/频率)并把首帧置为立即到期;
       之后 send() 内部 Mark_Sent() 记时刻,App 用 Is_Send_Due() 判断该不该发 */
    Set_Control_Freq(__freq);

    //总线序号
    if (hfdcan != nullptr && hfdcan->Instance == FDCAN2)      bus_index = 1;
    else if (hfdcan != nullptr && hfdcan->Instance == FDCAN3) bus_index = 2;
    else                                                      bus_index = 0;

    //组序号:0x201~0x204 -> 0,0x205~0x208 -> 1,0x209~0x20B -> 2
    if (CAN_id >= CAN_M1_ID && CAN_id <= CAN_MB_ID)
        group_index = (uint8_t)((CAN_id - CAN_M1_ID) / 4);
    else
        group_index = 0;

    if (group_index > 2) group_index = 2;

    /* 复位所有运行态:重复 Init 时不会带着上一次的命令位/目标/积分跑 */
    Send_Status_Cmd = Motor_Status_DISABLE;
    Motor_Status    = Motor_Status_DISABLE;
    send_data       = Struct_send_motor{};
    out             = 0.0f;
    Tx_Fail         = 0u;
    PID_Angle.Clear();
    PID_speed.Clear();
    W_Damp_Filter   = 0.0f;
    W_Damp_Ready    = false;
    Last_Calc_Mode  = motor_speed;
    dj_recv = Struct_recv_motor_DJ{};
    Encoder_Offset = 0;

    switch (model)
    {
        case MOTOR_DJ_M2006:
            Gearbox_Rate = 36.0f;
            // 扭矩电流常数
            CURRENT_TO_TORQUE = 0.18f / 36.0f;
            // 扭矩电流到输出的转化系数
            CURRENT_TO_OUT = 10000.0f / 10.0f;
            // 最大输出刻度
            OUT_MAX = 10000.0f;
        break;
        case MOTOR_DJ_M3508:

            Gearbox_Rate=19.2f;
            CURRENT_TO_TORQUE = 0.3f / (3591.0f / 187.0f);
            CURRENT_TO_OUT = 16384.0f / 20.0f;
            OUT_MAX = 16384.0f;  

        break;
        case MOTOR_DJ_M3508_BARE:
            /* ★ 3508 屁股(无减速箱,1:1):
               电流/扭矩常数同 M3508,Geabox_Rate=1.0
               ⇒ Get_Now_pos()/Get_Now_W()/Get_Now_T() 直接就是**转子侧**的值 ✓ */
            Gearbox_Rate = 1.0f;
            CURRENT_TO_TORQUE = 0.3f / (3591.0f / 187.0f);
            CURRENT_TO_OUT = 16384.0f / 20.0f;
            OUT_MAX = 16384.0f;
        break;
        case MOTOR_DJ_M6020:

            Gearbox_Rate=1.0f;
            CURRENT_TO_TORQUE = 0.741f;
            CURRENT_TO_OUT = 16384.0f / 3.0f;
            OUT_MAX = 16384.0f;

        break;
    }
}

/**
 * @brief dji电机的数据的存储:把 out 拆成两个字节写入本总线本组的发送缓冲
 * @return void
 */
void Class_Motor_DJ::can_send_Data(){
    if (bus_index > 2 || group_index > 2) return;
    if (CAN_id < CAN_M1_ID || CAN_id > CAN_MB_ID) return;

    //本电机在组内的字节偏移:组内第 n 台(0~3) -> 第 2n、2n+1 字节
    uint8_t offset = (uint8_t)(((CAN_id - CAN_M1_ID) % 4) * 2);
    uint16_t tmp = (uint16_t)(int16_t)out;

    CAN_Tx_Data[bus_index][group_index][offset]     = (uint8_t)(tmp >> 8);
    CAN_Tx_Data[bus_index][group_index][offset + 1] = (uint8_t)(tmp & 0xFF);
}

/**
 * @brief 发送本电机所在总线、所在组的广播帧
 * @return true = 已进 TX FIFO;false = 参数错 / TX FIFO 满 / 总线没 Start
 */
bool Class_Motor_DJ::send_group_frame(){
    if (bus_index > 2 || group_index > 2 || FDcan == nullptr) return false;

    return (fdcan_send_data_stand(FDcan, CAN_Group_ID[group_index],
                                  CAN_Tx_Data[bus_index][group_index], 8) == 0u);
}

/**
 * @brief 运用pid处理数据
 * @return void
 */
void Class_Motor_DJ::PID_Calculate_Data(){
    /* 命令位 DISABLE:不跑 PID,直接把自己的槽位写 0
       (组里其它台照常算;不能让失能的那台被 PID 算出非零电流) */
    if (Send_Status_Cmd != Motor_Status_ENABLE)
    {
        out = 0.0f;
        can_send_Data();
        return;
    }

    /* 从别的模式切进位置环:阻尼基准要用当前转速重新对齐,否则会拿旧值踢一下 */
    if ((Motor_Control_mode == motor_pos) && (Last_Calc_Mode != motor_pos))
    {
        W_Damp_Ready = false;
    }
    Last_Calc_Mode = Motor_Control_mode;

    switch (Motor_Control_mode)
    {
        case (motor_T):
        {
        break;
        }
        case (motor_speed):
        {
            PID_speed.Set_Target(send_data.W);
            PID_speed.Set_Now(recv_data.Now_W);
            PID_speed.Cout();

            send_data.T = PID_speed.Get_Out();

            break;
        }
        case (motor_pos):
        {
            /* ★ 180° 边界消歧:目标与当前正好差 π 时,最短路径左右都成立,
               浮点噪声会让误差在 +π/-π 之间来回跳 ⇒ 电机左右来回选方向。
               给目标加一个极小的固定偏置(1e-4 rad ≈ 0.006°,远小于精度),
               把"正好 π"变成"π-1e-4",方向就固定了。 */
            PID_Angle.Set_Target(Math_Angle_Wrap_2Pi(send_data.Pos) + 1.0e-4f);
            PID_Angle.Set_Now(recv_data.Now_Pos);

            const fp32 previous_error = PID_Angle.Get_Error();
            PID_Angle.Cout();

            fp32 w_cmd = PID_Angle.Get_Out();
            const fp32 position_error = PID_Angle.Get_Error();

            /* 同一台电机的位置误差变号时，清掉速度环的旧积分。 */
            if (position_error * previous_error < 0.0f)
            {
                PID_speed.Clear();
            }

            /* ★ 位置环速度阻尼:直接用实测转速,不用位置差分。
               PID_Angle 的 Kd 是"每拍"增益(1kHz 下 Kd*ω*0.001),几乎没有阻尼;
               而 6020/舵轮这种"位置→速度→电流"的串级,超调几乎全部来自
               "到点前没有刹车"。这里用反馈转速做真正的阻尼项。 */
            if (Pos_Vel_Damp > 0.0f)
            {
                if (!W_Damp_Ready)
                {
                    /* 第一拍只对齐基准,不参与,避免使能瞬间 D 项踢一下 */
                    W_Damp_Filter = recv_data.Now_W;
                    W_Damp_Ready  = true;
                }
                else
                {
                    /* 一阶低通 α=0.05 ≈ 20ms 时间常数(1kHz):压掉 1rpm 量化抖动 */
                    W_Damp_Filter += 0.05f * (recv_data.Now_W - W_Damp_Filter);
                }
                w_cmd -= Pos_Vel_Damp * W_Damp_Filter;
            }

            /* ★ 刹车包线:离目标越近,允许的速度越低。
               w_allow = sqrt(2·a_dec·|e|) —— 以 a_dec 减速,正好在目标处停住。
               远处不受影响(照样能到 Pos_Max_Speed),只有快到时才自动收油,
               这样 180° 这种大阶跃不会"满速冲过去再荡回来"。 */
            if (Pos_Brake_Decel > 0.0f)
            {
                const fp32 e_abs = fabsf(position_error);
                fp32 w_allow = sqrtf(2.0f * Pos_Brake_Decel * e_abs);

                if (w_cmd >  w_allow) { w_cmd =  w_allow; }
                if (w_cmd < -w_allow) { w_cmd = -w_allow; }
            }

            /* 位置死区:到点后把速度目标置 0,避免在目标附近来回抖 */
            if ((Pos_Deadband > 0.0f) &&
                (fabsf(position_error) < Pos_Deadband))
            {
                w_cmd = 0.0f;
            }

            /* ★ 速度前馈放在刹车包线/死区之后:
               小陀螺要靠它直接把"云台该反向转多快"给出去;
               放在死区之前会被置 0(误差小的时候),云台就跟不上底盘了。 */
            w_cmd += Pos_Vel_FF;

            /* 速度目标限幅:位置环不能要求电机跑不到的速度,否则到点就是满速 */
            if (Pos_Max_Speed > 0.0f)
            {
                if (w_cmd >  Pos_Max_Speed) { w_cmd =  Pos_Max_Speed; }
                if (w_cmd < -Pos_Max_Speed) { w_cmd = -Pos_Max_Speed; }
            }

            send_data.W = w_cmd;

            PID_speed.Set_Target(send_data.W);
            PID_speed.Set_Now(recv_data.Now_W);
            PID_speed.Cout();

            send_data.T = PID_speed.Get_Out();

            break;
        }
        default:
        {
            send_data.T = 0.0f;

            break;
        }
    }
    out = (send_data.T) / CURRENT_TO_TORQUE / Gearbox_Rate * CURRENT_TO_OUT;
    out = motor_max_min(out, OUT_MAX, -OUT_MAX);
	can_send_Data();
}

/**
 * @brief 单台闭环入口:算 PID 并把 out 写进本组帧缓冲
 * @note 组对象用 PID_Calculate_All() 即可;单台 App 在 send() 前调这个
 */
void Class_Motor_DJ::Calculate(){
    PID_Calculate_Data();
}

/**
 * @brief dji电机的返回数据处理
 * @param *data 接收数组的指针 
 * @return void
 */    
void Class_Motor_DJ::can_recv(const uint8_t *data){
    
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;

    tmp_encoder = (uint16_t)((data[0]<<8)|data[1]);
	tmp_omega =  (int16_t)((data[2]<<8)|data[3]);
	tmp_current = (int16_t)((data[4]<<8)|data[5]);

    // 计算圈数与总编码器值
    if (dj_recv.First_Frame)
    {
        /* ★ 首帧 Pre_Encoder=0,不能拿它做圈数判断:
           若上电时编码器值 > 4096,会被误判成"反方向转过一圈",
           角度直接偏 90°(2π/4)。首帧只对齐。 */
        dj_recv.First_Frame = false;
    }
    else
    {
        delta_encoder = (int16_t)(tmp_encoder - dj_recv.Pre_Encoder);
        if (delta_encoder < -ENCODER_NUM_PER_ROUND / 2)
        {
            // 正方向转过了一圈
            dj_recv.Total_Round++;
        }
        else if (delta_encoder > ENCODER_NUM_PER_ROUND / 2)
        {
            // 反方向转过了一圈
            dj_recv.Total_Round--;
        }
    }
    dj_recv.Total_Encoder = dj_recv.Total_Round * ENCODER_NUM_PER_ROUND + tmp_encoder + Encoder_Offset;

    // 计算电机本身信息
    /* 电机内部位置统一归一到 [0, 2π):超过 360° 折回,不再无限累加 */
    recv_data.Now_Pos = Math_Angle_Wrap_2Pi(
        (float) dj_recv.Total_Encoder / (float) ENCODER_NUM_PER_ROUND * 2.0f * PI / Gearbox_Rate);
    recv_data.Now_W = (float) tmp_omega * MATH_RPM_TO_RADPS /Gearbox_Rate;
    recv_data.Now_T = tmp_current / CURRENT_TO_OUT * CURRENT_TO_TORQUE *Gearbox_Rate;
    recv_data.CurrentValue = tmp_current;      /* C620 原始电流计数,±16384 ↔ ±20A */
    recv_data.Temperature = data[6];
   
    // 存储预备信息
    dj_recv.Pre_Encoder = tmp_encoder;
    dj_recv.Feedback_Offset = Encoder_Offset;
    dj_recv.Feedback_Position = recv_data.Now_Pos;

    /* 大疆反馈里没有"使能/模式"字段:Motor_Status 只镜像命令值(enable/lose),
       这里**不要**按反馈改它 —— 否则 lose() 之后收到一帧又被"改回使能"。
       "是否真的收到反馈"由 App 侧的时间戳(dj_rx_us)判断。 */
}

/**
 * @brief 解析一帧反馈数据(统一入口)
 * @param _id   收到帧的 id,为 0 表示不校验
 * @param _data 数据指针
 * @param _len  数据长度
 * @return void
 */
void Class_Motor_DJ::recv(uint32_t _id, const uint8_t *_data, uint8_t _len) {
    if (_data == nullptr || _len < 7) return;
    if (_id != 0u && _id != CAN_id) return;

    can_recv(_data);
}

/**
 * @brief 发送一帧控制量(把当前 out 写入本组广播缓冲后发送本组帧)
 * @return void
 */
void Class_Motor_DJ::send() {
    /* 大疆没有"失能帧":电调靠持续收帧 + 电流指令工作。
       · 命令位 ENABLE:发当前 out
       · 命令位 DISABLE:仍然继续发,但强制 out=0
         (不能停发 —— 停发时电调保持上一拍电流)
       · 不在这里做"自动补使能":急停(lose)之后绝不自愈 */
    if (Send_Status_Cmd != Motor_Status_ENABLE)
    {
        out = 0.0f;
    }

    can_send_Data();

    if (send_group_frame())
    {
        Mark_Sent();      //记录本拍时刻,供 App 的 Is_Send_Due() 判断
    }
    else
    {
        Tx_Fail++;        //TX FIFO 满/总线没起来:不记时刻,下一拍重试
    }

    /* ⚠️ 大疆是一拖四:同组 4 台的 send_time 会各自记,App 若逐台判断就会把同一帧
       按各自相位发出去(DJI 那组一帧可能变成最多 4 帧)。所以 DJI 建议由"组长"
       或 App 按组统一发一次,组内其它电机不要单独判 Is_Send_Due()。 */
}

/**
 * @brief 允许发送控制帧
 * @return void
 */
void Class_Motor_DJ::enable() {
    PID_Angle.Clear();
    PID_speed.Clear();
    W_Damp_Filter = 0.0f;
    W_Damp_Ready  = false;
    out = 0.0f;
    Send_Status_Cmd = Motor_Status_ENABLE;
    Motor_Status    = Motor_Status_ENABLE;   //大疆无模式回读 → 镜像命令值
}

/**
 * @brief 停止输出:先发一帧 0 电流再置为失能
 * @return void
 */
void Class_Motor_DJ::lose() {
    out = 0.0f;
    can_send_Data();
    if (!send_group_frame())
    {
        Tx_Fail++;
    }

    PID_Angle.Clear();
    PID_speed.Clear();
    W_Damp_Filter = 0.0f;
    W_Damp_Ready  = false;
    send_data = Struct_send_motor{};          //清目标 ⇒ 下次使能不会冲旧目标
    Send_Status_Cmd = Motor_Status_DISABLE;
    Motor_Status    = Motor_Status_DISABLE;  //大疆无模式回读 → 镜像命令值
}

/**
 * @brief 把当前编码器值记为机械零点
 * @return void
 */
void Class_Motor_DJ::zero() {
    zero_to_angle(0.0f);
}

/**
 * @brief 把当前机械位置标记为指定角度(光电找零)
 * @param angle_rad 期望的当前角度(rad),例如 0 或 PI
 *
 * 编码器换算:
 *   Now_Pos = Total_Encoder / 8192 * 2π / Gearbox_Rate
 * 所以 angle_rad 对应:
 *   counts = angle_rad * 8192 * Gearbox_Rate / (2π)
 */
void Class_Motor_DJ::zero_to_encoder(uint16_t raw_encoder)
{
    Encoder_Offset = -(int32_t)raw_encoder;
    dj_recv.Total_Encoder = 0;
    recv_data.Now_Pos = 0.0f;
}

void Class_Motor_DJ::zero_to_angle(float angle_rad) {
    const float counts_per_rad =
        (float)ENCODER_NUM_PER_ROUND * Gearbox_Rate / (2.0f * PI);
    int32_t counts = (int32_t)(angle_rad * counts_per_rad);

    //把"当前累计编码器值"记为 counts:Total_Round 不能清零,否则偏移与圈数不再对应
    Encoder_Offset = -(dj_recv.Total_Round * ENCODER_NUM_PER_ROUND
                       + (int32_t)dj_recv.Pre_Encoder)
                     + counts;

    dj_recv.Total_Encoder = counts;
    recv_data.Now_Pos = Math_Angle_Wrap_2Pi(angle_rad);
}

/* ==================== 大疆"一组"的封装 ====================
   一个对象初始化同组 4 台 + 同一帧发送;Class_Motor_DJ 本身不做任何改动。
   =========================================================== */

/**
 * @brief 一次初始化 4 台(同一总线、同一组)
 * @param hfdcan 该组所在总线
 * @param ids    4 个反馈 id(0x201~0x20B);ids[i]==0 视为空位
 * @param __model 型号(4 台相同;要混型号请用逐台版 Init)
 * @param __mode  PID 控制模式(4 台相同)
 * @return true = 全部成功(有一个非法就整体不动)
 */
bool Class_Motor_DJ_Group::Init(FDCAN_HandleTypeDef *hfdcan, const uint16_t ids[MOTOR_DJ_UNIT_MAX],
                                const motor_DJ_Model __model, const motor_Control_mode &__mode,
                                const Enum_Motor_Control_Freq &__freq)
{
    if (hfdcan == nullptr || ids == nullptr) return false;

    //先把 4 个 id 全校验一遍(非空、合法、同组),避免"初始化了一半"
    uint8_t g = 0xFFu;
    uint8_t n = 0;

    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (ids[i] == 0u) continue;
        if (ids[i] < CAN_M1_ID || ids[i] > CAN_MB_ID) return false;

        uint8_t gi = (uint8_t)((ids[i] - CAN_M1_ID) / 4u);
        if (gi > 2u) return false;

        if (g == 0xFFu) g = gi;
        else if (gi != g) return false;          //不在同一组
        n++;
    }
    if (n == 0u) return false;

    //再逐个真正初始化(频率整组一致,4 台共用一帧)
    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (ids[i] == 0u) continue;
        if (!Init(hfdcan, i, ids[i], __model, __mode, __freq)) return false;
    }
    return true;
}

/**
 * @brief 逐台初始化(想混型号/混模式时用)
 *
 * 注:Class_Motor_DJ::Init() 返回 void,所以这里**自己先把 id/总线/组校验一遍**,
 *     校验通过才调用它 —— 不需要改动单台那版。
 * @param __freq 发送频率:整组共用一帧,所以以**第一台**传进来的频率为准,
 *               后面几台传的频率会被忽略(想改就运行中调 Set_Control_Freq)
 * @return true = 成功(同一台不能初始化两次;不在同一组/同一总线会失败)
 */
bool Class_Motor_DJ_Group::Init(FDCAN_HandleTypeDef *hfdcan, uint8_t idx, uint16_t __id,
                                const motor_DJ_Model __model, const motor_Control_mode &__mode,
                                const Enum_Motor_Control_Freq &__freq)
{
    if (hfdcan == nullptr || idx >= MOTOR_DJ_UNIT_MAX) return false;
    if (used[idx]) return false;                    //这一台已经初始化过
    if (__id < CAN_M1_ID || __id > CAN_MB_ID) return false;

    //组号由 CAN_id 决定(4 台必须同组)
    uint8_t g = (uint8_t)((__id - CAN_M1_ID) / 4u);
    if (g > 2u) return false;

    if (group_index == 0xFFu)
    {
        group_index = g;
        FDcan       = hfdcan;
    }
    else if (g != group_index || hfdcan != FDcan)
    {
        return false;                               //不在同一组/同一总线
    }

    //组内所有台都用同一个发送频率(以第一台为准),否则 Is_Send_Due 各说各话
    Enum_Motor_Control_Freq freq = (unit_count == 0u) ? __freq : motors[0].Get_Control_Freq();

    motors[idx].Init(hfdcan, __id, __model, __mode, freq);   //单台初始化

    used[idx] = true;
    unit_count++;

    return true;
}

/**
 * @brief 4 台一起算 PID(各自的控制模式可以不同)
 *        PID_Calculate_Data() 内部已经把各自的 out 写进本组帧缓冲,所以之后只需 Send()
 * @return void
 */
void Class_Motor_DJ_Group::PID_Calculate_All()
{
    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (used[i]) motors[i].PID_Calculate_Data();
    }
}

/**
 * @brief 一帧发出:组内 4 台的值在同一帧里,只发一次
 * @return void
 */
void Class_Motor_DJ_Group::Send()
{
    if (unit_count == 0u || FDcan == nullptr) return;

    //找一台已初始化的发本组帧(帧 id 由组号决定,组内一致)
    bool sent = false;

    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (used[i])
        {
            sent = motors[i].send_group_frame();
            if (!sent) motors[i].Tx_Fail++;
            break;
        }
    }

    if (!sent) return;

    //给每台都记发送时刻 → App 可以用 grp[0].Is_Send_Due() 代表整组排程
    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (used[i]) motors[i].Mark_Sent();
    }
}

/**
 * @brief 这一组该不该发(只读)
 * @return true = 到点了
 */
bool Class_Motor_DJ_Group::Is_Send_Due() const
{
    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (used[i]) return motors[i].Is_Send_Due();
    }
    return false;
}

/**
 * @brief 设置整组发送频率(转发给每台)
 * @return void
 */
void Class_Motor_DJ_Group::Set_Control_Freq(const Enum_Motor_Control_Freq &__freq)
{
    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (used[i]) motors[i].Set_Control_Freq(__freq);
    }
}

/**
 * @brief 整组使能(不额外发帧;下一拍正常 Send() 就会带上使能位对应的电流)
 * @return void
 */
void Class_Motor_DJ_Group::Enable()
{
    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (used[i]) motors[i].enable();
    }
}

/**
 * @brief 整组失能:每台把自己的槽位写 0,然后**只发一帧**
 *        (若逐台调 lose() 会发 4 帧 0 电流,所以这里自己组织)
 * @return void
 */
void Class_Motor_DJ_Group::Lose()
{
    if (unit_count == 0u || FDcan == nullptr) return;

    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (!used[i]) continue;

        motors[i].out = 0.0f;
        motors[i].PID_Angle.Clear();
        motors[i].PID_speed.Clear();
        motors[i].send_data = Struct_send_motor{};            //清目标 ⇒ 下次使能不冲
        motors[i].Send_Status_Cmd = Motor_Status_DISABLE;
        motors[i].Motor_Status    = Motor_Status_DISABLE;   //大疆无模式回读 → 镜像命令值
        motors[i].can_send_Data();                          //把 0 写进本组帧缓冲
    }

    //一帧发出去(0 电流)
    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (used[i])
        {
            if (!motors[i].send_group_frame()) motors[i].Tx_Fail++;
            break;
        }
    }
}

/**
 * @brief 整组标零(不涉及发送)
 * @return void
 */
void Class_Motor_DJ_Group::Zero()
{
    for (uint8_t i = 0; i < MOTOR_DJ_UNIT_MAX; i++)
    {
        if (used[i]) motors[i].zero();
    }
}

/**
 * @brief 本组这一帧的 8 个字节(诊断用:看槽位填得对不对)
 * @return 指向全局帧缓冲里本组那一帧
 */
const uint8_t *Class_Motor_DJ_Group::Get_Tx_Frame() const
{
    if (group_index > 2u) return nullptr;

    //与 can_send_Data() 用的是同一份缓冲(文件级静态,按 [总线][组])
    uint8_t bus = 0;
    if (FDcan != nullptr)
    {
        if (FDcan->Instance == FDCAN2)      bus = 1;
        else if (FDcan->Instance == FDCAN3) bus = 2;
    }
    return CAN_Tx_Data[bus][group_index];
}
