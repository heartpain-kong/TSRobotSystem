//
// Created by 20159 on 2026/3/13.
//

#include "motor_DM.h"
#include "math_support.h"
#include "bsp_dwt.h"

/**
 * @brief 达妙电机的初始化(统一入口)
 * @param _motor_init 初始化参数,model 填 motor_DM_Model,control_mode 填 motor_Control_mode
 * @return void
 */
void Class_Motor_DM:: Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,motor_DM_Model __model,
    const motor_Control_mode &__control_mode , const Enum_Motor_Control_Freq &__freq)
{
    motor_init.FDcan = hfdcan;
    motor_init.communication = Enum_FD_CAN;
    motor_init.id = __id;
    motor_init.master_id  = motor_DM_master_id ;
    motor_init.control_mode = __control_mode;
    motor_init.Control_Freq = __freq;

    /* ★ 频率控制生效的地方:Set_Control_Freq 算出 Control_Period_Us(= 1000000/频率),
       并把 send_time 回拨一个周期 —— 于是第一帧立即到期,之后 App 用
           if (motor_dm[i].Is_Send_Due()) motor_dm[i].send();
       按周期发。只写 motor_init.Control_Freq 是不够的,那只是留了一份记录。 */
    Set_Control_Freq(__freq);

    model=__model;
    //型号默认限幅(结构体未指定时使用)
    Apply_Model_Limit(model);

    Motor_Status = Motor_Status_DISABLE;
    send_data = Struct_send_motor{};
    recv_data = Struct_recv_motor{};
}

/**
 * @brief 按型号补齐 MIT 协议限幅
 *        注意:这里的 PMAX/VMAX/TMAX 必须与达妙上位机里设置的参数一致,
 *        否则组帧换算出来的位置/速度/力矩会整体偏比例。
 *        只有在结构体中未给定限幅(位置限幅为 0)时才套用默认值。
 * @param __model 电机型号
 * @return void
 */
void Class_Motor_DM::Apply_Model_Limit(const motor_DM_Model &__model)
{
    if (motor_init.Pos_max != 0.0f || motor_init.Pos_min != 0.0f) {
        return;
    }

    switch (__model)
    {
        case MOTOR_DM_J10010L:
            motor_init.Pos_max =  12.5f;  motor_init.Pos_min = -12.5f;
            motor_init.W_max   =  25.0f;  motor_init.W_min   = -25.0f;
            motor_init.T_max   = 200.0f;  motor_init.T_min   = -200.0f;
        break;

        case MOTOR_DM_J4310:
            motor_init.Pos_max =  12.5f;  motor_init.Pos_min = -12.5f;
            motor_init.W_max   =  30.0f;  motor_init.W_min   = -30.0f;
            motor_init.T_max   =  10.0f;  motor_init.T_min   = -10.0f;
        break;

        case MOTOR_DM_G6220:
            motor_init.Pos_max =  12.5f;  motor_init.Pos_min = -12.5f;
            motor_init.W_max   =  45.0f;  motor_init.W_min   = -45.0f;
            motor_init.T_max   =  10.0f;  motor_init.T_min   = -10.0f;
        break;

        default:
            motor_init.Pos_max =  12.5f;  motor_init.Pos_min = -12.5f;
            motor_init.W_max   =  30.0f;  motor_init.W_min   = -30.0f;
            motor_init.T_max   =  10.0f;  motor_init.T_min   = -10.0f;
        break;
    }

    motor_init.Kp_max = 500.0f;  motor_init.Kp_min = 0.0f;
    motor_init.Kd_max =   5.0f;  motor_init.Kd_min = 0.0f;
}

/**
 * @brief 按通信方式发送一帧数据
 * @param _data 数据指针
 * @param _len  数据长度
 * @param _id   帧 id
 * @return void
 */
void Class_Motor_DM::Transmit( uint32_t _id, uint8_t *_data, uint8_t _len)
{
    if (_data == nullptr) return;

    if (motor_init.communication == Enum_FD_CAN) {
        if (motor_init.FDcan!= nullptr)
            fdcan_send_data_stand(motor_init.FDcan, _id, _data, _len);
    }
    else {
        if (motor_init.UART != nullptr)
            UART_Transmit_Data(motor_init.UART, _data, _len);
    }
}

/**
 * @brief MIT 协议组帧:位置(16) 速度(12) Kp(12) Kd(12) 力矩(12)
 * @param _data 8 字节发送缓冲
 * @return void
 */
void Class_Motor_DM::MIT_Pack(uint8_t *_data) const {
    fp32 pos = motor_max_min(send_data.Pos, motor_init.Pos_max, motor_init.Pos_min);
    fp32 vel = motor_max_min(send_data.W,   motor_init.W_max,   motor_init.W_min);
    fp32 kp  = motor_max_min(send_data.Kp,  motor_init.Kp_max,  motor_init.Kp_min);
    fp32 kd  = motor_max_min(send_data.Kd,  motor_init.Kd_max,  motor_init.Kd_min);
    fp32 tor = motor_max_min(send_data.T,   motor_init.T_max,   motor_init.T_min);

    uint16_t pos_u = float_to_uint_f(pos, motor_init.Pos_min, motor_init.Pos_max, 16);
    uint16_t vel_u = float_to_uint_f(vel, motor_init.W_min,   motor_init.W_max,   12);
    uint16_t kp_u  = float_to_uint_f(kp,  motor_init.Kp_min,  motor_init.Kp_max,  12);
    uint16_t kd_u  = float_to_uint_f(kd,  motor_init.Kd_min,  motor_init.Kd_max,  12);
    uint16_t tor_u = float_to_uint_f(tor, motor_init.T_min,   motor_init.T_max,   12);

    _data[0] = (uint8_t)(pos_u >> 8);
    _data[1] = (uint8_t)(pos_u);
    _data[2] = (uint8_t)(vel_u >> 4);
    _data[3] = (uint8_t)(((vel_u & 0x0F) << 4) | (kp_u >> 8));
    _data[4] = (uint8_t)(kp_u);
    _data[5] = (uint8_t)(kd_u >> 4);
    _data[6] = (uint8_t)(((kd_u & 0x0F) << 4) | (tor_u >> 8));
    _data[7] = (uint8_t)(tor_u);
}

/**
 * @brief MIT 协议解帧:位置(16) 速度(12) 力矩(12) + MOS/线圈温度
 *        注意反馈帧的排布与控制帧不同(力矩紧跟在速度后面)
 * @param _data 8 字节反馈数据
 * @return void
 */
void Class_Motor_DM::MIT_Parse(const uint8_t *_data) {
    static uint16_t Pos,W,T;
    recv_data.Temperature  = _data[7]>_data[6] ? (fp32)_data[7] : (fp32)_data[6];
    recv_data.MError  = _data[0]>>4;

    Pos= ((uint16_t)(_data[1]<<8)|(uint16_t)(_data[2]));
    W = ((uint16_t)(_data[3]<<4)|(uint16_t)(_data[4]>>4));
    T = ((uint16_t)((_data[4]&0xF)<<8)|(uint16_t)_data[5]);

    recv_data.Now_Pos =uint_to_float(Pos, motor_init.Pos_min, motor_init.Pos_max,16);
    recv_data.Now_W  = uint_to_float(W, motor_init.W_min, motor_init.W_max,12);
    //⚠️ 这里曾经写成 (T, T_max, T_min, 12) —— 与上面两处、以及原型
    //   uint_to_float(x_int, x_min, x_max, bits) 的顺序相反。后果不只是量程不同:
    //   span 变成负数、offset 取成 T_max ⇒ **整个力矩反馈符号反过来**,
    //   任何基于 Now_T 的控制/观测都会反向。顺序必须是 (min, max)。
    recv_data.Now_T  = uint_to_float(T, motor_init.T_min, motor_init.T_max,12);

}

/**
 * @brief 发送一帧 MIT 控制量
 *        不做"该不该发"的门控 —— 那个由 App 用 Is_Send_Due() 判断(状态在电机里,判断在 App 里);
 *        这里只管发,发完 Mark_Sent() 把时刻记进 send_time。
 *
 * ★ 发送前会检查"是否已使能",没使能就**补一次使能**(前提:没有故障):
 *     · 达妙不发使能帧就不输出力矩,而且它的**通信看门狗**会在停发一段时间后
 *       自己失能(ERR=0xD)—— 只靠初始化时 enable 一次,掉线后永远回不来;
 *     · 所以每次要发之前自检:未使能 且 无故障 ⇒ enable() 后再发;
 *       有故障 ⇒ **不使能也不发**(带故障强行使能可能更危险,
 *       由上层决定:Clear_Error() 清错后再让它自动恢复)。
 *     · ERR=0xD(通信丢失)是"我们停发"造成的,属于可自愈的那一类,允许自动重使能。
 * @return void
 */
void Class_Motor_DM::send() {
    /* ---- ① 命令位:上层到底要不要它转 ----
       lose()(急停/手动失能)会把这里置 DISABLE ⇒ 此时**不补使能也不发控制帧**,
       绝不能"自愈"回使能状态(否则急停失效)。 */
    if (Send_Status_Cmd != Motor_Status_ENABLE)
    {
        return;
    }

    /* ---- ② 发送前检查"实际是否已使能"(看反馈里的 ERR)----
       命令位是"要它转",但反馈说没使能 —— 典型是**通信看门狗超时(ERR=0xD)自己掉了**
       (停发一会儿就会这样)。此时:
         · 没有真故障 ⇒ 补一次使能帧(0xFC),让它接着受控;
         · 有真故障   ⇒ 不硬使能、也不发,等上层 Clear_Error() + enable()。 */
    if (!Is_Enabled())
    {
        if (Has_Fault())
        {
            return;
        }
        enable();     //补 0xFC(enable() 内部也会把命令位再置 ENABLE)
    }

    /* ---- ③ 发这一拍的控制量 ---- */
    uint8_t data[8];

    MIT_Pack(data);
    Transmit(motor_init.id, data,  8);

    Mark_Sent();      //记录本拍时刻,供 App 的 Is_Send_Due() 判断
}

/**
 * @brief 解析一帧 MIT 反馈
 * @param _id   收到帧的 id(达妙反馈帧用的是 Master ID)
 * @param _data 数据指针
 * @param _len  数据长度
 * @return void
 */
void Class_Motor_DM::recv(uint32_t _id, const uint8_t *_data, uint8_t _len) {
    //达妙反馈帧的 CAN id 是**共用的 Master ID**,没有身份信息,所以不校验它
    (void)_id;

    if (_data == nullptr || _len < 8) return;

#if (MOTOR_DM_CHECK_FEEDBACK_ID != 0)
    /* 真正的电机身份在**数据里**:data[0] 的低 4 位 = 电机自己的 id(高 4 位是 ERR)。
       同一条总线上挂多台 DM 时,不校验就会把别人的反馈写进我的 recv_data。
       见 motor_DM.h 里 MOTOR_DM_CHECK_FEEDBACK_ID 的说明。 */
    if ((uint16_t)(_data[0] & 0x0F) != motor_init.id) return;
#endif

    MIT_Parse(_data);

    /* ★ 实际状态回读(你自己的写法,保留):达妙反馈帧里没有"模式"字段,
       这里拿 _data[0]>>4(=ERR/状态码)当状态用 —— ⚠️ 这只是**约定**,
       若实机上该值不是 1,Motor_Status 会一直显示 DISABLE(已不影响发帧,
       闸门改用 Send_Status_Cmd 了)。真正"电机是否在 FOC"最可靠的判据是:
       它有没有在回反馈 + 力矩是否响应;故障恢复用 Clear_Error() + enable()。 */
    Motor_Status = recv_data.MError == 1 ? Motor_Status_ENABLE : Motor_Status_DISABLE ;
}

/**
 * @brief 设置机械零点
 * @return void
 */
void Class_Motor_DM::zero() {
    Transmit(motor_init.id , motor_send_Data_zero, 8);
}

/**
 * @brief 电机使能
 * @return void
 */
void Class_Motor_DM::enable() {
    Send_Status_Cmd = Motor_Status_ENABLE;   //命令位:允许 send() 开始发运控帧
    Transmit(motor_init.id ,motor_send_Data_enable, 8);
}

/**
 * @brief 电机失能
 * @return void
 */
void Class_Motor_DM::lose() {
    Send_Status_Cmd = Motor_Status_DISABLE;  //命令位:send() 从此不再发运控帧
    Transmit(motor_init.id , motor_send_Data_lose, 8);

    //失能后清空设定值,避免下次使能瞬间冲出
    send_data = Struct_send_motor{};
}

/**
 * @brief 清除电机错误(FF..FB)
 *        故障源消失后电机**不会自恢复**,要清错 + 重新 enable() 才能继续受控
 *        (ERR:0x8 过压 / 0x9 欠压 / 0xA 过流 / 0xB MOS 过温 / 0xC 线圈过温 /
 *              0xD 通信丢失 / 0xE 过载)。详见 motor_DM.h 里的说明。
 * @return void
 */
void Class_Motor_DM::Clear_Error() {
    //清错帧属于"命令帧",和 enable/lose/zero 一样直接发,不受发送周期闸门约束
    Transmit(motor_init.id, motor_send_Data_clear_error, 8);
}
