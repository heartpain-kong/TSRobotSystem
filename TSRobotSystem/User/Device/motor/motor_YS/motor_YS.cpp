//
// Created by 20159 on 2026/3/13.
//
// Class_Motor_YS 实现:一个类两种协议,按 model 分派
//   协议 A(8010)  :依据《GO-M8010-6 Motor 使用手册 V1.0》表1/表2
//   协议 B(IM6014):依据官方 N6014B_HALDemo/App/protocol.c
//                   modify_data()  → pack_send_6014()
//                   extract_data() → parse_recv_6014()
// 各字段位置/量化系数/量程/饱和值全部照抄,详见 motor_YS.h 头部的两张对照表。
//

#include "motor_YS.h"
#include "crc_ccitt.h"
#include "crc32.h"
#include "math_support.h"
#include <cstring>

//收发缓冲按最长协议开(发 max(17,20)=20,收 max(16,26)=26),放 AXI_SRAM 因为 DMA
__attribute__((section (".AXI_SRAM"))) static uint8_t motor_YS_send_buf[MOTOR_6014_SEND_LEN];
//接收拷贝缓冲:IDLE 回调期间 DMA 仍指向它正在写的那个缓冲,必须先拷出来再解析(见 recv())
__attribute__((section (".AXI_SRAM"))) static uint8_t motor_YS_recv_buf[MOTOR_6014_RECV_LEN];

/* 小端读写小工具(两代协议的多字节字段都是小端) */
static inline int16_t  rd_i16(const uint8_t *p) { return (int16_t)((uint16_t)p[1] << 8 | p[0]); }
static inline int32_t  rd_i32(const uint8_t *p) {
    return (int32_t)(((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[1] << 8)  | ((uint32_t)p[0]));
}
static inline uint32_t rd_u32(const uint8_t *p) {
    return (((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) |
            ((uint32_t)p[1] << 8)  | ((uint32_t)p[0]));
}
static inline uint16_t rd_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[1] << 8 | p[0]); }

/* 有符号 16 位饱和
   ⚠️ 两代协议的量程上限都**正好是 raw = 32767 的边界**(6014 官方 SATURATE 的
   τ/ω/Kp/Kd 四个上限算出来就是 32767.002 这种值,余量不到 1 LSB)。
   浮点舍入只要往上偏一点点,(int16_t)32768 就会翻成 −32768 —— 指令极性反转。
   所有写进 2 字节字段的量都过一遍这里,把这类边界事故彻底堵死。 */
static inline int16_t sat_i16(int32_t v)
{
    if (v > 32767)  return (int16_t)32767;
    if (v < -32768) return (int16_t)(-32768);
    return (int16_t)v;
}

/**
 * @brief 初始化(按型号选协议)
 * @param huart 串口编号(485)⚠️ 波特率 4 Mbps
 * @param __id  电机ID(0~14;15 = 广播)
 * @param __model 型号:8010 或 IM6014
 * @param __control_mode 控制模式
 * @param __timeout_enable 仅 6014 有意义:是否开启 1 s 超时保护
 * @return void
 */
void Class_Motor_YS::Init(UART_HandleTypeDef *huart,uint16_t __id,const motor_YS_Model &__model,
        const motor_Control_mode &__control_mode, const Enum_Motor_Control_Freq &__freq,
        const bool __timeout_enable)
{
    motor_init.UART = huart;
    motor_init.communication = Enum_UART;
    motor_init.id = __id;
    motor_init.control_mode = __control_mode;
    motor_init.Control_Freq = __freq;
    model = __model;

    //型号:减速比 + 帧长 + 校验长度 + 反馈包头 + 量程
    Apply_Model_Limit(model);

    /* ★ 发送频率:Set_Control_Freq 算出 Control_Period_Us(= 1000000/频率)
       并把首帧置为立即到期;之后 send() 里 Mark_Sent() 记时刻,
       App 用 Is_Send_Due() 判断该不该发(485 半双工:一问一答,别抢跑) */
    Set_Control_Freq(__freq);

    Motor_Status = Motor_Status_DISABLE;      //实际状态:上电默认"锁定"(协议默认值)
    Send_Status_Cmd = Motor_Status_DISABLE;   //命令状态:我们也先不发
    Motor_Control_mode = __control_mode;
    send_data = Struct_send_motor{};
    recv_data = Struct_recv_motor{};

    init_Pos = 0.0f;
    Timeout_Protect = __timeout_enable;

    Force_Raw = 0;
    Over_Temperature = false;
    Winding_Temperature = 0;
    Voltage = 0.0f;
    MError32 = 0;
    MWarn = 0;
    Motor_Timeout = false;
    Recv_Crc_Error_Count = 0;
}

/**
 * @brief 按型号填协议参数与量程
 *
 * 量程来源:
 *   8010  :手册表1(★是**转子侧**的值)按对外单位约定折算
 *            τ 127.99 N·m(转子)→ 关节 = ×Gearbox;ω 804 rad/s(转子)→ 关节 = ÷Gearbox
 *            θ 表1 无关节限位(多圈累加 int32),只按"编码不溢出"反推
 *   6014  :官方 SATURATE(★本来就是**输出端**的值),直接填
 *
 * @param __model 电机型号
 * @return void
 */
void Class_Motor_YS::Apply_Model_Limit(const motor_YS_Model &__model)
{
    if (motor_init.Pos_max != 0.0f || motor_init.Pos_min != 0.0f) return;

    switch (__model)
    {
        case MOTOR_YS_IM6014:
            /*
             * 协议 B:IM6014(J288/S288) 20 B 发 / 26 B 收,CRC32,减速比 38/3
             */
            Gearbox_Rate = MOTOR_6014_RATIO;
            Send_Len     = MOTOR_6014_SEND_LEN;
            Recv_Len     = MOTOR_6014_RECV_LEN;
            Crc_Send_Len = MOTOR_6014_CRC_SEND_LEN;
            Crc_Recv_Len = MOTOR_6014_CRC_RECV_LEN;
            Recv_Head_0  = MOTOR_6014_RECV_HEAD_0;

            //官方 SATURATE 的上下限(输出端,非对称)
            motor_init.T_max   =  MOTOR_6014_TOR_MAX;   motor_init.T_min   = MOTOR_6014_TOR_MIN;
            motor_init.W_max   =  MOTOR_6014_SPD_MAX;   motor_init.W_min   = MOTOR_6014_SPD_MIN;
            motor_init.Pos_max =  MOTOR_6014_POS_MAX;   motor_init.Pos_min = MOTOR_6014_POS_MIN;
            motor_init.Kp_max  =  MOTOR_6014_KP_MAX;    motor_init.Kp_min  = 0.0f;
            motor_init.Kd_max  =  MOTOR_6014_KD_MAX;    motor_init.Kd_min  = 0.0f;
        break;

        case MOTOR_YS_M8010:
        default:                      //未知型号按 M8010 处理,总比留一组未初始化的系数好
            /*
             * 协议 A:GO-M8010-6 17 B 发 / 16 B 收,CRC16,减速比 6.33
             */
            Gearbox_Rate = MOTOR_YS_GEARBOX_M8010;
            Send_Len     = MOTOR_YS_SEND_LEN;
            Recv_Len     = MOTOR_YS_RECV_LEN;
            Crc_Send_Len = MOTOR_YS_CRC_SEND_LEN;
            Crc_Recv_Len = MOTOR_YS_CRC_RECV_LEN;
            Recv_Head_0  = MOTOR_YS_RECV_HEAD_0;

            //力矩:转子侧量程 × Gearbox → 关节侧
            motor_init.T_max   =  MOTOR_YS_T_MAX_ROTOR * Tq_Raw2Out();
            motor_init.T_min   = -motor_init.T_max;

            //速度:转子侧量程 ÷ Gearbox → 关节侧
            motor_init.W_max   =  MOTOR_YS_W_MAX_ROTOR * Kin_Raw2Out();
            motor_init.W_min   = -motor_init.W_max;

            //位置:手册无关节限位,只按"int32 编码不溢出"反推
            //  pos_rotor = raw / P_SCALE(表1 的逆运算),再 ÷Gearbox 到关节侧;
            //  ⚠️ 这里**不能**再乘 π —— 以前 P_SCALE 少了 π,才用 *PI 去凑,
            //     现在 P_SCALE 已按手册修正(32768/(2π)),π 必须去掉。
            motor_init.Pos_max =  2147483647.0f / (MOTOR_YS_P_SCALE * Kin_Out2Raw());
            motor_init.Pos_min = -motor_init.Pos_max;

            //刚度/阻尼:手册 0 ≤ Kp,Kd ≤ 25.599,与减速比无关
            motor_init.Kp_max  =  MOTOR_YS_KP_MAX;   motor_init.Kp_min  = 0.0f;
            motor_init.Kd_max  =  MOTOR_YS_KD_MAX;   motor_init.Kd_min  = 0.0f;
        break;
    }
}

/**
 * @brief 按通信方式发一帧(走 485/UART)
 * @param *_data 数据指针
 * @param _len  数据长度
 * @return void
 */
void Class_Motor_YS::Transmit(uint8_t *_data, uint8_t _len)
{
    if (motor_init.UART == nullptr || _data == nullptr) return;

    UART_Transmit_Data(motor_init.UART, _data, _len);
}

/* ═════════════════ 协议 A:GO-M8010-6 ═════════════════ */

/**
 * @brief 组帧(17 字节,手册表1)
 *        先按"对外侧"物理量夹紧,再折算成转子侧、乘量化系数,最后算 CRC16。
 * @return void
 */
void Class_Motor_YS::pack_send_8010()
{
    fp32 t   = motor_max_min(send_data.T,              motor_init.T_max,   motor_init.T_min);
    fp32 w   = motor_max_min(send_data.W,              motor_init.W_max,   motor_init.W_min);
    fp32 pos = motor_max_min(send_data.Pos + init_Pos, motor_init.Pos_max, motor_init.Pos_min);
    fp32 kp  = motor_max_min(send_data.Kp,             motor_init.Kp_max,  motor_init.Kp_min);
    fp32 kd  = motor_max_min(send_data.Kd,             motor_init.Kd_max,  motor_init.Kd_min);

    //表1:τ_set = τ_ff*256 ; ω_set = ω_des/(2π)*256 ; θ_set = θ_des/(2π)*32768
    int16_t t_raw   = sat_i16((int32_t)(t   * Tq_Out2Raw()  * MOTOR_YS_T_SCALE));
    int16_t w_raw   = sat_i16((int32_t)(w   * Kin_Out2Raw() * MOTOR_YS_W_SCALE));
    int32_t pos_raw = (int32_t)(pos * Kin_Out2Raw() * MOTOR_YS_P_SCALE);
    int16_t kp_raw  = sat_i16((int32_t)(kp  * MOTOR_YS_KP_SCALE));
    int16_t kd_raw  = sat_i16((int32_t)(kd  * MOTOR_YS_KD_SCALE));

    //表1 STATUS:0=锁定(Default) 1=FOC 闭环 2=编码器校准(本类只用 0/1)
    uint8_t status = (Send_Status_Cmd == Motor_Status_ENABLE) ? MOTOR_YS_STATUS_FOC
                                                          : MOTOR_YS_STATUS_LOCK;

    motor_YS_send_buf[0] = MOTOR_YS_SEND_HEAD_0;
    motor_YS_send_buf[1] = MOTOR_YS_SEND_HEAD_1;
    //第 3 字节:低 4 位 ID,bits 20-22 STATUS,最高位保留
    motor_YS_send_buf[2] = (uint8_t)(((uint32_t)motor_init.id & MOTOR_YS_ID_MASK)
                          | (((uint32_t)status & MOTOR_YS_STATUS_MASK) << MOTOR_YS_STATUS_SHIFT));

    motor_YS_send_buf[3]  = (uint8_t)(t_raw & 0xFF);
    motor_YS_send_buf[4]  = (uint8_t)((t_raw >> 8) & 0xFF);
    motor_YS_send_buf[5]  = (uint8_t)(w_raw & 0xFF);
    motor_YS_send_buf[6]  = (uint8_t)((w_raw >> 8) & 0xFF);
    motor_YS_send_buf[7]  = (uint8_t)(pos_raw & 0xFF);
    motor_YS_send_buf[8]  = (uint8_t)((pos_raw >> 8) & 0xFF);
    motor_YS_send_buf[9]  = (uint8_t)((pos_raw >> 16) & 0xFF);
    motor_YS_send_buf[10] = (uint8_t)((pos_raw >> 24) & 0xFF);
    motor_YS_send_buf[11] = (uint8_t)(kp_raw & 0xFF);
    motor_YS_send_buf[12] = (uint8_t)((kp_raw >> 8) & 0xFF);
    motor_YS_send_buf[13] = (uint8_t)(kd_raw & 0xFF);
    motor_YS_send_buf[14] = (uint8_t)((kd_raw >> 8) & 0xFF);

    //表1 CRC16:CRC16_CCITT 对前 15 字节计算,小端写回
    uint16_t crc = crc_ccitt(motor_YS_send_buf, Crc_Send_Len);
    motor_YS_send_buf[15] = (uint8_t)(crc & 0xFF);
    motor_YS_send_buf[16] = (uint8_t)((crc >> 8) & 0xFF);
}

/**
 * @brief 解帧(16 字节,手册表2)
 * @param *_data 反馈数据指针
 * @return true = 有效,已写入 recv_data
 */
bool Class_Motor_YS::parse_recv_8010(const uint8_t *_data)
{
    //表2 包头是 0xFD 0xEE(原注:与来自主机方向的包头不同)
    if (_data[0] != Recv_Head_0 || _data[1] != MOTOR_YS_RECV_HEAD_1) return false;

#if (MOTOR_YS_CHECK_RECV_CRC)
    {
        //表2 CRC16:对前 14 字节计算,小端存放在第 15、16 字节
        uint16_t crc_calc = crc_ccitt(_data, Crc_Recv_Len);
        uint16_t crc_recv = rd_u16(&_data[14]);
        if (crc_calc != crc_recv) { Recv_Crc_Error_Count++; return false; }
    }
#endif

    //表2:τ = τ_fbk/256 ; ω = ω_fbk/256*2π ; θ = θ_fbk/32768*2π (都是转子侧)
    recv_data.Now_T   = (fp32)rd_i16(&_data[3]) / MOTOR_YS_T_SCALE * Tq_Raw2Out();
    recv_data.Now_W   = (fp32)rd_i16(&_data[5]) / MOTOR_YS_W_FBK_DIV * (2.0f * PI) * Kin_Raw2Out();
    recv_data.Now_Pos = (fp32)rd_i32(&_data[7]) / MOTOR_YS_P_FBK_DIV * (2.0f * PI) * Kin_Raw2Out();

    //表2 第 12 字节:温度 int8,℃;≥90℃ 电机自保护(需重新上电)
    recv_data.Temperature = (fp32)(int8_t)_data[11];
    Over_Temperature = (recv_data.Temperature >= MOTOR_YS_TEMP_PROTECT);

    //表2 bits 96-98 [3bit] MERROR
    recv_data.MError = (uint8_t)(_data[12] & 0x07u);

    //表2 bits 20-22 [3bit] STATUS(bit23 是保留位,不能一起取)
    recv_data.mode = (uint8_t)((_data[2] >> MOTOR_YS_STATUS_SHIFT) & MOTOR_YS_STATUS_MASK);

    //表2 bits 99-110 [12bit] FORCE = 足端力原始数据(0-4095)
    Force_Raw = (uint16_t)((((uint16_t)_data[13] & 0x7Fu) << 5)
                         | (((uint16_t)_data[12] >> 3) & 0x1Fu));
    //★ 实际状态回读:反馈 mode 的 STATUS 才是电机**真实**模式
    //   1 = FOC 闭环 → ENABLE;0 = 锁定、2 = 编码器校准 → DISABLE(原始 3bit 值在 recv_data.mode)
    Motor_Status = (recv_data.mode == MOTOR_YS_STATUS_FOC) ? Motor_Status_ENABLE
                                                          : Motor_Status_DISABLE;

    return true;
}

/* ═════════════════ 协议 B:IM6014(J288/S288)═════════════════ */

/**
 * @brief 组帧(20 字节,官方 ControlData_t)
 *        输出端浮点 → 夹紧 → 定点 → CRC32(前 16 字节,含包头)
 * @return void
 */
void Class_Motor_YS::pack_send_6014()
{
    //先按官方 SATURATE 夹紧(输出端单位)
    fp32 t   = motor_max_min(send_data.T,              motor_init.T_max,   motor_init.T_min);
    fp32 w   = motor_max_min(send_data.W,              motor_init.W_max,   motor_init.W_min);
    fp32 pos = motor_max_min(send_data.Pos + init_Pos, motor_init.Pos_max, motor_init.Pos_min);
    fp32 kp  = motor_max_min(send_data.Kp,             motor_init.Kp_max,  motor_init.Kp_min);
    fp32 kd  = motor_max_min(send_data.Kd,             motor_init.Kd_max,  motor_init.Kd_min);

    //官方换算(输出端 → 转子端 → 定点)
    int16_t tor_des = sat_i16((int32_t)(t   * Tq_Out2Raw()  * MOTOR_6014_TOR_SCALE));
    int16_t spd_des = sat_i16((int32_t)(w   * Kin_Out2Raw() * MOTOR_6014_SPD_SCALE / (2.0f * PI)));
    int32_t pos_des = (int32_t)(pos * Kin_Out2Raw() * MOTOR_6014_POS_SCALE / (2.0f * PI));
    int16_t k_pos   = sat_i16((int32_t)(kp  / (Gearbox_Rate * Gearbox_Rate) * MOTOR_6014_KP_SCALE));
    int16_t k_spd   = sat_i16((int32_t)(kd  / (Gearbox_Rate * Gearbox_Rate) * MOTOR_6014_KD_SCALE));

    uint8_t *b = motor_YS_send_buf;

    b[0] = MOTOR_6014_SEND_HEAD_0;
    b[1] = MOTOR_6014_SEND_HEAD_1;

    //mode 字节:id(4) | status(3)<<4 | timeout(1)<<7
    uint8_t status = (Send_Status_Cmd == Motor_Status_ENABLE) ? MOTOR_YS_STATUS_FOC
                                                          : MOTOR_YS_STATUS_LOCK;
    b[2] = (uint8_t)(((uint32_t)motor_init.id & MOTOR_YS_ID_MASK)
                   | (((uint32_t)status & MOTOR_YS_STATUS_MASK) << MOTOR_YS_STATUS_SHIFT)
                   | (Timeout_Protect ? MOTOR_6014_TIMEOUT_BIT : 0u));

    b[3] = 0u;                                   //res 保留

    b[4] = (uint8_t)(tor_des & 0xFF);            b[5]  = (uint8_t)((tor_des >> 8) & 0xFF);
    b[6] = (uint8_t)(spd_des & 0xFF);            b[7]  = (uint8_t)((spd_des >> 8) & 0xFF);
    b[8]  = (uint8_t)(pos_des & 0xFF);           b[9]  = (uint8_t)((pos_des >> 8) & 0xFF);
    b[10] = (uint8_t)((pos_des >> 16) & 0xFF);   b[11] = (uint8_t)((pos_des >> 24) & 0xFF);
    b[12] = (uint8_t)(k_pos & 0xFF);             b[13] = (uint8_t)((k_pos >> 8) & 0xFF);
    b[14] = (uint8_t)(k_spd & 0xFF);             b[15] = (uint8_t)((k_spd >> 8) & 0xFF);

    //CRC32:覆盖前 16 字节(含包头),小端写在 16..19
    uint32_t crc = crc32_mpeg2(b, Crc_Send_Len);
    b[16] = (uint8_t)(crc & 0xFF);         b[17] = (uint8_t)((crc >> 8) & 0xFF);
    b[18] = (uint8_t)((crc >> 16) & 0xFF); b[19] = (uint8_t)((crc >> 24) & 0xFF);
}

/**
 * @brief 解帧(26 字节,官方 extract_data)
 * @param *_data 反馈数据指针
 * @return true = 有效,已写入 recv_data
 */
bool Class_Motor_YS::parse_recv_6014(const uint8_t *_data)
{
    //包头 0xFC 0xEE(与 8010 的 0xFD 0xEE、主机方向的 0xFE 0xEE 都不同)
    if (_data[0] != Recv_Head_0 || _data[1] != MOTOR_6014_RECV_HEAD_1) return false;

#if (MOTOR_YS_CHECK_RECV_CRC)
    {
        //官方:CRC32 覆盖 Byte2..21 共 20 字节(不含包头),小端在 22..25
        uint32_t crc_calc = crc32_mpeg2(&_data[2], Crc_Recv_Len);
        uint32_t crc_recv = rd_u32(&_data[22]);
        if (crc_calc != crc_recv) { Recv_Crc_Error_Count++; return false; }
    }
#endif

    //官方:outputTor = torque/2560*RATIO ; outputSpd = speed/64*2π/RATIO
    //      outputPos = 2π*pos/32768/RATIO ; vol = vol_raw/2
    recv_data.Now_T   = (fp32)rd_i16(&_data[6])  / MOTOR_6014_TOR_SCALE * Tq_Raw2Out();
    recv_data.Now_W   = (fp32)rd_i16(&_data[8])  / MOTOR_6014_SPD_SCALE * (2.0f * PI) * Kin_Raw2Out();
    recv_data.Now_Pos = (2.0f * PI) * (fp32)rd_i32(&_data[10]) / MOTOR_6014_POS_SCALE * Kin_Raw2Out();

    recv_data.Temperature = (fp32)(int8_t)_data[3];     //壳体温度 ℃
    Winding_Temperature   = _data[4];                   //绕组温度 ℃
    Voltage               = (fp32)_data[5] / 2.0f;      //0.5 V 分辨率

    MError32         = rd_u32(&_data[14]);
    recv_data.MError = (uint8_t)(MError32 & 0xFFu);     //基类字段 8 位,放低 8 位;完整值用 Get_MError32()

    MWarn = (uint16_t)((rd_u16(&_data[18]) >> 13) & 0x07u);   //[Res:13 | ExFlag:3]

    recv_data.mode = (uint8_t)((_data[2] >> MOTOR_YS_STATUS_SHIFT) & MOTOR_YS_STATUS_MASK);
    Motor_Timeout  = (_data[2] & MOTOR_6014_TIMEOUT_BIT) != 0u;
    //★ 实际状态回读:反馈 mode 的 STATUS 才是电机**真实**模式
    //   1 = FOC 闭环 → ENABLE;0 = 锁定、2 = 编码器校准 → DISABLE(原始 3bit 值在 recv_data.mode)
    Motor_Status = (recv_data.mode == MOTOR_YS_STATUS_FOC) ? Motor_Status_ENABLE
                                                          : Motor_Status_DISABLE;

    return true;
}

/* ═════════════════ 公共:组帧/解帧分派、收发 ═════════════════ */

/**
 * @brief 组帧(按型号分派)
 * @return void
 */
void Class_Motor_YS::pack_send_frame()
{
    if (model == MOTOR_YS_IM6014) {
        pack_send_6014();
    } else {
        pack_send_8010();
    }
}

/**
 * @brief 解帧(按型号分派)
 * @param *_data 反馈数据指针
 * @return true = 有效
 */
bool Class_Motor_YS::unpack_recv_frame(const uint8_t *_data)
{
    if (model == MOTOR_YS_IM6014) {
        return parse_recv_6014(_data);
    }
    return parse_recv_8010(_data);
}

/**
 * @brief 发送一帧控制量
 *        不做门控 —— "该不该发"由 App 用 Is_Send_Due() 判断(状态在电机里,判断在 App 里);
 *        这里只管发,发完 Mark_Sent() 把时刻记进 send_time。
 * @return void
 */
void Class_Motor_YS::send() {
    //★ 闸门看**命令位**:宇树是"每收一帧命令回一帧反馈"(手册 8.2),使能帧也会
    //  引来一帧带 STATUS 的反馈,所以判 Motor_Status 通常也能通;但反馈丢帧/CRC 不过时
    //  会白丢一拍,读到的旧 STATUS 也可能让闸门误关。闸门只认命令,Motor_Status 只反映实际。
    if (Send_Status_Cmd != Motor_Status_ENABLE) return;

    /* ★ 发送前自检"是否在 FOC":命令位是"要它转",但反馈 STATUS 说它没在 FOC
       (上电默认锁定、掉线后自己停了),且**没有故障** ⇒ 补一帧使能命令
       (STATUS=FOC,就是 enable() 的动作)再发控制帧。
       ⚠️ 放在闸门之后:急停(lose())过之后命令位是 DISABLE,这里不会执行,
          绝不会把急停后的电机自动使能回来。 */
    if (!Is_Enabled())
    {
        if (Has_Fault()) return;
        enable();       /* 使能帧本身就带当前 T/W/Pos,不要再紧接着发第二帧:
                           485 半双工下第二帧会撞上第一帧的 DMA,直接失败 */
        Mark_Sent();
        return;
    }

    pack_send_frame();
    Transmit(motor_YS_send_buf, Send_Len);

    Mark_Sent();      //记录本拍时刻,供 App 的 Is_Send_Due() 判断
}

/**
 * @brief 解析一帧反馈
 *
 * ★ 接收侧也是 DMA(BSP 的 485 引擎 = DMA 双缓冲 DBM + IDLE 帧结束),
 *   而且 HAL_UARTEx_RxEventCallback 是**先回调、后手动翻转 CT** ——
 *   回调执行期间 DMA 仍指向它正在收的那个缓冲,下一帧字节一到就会覆盖当前帧。
 *   所以必须**先 memcpy 到自己的缓冲再解析**。
 *
 * 注:这里会把**反馈的 STATUS 回读**写进 Motor_Status(实际状态),但**不会**去动
 *     Send_Status_Cmd(命令状态)——否则 lose() 之后一收到旧反馈就又被"重新使能"了。
 *
 * @param _id   帧 id(485 无 id,忽略)
 * @param _data 数据指针(指向 DMA 双缓冲中的当前缓冲)
 * @param _len  数据长度(8010 应为 16,6014 应为 26)
 * @return void
 */
void Class_Motor_YS::recv(uint32_t _id, const uint8_t *_data, uint8_t _len) {
    (void)_id;

    if (_data == nullptr || _len < Recv_Len) return;

    memcpy(motor_YS_recv_buf, _data, Recv_Len);
    (void)unpack_recv_frame(motor_YS_recv_buf);
}

/**
 * @brief 电机使能
 *        没有独立的使能命令帧:使能位就是**每一帧** mode 字节的 STATUS,
 *        STATUS=1 = FOC 闭环(能响应控制量)。所以要实打实发一帧才生效。
 *
 * ★ 这里只改**命令位** Send_Status_Cmd,不动 Motor_Status ——
 *   Motor_Status 表示"电机实际处于什么模式",要等下一帧反馈的 STATUS 回读才知道。
 *   于是上层可以这样确认使能是否真的生效:
 *       motor.enable();
 *       ... 等一帧反馈 ...
 *       if (motor.Get_Motor_Status() == Motor_Status_ENABLE) { 真的进 FOC 了 }
 * @return void
 */
void Class_Motor_YS::enable(){
    Send_Status_Cmd = Motor_Status_ENABLE;
    pack_send_frame();
    Transmit(motor_YS_send_buf, Send_Len);
}

/**
 * @brief 电机失能(STATUS=0 = 锁定 Default)
 * @return void
 */
void Class_Motor_YS::lose(){
    Send_Status_Cmd = Motor_Status_DISABLE;
    pack_send_frame();
    Transmit(motor_YS_send_buf, Send_Len);

    //失能后清空设定值,避免下次使能瞬间冲出
    send_data = Struct_send_motor{};
}

/**
 * @brief 设置机械零点(软件办法)
 *        两代协议都没有"设置机械零点"命令(STATUS=2 是编码器校准,不是零点),
 *        只能把偏置加在**发送侧**:把此刻的机械位置记为 0 —— 之后 Set_Pos(0)
 *        就是当前位姿。反馈侧保持电机自己的绝对位置不动(便于上层做标定/对齐)。
 * @return void
 */
void Class_Motor_YS::zero(){
    init_Pos = -recv_data.Now_Pos;
}
