//
// Created by 20159 on 2026/3/13.
//

#include "motor_LZ.h"
#include "math_support.h"

/* ===== 命令帧载荷(类型 0x03/0x04/0x06/0x16/0x18)===== */

//使能(类型 0x03):数据区全 0
static uint8_t motor_send_LZ_Data_enable[8] = {00,00,00,00,00,00,00,00};
//失能/停止(类型 0x04):data[0]=1 表示顺带清除故障位
static uint8_t motor_send_LZ_Data_disable[8] = {0x01,00,00,00,00,00,00,00};
//设置机械零位(类型 0x06):data[0]=1
static uint8_t motor_send_LZ_Data_zero[8] = {0x01,00,00,00,00,00,00,00};
//主动上报(类型 0x18)
static uint8_t motor_send_LZ_Data_active_recv[8] = {01,02,03,04,05,06,00,00};
//电机数据保存(类型 0x16):官方规定数据区必须是 01..08
static uint8_t motor_send_LZ_Data_save[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

/**
 * @brief 灵足电机的初始化
 * @param hfdcan CAN编号
 * @param __id   电机 CAN_ID
 * @param __model 电机的型号
 * @param __control_mode 控制模式(motor_pos / motor_speed / motor_T)
 * @return void
 */
void Class_Motor_LZ::Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,const motor_LZ_Model &__model,
        const motor_Control_mode &__control_mode, const Enum_Motor_Control_Freq &__freq)
{
    motor_init.FDcan = hfdcan;
    motor_init.communication = Enum_FD_CAN;
    motor_init.id = __id;
    motor_init.master_id = motor_LZ_user_id;
    motor_init.control_mode = __control_mode;
    motor_init.Control_Freq = __freq;
    model = __model;

    //型号默认量程(结构体未指定时使用)
    Apply_Model_Limit(model);

    /* ★ 发送频率:Set_Control_Freq 算出 Control_Period_Us(= 1000000/频率)
       并把首帧置为立即到期;之后 send() 里 Mark_Sent() 记时刻,
       App 用 Is_Send_Due() 判断该不该发 */
    Set_Control_Freq(__freq);

    Motor_Status = Motor_Status_DISABLE;
    send_data = Struct_send_motor{};
    recv_data = Struct_recv_motor{};
    mode = CANCOM_ANNOUNCE_DEVID;
    RunMode = 0u;
    RunModeValid = 0u;
    LastRxTick = 0u;
    zero_result = 0u;
}

/**
 * @brief 按型号补齐 16bit 编码量程
 *        ⚠️ 这里的 PMAX/VMAX/TMAX 必须与上位机里设置的参数一致,
 *           否则组帧换算出来的位置/速度/力矩会整体偏比例。
 *           只有在结构体中未给定(位置量程为 0)时才套用默认值。
 * @param __model 电机型号
 * @return void
 */
void Class_Motor_LZ::Apply_Model_Limit(const motor_LZ_Model &__model)
{
    if (motor_init.Pos_max != 0.0f || motor_init.Pos_min != 0.0f) return;

    /* 数值来源:官方 Python 驱动 DM_RS-main/constants.py 的三张表
     *   MODEL_MIT_POSITION_TABLE / VELOCITY_TABLE / TORQUE_TABLE / KP_TABLE / KD_TABLE
     *   · 位置:全部型号都是 ±4π
     *   · 速度/力矩:Kp/Kd 上限**按型号不同**(小电机 500/5,大电机 5000/100)
     * 型号名对应:我们的 MOTOR_LZ_0N ↔ 官方的 rs-0N
     */
    switch (__model)
    {
        case MOTOR_LZ_00:                       //rs-00
            motor_init.Pos_max =  MOTOR_LZ_POS_MAX_DEFAULT;  motor_init.Pos_min = -MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.W_max   =  50.0f;   motor_init.W_min = -50.0f;
            motor_init.T_max   =  17.0f;   motor_init.T_min = -17.0f;
            motor_init.Kp_max  =  500.0f;  motor_init.Kd_max = 5.0f;
        break;

        case MOTOR_LZ_01:                       //rs-01
            motor_init.Pos_max =  MOTOR_LZ_POS_MAX_DEFAULT;  motor_init.Pos_min = -MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.W_max   =  44.0f;   motor_init.W_min = -44.0f;
            motor_init.T_max   =  17.0f;   motor_init.T_min = -17.0f;
            motor_init.Kp_max  =  500.0f;  motor_init.Kd_max = 5.0f;
        break;

        case MOTOR_LZ_02:                       //rs-02
            motor_init.Pos_max =  MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.Pos_min = -MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.W_max   =  MOTOR_LZ_W_MAX_DEFAULT;
            motor_init.W_min   = -MOTOR_LZ_W_MAX_DEFAULT;
            motor_init.T_max   =  MOTOR_LZ_T_MAX_DEFAULT;
            motor_init.T_min   = -MOTOR_LZ_T_MAX_DEFAULT;
            motor_init.Kp_max  =  MOTOR_LZ_KP_MAX;  motor_init.Kd_max = MOTOR_LZ_KD_MAX;
        break;

        case MOTOR_LZ_03:                       //rs-03
            motor_init.Pos_max =  MOTOR_LZ_POS_MAX_DEFAULT;  motor_init.Pos_min = -MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.W_max   =  50.0f;   motor_init.W_min = -50.0f;
            motor_init.T_max   =  60.0f;   motor_init.T_min = -60.0f;
            motor_init.Kp_max  = 5000.0f;  motor_init.Kd_max = 100.0f;
        break;

        case MOTOR_LZ_04:                       //rs-04
            motor_init.Pos_max =  MOTOR_LZ_POS_MAX_DEFAULT;  motor_init.Pos_min = -MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.W_max   =  15.0f;   motor_init.W_min = -15.0f;
            motor_init.T_max   = 120.0f;   motor_init.T_min = -120.0f;
            motor_init.Kp_max  = 5000.0f;  motor_init.Kd_max = 100.0f;
        break;

        case MOTOR_LZ_05:                       //rs-05
            //⚠️ 这里原来写的是 vel 50 / T 5.5,与官方表(33 / 17)不符,已按官方修正
            motor_init.Pos_max =  MOTOR_LZ_POS_MAX_DEFAULT;  motor_init.Pos_min = -MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.W_max   =  33.0f;   motor_init.W_min = -33.0f;
            motor_init.T_max   =  17.0f;   motor_init.T_min = -17.0f;
            motor_init.Kp_max  =  500.0f;  motor_init.Kd_max = 5.0f;
        break;

        case MOTOR_LZ_06:                       //rs-06
            motor_init.Pos_max =  MOTOR_LZ_POS_MAX_DEFAULT;  motor_init.Pos_min = -MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.W_max   =  20.0f;   motor_init.W_min = -20.0f;
            motor_init.T_max   =  60.0f;   motor_init.T_min = -60.0f;
            motor_init.Kp_max  = 5000.0f;  motor_init.Kd_max = 100.0f;
        break;

        //型号不明时按 RS-02 处理 —— 总比留一组未初始化的量程去编码 CAN 帧好
        //  ⚠️ 这里曾经写成 0.0 / -0.0:量程夹紧会把所有速度/力矩指令夹成 0,电机根本不会动。
        default:
            //用头文件里的 *_DEFAULT 宏,和上面 MOTOR_LZ_02 保持同一个来源
            motor_init.Pos_max =  MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.Pos_min = -MOTOR_LZ_POS_MAX_DEFAULT;
            motor_init.W_max   =  MOTOR_LZ_W_MAX_DEFAULT;
            motor_init.W_min   = -MOTOR_LZ_W_MAX_DEFAULT;
            motor_init.T_max   =  MOTOR_LZ_T_MAX_DEFAULT;
            motor_init.T_min   = -MOTOR_LZ_T_MAX_DEFAULT;
            motor_init.Kp_max  =  MOTOR_LZ_KP_MAX;  motor_init.Kd_max = MOTOR_LZ_KD_MAX;
        break;
    }

    //刚度/阻尼下限两种型号都是 0
    motor_init.Kp_min = 0.0f;
    motor_init.Kd_min = 0.0f;
}

/**
 * @brief 组装扩展帧 id:| 类型(5) | 数据段(16) | 电机id(8) |
 *        运控帧的数据段是力矩,命令帧的数据段是主机 CAN ID
 * @param Id_data 数据段(16bit)
 * @return 扩展帧 id
 */
uint32_t Class_Motor_LZ::make_frame_id(uint32_t Id_data) const {
    return (((uint32_t)mode) << 24) | ((Id_data & 0xFFFFu) << 8) | (uint32_t)(motor_init.id & 0xFFu);
}

/**
 * @brief 按通信方式发一帧(不等 FIFO)
 * @param _id   扩展帧 id
 * @param *_data 数据指针
 * @param _len  数据长度
 * @return void
 */
void Class_Motor_LZ::Transmit(uint32_t _id, uint8_t *_data, uint8_t _len)
{
    if (_data == nullptr) return;

    if (motor_init.FDcan != nullptr)
        fdcan_send_data_Exten(motor_init.FDcan, _id, _data, _len);
}

/**
 * @brief 按通信方式发一帧,并等 TX FIFO 有空位
 * @param _id   扩展帧 id
 * @param *_data 数据指针
 * @param _len  数据长度
 * @param _timeout_ms 等空位的超时 [ms]
 * @return 0 = 已进 TX FIFO;1 = 超时(这一帧没发出去,调用方必须处理)
 */
uint8_t Class_Motor_LZ::Transmit_Wait(uint32_t _id, uint8_t *_data, uint8_t _len, uint32_t _timeout_ms)
{
    if (motor_init.FDcan == nullptr || _data == nullptr) return 1u;

    return fdcan_send_data_Exten_wait(motor_init.FDcan, _id, _data, _len, _timeout_ms);
}

/**
 * @brief 灵足电机使能(官方类型 0x03)
 * @return void
 */
void Class_Motor_LZ::enable(){
    mode = CANCOM_MOTOR_IN;
    Send_Status_Cmd = Motor_Status_ENABLE;   //命令位:允许发运控帧(实际状态等反馈回读)
    Transmit(make_frame_id(motor_init.master_id), motor_send_LZ_Data_enable, 8);
}

/**
 * @brief 灵足电机失能/停止(官方类型 0x04,data[0]=1 顺带清故障)
 * @return void
 */
void Class_Motor_LZ::lose(){
    mode = CANCOM_MOTOR_RESET;
    Send_Status_Cmd = Motor_Status_DISABLE;  //命令位:停止发运控帧
    Transmit(make_frame_id(motor_init.master_id), motor_send_LZ_Data_disable, 8);
}

/**
 * @brief 主动上报开关(官方类型 0x18)
 * @param F_CMD 1 为开启 0 为关闭
 * @return void
 */
void Class_Motor_LZ::active_recv(uint8_t F_CMD){
    mode = CANCOM_MODE_ACTIVE_RECV;
    motor_send_LZ_Data_active_recv[6] = (F_CMD == 1);
    Transmit(make_frame_id(motor_init.master_id), motor_send_LZ_Data_active_recv, 8);
}

/**
 * @brief 设置机械零位(官方类型 0x06,data[0]=1)
 * @return void,结果见 Get_Zero_Result()
 *
 * ① 用"等 TX FIFO 空位"的发送:FIFO 只有 8 槽,一条总线上多台电机连续置零时,
 *    不等空位会把后面的帧静默丢掉(真机现象:"每条总线只生效前 3 个电机")。
 *    结果不再被吞 —— 存进 zero_result,由 Get_Zero_Result() 读。
 * ② 结尾发的是 lose() 而**不是 enable()**:类型 0x06 只允许在未使能时执行。
 * ③ 可选补发类型 0x16(保存)让零点断电保留,见 MOTOR_LZ_ZERO_SEND_SAVE。
 */
void Class_Motor_LZ::zero(){
    zero_result = 0u;

    lose();                                   //置零只允许在未使能时执行
    mode = CANCOM_MOTOR_ZERO;
    zero_result |= Transmit_Wait(make_frame_id(motor_init.master_id), motor_send_LZ_Data_zero, 8,
                                 MOTOR_LZ_TX_WAIT_TIMEOUT_MS);
#if MOTOR_LZ_ZERO_SEND_SAVE
    mode = CANCOM_MODE_TOTAL;                 //类型 22 = 0x16 = 数据保存(不是 24!)
    zero_result |= Transmit_Wait(make_frame_id(motor_init.master_id), motor_send_LZ_Data_save, 8,
                                 MOTOR_LZ_TX_WAIT_TIMEOUT_MS);
#endif
    lose();                                   //结束时保证电机是失能的(不再 enable)
}

/**
 * @brief 读一个参数(官方类型 0x11)
 *        data = [index_lo, index_hi, 0,0,0,0,0,0]
 *        应答是同样类型的帧:data[0..1] = 参数地址(小端),值在 data[4..7]
 * @param index 参数地址,例如 0x7005(运行模式)
 * @return void
 */
void Class_Motor_LZ::read_param(uint16_t index){
    uint8_t txdata[8] = {0};

    mode = CANCOM_PARA_READ;
    txdata[0] = (uint8_t)(index & 0xFF);
    txdata[1] = (uint8_t)(index >> 8);
    Transmit(make_frame_id(motor_init.master_id), txdata, 8);
}

/**
 * @brief 通用写参数(官方类型 0x12)
 *        data = [idx_lo, idx_hi, 位宽, 0, 值(小端,最多 4 字节)]
 *        ⚠️ 掉电丢失,写完必须 save_param()
 * @param index 参数地址
 * @param value 值指针
 * @param width 位宽(1/2/4)
 * @return void
 */
void Class_Motor_LZ::write_param(uint16_t index, const void *value, uint8_t width){
    uint8_t txdata[8] = {0};
    const uint8_t *p = (const uint8_t *)value;

    mode = CANCOM_PARA_WRITE;
    txdata[0] = (uint8_t)(index & 0xFF);
    txdata[1] = (uint8_t)(index >> 8);
    txdata[2] = width;                        //官方:Byte2~3 = 位宽
    txdata[3] = 0u;
    if ((p != nullptr) && (width > 0u)) {
        uint8_t n = (width > 4u) ? 4u : width;
        for (uint8_t i = 0u; i < n; i++) {
            txdata[4u + i] = p[i];            //低字节在前
        }
    }
    Transmit_Wait(make_frame_id(motor_init.master_id), txdata, 8, MOTOR_LZ_TX_WAIT_TIMEOUT_MS);
}

/**
 * @brief 写一个 u8 参数(官方类型 0x12)
 * @param index 参数地址,例如 0x7005(运行模式)
 * @param value 参数值
 * @return void
 */
void Class_Motor_LZ::write_param_u8(uint16_t index, uint8_t value){
    write_param(index, &value, 1u);
}

/**
 * @brief 设置运行模式(参数 0x7005)并立刻回读确认
 * @param run_mode MOTOR_LZ_RUNMODE_*(0 = 运控模式,本工程必须用它)
 *
 * 为什么必须有这一步:运控帧(类型 0x01)只在运行模式 = 运控模式时被执行。
 * 回读结果由 recv() → param_answer() 填进 RunMode / RunModeValid。
 * @return void
 */
void Class_Motor_LZ::set_run_mode(uint8_t run_mode){
    write_param_u8(0x7005u, run_mode);
    read_param(0x7005u);
}

/**
 * @brief 保存到电机 Flash(官方类型 0x16,数据区固定 01..08)
 *        **写完任何参数都必须调它**,否则掉电丢失
 * @return 0 = 已进 TX FIFO;1 = 超时
 */
uint8_t Class_Motor_LZ::save_param(void){
    mode = CANCOM_MODE_TOTAL;
    return Transmit_Wait(make_frame_id(motor_init.master_id), motor_send_LZ_Data_save, 8,
                         MOTOR_LZ_TX_WAIT_TIMEOUT_MS);
}

/**
 * @brief 0x7029 zero_sta:写 1 标零(官方:csp/运控模式下可标零,PP 模式会被屏蔽)
 * @param v 1 = 标零
 * @return void
 */
void Class_Motor_LZ::set_zero_sta(uint8_t v){
    write_param(0x7029u, &v, 1u);
}

/**
 * @brief 0x2024 add_offset:把电机零点再挪 v 弧度
 *        官方原意:在够不着零点的地方就地标零,再用它把零点挪到行程一端。
 * @param v 偏置(弧度)
 * @return void
 */
void Class_Motor_LZ::set_add_offset(float v){
    write_param(0x2024u, &v, 4u);
}

/**
 * @brief 0x7026 EPSCAN_time:主动上报间隔
 * @param v 间隔
 * @return void
 */
void Class_Motor_LZ::set_report_time(uint16_t v){
    write_param(0x7026u, &v, 2u);
}

/**
 * @brief 参数应答帧处理(官方类型 0x11 的回帧)
 *        布局:bits 15-8 = 电机 ID;data[0..1] = 参数地址(小端);data[4..7] = 值
 *        本类只关心 0x7005(运行模式)—— 它决定运控帧会不会被执行。
 * @param data 应答帧的 8 字节
 * @return void
 */
void Class_Motor_LZ::param_answer(const uint8_t *data){
    uint16_t index = (uint16_t)((data[1] << 8) | data[0]);

    if (index == 0x7005u) {
        RunMode = data[4];
        RunModeValid = 1u;
    }
}

/**
 * @brief 修改 CAN ID(官方类型 0x07)
 * @param set_id 新的 CAN ID
 *
 * 官方载荷约定(已核对 2026:**F446_CAN_Lingzu 官方例程** + 官方 Python 驱动
 * `robstride_driver.set_can_id()`),类型 0x07 的扩展帧 id 位域是:
 *
 *     bits 28-24 : 类型 = 7
 *     bits 23-16 : **新 ID**(Python 驱动注释里的 Preset ID)
 *     bits 15-8  : **主机 ID**(master/host id,与 enable/lose/zero 那几帧同一个字段)
 *     bits 7-0   : **当前(旧)电机 ID** —— 必须用旧 ID 寻址,否则电机收不到这一帧
 *     数据区     : 全 0(官方驱动不带数据)
 *
 * Python 驱动原文:`extra_data = (new_id << 8) | self.host_id;`
 *                 `ExtId = (7 << 24) | (extra_data << 8) | current_id`
 * 官方 C 例程(使能帧,mode=3)原文:`exdata = 0xfd; //主机ID`
 *                 `ExtId = (id&0xff) | ((exdata&0xffff)<<8) | ((mode&0x1f)<<24)`
 *   ⇒ 数据段(15-8)放主机 ID 这一点两处一致,而 23-16 放新 ID 是 0x07 帧独有的。
 *
 * ⚠️ 本工程未使用该接口(留作现场改 ID 用):
 *    · 必须先失能(官方要求未使能时改);
 *    · 发完这一帧才更新本对象的 id ⇒ 后续帧才用新 ID;
 *    · ID 是否掉电保持官方例程里没体现,需要的话再发一次"保存参数"(类型 0x16)。
 * @return void
 */
void Class_Motor_LZ::motor_set_CAN_ID(uint8_t set_id){
    lose();                                     //官方:未使能时才允许改 ID

    uint32_t frame_id = ((uint32_t)CANCOM_MOTOR_ID              << 24)   //类型 7
                      | ((uint32_t)(set_id & 0xFFu)             << 16)   //新 ID
                      | ((uint32_t)(motor_init.master_id & 0xFFu) << 8)  //主机 ID
                      | ((uint32_t)(motor_init.id & 0xFFu));             //当前(旧)ID

    mode = CANCOM_MOTOR_ID;
    Transmit(frame_id, motor_send_LZ_Data_zero, 8);
    motor_init.id = set_id;                     //发完再更新:这一帧仍然发给旧 ID
}

/**
 * @brief 灵足电机的反馈帧解析
 * @param Data 反馈帧的扩展帧 id(bits 23-22 = pattern,bits 21-16 = 故障码)
 * @param *data 反馈数据数组指针(data[0..1] 位置, [2..3] 速度, [4..5] 力矩, [6..7] 温度)
 * @return void
 */
void Class_Motor_LZ::can_recv(uint32_t Data , const uint8_t *data){
    LastRxTick = HAL_GetTick();               //记下"还活着"的时间戳

    fp32 pos = uint_to_float((uint16_t)((data[0] << 8) | data[1]),
                             motor_init.Pos_min, motor_init.Pos_max, 16);

    recv_data.Now_Pos = pos;
    recv_data.Now_W = uint_to_float((uint16_t)((data[2] << 8) | data[3]),
                                    motor_init.W_min, motor_init.W_max, 16);
    recv_data.Now_T = uint_to_float((uint16_t)((data[4] << 8) | data[5]),
                                    motor_init.T_min, motor_init.T_max, 16);

    recv_data.MError = (uint8_t)((Data >> 16) & 0x3Fu);   //bits 21-16 故障码
    recv_data.mode   = (uint8_t)((Data >> 22) & 0x03u);   //bits 23-22 电机状态 pattern

    fp32 temp = (fp32)(uint16_t)((data[6] << 8) | data[7]) / 10.0f;
    if ((temp > MOTOR_LZ_TEMP_VALID_MAX) || (temp < MOTOR_LZ_TEMP_VALID_MIN)) {
        temp = -1.0f;
    }
    recv_data.Temperature = temp;

    /* ★ 实际状态回读:pattern(0 复位/1 标定/2 运行)= 电机自己报的状态。
       注意它表示的是"电机状态机跑到哪一步",不完全等于"我们使能了没有"。 */
    Motor_Status = recv_data.mode == 2? Motor_Status_ENABLE : Motor_Status_DISABLE;
}

/**
 * @brief 灵足电机的运控帧发送(官方类型 0x01)
 *        打包:data[0..1] 位置, data[2..3] 速度, data[4..5] Kp, data[6..7] Kd,
 *              力矩 → 帧 id 的数据段(bits 23-8)
 * @return void
 */
void Class_Motor_LZ::can_send(){
    //★ 闸门看命令位:Motor_Status 由反馈的 pattern(0复位/1标定/2运行)回读,
    //  电机还在复位/标定阶段时它是 DISABLE —— 拿它当闸门就会在最需要持续发帧的
    //  那段时间把帧停掉,所以闸门只看"我们命令了使能没有"。
    if (Send_Status_Cmd != Motor_Status_ENABLE) return;

    /* ★ 发送前自检"是否在运行":命令位是"要它转",但反馈 pattern 说它不在运行
       (上电/复位后还在复位或编码器标定、或掉线后自己停了),且**没有故障** ⇒
       补一次使能帧(0x03)再发 —— 只靠初始化时 enable 一次的话,掉了就永远回不来。
       ⚠️ 放在闸门之后:急停(lose())过之后命令位是 DISABLE,这里根本不会执行,
          绝不会把急停后的电机自动使能回来。 */
    if (!Is_Enabled())
    {
        if (Has_Fault()) return;
        enable();
    }

    mode = CANCOM_MOTOR_CTRL;

    /* 16bit 定点编码的量程夹紧 —— 不夹会溢出翻转,这是编码必需,不是关节软限位。
       夹的是局部量,不改动 send_data。 */
    fp32 pos = motor_max_min(send_data.Pos, motor_init.Pos_max, motor_init.Pos_min);
    fp32 w   = motor_max_min(send_data.W,   motor_init.W_max,   motor_init.W_min);
    fp32 t   = motor_max_min(send_data.T,   motor_init.T_max,   motor_init.T_min);
    fp32 kp  = motor_max_min(send_data.Kp,  motor_init.Kp_max,  motor_init.Kp_min);
    fp32 kd  = motor_max_min(send_data.Kd,  motor_init.Kd_max,  motor_init.Kd_min);

    uint16_t pos_u = float_to_uint_f(pos, motor_init.Pos_min, motor_init.Pos_max, 16);
    uint16_t w_u   = float_to_uint_f(w,   motor_init.W_min,   motor_init.W_max,   16);
    uint16_t t_u   = float_to_uint_f(t,   motor_init.T_min,   motor_init.T_max,   16);
    uint16_t kp_u  = float_to_uint_f(kp,  motor_init.Kp_min,  motor_init.Kp_max,  16);
    uint16_t kd_u  = float_to_uint_f(kd,  motor_init.Kd_min,  motor_init.Kd_max,  16);

    uint8_t data[8];
    data[0] = (uint8_t)(pos_u >> 8);
    data[1] = (uint8_t)(pos_u & 0xFF);
    data[2] = (uint8_t)(w_u >> 8);
    data[3] = (uint8_t)(w_u & 0xFF);
    data[4] = (uint8_t)(kp_u >> 8);
    data[5] = (uint8_t)(kp_u & 0xFF);
    data[6] = (uint8_t)(kd_u >> 8);
    data[7] = (uint8_t)(kd_u & 0xFF);

    Transmit(make_frame_id(t_u), data, 8);

    Mark_Sent();      //记录本拍时刻,供 App 的 Is_Send_Due() 判断
}

/**
 * @brief 发送一帧运控帧
 * @return void
 */
void Class_Motor_LZ::send() {
    can_send();
}

/**
 * @brief 解析一帧数据(按帧类型分发)
 *        · 类型 0x11 参数应答 → param_answer()
 *        · 类型 0x02 指令应答反馈 / 0x18 主动上报 → can_recv()
 * @param _id   收到帧的扩展帧 id
 * @param _data 数据指针
 * @param _len  数据长度
 * @return void
 */
void Class_Motor_LZ::recv(uint32_t _id, const uint8_t *_data, uint8_t _len) {
    if (_data == nullptr || _len < 8) return;

    if ((uint8_t)((_id >> 24) & 0x1Fu) == CANCOM_PARA_READ) {
        param_answer(_data);                 //参数应答:填运行模式,不是反馈帧
        return;
    }

    can_recv(_id, _data);
}
