//
// Created by heartpain on 2026/9/24.
//

#ifndef MOTOR_MIT_H
#define MOTOR_MIT_H

#include "struct_typedef.h"
#include "alg_math_convert.h"   //定点/浮点换算(实现已移到算法层,三家电机共用)
#include "bsp_fdcan.h"
#include "alg_math_convert.h"   //定点/浮点换算(实现已移到算法层,三家电机共用)
#include "bsp_uart.h"
#include "alg_math_convert.h"   //定点/浮点换算(实现已移到算法层,三家电机共用)
#include "bsp_timestamp.h" //Is_Send_Due()/Mark_Sent() 用 TIM5 微秒时间戳(Timestamp_Get_Us32)

//通信方式
enum Enum_communication
{
    Enum_UART = 0,
    Enum_FD_CAN,
};

/* 运控帧发送频率
 *  ★ 枚举值**直接就是 Hz** —— 这样 period_us = 1000000 / 值,不需要查表,
 *    也就没有"枚举和表不同步"的隐患;带宽预算也是 freq/1000 帧每周期,直接算。
 *    (索引式枚举 0/1/2/3 的唯一好处是能当数组下标,本工程没这需求)
 *
 *  ⚠️ 取值只列**能整除 1000** 的档位,配合 1ms 的 FreeRTOS tick 做粗调度好看好算。
 *  ⚠️ 1ms tick 下有效发送频率**上限就是 1000Hz** —— 任务 1ms 醒一次,
 *     再高的频率也只能按 1ms 发。要 2kHz 得提高 configTICK_RATE_HZ,
 *     或让定时器中断 vTaskNotifyGiveFromISR 把任务叫醒。
 *  ⚠️ 带宽是硬约束:同一根 CAN 上所有运控帧 + 反馈帧共享 1Mbps,
 *     实测 12 台灵足 @500Hz 就能把总线灌到 ~97%,别只看单台。 */
enum Enum_Motor_Control_Freq
{
    Motor_Control_Freq_100HZ  = 100,
    Motor_Control_Freq_200HZ  = 200,
    Motor_Control_Freq_500HZ  = 500,
    Motor_Control_Freq_1000HZ = 1000,
};

//电机状态
enum Enum_Motor_Status
{
    Motor_Status_DISABLE = 0,
    Motor_Status_ENABLE,
};

//电机控制模式
enum motor_Control_mode
{
    motor_pos = 0,
    motor_speed = 1,
    motor_T     = 2,
};

//统一发送量
struct Struct_send_motor
{
    fp32 Pos;
    fp32 T;
    fp32 W;
    fp32 Kp;
    fp32 Kd;
};

//统一反馈量
struct Struct_recv_motor
{
    uint8_t MError;
    uint8_t mode;
    fp32 Now_Pos;
    fp32 Now_W;                    //角速度
    fp32 Now_T;                    //力矩
    fp32 Temperature;
    int32_t CurrentValue;
};

//统一初始化参数
//  各品牌自己的型号/控制模式枚举直接以数值塞进 model / control_mode,
//  品牌类在重写 Init 时再转回自己的枚举类型。
//  限幅字段留 0 表示"用该型号的默认限幅"(见各品牌 Init 实现)。
struct Struct_motor_init
{
    Enum_communication  communication;
    uint16_t id;                   //电机自身 id(CAN 标准帧 id / 485 id)
    uint16_t master_id;            //主机 id(达妙反馈 id / 灵足 user id 等)
    motor_Control_mode  control_mode;         //控制模式(各品牌枚举值)
    fp32 Kp_max;
    fp32 Kp_min;
    fp32 Kd_max;
    fp32 Kd_min;
    fp32 T_max;
    fp32 T_min;
    fp32 Pos_max;
    fp32 Pos_min;
    fp32 W_max;
    fp32 W_min;
    FDCAN_HandleTypeDef *FDcan;
    UART_HandleTypeDef *UART;
    //发送频率(初始化时指定)
    //  各品牌 Init 签名不同:DM 已经把频率作为 Init 的最后一个参数收进来,
    //  并同步到基类活值(Set_Control_Freq) + 在这里留一份记录。
    //  ⚠️ 真正决定"多久发一帧"的是基类的 motor_mit::Control_Freq / Control_Period_Us,
    //     这个字段只是给上层读的副本。
    //  给个默认值:不显式设置时就是 1000Hz,和基类 Control_Freq 的默认值一致
    //  (否则这里会是 0,而基类周期仍是默认 1000us,两边对不上)。
    Enum_Motor_Control_Freq Control_Freq = Motor_Control_Freq_1000HZ;
};

class motor_mit {
    public:

    motor_mit() = default;
    virtual ~motor_mit() = default;

    /*================ 纯接口:只声明不实现,各品牌电机各自重写 ================*/
    /**
     * @brief 按当前设定值组帧并发送一帧控制量
     */
    virtual void send() = 0;

    /**
     * @brief 解析一帧反馈数据
     * @param _id   收到帧的 id(标准帧/扩展帧 id;UART 电机可传 0)
     * @param _data 数据指针
     * @param _len  数据长度
     */
    virtual void recv(uint32_t _id, const uint8_t *_data, uint8_t _len) = 0;

    /**
     * @brief 设置机械零点
     */
    virtual void zero() = 0;

    /**
     * @brief 电机使能
     */
    virtual void enable() = 0;

    /**
     * @brief 电机失能
     */
    virtual void lose() = 0;

    /*================ 公共数据契约(四类电机共用的设定值/反馈量/状态) ================*/
    fp32 Get_Now_Pos() const;

    fp32 Get_Now_W() const;

    fp32 Get_Now_T() const;

    fp32 Get_Motor_Temperature() const;

    uint8_t Get_MError() const ;

    uint8_t Get_Feedback_Mode() const;


    void Set_Send_Data(const Struct_send_motor &_send);
    Struct_send_motor Get_Send_Data() const;

    void Set_Recv_Data(const Struct_recv_motor &_recv);
    Struct_recv_motor Get_Recv_Data() const;

    void Set_Pos(const fp32 _Pos);
    fp32 Get_Pos() const;

    void Set_W(const fp32 _W);
    fp32 Get_W() const;

    void Set_T(const fp32 _T);
    fp32 Get_T() const;

    void Set_Kp(const fp32 _Kp);
    fp32 Get_Kp() const;

    void Set_Kd(const fp32 _Kd);
    fp32 Get_Kd() const;

    void Set_Motor_Status(const Enum_Motor_Status _Motor_Status);
    Enum_Motor_Status Get_Motor_Status() const;

    /*================ 发送前自检:是否已使能 / 有没有真故障 ================
     * 用途:各品牌 send() 里统一做一次
     *     if (Send_Status_Cmd != ENABLE) return;        // 命令位:尊重急停,不自愈
     *     if (!Is_Enabled()) {                          // 反馈说实际没使能
     *         if (Has_Fault()) return;                  // 有真故障:不硬使能
     *         enable();                                 // 无故障 ⇒ 补一次使能
     *     }
     *   ⇒ 解决两类问题:
     *      · 上电时没使能过 / 使能帧丢了;
     *      · **看门狗掉线自己失能**(达妙 0xD、灵足回到复位/标定、宇树 STATUS 掉出 FOC)
     *        —— 只靠初始化 enable() 一次的话,掉了就永远回不来。
     *
     * 各品牌怎么填:
     *   · 达妙:Is_Enabled = (ERR==1);Has_Fault = ERR 属 0x8~0xE 且非 0xD(掉线可自愈)
     *   · 灵足:Is_Enabled = (反馈 pattern==2 运行);Has_Fault = (故障码 != 0)
     *   · 宇树:Is_Enabled = (反馈 STATUS==FOC);    Has_Fault = (MError != 0)
     *   · 大疆:**没有使能帧**,反馈里也没有状态/故障字段 ⇒ 用默认值(恒"已使能/无故障"),
     *          即"不需要也不会"自动补使能(它的 enable() 只是置命令位 + 清 PID)。
     *
     * ⚠️ 默认值故意取"已使能 + 无故障":没实现该语义的品牌 ⇒ 这段自愈代码永不生效,
     *    不会出现"凭默认值瞎发使能帧"的情况。 */
    virtual bool Is_Enabled() const { return true; }
    virtual bool Has_Fault()  const { return false; }

    /*================ 命令状态 vs 实际状态(必须分开!)================
     *   Send_Status_Cmd —— 我们**命令**电机处于的模式:enable()/lose() 改它,
     *                       send() 用它当闸门,也是运控帧里 STATUS 字段的来源。
     *   Motor_Status    —— 电机**实际**报回来的状态:能回读反馈的品牌(宇树看反馈
     *                       STATUS、灵足看 pattern)由 recv() 更新;
     *                       协议里没有模式回读的品牌(达妙/大疆)只能镜像命令值。
     *                       Get_Motor_Status() 读的就是它。
     *
     * ⚠️ 为什么 send() 的闸门**推荐**用 Send_Status_Cmd,而不是 Motor_Status:
     *   注意前提 —— 这类电机是"每收到一帧命令就回一帧",所以发使能帧本身也会
     *   引来一帧反馈,靠反馈判状态**通常**是通的(不是无条件死锁)。但闸门挂在
     *   反馈上有两个隐患:
     *     ① 反馈丢帧/CRC 不过/还没到 → 这一拍不发(下一拍能自愈,只丢一拍);
     *     ② 电机报的是**故障码**而不是"使能"(如达妙 ERR=8~E) → Motor_Status 一直
     *        是 DISABLE → 闸门**永远关着**,而某些故障恰恰要靠继续收帧才能恢复
     *        ⇒ 这时才是真死锁。
     *   所以:"该不该发"看命令(我们允许它转),"它现在怎样"看反馈(Motor_Status)。
     */
    Enum_Motor_Status Get_Send_Status() const;

    void Set_Motor_Control_mode(const motor_Control_mode _Motor_Control_mode);
    motor_Control_mode Get_Motor_Control_mode() const;

    /*================ 控制周期(调度用,不是协议的一部分)================
     * 分工:**状态放电机里,判断放 App 里**
     *   · 各品牌 send() 真正把帧发出去之后会 Mark_Sent(),把时刻记进 send_time
     *   · App 每拍只做一件事:
     *         for (每个电机 m) if (m->Is_Send_Due()) m->send();
     *   · Is_Send_Due() **只读、不改任何状态** —— 所以"先判断再发"是安全的,
     *     不存在"判过一次就把下次到期时刻推走、结果一帧都发不出去"的双重门控坑;
     *     时刻只由 send() 内部记录,App 不会忘。
     */

    /** 设置发送频率(顺便算出周期,并让第一帧立即到期) */
    void Set_Control_Freq(const Enum_Motor_Control_Freq _Control_Freq);
    Enum_Motor_Control_Freq Get_Control_Freq() const;
    /** 当前发送周期 [us] = 1000000 / 频率 */
    uint32_t Get_Control_Period_Us() const;

    /** 上一次发送的时刻 [us](Timestamp_Get_Us32 的值域,71.6 分钟回绕) */
    uint32_t Get_Send_Time() const;
    /** 距上一次发送过了多久 [us](只读;回绕安全) */
    uint32_t Get_Send_Interval_Us() const;
    /** 这一拍该不该发:距上次发送 >= 一个周期(只读,不改状态) */
    bool Is_Send_Due() const;

    protected:

    //命令:我们要电机处于的模式(enable()/lose() 改;send() 的闸门)
    //  注意:它必须在 protected 里,而上面的 Set/Get_Control_Freq、Is_Send_Due 等
    //  是 **public**(App 要用来排程)—— 别把这里的 protected: 提前,否则会把它们一起关进来
    Enum_Motor_Status Send_Status_Cmd = Motor_Status_DISABLE;

    /** 各品牌 send() 里真正把帧发出去之后调用:记录本拍时刻 */
    void Mark_Sent();

    //设定值
    Struct_send_motor send_data{};

    //反馈量(带品牌私有反馈结构的电机,可重写上面的 Get_Now_xxx 用自己的数据)
    Struct_recv_motor recv_data{};

    //状态
    Enum_Motor_Status  Motor_Status = Motor_Status_DISABLE;

    //控制模式
    motor_Control_mode Motor_Control_mode = motor_T;

    //运控帧发送频率(活值,Set_Control_Freq 改)
    Enum_Motor_Control_Freq Control_Freq = Motor_Control_Freq_1000HZ;
    //发送周期 [us] = 1000000 / Control_Freq
    uint32_t Control_Period_Us = (1000000u / (uint32_t)Motor_Control_Freq_1000HZ);
    //上一次发送的时刻 [us](由 Mark_Sent() 更新,App 只读)
    uint32_t send_time = 0;
};

extern  uint8_t motor_send_Data_lose[8];
//使能
extern  uint8_t motor_send_Data_enable[8];
//零点设置
extern  uint8_t motor_send_Data_zero[8];
//清除错误(达妙 0xFB;过温/过流等故障排除后要发这一帧才能恢复) —— 只有达妙用得上
extern  uint8_t motor_send_Data_clear_error[8];

fp32 motor_max_min(fp32 proto,fp32 max,fp32 min);

inline void motor_mit::Set_Motor_Status(const Enum_Motor_Status _Motor_Status) {
    Motor_Status = _Motor_Status;
}

inline Enum_Motor_Status motor_mit::Get_Motor_Status() const {
    return Motor_Status;
}

inline void motor_mit::Set_Motor_Control_mode(const motor_Control_mode _Motor_Control_mode) {
    Motor_Control_mode = _Motor_Control_mode;
}

inline motor_Control_mode motor_mit::Get_Motor_Control_mode() const {
    return Motor_Control_mode;
}

/*---- 命令状态 ----*/

inline Enum_Motor_Status motor_mit::Get_Send_Status() const {
    return Send_Status_Cmd;
}

/*---- 控制周期:状态在电机里,判断在 App 里 ----*/

inline void motor_mit::Set_Control_Freq(const Enum_Motor_Control_Freq _Control_Freq) {
    Control_Freq = _Control_Freq;
    if ((uint32_t)_Control_Freq != 0u) {
        Control_Period_Us = 1000000u / (uint32_t)_Control_Freq;
    }
    //把"上次发送时刻"回拨一个周期 → 第一帧立即到期,不用等一个周期才动
    send_time = Timestamp_Get_Us32() - Control_Period_Us;
}

inline Enum_Motor_Control_Freq motor_mit::Get_Control_Freq() const {
    return Control_Freq;
}

inline uint32_t motor_mit::Get_Control_Period_Us() const {
    return Control_Period_Us;
}

inline void motor_mit::Mark_Sent() {
    send_time = Timestamp_Get_Us32();
}

inline uint32_t motor_mit::Get_Send_Time() const {
    return send_time;
}

inline uint32_t motor_mit::Get_Send_Interval_Us() const {
    /* 无符号相减 = 取模减法,回绕安全:差值在 2^32 us(约 71.6 分钟)内永远正确 */
    return (uint32_t)(Timestamp_Get_Us32() - send_time);
}

inline bool motor_mit::Is_Send_Due() const {
    /* 只读判断,不改任何状态 —— 所以"先判断再 send()"是安全的,
       时刻由 send() 内部 Mark_Sent() 记录,不存在双重门控 */
    return (Get_Send_Interval_Us() >= Control_Period_Us);
}

inline void motor_mit::Set_Send_Data(const Struct_send_motor &_send) {
    send_data = _send;
}

inline Struct_send_motor motor_mit::Get_Send_Data() const {
    return send_data;
}

inline void motor_mit::Set_Recv_Data(const Struct_recv_motor &_recv) {
    recv_data = _recv;
}

inline Struct_recv_motor motor_mit::Get_Recv_Data() const {
    return recv_data;
}

/*---- 反馈接口的默认实现:直接读公共的 recv_data ----*/

inline fp32 motor_mit::Get_Now_Pos() const {
    return recv_data.Now_Pos;
}

inline fp32 motor_mit::Get_Now_W() const {
    return recv_data.Now_W;
}

inline fp32 motor_mit::Get_Now_T() const {
    return recv_data.Now_T;
}

inline fp32 motor_mit::Get_Motor_Temperature() const {
    return recv_data.Temperature;
}

inline uint8_t motor_mit::Get_MError() const {
    return recv_data.MError;
}

inline uint8_t motor_mit::Get_Feedback_Mode() const {
    return recv_data.mode;
}

inline void motor_mit::Set_Pos(const fp32 _Pos) {
    send_data.Pos = _Pos;
}

inline fp32 motor_mit::Get_Pos() const {
    return send_data.Pos;
}

inline void motor_mit::Set_W(const fp32 _W) {
    send_data.W = _W;
}

inline fp32 motor_mit::Get_W() const {
    return send_data.W;
}

inline void motor_mit::Set_T(const fp32 _T) {
    send_data.T = _T;
}

inline fp32 motor_mit::Get_T() const {
    return send_data.T;
}

inline void motor_mit::Set_Kp(const fp32 _Kp) {
    send_data.Kp = _Kp;
}

inline fp32 motor_mit::Get_Kp() const {
    return send_data.Kp;
}

inline void motor_mit::Set_Kd(const fp32 _Kd) {
    send_data.Kd = _Kd;
}

inline fp32 motor_mit::Get_Kd() const {
    return send_data.Kd;
}

#endif //TSROBOTSYSTEM_MOTOR_MIT_H
