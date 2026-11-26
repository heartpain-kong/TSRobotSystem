/* ============================================================================
 * alg_pid.h — PID 控制器(P / PI / PD / PID 都靠它)
 *
 * 输出 = Pout + Iout + Dout + Fout
 *   Pout = Kp * e                e = Target - Now
 *   Iout = Iout + Ki * e         每拍累加,受 max_iout 限幅
 *   Dout = Kd * d                差分;d 怎么算见下面两个开关
 *   Fout = KF * Target + FF      KF 是设定值前馈;FF 是外部每拍注入的前馈
 *
 * ★ Ki / Kd 是"**每拍**增益"(不看 dt),和 RM 的写法一致:
 *   调用周期固定时直接调就行了(1kHz 任务里,积分就是每秒积 1000 次)。
 *
 * 最常用形态(位置环/速度环的 PD):
 *     PID.Init_PD(30.0f, 1.0f, 20.0f, 0.3f);   // Kp, Kd, 输出限幅, D 项低通系数
 *     PID.Set_Target(目标);
 *     PID.Set_Now(电机反馈);
 *     PID.Cout();
 *     float out = PID.Get_Out();
 * 想加积分兜静差:Init_PD 之后 Set_KI(小值) + Set_Max_iout(…) 即可
 * (抗积分饱和**默认就是开的** ✓,不用再手动 Add_Improve)。
 *
 * 想要"最朴素、不带任何额外环节"的 PID(和最早版本完全一致):
 *     PID.Set_Improve(PID_Improve_NONE);
 *
 * 布局:类里只有**声明**;一行式 setter/getter 的实现统一放在**文件底部**
 *       (见 "inline 实现" 一节);真正的算法在 alg_pid.cpp。
 * 详细说明见同目录 alg_pid.md。
 * ========================================================================== */

#ifndef ALG_PID_H
#define ALG_PID_H

#include "alg_math_angle.h"
#include "math_support.h"

/* PID 的可选优化环节(位掩码,可或起来送给 Set_Improve / Add_Improve)
 *
 * 每个开关独占一位(值是 2 的幂)⇒ 互不干扰,可任意组合。
 * 历史上还堆过:输出低通、误差死区、积分分离、变速积分、梯形积分 —— 全工程没有任何
 * 调用者,却让 Cout() 变成一堆分支,已删除;需要时按 alg_pid.md 末尾的说明加回来。 */
enum Enum_PID_Improve
{
    PID_Improve_NONE                      = 0x00,  /* 全关 = 最朴素的 PID(最初的行为) */
    PID_Improve_Derivative_On_Measurement = 0x01,  /* 微分先行:只对反馈微分(目标阶跃时 D 不突变) */
    PID_Improve_Derivative_Filter         = 0x02,  /* D 项一阶低通:压反馈噪声 */
    PID_Improve_Anti_Windup               = 0x04,  /* 抗积分饱和:上一拍输出被限幅过,这一拍不积分 */

    /* ★ 默认值:三个全开
       Ki = 0(纯 PD)时抗积分饱和是空操作(Iout 恒为 0),不影响任何东西;
       一旦加了 Ki,它正好防住"冲出去回不来"。所以默认全开最省心。 */
    PID_Improve_DEFAULT                   = 0x07,
};

/**
 * @brief PID 控制器
 */
class Class_PID
{
public:

    
    /**
     * @brief 切换到"角度环"模式:误差取**最短夹角**(-π,π]
     *   解决 2π→0 (或 0→2π) 时 error 跳一圈导致的抖动 ✓
     */
    inline void Set_Angle_Mode(const bool __on);

/* ---------------- 初始化 ---------------- */

    /**
     * @brief 初始化(参数顺序同 RM 的习惯写法)
     * @param __KP       比例
     * @param __KI       积分(每拍增益;填 0 = 不要积分,即 PD)
     * @param __KD       微分(每拍增益)
     * @param __KF       设定值前馈系数(Fout 里加 KF*Target;不用填 0)
     * @param __max_out  输出限幅(<= 0 = 不限制)
     * @param __max_iout 积分项限幅(<= 0 = 不限制)
     * @note 只清运行状态,**不动** Improve 位掩码(所以 Set_Improve 可以先调)
     */
    void Init(const fp32 &__KP, const fp32 &__KI, const fp32 &__KD, const fp32 &__KF,
              const fp32 __max_out, const fp32 __max_iout);

    /**
     * @brief 按 PD 初始化(位置环最常用):Ki = 0,并确保「微分先行 + D 项低通」是开的
     * @param __KP      比例
     * @param __KD      微分(每拍增益)
     * @param __max_out 输出限幅(<= 0 = 不限制)
     * @param __d_alpha D 项低通系数(0~1,越小越平滑、相位滞后越大;推荐 0.1~0.3)
     * @note 用的是 Add_Improve(**只开不关**),所以默认打开的「抗积分饱和」不会被它清掉
     */
    void Init_PD(const fp32 &__KP, const fp32 &__KD, const fp32 __max_out, const fp32 __d_alpha = 0.2f);

    /* ---------------- 计算 ---------------- */

    /** 跑一拍:先 Set_Target / Set_Now(要前馈再 Set_Feedforward),然后调它 */
    void Cout();

    /** 清运行状态(积分、误差历史、D 项低通、饱和标志),并让微分基准重新对齐 —— 使能/切模式时调 */
    void Clear();

    /* ---------------- 设定值 / 反馈 ---------------- */

    inline void Set_Target(const fp32 &__Target);
    inline fp32 Get_Target() const;
    inline void Set_Now(const fp32 &__Now);
    inline fp32 Get_Now() const;

    /* ---------------- 调参 ---------------- */

    inline void Set_KP(const fp32 &__KP);
    inline fp32 Get_KP() const;
    inline void Set_KI(const fp32 &__KI);
    inline fp32 Get_KI() const;
    inline void Set_KD(const fp32 &__KD);
    inline fp32 Get_KD() const;
    inline void Set_KF(const fp32 &__KF);
    inline fp32 Get_KF() const;
    /** 整体替换(没列出的位会被关掉) */
    inline void Set_Improve(const Enum_PID_Improve &__improve);
    inline Enum_PID_Improve Get_Improve() const;
    /** 打开若干环节(只动这些位,其它位不变) */
    inline void Add_Improve(const Enum_PID_Improve &__improve);
    /** 关掉若干环节(只动这些位,其它位不变) */
    inline void Del_Improve(const Enum_PID_Improve &__improve);

    /** D 项低通系数(0~1),配合 PID_Improve_Derivative_Filter */
    inline void Set_D_Filter_Alpha(const fp32 &__alpha);
    inline fp32 Get_D_Filter_Alpha() const;

    /* ---------------- 外部前馈 ---------------- */

    /** 每拍注入一次(重力/摩擦补偿、上一环输出…);不会自动清零 */
    inline void Set_Feedforward(const fp32 __ff);
    inline void Add_Feedforward(const fp32 __ff);
    inline fp32 Get_Feedforward() const;

    inline fp32 Get_Out() const;
    inline fp32 Get_Pout() const;
    inline fp32 Get_Iout() const;
    inline fp32 Get_Dout() const;
    inline fp32 Get_Fout() const;
    /** 最近一拍的误差(Target - Now) */
    inline fp32 Get_Error() const;
    /** 上一拍输出是否被限幅削掉过(抗积分饱和的依据) */
    inline bool Is_Saturated() const;

    bool angle_mode = false;      /* true = 角度环(最短夹角) */

protected:

    inline void Set_Max_out(const fp32 &__max_out);
    inline fp32 Get_Max_out() const;

    /** 积分项限幅(<= 0 = 不限制) */
    inline void Set_Max_iout(const fp32 &__max_iout);
    inline fp32 Get_Max_iout() const;

    /** 不对称输出限幅:输出截到 [__min, __max](需要"正转多、反转少"时用) */
    inline void Set_Out_Limit(const fp32 __min, const fp32 __max);


    /* ---- 输入 ---- */
    fp32 Target = 0.0f;                     /* 目标值 */
    fp32 Now    = 0.0f;                     /* 当前反馈 */

    /* ---- 系数 ---- */
    fp32 Kp = 0.0f;
    fp32 Ki = 0.0f;
    fp32 Kd = 0.0f;
    fp32 KF = 0.0f;                         /* 设定值前馈 */

    /* ---- 限幅(<= 0 = 不限制) ---- */
    fp32 max_out  = 0.0f;                   /* 对称输出限幅(Get_Max_out 看它) */
    fp32 max_iout = 0.0f;                   /* 积分项限幅 */
    fp32 out_max  = 0.0f;                   /* 实际用的上限 */
    fp32 out_min  = 0.0f;                   /* 实际用的下限 */

    /* ---- 各项输出 ---- */
    fp32 out  = 0.0f;
    fp32 Pout = 0.0f;
    fp32 Iout = 0.0f;
    fp32 Dout = 0.0f;
    fp32 Fout = 0.0f;                       /* KF*Target + FF */

    /* ---- 误差历史(error[0] 本拍,error[1] 上一拍) ---- */
    fp32 error[2] = {0.0f, 0.0f};

    /* ---- 优化环节的状态 ---- */
    Enum_PID_Improve Improve = PID_Improve_DEFAULT;   /* ★ 默认三个全开 */
    fp32 Last_Now = 0.0f;                   /* 上一拍反馈(微分先行用) */
    bool D_Ready  = false;                  /* 微分基准是否已对齐(没对齐时本拍 D 记 0) */
    fp32 D_Alpha  = 0.2f;                   /* D 项低通系数 */
    fp32 Dfilter  = 0.0f;                   /* D 项低通状态 */
    fp32 FF       = 0.0f;                   /* 外部前馈(每拍注入) */
    bool saturated = false;                 /* 上一拍输出是否被限幅过 */
};


/* ============================================================================
 * inline 实现(一行式 setter/getter;算法在 alg_pid.cpp)
 * ========================================================================== */

inline void Class_PID::Set_Angle_Mode(const bool __on) {
    angle_mode = __on;
}

inline void Class_PID::Set_Target(const fp32 &__Target) {
    Target = __Target;
}
inline fp32 Class_PID::Get_Target() const {
    return Target;
}
inline void Class_PID::Set_Now(const fp32 &__Now) {
    Now = __Now;
}
inline fp32 Class_PID::Get_Now() const {
    return Now;
}

inline void Class_PID::Set_KP(const fp32 &__KP) {
    Kp = __KP;
}
inline fp32 Class_PID::Get_KP() const {
    return Kp;
}
inline void Class_PID::Set_KI(const fp32 &__KI) {
    Ki = __KI;
}

inline fp32 Class_PID::Get_KI() const {
    return Ki;
}

inline void Class_PID::Set_KD(const fp32 &__KD) {
    Kd = __KD;
}

inline fp32 Class_PID::Get_KD() const {
    return Kd;
}

inline void Class_PID::Set_KF(const fp32 &__KF) {
    KF = __KF;
}
inline fp32 Class_PID::Get_KF() const {
    return KF;
}

inline void Class_PID::Set_Max_out(const fp32 &__max_out)
{
    max_out = __max_out;
    out_max = __max_out;
    out_min = -__max_out;
}
inline fp32 Class_PID::Get_Max_out() const {
    return max_out;
}

inline void Class_PID::Set_Max_iout(const fp32 &__max_iout) {
    max_iout = __max_iout;
}
inline fp32 Class_PID::Get_Max_iout() const {
    return max_iout;
}

inline void Class_PID::Set_Out_Limit(const fp32 __min, const fp32 __max)
{
    out_min = __min;
    out_max = __max;
}

inline void Class_PID::Set_Improve(const Enum_PID_Improve &__improve) {
    Improve = __improve;
}

inline Enum_PID_Improve Class_PID::Get_Improve() const {
    return Improve;
}

inline void Class_PID::Add_Improve(const Enum_PID_Improve &__improve)
{
    Improve = (Enum_PID_Improve)(Improve | __improve);      /* 只开这几位 */
}
inline void Class_PID::Del_Improve(const Enum_PID_Improve &__improve)
{
    Improve = (Enum_PID_Improve)(Improve & (~__improve));   /* 只关这几位 */
}

inline void Class_PID::Set_D_Filter_Alpha(const fp32 &__alpha) {
    D_Alpha = __alpha;
}

inline fp32 Class_PID::Get_D_Filter_Alpha() const {
    return D_Alpha;
}

inline void Class_PID::Set_Feedforward(const fp32 __ff) {
    FF = __ff;
}

inline void Class_PID::Add_Feedforward(const fp32 __ff) {
    FF += __ff;
}

inline fp32 Class_PID::Get_Feedforward() const {
    return FF;
}

inline fp32 Class_PID::Get_Out() const {
    return out;
}

inline fp32 Class_PID::Get_Pout() const {
    return Pout;
}

inline fp32 Class_PID::Get_Iout() const {
    return Iout;
}

inline fp32 Class_PID::Get_Dout() const {
    return Dout;
}

inline fp32 Class_PID::Get_Fout() const {
    return Fout;
}

inline fp32 Class_PID::Get_Error() const {
    return error[0];
}

inline bool Class_PID::Is_Saturated() const {
    return saturated;
}

#endif // ALG_PID_H
