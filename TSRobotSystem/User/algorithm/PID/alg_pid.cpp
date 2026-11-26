//
// alg_pid.cpp — PID 算法实现(接口在 alg_pid.h)
//
// 整个 PID 就这三件事:P 用误差、I 累加(带限幅/抗饱和)、D 差分(可只对反馈、可低通)。
// 每一项为什么这么写,都写在下面每一步的注释里。
//

#include "alg_pid.h"

#include <math.h>       /* fabsf 之类备用 */

/**
 * @brief 初始化
 * @param __KP 比例
 * @param __KI 积分(每拍增益)
 * @param __KD 微分(每拍增益)
 * @param __KF 设定值前馈系数
 * @param __max_out  输出限幅(<= 0 = 不限制)
 * @param __max_iout 积分项限幅(<= 0 = 不限制)
 */
void Class_PID::Init(const fp32 &__KP, const fp32 &__KI, const fp32 &__KD, const fp32 &__KF,
                     const fp32 __max_out, const fp32 __max_iout)
{
    Kp = __KP;
    Ki = __KI;
    Kd = __KD;
    KF = __KF;

    /* 限幅:max_out 是"对称"的对外口径,真正参与运算的是 out_min/out_max */
    max_out  = __max_out;
    max_iout = __max_iout;
    out_max  = __max_out;
    out_min  = -__max_out;

    /* ⚠️ 这里**不动** Improve 位掩码 —— 调用方可能先 Set_Improve 再 Init */
    Clear();
}

/**
 * @brief 按 PD 初始化(Ki = 0;确保「微分先行 + D 项低通」是开的)
 * @note 用 Add_Improve(**只开不关**)⇒ 默认就打开的「抗积分饱和」不会被清掉
 */
void Class_PID::Init_PD(const fp32 &__KP, const fp32 &__KD, const fp32 __max_out, const fp32 __d_alpha)
{
    Init(__KP, 0.0f, __KD, 0.0f, __max_out, 0.0f);

    Add_Improve((Enum_PID_Improve)(PID_Improve_Derivative_On_Measurement
                                 | PID_Improve_Derivative_Filter));
    D_Alpha = __d_alpha;
}

/**
 * @brief 算一拍,结果在 out / Pout / Iout / Dout / Fout 里
 *
 * 顺序:误差 → P → I(带抗积分饱和)→ D(带微分先行/低通)→ 前馈 → 求和 → 限幅
 */
void Class_PID::Cout()
{
    /* ---------------- 误差 ---------------- */
    error[1] = error[0];
    /* ★ 角度环取最短夹角,避免 2π→0 抖动 */
    error[0] = angle_mode ? Math_Angle_Diff_Pi(Target, Now) : (Target - Now);

    /* ---------------- P ---------------- */
    Pout = Kp * error[0];

    /* ---------------- I ----------------
       抗积分饱和:上一拍输出已经被限幅削掉过(顶在限幅上),这一拍就**别再往同方向堆积分**,
       否则误差反回来时要等积分"退"完才能动,表现为"冲出去回不来"。 */
    if (!((Improve & PID_Improve_Anti_Windup) && saturated))
    {
        Iout += Ki * error[0];
    }
    LimitMax(Iout, max_iout);        /* max_iout <= 0 = 不限制 */

    /* ---------------- D ---------------- */
    fp32 d_raw;

    if (Improve & PID_Improve_Derivative_On_Measurement)
    {
        /* 微分先行:对**反馈**微分(取负号 = 目标不变时的 d(error)/dt)。
           好处:目标阶跃时 D 项不会踢出一个尖峰,只有反馈噪声影响它。
           ★ 第一次(或 Clear() 之后第一次)只把基准对齐到当前反馈、D 记 0 ——
             否则 Last_Now 还是 0,会在"电机当前不在零位"时造出一个假速度,使能瞬间弹一下。 */
        if (!D_Ready)
        {
            D_Ready = true;
            d_raw = 0.0f;
        }
        else
        {
            /* 角度环:Now 已经归一化到 [0,2π),差分必须取最短角,
               否则 2π→0 会产生 -2π 的假速度尖峰 */
            d_raw = angle_mode ? -Math_Angle_Diff_Pi(Now, Last_Now)
                               : -(Now - Last_Now);
        }
    }
    else
    {
        /* 默认:对误差微分(目标阶跃会在这里出尖峰,所以位置环一般用上面的微分先行) */
        d_raw = error[0] - error[1];
    }
    Last_Now = Now;

    /* D 项一阶低通:Dfilter += alpha * (新值 - 旧值);alpha 越小越平滑 */
    if (Improve & PID_Improve_Derivative_Filter)
    {
        Dfilter += D_Alpha * (d_raw - Dfilter);
        d_raw = Dfilter;
    }
    Dout = Kd * d_raw;

    /* ---------------- 前馈 ---------------- */
    Fout = KF * Target + FF;

    /* ---------------- 求和 → 限幅 ---------------- */
    const fp32 out_raw = Pout + Iout + Dout + Fout;

    out = out_raw;
    if (max_out > 0.0f)
    {
        LimitMinMax(out, out_max, out_min);
    }

    /* 给下一拍的抗积分饱和用:输出被削掉过就说明饱和了 */
    saturated = (out != out_raw);
}

/**
 * @brief 清运行状态(积分、误差历史、D 项低通、饱和标志),并让微分基准重新对齐
 * @note 使能瞬间、切换控制模式、或者目标发生大跳变时调一次
 */
void Class_PID::Clear()
{
    error[0] = error[1] = 0.0f;
    out = Pout = Iout = Dout = Fout = 0.0f;

    Last_Now  = 0.0f;
    Dfilter   = 0.0f;
    saturated = false;

    /* 下次 Cout() 先对齐微分基准,再开始微分 */
    D_Ready = false;
}
