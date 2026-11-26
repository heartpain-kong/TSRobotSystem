//
// alg_math_angle.cpp — 角度工具(弧度)
//

#include "alg_math_angle.h"
#include "math_support.h"      // 只为 PI2(常量仍统一放在 math_support)

/**
 * @brief 归一到 [0, 2π)
 */
fp32 Math_Angle_Wrap_2Pi(fp32 __angle)
{
    while (__angle < 0.0f)
    {
        __angle += 2.0f * PI;
    }
    while (__angle >= 2.0f * PI)
    {
        __angle -= 2.0f * PI;
    }
    return __angle;
}

/**
 * @brief 归一到 (-π, π]
 */
fp32 Math_Angle_Wrap_Pi(fp32 __angle)
{
    return Math_Angle_Wrap_2Pi(__angle + PI) - PI;
}

/**
 * @brief 最短带符号夹角 ∈ (-π, π]
 */
fp32 Math_Angle_Diff_Pi(fp32 __target, fp32 __now)
{
    fp32 d = Math_Angle_Wrap_2Pi(__target) - Math_Angle_Wrap_2Pi(__now);

    if (d > PI)
    {
        d -= 2.0f * PI;
    }
    else if (d < -PI)
    {
        d += 2.0f * PI;
    }
    return d;
}

/**
 * @brief 最短夹角绝对值 ∈ [0, π]
 * @note 原 chassis_math_angle_set_ref 的实现原样保留(两分支写法等价于 |Diff_Pi|)
 */
fp32 Math_Angle_Abs_Diff(fp32 __ref, fp32 __set)
{
    if (__set > __ref)
    {
        return (__set - __ref <= __ref + PI2 - __set) ? (__set - __ref) : (__ref + PI2 - __set);
    }
    return (__ref - __set <= __set + PI2 - __ref) ? (__ref - __set) : (__set + PI2 - __ref);
}