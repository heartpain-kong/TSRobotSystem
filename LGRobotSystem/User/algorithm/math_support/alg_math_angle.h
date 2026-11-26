/* ============================================================================
 * alg_math_angle.h — 角度工具(全部用**弧度**)
 *
 * 从 math_support 细分出来的:原来"角度"只有一个 chassis_math_angle_set_ref,
 * 而工程里到处在重复实现"归一到 [0,2π) / 取最短夹角"(底盘、PID、舵轮…),
 * 这里统一一份,大家共用。
 *
 * ★ 为什么需要 Math_Angle_Diff_Pi:
 *   陀螺仪 yaw 过 ±π 会瞬间从 π 跳到 -π(不连续),直接相减会凭空差 2π;
 *   送 cos/sin 之前先取最短带符号夹角即可 —— 见 alg_math_angle.md。
 * ========================================================================== */

#ifndef ALG_MATH_ANGLE_H
#define ALG_MATH_ANGLE_H

#include "struct_typedef.h"

/**
 * @brief 归一到 [0, 2π)
 * @param __angle 任意角(rad)
 * @return [0, 2π) 内的等价角
 */
fp32 Math_Angle_Wrap_2Pi(fp32 __angle);

/**
 * @brief 归一到 (-π, π]
 * @param __angle 任意角(rad)
 * @return (-π, π] 内的等价角
 */
fp32 Math_Angle_Wrap_Pi(fp32 __angle);

/**
 * @brief 两角的最短**带符号**夹角(★ 抗 ±π 跳变)
 * @param __target 目标角(rad)
 * @param __now    当前角(rad)
 * @return ∈ (-π, π],即"从 now 转到 target 的最短方向与角度"
 * @note 等价于把两个角各自归一后再相减并折到 (-π, π],所以对未归一的输入也安全
 */
fp32 Math_Angle_Diff_Pi(fp32 __target, fp32 __now);

/**
 * @brief 两角的最短夹角(**绝对值**,∈ [0, π])
 * @param __ref 反馈角(rad)
 * @param __set 设定角(rad)
 * @return ∈ [0, π]
 * @note 原 math_support 里的 chassis_math_angle_set_ref,语义不变(只是改了名)
 */
fp32 Math_Angle_Abs_Diff(fp32 __ref, fp32 __set);

#endif // ALG_MATH_ANGLE_H