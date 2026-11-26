/* ============================================================================
 * alg_math_limit.h — 限幅(从 math_support 细分出来)
 *
 * ⚠️ 函数名与签名**保持不变** —— 调用方零影响。
 * ★ 历史坑:LimitMax/LimitMinMax 的形参必须是**引用**。以前按值传递 ⇒
 *   Class_PID::Cout() 里的积分限幅/输出限幅**从来没生效过**(积分无限累积)。
 * ========================================================================== */

#ifndef ALG_MATH_LIMIT_H
#define ALG_MATH_LIMIT_H

#include "struct_typedef.h"

/**
 * @brief 对称限幅:|input| > max 时截到 ±max
 * @param input 被限幅的量(**必须传左值**:函数内直接改它)
 * @param max   ±上限;max <= 0 表示**不限制**
 */
void LimitMax(fp32 &input, fp32 max);

/**
 * @brief 不对称限幅:截到 [min, max]
 * @param input 被限幅的量(**必须传左值**)
 * @param max   上限
 * @param min   下限
 * @note max <= min 视为"不限制";参数顺序与 motor_max_min 一致(先 max 后 min)
 */
void LimitMinMax(fp32 &input, fp32 max, fp32 min);

#endif // ALG_MATH_LIMIT_H