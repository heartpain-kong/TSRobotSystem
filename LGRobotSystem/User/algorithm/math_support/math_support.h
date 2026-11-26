/* ============================================================================
 * math_support.h — 数学基础设施(**纯头文件汇总入口,没有对应 .cpp**)
 *
 * 本目录按职责细分,这里只做两件事:
 *   ① 放几个通用常量(用 C++17 的 inline constexpr,不再需要 .cpp 定义);
 *   ② 转发下面 4 个子模块 ⇒ 老代码继续 #include "math_support.h" 即可全都能用。
 *
 *     alg_math_angle.h    角度(归一 / 最短夹角)
 *     alg_math_limit.h    限幅(LimitMax / LimitMinMax)
 *     alg_math_convert.h  定点↔浮点换算 + 大小端
 *     alg_math_matrix.h   4×4 矩阵 + D-H
 *
 * 目录级说明见同目录 math_support.md(它讲的是**整个目录**,不对应某个 .h/.cpp)。
 * ⚠️ motor_max_min 的定义在 motor_mit.cpp(它的家),这里不声明。
 * ========================================================================== */

#ifndef MATH_SUPPORT_H
#define MATH_SUPPORT_H

#include "struct_typedef.h"
#include "arm_math.h"

#include "alg_math_angle.h"
#include "alg_math_limit.h"
#include "alg_math_convert.h"
#include "alg_math_matrix.h"

/* ---- 通用常量(C++17 inline constexpr:头文件里定义,全工程一份) ---- */
inline constexpr float PI2 = 6.2831853084f;                 /* 2π */
inline constexpr float MATH_RPM_TO_RADPS = 2.0f * PI / 60.0f;   /* rpm → rad/s(1rpm = 0.10472) */
inline constexpr float MATH_DEG_TO_RAD = PI / 180.0f;           /* 度 → 弧度 */
inline constexpr float MATH_CELSIUS_TO_KELVIN = 273.15f;        /* ℃ → K */

/**
 * @brief 取绝对值(模板)
 * @note 原来是 (x > 0) ? x : -x:x == 0 时会返回 -0.0f;改成先判负更干净
 */
template<typename Type>
Type Basic_Math_Abs(Type x)
{
    return ((x < Type(0)) ? Type(-x) : x);
}

#endif // MATH_SUPPORT_H