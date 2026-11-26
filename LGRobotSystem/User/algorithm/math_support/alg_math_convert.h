/* ============================================================================
 * alg_math_convert.h — 定点/浮点换算 与 字节序转换
 *
 * 从两处收拢而来(原来定义在 motor_mit.cpp,却在 math_support.h 和 motor_mit.h
 * 里各声明了一遍 —— 跨模块重复声明):
 *   · uint_to_float / float_to_uint_f / float_to_uint_i:协议定点量 ↔ 浮点量,
 *     达妙(DM)、瓴控(LZ)、MIT 三家都在用,所以提到算法层共用;
 *   · Math_Endian_Reverse_16:大小端翻转。
 *
 * 换算公式(与原来一字不差):
 *     定点 → 浮点:  x = raw * span / (2^bits - 1) + x_min
 *     浮点 → 定点:  raw = (x - x_min) * (2^bits - 1) / span
 *   其中 span = x_max - x_min。
 * ========================================================================== */

#ifndef ALG_MATH_CONVERT_H
#define ALG_MATH_CONVERT_H

#include "struct_typedef.h"

#include <stdbool.h>

/**
 * @brief 定点量(int) → 浮点量
 * @param x_int 原始定点量
 * @param x_min 该物理量的最小值
 * @param x_max 该物理量的最大值
 * @param bits  定点量位数(协议里一般 12 或 16)
 * @return 浮点物理量
 */
fp32 uint_to_float(uint16_t x_int, fp32 x_min, fp32 x_max, uint8_t bits);

/**
 * @brief 浮点量 → 定点量(上下限也是浮点)
 * @return 定点量(未做饱和,超范围会回绕,调用方自己先限幅)
 */
uint16_t float_to_uint_f(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits);

/**
 * @brief 浮点量 → 定点量(上下限用整数给)
 * @return 定点量
 */
uint16_t float_to_uint_i(fp32 x, uint16_t x_min, uint16_t x_max, uint8_t bits);

/**
 * @brief 16 位大小端翻转
 * @param Source      源(2 字节)
 * @param Destination 目标;传 nullptr 则只返回、不写回
 * @return 翻转后的 16 位值
 */
uint16_t Math_Endian_Reverse_16(void *Source, void *Destination);

#endif // ALG_MATH_CONVERT_H