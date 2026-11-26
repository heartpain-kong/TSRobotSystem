/* ============================================================================
 * alg_math_matrix.h — 4×4 矩阵与 D-H 变换(从 math_support 细分出来)
 *
 * 说明:这一组目前**全工程没有调用点**(整套原样搬过来保留,以后做机械臂/正逆解可用)。
 * ★ 搬过来时修正了 math_matrix_DH / math_matrix_DH3x3 里 [0][1] 的笔误:
 *   按函数下方注释给出的公式,T[0][1] 应为 **-sin(theta)**,原来写成了 cos(theta)。
 * ========================================================================== */

#ifndef ALG_MATH_MATRIX_H
#define ALG_MATH_MATRIX_H

#include "struct_typedef.h"

#include <stdbool.h>

/** 逐元素比较两个 4×4 是否在误差 __Erroe 内一致 */
bool math_matrix_contrastl4x4(const fp32 __array_set1[4][4], const fp32 __array_set2[4][4], const fp32 __Erroe);

/** 4×4 复制 */
void math_matrix_equal4x4(const fp32 __array_set[4][4], fp32 __array_ret[4][4]);

/** 4×4 相乘:ret = a1 × a2 */
void math_matrix_multiplication4x4(const fp32 __array1[4][4], const fp32 __array2[4][4], fp32 __array_ret[4][4]);

/** D-H 变换矩阵(含 a、d) */
void math_matrix_DH(fp32 __array_ret[4][4], const fp32 __theta, const fp32 __alpha, const fp32 __a, const fp32 __d);

/** D-H 旋转部分(3×3,不含 a、d) */
void math_matrix_DH3x3(fp32 __array_ret[4][4], const fp32 __theta, const fp32 __alpha);

#endif // ALG_MATH_MATRIX_H