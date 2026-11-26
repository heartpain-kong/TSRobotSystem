//
// alg_math_matrix.cpp — 4×4 矩阵与 D-H 变换
//

#include "alg_math_matrix.h"
#include "math_support.h"      // arm_math(arm_sin_f32/arm_cos_f32)

void math_matrix_equal4x4(const fp32 __array_set[4][4], fp32 __array_ret[4][4])
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            __array_ret[i][j] = __array_set[i][j];
        }
    }
}

bool math_matrix_contrastl4x4(const fp32 __array_set1[4][4], const fp32 __array_set2[4][4], const fp32 __Erroe)
{
    bool __bl = 0;

    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            if (__array_set1[i][j] >= __array_set2[i][j] - __Erroe &&
                __array_set1[i][j] <= __array_set2[i][j] + __Erroe)
            {
                __bl = 1;
            }
            else
            {
                return 0;
            }
        }
    }
    return __bl;
}

void math_matrix_multiplication4x4(const fp32 __array1[4][4], const fp32 __array2[4][4], fp32 __array_ret[4][4])
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            __array_ret[i][j] = 0;
            for (uint8_t v = 0; v < 4; ++v)
            {
                __array_ret[i][j] += __array1[i][v] * __array2[v][j];
            }
        }
    }
}

//
// D-H 变换矩阵
// T = [cos(theta), -sin(theta),          0,          a;
//      sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
//      sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
//      0, 0, 0, 1];
//
void math_matrix_DH(fp32 __array_ret[4][4], const fp32 __theta, const fp32 __alpha, const fp32 __a, const fp32 __d)
{
    __array_ret[0][0] = arm_cos_f32(__theta);
    __array_ret[0][1] = -arm_sin_f32(__theta);          /* ★ 原来是 cos(theta),按上面公式应为 -sin */
    __array_ret[0][2] = 0;
    __array_ret[0][3] = __a;

    __array_ret[1][0] = arm_sin_f32(__theta) * arm_cos_f32(__alpha);
    __array_ret[1][1] = arm_cos_f32(__theta) * arm_cos_f32(__alpha);
    __array_ret[1][2] = -arm_sin_f32(__alpha);
    __array_ret[1][3] = -__d * arm_sin_f32(__alpha);

    __array_ret[2][0] = arm_sin_f32(__theta) * arm_sin_f32(__alpha);
    __array_ret[2][1] = arm_cos_f32(__theta) * arm_sin_f32(__alpha);
    __array_ret[2][2] = arm_cos_f32(__alpha);
    __array_ret[2][3] = __d * arm_cos_f32(__alpha);

    __array_ret[3][0] = 0;
    __array_ret[3][1] = 0;
    __array_ret[3][2] = 0;
    __array_ret[3][3] = 1;
}

void math_matrix_DH3x3(fp32 __array_ret[4][4], const fp32 __theta, const fp32 __alpha)
{
    __array_ret[0][0] = arm_cos_f32(__theta);
    __array_ret[0][1] = -arm_sin_f32(__theta);          /* ★ 同上 */

    __array_ret[1][0] = arm_sin_f32(__theta) * arm_cos_f32(__alpha);
    __array_ret[1][1] = arm_cos_f32(__theta) * arm_cos_f32(__alpha);

    __array_ret[2][0] = arm_sin_f32(__theta) * arm_sin_f32(__alpha);
    __array_ret[2][1] = arm_cos_f32(__theta) * arm_sin_f32(__alpha);
}