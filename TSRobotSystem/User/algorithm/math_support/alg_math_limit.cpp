//
// alg_math_limit.cpp — 限幅
//

#include "alg_math_limit.h"

void LimitMax(fp32 &input, fp32 max)
{
    /* max <= 0 视为"不限制":以前会把输出压成 0,和"0 为不限制"的注释矛盾 */
    if (max <= 0.0f)
    {
        return;
    }

    if (input > max)
    {
        input = max;
    }
    else if (input < -max)
    {
        input = -max;
    }
}

void LimitMinMax(fp32 &input, fp32 max, fp32 min)
{
    if (max <= min)
    {
        return;
    }

    if (input > max)
    {
        input = max;
    }
    else if (input < min)
    {
        input = min;
    }
}