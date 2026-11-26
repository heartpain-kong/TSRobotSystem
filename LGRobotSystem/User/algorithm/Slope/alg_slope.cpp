/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化
 *
 * @param __Increase_Value 增长最大幅度
 * @param __Decrease_Value 降低最大幅度
 * @param __Slope_First 规划优先类型, 分为目标值优先和真实值优先
 */
void Class_Slope::Init(const float &__Increase_Value, const float &__Decrease_Value, const Enum_Slope_First &__Slope_First)
{
    Increase_Value = __Increase_Value;
    Decrease_Value = __Decrease_Value;
    Slope_First = __Slope_First;
}

/**
 * @brief 斜坡函数调整值, 计算周期取决于调用者
 *
 */
const fp32 Class_Slope::TIM_Calculate_PeriodElapsedCallback(const float &__Now_Real,const float &__Target){

    // 规划为当前真实值优先的额外逻辑
    if (Slope_First == Slope_First_REAL)
    {
        if ((__Target >= __Now_Real && __Now_Real >= Now_Planning) || (__Target <= __Now_Real && __Now_Real <= Now_Planning))
        {
            Out = Now_Real;
        }
    }

    if (Now_Planning > 0.0f)
    {
        if (__Target > Now_Planning)
        {
            // 正值加速
            if (Basic_Math_Abs(Now_Planning - __Target) > Increase_Value)
            {
                Out += Increase_Value;
            }
            else
            {
                Out = __Target;
            }
        }
        else if (__Target < Now_Planning)
        {
            // 正值减速
            if (Basic_Math_Abs(Now_Planning - __Target) > Decrease_Value)
            {
                Out -= Decrease_Value;
            }
            else
            {
                Out = __Target;
            }
        }
    }
    else if (Now_Planning < 0.0f)
    {
        if (__Target < Now_Planning)
        {
            // 负值加速
            if (Basic_Math_Abs(Now_Planning - __Target) > Increase_Value)
            {
                Out -= Increase_Value;
            }
            else
            {
                Out = __Target;
            }
        }
        else if (__Target > Now_Planning)
        {
            // 负值减速
            if (Basic_Math_Abs(Now_Planning - __Target) > Decrease_Value)
            {
                Out += Decrease_Value;
            }
            else
            {
                Out = __Target;
            }
        }
    }
    else
    {
        if (__Target > Now_Planning)
        {
            // 0值正加速
            if (Basic_Math_Abs(Now_Planning - __Target) > Increase_Value)
            {
                Out += Increase_Value;
            }
            else
            {
                Out = __Target;
            }
        }
        else if (__Target < Now_Planning)
        {
            // 0值负加速
            if (Basic_Math_Abs(Now_Planning - __Target) > Increase_Value)
            {
                Out -= Increase_Value;
            }
            else
            {
                Out = __Target;
            }
        }
    }

    // 善后工作
    Now_Planning = Out;
    return Out;

 }

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/