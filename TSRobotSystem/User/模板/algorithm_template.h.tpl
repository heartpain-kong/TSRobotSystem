/* ============================================================================
 * alg_xxx.h — 一句话说清这是什么算法(纯数学:不碰硬件)
 * 说明 / 用法 / 坑 见同目录 alg_xxx.md
 * ❌ 禁止:#include 任何 App/Device/Bsp 的头(算法层不许反向依赖);放 static 实例
 *   参数一律由调用方传进来(来自 robot_def.h),算法里不写死几何/限幅 ✓
 *   红线见 User/代码架构框架.md §5.3 / §10.1
 * ★ 用法:复制到 User/algorithm/<模块>/ → 去掉 .tpl → 全文替换 xxx/Xxx
 * ========================================================================== */

#ifndef ALG_XXX_H
#define ALG_XXX_H

#include "math_support.h"       /* 常量/限幅/角度工具;需要矩阵再加 arm_math.h */
#include "struct_typedef.h"     /* fp32 / fp64 */

/**
 * @brief 纯函数形式:输入 → 输出(无状态算法用这种)
 * @param __input 输入(单位写清,如 rad)
 * @return 输出(单位写清)
 */
fp32 Alg_Xxx_Calc(fp32 __input);

/* ============================================================================
 * 有状态的算法用类(实例放在使用它的 App 模块里,算法层不放实例)
 * ========================================================================== */

class Class_Xxx
{
public:
    /**
     * @brief 初始化
     * @param __kp 刚度(含义/单位/推荐值写清)
     */
    void Init(fp32 __kp);

    /**
     * @brief 算一拍
     * @param __target 目标(rad)
     * @param __now    实测(rad)
     * @return 输出
     */
    fp32 Cout(fp32 __target, fp32 __now);

    fp32 Get_Output() const;

protected:
    /* ---- 初始化相关常量 ---- */
    fp32 kp = 0.0f;
    /* ---- 内部变量 ---- */
    fp32 output = 0.0f;
};

/* ---- 一行 getter/setter 一律 inline 放这里 ---- */
inline fp32 Class_Xxx::Get_Output() const { return output; }

#endif // ALG_XXX_H
