/* ============================================================================
 * alg_mahony.h — Mahony 互补滤波姿态解算
 * 说明 / 用法 / 坑 见同目录 alg_mahony.md
 * ========================================================================== */

#ifndef ALG_MAHONY_H
#define ALG_MAHONY_H

#include "math_support.h"

#include <math.h>

/**
 * @brief Reusable, Mahony 互补滤波器
 */
class Class_Mahony
{
public:
    /**
     * @brief 初始化
     * @param __Kp 加速度计修正的比例增益(参考起步值 1.0f)
     * @param __Ki 加速度计修正的积分增益(参考起步值 0.0f)
     * @param __dt 调用周期(s,固定值)
     */
    void Init(const fp32 __Kp, const fp32 __Ki, const fp32 __dt);

    /**
     * @brief 一条龙:喂数据 → 解算 → 算姿态角
     * @param __gx,__gy,__gz 陀螺(rad/s)
     * @param __ax,__ay,__az 加速度计(m/s²)
     * @note 等价参考的 mahony_input + mahony_update + mahony_output
     */
    void Update(const fp32 __gx, const fp32 __gy, const fp32 __gz,
                const fp32 __ax, const fp32 __ay, const fp32 __az);

    /// 对应参考的 mahony_input:只喂数据(陀螺 rad/s、加速度 m/s²)
    void Set_Input(const fp32 __gx, const fp32 __gy, const fp32 __gz,
                   const fp32 __ax, const fp32 __ay, const fp32 __az);

    /// 对应参考的 mahony_update:互补滤波 + 四元数积分 + 更新旋转矩阵
    void Calculate();

    /// 对应参考的 mahony_output:由旋转矩阵算 roll / pitch / yaw
    void Calculate_Attitude();

    /* ---------------- 输出(弧度,与参考一致) ---------------- */

    inline fp32 Get_Roll() const;
    inline fp32 Get_Pitch() const;
    inline fp32 Get_Yaw() const;

    /* ---------------- 输出(度,方便看数) ---------------- */

    inline fp32 Get_Roll_Deg() const;
    inline fp32 Get_Pitch_Deg() const;
    inline fp32 Get_Yaw_Deg() const;

    /// 四元数 [q0 q1 q2 q3]
    inline void Get_Quaternion(fp32 __q[4]) const;

    /// 旋转矩阵(机体→大地)的第 i 行第 j 列
    inline fp32 Get_Rotation_Matrix(const uint8_t &__i, const uint8_t &__j) const;

    /* ---------------- 参数 ---------------- */

    inline fp32 Get_Kp() const;
    inline fp32 Get_Ki() const;
    inline fp32 Get_Dt() const;

    inline void Set_Kp(const fp32 __Kp);
    inline void Set_Ki(const fp32 __Ki);
    inline void Set_Dt(const fp32 __dt);

protected:
    /* ---------------- 内部变量 ---------------- */

    // 参数
    fp32 Kp = 0.0f;
    fp32 Ki = 0.0f;
    fp32 dt = 0.0f;

    // 本拍输入
    fp32 Gyro[3] = {0.0f, 0.0f, 0.0f};
    fp32 Acc[3] = {0.0f, 0.0f, 0.0f};

    // 误差积分(参考里的 exInt/eyInt/ezInt)
    fp32 Ex_Int = 0.0f;
    fp32 Ey_Int = 0.0f;
    fp32 Ez_Int = 0.0f;

    // 四元数(机体→大地)
    fp32 q0 = 1.0f;
    fp32 q1 = 0.0f;
    fp32 q2 = 0.0f;
    fp32 q3 = 0.0f;

    // 旋转矩阵
    fp32 Rotation_Matrix[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    // 输出(弧度)
    fp32 Roll = 0.0f;
    fp32 Pitch = 0.0f;
    fp32 Yaw = 0.0f;

    /* ---------------- 内部函数 ---------------- */

    /// 对应参考的 RotationMatrix_update:由四元数刷新旋转矩阵
    void Rotation_Matrix_Update();

    /// 互补滤波 + 四元数积分(参考 mahony_update 的主体)
    void Update_Quaternion();
};

/* ============================================================================
 * inline 实现
 * ========================================================================== */

inline fp32 Class_Mahony::Get_Roll() const
{
    return Roll;
}

inline fp32 Class_Mahony::Get_Pitch() const
{
    return Pitch;
}

inline fp32 Class_Mahony::Get_Yaw() const
{
    return Yaw;
}

inline fp32 Class_Mahony::Get_Roll_Deg() const
{
    return Roll * 57.29578f;
}

inline fp32 Class_Mahony::Get_Pitch_Deg() const
{
    return Pitch * 57.29578f;
}

inline fp32 Class_Mahony::Get_Yaw_Deg() const
{
    return Yaw * 57.29578f;
}

inline void Class_Mahony::Get_Quaternion(fp32 __q[4]) const
{
    __q[0] = q0;
    __q[1] = q1;
    __q[2] = q2;
    __q[3] = q3;
}

inline fp32 Class_Mahony::Get_Rotation_Matrix(const uint8_t &__i, const uint8_t &__j) const
{
    return ((__i < 3u) && (__j < 3u)) ? Rotation_Matrix[__i][__j] : 0.0f;
}

inline fp32 Class_Mahony::Get_Kp() const
{
    return Kp;
}

inline fp32 Class_Mahony::Get_Ki() const
{
    return Ki;
}

inline fp32 Class_Mahony::Get_Dt() const
{
    return dt;
}

inline void Class_Mahony::Set_Kp(const fp32 __Kp)
{
    Kp = __Kp;
}

inline void Class_Mahony::Set_Ki(const fp32 __Ki)
{
    Ki = __Ki;
}

inline void Class_Mahony::Set_Dt(const fp32 __dt)
{
    dt = __dt;
}

#endif // ALG_MAHONY_H
