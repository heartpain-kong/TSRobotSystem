//
// alg_mahony.cpp — Mahony 互补滤波实现(移植自 chaiss/dm02 的 mahony_filter.c)
// 说明 / 用法 / 坑 见同目录 alg_mahony.md

#include "alg_mahony.h"

/**
 * @brief 初始化
 * @param __Kp 比例增益(参考起步值 1.0f)
 * @param __Ki 积分增益(参考起步值 0.0f)
 * @param __dt 调用周期(s)
 */
void Class_Mahony::Init(const fp32 __Kp, const fp32 __Ki, const fp32 __dt)
{
    Kp = __Kp;
    Ki = __Ki;
    dt = __dt;

    /* 单位四元数 ⇒ 初始姿态为"零姿态" */
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    Ex_Int = 0.0f;
    Ey_Int = 0.0f;
    Ez_Int = 0.0f;

    Gyro[0] = Gyro[1] = Gyro[2] = 0.0f;
    Acc[0] = Acc[1] = Acc[2] = 0.0f;

    Roll = 0.0f;
    Pitch = 0.0f;
    Yaw = 0.0f;

    /* 让旋转矩阵一开始就与四元数一致(参考要等第一拍 update 才刷) */
    Rotation_Matrix_Update();
}

/**
 * @brief 一条龙:喂数据 → 解算 → 姿态角
 */
void Class_Mahony::Update(const fp32 __gx, const fp32 __gy, const fp32 __gz,
                          const fp32 __ax, const fp32 __ay, const fp32 __az)
{
    Set_Input(__gx, __gy, __gz, __ax, __ay, __az);
    Calculate();
    Calculate_Attitude();
}

/**
 * @brief 喂数据(对应参考的 mahony_input)
 */
void Class_Mahony::Set_Input(const fp32 __gx, const fp32 __gy, const fp32 __gz,
                             const fp32 __ax, const fp32 __ay, const fp32 __az)
{
    Gyro[0] = __gx;
    Gyro[1] = __gy;
    Gyro[2] = __gz;

    Acc[0] = __ax;
    Acc[1] = __ay;
    Acc[2] = __az;
}

/**
 * @brief 互补滤波一拍(对应参考的 mahony_update)
 */
void Class_Mahony::Calculate()
{
    Update_Quaternion();
    Rotation_Matrix_Update();
}

/**
 * @brief 互补滤波 + 四元数积分(参考 mahony_update 的主体)
 */
void Class_Mahony::Update_Quaternion()
{
    fp32 normalise;
    fp32 ex, ey, ez;
    fp32 q0_last, q1_last, q2_last, q3_last;
    fp32 half_t;

    /* ---- 加速度计归一化 ---- */
    normalise = sqrtf(Acc[0] * Acc[0] + Acc[1] * Acc[1] + Acc[2] * Acc[2]);

    /* ★ 保护①:模长为 0 直接返回(参考没判 ⇒ 后面全是 NaN) */
    if (normalise <= 0.0f)
    {
        return;
    }

    Acc[0] /= normalise;
    Acc[1] /= normalise;
    Acc[2] /= normalise;

    /* ---- 加速度计方向 与 旋转矩阵重力方向 的叉积 = 误差 ---- */
    ex = (Acc[1] * Rotation_Matrix[2][2] - Acc[2] * Rotation_Matrix[2][1]);
    ey = (Acc[2] * Rotation_Matrix[2][0] - Acc[0] * Rotation_Matrix[2][2]);
    ez = (Acc[0] * Rotation_Matrix[2][1] - Acc[1] * Rotation_Matrix[2][0]);

    /* ---- 误差积分 ---- */
    Ex_Int += Ki * ex * dt;
    Ey_Int += Ki * ey * dt;
    Ez_Int += Ki * ez * dt;

    /* ---- PI 补偿,修正陀螺 ---- */
    Gyro[0] += Kp * ex + Ex_Int;
    Gyro[1] += Kp * ey + Ey_Int;
    Gyro[2] += Kp * ez + Ez_Int;

    /* ---- 一阶龙格-库塔更新四元数 ---- */
    q0_last = q0;
    q1_last = q1;
    q2_last = q2;
    q3_last = q3;
    half_t = dt * 0.5f;

    q0 += (-q1_last * Gyro[0] - q2_last * Gyro[1] - q3_last * Gyro[2]) * half_t;
    q1 += ( q0_last * Gyro[0] + q2_last * Gyro[2] - q3_last * Gyro[1]) * half_t;
    q2 += ( q0_last * Gyro[1] - q1_last * Gyro[2] + q3_last * Gyro[0]) * half_t;
    q3 += ( q0_last * Gyro[2] + q1_last * Gyro[1] - q2_last * Gyro[0]) * half_t;

    /* ---- 四元数归一化 ---- */
    normalise = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    /* ★ 保护②:模长为 0 就不归一(参考会除出 NaN) */
    if (normalise <= 0.0f)
    {
        return;
    }

    q0 /= normalise;
    q1 /= normalise;
    q2 /= normalise;
    q3 /= normalise;
}

/**
 * @brief 由四元数刷新旋转矩阵(对应参考的 RotationMatrix_update)
 */
void Class_Mahony::Rotation_Matrix_Update()
{
    fp32 q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q3 = q2 * q3;

    Rotation_Matrix[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    Rotation_Matrix[0][1] = 2.0f * (q1q2 - q0q3);
    Rotation_Matrix[0][2] = 2.0f * (q1q3 + q0q2);

    Rotation_Matrix[1][0] = 2.0f * (q1q2 + q0q3);
    Rotation_Matrix[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    Rotation_Matrix[1][2] = 2.0f * (q2q3 - q0q1);

    Rotation_Matrix[2][0] = 2.0f * (q1q3 - q0q2);
    Rotation_Matrix[2][1] = 2.0f * (q2q3 + q0q1);
    Rotation_Matrix[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

/**
 * @brief 由旋转矩阵算姿态角(对应参考的 mahony_output)
 * @note 单位:**弧度**(与参考一致);要度用 Get_*_Deg()
 */
void Class_Mahony::Calculate_Attitude()
{
    fp32 sin_pitch = Rotation_Matrix[2][0];

    /* ★ 保护④:夹到 [-1, 1],避免浮点误差让 asinf 出 NaN */
    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    Pitch = asinf(sin_pitch);
    Roll = atan2f(Rotation_Matrix[2][1], Rotation_Matrix[2][2]);
    Yaw = atan2f(Rotation_Matrix[1][0], Rotation_Matrix[0][0]);
}
