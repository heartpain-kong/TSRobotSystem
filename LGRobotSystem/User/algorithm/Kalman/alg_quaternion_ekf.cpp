//
// alg_quaternion_ekf.cpp — 四元数 EKF 实现(移植自 chaiss/dm02 的 quaternion_ekf.c)
// 说明 / 用法 / 坑 见同目录 Kalman.md

#include "alg_quaternion_ekf.h"

#include <string.h>

/* ============================================================================
 * 初始矩阵(对应参考里文件作用域的 IMU_QuaternionEKF_F / IMU_QuaternionEKF_P)
 *   F:单位阵(每拍 Update 会重新拷一遍,再把角速度填进去)
 *   P:对角 1、非对角 0.1
 * ========================================================================== */

static const float QUAT_EKF_F_INIT[36] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

static const float QUAT_EKF_P_INIT[36] = {
    1.0f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 1.0f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 1.0f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 1.0f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 1.0f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 1.0f};

/* ============================================================================
 * 生命周期
 * ========================================================================== */

/**
 * @brief 初始化
 * @param __process_noise1 四元数过程噪声 Q1
 * @param __process_noise2 陀螺零偏过程噪声 Q2
 * @param __measure_noise  加速度计量测噪声 R
 * @param __lambda         渐消记忆因子(>1 夹到 1)
 * @param __lpf            加速度计低通时间常数(s);**0 = 关闭低通**
 */
void Class_QuaternionEKF::Init(float __process_noise1, float __process_noise2, float __measure_noise,
                               float __lambda, float __lpf)
{
    /* 先让基类把 6 状态 / 0 输入 / 3 量测的矩阵建好并清零 */
    Class_Kalman::Init(6u, 0u, 3u);

    /* ---- 噪声参数 ---- */
    Q1 = __process_noise1;
    Q2 = __process_noise2;
    MeasureNoise = __measure_noise;

    if (__lambda > 1.0f)
    {
        __lambda = 1.0f;
    }
    lambda = __lambda;
    accLPFcoef = __lpf;

    /* ---- 状态标志与计数 ---- */
    ChiSquareTestThreshold = 1e-8f;
    ConvergeFlag = false;
    StableFlag = false;
    ErrorCount = 0u;
    UpdateCount = 0u;

    /* ---- 输出与中间量 ---- */
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    GyroBias[0] = 0.0f;
    GyroBias[1] = 0.0f;
    GyroBias[2] = 0.0f;
    Gyro[0] = Gyro[1] = Gyro[2] = 0.0f;
    Accel[0] = Accel[1] = Accel[2] = 0.0f;
    OrientationCosine[0] = OrientationCosine[1] = OrientationCosine[2] = 0.0f;
    gyro_norm = 0.0f;
    accl_norm = 0.0f;
    AdaptiveGainScale = 1.0f;
    dt = 0.0f;
    Roll = 0.0f;
    Pitch = 0.0f;
    Yaw = 0.0f;
    YawTotalAngle = 0.0f;
    YawRoundCount = 0;
    YawAngleLast = 0.0f;
    ChiSquare_Data[0] = 0.0f;

    /* ---- 1×1 的卡方矩阵(用自己的数据区) ---- */
    arm_mat_init_f32(&ChiSquare, 1u, 1u, ChiSquare_Data);

    /* ---- 初始四元数:单位四元数 ---- */
    xhat_data[0] = 1.0f;
    xhat_data[1] = 0.0f;
    xhat_data[2] = 0.0f;
    xhat_data[3] = 0.0f;

    /* ---- 通用 SetK / xhatUpdate 不用:EKF 在 xhatUpdate_Custom() 里自己算 ---- */
    SkipEq3 = true;
    SkipEq4 = true;

    /* ---- 初始 F / P ---- */
    memcpy(F_data, QUAT_EKF_F_INIT, sizeof(QUAT_EKF_F_INIT));
    memcpy(P_data, QUAT_EKF_P_INIT, sizeof(QUAT_EKF_P_INIT));

    Initialized = true;
}

/* ============================================================================
 * 单步
 * ========================================================================== */

/**
 * @brief 跑一拍
 */
void Class_QuaternionEKF::Update(float __gx, float __gy, float __gz,
                                 float __ax, float __ay, float __az, float __dt)
{
    float halfgxdt, halfgydt, halfgzdt;
    float accel_inv_norm;
    float gyro_inv_norm;

    if (!Initialized)
    {
        return;
    }

    dt = __dt;

    /* ---- 陀螺去零偏 ---- */
    Gyro[0] = __gx - GyroBias[0];
    Gyro[1] = __gy - GyroBias[1];
    Gyro[2] = __gz - GyroBias[2];

    halfgxdt = 0.5f * Gyro[0] * dt;
    halfgydt = 0.5f * Gyro[1] * dt;
    halfgzdt = 0.5f * Gyro[2] * dt;

    /* ---- F 每拍复位成单位阵,再填角速度项 ---- */
    memcpy(F_data, QUAT_EKF_F_INIT, sizeof(QUAT_EKF_F_INIT));

    F_data[1]  = -halfgxdt;
    F_data[2]  = -halfgydt;
    F_data[3]  = -halfgzdt;
    F_data[6]  =  halfgxdt;
    F_data[8]  =  halfgzdt;
    F_data[9]  = -halfgydt;
    F_data[12] =  halfgydt;
    F_data[13] = -halfgzdt;
    F_data[15] =  halfgxdt;
    F_data[18] =  halfgzdt;
    F_data[19] =  halfgydt;
    F_data[20] = -halfgxdt;

    /* ---- 加速度计一阶低通(首拍直接取输入) ---- */
    if (UpdateCount == 0u)
    {
        Accel[0] = __ax;
        Accel[1] = __ay;
        Accel[2] = __az;
    }

    Accel[0] = Accel[0] * accLPFcoef / (dt + accLPFcoef) + __ax * dt / (dt + accLPFcoef);
    Accel[1] = Accel[1] * accLPFcoef / (dt + accLPFcoef) + __ay * dt / (dt + accLPFcoef);
    Accel[2] = Accel[2] * accLPFcoef / (dt + accLPFcoef) + __az * dt / (dt + accLPFcoef);

    /* ---- 归一化加速度计 → 量测向量 ---- */
    accel_inv_norm = Inv_Sqrt(Accel[0] * Accel[0] + Accel[1] * Accel[1] + Accel[2] * Accel[2]);
    for (uint8_t i = 0u; i < 3u; i++)
    {
        MeasuredVector[i] = Accel[i] * accel_inv_norm;
    }

    /* ---- 陀螺/加速度模长(参考里名字叫 norm,其实是模长) ---- */
    gyro_inv_norm = Inv_Sqrt(Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2]);
    gyro_norm = (gyro_inv_norm > 0.0f) ? (1.0f / gyro_inv_norm) : 0.0f;
    accl_norm = (accel_inv_norm > 0.0f) ? (1.0f / accel_inv_norm) : 0.0f;

    /* ---- 静止可信判定:陀螺慢 + 加速度模长≈1g ---- */
    if ((gyro_norm < 0.3f) && (accl_norm > (9.8f - 0.5f)) && (accl_norm < (9.8f + 0.5f)))
    {
        StableFlag = true;
    }
    else
    {
        StableFlag = false;
    }

    /* ---- 过程噪声 Q(前 4 个状态用 Q1,后 2 个零偏用 Q2) ---- */
    Q_data[0]  = Q1 * dt;
    Q_data[7]  = Q1 * dt;
    Q_data[14] = Q1 * dt;
    Q_data[21] = Q1 * dt;
    Q_data[28] = Q2 * dt;
    Q_data[35] = Q2 * dt;

    /* ---- 量测噪声 R(3×3 对角) ---- */
    R_data[0] = MeasureNoise;
    R_data[4] = MeasureNoise;
    R_data[8] = MeasureNoise;

    /* ================== 与参考 Kalman_Filter_Update 相同的顺序 ================== */

    /* ① 取量测:z_data ← MeasuredVector */
    Measure();

    /* ② 先验状态 xhat⁻ = F·xhat */
    xhatMinusUpdate();

    /* ③ Hook_1:四元数归一 + F 对零偏的线性化 + P 渐消 */
    Linearization_And_P_Fading();

    /* ④ 先验协方差 P⁻ = F·P·Fᵀ + Q */
    PminusUpdate();

    /* ⑤ Hook_2:量测矩阵 H */
    Set_Measurement_Matrix_H();

    /* 注:参考这里还会调 SetK() 与 xhatUpdate(),但 EKF 把 SkipEq3/SkipEq4 置了 true,
           两步都是空转(它要在 Hook_3 里自己算 S/K/xhat)⇒ 本移植直接省掉这两次调用 */

    /* ⑥ Hook_3:卡方检验 + 自适应增益 + 自定义 K 与状态更新 */
    xhatUpdate_Custom();

    /* ⑦ 协方差更新(是否真更新由 Hook_3 里的 SkipEq5 决定) */
    P_Update();

    /* ⑧ 收尾:P 对角元下限 + FilteredValue ← xhat */
    Finalize();

    /* ---- ⑨ 输出 ---- */
    q[0] = FilteredValue[0];
    q[1] = FilteredValue[1];
    q[2] = FilteredValue[2];
    q[3] = FilteredValue[3];

    GyroBias[0] = FilteredValue[4];
    GyroBias[1] = FilteredValue[5];
    GyroBias[2] = 0.0f;      /* 零偏 z 不估计(参考同样写死 0) */

    /* 姿态角(度) —— 与参考逐字相同 */
    Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                   2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    Roll = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;

    /* 连续 Yaw:过 ±180 时按圈数续上(底盘的小陀螺零位就靠这个) */
    if ((Yaw - YawAngleLast) > 180.0f)
    {
        YawRoundCount--;
    }
    else if ((Yaw - YawAngleLast) < -180.0f)
    {
        YawRoundCount++;
    }

    YawTotalAngle = 360.0f * (float)YawRoundCount + Yaw;
    YawAngleLast = Yaw;

    UpdateCount++;
}

/* ============================================================================
 * 内部函数(对应参考的四个钩子)
 * ========================================================================== */

/**
 * @brief Hook_1:四元数归一 + F 线性化 + P 渐消
 * @note 参考里这几个局部量是 static,本移植改成普通局部量(行为一致,且可重入)
 */
void Class_QuaternionEKF::Linearization_And_P_Fading()
{
    float q0, q1, q2, q3, q_inv_norm;

    q0 = xhatminus_data[0];
    q1 = xhatminus_data[1];
    q2 = xhatminus_data[2];
    q3 = xhatminus_data[3];

    q_inv_norm = Inv_Sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (q_inv_norm > 0.0f)
    {
        for (uint8_t i = 0u; i < 4u; i++)
        {
            xhatminus_data[i] *= q_inv_norm;
        }
    }

    /* F 里"四元数对陀螺零偏"的偏导(线性化) */
    F_data[4]  =  q1 * dt / 2.0f;
    F_data[5]  =  q2 * dt / 2.0f;
    F_data[10] = -q0 * dt / 2.0f;
    F_data[11] =  q3 * dt / 2.0f;
    F_data[16] = -q3 * dt / 2.0f;
    F_data[17] = -q0 * dt / 2.0f;
    F_data[22] =  q2 * dt / 2.0f;
    F_data[23] = -q1 * dt / 2.0f;

    /* 渐消记忆:只对两个零偏状态做,并设上限防止发散 */
    P_data[28] /= lambda;
    P_data[35] /= lambda;

    if (P_data[28] > 10000.0f)
    {
        P_data[28] = 10000.0f;
    }
    if (P_data[35] > 10000.0f)
    {
        P_data[35] = 10000.0f;
    }
}

/**
 * @brief Hook_2:量测矩阵 H = ∂(预测重力方向)/∂x
 */
void Class_QuaternionEKF::Set_Measurement_Matrix_H()
{
    float doubleq0, doubleq1, doubleq2, doubleq3;

    doubleq0 = 2.0f * xhatminus_data[0];
    doubleq1 = 2.0f * xhatminus_data[1];
    doubleq2 = 2.0f * xhatminus_data[2];
    doubleq3 = 2.0f * xhatminus_data[3];

    memset(H_data, 0, sizeof(float) * (size_t)zSize * (size_t)xhatSize);

    H_data[0]  = -doubleq2;
    H_data[1]  =  doubleq3;
    H_data[2]  = -doubleq0;
    H_data[3]  =  doubleq1;
    H_data[6]  =  doubleq1;
    H_data[7]  =  doubleq0;
    H_data[8]  =  doubleq3;
    H_data[9]  =  doubleq2;
    H_data[12] =  doubleq0;
    H_data[13] = -doubleq1;
    H_data[14] = -doubleq2;
    H_data[15] =  doubleq3;
}

/**
 * @brief Hook_3:卡方检验 + 自适应增益 + 自定义增益 K 与状态更新
 * @note 逐行对应参考的 IMU_QuaternionEKF_xhatUpdate;含原作者"不修正 yaw 轴数据"
 *       的那一行(见 .md 第 4 节)
 */
void Class_QuaternionEKF::xhatUpdate_Custom()
{
    float q0, q1, q2, q3;

    /* ---- S = H·P⁻·Hᵀ + R,再求逆 ---- */
    MatStatus = (int8_t)arm_mat_trans_f32(&Matrix_H, &HT);

    temp_matrix.numRows = Matrix_H.numRows;
    temp_matrix.numCols = Pminus.numCols;
    MatStatus = (int8_t)arm_mat_mult_f32(&Matrix_H, &Pminus, &temp_matrix);

    temp_matrix1.numRows = temp_matrix.numRows;
    temp_matrix1.numCols = HT.numCols;
    MatStatus = (int8_t)arm_mat_mult_f32(&temp_matrix, &HT, &temp_matrix1);

    S.numRows = Matrix_R.numRows;
    S.numCols = Matrix_R.numCols;
    MatStatus = (int8_t)arm_mat_add_f32(&temp_matrix1, &Matrix_R, &S);

    MatStatus = (int8_t)arm_mat_inverse_f32(&S, &temp_matrix1);   /* temp_matrix1 ← S⁻¹ */

    /* ---- 预测的重力方向(量测模型的预测值) ---- */
    q0 = xhatminus_data[0];
    q1 = xhatminus_data[1];
    q2 = xhatminus_data[2];
    q3 = xhatminus_data[3];

    temp_vector.numRows = Matrix_H.numRows;
    temp_vector.numCols = 1;
    temp_vector_data[0] = 2.0f * (q1 * q3 - q0 * q2);
    temp_vector_data[1] = 2.0f * (q0 * q1 + q2 * q3);
    temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 量测方向与估计重力方向的夹角(后面用来压低零偏修正量) */
    for (uint8_t i = 0u; i < 3u; i++)
    {
        OrientationCosine[i] = acosf(fabsf(temp_vector_data[i]));
    }

    /* ---- 量测残差 ---- */
    temp_vector1.numRows = z.numRows;
    temp_vector1.numCols = 1;
    MatStatus = (int8_t)arm_mat_sub_f32(&z, &temp_vector, &temp_vector1);

    /* ---- 卡方统计量 χ² = resᵀ·S⁻¹·res ---- */
    temp_matrix.numRows = temp_vector1.numRows;
    temp_matrix.numCols = 1;
    MatStatus = (int8_t)arm_mat_mult_f32(&temp_matrix1, &temp_vector1, &temp_matrix);

    temp_vector.numRows = 1u;
    temp_vector.numCols = temp_vector1.numRows;
    MatStatus = (int8_t)arm_mat_trans_f32(&temp_vector1, &temp_vector);

    MatStatus = (int8_t)arm_mat_mult_f32(&temp_vector, &temp_matrix, &ChiSquare);

    if (ChiSquare_Data[0] < 0.5f * ChiSquareTestThreshold)
    {
        ConvergeFlag = true;
    }

    if ((ChiSquare_Data[0] > ChiSquareTestThreshold) && ConvergeFlag)
    {
        /* 量测明显不对(比如被撞、被抬起) */
        if (StableFlag)
        {
            ErrorCount++;
        }
        else
        {
            ErrorCount = 0u;
        }

        if (ErrorCount > 50u)
        {
            /* 连续 50 拍:认为真的出问题了,重新开始收敛 */
            ConvergeFlag = false;
            SkipEq5 = false;
        }
        else
        {
            /* 暂时不采纳这一拍量测:直接拿先验当后验,并跳过 P 更新 */
            memcpy(xhat_data, xhatminus_data, sizeof(float) * xhatSize);
            memcpy(P_data, Pminus_data, sizeof(float) * xhatSize * xhatSize);
            SkipEq5 = true;
            return;
        }
    }
    else
    {
        if ((ChiSquare_Data[0] > 0.1f * ChiSquareTestThreshold) && ConvergeFlag)
        {
            AdaptiveGainScale = (ChiSquareTestThreshold - ChiSquare_Data[0]) / (0.9f * ChiSquareTestThreshold);
        }
        else
        {
            AdaptiveGainScale = 1.0f;
        }

        ErrorCount = 0u;
        SkipEq5 = false;
    }

    /* ---- K = P⁻·Hᵀ·S⁻¹ ---- */
    temp_matrix.numRows = Pminus.numRows;
    temp_matrix.numCols = HT.numCols;
    MatStatus = (int8_t)arm_mat_mult_f32(&Pminus, &HT, &temp_matrix);

    MatStatus = (int8_t)arm_mat_mult_f32(&temp_matrix, &temp_matrix1, &K);

    /* 自适应缩放 */
    for (uint8_t i = 0u; i < (uint8_t)(K.numRows * K.numCols); i++)
    {
        K_data[i] *= AdaptiveGainScale;
    }

    /* 零偏两行再按"量测可信度"缩放(夹角越小越可信) */
    for (uint8_t i = 4u; i < 6u; i++)
    {
        for (uint8_t j = 0u; j < 3u; j++)
        {
            K_data[i * 3u + j] *= OrientationCosine[i - 4u] / 1.5707963f;
        }
    }

    /* ---- 状态修正量 Δx = K·res ---- */
    temp_vector.numRows = K.numRows;
    temp_vector.numCols = 1;
    MatStatus = (int8_t)arm_mat_mult_f32(&K, &temp_vector1, &temp_vector);

    /* 收敛后限制零偏每拍修正量(1e-2·dt),防止零偏跳变 */
    if (ConvergeFlag)
    {
        for (uint8_t i = 4u; i < 6u; i++)
        {
            if (temp_vector.pData[i] > 1e-2f * dt)
            {
                temp_vector.pData[i] = 1e-2f * dt;
            }
            if (temp_vector.pData[i] < -1e-2f * dt)
            {
                temp_vector.pData[i] = -1e-2f * dt;
            }
        }
    }

    /* ★ 原作者的注释:「不修正 yaw 轴数据」
       (RM 原版 EKF 里同一行、同一注释;两份代码完全一致,故照搬 —— 详见 .md 第 4 节) */
    temp_vector.pData[3] = 0.0f;

    MatStatus = (int8_t)arm_mat_add_f32(&xhatminus, &temp_vector, &xhat);
}

/* ============================================================================
 * 工具
 * ========================================================================== */

/**
 * @brief 快速平方根倒数(一次牛顿迭代)
 * @param __x 输入
 * @return 1/√x;__x <= 0 时返回 0(参考没判,会得到 NaN/Inf)
 * @note 参考用 long 做位转换,这里用 int32_t —— 保证在 PC 与 MCU 上都是 4 字节
 */
float Class_QuaternionEKF::Inv_Sqrt(float __x)
{
    float halfx, y;
    int32_t i;

    if (__x <= 0.0f)
    {
        return 0.0f;
    }

    halfx = 0.5f * __x;
    y = __x;

    i = *(int32_t *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}
