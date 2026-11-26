//
// alg_kalman.cpp — 通用卡尔曼滤波框架(移植自 chaiss/dm02 的 kalman_filter.c)
// 说明 / 用法 / 坑 见同目录 Kalman.md

#include "alg_kalman.h"

#include <string.h>

/* ============================================================================
 * 生命周期
 * ========================================================================== */

/**
 * @brief 初始化:把内部矩阵句柄绑到定长数据区上,并清零
 * @param __xhatSize 状态维度(<= KALMAN_MAX_XHAT_SIZE)
 * @param __uSize    输入维度(可 0)
 * @param __zSize    量测维度(<= KALMAN_MAX_Z_SIZE)
 */
void Class_Kalman::Init(uint8_t __xhatSize, uint8_t __uSize, uint8_t __zSize)
{
    xhatSize = __xhatSize;
    uSize = __uSize;
    zSize = __zSize;

    MeasurementValidNum = 0u;
    MatStatus = 0;

    /* ---- 各数据区清零 ---- */
    memset(FilteredValue, 0, sizeof(FilteredValue));
    memset(temp, 0, sizeof(temp));

    memset(xhat_data, 0, sizeof(xhat_data));
    memset(xhatminus_data, 0, sizeof(xhatminus_data));
    memset(u_data, 0, sizeof(u_data));
    memset(z_data, 0, sizeof(z_data));

    memset(P_data, 0, sizeof(P_data));
    memset(Pminus_data, 0, sizeof(Pminus_data));
    memset(F_data, 0, sizeof(F_data));
    memset(FT_data, 0, sizeof(FT_data));
    memset(B_data, 0, sizeof(B_data));
    memset(H_data, 0, sizeof(H_data));
    memset(HT_data, 0, sizeof(HT_data));
    memset(Q_data, 0, sizeof(Q_data));
    memset(R_data, 0, sizeof(R_data));
    memset(K_data, 0, sizeof(K_data));

    memset(S_data, 0, sizeof(S_data));
    memset(temp_matrix_data, 0, sizeof(temp_matrix_data));
    memset(temp_matrix_data1, 0, sizeof(temp_matrix_data1));
    memset(temp_vector_data, 0, sizeof(temp_vector_data));
    memset(temp_vector_data1, 0, sizeof(temp_vector_data1));

    memset(MeasuredVector, 0, sizeof(MeasuredVector));
    memset(ControlVector, 0, sizeof(ControlVector));
    memset(MeasurementMap, 0, sizeof(MeasurementMap));
    memset(MeasurementDegree, 0, sizeof(MeasurementDegree));
    memset(MatR_DiagonalElements, 0, sizeof(MatR_DiagonalElements));
    memset(StateMinVariance, 0, sizeof(StateMinVariance));

    /* ---- 矩阵句柄绑定到定长存储 ---- */
    arm_mat_init_f32(&xhat, xhatSize, 1, xhat_data);
    arm_mat_init_f32(&xhatminus, xhatSize, 1, xhatminus_data);

    if (uSize != 0u)
    {
        arm_mat_init_f32(&u, uSize, 1, u_data);
    }

    arm_mat_init_f32(&z, zSize, 1, z_data);

    arm_mat_init_f32(&Matrix_P, xhatSize, xhatSize, P_data);
    arm_mat_init_f32(&Pminus, xhatSize, xhatSize, Pminus_data);

    arm_mat_init_f32(&Matrix_F, xhatSize, xhatSize, F_data);
    arm_mat_init_f32(&FT, xhatSize, xhatSize, FT_data);

    if (uSize != 0u)
    {
        arm_mat_init_f32(&Matrix_B, xhatSize, uSize, B_data);
    }

    arm_mat_init_f32(&Matrix_H, zSize, xhatSize, H_data);
    arm_mat_init_f32(&HT, xhatSize, zSize, HT_data);

    arm_mat_init_f32(&Matrix_Q, xhatSize, xhatSize, Q_data);
    arm_mat_init_f32(&Matrix_R, zSize, zSize, R_data);
    arm_mat_init_f32(&K, xhatSize, zSize, K_data);

    arm_mat_init_f32(&S, xhatSize, xhatSize, S_data);
    arm_mat_init_f32(&temp_matrix, xhatSize, xhatSize, temp_matrix_data);
    arm_mat_init_f32(&temp_matrix1, xhatSize, xhatSize, temp_matrix_data1);
    arm_mat_init_f32(&temp_vector, xhatSize, 1, temp_vector_data);
    arm_mat_init_f32(&temp_vector1, xhatSize, 1, temp_vector_data1);

    /* ---- 跳过标志归零(调用方按需再置) ---- */
    SkipEq1 = false;
    SkipEq2 = false;
    SkipEq3 = false;
    SkipEq4 = false;
    SkipEq5 = false;
}

/* ============================================================================
 * 单步
 * ========================================================================== */

/**
 * @brief 取量测与控制:MeasuredVector → z_data,ControlVector → u_data
 * @note 参考实现在这里把 MeasuredVector/ControlVector **清零** ⇒ 调用方必须
 *       **每拍先填、再调本函数**(填一次只生效一拍)
 */
void Class_Kalman::Measure()
{
    if (UseAutoAdjustment)
    {
        H_KR_Adjustment();
    }
    else
    {
        memcpy(z_data, MeasuredVector, sizeof(float) * zSize);
        memset(MeasuredVector, 0, sizeof(float) * zSize);
    }

    memcpy(u_data, ControlVector, sizeof(float) * uSize);
}

/**
 * @brief 先验状态:xhat⁻ = F·xhat + B·u(uSize 为 0 时退化成 F·xhat)
 */
void Class_Kalman::xhatMinusUpdate()
{
    if (SkipEq1)
    {
        return;
    }

    if (uSize > 0u)
    {
        temp_vector.numRows = xhatSize;
        temp_vector.numCols = 1;
        MatStatus = (int8_t)arm_mat_mult_f32(&Matrix_F, &xhat, &temp_vector);

        temp_vector1.numRows = xhatSize;
        temp_vector1.numCols = 1;
        MatStatus = (int8_t)arm_mat_mult_f32(&Matrix_B, &u, &temp_vector1);

        MatStatus = (int8_t)arm_mat_add_f32(&temp_vector, &temp_vector1, &xhatminus);
    }
    else
    {
        MatStatus = (int8_t)arm_mat_mult_f32(&Matrix_F, &xhat, &xhatminus);
    }
}

/**
 * @brief 先验协方差:P⁻ = F·P·Fᵀ + Q
 */
void Class_Kalman::PminusUpdate()
{
    if (SkipEq2)
    {
        return;
    }

    MatStatus = (int8_t)arm_mat_trans_f32(&Matrix_F, &FT);
    MatStatus = (int8_t)arm_mat_mult_f32(&Matrix_F, &Matrix_P, &Pminus);

    temp_matrix.numRows = Pminus.numRows;
    temp_matrix.numCols = FT.numCols;
    MatStatus = (int8_t)arm_mat_mult_f32(&Pminus, &FT, &temp_matrix);

    MatStatus = (int8_t)arm_mat_add_f32(&temp_matrix, &Matrix_Q, &Pminus);
}

/**
 * @brief 通用卡尔曼增益:K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
 */
void Class_Kalman::SetK()
{
    if (SkipEq3)
    {
        return;
    }

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

    MatStatus = (int8_t)arm_mat_inverse_f32(&S, &temp_matrix1);

    temp_matrix.numRows = Pminus.numRows;
    temp_matrix.numCols = HT.numCols;
    MatStatus = (int8_t)arm_mat_mult_f32(&Pminus, &HT, &temp_matrix);

    MatStatus = (int8_t)arm_mat_mult_f32(&temp_matrix, &temp_matrix1, &K);
}

/**
 * @brief 通用状态更新:xhat = xhat⁻ + K·(z - H·xhat⁻)
 */
void Class_Kalman::xhatUpdate()
{
    if (SkipEq4)
    {
        return;
    }

    temp_vector.numRows = Matrix_H.numRows;
    temp_vector.numCols = 1;
    MatStatus = (int8_t)arm_mat_mult_f32(&Matrix_H, &xhatminus, &temp_vector);

    temp_vector1.numRows = z.numRows;
    temp_vector1.numCols = 1;
    MatStatus = (int8_t)arm_mat_sub_f32(&z, &temp_vector, &temp_vector1);

    temp_vector.numRows = K.numRows;
    temp_vector.numCols = 1;
    MatStatus = (int8_t)arm_mat_mult_f32(&K, &temp_vector1, &temp_vector);

    MatStatus = (int8_t)arm_mat_add_f32(&xhatminus, &temp_vector, &xhat);
}

/**
 * @brief 通用协方差更新:P = P⁻ - K·H·P⁻
 */
void Class_Kalman::P_Update()
{
    if (SkipEq5)
    {
        return;
    }

    temp_matrix.numRows = K.numRows;
    temp_matrix.numCols = Matrix_H.numCols;
    temp_matrix1.numRows = temp_matrix.numRows;
    temp_matrix1.numCols = Pminus.numCols;
    MatStatus = (int8_t)arm_mat_mult_f32(&K, &Matrix_H, &temp_matrix);

    MatStatus = (int8_t)arm_mat_mult_f32(&temp_matrix, &Pminus, &temp_matrix1);

    MatStatus = (int8_t)arm_mat_sub_f32(&Pminus, &temp_matrix1, &Matrix_P);
}

/**
 * @brief 收尾:P 对角元下限保护 + 把 xhat 拷进 FilteredValue
 */
void Class_Kalman::Finalize()
{
    for (uint8_t i = 0u; i < xhatSize; i++)
    {
        if (P_data[i * xhatSize + i] < StateMinVariance[i])
        {
            P_data[i * xhatSize + i] = StateMinVariance[i];
        }
    }

    memcpy(FilteredValue, xhat_data, sizeof(float) * xhatSize);
}

/**
 * @brief 定常线性 KF 的一条龙流程(等价参考的 Kalman_Filter_Update)
 * @return FilteredValue
 */
float *Class_Kalman::Update()
{
    Measure();

    xhatMinusUpdate();
    PminusUpdate();

    if ((MeasurementValidNum != 0u) || (!UseAutoAdjustment))
    {
        SetK();
        xhatUpdate();
        P_Update();
    }
    else
    {
        /* 本拍一个有效量测都没有:直接拿先验当后验 */
        memcpy(xhat_data, xhatminus_data, sizeof(float) * xhatSize);
        memcpy(P_data, Pminus_data, sizeof(float) * xhatSize * xhatSize);
    }

    Finalize();

    return FilteredValue;
}

/* ============================================================================
 * 内部函数
 * ========================================================================== */

/**
 * @brief 自适应量测:把"等于 0 的量测"当成无效剔掉,顺带缩小 H/R/K/z
 * @note 与参考的 H_K_R_Adjustment 完全相同(含 MeasurementMap 从 1 开始数这一点)
 */
void Class_Kalman::H_KR_Adjustment()
{
    MeasurementValidNum = 0u;

    memcpy(z_data, MeasuredVector, sizeof(float) * zSize);
    memset(MeasuredVector, 0, sizeof(float) * zSize);

    memset(R_data, 0, sizeof(float) * zSize * zSize);
    memset(H_data, 0, sizeof(float) * xhatSize * zSize);

    for (uint8_t i = 0u; i < zSize; i++)
    {
        if (z_data[i] != 0.0f)
        {
            z_data[MeasurementValidNum] = z_data[i];
            temp[MeasurementValidNum] = i;
            H_data[xhatSize * MeasurementValidNum + MeasurementMap[i] - 1u] = MeasurementDegree[i];
            MeasurementValidNum++;
        }
    }

    for (uint8_t i = 0u; i < MeasurementValidNum; i++)
    {
        R_data[i * MeasurementValidNum + i] = MatR_DiagonalElements[temp[i]];
    }

    Matrix_H.numRows = MeasurementValidNum;
    Matrix_H.numCols = xhatSize;
    HT.numRows = xhatSize;
    HT.numCols = MeasurementValidNum;
    Matrix_R.numRows = MeasurementValidNum;
    Matrix_R.numCols = MeasurementValidNum;
    K.numRows = xhatSize;
    K.numCols = MeasurementValidNum;
    z.numRows = MeasurementValidNum;
}
