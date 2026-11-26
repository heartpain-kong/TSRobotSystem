/* ============================================================================
 * alg_kalman.h — 通用卡尔曼滤波框架
 * 说明 / 用法 / 坑 见同目录 Kalman.md
 * ========================================================================== */

#ifndef ALG_KALMAN_H
#define ALG_KALMAN_H

#include "math_support.h"

#include <stdbool.h>

/* ---- 维度上限(定长存储;按需改大)----
 * 注:内部临时向量按 XHAT 上限分配,又被当作 1×zSize 的行向量用过
 *     ⇒ 请保证 XHAT 上限 >= Z 上限(EKF 是 8 >= 6,满足) */
#define KALMAN_MAX_XHAT_SIZE    8u
#define KALMAN_MAX_U_SIZE       4u
#define KALMAN_MAX_Z_SIZE       6u

/**
 * @brief Reusable, 通用卡尔曼滤波框架
 *
 * 状态/输入/量测三个维度在 Init() 时给出(运行期可小于上限);
 * 噪声矩阵、量测映射等都在 public 区,**和参考实现一样直接改**。
 */
class Class_Kalman
{
public:
    /* ---------------- 需要自己配的矩阵/向量 ---------------- */

    // 状态转移矩阵 xhat×xhat
    arm_matrix_instance_f32 Matrix_F;
    // 控制矩阵 xhat×u(uSize == 0 时不使用)
    arm_matrix_instance_f32 Matrix_B;
    // 量测矩阵 z×xhat
    arm_matrix_instance_f32 Matrix_H;
    // 过程噪声协方差 xhat×xhat
    arm_matrix_instance_f32 Matrix_Q;
    // 量测噪声协方差 z×z
    arm_matrix_instance_f32 Matrix_R;
    // 误差协方差 xhat×xhat
    arm_matrix_instance_f32 Matrix_P;

    // 每拍由调用方填:量测向量(传感器)
    float MeasuredVector[KALMAN_MAX_Z_SIZE];
    // 每拍由调用方填:控制向量
    float ControlVector[KALMAN_MAX_U_SIZE];

    /* ---------------- 自适应量测(可选) ----------------
     * UseAutoAdjustment = true 时:量测向量里**等于 0 的项视为"本拍无效"**,
     * H/R/K/z 会自动缩小到只剩有效项(参考实现里的 H_K_R_Adjustment)。 */
    bool    UseAutoAdjustment = false;
    // 第 i 个量测对应第几个状态(**从 1 开始数**,0 表示不映射)
    uint8_t MeasurementMap[KALMAN_MAX_Z_SIZE];
    // 第 i 个量测在 H 里的系数
    float   MeasurementDegree[KALMAN_MAX_Z_SIZE];
    // 各量测的 R 对角元
    float   MatR_DiagonalElements[KALMAN_MAX_Z_SIZE];
    // P 对角元下限,防止协方差被压塌(逐状态)
    float   StateMinVariance[KALMAN_MAX_XHAT_SIZE];

    /* ---------------- 跳过某一步(与参考同名同义) ---------------- */
    bool SkipEq1 = false;   // 跳过 xhat 先验估计
    bool SkipEq2 = false;   // 跳过 P 先验估计
    bool SkipEq3 = false;   // 跳过通用 SetK()   —— 自己算 K 时置 true
    bool SkipEq4 = false;   // 跳过通用 xhatUpdate() —— 同上
    bool SkipEq5 = false;   // 跳过通用 P_Update()   —— 同上

    /* ---------------- 生命周期 ---------------- */

    /**
     * @brief 初始化(分配内部矩阵到定长存储上)
     * @param __xhatSize 状态维度(<= KALMAN_MAX_XHAT_SIZE)
     * @param __uSize    输入维度(可为 0;<= KALMAN_MAX_U_SIZE)
     * @param __zSize    量测维度(<= KALMAN_MAX_Z_SIZE)
     */
    void Init(uint8_t __xhatSize, uint8_t __uSize, uint8_t __zSize);

    /* ---------------- 单步(按参考的函数一一对应) ---------------- */

    /// 取量测/控制(UseAutoAdjustment 时做自适应缩放)
    void Measure();

    /// 先验状态:xhat⁻ = F·xhat + B·u
    void xhatMinusUpdate();

    /// 先验协方差:P⁻ = F·P·Fᵀ + Q
    void PminusUpdate();

    /// 通用卡尔曼增益:K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
    void SetK();

    /// 通用状态更新:xhat = xhat⁻ + K·(z - H·xhat⁻)
    void xhatUpdate();

    /// 通用协方差更新:P = P⁻ - K·H·P⁻
    void P_Update();

    /// 收尾:P 对角元下限保护 + 把 xhat 拷进 FilteredValue
    void Finalize();

    /**
     * @brief 定常线性 KF 的一条龙:Measure → 先验 → SetK → 更新 → P → Finalize
     * @return 滤波结果指针(长度 = xhatSize)
     * @note 时变/非线性场合**别用这个**,按上面"两种用法②"自己调各步
     */
    float *Update();

    /* ---------------- 只读观察(调试用) ---------------- */

    inline uint8_t Get_XhatSize() const;
    inline uint8_t Get_USize() const;
    inline uint8_t Get_ZSize() const;

    /// 滤波结果(长度 = xhatSize)
    inline float *Get_FilteredValue();
    inline const float *Get_FilteredValue() const;

    /// 第 i 个状态的滤波值
    inline float Get_State(const uint8_t &__index) const;

    /// 本拍实际参与运算的量测个数(UseAutoAdjustment 时才有意义)
    inline uint8_t Get_MeasurementValidNum() const;

    /// 最近一次矩阵运算的返回码(arm_status;0 = ARM_MATH_SUCCESS)
    inline int8_t Get_MatStatus() const;

    /// 调试:看 P(误差协方差)矩阵本体
    inline arm_matrix_instance_f32 *Get_Matrix_P();
    /// 调试:看 K(卡尔曼增益)矩阵本体
    inline arm_matrix_instance_f32 *Get_Matrix_K();

protected:
    /* ---------------- 内部变量 ---------------- */

    uint8_t xhatSize = 0u;
    uint8_t uSize = 0u;
    uint8_t zSize = 0u;

    // 本拍有效量测个数
    uint8_t MeasurementValidNum = 0u;
    // 矩阵运算返回码(调试用)
    int8_t  MatStatus = 0;

    // 输出:滤波结果
    float FilteredValue[KALMAN_MAX_XHAT_SIZE];

    // 自适应量测里"第 i 个有效量测原本是第几个"的暂存
    uint8_t temp[KALMAN_MAX_Z_SIZE];

    /* ---------------- 内部矩阵的数据区 ---------------- */

    float xhat_data[KALMAN_MAX_XHAT_SIZE];
    float xhatminus_data[KALMAN_MAX_XHAT_SIZE];
    float u_data[KALMAN_MAX_U_SIZE];
    float z_data[KALMAN_MAX_Z_SIZE];

    float P_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_XHAT_SIZE];
    float Pminus_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_XHAT_SIZE];
    float F_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_XHAT_SIZE];
    float FT_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_XHAT_SIZE];
    float B_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_U_SIZE];
    float H_data[KALMAN_MAX_Z_SIZE * KALMAN_MAX_XHAT_SIZE];
    float HT_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_Z_SIZE];
    float Q_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_XHAT_SIZE];
    float R_data[KALMAN_MAX_Z_SIZE * KALMAN_MAX_Z_SIZE];
    float K_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_Z_SIZE];

    float S_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_XHAT_SIZE];
    float temp_matrix_data[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_XHAT_SIZE];
    float temp_matrix_data1[KALMAN_MAX_XHAT_SIZE * KALMAN_MAX_XHAT_SIZE];
    float temp_vector_data[KALMAN_MAX_XHAT_SIZE];
    float temp_vector_data1[KALMAN_MAX_XHAT_SIZE];

    /* ---------------- 内部矩阵句柄 ---------------- */

    arm_matrix_instance_f32 xhat;         // 后验状态
    arm_matrix_instance_f32 xhatminus;    // 先验状态
    arm_matrix_instance_f32 u;            // 控制输入
    arm_matrix_instance_f32 z;            // 量测
    arm_matrix_instance_f32 Pminus;       // 先验协方差
    arm_matrix_instance_f32 FT;           // F 的转置
    arm_matrix_instance_f32 HT;           // H 的转置
    arm_matrix_instance_f32 K;            // 卡尔曼增益
    arm_matrix_instance_f32 S;            // 中间量(H·P⁻·Hᵀ + R)
    arm_matrix_instance_f32 temp_matrix;  // 暂存
    arm_matrix_instance_f32 temp_matrix1; // 暂存
    arm_matrix_instance_f32 temp_vector;  // 暂存
    arm_matrix_instance_f32 temp_vector1; // 暂存

    /* ---------------- 内部函数 ---------------- */

    /// 自适应量测:把无效量测剔掉,缩小 H/R/K/z
    void H_KR_Adjustment();
};

/* ============================================================================
 * inline 实现(一行式,放文件底部 —— 与 alg_pid.h / alg_slope.h 同一个风格)
 * ========================================================================== */

inline uint8_t Class_Kalman::Get_XhatSize() const
{
    return xhatSize;
}

inline uint8_t Class_Kalman::Get_USize() const
{
    return uSize;
}

inline uint8_t Class_Kalman::Get_ZSize() const
{
    return zSize;
}

inline float *Class_Kalman::Get_FilteredValue()
{
    return FilteredValue;
}

inline const float *Class_Kalman::Get_FilteredValue() const
{
    return FilteredValue;
}

inline float Class_Kalman::Get_State(const uint8_t &__index) const
{
    return (__index < xhatSize) ? FilteredValue[__index] : 0.0f;
}

inline uint8_t Class_Kalman::Get_MeasurementValidNum() const
{
    return MeasurementValidNum;
}

inline int8_t Class_Kalman::Get_MatStatus() const
{
    return MatStatus;
}

inline arm_matrix_instance_f32 *Class_Kalman::Get_Matrix_P()
{
    return &Matrix_P;
}

inline arm_matrix_instance_f32 *Class_Kalman::Get_Matrix_K()
{
    return &K;
}

#endif // ALG_KALMAN_H
