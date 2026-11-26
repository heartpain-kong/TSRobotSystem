/* ============================================================================
 * alg_quaternion_ekf.h — 四元数扩展卡尔曼滤波(姿态解算)
 * 说明 / 用法 / 坑 见同目录 Kalman.md
 * ========================================================================== */

#ifndef ALG_QUATERNION_EKF_H
#define ALG_QUATERNION_EKF_H

#include "alg_kalman.h"

#include <stdbool.h>

class Class_QuaternionEKF : public Class_Kalman
{
public:
    /**
     * @brief 初始化(维度固定 6 状态 / 0 输入 / 3 量测)
     * @param __process_noise1 四元数过程噪声 Q1
     * @param __process_noise2 陀螺零偏过程噪声 Q2
     * @param __measure_noise  加速度计量测噪声 R
     * @param __lambda         渐消记忆因子(>1 夹到 1)
     * @param __lpf            加速度计低通时间常数(s)
     */
    void Init(float __process_noise1, float __process_noise2, float __measure_noise, float __lambda, float __lpf);

    /**
     * @brief 跑一拍(必须先 Init)
     * @param __gx,__gy,__gz 陀螺(rad/s)
     * @param __ax,__ay,__az 加速度计(m/s²)
     * @param __dt           本拍周期(s)
     * @note 建议 1kHz 调用;dt 用实测值(本工程可用 DWT_GetDeltaT 那套)
     */
    void Update(float __gx, float __gy, float __gz, float __ax, float __ay, float __az, float __dt);

    /* ---------------- 输出(度:与参考一致) ---------------- */

    inline float Get_Roll() const;
    inline float Get_Pitch() const;
    /// 偏航角,**[-180, 180] 度**(过 ±180 会跳)
    inline float Get_Yaw() const;
    /// 偏航角累计值:过了 ±180 会按圈数续上(**连续,不跳**)
    inline float Get_YawTotalAngle() const;

    /* ---------------- 输出(弧度:本工程常用) ---------------- */

    inline float Get_Roll_Rad() const;
    inline float Get_Pitch_Rad() const;
    inline float Get_Yaw_Rad() const;
    inline float Get_YawTotalAngle_Rad() const;

    /// 四元数 [q0 q1 q2 q3](机体→大地)
    inline void Get_Quaternion(float __q[4]) const;
    /// 陀螺零偏估计 [b_x b_y b_z](z 恒为 0)
    inline void Get_GyroBias(float __bias[3]) const;

    /* ---------------- 状态/调试量 ---------------- */

    /// 是否已初始化
    inline bool Is_Initialized() const;
    /// 是否已收敛(卡方检验通过过)
    inline bool Is_Converged() const;
    /// 本拍是否处于"静止可信"状态(陀螺慢 + 加速度模长接近 9.8)
    inline bool Is_Stable() const;
    /// 校验失败累计次数
    inline uint64_t Get_ErrorCount() const;
    /// 累计运行拍数
    inline uint64_t Get_UpdateCount() const;
    /// 本拍卡方统计量(越大越不可信)
    inline float Get_ChiSquare() const;
    /// 本拍自适应增益缩放(1 = 完全相信量测)
    inline float Get_AdaptiveGainScale() const;

protected:
    /* ---------------- 内部变量(对应参考的 QEKF_INS) ---------------- */

    // 状态标志
    bool Initialized = false;
    bool ConvergeFlag = false;
    bool StableFlag = false;

    // 计数
    uint64_t ErrorCount = 0u;
    uint64_t UpdateCount = 0u;

    // 输出:四元数与陀螺零偏
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float GyroBias[3] = {0.0f, 0.0f, 0.0f};

    // 本拍输入
    float Gyro[3] = {0.0f, 0.0f, 0.0f};
    float Accel[3] = {0.0f, 0.0f, 0.0f};

    // 量测可信度(OrientationCosine:量测方向与估计重力方向的夹角)
    float OrientationCosine[3] = {0.0f, 0.0f, 0.0f};

    // 参数
    float Q1 = 0.0f;                  // 四元数过程噪声
    float Q2 = 0.0f;                  // 陀螺零偏过程噪声
    float MeasureNoise = 0.0f;        // 加速度计量测噪声(参考里叫 R)
    float lambda = 1.0f;              // 渐消记忆因子
    float accLPFcoef = 0.0f;          // 加速度计低通时间常数(s)

    // 中间量
    float gyro_norm = 0.0f;
    float accl_norm = 0.0f;
    float AdaptiveGainScale = 1.0f;
    float dt = 0.0f;

    // 卡方检验
    arm_matrix_instance_f32 ChiSquare;
    float ChiSquare_Data[1] = {0.0f};
    float ChiSquareTestThreshold = 1e-8f;

    // 输出(度)
    float Roll = 0.0f;
    float Pitch = 0.0f;
    float Yaw = 0.0f;
    float YawTotalAngle = 0.0f;

    // 连续 Yaw 的圈数统计
    int16_t YawRoundCount = 0;
    float YawAngleLast = 0.0f;

    /* ---------------- 内部函数(对应参考的 4 个钩子) ---------------- */

    /// 对应 Hook_1:四元数归一 + F 线性化 + P 渐消(参考 IMU_QuaternionEKF_F_Linearization_P_Fading)
    void Linearization_And_P_Fading();

    /// 对应 Hook_2:量测矩阵 H(参考 IMU_QuaternionEKF_SetH)
    void Set_Measurement_Matrix_H();

    /// 对应 Hook_3:卡方检验 + 自适应增益 + 自定义增益/状态更新(参考 IMU_QuaternionEKF_xhatUpdate)
    void xhatUpdate_Custom();

    /// 快速平方根倒数(参考的 invSqrt:一次牛顿迭代)
    static float Inv_Sqrt(float __x);
};

/* ============================================================================
 * inline 实现
 * ========================================================================== */

inline float Class_QuaternionEKF::Get_Roll() const
{
    return Roll;
}

inline float Class_QuaternionEKF::Get_Pitch() const
{
    return Pitch;
}

inline float Class_QuaternionEKF::Get_Yaw() const
{
    return Yaw;
}

inline float Class_QuaternionEKF::Get_YawTotalAngle() const
{
    return YawTotalAngle;
}

inline float Class_QuaternionEKF::Get_Roll_Rad() const
{
    return Roll * 0.01745329252f;
}

inline float Class_QuaternionEKF::Get_Pitch_Rad() const
{
    return Pitch * 0.01745329252f;
}

inline float Class_QuaternionEKF::Get_Yaw_Rad() const
{
    return Yaw * 0.01745329252f;
}

inline float Class_QuaternionEKF::Get_YawTotalAngle_Rad() const
{
    return YawTotalAngle * 0.01745329252f;
}

inline void Class_QuaternionEKF::Get_Quaternion(float __q[4]) const
{
    __q[0] = q[0];
    __q[1] = q[1];
    __q[2] = q[2];
    __q[3] = q[3];
}

inline void Class_QuaternionEKF::Get_GyroBias(float __bias[3]) const
{
    __bias[0] = GyroBias[0];
    __bias[1] = GyroBias[1];
    __bias[2] = GyroBias[2];
}

inline bool Class_QuaternionEKF::Is_Initialized() const
{
    return Initialized;
}

inline bool Class_QuaternionEKF::Is_Converged() const
{
    return ConvergeFlag;
}

inline bool Class_QuaternionEKF::Is_Stable() const
{
    return StableFlag;
}

inline uint64_t Class_QuaternionEKF::Get_ErrorCount() const
{
    return ErrorCount;
}

inline uint64_t Class_QuaternionEKF::Get_UpdateCount() const
{
    return UpdateCount;
}

inline float Class_QuaternionEKF::Get_ChiSquare() const
{
    return ChiSquare_Data[0];
}

inline float Class_QuaternionEKF::Get_AdaptiveGainScale() const
{
    return AdaptiveGainScale;
}

#endif // ALG_QUATERNION_EKF_H
