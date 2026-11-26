/* ============================================================================
 * imu_task.h — 姿态解算任务(BMI088 + 四元数 EKF)
 * 说明 / 用法 / 坑 见同目录 imu_task.md
 * ========================================================================== */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 通道:0 = 底盘 imu,1 = 云台 imu
   本板只有一颗 BMI088(SPI2,PC0/PC3 双片选)⇒ 通道 1 目前镜像通道 0,见 .md */
#define IMU_CHANNEL_CHASSIS     0u
#define IMU_CHANNEL_GIMBAL      1u
#define IMU_CHANNEL_NUM         2u

/**
 * @brief 发布在 MSG_TOPIC_IMU_CHASSIS / MSG_TOPIC_IMU_GIMBAL 的姿态数据
 *        单位一律 SI:角度 rad、角速度 rad/s、加速度 m/s²
 */
typedef struct
{
    float Euler[3];         /* roll, pitch, yaw(rad;yaw ∈ -π~π) */
    float Yaw_Total;        /* 连续 yaw(rad,不跳变)—— 小陀螺/云台用这个 */
    float Gyro[3];          /* rad/s(原始,未去零偏) */
    float Accel[3];         /* m/s² */
    float Quat[4];          /* 四元数 w,x,y,z */
    float Gyro_Bias[3];     /* EKF 估出来的陀螺零偏(rad/s) */
    float Temp_C;           /* 温度(℃) */
    float ChiSquare;        /* 卡方值:越大说明模型/测量越不一致 */
    uint32_t Update_Count;  /* EKF 跑了几拍 */
    uint32_t Error_Count;   /* 卡方门控拒了几次 */
    uint32_t Seq;           /* 每拍 +1 */
    bool Online;            /* BMI088 在线且 EKF 在跑 */
    bool Converged;         /* EKF 是否已收敛 */
} Imu_Data_t;

/**
 * @brief 初始化姿态解算(EKF 参数 + 状态清零)
 * @note BMI088 的**设备初始化**在 Robot_Init() 里做;本函数只初始化 EKF
 */
void Imu_Init(void);

/**
 * @brief 跑一拍(1kHz,由 IMU 任务调用)
 *   BMI088_Task() → 读角速度/加速度 → EKF 融合 → 发布两个话题 + 写 error 快照
 */
void Imu_Task(void);

/**
 * @brief 读某个通道的姿态数据
 * @param __channel IMU_CHANNEL_CHASSIS / IMU_CHANNEL_GIMBAL
 */
const Imu_Data_t *Imu_Get(uint8_t __channel);

/**
 * @brief 把当前 pitch 角记为 0(云台 4310 机械零点对齐用;上板云台 pitch 在启动时调用)
 */
void Imu_Zero_Pitch_Now(void);

/**
 * @brief 该通道是否在线
 */
bool Imu_Is_Online(uint8_t __channel);

/**
 * @brief 连续 yaw(rad)—— 给底盘小陀螺/云台用,不受 ±π 跳变影响
 */
float Imu_Get_Yaw_Total(uint8_t __channel);

/**
 * @brief EKF 是否已收敛
 */
bool Imu_Is_Converged(uint8_t __channel);

#ifdef __cplusplus
}
#endif

#endif // IMU_TASK_H
