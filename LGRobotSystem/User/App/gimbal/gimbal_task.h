/* ============================================================================
 * gimbal_task.h — 云台任务(yaw/pitch 关节角控制 + 小陀螺前馈)
 * 说明 / 用法 / 坑 见同目录 gimbal_task.md
 * ========================================================================== */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_AXIS_YAW     0u
#define GIMBAL_AXIS_PITCH   1u
#define GIMBAL_AXIS_N       2u

/**
 * @brief 发布在 MSG_TOPIC_GIMBAL 的云台数据(角度一律 rad,关节侧)
 */
typedef struct
{
    float Target[GIMBAL_AXIS_N];    /* 关节目标角:[0]=yaw [1]=pitch */
    float Now[GIMBAL_AXIS_N];       /* 关节实测角(电机反馈) */
    float Imu_Yaw_Total;            /* 云台 imu 连续 yaw(rad) */
    float Imu_Pitch;                /* 云台 imu pitch(rad) */
    float Spin_Comp;                /* 小陀螺前馈补偿量(rad) */
    float Kp;                       /* 当前用的刚度(调试用) */
    float Kd;
    uint8_t Axis_Count;             /* 实际接了几路电机(0 = 没接,只发状态) */
    bool Enable;                    /* 指令里的使能位 */
    bool Online;                    /* IMU 在线 */
    uint32_t Seq;
} Gimbal_Data_t;

/**
 * @brief 初始化(清零 + 记录初始关节角)
 */
void Gimbal_Init(void);

/**
 * @brief 跑一拍(500Hz,由 GIMBAL 任务调用)
 *   收 CMD + IMU_GIMBAL + CHASSIS → 算目标角 → 写 MIT 位置目标 → 发布 GIMBAL
 */
void Gimbal_Task(void);

/**
 * @brief 读云台数据
 */
const Gimbal_Data_t *Gimbal_Get(void);

/**
 * @brief 是否就绪
 */
bool Gimbal_Is_Online(void);

/**
 * @brief 取某轴目标角(rad)
 */
float Gimbal_Get_Target(uint8_t __axis);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_TASK_H
