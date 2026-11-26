/* ============================================================================
 * chassis_task.h — 底盘任务(麦轮/舵轮:指令 + IMU → 四轮目标 + 里程计)
 * 说明 / 用法 / 坑 见同目录 chassis_task.md
 * ========================================================================== */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_UNIT_N      4u      /* 底盘单元数(麦轮/舵轮都是 4;按实车改) */

/**
 * @brief 发布在 MSG_TOPIC_CHASSIS 的底盘数据
 *        单位:速度 m/s、角速度 rad/s、轮速 rpm、舵向 rad
 */
typedef struct
{
    float Vx;                   /* 里程计:车体前后速度(m/s) */
    float Vy;                   /* 里程计:车体左右速度(m/s) */
    float W;                    /* 里程计:车体旋转角速度(rad/s) */
    float Wheel_Speed[CHASSIS_UNIT_N];      /* 各轮**目标**转速(rpm) */
    float Wheel_Now_Speed[CHASSIS_UNIT_N];  /* 各轮**实测**转速(rpm) */
    float Wheel_Seat[CHASSIS_UNIT_N];       /* 舵轮:各轮转向目标(rad);麦轮恒 0 */
    float Yaw_Total;            /* 本拍用的连续 yaw(rad) */
    float Top_Yaw;              /* 小陀螺零位(rad) */
    float Power_Est;            /* 估计功率(W;由实测力矩×转速估) */
    float Power_Est_Current;    /* C620 电流上限估计(W,默认关闭) */
    uint16_t Power_Limit;       /* 裁判系统给的功率上限(W;0 = 没限制) */
    uint8_t Mode;               /* 当前控制模式(转发自 CMD) */
    uint8_t Unit_Count;         /* 实际接了几路电机 */
    bool Spin;                  /* 小陀螺 */
    bool Online;                /* IMU 在线且至少一台电机可用 */
    uint32_t Seq;
} Chassis_Data_t;

/**
 * @brief 初始化底盘解算(几何参数 + 麦轮装配表)
 * @note 电机初始化在 Robot_Init() 里(motor_task_init);本函数只初始化解算层
 */
void Chassis_Init(void);

/**
 * @brief 跑一拍(500Hz,由 CHASSIS 任务调用)
 *   收 CMD + IMU_CHASSIS → 解算 → 写电机目标 → 正解里程计 → 发布 CHASSIS
 */
void Chassis_Task(void);

/**
 * @brief 读底盘数据
 */
const Chassis_Data_t *Chassis_Get(void);

/**
 * @brief 是否就绪(IMU 在线 + 有电机可用)
 */
bool Chassis_Is_Online(void);

/**
 * @brief 某一路的目标轮速(rpm)
 */
float Chassis_Get_Wheel_Rpm(uint8_t __unit);

/** 舵向实测角(0=202,1=204),rad,已映射到"轮子朝向"坐标系 */
float Chassis_Get_Seat_Now(uint8_t __unit);
/** 舵向闭环目标角(0=202,1=204),rad */
float Chassis_Get_Seat_Target(uint8_t __unit);
/** 航向闭环本次用的误差 [rad](调试遥测;超过 8° 会把目标重锁到当前朝向,之后恒为 0) */
float Chassis_Get_Heading_Error(void);
/** 本拍真正送进运动学的 ω [rad/s](含航向修正;两轮 toe-in 就是它造成的) */
float Chassis_Get_Yaw_Rate_Cmd(void);

/**
 * @brief 手动把小陀螺零位对齐到当前 yaw(调试用;正常由 Spin 上升沿自动对齐)
 */
void Chassis_Set_Top_Yaw_Now(void);

/**
 * @brief 小陀螺时用遥控器改云台朝向 [rad/s](正的 = 世界系逆时针)
 * @note  预留接口:cmd 层拿到遥控器 yaw 杆量后调它即可;0 = 不改朝向
 */
void Chassis_Set_Spin_Yaw_Rate(float __rate);

/**
 * @brief 舵轮找零状态(调试帧用):0=IDLE 1=WAIT 2=SEARCH 3=DONE 4=ERROR,0xFF=未启用
 */
uint8_t Chassis_Homing_State(uint8_t __unit);

/**
 * @brief 舵轮找零当前扫描角 [rad](正=正向扫,负=反向兜底扫)
 */
float Chassis_Homing_Scan(uint8_t __unit);

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_TASK_H
