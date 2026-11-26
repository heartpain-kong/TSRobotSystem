/* ============================================================================
 * dev_referee.h — 裁判系统(USART1:115200 8N1)
 * 说明 / 用法 / 坑 见同目录 dev_referee.md
 * ========================================================================== */

#ifndef DEV_REFEREE_H
#define DEV_REFEREE_H

#include "usart.h"      /* UART_HandleTypeDef / huart1 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 协议常量 ---------------- */
#define REFEREE_SOF             0xA5u   /* 帧头 */
#define REFEREE_MIN_FRAME       9u      /* 最短帧(无数据段) */
#define REFEREE_MAX_DATA        127u    /* 单帧最大数据段(缓冲 136 已够) */
#define REFEREE_LOST_TIMEOUT_MS 1000u   /* 1s 没帧 = 离线(裁判系统 10Hz 左右) */

/* 关心的指令 ID */
#define REFEREE_ID_GAME_STATUS       0x0001u    /* 比赛状态 */
#define REFEREE_ID_ROBOT_STATUS      0x0201u    /* 机器人状态(血量/热量/弹量/功率上限) */
#define REFEREE_ID_POWER_HEAT        0x0202u    /* 当前热量 / 底盘缓冲能量 */
#define REFEREE_ID_ROBOT_HP          0x0003u    /* 机器人血量 */
#define REFEREE_ID_HURT              0x0206u    /* 受击事件 */
#define REFEREE_ID_SHOOT             0x0207u    /* 发弹/弹速 */
#define REFEREE_ID_ALLOWANCE         0x0208u    /* 剩余发弹额度 */
#define REFEREE_ID_EVENT_DATA        0x0101u    /* 场地事件 */

/* ---------------- 数据 ---------------- */
typedef struct
{
    /* 比赛状态(0x0001) */
    uint8_t Game_Type;              /* 比赛类型 */
    uint8_t Game_Progress;          /* 0 未开始 1 准备 2 自检 3 倒计时 4 比赛中 5 结算 */
    uint16_t Stage_Remain_S;        /* 当前阶段剩余时间(s) */

    /* 我方机器人状态(0x0201) */
    uint8_t Robot_Id;
    uint8_t Robot_Level;
    uint16_t Hp;
    uint16_t Max_Hp;
    uint16_t Heat;                  /* 枪口热量 */
    uint16_t Heat_Limit;            /* 热量上限 */
    uint16_t Barrel_Cooling;        /* 冷却值 */
    uint16_t Chassis_Power_Limit;   /* 底盘功率上限(W)—— 底盘任务用它限功率 */
    float    Shooter_Speed_Limit;   /* 射击初速度上限(m/s) */
    uint8_t  Power_Management;      /* 电源管理输出位图 */
    uint8_t Power_Gimbal_Out;       /* 功率管理:云台输出 */
    uint8_t Power_Chassis_Out;      /* 功率管理:底盘输出 */
    uint8_t Power_Shooter_Out;      /* 功率管理:发射输出 */

    /* 0x0202 功率/热量反馈 */
    uint16_t Chassis_Power_Buffer;  /* 底盘缓冲能量(J) */
    float    Chassis_Power;         /* 底盘实际功率(W) */

    /* 0x0206 受击 / 0x0207 发弹 / 0x0208 发弹额度 */
    uint8_t  Armor_Id;
    uint8_t  Hurt_Reason;
    uint16_t Hurt_Seq;
    uint16_t Projectile_Allowance;
    float    Initial_Speed;
    uint32_t Heat_Update_Ms;
    uint32_t Power_Update_Ms;

    /* 通信状态 */
    bool Online;
    uint32_t Last_Rx_Ms;
    uint32_t Rx_Frames;             /* 校验通过并解出的帧数 */
    uint32_t Crc8_Errors;
    uint32_t Crc16_Errors;
    uint32_t Reject_Count;          /* 帧长/帧头不对 */
    uint32_t Lost_Count;
} Referee_Data_t;

/**
 * @brief 初始化:注册 UART 接收回调
 * @param __huart 串口句柄;传 NULL 用默认 &huart1
 * @return true = 成功
 */
bool Referee_Init(UART_HandleTypeDef *__huart);

/**
 * @brief 周期跑(建议 100Hz):掉线检测 + 有新数据就发布到 MSG_TOPIC_REFEREE
 */
void Referee_Task(void);

/**
 * @brief 读当前裁判系统数据
 */
const Referee_Data_t *Referee_Get(void);

/**
 * @brief 是否在线
 */
bool Referee_Is_Online(void);

/**
 * @brief 比赛是否进行中(Game_Progress == 4)
 */
bool Referee_Is_In_Match(void);

/**
 * @brief 当前阶段剩余时间(s)
 */
uint16_t Referee_Get_Stage_Remain(void);

#ifdef __cplusplus
}
#endif

#endif // DEV_REFEREE_H
