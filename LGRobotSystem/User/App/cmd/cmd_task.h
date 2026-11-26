/* ============================================================================
 * cmd_task.h — 指令任务:遥控器/键鼠 + 裁判系统 + 上位机 → 统一控制指令
 * 说明 / 用法 / 坑 见同目录 cmd_task.md
 * ========================================================================== */

#ifndef CMD_TASK_H
#define CMD_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 控制模式(由右拨杆 S1 决定;不满足条件时是 IDLE = 全零安全态)
 */
typedef enum
{
    CMD_MODE_IDLE = 0,      /* 空闲:遥控器掉线 / 拨杆打到 IDLE ⇒ 全零,机器人不动 */
    CMD_MODE_REMOTE,        /* 遥控:S1 上 */
    CMD_MODE_AUTO,          /* 自动:上位机/视觉 */
    CMD_MODE_TEST           /* 测试:标定/自检用 */
} Enum_Cmd_Mode;

/* 本拍指令的来源(位或) */
#define CMD_SRC_RC          (1u << 0)
#define CMD_SRC_KEYBOARD    (1u << 1)
#define CMD_SRC_HOST        (1u << 2)
#define CMD_SRC_REFEREE     (1u << 3)

/**
 * @brief 统一控制指令(发布在 MSG_TOPIC_CMD,chassis/gimbal/highRise 各取所需)
 */
typedef struct
{
    uint8_t Mode;                   /* Enum_Cmd_Mode */
    uint8_t Src;                    /* CMD_SRC_xxx 位或 */

    /* 底盘:物理量,由底盘任务换算成四轮转速 */
    float Chassis_Vx;               /* m/s,前为正 */
    float Chassis_Vy;               /* m/s,右为正 */
    float Chassis_W;                /* rad/s,逆时针为正 */
    bool Chassis_Spin;              /* 小陀螺 */
    uint16_t Chassis_Power_Limit;   /* 裁判系统给的底盘功率上限(W);0 = 没拿到 */
    uint16_t Chassis_Power_Buffer;  /* 底盘缓冲能量(J),来自 0x0202 */

    /* 云台:相对角(rad) */
    float Gimbal_Yaw;
    float Gimbal_Pitch;
    bool Gimbal_Enable;

    /* 发射 */
    bool Shoot_Enable;              /* 摩擦轮 */
    uint8_t Shoot_Rate;             /* 发弹档位:0 停,1 单发,2 连发 */

    /* 元信息 */
    uint32_t Seq;                   /* 每拍 +1 */
    bool Referee_Online;
    bool Referee_Shoot_Allowed;     /* 比赛允许发弹(裁判系统离线时按允许处理) */
    uint16_t Referee_Heat;          /* 枪口热量(转发给末端机构自己限速) */
    uint16_t Referee_Heat_Limit;    /* 热量上限(0 = 没拿到) */
} Cmd_Data_t;

/**
 * @brief 初始化(清状态)
 * @note 遥控器/裁判系统的**设备初始化**在 Robot_Init() 里做;本任务只负责每拍推进它们
 */
void Cmd_Init(void);

/**
 * @brief 跑一拍(100Hz,由 CMD 任务调用)
 *   Remote_Task() / Referee_Task() → 收 MSG_TOPIC_RC/REFEREE/HOST → 算指令 → 发 MSG_TOPIC_CMD
 */
void Cmd_Task(void);

/**
 * @brief 读当前指令
 */
const Cmd_Data_t *Cmd_Get(void);

/**
 * @brief 当前模式
 */
Enum_Cmd_Mode Cmd_Get_Mode(void);

/**
 * @brief 指令序号
 */
uint32_t Cmd_Get_Seq(void);

/**
 * @brief 模式名(日志/调试用)
 */
const char *Cmd_Get_Mode_Name(Enum_Cmd_Mode __mode);

#ifdef __cplusplus
}
#endif

#endif // CMD_TASK_H
