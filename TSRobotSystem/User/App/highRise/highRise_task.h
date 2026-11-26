/* ============================================================================
 * highRise_task.h — 末端机构任务(默认:发射/摩擦轮+拨弹;预留:升降/抬升)
 * 说明 / 用法 / 坑 见同目录 highRise_task.md
 * ========================================================================== */

#ifndef HIGHRISE_TASK_H
#define HIGHRISE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 机构类型(★ app 层按比赛选择:RC(ROBOCON) 与 RM 共用本系统,差异集中在这里) */
#define HIGHRISE_MECH_SHOOT     0u      /* 发射:2 个摩擦轮 + 1 个拨弹 */
#define HIGHRISE_MECH_LIFT      1u      /* 升降/抬升(接口已预留) */

#define HIGHRISE_FRICTION_N     2u      /* 摩擦轮数量(左右) */

/**
 * @brief 发布在 MSG_TOPIC_HIGHRISE 的末端机构数据
 */
typedef struct
{
    uint8_t Mech;                       /* 机构类型(HIGHRISE_MECH_xxx) */

    /* ---- 发射(HIGHRISE_MECH_SHOOT) ---- */
    float Friction_Target[HIGHRISE_FRICTION_N];     /* 摩擦轮目标转速(rpm) */
    float Friction_Now[HIGHRISE_FRICTION_N];        /* 摩擦轮实测(rpm) */
    float Trigger_Pos;                  /* 拨弹盘实测角(rad) */
    float Trigger_Target;               /* 拨弹盘目标角(rad) */
    uint8_t Trigger_Rate;               /* 0 停 / 1 单发 / 2 连发 */
    uint32_t Shot_Count;                /* 累计发弹数(拨弹盘走过的格数) */
    uint8_t Friction_Count;             /* 实际接了几路摩擦轮 */

    /* ---- 升降(预留,HIGHRISE_MECH_LIFT) ---- */
    float Lift_Target;                  /* 目标位置(rad,丝杆/升降电机) */
    float Lift_Now;                     /* 实测位置(rad) */
    uint8_t Lift_Count;                 /* 实际接了几路升降电机 */

    /* ---- 门控与状态 ---- */
    uint16_t Heat;                      /* 枪口热量(转发自裁判系统,经 cmd) */
    uint16_t Heat_Limit;                /* 热量上限 */
    bool Heat_Blocked;                  /* 因热量/门控被禁止发弹 */
    bool Enable;                        /* 指令使能 */
    bool Online;                        /* 至少一路电机可用 */
    uint32_t Seq;
} HighRise_Data_t;

/**
 * @brief 初始化(机构类型 + 记录拨弹盘初始角)
 */
void HighRise_Init(void);

/**
 * @brief 跑一拍(500Hz,由 HIGHRISE 任务调用)
 *   收 CMD(+CHASSIS/GIMBAL 做安全联锁)→ 摩擦轮/拨弹(或升降)→ 发布 HIGHRISE
 */
void HighRise_Task(void);

/**
 * @brief 读末端机构数据
 */
const HighRise_Data_t *HighRise_Get(void);

/**
 * @brief 是否就绪
 */
bool HighRise_Is_Online(void);

/**
 * @brief 累计发弹数
 */
uint32_t HighRise_Get_Shot_Count(void);

#ifdef __cplusplus
}
#endif

#endif // HIGHRISE_TASK_H
