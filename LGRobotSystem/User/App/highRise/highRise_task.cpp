//
// highRise_task.cpp — 末端机构任务(默认:发射/摩擦轮+拨弹;预留:升降/抬升)
// 说明 / 用法 / 坑 见同目录 highRise_task.md
//

#include "highRise_task.h"

#include "chassis_task.h"       /* 流程图里的"chassis → highRise"(安全联锁用) */
#include "cmd_task.h"           /* 控制指令(含裁判系统的发弹门控) */
#include "gimbal_task.h"        /* 流程图里的"gimbal → highRise" */
#include "motor_task.h"         /* 电机目标 */
#include "msg_center.h"
#include "robot_def.h"
#include "cmd_485.h"          /* HIGHRISE_MECH + 电机分配 */

#include <string.h>

static_assert(sizeof(HighRise_Data_t) <= MSG_SLOT_SIZE, "HighRise_Data_t 超过 MSG_SLOT_SIZE");

/* ============================================================================
 * 参数(★ 按实车改)
 * ========================================================================== */

#if (HIGHRISE_MECH == HIGHRISE_MECH_SHOOT)

#define HIGHRISE_FRICTION_RPM       CFG_FRICTION_RPM   /* 摩擦轮目标转速(rpm) */
#define HIGHRISE_FRICTION_KD        1.0f
#define HIGHRISE_TRIGGER_KP         5.0f        /* 拨弹盘位置刚度 */
#define HIGHRISE_TRIGGER_KD         0.5f
#define HIGHRISE_TRIGGER_SLOTS      CFG_TRIGGER_SLOTS   /* 拨弹盘一圈几格(单发走一格) */
#define HIGHRISE_TRIGGER_RPM        CFG_TRIGGER_RPM     /* 连发时拨弹盘转速(rpm) ⇒ 射速 */
#define HIGHRISE_HEAT_MARGIN        20u         /* 热量留多少余量就不再发弹 */
#define HIGHRISE_ANTI_JAM_RAD       0.35f       /* 卡弹判定:目标与实测差多少算卡住 */

/* 拨弹盘走一格的角度 */
#define HIGHRISE_TRIGGER_STEP       (6.2831853f / (float)HIGHRISE_TRIGGER_SLOTS)

#elif (HIGHRISE_MECH == HIGHRISE_MECH_LIFT)

#define HIGHRISE_LIFT_KP            20.0f
#define HIGHRISE_LIFT_KD            1.0f
#define HIGHRISE_LIFT_MIN           (-1.0f)     /* 升降行程(rad,按丝杆导程换算) */
#define HIGHRISE_LIFT_MAX           (6.0f)
#define HIGHRISE_LIFT_RATE          0.002f      /* 每拍最大变化(rad/拍,斜坡限制) */

#else
#error "HIGHRISE_MECH 取值非法(见 robot_def.h)"
#endif

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static HighRise_Data_t s_data;
static Cmd_Data_t s_cmd;
static uint32_t s_cmd_seq = 0u;
static uint32_t s_chassis_seq = 0u;
static uint32_t s_gimbal_seq = 0u;
static bool s_last_shoot_enable = false;
static bool s_inited = false;

/* ============================================================================
 * 小工具
 * ========================================================================== */

static bool Motor_Ok(uint8_t __index)
{
    if (__index == 0xFFu)
    {
        return false;
    }
    if (__index >= motor_task_get_count())
    {
        return false;
    }
    return motor_task_is_init_ok(__index);
}

static float Clamp_F(float __v, float __min, float __max)
{
    if (__v > __max)
    {
        return __max;
    }
    if (__v < __min)
    {
        return __min;
    }
    return __v;
}

void HighRise_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd_seq = 0u;
    s_chassis_seq = 0u;
    s_gimbal_seq = 0u;
    s_last_shoot_enable = false;

    s_data.Mech = (uint8_t)HIGHRISE_MECH;

#if (HIGHRISE_MECH == HIGHRISE_MECH_SHOOT)
    if (Motor_Ok(HIGHRISE_MOTOR_TRIGGER))
    {
        s_data.Trigger_Pos = motor_task_get_now_pos(HIGHRISE_MOTOR_TRIGGER);
        s_data.Trigger_Target = s_data.Trigger_Pos;
    }
#else
    if (Motor_Ok(HIGHRISE_MOTOR_LIFT))
    {
        s_data.Lift_Now = motor_task_get_now_pos(HIGHRISE_MOTOR_LIFT);
        s_data.Lift_Target = s_data.Lift_Now;
    }
#endif

    s_inited = true;
}

void HighRise_Task(void)
{
    Cmd_Data_t cmd;
    Chassis_Data_t chas;
    Gimbal_Data_t gim;
    uint8_t i;
    uint8_t count = 0u;

    if (!s_inited)
    {
        return;
    }

    s_data.Seq++;

    /* ① 收指令 + 两条联锁线(chassis / gimbal) */
    if (Msg_Is_New(MSG_TOPIC_CMD, s_cmd_seq) &&
        Msg_Latest(MSG_TOPIC_CMD, &cmd, (uint16_t)sizeof(cmd), &s_cmd_seq))
    {
        s_cmd = cmd;
    }
    if (Msg_Is_New(MSG_TOPIC_CHASSIS, s_chassis_seq))
    {
        (void)Msg_Latest(MSG_TOPIC_CHASSIS, &chas, (uint16_t)sizeof(chas), &s_chassis_seq);
    }
    else
    {
        chas = *Chassis_Get();
    }
    if (Msg_Is_New(MSG_TOPIC_GIMBAL, s_gimbal_seq))
    {
        (void)Msg_Latest(MSG_TOPIC_GIMBAL, &gim, (uint16_t)sizeof(gim), &s_gimbal_seq);
    }
    else
    {
        gim = *Gimbal_Get();
    }

    s_data.Heat = s_cmd.Referee_Heat;
    s_data.Heat_Limit = s_cmd.Referee_Heat_Limit;
    s_data.Enable = s_cmd.Shoot_Enable;

    /* ② 发弹门控:指令使能 + 裁判系统允许 + 热量还有余量(热量上限为 0 时表示没数据,不拦) */
#if (ROBOT_RUN_MODE == ROBOT_RUN_COMPETITION)
    s_data.Heat_Blocked = (!s_cmd.Shoot_Enable) ||
                          (!s_cmd.Referee_Shoot_Allowed) ||
                          ((s_data.Heat_Limit != 0u) &&
                           (s_data.Heat + HIGHRISE_HEAT_MARGIN >= s_data.Heat_Limit));
#else
    /* 练习模式:没有裁判热量也允许调试,只尊重手动使能 */
    s_data.Heat_Blocked = (!s_cmd.Shoot_Enable);
#endif

#if (HIGHRISE_MECH == HIGHRISE_MECH_SHOOT)

    /* ③ 摩擦轮:两路纯速度控制(Shift 按下就转) */
    {
        const float frpm = s_cmd.Shoot_Enable ? HIGHRISE_FRICTION_RPM : 0.0f;

        for (i = 0u; i < HIGHRISE_FRICTION_N; i++)
        {
            s_data.Friction_Target[i] = frpm;
            s_data.Friction_Now[i] = 0.0f;
            motor_task_friction_set_rpm((uint8_t)i, frpm);
            count++;
        }
        s_data.Friction_Count = count;
    }

    /* ④ 拨弹盘:单发 = 走一格(位置),连发 = 定速(速度) */
    /* 拨弹盘 0x206:直接读 485 的上板拨杆状态,定速转 */
    {
        uint8_t up_en = 0u, up_rate = 0u;
        (void)Cmd_485_Get_Shoot(&up_en, &up_rate);

        /* 反向退弹:发射时拨盘反转 */
        const float trpm = ((up_en != 0u) && (up_rate != 0u))
                         ? HIGHRISE_TRIGGER_RPM : 0.0f;

        s_data.Trigger_Rate = up_rate;
        motor_task_trigger_set_rpm(trpm);
    }

#else   /* ---------------- 升降/抬升(预留) ---------------- */

    /* 目标来自指令的"发射档位"字段先借位:0 停 / 1 上 / 2 下(以后换成真正的升降指令) */
    {
        float step = 0.0f;
        if (s_cmd.Shoot_Rate == 1u)
        {
            step = HIGHRISE_LIFT_RATE;
        }
        else if (s_cmd.Shoot_Rate == 2u)
        {
            step = -HIGHRISE_LIFT_RATE;
        }
        s_data.Lift_Target = Clamp_F(s_data.Lift_Target + step, HIGHRISE_LIFT_MIN, HIGHRISE_LIFT_MAX);
    }

    if (Motor_Ok(HIGHRISE_MOTOR_LIFT))
    {
        s_data.Lift_Now = motor_task_get_now_pos(HIGHRISE_MOTOR_LIFT);
        s_data.Lift_Count = 1u;
        (void)motor_task_set_mit(HIGHRISE_MOTOR_LIFT, s_data.Lift_Target, 0.0f, 0.0f,
                                 HIGHRISE_LIFT_KP, HIGHRISE_LIFT_KD);
    }

#endif

    /* ⑤ 发布 */
    s_data.Online = (count > 0u) || (gim.Online);
    (void)Msg_Publish(MSG_TOPIC_HIGHRISE, &s_data, (uint16_t)sizeof(s_data));

    /* 安全联锁占位:底盘在动/云台异常时可以在这里禁止发弹(接口已留好) */
    (void)chas;
    (void)gim;
}

const HighRise_Data_t *HighRise_Get(void)
{
    return &s_data;
}

bool HighRise_Is_Online(void)
{
    return s_data.Online;
}

uint32_t HighRise_Get_Shot_Count(void)
{
    return s_data.Shot_Count;
}
