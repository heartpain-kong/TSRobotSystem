//
// xxx_task.cpp — 一句话说清这是什么任务(照 User/模板/task_template.cpp.tpl 改)
// 说明 / 用法 / 坑 见同目录 xxx_task.md
//

#include "xxx_task.h"

#include "cmd_task.h"           /* 需要哪层就 include 哪层:App 之间只通过话题通信 */
#include "bsp_timestamp.h"      /* 时间:超时/限速用 TIM5 */
#include "error_task.h"         /* 关键数据快照 + 错误位 */
#include "msg_center.h"         /* 收/发都走环形数组 */
#include "robot_def.h"          /* 电机分配等集中配置 */

#include <string.h>

static_assert(sizeof(Xxx_Data_t) <= MSG_SLOT_SIZE, "Xxx_Data_t 超过 MSG_SLOT_SIZE");

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Xxx_Data_t s_data;
static uint32_t s_cmd_seq = 0u;         /* 消费者各自记 seq(广播型话题) */
static bool s_inited = false;

/* ============================================================================
 * 对外接口
 * ========================================================================== */

void Xxx_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    s_cmd_seq = 0u;
    s_inited = true;
}

void Xxx_Task(void)
{
    Cmd_Data_t cmd;
    Error_Snapshot_t *snap;

    if (!s_inited)
    {
        return;
    }

    s_data.Seq++;

    /* ① 收:广播型话题标准写法(取最新、不弹出) */
    if (Msg_Is_New(MSG_TOPIC_CMD, s_cmd_seq) &&
        Msg_Latest(MSG_TOPIC_CMD, &cmd, (uint16_t)sizeof(cmd), &s_cmd_seq))
    {
        /* 拿到新指令 */
    }

    /* ② 算:你的逻辑(物理量用 SI;换算只在 motor_task 的 wheel_rpm 接口里做) */
    s_data.Value = 0.0f;

    /* ③ 写执行器(可选) */
    /* (void)motor_task_set_wheel_rpm(XXX_MOTOR_0, rpm, kd); */

    /* ④ 状态与异常 */
    s_data.Online = true;
    Error_Set_Flag(ERROR_FLAG_MASK(ERROR_FLAG_CUSTOM), !s_data.Online);

    /* ⑤ 发:数据有变化/每拍发都行,但别发无意义的重复(会白刷环形数组) */
    (void)Msg_Publish(MSG_TOPIC_XXX, &s_data, (uint16_t)sizeof(s_data));

    /* ⑥ 关键数据写进快照(它按 10Hz 落盘,事后复盘靠它) */
    snap = Error_Get_Snapshot();
    (void)snap;
}

const Xxx_Data_t *Xxx_Get(void)
{
    return &s_data;
}

bool Xxx_Is_Online(void)
{
    return s_data.Online;
}
