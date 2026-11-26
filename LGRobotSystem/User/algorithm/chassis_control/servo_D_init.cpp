//
// servo_D_init.cpp — 舵轮光电找零(位置单方向遍历)
//

#include "servo_D_init.h"

#include "bsp_timestamp.h"
#include "math_support.h"

#include <math.h>

#define SERVO_D_STEP_RAD        (0.10f * PI / 180.0f)  /* 0.1° / 2ms = 50°/s */
#define SERVO_D_STEP_MS         2u                     /* 每 2ms 走一步 */

/* 只认 0→1(上升沿)结算;1→0 跳过继续扫。
   第一遍(正向)最多 1.25 圈没找到 0→1,就反向再扫一遍,这一遍接受任意边沿
   (0→1 用 zero_on_rise,1→0 用 zero_on_fall),避免某一路光电只有下降沿时
   一直扫到超时、整车不动。 */
#define SERVO_D_RISE_ONLY       1
#define SERVO_D_SCAN_LIMIT      (1.25f * 6.28318530f)  /* 每遍 1.25 圈 ≈ 9.1s,两遍 18.2s < 20s */

#define SERVO_D_EDGE_CONFIRM    3u                     /* 连续 3 拍(6ms)保持新电平才认边沿,滤掉毛刺 */
#define SERVO_D_TIMEOUT_MS      20000u

static void Servo_D_Send_Pos(Servo_D_Init *__home, float __target)
{
    Class_Motor_DJ *m = __home->motor;
    if (m == nullptr) return;

    m->Set_Motor_Control_mode(motor_pos);
    m->Set_Pos(__target);
    if (m->Is_Send_Due())
    {
        m->Calculate();
        m->send();
    }
}

void Servo_D_Init_Start(Servo_D_Init *__home, Class_Motor_DJ *__motor,
                        Enum_Electric_Input __photo,
                        float __zero_on_rise, float __zero_on_fall,
                        int8_t __direction,
                        float __search_w, float __bias)
{
    (void)__direction;
    (void)__search_w;

    if (__home == nullptr) return;

    __home->motor = __motor;
    __home->photo = __photo;
    __home->state = SERVO_D_INIT_SEARCH;
    __home->zero_on_rise = __zero_on_rise;
    __home->zero_on_fall = __zero_on_fall;
    __home->bias = __bias;
    __home->final_angle = 0.0f;

    __home->scan_angle = 0.0f;
    __home->last_level = 0xFFu;
    __home->stable_count = 0u;
    __home->edge_count = 0u;
    __home->first_edge_valid = 0u;
    __home->first_rise = 0u;
    __home->dir = +1;
    __home->retried = 0u;
    __home->start_ms = (uint32_t)Timestamp_Get_Ms();
    __home->step_ms = __home->start_ms;
    __home->timeout_ms = SERVO_D_TIMEOUT_MS;

    if (__motor != nullptr)
    {
        /* 先把当前位置定义成 0°,再从这里向正方向遍历一整圈 */
        __motor->zero_to_angle(0.0f);
        __motor->Set_Motor_Control_mode(motor_pos);
        __motor->PID_Angle.Clear();
        __motor->PID_speed.Clear();
        __motor->enable();
        __motor->Set_Pos(0.0f);
        if (__motor->Is_Send_Due())
        {
            __motor->Calculate();
            __motor->send();
        }
    }
}

void Servo_D_Init_Task(Servo_D_Init *__home)
{
    Class_Motor_DJ *m;
    uint32_t now_ms;
    uint8_t level;

    if (__home == nullptr || __home->state == SERVO_D_INIT_IDLE ||
        __home->state == SERVO_D_INIT_DONE || __home->state == SERVO_D_INIT_ERROR)
    {
        return;
    }

    m = __home->motor;
    if (m == nullptr)
    {
        __home->state = SERVO_D_INIT_ERROR;
        return;
    }

    now_ms = (uint32_t)Timestamp_Get_Ms();
    level = Electric_Input_Get_Raw(__home->photo);

    if (__home->last_level == 0xFFu)
    {
        __home->last_level = level;
    }
    else if (level != __home->last_level)
    {
        if (__home->stable_count < 255u) __home->stable_count++;
    }
    else
    {
        __home->stable_count = 0u;
    }

    /* ★ 统一只用 0→1 边沿结算:1→0 只记一笔,继续往前扫到下一个 0→1。
       这样无论上电时轮子停在哪儿,最后落到的是同一个零点坐标
       (不会再出现"这次 0°、下次 180°");两个轮子也总是按同一极性收尾。 */
    if (__home->stable_count >= SERVO_D_EDGE_CONFIRM)
    {
        const uint8_t is_rise = (__home->last_level == 0u && level == 1u) ? 1u : 0u;

        /* ★ 必须记下这一拍:否则 last_level 还停在旧电平,下一拍会把同一个边沿再认一次 */
        __home->last_level = level;
        __home->stable_count = 0u;
        if (__home->edge_count < 255u) __home->edge_count++;

#if SERVO_D_RISE_ONLY
        /* 第一遍只收上升沿;反向兜底那一遍什么边沿都收 */
        const uint8_t finish_here = (is_rise || (__home->retried != 0u)) ? 1u : 0u;
#else
        const uint8_t finish_here = 1u;
#endif

        if (finish_here)
        {
            const float edge_assign = is_rise ? __home->zero_on_rise : __home->zero_on_fall;
            const float target_angle = Math_Angle_Wrap_2Pi(edge_assign + __home->bias);

            __home->final_angle = target_angle;
            m->zero_to_angle(Math_Angle_Wrap_2Pi(edge_assign));
            m->Set_Motor_Control_mode(motor_pos);
            m->PID_Angle.Clear();
            /* 扫描期间速度环积分已经积满,不清掉会在到点后继续顶,造成定点来回抖 */
            m->PID_speed.Clear();
            m->Set_Pos(target_angle);
            m->enable();
            __home->state = SERVO_D_INIT_DONE;
            return;
        }
        /* 1→0:不结算,继续扫(下一个 0→1 就在半圈之后) */
    }

    if ((uint32_t)(now_ms - __home->start_ms) > __home->timeout_ms)
    {
        __home->state = SERVO_D_INIT_ERROR;
        return;
    }

    /* 0.1° / 2ms 单方向遍历;一遍没找到就反向再来一遍 */
    if ((uint32_t)(now_ms - __home->step_ms) >= SERVO_D_STEP_MS)
    {
        __home->step_ms = now_ms;
        __home->scan_angle += (float)__home->dir * SERVO_D_STEP_RAD;

        if (__home->scan_angle >= SERVO_D_SCAN_LIMIT)
        {
            if (__home->retried == 0u)
            {
                /* 第一遍正向 1.25 圈没找到 0→1:反向兜底扫一遍(接受任意边沿) */
                __home->retried = 1u;
                __home->dir = -1;
                __home->scan_angle = 0.0f;
                __home->last_level = 0xFFu;      /* 重新对齐电平 */
                __home->stable_count = 0u;
            }
            else
            {
                __home->state = SERVO_D_INIT_ERROR;
                return;
            }
        }
        else if (__home->scan_angle <= -SERVO_D_SCAN_LIMIT)
        {
            __home->state = SERVO_D_INIT_ERROR;
            return;
        }

        Servo_D_Send_Pos(__home, __home->scan_angle);
    }
}

bool Servo_D_Init_Is_Done(const Servo_D_Init *__home)
{
    return (__home != nullptr) && (__home->state == SERVO_D_INIT_DONE);
}

bool Servo_D_Init_Is_Error(const Servo_D_Init *__home)
{
    return (__home != nullptr) && (__home->state == SERVO_D_INIT_ERROR);
}
