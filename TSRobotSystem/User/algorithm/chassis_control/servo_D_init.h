/* ============================================================================
 * servo_D_init.h — 舵轮光电找零(位置单方向遍历)
 *
 * 不用二分。位置环按 0.1° / 2ms 从 0° 向 360° 连续走:
 *   0.1° / 2ms = 50°/s
 *   一整圈约 7.2 秒
 *
 * 只用 0→1(上升沿)结算:
 *   上电时轮子可能先看到 1→0 —— 这时不结算,继续扫到下一个 0→1。
 *   好处:上电停在哪,最后都落到同一个零点坐标,不会这次 0°、下次 180°。
 *   代价:最多多走半圈;最坏 2 圈(14.4s)< 20s 超时。
 *
 * 每个轮的零点 = zero_on_rise + bias(202→0°, 204→180°, 按同向/镜像安装各配)。
 * zero_on_fall 只在 SERVO_D_RISE_ONLY=0(退回双边沿)时使用。
 * ========================================================================== */

#ifndef SERVO_D_INIT_H
#define SERVO_D_INIT_H

#include "dev_electric.h"
#include "motor_DJ.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SERVO_D_INIT_IDLE = 0,
    SERVO_D_INIT_WAIT_FEEDBACK,
    SERVO_D_INIT_SEARCH,
    SERVO_D_INIT_DONE,
    SERVO_D_INIT_ERROR
} ServoDInitState;

typedef struct
{
    Class_Motor_DJ     *motor;
    Enum_Electric_Input photo;
    ServoDInitState     state;

    float               zero_on_rise;    /* 0→1 时赋的角度 */
    float               zero_on_fall;    /* 1→0 时赋的角度 */
    float               bias;            /* 外部偏置 rad */
    float               final_angle;     /* 找零完成后的最终目标角 */

    float               scan_angle;      /* 当前遍历目标角 */
    uint8_t             last_level;
    uint8_t             stable_count;

    uint32_t            start_ms;
    uint32_t            step_ms;
    uint32_t            timeout_ms;
    uint8_t             edge_count;
    uint8_t             first_edge_valid;
    uint8_t             first_rise;
    int8_t              dir;             /* 扫描方向:+1 正转,-1 反转(第一次失败后兜底) */
    uint8_t             retried;         /* 已经反向重扫过一次 */
} Servo_D_Init;

void Servo_D_Init_Start(Servo_D_Init *__home, Class_Motor_DJ *__motor,
                        Enum_Electric_Input __photo,
                        float __zero_on_rise, float __zero_on_fall,
                        int8_t __direction,
                        float __search_w, float __bias);

void Servo_D_Init_Task(Servo_D_Init *__home);

bool Servo_D_Init_Is_Done(const Servo_D_Init *__home);
bool Servo_D_Init_Is_Error(const Servo_D_Init *__home);

#ifdef __cplusplus
}
#endif

#endif // SERVO_D_INIT_H
