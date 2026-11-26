//
// robot_solution.cpp — D2Q2 运动学/动力学实现
//

#include "robot_solution.h"

#include <math.h>

#define ROBOT_PI       3.14159265358979f
#define ROBOT_TWO_PI   (2.0f * ROBOT_PI)

static float Robot_WrapPi(float __a)
{
    while (__a > ROBOT_PI)  __a -= ROBOT_TWO_PI;
    while (__a < -ROBOT_PI) __a += ROBOT_TWO_PI;
    return __a;
}

static float Robot_Wrap2Pi(float __a)
{
    while (__a >= ROBOT_TWO_PI) __a -= ROBOT_TWO_PI;
    while (__a < 0.0f)          __a += ROBOT_TWO_PI;
    return __a;
}

int RobotSolution_Inverse2(const RobotVelocity *__vel,
                           const RobotWheelGeometry __wheel[2],
                           float __wheel_radius,
                           float __max_wheel_rpm,
                           RobotWheelCommand __out[2])
{
    uint8_t i;

    if ((__vel == 0) || (__wheel == 0) || (__out == 0) || (__wheel_radius <= 0.0f))
    {
        return -1;
    }

    for (i = 0u; i < 2u; i++)
    {
        const float vx = __vel->vx - __vel->w * __wheel[i].y;
        const float vy = __vel->vy + __vel->w * __wheel[i].x;
        const float mag = sqrtf(vx * vx + vy * vy);

        __out[i].vx = vx;
        __out[i].vy = vy;
        __out[i].flipped = 0u;

        if (mag < 0.0001f)
        {
            __out[i].steer_rad = 0.0f;
            __out[i].wheel_rpm = 0.0f;
            continue;
        }

        float steer = atan2f(vy, vx);
        float rpm = mag / __wheel_radius * 30.0f / ROBOT_PI;

        /* 允许时走短路径: 与当前零位差超过 90° 就翻 180° 并反转轮速。
           当前机械零位未知时,这里只对几何零位做翻转,上层可再次处理。 */
        steer = Robot_WrapPi(steer - __wheel[i].zero_rad);

        if (__max_wheel_rpm > 0.0f && rpm > __max_wheel_rpm)
        {
            rpm = __max_wheel_rpm;
        }

        __out[i].steer_rad = Robot_Wrap2Pi(steer * __wheel[i].dir);
        __out[i].wheel_rpm = rpm * __wheel[i].dir;
    }

    return 0;
}

int RobotSolution_Forward2(const RobotWheelFeedback __wheel[2],
                           const RobotWheelGeometry __geom[2],
                           float __wheel_radius,
                           RobotVelocity *__out)
{
    float vx[2], vy[2];
    float dx, dy;
    uint8_t i;

    if ((__wheel == 0) || (__geom == 0) || (__out == 0) || (__wheel_radius <= 0.0f))
    {
        return -1;
    }

    for (i = 0u; i < 2u; i++)
    {
        const float speed = __wheel[i].wheel_rpm * ROBOT_PI / 30.0f * __wheel_radius;
        const float steer = Robot_Wrap2Pi(__wheel[i].steer_rad * __geom[i].dir)
                          + __geom[i].zero_rad;
        vx[i] = speed * cosf(steer);
        vy[i] = speed * sinf(steer);
    }

    dx = __geom[0].x - __geom[1].x;
    dy = __geom[0].y - __geom[1].y;

    if (fabsf(dx) > 0.0001f)
    {
        __out->w = (vy[1] - vy[0]) / dx;
    }
    else if (fabsf(dy) > 0.0001f)
    {
        __out->w = (vx[0] - vx[1]) / dy;
    }
    else
    {
        __out->w = 0.0f;
    }

    __out->vx = 0.5f * (vx[0] + vx[1]) + __out->w * 0.5f * (__geom[0].y + __geom[1].y);
    __out->vy = 0.5f * (vy[0] + vy[1]) - __out->w * 0.5f * (__geom[0].x + __geom[1].x);
    return 0;
}

float RobotSolution_PowerEstimate(const RobotVelocity *__vel,
                                  const RobotWheelCommand __wheel[2],
                                  float __torque_per_wheel_nm,
                                  float __efficiency,
                                  float __idle_w)
{
    float p = 0.0f;
    uint8_t i;

    (void)__vel;

    if (__wheel == 0) return __idle_w;
    if (__efficiency <= 0.01f) __efficiency = 0.01f;

    for (i = 0u; i < 2u; i++)
    {
        const float w = __wheel[i].wheel_rpm * ROBOT_PI / 30.0f;
        p += fabsf(w * __torque_per_wheel_nm);
    }

    return p / __efficiency + __idle_w;
}
