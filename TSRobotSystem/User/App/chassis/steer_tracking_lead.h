#pragma once

#include <math.h>
#include <stdint.h>

/* Bounded position lead for continuously moving steering targets.
 * Units: output-side radians and milliseconds. */
class SteerTrackingLead
{
public:
    float Update(bool enabled, float nominal, float measured, uint32_t now_ms)
    {
        if (!enabled || !isfinite(nominal) || !isfinite(measured))
        {
            Reset();
            return nominal;
        }
        if (!valid_)
        {
            valid_ = true;
            last_target_ = nominal;
            last_ms_ = now_ms;
            return nominal;
        }
        const uint32_t elapsed_ms = now_ms - last_ms_;
        const float step = Wrap(nominal - last_target_);
        last_target_ = nominal;
        last_ms_ = now_ms;
        /* Reset at mode/arc changes or missed cycles; never differentiate a step. */
        if (elapsed_ms == 0u || elapsed_ms > 10u || fabsf(step) > 0.12f)
        {
            rate_ = 0.0f;
            return nominal;
        }
        const float dt = (float)elapsed_ms * 0.001f;
        float rate = step / dt;
        if (rate > 2.5f) rate = 2.5f;
        if (rate < -2.5f) rate = -2.5f;
        rate_ += (dt / (0.025f + dt)) * (rate - rate_);
        float lead = 0.025f * rate_;
        if (lead > 0.06f) lead = 0.06f;
        if (lead < -0.06f) lead = -0.06f;
        /* Do not let compensation undo the selected <=90 degree short arc. */
        const float half_pi = 1.57079632679f;
        float arc = Wrap(nominal - measured) + lead;
        if (arc > half_pi) arc = half_pi;
        if (arc < -half_pi) arc = -half_pi;
        return measured + arc;
    }
    void Reset() { valid_ = false; rate_ = 0.0f; }

private:
    static float Wrap(float angle)
    {
        const float pi = 3.14159265359f;
        const float two_pi = 6.28318530718f;
        angle = fmodf(angle + pi, two_pi);
        if (angle < 0.0f) angle += two_pi;
        return angle - pi;
    }
    float last_target_ = 0.0f;
    float rate_ = 0.0f;
    uint32_t last_ms_ = 0u;
    bool valid_ = false;
};
