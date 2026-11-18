#include "motion.h"

#include <math.h>

#include "mathutil.h"

#define SETTLE_VELOCITY_BAND 0.005f
#define SETTLE_ACCELERATION_BAND 1.0f
/* Exact-zero equality on a value that approaches zero through float jerk
   limiting rarely lands on the exact bit pattern, so a stop could hang short
   of done indefinitely; a small tolerance is the fix. */
#define STOPPED_EPS 0.0001f

static float approach(float value, float target, float maximum_step)
{
    if (value < target - maximum_step)
    {
        return value + maximum_step;
    }
    if (value > target + maximum_step)
    {
        return value - maximum_step;
    }
    return target;
}

bool motion_is_stopped(float velocity, float acceleration)
{
    return fabsf(velocity) < STOPPED_EPS && fabsf(acceleration) < STOPPED_EPS;
}

void motion_profile_step(float target, float *velocity, float *acceleration)
{
    const float jerk_step = MAX_JRK * CONTROL_DT_S;
    float neutral_velocity = *velocity +
                             *acceleration * fabsf(*acceleration) /
                             (2.0f * MAX_JRK);
    float neutral_error = target - neutral_velocity;
    float desired_acceleration = 0.0f;

    if (neutral_error > 0.00001f)
    {
        desired_acceleration = MAX_ACC;
    }
    else if (neutral_error < -0.00001f)
    {
        desired_acceleration = -MAX_ACC;
    }

    float old_acceleration = *acceleration;
    *acceleration = approach(*acceleration, desired_acceleration, jerk_step);
    *acceleration = clampf(*acceleration, -MAX_ACC, MAX_ACC);
    *velocity += 0.5f * (old_acceleration + *acceleration) * CONTROL_DT_S;

    /* The jerk limiter moves velocity in steps of up to jerk_step * dt, so a
       band narrower than that is never entered and the profile hunts around
       the target forever (up to 0.0007 mm/s and 0.45 mm/s^2). */
    if (fabsf(target - *velocity) < SETTLE_VELOCITY_BAND &&
        fabsf(*acceleration) < SETTLE_ACCELERATION_BAND)
    {
        *velocity = target;
        *acceleration = 0.0f;
    }
    *velocity = clampf(*velocity, -MAX_VEL, MAX_VEL);
}

static void integrate_segment(float *position, float *velocity,
                              float *acceleration, float jerk, float duration)
{
    *position += *velocity * duration +
                 0.5f * *acceleration * duration * duration +
                 jerk * duration * duration * duration / 6.0f;
    *velocity += *acceleration * duration +
                 0.5f * jerk * duration * duration;
    *acceleration += jerk * duration;
}

float motion_stopping_distance(float velocity, float acceleration)
{
    float position = 0.0f;
    if (velocity <= 0.0f)
    {
        return 0.0f;
    }

    if (acceleration < 0.0f)
    {
        float discriminant = acceleration * acceleration -
                             2.0f * MAX_JRK * velocity;
        if (discriminant >= 0.0f)
        {
            float duration = (-acceleration - sqrtf(discriminant)) / MAX_JRK;
            integrate_segment(&position, &velocity, &acceleration,
                              MAX_JRK, duration);
            return position > 0.0f ? position : 0.0f;
        }
    }

    float peak = sqrtf(MAX_JRK * velocity +
                       0.5f * acceleration * acceleration);
    if (peak <= MAX_ACC)
    {
        float first = (acceleration + peak) / MAX_JRK;
        integrate_segment(&position, &velocity, &acceleration,
                          -MAX_JRK, first);
        integrate_segment(&position, &velocity, &acceleration,
                          MAX_JRK, peak / MAX_JRK);
    }
    else
    {
        float first = (acceleration + MAX_ACC) / MAX_JRK;
        integrate_segment(&position, &velocity, &acceleration,
                          -MAX_JRK, first);
        float final_ramp_loss = MAX_ACC * MAX_ACC / (2.0f * MAX_JRK);
        float hold = (velocity - final_ramp_loss) / MAX_ACC;
        if (hold > 0.0f)
        {
            integrate_segment(&position, &velocity, &acceleration,
                              0.0f, hold);
        }
        integrate_segment(&position, &velocity, &acceleration,
                          MAX_JRK, MAX_ACC / MAX_JRK);
    }
    return position > 0.0f ? position : 0.0f;
}
