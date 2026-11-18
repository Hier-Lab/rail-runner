#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>

#define MAX_JRK 150.0f
#define MAX_ACC 100.0f
#define MAX_VEL 32.0f
#define CONTROL_DT_S 0.001f

/** True once the profile has settled at zero velocity and acceleration. */
bool motion_is_stopped(float velocity, float acceleration);

void motion_profile_step(float target, float *velocity, float *acceleration);
float motion_stopping_distance(float velocity, float acceleration);

#endif
