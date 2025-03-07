#ifndef _VELOCITY_H_
#define _VELOCITY_H_

#include "zf_common_headfile.h"

#define V_KALMAN_MULTIPLE 500
#define VELOCITY_KALMAN_FILTER

struct Velocity_Motor {
    int32 momentumFront;
    int32 momentumBack;
    int32 bottom;
    float bottomReal;
    float bottomFiltered;
    int32 bottomSum;
    int32 velocityDiff;
};

extern struct Velocity_Motor g_vel_motor;

void velocity_init(struct Velocity_Motor* vel_motor);
void velocity_update(struct Velocity_Motor* vel_motor);
#endif