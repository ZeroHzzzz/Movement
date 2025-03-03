#ifndef _VELOCITY_H_
#define _VELOCITY_H_

#include "zf_common_headfile.h"

typedef struct {
    int32 momentumFront;
    int32 momentumBack;
    int32 bottom;
    float bottomReal;
    float bottomFiltered;
    int32 bottomSum;
    int32 velocityDiff;
} Velocity_Motor;

extern Velocity_Motor g_vel_motor;

#endif