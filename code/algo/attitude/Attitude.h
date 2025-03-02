#ifndef Mahony_ATTITUDE_H
#define Mahony_ATTITUDE_H

#include "zf_common_headfile.h"

#define USE_EKF

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle;

void attitude_init();     // attitude init
void attitude_cal_ekf();  // attitude calculate
void attitude_cal_amend();
void attitude_show();

extern EulerAngle g_euler_angle;
extern uint8 g_attitude_cal_flag;
#endif
