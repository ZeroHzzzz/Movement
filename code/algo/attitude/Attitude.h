#ifndef Mahony_ATTITUDE_H
#define Mahony_ATTITUDE_H

#include "control.h"
#include "velocity.h"
#include "zf_common_headfile.h"

#define USE_EKF

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle;

void attitude_init();     // attitude init
void attitude_cal_ekf();  // attitude calculate
void attitude_cal_amend(Control_Turn_Manual_Params* turn_param,
                        Control_Target* control_target,
                        Velocity_Motor* velocity_motor);
void attitude_show();

extern EulerAngle g_euler_angle;
extern uint8 g_attitude_cal_flag;
#endif
