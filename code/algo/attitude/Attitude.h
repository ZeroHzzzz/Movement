#ifndef Mahony_ATTITUDE_H
#define Mahony_ATTITUDE_H

#include "control.h"
#include "imu.h"
#include "velocity.h"
#include "zf_common_headfile.h"

#define USE_EKF

// front direction
#define currentFrontAngle g_euler_angle.pitch
#define currentFrontAngleVelocity g_imu_data.gyro.y
#define currentFrontAcceleration g_imu_data.acc.x
// side direction
#define currentSideAngle g_euler_angle.roll
#define currentSideAngleVelocity g_imu_data.gyro.x
#define currentSideAcceleration g_imu_data.acc.y
// yaw direction
#define yawAngle g_euler_angle.yaw
#define yawAngleVelocity g_imu_data.gyro.z
#define zAngleAcceleration g_imu_data.acc.z

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
