#ifndef Mahony_ATTITUDE_H
#define Mahony_ATTITUDE_H

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
#define AngleAcceleration g_imu_data.acc.z

struct EulerAngle {
    float roll;
    float pitch;
    float yaw;
};

struct Control_Turn_Manual_Params;
struct Control_Target;
struct Velocity_Motor;

void attitude_init();     // attitude init
void attitude_cal_ekf();  // attitude calculate
void attitude_cal_amend(struct Control_Turn_Manual_Params* turn_param,
                        struct Control_Target* control_target,
                        struct Velocity_Motor* velocity_motor,
                        struct EulerAngle* euler_angle);
void attitude_show();

extern struct EulerAngle g_euler_angle;
extern struct EulerAngle g_euler_angle_bias;
extern uint8 g_attitude_cal_flag;
#endif
