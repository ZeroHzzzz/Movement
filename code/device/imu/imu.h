#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"

#define GravityAcc 9.7936f
#define cheat_define 0.0016f
#define gyroscope_threshold 5
#define DEG2RAD 0.0174533f

typedef struct {
    float x;
    float y;
    float z;
} Axis3f;

typedef struct IMUData {
    Axis3f gyro;
    Axis3f acc;
} IMU_DATA;

void imu_init();
void imu_get_data(IMU_DATA* data);

void imu_init_offset();
void imu_remove_offset(IMU_DATA* data);

extern IMU_DATA g_imu_data;
extern float gyroOffset[3];  // gyroOffset

#endif