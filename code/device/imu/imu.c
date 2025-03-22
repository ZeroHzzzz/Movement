#include "imu.h"

IMU_DATA g_imu_data;
float gyroOffset[3] = {0.0f, 0.0f, 0.0f};

void imu_init() {
    bool tmp_init_flag = imu660rb_init();
    if (tmp_init_flag == 1) {
        printf("IMU660RB init Failed\n");
        return;
    }
}

void imu_get_data(IMU_DATA* data) {
    // if (g_imu_use_imu963ra_flag != 0) {
    //     imu963ra_get_gyro();
    //     imu963ra_get_acc();

    //     data->gyro.x = imu963ra_gyro_transition(imu963ra_gyro_x) * DEG2RAD;
    //     data->gyro.y = -imu963ra_gyro_transition(imu963ra_gyro_y) * DEG2RAD;
    //     data->gyro.z = imu963ra_gyro_transition(imu963ra_gyro_z) * DEG2RAD;

    //     data->acc.x = imu963ra_acc_transition(imu963ra_acc_x) * GravityAcc;
    //     data->acc.y = -imu963ra_acc_transition(imu963ra_acc_y) * GravityAcc;
    //     data->acc.z = imu963ra_acc_transition(imu963ra_acc_z) * GravityAcc;
    // } else {
    imu660rb_get_acc();
    imu660rb_get_gyro();

    data->gyro.x = imu660rb_gyro_transition(imu660rb_gyro_x) * DEG2RAD;
    data->gyro.y = -imu660rb_gyro_transition(imu660rb_gyro_y) * DEG2RAD;
    data->gyro.z = imu660rb_gyro_transition(imu660rb_gyro_z) * DEG2RAD;

    data->acc.x = imu660rb_acc_transition(imu660rb_acc_x) * GravityAcc;
    data->acc.y = -imu660rb_acc_transition(imu660rb_acc_y) * GravityAcc;
    data->acc.z = imu660rb_acc_transition(imu660rb_acc_z) * GravityAcc;
    // }
}

void imu_init_offset() {
    IMU_DATA data;
    for (int i = 0; i < 2000; i++) {
        imu_get_data(&data);
        if (fabsf(data.gyro.x) + fabsf(data.gyro.y) + fabsf(data.gyro.z) >
            gyroscope_threshold) {
            i--;
            continue;
        }
        gyroOffset[0] += data.gyro.x;
        gyroOffset[1] += data.gyro.y;
        gyroOffset[2] += data.gyro.z;
        system_delay_ms(1);
    }
    gyroOffset[0] *= 0.0005f;
    gyroOffset[1] *= 0.0005f;
    gyroOffset[2] *= 0.0005f;
}

void imu_remove_offset(IMU_DATA* data) {
    data->gyro.x -= gyroOffset[0];
    data->gyro.y -= gyroOffset[1];
    data->gyro.z -= gyroOffset[2];
}