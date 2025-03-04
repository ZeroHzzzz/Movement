#include "imu.h"

IMU_DATA g_imu_data;
float gyroOffset[3] = {0.0f, 0.0f, 0.0f};

void imu_init() {
    printf("Use IMU963RA\n");
    bool tmp_flag = imu963ra_init();
    if (tmp_flag == 1) {
        printf("IMU963RA init Failed\n");
        return;  // TODO: 改成直接退出代码
    }
}

void imu_get_data(IMU_DATA* data) {
    imu963ra_get_gyro();
    imu963ra_get_acc();

    data->gyro.x = imu963ra_gyro_transition(imu963ra_gyro_x) * DEG2RAD;
    data->gyro.y = imu963ra_gyro_transition(imu963ra_gyro_y) * DEG2RAD;
    data->gyro.z = imu963ra_gyro_transition(imu963ra_gyro_z) * DEG2RAD;

    data->acc.x = imu963ra_acc_transition(imu963ra_acc_x) * GravityAcc;
    data->acc.y = imu963ra_acc_transition(imu963ra_acc_y) * GravityAcc;
    data->acc.z = imu963ra_acc_transition(imu963ra_acc_z) * GravityAcc;
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