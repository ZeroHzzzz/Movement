#include "Attitude.h"
#include "QuaternionEKF.h"
#include "control.h"
#include "imu.h"
#include "velocity.h"
#include "zf_common_headfile.h"

EulerAngle g_euler_angle;
uint8 g_attitude_cal_flag = 0;

void attitude_init() {
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0);  // ekf初始化
    imu_init_offset();                                          // 初始化零飘
}

void attitude_cal_ekf() {
    // EKF 姿态解算
    IMU_DATA tmp_imu_data;
    imu_get_data(&tmp_imu_data);
    imu_remove_offset(&tmp_imu_data);
    // // LowPassFilter
    // LowPassFilter(&Accelerometer[0], IMUdata.acc.x);
    // LowPassFilter(&Accelerometer[1], IMUdata.acc.y);
    // LowPassFilter(&Accelerometer[2], IMUdata.acc.z);

    IMU_QuaternionEKF_Update(&tmp_imu_data);
}

void attitude_cal_amend() {
    // 修正姿态计算
    if (g_attitude_cal_flag == 0) {
        return;
    } else {
        g_attitude_cal_flag = 0;
    }

    if (curvatureStartFlag) {
        float x = (float)controlTarget.turnAngleVelocity * 0.01f *
                  fabsf((float)motorVelocity.bottomFiltered * 0.01f);
        controlTarget.bucking = buckingK * x;
        restrictValueF(&controlTarget.bucking, 10.5f, -10.5f);
        controlTarget.Fbucking = x * bucklingFrontCoefficientT;
        restrictValueF(&controlTarget.Fbucking, 5.0f, -5.0f);
    }
#ifdef USE_MAHONY
    Mahony_update(imu963raSensorData.gyro.x, imu963raSensorData.gyro.y,
                  imu963raSensorData.gyro.z, imu963raSensorData.acc.x,
                  imu963raSensorData.acc.y, imu963raSensorData.acc.z, 0, 0, 0);
    Mahony_computeAngles();
    g_euler_angle.roll = getRoll() + controlTarget.bucking;
    g_euler_angle.pitch = getPitch();
    g_euler_angle.yaw =
        getYaw() - 180 +
        yawAngleCorrection;  //-180 because the direction of the sensor is
                             // opposite to the direction of the motor
#endif
#ifdef USE_EKF
    // Attitude_Calculate(imu963raSensorData.gyro.x,imu963raSensorData.gyro.y,imu963raSensorData.gyro.z,imu963raSensorData.acc.x,imu963raSensorData.acc.y,imu963raSensorData.acc.z);
    attitude_cal_ekf();
    g_euler_angle.roll =
        QEKF_INS.Roll + controlTarget.bucking;  // + convergenceGain;
    g_euler_angle.pitch = QEKF_INS.Pitch + controlTarget.Fbucking;
    g_euler_angle.yaw = QEKF_INS.Yaw;
#endif
    // g_euler_angle.yaw += 180; // transfer to the same direction
    g_euler_angle.yaw = 360.0f - g_euler_angle.yaw;  // opposite direction
    // g_euler_angle.yaw += yawAngleCorrection;
    g_euler_angle.yaw > 360 ? (g_euler_angle.yaw -= 360)
                            : g_euler_angle.yaw;  // 0~360
    // update module state
    // moduleState.attitude = 1;
}

void attitude_show() {
    tft180_full(RGB565_WHITE);
    tft180_set_color(RGB565_RED, RGB565_WHITE);
    while (keymsg.key != KEY_L) {
        tft180_show_float(0, 0 * 16, Get_Pitch(), 3, 3);
        tft180_show_float(0, 1 * 16, Get_Roll(), 3, 3);
        tft180_show_float(0, 2 * 16, Get_Yaw(), 3, 3);
    }
    tft180_full(RGB565_WHITE);
}