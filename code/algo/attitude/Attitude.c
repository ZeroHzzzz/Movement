#include "Attitude.h"
#include "QuaternionEKF.h"
#include "control.h"
#include "velocity.h"
#include "zf_common_headfile.h"

struct EulerAngle g_euler_angle;
struct EulerAngle g_euler_angle_bias;

uint8 g_attitude_cal_flag = 0;

void attitude_init() {
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0);  // ekf初始化
    imu_init_offset();                                          // 初始化零飘
}

void attitude_cal_ekf() {
    // EKF 姿态解算
    imu_get_data(&g_imu_data);
    imu_remove_offset(&g_imu_data);
    // // LowPassFilter
    // LowPassFilter(&Accelerometer[0], IMUdata.acc.x);
    // LowPassFilter(&Accelerometer[1], IMUdata.acc.y);
    // LowPassFilter(&Accelerometer[2], IMUdata.acc.z);

    IMU_QuaternionEKF_Update(&g_imu_data);
}

void attitude_cal_amend(struct Control_Turn_Manual_Params* turn_param,
                        struct Control_Target* control_target,
                        struct Velocity_Motor* velocity_motor,
                        struct EulerAngle* euler_angle) {
    // 修正姿态计算
    if (g_attitude_cal_flag == 0) {
        return;
    } else {
        g_attitude_cal_flag = 0;
    }

    if (g_turn_start_flag) {
        float x = (float)control_target->turnAngleVelocity * 0.01f *
                  fabsf((float)velocity_motor->bottomFiltered * 0.01f);
        control_target->bucking = turn_param->buckling_turn_coefficient * x;
        restrictValueF(&control_target->bucking, 10.5f, -10.5f);
        control_target->Fbucking = x * turn_param->buckling_front_coefficientT;
        restrictValueF(&control_target->Fbucking, 5.0f, -5.0f);
    }
#ifdef USE_MAHONY
    Mahony_update(imu963raSensorData.gyro.x, imu963raSensorData.gyro.y,
                  imu963raSensorData.gyro.z, imu963raSensorData.acc.x,
                  imu963raSensorData.acc.y, imu963raSensorData.acc.z, 0, 0, 0);
    Mahony_computeAngles();
    euler_angle->roll = getRoll() + control_target->bucking;
    euler_angle->pitch = getPitch();
    euler_angle->yaw =
        getYaw() - 180 +
        yawAngleCorrection;  //-180 because the direction of the sensor is
                             // opposite to the direction of the motor
#endif
#ifdef USE_EKF
    attitude_cal_ekf();
    euler_angle->roll =
        QEKF_INS.Roll + control_target->bucking;  // + convergenceGain;
    euler_angle->pitch = QEKF_INS.Pitch + control_target->Fbucking;
    euler_angle->yaw = QEKF_INS.Yaw;
#endif
    // g_euler_angle->yaw += 180; // transfer to the same direction
    euler_angle->yaw = 360.0f - euler_angle->yaw;  // opposite direction
    // g_euler_angle->yaw += yawAngleCorrection;
    euler_angle->yaw > 360 ? (euler_angle->yaw -= 360)
                           : euler_angle->yaw;  // 0~360
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
