#include "velocity.h"
#include "attitude.h"
#include "kalman_filter_velocity.h"

struct Velocity_Motor g_vel_motor;
static kalman_filter_velocity_t s_kf;
static uint8 s_vel_update_flag = 0;

void velocity_init(struct Velocity_Motor* vel_motor) {
    vel_motor->momentumFront = 0;
    vel_motor->momentumBack = 0;
    vel_motor->bottom = 0;
    vel_motor->bottomFiltered = 0;
    vel_motor->velocityDiff = 0;
    vel_motor->bottomSum = 0;
#ifdef VELOCITY_KALMAN_FILTER
    // velocityKalmanFilterInit();
    kalman_filter_velocity_init(&s_kf);
    s_vel_update_flag = 1;
#endif
}

void velocity_update(struct Velocity_Motor* vel_motor) {
    get_momentum_encoder(vel_motor);
    get_momentum_encoder(vel_motor);
#ifdef VELOCITY_KALMAN_FILTER
    if (s_vel_update_flag == 0) {
        return;
    }
    // velocityKalmanFilterPredict();
    // velocityKalmanFilterUpdate();
    // motorVelocity.bottomReal = filter.velocityEstimate;
    // motorVelocity.bottomFiltered = filter.velocityEstimate *
    // V_KALMAN_MULTIPLE;

    kalman_filter_velocity_predict(&s_kf);
    float z_1 = (float)vel_motor->bottom * 2.077e-3f;
    float z_2 =
        -currentFrontAcceleration / cosf(ANGLE_TO_RAD(currentFrontAngle));
    float z[MEASUREMENT_SIZE] = {z_1, z_2};
    kalman_filter_velocity_update(&s_kf, z);
    vel_motor->bottomReal = s_kf.x[0];
    vel_motor->bottomFiltered = s_kf.x[0] * V_KALMAN_MULTIPLE;
#endif

    // motorVelocity.bottomFiltered = (float)motorVelocity.bottom * 2.077e-3f;
}