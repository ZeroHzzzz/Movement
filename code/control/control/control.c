#include "control.h"
#include "menu_input.h"
#include "motor.h"
#include "pid.h"
#include "system.h"
#include "velocity.h"
#include "zf_common_headfile.h"

// global
uint8 g_turn_start_flag = 0;
struct Control_Turn_Manual_Params g_control_turn_manual_params;
struct Control_Target g_control_target;
struct Control_Flag g_control_flag;
struct Control_Time g_control_time;
struct Control_Motion_Manual_Parmas g_control_motion_params;

// static
static int32 s_bottom_balance_duty = 0;
static int32 s_side_balance_duty = 0;
static int32 s_momentum_diff = 0;

static void control_param_init(pid_type_def* pid,
                               const uint32 para[3],
                               float coefficient,
                               float maxOut,
                               float maxIOut);
// static float control_bottom_feedforward();
// static void control_shutdown(struct Control_Target* control_target,
//                              struct EulerAngle* euler_angle_bias);

// bottom
static void control_bottom_velocity(struct Velocity_Motor* vel_motor,
                                    struct Control_Target* control_target);
static void control_bottom_angle(struct EulerAngle* euler_angle_bias,
                                 struct Control_Target* control_target);
static void control_bottom_angle_velocity(
    struct Control_Target* control_target);

// side
static void control_side_velocity(
    struct Velocity_Motor* vel_motor,
    struct Control_Target* control_target,
    struct Control_Turn_Manual_Params* control_turn_params);
static void control_side_angle(struct EulerAngle* euler_angle_bias,
                               struct Control_Target* control_target);
static void control_side_angle_velocity(struct Control_Target* control_target);

// PID
// bottom wheel
pid_type_def bottom_angle_velocity_PID;
pid_type_def bottom_angle_PID;
pid_type_def bottom_velocity_PID;

// momentum wheel pid
pid_type_def side_angle_velocity_PID;
pid_type_def side_angle_PID;
pid_type_def side_velocity_PID;

// turn pid
pid_type_def turn_angle_PID;
pid_type_def turn_angle_velocity_PID;

int32 get_bottom_duty() {
    return s_bottom_balance_duty;
}

int32 get_side_duty() {
    return s_side_balance_duty;
}

int get_momentum_diff() {
    return s_momentum_diff;
}

void control_bottom_balance(struct Control_Target* control_target,
                            struct Control_Flag* control_flag,
                            struct Velocity_Motor* vel_motor,
                            struct EulerAngle* euler_angle_bias) {
    if (control_flag->frontVelocity) {
        control_flag->frontVelocity = 0;
        control_bottom_velocity(vel_motor, control_target);
    }
    if (control_flag->frontAngle) {
        control_flag->frontAngle = 0;
        control_bottom_angle(euler_angle_bias, control_target);
    }
    if (control_flag->frontAngleVelocity) {
        control_flag->frontAngleVelocity = 0;
        control_bottom_angle_velocity(control_target);
    }
    // restrictValueI(&s_bottom_balance_duty,-3000,3000);
    // fuzzyBottomMotorHertz();
    if (bottom_motor_deadzone != 0) {  // apply feedforward if enabled
        int32 vel_motorDeadV = vel_motor->bottom;
        int8 d = 80;
        restrictValueI(&vel_motorDeadV, -d, d);
        vel_motorDeadV = (abs(d - abs(vel_motorDeadV)) /
                          d);  // normalize to 0~1, 0 is the max deadzone pwm
        if (s_bottom_balance_duty > 0) {
            s_bottom_balance_duty +=
                bottom_motor_deadzone *
                (vel_motorDeadV +
                 0.1f);  // 0.1f is the minimum pwm to avoid shaking
        } else {
            s_bottom_balance_duty -=
                bottom_motor_deadzone * (vel_motorDeadV + 0.1f);
        }
    }
    set_bottom_motor_pwn(
        (int32)(s_bottom_balance_duty));  // set bottom motor pwm to
                                          // keep front balance
}

static void control_bottom_velocity(struct Velocity_Motor* vel_motor,
                                    struct Control_Target* control_target) {
#ifdef VELOCITY_KALMAN_FILTER
    control_target->frontAngle =
        -0.1f * PID_calc_Position(&bottom_velocity_PID,
                                  (float)vel_motor->bottomFiltered,
                                  control_target->frontVelocity);
#endif
#ifndef VELOCITY_KALMAN_FILTER
    control_target->frontAngle =
        -0.1f * PID_calc_Position(&bottom_velocity_PID,
                                  (float)vel_motor->bottom,
                                  control_target->frontVelocity);
#endif
    // control_target->frontAngle = g_euler_angle_bias->roll - 0.1f *
    // PID_calc_Position_DynamicI(&bottom_velocity_PID,(float)vel_motor->bottom,control_target->frontVelocity,
    // 80, 1.5f);
    // TODO:tune the parameter
    restrictValueF(&control_target->frontAngle, 15.0f, -15.0f);
    // {
    //     control_target->frontAngle = g_euler_angle_bias->roll;
    // }
}

static void control_bottom_angle(struct EulerAngle* euler_angle_bias,
                                 struct Control_Target* control_target) {
    static float angleControlFilter[2] = {0};
    angleControlFilter[1] = angleControlFilter[0];
    angleControlFilter[0] = currentFrontAngle;
    // lowPassFilter(&angleControlFilter[0],&angleControlFilter[1],0.1f);
    // noiseFilter(angleControlFilter[0],0.002f);

    // simpleFuzzyProcess(&frontBalanceSimpleFuzzy,angleControlFilter[0],control_target->frontAngle,&bottom_angle_PID);
    control_target->frontAngleVelocity = (PID_calc_Position(
        &bottom_angle_PID, (angleControlFilter[0] - euler_angle_bias->roll),
        control_target->frontAngle));
}

static void control_bottom_angle_velocity(
    struct Control_Target* control_target) {
    // imu963raPushingSensorData();
    static float angleVelocityControlFilter[2] = {0};
    angleVelocityControlFilter[1] = angleVelocityControlFilter[0];
    angleVelocityControlFilter[0] = currentFrontAngleVelocity;

    s_bottom_balance_duty = (int32)(PID_calc_DELTA(
        &bottom_angle_velocity_PID, angleVelocityControlFilter[0],
        control_target->frontAngleVelocity));
}

void control_side_balance(
    struct Control_Target* control_target,
    struct Control_Flag* control_flag,
    struct Control_Turn_Manual_Params* control_turn_params,
    struct Velocity_Motor* vel_motor,
    struct EulerAngle* euler_angle_bias) {
    if (control_flag->sideAngleVelocity) {
        control_flag->sideAngleVelocity = 0;
        control_side_angle_velocity(control_target);
    }
    if (control_flag->sideAngle) {
        control_flag->sideAngle = 0;
        control_side_angle(euler_angle_bias, control_target);
    }
    if (control_flag->sideVelocity) {
        control_flag->sideVelocity = 0;
        control_side_velocity(vel_motor, control_target, control_turn_params);
    }
    // turnControl();
    int32 left_motor_duty, right_motor_duty;
    left_motor_duty = s_side_balance_duty - s_momentum_diff;
    right_motor_duty = -s_side_balance_duty - s_momentum_diff;

    restrictValueI(&s_side_balance_duty, -9999, 9999);
    set_momentum_motor_pwm(left_motor_duty, right_motor_duty);
    // set momentum motor pwm to keep side balance
}

static void control_side_velocity(
    struct Velocity_Motor* vel_motor,
    struct Control_Target* control_target,
    struct Control_Turn_Manual_Params* control_turn_params) {
    float alpha =
        fabsf(control_turn_params->turnCurvature / CONTROL_LAW_CONSTRAINT);
    static float momentumVelocityFilter[3] = {0};
    momentumVelocityFilter[0] =
        (float)(vel_motor->momentumFront - vel_motor->momentumBack);
    momentumVelocityFilter[1] =
        (float)(vel_motor->momentumFront + vel_motor->momentumBack);
    momentumVelocityFilter[2] =
        momentumVelocityFilter[0] -
        0.7f * (1 - alpha) * momentumVelocityFilter[1];  // TODO:添加到压弯

    control_target->sideAngle = (float)PID_calc_Position(
        &side_velocity_PID, (float)momentumVelocityFilter[2], 0.0f);
}

static void control_side_angle(struct EulerAngle* euler_angle_bias,
                               struct Control_Target* control_target) {
    static float momentumAngleFilter[2] = {0};  // 角度滤波
    momentumAngleFilter[1] = momentumAngleFilter[0];
    momentumAngleFilter[0] = currentSideAngle;
    // noiseFilter(momentumAngleFilter[0],0.02f);
    // lowPassFilter(&momentumAngleFilter[0],&momentumAngleFilter[1],0.1f);
    control_target->sideAngleVelocity = PID_calc_Position(
        &side_angle_PID, (momentumAngleFilter[0] - euler_angle_bias->pitch),
        control_target->sideAngle);
}

static void control_side_angle_velocity(struct Control_Target* control_target) {
    static float momentumGyroFilter[2] = {0};  // 角度速度滤波
    momentumGyroFilter[1] = momentumGyroFilter[0];
    momentumGyroFilter[0] = currentSideAngleVelocity;

    s_side_balance_duty =
        (int32)(PID_calc_DELTA(&side_angle_velocity_PID, momentumGyroFilter[0],
                               control_target->sideAngleVelocity));
}

void control_shutdown(struct Control_Target* control_target,
                      struct EulerAngle* euler_angle_bias) {
    if (fabsf(currentFrontAngle - euler_angle_bias->pitch -
              control_target->sideAngle) > 60) {
        stop_bottom_motor();
        stop_momentum_motor();

        printf("control_shutdown 1\n");
        zf_assert(0);
        runState = CAR_STOP;
    }
    if (fabsf(currentSideAngle - euler_angle_bias->roll -
              control_target->frontAngle) > 28) {
        stop_bottom_motor();
        stop_momentum_motor();

        printf("control_shutdown 2\n");
        char tmp[10];
        snprintf(tmp, 10, "%.3f", euler_angle_bias->roll);
        zf_log(0, tmp);
        runState = CAR_STOP;
    }
}

void control_init(struct Control_Motion_Manual_Parmas* control_motion_params) {
    control_param_init(&bottom_angle_velocity_PID,
                       control_motion_params->bottom_angle_velocity_parameter,
                       10, MOTOR_PWM_MAX, 9999);
    control_param_init(&bottom_angle_PID,
                       control_motion_params->bottom_angle_parameter,
                       pidCoefficient, 9999, 10.0f);
    control_param_init(&bottom_velocity_PID,
                       control_motion_params->bottom_velocity_parameter,
                       pidCoefficient, 9999, 2.5f);
    // momentum wheel pid
    control_param_init(&side_angle_velocity_PID,
                       control_motion_params->side_angle_velocity_parameter, 1,
                       MOMENTUM_MOTOR_PWM_MAX, 9999);
    control_param_init(&side_angle_PID,
                       control_motion_params->side_angle_parameter, 10, 9999,
                       3.0f);
    control_param_init(&side_velocity_PID,
                       control_motion_params->side_velocity_parameter, 1000,
                       9999, 1.5f);
    control_param_init(&turn_angle_PID,
                       control_motion_params->turn_angle_parameter, 100, 9999,
                       5);
    control_param_init(&turn_angle_velocity_PID,
                       control_motion_params->turn_velocity_parameter, 1, 9999,
                       500);
}

/// @brief init the control parameter in the menu
void control_manual_param_init() {
    bottom_angle_velocity_PID.Kp = 0;
    bottom_angle_velocity_PID.Ki = 0;
    bottom_angle_velocity_PID.Kd = 0;
    bottom_angle_PID.Kp = 0;
    bottom_angle_PID.Ki = 0;
    bottom_angle_PID.Kd = 0;
    bottom_velocity_PID.Kp = 0;
    bottom_velocity_PID.Ki = 0;
    bottom_velocity_PID.Kd = 0;
    side_angle_velocity_PID.Kp = 0;
    side_angle_velocity_PID.Ki = 0;
    side_angle_velocity_PID.Kd = 0;
    side_angle_PID.Kp = 0;
    side_angle_PID.Ki = 0;
    side_angle_PID.Kd = 0;
    side_velocity_PID.Kp = 0;
    side_velocity_PID.Ki = 0;
    side_velocity_PID.Kd = 0;
    turn_angle_PID.Kp = 0;
    turn_angle_PID.Ki = 0;
    turn_angle_PID.Kd = 0;
    turn_angle_velocity_PID.Kp = 0;
    turn_angle_velocity_PID.Ki = 0;
    turn_angle_velocity_PID.Kd = 0;
}

static void control_param_init(pid_type_def* pid,
                               const uint32 para[3],
                               float coefficient,
                               float maxOut,
                               float maxIOut) {
    float temp_pid[3];
    temp_pid[0] = (float)para[0] / coefficient;
    temp_pid[1] = (float)para[1] / coefficient;
    temp_pid[2] = (float)para[2] / coefficient;
    PID_init_Position(pid, temp_pid, maxOut, maxIOut);
}