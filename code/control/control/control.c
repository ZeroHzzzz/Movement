#include "control.h"
#include "menu_input.h"
#include "motor.h"
#include "pid.h"
#include "velocity.h"
#include "zf_common_headfile.h"

uint8 g_turn_start_flag = 0;
struct Control_Turn_Manual_Params g_turn_manual_params;
struct Control_Target g_control_target;
struct Control_Flag g_control_flag;

static int32 s_bottom_balance_duty = 0;
static int32 s_side_balance_duty = 0;
static int32 s_momentum_diff = 0;

static void control_param_init(pid_type_def* pid,
                               const uint32 para[3],
                               float coefficient,
                               float maxOut,
                               float maxIOut);
// static float control_bottom_feedforward();
static void control_shutdown(struct Control_Target* control_target,
                             struct EulerAngle* euler_angle_bias);

// bottom
static void control_bottom_velocity(struct Velocity_Motor* vel_motor,
                                    struct Control_Target* control_target);
static void control_bottom_angle(struct EulerAngle* euler_angle_bias,
                                 struct Control_Target* control_target);
static void control_bottom_angle_velocity(
    struct Control_Target* control_target);

// side
static void control_side_velocity(struct Velocity_Motor* vel_motor,
                                  struct Control_Target* control_target);
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
    // {
    //     control_target->frontAngleVelocity = 0;
    // }
}

static void control_bottom_angle_velocity(
    struct Control_Target* control_target) {
    // imu963raPushingSensorData();
    static float angleVelocityControlFilter[2] = {0};
    angleVelocityControlFilter[1] = angleVelocityControlFilter[0];
    angleVelocityControlFilter[0] = currentFrontAngleVelocity;
    // lowPassFilter(&angleVelocityControlFilter[0],&angleVelocityControlFilter[1],0.1f);
    // noiseFilter(angleVelocityControlFilter[0],1.0f);

    // restrictValueF(&angleVelocityControlFilter[0],-150,150);
    // test_t = frontFeedforward();
    s_bottom_balance_duty = (int32)(PID_calc_DELTA(
        &bottom_angle_velocity_PID, angleVelocityControlFilter[0],
        control_target->frontAngleVelocity));

    // s_bottom_balance_duty = (int32)
    // (PID_calc_DELTA(&bottom_angle_velocity_PID,angleVelocityControlFilter[0],control_target->frontAngleVelocity))
    // + frontFeedforward();
}

void control_side_balance(struct Control_Target* control_target,
                          struct Control_Flag* control_flag,
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
        control_side_velocity(vel_motor, control_target);
    }
    // turnControl();
    int32 left_motor_duty, right_motor_duty;
    left_motor_duty = s_side_balance_duty - s_momentum_diff;
    right_motor_duty = -s_side_balance_duty - s_momentum_diff;
    // leftMotorDuty=s_side_balance_duty;
    // rightMotorDuty=-s_side_balance_duty;
    // leftMotorDuty = -s_momentum_diff;
    // rightMotorDuty = s_momentum_diff;
    // {
    // tft180_show_int(50,10,s_side_balance_duty,6);
    // }
    restrictValueI(&s_side_balance_duty, -9999, 9999);
    set_momentum_motor_pwm(left_motor_duty, right_motor_duty);
    // set momentum motor pwm to keep side balance
}

static void control_side_velocity(struct Velocity_Motor* vel_motor,
                                  struct Control_Target* control_target) {
    float alpha =
        fabsf(g_turn_manual_params.turnCurvature / CONTROL_LAW_CONSTRAINT);
    static float momentumVelocityFilter[3] = {0};
    momentumVelocityFilter[0] =
        (float)(vel_motor->momentumFront - vel_motor->momentumBack);
    momentumVelocityFilter[1] =
        (float)(vel_motor->momentumFront + vel_motor->momentumBack);
    momentumVelocityFilter[2] =
        momentumVelocityFilter[0] -
        0.7f * (1 - alpha) * momentumVelocityFilter[1];  // TODO:添加到压弯
    // convergenceGain = momentumVelocityFilter[2] * 0.03f;
    // test_t = momentumVelocityFilter[2];
    // noiseFilter(momentumVelocityFilter[0],0.02f);
    // lowPassFilter(&momentumVelocityFilter[0],&momentumVelocityFilter[1],0.1f);//TODO:fix
    // bug

    // if((vel_motor->momentumFront > 0)){
    //     momentumVelocityFilter[0] = ((float)abs(vel_motor->momentumFront)
    //     + (float)abs(vel_motor->momentumBack));
    // }
    // else if((vel_motor->momentumFront < 0)){
    //     momentumVelocityFilter[0] = -((float)abs(vel_motor->momentumFront)
    //     + (float)abs(vel_motor->momentumBack));
    // }
    // int32 v[2] = {0};
    // v[0] = vel_motor->momentumFront;
    // v[1] = -vel_motor->momentumBack;
    // momentumVelocityFilter[0] = (float)(v[0]+v[1]);
    // momentumVelocityFilter[0] =
    // (float)(vel_motor->momentumFront-vel_motor->momentumBack);
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
    // {
    //     float error = control_target->sideAngle - momentumAngleFilter[0];
    //     error = fabsf(error);
    //     if(error<=0.15){
    //         if(control_target->sideAngleVelocity > 0){
    //             control_target->sideAngleVelocity += (float)(expf(error
    //             + 1.1)
    //             - 1.0f)
    //             *(1.0f-(float)control_flag->sideAngleCount/(float)menuSideControlTimeParameter[1]);
    //         }else{
    //             control_target->sideAngleVelocity -= (float)(expf(error
    //             + 1.1)
    //             - 1.0f)
    //             *(1.0f-(float)control_flag->sideAngleCount/(float)menuSideControlTimeParameter[1]);
    //         }
    //     }else if(error < 0.3){
    //         if(control_target->sideAngleVelocity > 0){
    //             control_target->sideAngleVelocity += (float)logf(1 +
    //             1/(error-0.05))
    //             *(1.0f-(float)control_flag->sideAngleCount/(float)menuSideControlTimeParameter[1]);
    //         }else{
    //             control_target->sideAngleVelocity -= (float)logf(1 +
    //             1/(error-0.05))
    //             *(1.0f-(float)control_flag->sideAngleCount/(float)menuSideControlTimeParameter[1]);
    //         }
    //     }else{
    //         if(control_target->sideAngleVelocity > 0){
    //             control_target->sideAngleVelocity += (float)(expf(error +
    //             0.9)
    //             - 1.0f)*2.5f
    //             *(1.0f-(float)control_flag->sideAngleCount/(float)menuSideControlTimeParameter[1]);
    //         }else{
    //             control_target->sideAngleVelocity -= (float)(expf(error +
    //             0.9)
    //             - 1.0f)*2.5f
    //             *(1.0f-(float)control_flag->sideAngleCount/(float)menuSideControlTimeParameter[1]);
    //         }
    //     }
    // }
    // {
    //     control_target->sideAngleVelocity = 0;
    // }
}

static void control_side_angle_velocity(struct Control_Target* control_target) {
    static float momentumGyroFilter[2] = {0};  // 角度速度滤波
    momentumGyroFilter[1] = momentumGyroFilter[0];
    momentumGyroFilter[0] = currentSideAngleVelocity;
    // noiseFilter(momentumGyroFilter[0],0.02f);
    // lowPassFilter(&momentumGyroFilter[0],&momentumGyroFilter[1],0.1f);

    s_side_balance_duty =
        (int32)(PID_calc_DELTA(&side_angle_velocity_PID, momentumGyroFilter[0],
                               control_target->sideAngleVelocity));

    // TODO:feedforward
    //  int32 v = abs(vel_motor->momentumFront);
    //  restrictValueI(&v,0,12);
    //  if(s_side_balance_duty >= 0){
    //      s_side_balance_duty += v * 100;
    //  }else{
    //      s_side_balance_duty -= v * 100;
    //  }
    //  if(s_side_balance_duty >= 0){
    //      s_side_balance_duty += PID_calc_Position(&compensatePID,v,5);
    //  }else{
    //      s_side_balance_duty -= PID_calc_Position(&compensatePID,v,-5);
    //  }
    //  {
    //  static vel_motor preSideVelocity;
    //  static uint8 cntFront = 0;
    //  static uint8 cntBack = 0;
    //  preSideVelocity.momentumBack = vel_motor->momentumBack;
    //  preSideVelocity.momentumFront = vel_motor->momentumFront;
    //  //if the motor velocity is increasing
    //  if((preSideVelocity.momentumFront - vel_motor->momentumFront > 0) &&
    //  (abs(preSideVelocity.momentumFront - vel_motor->momentumFront) < 2
    //  /*avoid shaking*/)){
    //      cntFront++;
    //  }else if(preSideVelocity.momentumFront - vel_motor->momentumFront <
    //  0){
    //      cntFront = 0;
    //  }
    //  if((preSideVelocity.momentumBack - vel_motor->momentumBack > 0) &&
    //  (abs(preSideVelocity.momentumBack - vel_motor->momentumBack) < 2
    //  /*avoid shaking*/)){
    //      cntBack++;
    //  }else if(preSideVelocity.momentumBack - vel_motor->momentumBack < 0){
    //      cntBack = 0;
    //  }
    //  //add feedforward
    //  int32 v = abs(vel_motor->momentumFront);
    //  static float preT = 0;
    //  restrictValueI(&v,0,12);
    //  float t = 0.012f*(float)v;
    //  restrictValueF(&t,0.01f,0.05f);
    //  //TODO:fix here
    //  if(s_side_balance_duty >= 0){
    //      if((fabsf(t - preT) < 0.036f)&&(cntFront > 0)){//restrict the change
    //      of t, avoid shaking
    //          g_euler_angle_bias->pitch = mechanicalPitchAngle*0.01f + t;
    //          // preT = t;
    //      }
    //      s_side_balance_duty += v * 100;
    //      if(cntFront > 1 && cntFront < 4 && v > 3){
    //          s_side_balance_duty += expf((float)cntFront + 1.5f) * (4.5f -
    //          0.5f * (float)cntFront);
    //      }
    //  }else{
    //      if((fabsf(t - preT) < 0.036f)&&(cntBack > 0)){//avoid shaking
    //          g_euler_angle_bias->pitch = mechanicalPitchAngle*0.01f - t;
    //          // preT = t;
    //      }
    //      s_side_balance_duty -= v * 100;
    //      if(cntBack > 1 && cntBack < 4 && v > 3){
    //          s_side_balance_duty -= expf((float)cntBack + 1.5f) * (4.5f -
    //          0.5f * (float)cntBack);
    //      }
    //  }
    //  preT = t;
    // }
}

void control_shutdown(struct Control_Target* control_target,
                      struct EulerAngle* euler_angle_bias) {
    if (fabsf(currentSideAngle - euler_angle_bias->pitch -
              control_target->sideAngle) > 28) {
        stop_bottom_motor();
        stop_momentum_motor();
        // runState = CAR_STOP;
    }
    if (fabsf(currentFrontAngle - euler_angle_bias->roll -
              control_target->frontAngle) > 60) {
        stop_bottom_motor();
        stop_momentum_motor();
        // runState = CAR_STOP;
    }
}

void control_init(struct Control_Motion_Manual_Parmas* control_motion_params) {
    // buckingK = (float)buckingTurnCoefficient * 0.01f;
    // turnGain = (float)turnGainCoefficient * 0.01f;
    // g_euler_angle_bias->yaw = (float)mechanicalYawAngle * 0.01f;
    // g_euler_angle_bias->pitch = (float)mechanicalPitchAngle * 0.01f;  // side
    // g_euler_angle_bias->roll = (float)mechanicalRollAngle * 0.01f;    //
    // front current wheel pid
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
    // control_param_init(&compensatePID, menuCompensateParameter, 1 , 9999,
    // 80);
    // TODO:add turn part
    // control_param_init(&sideAngleGainPID, menuSideAngleGainParameter,
    // pidCoefficient, 2500, 80);
    control_param_init(&turn_angle_PID,
                       control_motion_params->turn_angle_parameter, 100, 9999,
                       5);
    control_param_init(&turn_angle_velocity_PID,
                       control_motion_params->turn_velocity_parameter, 1, 9999,
                       500);

    // {
    // float outputKpSet[7] = {0};
    // float outputKiSet[7] = {0};
    // float outputKdSet[7] = {0};
    // for(uint8 i = 0; i < 7; i++){
    //     outputKpSet[i] = (float)menuSideAngleParameter[0] / 100.0f * abs(3 -
    //     i); outputKiSet[i] = (float)menuSideAngleParameter[1] / 100.0f *
    //     abs(3 - i); outputKdSet[i] = (float)menuSideAngleParameter[2] /
    //     100.0f * abs(3 - i);
    // }
    // float n[3] = {0};
    // for(uint8 i=0;i<3;i++){
    //     n[i] = (float)menuSideAngleParameter[i] / 100.0f;
    // }
    // outputKpSet[0] = 1.95f * n[0];
    // outputKpSet[1] = 1.95f * n[0];
    // outputKpSet[2] = 2 * n[0];
    // outputKpSet[3] = 1.2f * n[0];
    // outputKpSet[4] = outputKpSet[2];
    // outputKpSet[5] = outputKpSet[1];
    // outputKpSet[6] = outputKpSet[0];

    // outputKiSet[0] =  n[1];
    // outputKiSet[1] =  n[1];
    // outputKiSet[2] =  n[1];
    // outputKiSet[3] = n[1];
    // outputKiSet[4] = outputKiSet[2];
    // outputKiSet[5] = outputKiSet[1];
    // outputKiSet[6] = outputKiSet[0];

    // outputKdSet[0] = n[2];
    // outputKdSet[1] = n[2];
    // outputKdSet[2] = n[2];
    // outputKdSet[3] = n[2];
    // outputKdSet[4] = outputKdSet[2];
    // outputKdSet[5] = outputKdSet[1];
    // outputKdSet[6] = outputKdSet[0];
    // // fuzzySetInit(&sideBalanceFuzzy, inputSet, outputKpSet, outputKiSet,
    // outputKdSet);
    // // simpleFuzzySetInit(&sideBalanceSimpleFuzzy, inputSet, outputKpSet,
    // outputKiSet, outputKdSet); for(uint8 i=0;i<3;i++){
    //     n[i] = (float)menuFrontAngleParameter[i] / 100.0f;
    // }
    // outputKpSet[0] = 2.5 * n[0];
    // outputKpSet[1] = 2 * n[0];
    // outputKpSet[2] = 1.5 * n[0];
    // outputKpSet[3] = n[0];
    // outputKpSet[4] = outputKpSet[2];
    // outputKpSet[5] = outputKpSet[1];
    // outputKpSet[6] = outputKpSet[0];

    // outputKiSet[0] = 2.5 * n[1];
    // outputKiSet[1] = 2 * n[1];
    // outputKiSet[2] = 1.5 * n[1];
    // outputKiSet[3] = n[1];
    // outputKiSet[4] = outputKiSet[2];
    // outputKiSet[5] = outputKiSet[1];
    // outputKiSet[6] = outputKiSet[0];

    // outputKdSet[0] = 2.5 * n[2];
    // outputKdSet[1] = 2 * n[2];
    // outputKdSet[2] = 1.5 * n[2];
    // outputKdSet[3] = n[2];
    // outputKdSet[4] = outputKdSet[2];
    // outputKdSet[5] = outputKdSet[1];
    // outputKdSet[6] = outputKdSet[0];
    // fuzzySetInit(&frontBalanceFuzzy, inputSet, outputKpSet, outputKiSet,
    // outputKdSet); simpleFuzzySetInit(&frontBalanceSimpleFuzzy, inputSet,
    // outputKpSet, outputKiSet, outputKdSet);
    // }
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
    // turn
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

// static float control_bottom_feedforward() {  // TODO: fix bug
//     static float accZ[2] = {0};
//     accZ[0] = AngleAcceleration;
//     float g = (accZ[0] - accZ[1]) * (float)bucklingFrontCoefficientV * 10.0f;
//     accZ[1] = accZ[0];
//     restrictValueF(&g, 1000, -1000);
//     float ret = (control_target->frontVelocity - vel_motor->bottom) *
//                 (float)bucklingFrontCoefficientV * 0.01f;
//     restrictValueF(&ret, 2800, -2800);
//     return ret - fabsf(g);
// }