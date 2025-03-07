#ifndef _MENU_INPUT_H_
#define _MENU_INPUT_H_

#include "attitude.h"
#include "zf_common_headfile.h"

struct Menu_Manual_Param {
    // Euler Angle
    int32 mechanicalYawAngle;
    int32 mechanicalPitchAngle;
    int32 mechanicalRollAngle;

    int32 bottom_velocity;
    // PID
    uint32 bottom_velocity_parameter[3];
    uint32 bottom_angle_velocity_parameter[3];
    uint32 bottom_angle_parameter[3];

    uint32 side_angle_velocity_parameter[3];
    uint32 side_angle_parameter[3];
    uint32 side_velocity_parameter[3];

    uint32 turn_angle_parameter[3];
    uint32 turn_velocity_parameter[3];

    // 串级pid各个环的时间，时间由长到短，由外到内
    uint32 TurnControlTimeParameter[2];
    uint32 FrontControlTimeParameter[3];
    uint32 SideControlTimeParameter[3];

    uint32 buckingTurnCoefficient;
    uint32 turnGainCoefficient;
    uint32 bucklingFrontCoefficientV;
    uint32 bucklingFrontCoefficientT;
};

struct EulerAngle;
struct Control_Time;
struct Control_Turn_Manual_Params;
struct Control_Motion_Manual_Parmas;

void menu_manual_param_init();
void menu_get_params(
    struct EulerAngle* euler_angle_bias,
    struct Control_Time* control_time,
    struct Control_Turn_Manual_Params* control_turn_params,
    struct Control_Motion_Manual_Parmas* control_motion_params);

extern struct Menu_Manual_Param g_menu_manual_param;
#endif