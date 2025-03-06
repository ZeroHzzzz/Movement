#include "menu_input.h"
#include "attitude.h"
#include "control.h"

struct Menu_Manual_Param g_menu_manual_param;

void menu_manual_param_init() {
    memset(&g_menu_manual_param, 0, sizeof(struct Menu_Manual_Param));
}

void menu_get_params(
    struct EulerAngle* euler_angle_bias,
    struct Control_Time* control_time,
    struct Control_Turn_Manual_Params* control_turn_params,
    struct Control_Motion_Manual_Parmas* control_motion_params) {
    // euler angle
    euler_angle_bias->roll = g_menu_manual_param.mechanicalRollAngle;
    euler_angle_bias->pitch = g_menu_manual_param.mechanicalPitchAngle;
    euler_angle_bias->yaw = g_menu_manual_param.mechanicalYawAngle;

    // pid
    for (uint8 i = 0; i < 3; i++) {
        control_motion_params->bottom_velocity_parameter[i] =
            g_menu_manual_param.bottom_velocity_parameter[i];
        control_motion_params->bottom_angle_velocity_parameter[i] =
            g_menu_manual_param.bottom_angle_velocity_parameter[i];
        control_motion_params->bottom_angle_parameter[i] =
            g_menu_manual_param.bottom_angle_parameter[i];

        control_motion_params->side_angle_velocity_parameter[i] =
            g_menu_manual_param.side_angle_velocity_parameter[i];
        control_motion_params->side_angle_parameter[i] =
            g_menu_manual_param.side_angle_parameter[i];
        control_motion_params->side_velocity_parameter[i] =
            g_menu_manual_param.side_velocity_parameter[i];

        control_motion_params->turn_angle_parameter[i] =
            g_menu_manual_param.turn_angle_parameter[i];
        control_motion_params->turn_velocity_parameter[i] =
            g_menu_manual_param.turn_velocity_parameter[i];
    }

    // turn
    control_turn_params->buckling_turn_coefficient =
        (float)g_menu_manual_param.buckingTurnCoefficient * 0.01f;
    control_turn_params->turn_gain_coefficient =
        (float)g_menu_manual_param.turnGainCoefficient * 0.01f;
    control_turn_params->buckling_front_coefficientV =
        g_menu_manual_param.bucklingFrontCoefficientV;
    control_turn_params->buckling_front_coefficientT =
        g_menu_manual_param.bucklingFrontCoefficientT;

    control_time->turn[0] = g_menu_manual_param.TurnControlTimeParameter[0];
    control_time->turn[1] = g_menu_manual_param.TurnControlTimeParameter[1];
    control_time->bottom[0] = g_menu_manual_param.FrontControlTimeParameter[0];
    control_time->bottom[1] = g_menu_manual_param.FrontControlTimeParameter[1];
    control_time->bottom[2] = g_menu_manual_param.FrontControlTimeParameter[2];
    control_time->side[0] = g_menu_manual_param.SideControlTimeParameter[0];
    control_time->side[1] = g_menu_manual_param.SideControlTimeParameter[1];
    control_time->side[2] = g_menu_manual_param.SideControlTimeParameter[2];
}