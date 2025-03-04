#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h"

#define pidCoefficient 100
#define CONTROL_LAW_CONSTRAINT 0.28f  // simple trace

struct Control_Turn_Manual_Params {
    float buckling_turn_coefficient;  // 屈曲转动系数，这里存放的是已经被除过的
    uint32 buckling_front_coefficientV;  // 前部屈曲系数V
    uint32 buckling_front_coefficientT;  // 前部屈曲系数T
    uint32 turn_gain_coefficient;        // 转弯增益系数
    int32 turnCurvature;                 // 转弯曲率
};

struct Control_Motion_Manual_Parmas {
    // 这是底轮速度
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
};

struct Control_Time {
    // 串级pid各个环的时间，时间由长到短，由外到内
    uint32 turn[2];
    uint32 bottom[3];
    uint32 side[3];
};

struct Control_Target {
    // 控制目标状态

    // front side
    float frontAngle;
    float frontAngleVelocity;
    float frontVelocity;
    // side
    //  float sideVelocity;
    float sideAngle;
    float sideAngleVelocity;
    // turn
    float bucking;   // balance bucking
    float Fbucking;  // front balance bucking
    float turnAngle;
    float turnAngleVelocity;
};

struct Control_Flag {
    // 控制更新标志
    uint8_t frontAngle;
    uint8_t frontAngleVelocity;
    uint8_t frontVelocity;

    uint8_t sideAngle;
    uint8_t sideAngleVelocity;
    uint8_t sideVelocity;

    uint8_t turn;
    uint8_t turnAngle;
    uint8_t turnAngleDiffVelocity;

    uint8 frontAngleCount;
    uint8 frontAngleVelocityCount;
    uint8 frontVelocityCount;

    uint8 sideAngleCount;
    uint8 sideAngleVelocityCount;
    uint8 sideVelocityCount;

    uint8 turnAngleCount;
    uint8 turnAngleDiffVelocityCount;
};

struct EulerAngle;
struct Velocity_Motor;
struct EulerAngle;
struct Menu_Manual_Param;

extern uint8 g_turn_start_flag;
extern struct Control_Turn_Manual_Params g_control_turn_manual_params;
extern struct Control_Target g_control_target;
extern struct Control_Flag g_control_flag;
extern struct Control_Time g_control_time;
extern struct Control_Motion_Manual_Parmas g_control_motion_params;

void control_manual_param_init();
void control_bottom_balance(struct Control_Target* control_target,
                            struct Control_Flag* control_flag,
                            struct Velocity_Motor* vel_motor,
                            struct EulerAngle* euler_angle_bias);
void control_side_balance(
    struct Control_Target* control_target,
    struct Control_Flag* control_flag,
    struct Control_Turn_Manual_Params* control_turn_params,
    struct Velocity_Motor* vel_motor,
    struct EulerAngle* euler_angle_bias);

void control_turn_balance();
void control_shutdown(struct Control_Target* control_target,
                      struct EulerAngle* euler_angle_bias);
#endif