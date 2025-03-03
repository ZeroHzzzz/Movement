#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h"

typedef struct {
    float buckling_turn_coefficient;  // 屈曲转动系数，这里存放的是已经被除过的
    uint32 buckling_front_coefficientV;  // 前部屈曲系数V
    uint32 buckling_front_coefficientT;  // 前部屈曲系数T
    uint32 turn_gain_coefficient;        // 转弯增益系数
} Control_Turn_Manual_Params;

typedef struct {
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
} Control_Target;

typedef struct {
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

} Control_Flag;

extern uint8 g_turn_start_flag;
extern Control_Turn_Manual_Params g_turn_manual_params;
extern Control_Target g_control_target;
extern Control_Flag g_control_flag;

#endif