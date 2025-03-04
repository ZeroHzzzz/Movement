#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_headfile.h"
//===========================================encoder==============================================
#define ENCODER_BOTTOM TIM4_ENCODER
#define ENCODER_PIN0_BOTTOM TIM4_ENCODER_CH1_P02_8
#define ENCODER_PIN1_BOTTOM TIM4_ENCODER_CH2_P00_9

struct Velocity_Motor;

void encoder_init();
void get_momentum_encoder(struct Velocity_Motor* motorVelocity);
void get_bottom_encoder(struct Velocity_Motor* motorVelocity);

#endif