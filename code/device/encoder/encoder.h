#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "velocity.h"
#include "zf_common_headfile.h"
//===========================================encoder==============================================
#define MOMENTUM_ENCODER_FRONT TIM2_ENCODER
#define ENCODER_PIN0_FRONT TIM2_ENCODER_CH1_P33_7
#define ENCODER_PIN1_FRONT TIM2_ENCODER_CH2_P33_6

#define MOMENTUM_ENCODER_BACK TIM5_ENCODER
#define ENCODER_PIN0_BACK TIM5_ENCODER_CH1_P10_3
#define ENCODER_PIN1_BACK TIM5_ENCODER_CH2_P10_1

#define ENCODER_BOTTOM TIM4_ENCODER
#define ENCODER_PIN0_BOTTOM TIM4_ENCODER_CH1_P02_8
#define ENCODER_PIN1_BOTTOM TIM4_ENCODER_CH2_P00_9

void encoder_init();
void get_momentum_encoder(Velocity_Motor* motorVelocity);
void get_bottom_encoder(Velocity_Motor* motorVelocity);

#endif