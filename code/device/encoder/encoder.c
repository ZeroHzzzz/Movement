#include "encoder.h"
#include "zf_common_headfile.h"

static void pass_momentum_encoder(int32 frontValue,
                                  int32 backValue,
                                  Velocity_Motor* motorVelocity) {
    motorVelocity->momentumFront = -frontValue;
    motorVelocity->momentumBack = -backValue;
    motorVelocity->velocityDiff =
        motorVelocity->momentumFront + motorVelocity->momentumBack;
}

static void pass_bottom_encoder(int32 value, Velocity_Motor* motorVelocity) {
    // filter the noise
    if (abs(value) < 5) {
        value = 0;
    } else {
        value > 0 ? (value -= 5) : (value += 5);
    }
    motorVelocity->bottom = value;
    motorVelocity->bottomSum += value;
}

void encoder_init() {
    // bottom encoder
    encoder_dir_init(ENCODER_BOTTOM, ENCODER_PIN0_BOTTOM, ENCODER_PIN1_BOTTOM);
    // momentum encoder
    // front momentum encoder
    encoder_dir_init(MOMENTUM_ENCODER_FRONT, ENCODER_PIN0_FRONT,
                     ENCODER_PIN1_FRONT);
    // back momentum encoder
    encoder_dir_init(MOMENTUM_ENCODER_BACK, ENCODER_PIN0_BACK,
                     ENCODER_PIN1_BACK);
}

void get_momentum_encoder(Velocity_Motor* motorVelocity) {
    static int32 frontEncoder[3] = {0, 0, 0};
    static int32 backEncoder[3] = {0, 0, 0};
    frontEncoder[2] = frontEncoder[1];
    frontEncoder[1] = frontEncoder[0];
    backEncoder[2] = backEncoder[1];
    backEncoder[1] = backEncoder[0];
    int32 frontEncoderTemp = encoder_get_count(MOMENTUM_ENCODER_FRONT);
    int32 backEncoderTemp = encoder_get_count(MOMENTUM_ENCODER_BACK);
    // {
    //     tft180_show_int(0,20,frontEncoderTemp,3);
    //     tft180_show_int(0,40,backEncoderTemp,3);
    // }
    encoder_clear_count(MOMENTUM_ENCODER_FRONT);
    encoder_clear_count(MOMENTUM_ENCODER_BACK);
    // lowPass filter
    pass_momentum_encoder(frontEncoderTemp, backEncoderTemp, motorVelocity);
    // frontEncoder[0] = (int32)(0.879*(float)frontEncoderTemp +/tvfg5rf
    // 0.121*(float)frontEncoder[1]); backEncoder[0] =
    // (int32)(0.879*(float)backEncoderTemp +
    // 0.121*(float)backEncoder[1]);rsedrs
    // passMomentumEncoder(frontEncoder[0],backEncoder[0]);
}

void get_bottom_encoder(Velocity_Motor* motorVelocity) {
    // static int32 bottomEncoder[3] = {0,0,0};
    // bottomEncoder[2] = bottomEncoder[1];
    // bottomEncoder[1] = bottomEncoder[0];
    // int32 bottomEncoderTemp = encoder_get_count(ENCODER_BOTTOM);
    // // {
    // //     tft180_show_int(0,0,bottomEncoderTemp,3);
    // // }
    // encoder_clear_count(ENCODER_BOTTOM);
    // //lowPass filter
    // bottomEncoder[0] = (int32)(0.879*(float)bottomEncoderTemp +
    // 0.121*(float)bottomEncoder[1]); passBottomEncoder(bottomEncoder[0]);

    int32 bottom = -encoder_get_count(ENCODER_BOTTOM);

    encoder_clear_count(ENCODER_BOTTOM);
    pass_bottom_encoder(bottom, motorVelocity);
}
