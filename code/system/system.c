#include "system.h"
#include "Attitude.h"
#include "control.h"
#include "velocity.h"
#include "zf_common_headfile.h"

void system_init() {
    // init lcd
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();

    // init key
    key_init_rewrite(KEY_MAX);
    pit_ms_init(CCU60_CH1, 4);

    // init motor
    motor_init();

    // init encoder
    encoder_init();
}

void system_attitude_timer() {
    static uint8 imuCnt = 0;
    imuCnt++;
    if (imuCnt >= 2) {
        imuCnt = 0;
        g_attitude_cal_flag = 1;
        attitude_cal_amend(&g_turn_manual_params, &g_control_target,
                           &g_vel_motor, &g_euler_angle);
    } else {
        g_attitude_cal_flag = 0;
    }
}