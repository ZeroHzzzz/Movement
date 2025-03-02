#include "system.h"
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