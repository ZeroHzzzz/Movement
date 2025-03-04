#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "zf_common_headfile.h"
#define LED1_on gpio_init(P21_5, GPO, 0, GPO_PUSH_PULL)
#define LED1_off gpio_init(P21_5, GPO, 1, GPO_PUSH_PULL)
#define LED2_on gpio_init(P20_8, GPO, 0, GPO_PUSH_PULL)
#define LED2_off gpio_init(P20_8, GPO, 1, GPO_PUSH_PULL)
#define LED3_on gpio_init(P20_9, GPO, 0, GPO_PUSH_PULL)
#define LED3_off gpio_init(P20_9, GPO, 1, GPO_PUSH_PULL)

#define LED_all_off LED1_off, LED2_off, LED3_off
#define LED_all_on LED1_on, LED2_on, LED3_on

// #define programRunningInfo PRI

#define ERROR1   \
    while (1) {  \
        LED1_on; \
    }
#define ERROR2   \
    while (1) {  \
        LED2_on; \
    }
#define ERROR3   \
    while (1) {  \
        LED3_on; \
    }

void system_init();
void system_attitude_timer();
#endif