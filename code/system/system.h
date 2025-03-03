#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "zf_common_headfile.h"
#define LED3_on gpio_init(P33_11, GPO, 0, GPO_PUSH_PULL)
#define LED3_off gpio_init(P33_11, GPO, 1, GPO_PUSH_PULL)
#define LED3_toggle gpio_toggle(P33_11)
#define LED4_on gpio_init(P33_12, GPO, 0, GPO_PUSH_PULL)
#define LED4_off gpio_init(P33_12, GPO, 1, GPO_PUSH_PULL)

#define LED_all_off LED1_off, LED2_off, LED3_off, LED4_off
#define LED_all_on LED1_on, LED2_on, LED3_on, LED4_on

// #define programRunningInfo PRI
#define SW_UP 1 - gpio_get_level(P11_2)
#define SW_DOWN 1 - gpio_get_level(P11_9)
#define SW_LEFT 1 - gpio_get_level(P13_3)
#define SW_RIGHT 1 - gpio_get_level(P11_3)
#define SW_OK 1 - gpio_get_level(P11_6)

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
#define ERROR4   \
    while (1) {  \
        LED4_on; \
    }

void system_init();
void system_attitude_timer();
#endif