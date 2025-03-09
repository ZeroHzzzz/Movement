#include "motor.h"
#include "small_driver_uart_control.h"

uint32 bottom_motor_deadzone = 0;

void motor_init() {
    gpio_init(P22_3, GPO, 1, GPO_PUSH_PULL);
    gpio_set_level(P22_3, 1);

    pwm_init(MOTOR_BOTTOM, MOTOR_HZ, 0);
    gpio_init(DIR_BOTTOM, GPO, 1, GPO_PUSH_PULL);
}

// bottom motor
void set_bottom_motor_pwn(int32 pwm) {
    restrictValueI(&pwm, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    if (pwm >= 0) {
        gpio_set_level(DIR_BOTTOM, 0);
        pwm_set_duty(MOTOR_BOTTOM, pwm);
    } else {
        gpio_set_level(DIR_BOTTOM, 1);
        pwm_set_duty(MOTOR_BOTTOM, -pwm);
    }
}

void set_bottom_motor_hertz(int32 hertz) {
    restrictValueI(&hertz, MOTOR_HZ + MOTOR_HZ_RANGE,
                   MOTOR_HZ - MOTOR_HZ_RANGE);
    pwm_init(MOTOR_BOTTOM, hertz, 0);
}

void stop_bottom_motor(void) {
    set_bottom_motor_pwn(0);
}

// momentum motor
void set_momentum_motor_pwm(int32 pwmFront, int32 pwmBack) {
    // WARN: 正负值可能需要调整
    restrictValueI(&pwmFront, MOMENTUM_MOTOR_PWM_MIN, MOMENTUM_MOTOR_PWM_MAX);
    restrictValueI(&pwmBack, MOMENTUM_MOTOR_PWM_MIN, MOMENTUM_MOTOR_PWM_MAX);
    printf("%d, %d\n", pwmFront, pwmBack);
    small_driver_set_duty(pwmFront, pwmBack);
}

void stop_momentum_motor(void) {
    small_driver_set_duty(0, 0);
}