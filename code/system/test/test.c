#include "test.h"
#include "encoder.h"
#include "lcd.h"
#include "menu_input.h"
#include "motor.h"
#include "small_driver_uart_control.h"
#include "zf_common_headfile.h"

void test_bottom_motor() {
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: forward");
    lcd_show_string(0, 1, "KEY_D: backward");
    lcd_show_string(0, 4, "Press KEY_L to exit");
    while (g_key_msg.key != KEY_L) {
        if (g_key_msg.key == KEY_U)  // 向前
        {
            gpio_set_level(DIR_BOTTOM, 1);
            pwm_set_duty(MOTOR_BOTTOM, 1000);
        }
        if (g_key_msg.key == KEY_D)  // 向后
        {
            gpio_set_level(DIR_BOTTOM, 0);
            pwm_set_duty(MOTOR_BOTTOM, 1000);
        }
        lcd_show_string(0, 5, "Now Speed:");
        lcd_show_int(0, 6, encoder_get_count(ENCODER_BOTTOM), 3);
    }
    pwm_set_duty(MOTOR_BOTTOM, 0);
    lcd_clear();
}

void test_side_motor() {
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: left  forward");
    lcd_show_string(0, 1, "KEY_D: left backward");
    lcd_show_string(0, 2, "KEY_B: right forward");
    lcd_show_string(0, 3, "KEY_R:right backward");
    lcd_show_string(0, 4, "Press KEY_L to exit");
    while (g_key_msg.key != KEY_L) {
        if (g_key_msg.key == KEY_U)  // 向前
        {
            small_driver_set_duty(1000, 0);
        }
        if (g_key_msg.key == KEY_D)  // 向后
        {
            small_driver_set_duty(-1000, 0);
        }
        if (g_key_msg.key == KEY_B)  // 向前
        {
            small_driver_set_duty(0, 1000);
        }
        if (g_key_msg.key == KEY_R)  // 向后
        {
            small_driver_set_duty(0, -1000);
        }
        small_driver_get_speed();
        lcd_show_string(0, 5, "Now Speed Left:");
        lcd_show_int(16, 5, motor_value.receive_left_speed_data, 3);
        lcd_show_string(0, 6, "Now Speed Right:");
        lcd_show_int(16, 6, motor_value.receive_right_speed_data, 3);
    }
    small_driver_set_duty(0, 0);
    lcd_clear();
}