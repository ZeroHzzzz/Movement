#include "test.h"
#include "attitude.h"
#include "encoder.h"
#include "lcd.h"
#include "menu_input.h"
#include "motor.h"
#include "small_driver_uart_control.h"
#include "velocity.h"
#include "zf_common_headfile.h"

void test_bottom_motor() {
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: forward");
    lcd_show_string(0, 1, "KEY_D: backward");
    lcd_show_string(0, 4, "Press KEY_L to exit");
    while (keymsg.key != KEY_L) {
        if (keymsg.key == KEY_U)  // 向前
        {
            gpio_set_level(DIR_BOTTOM, 1);
            pwm_set_duty(MOTOR_BOTTOM, 8000);
        }
        if (keymsg.key == KEY_D)  // 向后
        {
            gpio_set_level(DIR_BOTTOM, 0);
            pwm_set_duty(MOTOR_BOTTOM, 8000);
        }
        lcd_show_int(0, 5, g_vel_motor.bottom, 5);
        lcd_show_float(0, 6, g_vel_motor.bottomReal, 5, 5);
        lcd_show_float(0, 7, g_vel_motor.bottomFiltered, 5, 5);
        // encoder_clear_count(ENCODER_BOTTOM);
    }
    pwm_set_duty(MOTOR_BOTTOM, 0);
    lcd_clear();
}

// void test_side_motor() {
//     lcd_clear();
//     lcd_show_string(0, 0, "KEY_U: left  forward");
//     lcd_show_string(0, 1, "KEY_D: left backward");
//     lcd_show_string(0, 2, "KEY_B: right forward");
//     lcd_show_string(0, 3, "KEY_R:right backward");
//     lcd_show_string(0, 4, "Press KEY_L to exit");
//     int count = 0;
//     while (keymsg.key != KEY_L) {
//         if (keymsg.key == KEY_U)  // 向前
//         {
//             small_driver_set_duty(1000, 0);
//         }
//         if (keymsg.key == KEY_D)  // 向后
//         {
//             small_driver_set_duty(-1000, 0);
//         }
//         if (keymsg.key == KEY_B)  // 向前
//         {
//             small_driver_set_duty(0, 1000);
//         }
//         if (keymsg.key == KEY_R)  // 向后
//         {
//             small_driver_set_duty(0, -1000);
//         }
//         count++;
//         lcd_show_string(0, 5, "Left:");
//         lcd_show_int(8, 5, motor_value.receive_left_speed_data, 5);
//         lcd_show_string(0, 6, "Right:");
//         lcd_show_int(8, 6, motor_value.receive_right_speed_data, 5);
//         lcd_show_string(0, 7, "count:");
//         lcd_show_int(8, 7, count, 5);
//     }
//     small_driver_set_duty(0, 0);
//     lcd_clear();
// }

void test_side_motor() {
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: left  forward");
    lcd_show_string(0, 1, "KEY_D: left backward");
    lcd_show_string(0, 2, "KEY_B: right forward");
    lcd_show_string(0, 3, "KEY_R:right backward");
    lcd_show_string(0, 4, "Press KEY_L to exit");
    int count = 0;
    while (keymsg.key != KEY_L) {
        if (keymsg.key == KEY_U) {
            small_driver_set_duty(-600, 600);
        } else if (keymsg.key == KEY_D) {
            small_driver_set_duty(-8000, 8000);
        }
    }
    small_driver_set_duty(0, 0);
    lcd_clear();
}

void test_attitude() {
    lcd_clear();
    while (keymsg.key != KEY_L) {
        lcd_show_string(0, 0, "Pitch:");
        lcd_show_float(0, 1, currentFrontAngle, 3, 3);
        lcd_show_string(0, 2, "Row:");
        lcd_show_float(0, 3, currentSideAngle, 3, 3);
        lcd_show_string(0, 4, "Yaw:");
        lcd_show_float(0, 5, yawAngle, 3, 3);
    }
    lcd_clear();
}

void test_imu() {
    lcd_clear();
    while (keymsg.key != KEY_L) {
        lcd_show_string(0, 0, "x:");
        lcd_show_float(0, 1, g_imu_data.gyro.x, 3, 3);
        lcd_show_float(8, 1, g_imu_data.acc.x, 3, 3);
        lcd_show_string(0, 2, "y:");
        lcd_show_float(0, 3, g_imu_data.gyro.y, 3, 3);
        lcd_show_float(8, 3, g_imu_data.acc.y, 3, 3);
        lcd_show_string(0, 4, "z:");
        lcd_show_float(0, 5, g_imu_data.gyro.z, 3, 3);
        lcd_show_float(8, 5, g_imu_data.acc.z, 3, 3);
    }
    lcd_clear();
}