#include "key.h"
#include "zf_common_headfile.h"

KEY_MSG_t keymsg = {KEY_B, KEY_UP};
gpio_pin_enum KEY_PTxn[KEY_MAX] = {P13_2, P13_1, P13_3, P14_6, P11_2};

KEY_MSG_t key_msg[KEY_MSG_FIFO_SIZE];
volatile uint8 key_msg_front = 0, key_msg_rear = 0;
volatile uint8 key_msg_flag = KEY_MSG_EMPTY;

void key_init_rewrite(KEY_e key) {
    if (key < KEY_MAX) {
        gpio_init(KEY_PTxn[key], GPI, 0, GPO_PUSH_PULL);
    } else {
        key = KEY_MAX;
        while (key--) {
            gpio_init(KEY_PTxn[key], GPI, 0, GPO_PUSH_PULL);
        }
    }
}

KEY_STATUS_e key_get_status(KEY_e key) {
    if (gpio_get_level(KEY_PTxn[key]) == KEY_DOWN) {
        return KEY_DOWN;
    }
    return KEY_UP;
}

KEY_STATUS_e key_check_status(KEY_e key) {
    if (key_get_status(key) == KEY_DOWN) {
        system_delay_ms(10);
        if (key_get_status(key) == KEY_DOWN) {
            return KEY_DOWN;
        }
    }
    return KEY_UP;
}

uint8 key_get_msg(KEY_MSG_t* keymsg) {
    uint8 tmp;

    if (key_msg_flag == KEY_MSG_EMPTY) {
        return 0;
    }

    keymsg->key = key_msg[key_msg_front].key;
    keymsg->status = key_msg[key_msg_front].status;

    key_msg_front++;

    if (key_msg_front >= KEY_MSG_FIFO_SIZE) {
        key_msg_front = 0;
    }
    tmp = key_msg_rear;
    if (key_msg_front == tmp) {
        key_msg_flag = KEY_MSG_EMPTY;
    } else {
        key_msg_flag = KEY_MSG_NORMAL;
    }

    return 1;
}

void key_send_msg(KEY_MSG_t keymsg) {
    uint8 tmp;

    if (key_msg_flag == KEY_MSG_FULL) {
        return;
    }
    key_msg[key_msg_rear].key = keymsg.key;
    key_msg[key_msg_rear].status = keymsg.status;

    key_msg_rear++;

    if (key_msg_rear >= KEY_MSG_FIFO_SIZE) {
        key_msg_rear = 0;
    }

    tmp = key_msg_rear;
    if (tmp == key_msg_front) {
        key_msg_flag = KEY_MSG_FULL;
    } else {
        key_msg_flag = KEY_MSG_NORMAL;
    }
}

void key_IRQHandler(void) {
    KEY_e keynum;
    static uint8 keytime[KEY_MAX] = {0};
    KEY_MSG_t keymsg;
    for (keynum = (KEY_e)0; keynum < KEY_MAX; keynum++) {
        if (key_get_status(keynum) == KEY_DOWN) {
            keytime[keynum]++;
            if (keytime[keynum] <= KEY_DOWN_TIME) {
                continue;
            } else if (keytime[keynum] <= KEY_HOLD_TIME) {
                keymsg.key = keynum;
                keymsg.status = KEY_DOWN;
                key_send_msg(keymsg);
            }
            //  else {
            //     keymsg.key = keynum;
            //     keymsg.status = KEY_HOLD;
            //     send_key_msg(keymsg);
            // }
        } else {
            if (keytime[keynum] > KEY_DOWN_TIME) {
                keymsg.key = keynum;
                keymsg.status = KEY_UP;
                key_send_msg(keymsg);
            }
            keytime[keynum] = 0;
        }
    }
}
