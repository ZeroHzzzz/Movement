#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_headfile.h"

#define KEY_DOWN_TIME 10
#define KEY_HOLD_TIME 11

#define KEY_MSG_FIFO_SIZE 20

typedef enum {
    KEY_U,
    KEY_D,
    KEY_L,
    KEY_R,
    KEY_B,
    KEY_MAX,
} KEY_e;

typedef enum {
    KEY_DOWN = 0,
    KEY_UP = 1,
    KEY_HOLD,
} KEY_STATUS_e;

typedef struct {
    KEY_e key;
    KEY_STATUS_e status;
} KEY_MSG_t;

typedef enum {
    KEY_MSG_EMPTY,
    KEY_MSG_NORMAL,
    KEY_MSG_FULL,
} key_msg_e;

void key_init_rewrite(KEY_e key);
KEY_STATUS_e key_get_status(KEY_e key);
KEY_STATUS_e key_check_status(KEY_e key);

uint8 key_get_msg(KEY_MSG_t* keymsg);
void key_send_msg(KEY_MSG_t keymsg);

void key_IRQHandler();

extern KEY_MSG_t keymsg;

#endif /* _KEY_H_ */
