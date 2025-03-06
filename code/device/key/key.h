#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_headfile.h"

#define KEY_DOWN_TIME 10
#define KEY_HOLD_TIME 50

#define KEY_MSG_FIFO_SIZE 20

typedef enum {
    KEY_U,
    KEY_D,
    KEY_L,
    KEY_R,
    KEY_B,
    KEY_MAX,
} KEY_TYPE;

typedef enum {
    KEY_DOWN = 0,
    KEY_UP = 1,
} KEY_STATUS;

typedef enum {
    KEY_MSG_EMPTY,
    KEY_MSG_NORMAL,
    KEY_MSG_FULL,
} KEY_QUEUE_STATUS;

typedef struct {
    KEY_TYPE key;
    KEY_STATUS status;
} KEY_MSG;

typedef struct {
    KEY_MSG buffer[KEY_MSG_FIFO_SIZE];
    volatile uint8 front, rear;
    volatile KEY_QUEUE_STATUS status;
} KEY_QUEUE;

void key_init_rewrite(KEY_TYPE key);
void key_queue_init(KEY_QUEUE* q);
uint8 key_enqueue(KEY_QUEUE* q, KEY_MSG msg);
uint8 key_dequeue(KEY_QUEUE* q, KEY_MSG* msg);
void key_listener();
void key_update();
KEY_TYPE key_await();

extern volatile KEY_MSG g_key_msg;

#endif