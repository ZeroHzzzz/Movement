#include "key.h"

gpio_pin_enum KEY_PTxn[KEY_MAX] = {P13_2, P13_1, P13_3, P14_6, P11_2};

KEY_QUEUE key_msg_queue = {{0}, 0, 0, KEY_MSG_EMPTY};
volatile KEY_MSG g_key_msg;

static KEY_STATUS key_get_status(KEY_TYPE key) {
    return gpio_get_level(KEY_PTxn[key]) == KEY_DOWN ? KEY_DOWN : KEY_UP;
}

static KEY_STATUS key_check_status(KEY_TYPE key) {
    if (key_get_status(key) == KEY_DOWN) {
        system_delay_ms(14);
        if (key_get_status(key) == KEY_DOWN) {
            return KEY_DOWN;
        }
    }
    return KEY_UP;
}

void key_init_rewrite(KEY_TYPE key) {
    if (key < KEY_MAX) {
        gpio_init(KEY_PTxn[key], GPI, 0, GPO_PUSH_PULL);
    } else {
        key = KEY_MAX;
        while (key--) {
            gpio_init(KEY_PTxn[key], GPI, 0, GPO_PUSH_PULL);
        }
    }

    // init key queue
    key_queue_init(&key_msg_queue);
}

void key_queue_init(KEY_QUEUE* q) {
    q->front = q->rear = 0;
    q->status = KEY_MSG_EMPTY;
}

uint8 key_enqueue(KEY_QUEUE* q, KEY_MSG msg) {
    if (q->status == KEY_MSG_FULL)
        return 0;
    q->buffer[q->rear] = msg;
    q->rear = (q->rear + 1) % KEY_MSG_FIFO_SIZE;
    q->status = (q->front == q->rear) ? KEY_MSG_FULL : KEY_MSG_NORMAL;
    return 1;
}

uint8 key_dequeue(KEY_QUEUE* q, KEY_MSG* msg) {
    if (q->status == KEY_MSG_EMPTY)
        return 0;
    *msg = q->buffer[q->front];
    q->front = (q->front + 1) % KEY_MSG_FIFO_SIZE;
    q->status = (q->front == q->rear) ? KEY_MSG_EMPTY : KEY_MSG_NORMAL;
    return 1;
}

void key_listener() {
    for (KEY_TYPE keynum = 0; keynum < KEY_MAX; keynum++) {
        static uint8 keytime[KEY_MAX] = {0};
        KEY_STATUS status = key_check_status(keynum);
        if (status == KEY_DOWN) {
            keytime[keynum]++;
            if (keytime[keynum] > KEY_DOWN_TIME) {
                KEY_MSG msg = {keynum, KEY_DOWN};
                key_enqueue(&key_msg_queue, msg);
            }
        } else {
            if (keytime[keynum] > KEY_DOWN_TIME) {
                KEY_MSG msg = {keynum, KEY_UP};
                key_enqueue(&key_msg_queue, msg);
            }
            keytime[keynum] = 0;
        }
    }
}

void key_update() {
    key_dequeue(&key_msg_queue, &g_key_msg);
}

KEY_TYPE key_await() {
    // 感觉这么写才对，等待过程中应该出队，因为中断会定时入队
    // 同时不能忙等待，否则阻塞主进程的，key_msg不能更新
    while (g_key_msg.status == KEY_UP) {
        key_update();
    }
    g_key_msg.status = KEY_UP;
    return g_key_msg.key;
}