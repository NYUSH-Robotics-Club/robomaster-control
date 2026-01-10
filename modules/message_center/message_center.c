#include "message_center.h"
#include "motor_driver.h"
#include "printing.h"
#include <string.h>

#if defined(USE_HAL_DRIVER)
#include "stm32f4xx_hal.h"
#define MC_HAS_CMSIS 1
#else
#define MC_HAS_CMSIS 0
#endif

#define MC_MAX_SUBS_PER_TOPIC 8

typedef struct {
    MsgCallback cb;
    void *user;
} Subscriber;

static MsgEvent *mc_queue = NULL;
static size_t mc_len = 0;
static volatile size_t mc_head = 0; // write index (next free)
static volatile size_t mc_tail = 0; // read index (oldest)

static Subscriber mc_subs[TOPIC_NUM_TOPICS][MC_MAX_SUBS_PER_TOPIC];
static uint8_t mc_inited = 0;

#if MC_HAS_CMSIS
#define MC_CS_ENTER()           \
    uint32_t _pri = __get_PRIMASK(); __disable_irq()
#define MC_CS_EXIT()            \
    do { if(!_pri) { __enable_irq(); } } while(0)
#else
#define MC_CS_ENTER() do {} while(0)
#define MC_CS_EXIT()  do {} while(0)
#endif

void MsgCenter_Init(MsgEvent *buffer, size_t length) {
    mc_queue = buffer;
    mc_len = (buffer && length) ? length : 0;
    mc_head = 0;
    mc_tail = 0;
    for (size_t t = 0; t < (size_t)TOPIC_NUM_TOPICS; ++t) {
        for (size_t i = 0; i < MC_MAX_SUBS_PER_TOPIC; ++i) {
            mc_subs[t][i].cb = NULL;
            mc_subs[t][i].user = NULL;
        }
    }
    mc_inited = (mc_len > 0);
}

static int mc_is_full(void) {
    size_t next = (mc_head + 1U) % mc_len;
    return (next == mc_tail);
}

static int mc_is_empty(void) {
    return (mc_head == mc_tail);
}

int MsgCenter_Publish(MsgTopic topic, const void *data, size_t size) {
    if (!mc_inited || !mc_queue || mc_len == 0) {
        return -1;
    }
    if ((size > MC_MAX_PAYLOAD) || (topic < 0) || (topic >= TOPIC_NUM_TOPICS)) {
        return -2;
    }

    MC_CS_ENTER();

    // Drop oldest if full (overwrite policy: drop oldest)
    if (mc_is_full()) {
        mc_tail = (mc_tail + 1U) % mc_len;
    }

    MsgEvent *ev = &mc_queue[mc_head];
    ev->topic = (uint16_t)topic;
    ev->size = (uint16_t)size;
    if (data && size) {
        memcpy(ev->data, data, size);
    }

    mc_head = (mc_head + 1U) % mc_len;

    MC_CS_EXIT();
    return 0;
}

int MsgCenter_Subscribe(MsgTopic topic, MsgCallback cb, void *user_data) {
    if (!mc_inited || (topic < 0) || (topic >= TOPIC_NUM_TOPICS) || !cb) {
        return -1;
    }
    for (size_t i = 0; i < MC_MAX_SUBS_PER_TOPIC; ++i) {
        if (mc_subs[topic][i].cb == NULL) {
            mc_subs[topic][i].cb = cb;
            mc_subs[topic][i].user = user_data;
            return 0;
        }
    }
    return -2;
}

void MsgCenter_Dispatch(void) {
    if (!mc_inited || !mc_queue || mc_len == 0) {
        return;
    }

    static uint32_t dispatch_count = 0;
    // Pop and dispatch all pending events
    // TODO: risk of thread safety
    for (;;) {
        MC_CS_ENTER();
        if (mc_is_empty()) {
            MC_CS_EXIT();
            break;
        }
        MsgEvent ev = mc_queue[mc_tail];
        mc_tail = (mc_tail + 1U) % mc_len;
        MC_CS_EXIT();

        dispatch_count++;
        MsgTopic t = (MsgTopic)ev.topic;
        if (t < 0 || t >= TOPIC_NUM_TOPICS) {
            continue;
        }
        for (size_t i = 0; i < MC_MAX_SUBS_PER_TOPIC; ++i) {
            if (mc_subs[t][i].cb) {
                mc_subs[t][i].cb(&ev, mc_subs[t][i].user);
            }
        }
    }
    MotorDriver_FlushAll();
}


