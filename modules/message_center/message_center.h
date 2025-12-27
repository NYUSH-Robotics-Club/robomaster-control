#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Topics aligned with plan; extend as needed
typedef enum {
    TOPIC_RC_UPDATE = 0,
    TOPIC_IMU_UPDATE,
    TOPIC_CAN_RX,
    TOPIC_MOTOR_FEEDBACK,
    TOPIC_GM6020_FEEDBACK,
    TOPIC_CHASSIS_CMD,
    TOPIC_SHOOT_CMD,
    TOPIC_GIMBAL_CMD,
    TOPIC_VISION_DATA,     // Vision data from upper computer
    TOPIC_NUM_TOPICS
} MsgTopic;

#ifndef MC_MAX_PAYLOAD
#define MC_MAX_PAYLOAD 128
#endif

typedef struct {
    uint16_t topic;     // MsgTopic
    uint16_t size;      // bytes used in data
    uint8_t  data[MC_MAX_PAYLOAD];
} MsgEvent;

typedef void (*MsgCallback)(const MsgEvent *ev, void *user_data);

// Initialize message center with user-provided ring buffer
void MsgCenter_Init(MsgEvent *buffer, size_t length);

// Publish event (ISR-safe). Returns 0 on success.
int MsgCenter_Publish(MsgTopic topic, const void *data, size_t size);

// Subscribe to a topic; returns 0 on success.
int MsgCenter_Subscribe(MsgTopic topic, MsgCallback cb, void *user_data);

// Dispatch pending events; call in main loop/tick.
void MsgCenter_Dispatch(void);

#ifdef __cplusplus
}
#endif

#endif // MESSAGE_CENTER_H


