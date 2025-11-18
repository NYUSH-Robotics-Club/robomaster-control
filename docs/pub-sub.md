# Publish-Subscribe System

## What is Pub-Sub?

The publish-subscribe pattern is like a radio station:
- **Publishers** broadcast messages on specific **topics** (like radio channels)
- **Subscribers** tune in to topics they care about and receive messages automatically
- Publishers and subscribers don't need to know about each other directly

This makes the code more modular - different modules can communicate without tight coupling.

## Message Center Overview

The message center (`modules/message_center/`) is the central hub for all inter-module communication. It uses a ring buffer (circular queue) to store messages and dispatches them to subscribers.

### Key Components

1. **Topics** - Different types of messages (like IMU data, motor feedback, commands)
2. **Events** - The actual message data wrapped in a structure
3. **Subscribers** - Callback functions that get called when a topic is published
4. **Ring Buffer** - A circular queue that stores pending messages

## Available Topics

All topics are defined in `message_center.h`:

```c
typedef enum {
    TOPIC_RC_UPDATE,        // Remote control data
    TOPIC_IMU_UPDATE,            // IMU sensor data
    TOPIC_CAN_RX,               // Raw CAN messages received
    TOPIC_MOTOR_FEEDBACK,       // Motor feedback (angle, speed, etc.)
    TOPIC_GM6020_FEEDBACK,      // GM6020 motor feedback
    TOPIC_CHASSIS_CMD,          // Chassis movement commands
    TOPIC_SHOOT_CMD,            // Shooter commands
    TOPIC_GIMBAL_CMD,           // Gimbal commands
    TOPIC_VISION_DATA,          // Vision system data
    TOPIC_NUM_TOPICS
} MsgTopic;
```

## How to Publish a Message

To publish a message, call `MsgCenter_Publish()`:

```c
// Example: Publishing IMU data
SensorData sensor_data;
// ... fill sensor_data with IMU readings ...

MsgCenter_Publish(TOPIC_IMU_UPDATE, &sensor_data, sizeof(SensorData));
```

**Important**: `MsgCenter_Publish()` is **ISR-safe** - you can call it from interrupt handlers (like CAN receive callbacks) without causing crashes.

### What Happens When You Publish

1. The message center checks if the topic is valid
2. If the ring buffer is full, it drops the oldest message (overwrite policy)
3. The message is copied into the ring buffer
4. The message will be dispatched later when `MsgCenter_Dispatch()` is called

## How to Subscribe to a Topic

To receive messages on a topic, subscribe with a callback function:

```c
// Callback function that gets called when TOPIC_CHASSIS_CMD is published
static void on_chassis_cmd(const MsgEvent *ev, void *user_data) {
    (void)user_data;  // user_data is optional, can be NULL
    
    if (ev->size == sizeof(ChassisCmd)) {
        ChassisCmd cmd;
        memcpy(&cmd, ev->data, sizeof(ChassisCmd));
        
        // Process the command
        // ...
    }
}

// Subscribe in initialization code
void ChassisApp_Init(void) {
    MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
}
```

### Callback Function Signature

```c
typedef void (*MsgCallback)(const MsgEvent *ev, void *user_data);
```

- **`ev`** - The message event containing topic, size, and data
- **`user_data`** - Optional pointer you can pass when subscribing (can be NULL)

## Message Event Structure

Each message is wrapped in a `MsgEvent`:

```c
typedef struct {
    uint16_t topic;              // Which topic this message is for
    uint16_t size;               // Size of data in bytes
    uint8_t  data[MC_MAX_PAYLOAD];  // The actual data (max 64 bytes)
} MsgEvent;
```

## Message Dispatch

Messages are not processed immediately when published. Instead, they are stored in the ring buffer and dispatched later in the main loop:

```c
// In main.c main loop
while (1) {
    // ... other code ...
    
    // Dispatch all pending messages
    MsgCenter_Dispatch();
    
    HAL_Delay(5);
}
```

`MsgCenter_Dispatch()`:
1. Pops messages from the ring buffer (oldest first)
2. Finds all subscribers for that topic
3. Calls each subscriber's callback function
4. Repeats until the buffer is empty

**Why dispatch in main loop?** This ensures callbacks run in a consistent context (not in interrupts), making the code safer and easier to debug.

## Real-World Example: Chassis Controller

Let's see how the chassis controller uses pub-sub:

```c
// 1. Subscribe to topics in initialization
void ChassisApp_Init(void) {
    ChassisController_Init(&s_ctrl);
    
    // Subscribe to chassis commands
    MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
    
    // Subscribe to IMU updates for orientation
    MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    
    // Subscribe to motor feedback
    MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
}

// 2. Callback for chassis commands
static void on_chassis_cmd(const MsgEvent *ev, void *user) {
    if (ev->size == sizeof(ChassisCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
        // Update controller and compute motor currents
        ChassisController_Update(&s_ctrl, &s_last_sensor);
        ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
    }
}

// 3. Callback for IMU updates
static void on_imu_update(const MsgEvent *ev, void *user) {
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

// 4. Callback for motor feedback
static void on_motor_feedback(const MsgEvent *ev, void *user) {
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
        if (m->id < 4) {  // Only process chassis motors
            ChassisController_UpdateMotorFeedback(&s_ctrl, m->id, 
                m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}
```

## Thread Safety

The message center uses interrupt disabling to ensure thread safety:

- **Publishing** (from ISRs): Uses `__disable_irq()` / `__enable_irq()` to protect the ring buffer
- **Dispatching** (from main loop): Also uses interrupt disabling when reading from the buffer

This means you can safely publish from interrupt handlers (like CAN receive callbacks) without worrying about race conditions.

## Limitations

1. **Maximum payload size**: 64 bytes (`MC_MAX_PAYLOAD`)
2. **Maximum subscribers per topic**: 8 (`MC_MAX_SUBS_PER_TOPIC`)
3. **Ring buffer size**: Configurable (default 128 in `main.c`)

If you need larger messages or more subscribers, you can adjust these constants in `message_center.h` and `message_center.c`.

