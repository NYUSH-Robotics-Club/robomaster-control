#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <stdint.h>

typedef struct {
    uint16_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanRxFrame;

// Simplified motor feedback parsed from standard motor frames (0x201..0x20B)
typedef struct {
    uint8_t  id;       // same as (StdId - 0x201), 0..7
    uint16_t angle;
    int16_t  speed;
    int16_t  current;
    uint8_t  temp;
    uint32_t tick_ms;
} MotorFeedbackEvent;

// GM6020 specific feedback (angle + speed), format differs from M3508/M2006
typedef struct {
    uint8_t  id;       // 1..7
    uint16_t angle;
    int16_t  speed;
    uint32_t tick_ms;
    int16_t   current; 
} GM6020FeedbackEvent;

// DM10010L feedback: position, velocity, torque, temperatures
typedef struct {
    uint8_t motor_id;  // Motor ID (from D[0] low 4 bits)
    uint8_t err;       // Error status (from D[0] high 4 bits)
    int16_t pos;       // Position (16-bit, D[1-2])
    int16_t vel;       // Velocity (12-bit, D[3-4] bits 11:0)
    int16_t torque;    // Torque (12-bit, D[4-5] bits 15:4 and 3:0)
    uint8_t t_mos;     // MOS temperature (D[6])
    uint8_t t_rotor;   // Rotor temperature (D[7])
    uint32_t timestamp;
} DM10010LFeedbackEvent;

#endif // CAN_COMM_H


