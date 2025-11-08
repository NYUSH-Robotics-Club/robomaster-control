#ifndef GM6020_MOTOR_H
#define GM6020_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "gyro_data.h"
#include "pid.h"

// Motor context structure (exposed for application layer access)
typedef struct {
    uint8_t   id;
    uint16_t  angle_raw;
    int16_t   speed_rpm;
    float     angle_target;
    uint8_t   angle_inited;
    float     w_chasis_raw;
    float     angle_correction;
    float     angle_correction_ramp;
    PID_Controller speed_pid;
    PID_Controller angle_pid;
    float     phase_offset_rad;
    float     pitch_direction;
    float     gravity_effort;
    float     max_encoder;
    float     target_angle_rad;
    float     angle_min;
    float     angle_max;
    float     joystick_sensitivity;
} GM6020_MotorContext;

// Module layer: Basic motor driver interface
void Motor_Init(uint8_t id, float KP, float KI, float KD, float initial_angle);
void GM6020_Motor_Feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm);

// Getter functions for application layer to access motor state
GM6020_MotorContext* GM6020_GetContext(uint8_t id);
bool GM6020_IsInitialized(uint8_t id);

#endif

