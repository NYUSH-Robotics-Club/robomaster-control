#ifndef YAW_STABILIZER_H
#define YAW_STABILIZER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "remote_control.h"
#include "motor_feedback.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"

typedef struct {
    //
    // Target yaw rate
    float target_yaw_rate;
    // PID controller for yaw stabilization
    PID_Controller yaw_pid;
    // Motor feedback for yaw control
    Motor_Feedback yaw_motor_feedback;
    // Output current for yaw motor
    int16_t output_current;
    // Stabilization enabled flag
    bool spinning;
} YawStabilizer;
