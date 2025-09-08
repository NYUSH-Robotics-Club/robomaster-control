#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "remote_control.h"

// Chassis motor count
#define CHASSIS_MOTOR_COUNT 4

// Chassis control parameters
#define CHASSIS_DEMO_TARGET_SPEED 7000
#define CHASSIS_RAMP_STEP 50.0f

// Motor feedback structure
typedef struct {
    uint16_t angle;
    int16_t  speed;
    int16_t  current;
    uint8_t  temp;
    uint32_t last_update_time;
} Motor_Feedback;

// Chassis controller structure
typedef struct {
    // Motor target speeds
    float target_speeds[CHASSIS_MOTOR_COUNT];
    // Smoothed target speeds
    float ramped_targets[CHASSIS_MOTOR_COUNT];
    // Running state
    bool running;
    // PID controllers
    PID_Controller speed_pids[CHASSIS_MOTOR_COUNT];
    // Motor feedbacks
    Motor_Feedback motor_feedbacks[CHASSIS_MOTOR_COUNT];
    // Output currents
    int16_t output_currents[CHASSIS_MOTOR_COUNT];
} ChassisController;

/**
 * @brief Initialize chassis controller
 * @param controller Chassis controller pointer
 */
void ChassisController_Init(ChassisController *controller);

/**
 * @brief Update chassis control logic
 * @param controller Chassis controller pointer
 * @param rc_data Remote control data pointer
 * @param current_tick Current timestamp
 */
void ChassisController_Update(ChassisController *controller, const RC_ctrl_t *rc_data, uint32_t current_tick);

/**
 * @brief Compute chassis motor currents
 * @param controller Chassis controller pointer
 * @param current_tick Current timestamp
 */
void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick);

/**
 * @brief Set chassis target speeds
 * @param controller Chassis controller pointer
 * @param speeds Target speeds array
 */
void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT]);

/**
 * @brief Stop chassis
 * @param controller Chassis controller pointer
 */
void ChassisController_Stop(ChassisController *controller);

/**
 * @brief Get output currents
 * @param controller Chassis controller pointer
 * @return Output currents array pointer
 */
const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller);

/**
 * @brief Check if any motor is running
 * @param controller Chassis controller pointer
 * @return true if any motor is running
 */
bool ChassisController_IsRunning(const ChassisController *controller);

#endif // CHASSIS_CONTROLLER_H
