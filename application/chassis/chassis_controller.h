#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "main.h"
#include "remote_control.h"
#include "motor_feedback.h"
#include "gyro_data.h"

// Chassis motor count
#define CHASSIS_MOTOR_COUNT 4

// Chassis control parameters
#define CHASSIS_DEMO_TARGET_SPEED 7000
#define CHASSIS_RAMP_STEP 50.0f


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
    PID_Controller angle_pids[2];

} ChassisController;



/**
 * @brief Initialize chassis controller
 * @param controller Chassis controller pointer
 */
void ChassisController_Init(ChassisController *controller);

/**
 * @brief Update chassis control logic
 * @param controller Chassis controller pointer
 * @param sensor_data Sensor data pointer
 */
void ChassisController_Update(ChassisController *controller, SensorData* sensor_data);

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

/**
 * @brief Update motor feedback (to be called from CAN receive path)
 * @param controller Chassis controller pointer
 * @param motor_id Motor ID in range 0..3
 * @param angle Encoder angle
 * @param speed Speed (RPM)
 * @param current Current
 * @param temp Temperature
 * @param current_tick Current timestamp
 */
void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick);

 typedef float fp32;
 typedef double fp64;
 extern fp32 vx_set, vy_set, wz_set;
 extern fp32 PID_CurrentLT1, PID_CurrentLT2, PID_CurrentRT1, PID_CurrentRT2;
 extern fp32 M3508_SPEED[4], MS7010_ANGLE[4];
 float Angle_Limit(float angle, float max);
 void chassis_vector_to_M3508_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4]);
 void chassis_vector_to_GM6020_wheel_angle(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4]);
 void chassis_cmd_calc(void);
 #endif // CHASSIS_CONTROLLER_H

