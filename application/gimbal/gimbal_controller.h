#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "gyro_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// Gimbal motor IDs
#define GIMBAL_PITCH_ID 7
#define GIMBAL_YAW_ID   6

// Gimbal command structure
typedef struct {
    bool enabled;              // Gimbal control enabled
    float pitch_rate;          // Pitch angular rate command (-1.0 to 1.0, normalized)
    float yaw_rate;            // Yaw angular rate command (-1.0 to 1.0, normalized)
} GimbalCmd;

/**
 * @brief Initialize gimbal controller
 * @param yaw_kp Yaw PID Kp
 * @param yaw_ki Yaw PID Ki
 * @param yaw_kd Yaw PID Kd
 * @param yaw_initial_angle Yaw initial angle
 * @param pitch_kp Pitch PID Kp
 * @param pitch_ki Pitch PID Ki
 * @param pitch_kd Pitch PID Kd
 * @param pitch_initial_angle Pitch initial angle
 */

/**
 * @brief Pitch control with normalized rate command
 * @param id Motor ID (7=Pitch)
 * @param rate_normalized Normalized pitch rate (-1.0 to 1.0)
 * @param sensor_data Sensor data pointer
 * @return Motor current command
 */
int16_t GimbalController_PitchControl(uint8_t id, float rate_normalized);

/**
 * @brief Yaw control with compensation (chassis rotation + gyro feedback)
 * @param rate_normalized Normalized yaw rate (-1.0 to 1.0)
 * @param sensor_data Sensor data pointer
 * @return Motor current command
 */
int16_t GimbalController_YawControlWithCompensation(float rate_normalized);

/**
 * @brief Target angle correction for yaw (chassis compensation)
 * @param sensor_data Sensor data pointer
 */
void GimbalController_TargetAngleCorrection();

/**
 * @brief Initialize gimbal application (message subscriptions and control)
 */
void GimbalApp_Init(void);

void GimbalComputeCurrent();

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_CONTROLLER_H

