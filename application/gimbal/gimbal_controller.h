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
    float yaw_rate; 
    float yaw_rate_memo;
    float yaw_target_memo;           // Yaw angular rate command (-1.0 to 1.0, normalized)
    bool vision_valid;
    float vision_yaw_err_rad;
    float vision_pitch_err_rad;
    uint32_t vision_ts_ms;
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
void GimbalController_Init(float yaw_kp, float yaw_ki, float yaw_kd, float yaw_initial_angle,
                           float pitch_kp, float pitch_ki, float pitch_kd, float pitch_initial_angle);

void last_data(float last_yaw_rate, float last_yaw_target);

/**
 * @brief Pitch control with normalized rate command
 * @param id Motor ID (7=Pitch)
 * @param rate_normalized Normalized pitch rate (-1.0 to 1.0)
 * @param sensor_data Sensor data pointer
 * @return Motor current command
 */

int16_t GimbalController_PitchControl(uint8_t id, float rate_normalized, SensorData* sensor_data);

/**
 * @brief Yaw control with compensation (chassis rotation + gyro feedback)
 * @param rate_normalized Normalized yaw rate (-1.0 to 1.0)
 * @param sensor_data Sensor data pointer
 * @param use_imu_feedback Use IMU gyro for speed feedback (true for spin mode, false for encoder)
 * @return Motor current command
 */
int16_t GimbalController_YawControlWithCompensation(float rate_normalized, SensorData* sensor_data, bool use_imu_feedback);

/**
 * @brief Target angle correction for yaw (chassis compensation)
 * @param sensor_data Sensor data pointer
 */
void GimbalController_TargetAngleCorrection(SensorData* sensor_data);

/**
 * @brief Initialize gimbal application (message subscriptions and control)
 */
void GimbalApp_Init(void);

/**
 * @brief Wait for gimbal to reach initial alignment position
 * @note This function sends gimbal commands and waits for both yaw and pitch
 *       to reach their initial positions before returning.
 *       Timeout: 10 seconds
 */
void Gimbal_WaitForAlignment(void);


#ifdef __cplusplus
}
#endif

#endif // GIMBAL_CONTROLLER_H

