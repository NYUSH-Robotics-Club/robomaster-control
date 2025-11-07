#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "gyro_data.h"

// Gimbal motor IDs
#define GIMBAL_PITCH_ID 7
#define GIMBAL_YAW_ID   6

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

/**
 * @brief Joystick control for gimbal motor
 * @param id Motor ID (6=Yaw, 7=Pitch)
 * @param joystick_ch1 Joystick channel 1 value
 * @param sensor_data Sensor data pointer
 * @return Motor current command
 */
int16_t GimbalController_JoystickControl(uint8_t id, int16_t joystick_ch1, SensorData* sensor_data);

/**
 * @brief Yaw control with compensation (chassis rotation + gyro feedback)
 * @param joystick_yaw Joystick yaw input
 * @param sensor_data Sensor data pointer
 * @return Motor current command
 */
int16_t GimbalController_YawControlWithCompensation(int16_t joystick_yaw, SensorData* sensor_data);

/**
 * @brief Target angle correction for yaw (chassis compensation)
 * @param sensor_data Sensor data pointer
 */
void GimbalController_TargetAngleCorrection(SensorData* sensor_data);

#endif // GIMBAL_CONTROLLER_H

