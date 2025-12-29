#ifndef SWERVE_CHASSIS_CONTROLLER_H
#define SWERVE_CHASSIS_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "main.h"
#include "remote_control.h"
#include "motor_feedback.h"
#include "gyro_data.h"
#include "config_types.h"

// Swerve module count (diagonal layout: 2 active modules)
#define SWERVE_MODULE_COUNT 2

// Swerve chassis control parameters
#define SWERVE_DEMO_TARGET_SPEED 7000
#define SWERVE_MAX_STEERING_ANGLE (2.0f * M_PI)  // Full rotation

/**
 * @brief Single swerve module controller
 * Each module has:
 * - 1x GM6020 steering motor (angle control)
 * - 1x M3508 drive motor (speed control)
 */
typedef struct {
    // Target values
    float steering_angle_target;  // Target steering angle in encoder units
    float drive_speed_target;     // Target drive speed in RPM

    // Motor IDs
    uint8_t steering_motor_id;    // GM6020 motor ID
    uint8_t drive_motor_id;       // M3508 motor ID

    // PID controllers
    PID_Controller steering_angle_pid;  // Steering angle control
    PID_Controller drive_speed_pid;     // Drive speed control

    // Motor feedbacks
    Motor_Feedback steering_feedback;
    Motor_Feedback drive_feedback;

    // Output currents
    int16_t steering_current;
    int16_t drive_current;
} SwerveModule_t;

/**
 * @brief Swerve chassis controller structure
 * For diagonal swerve drive with 2 modules:
 * - Module 0: Front-left
 * - Module 1: Back-right
 */
typedef struct {
    // Swerve modules
    SwerveModule_t modules[SWERVE_MODULE_COUNT];

    // Running state
    bool running;

    // Chassis velocity commands (chassis frame)
    float vx_target;  // Forward velocity (normalized)
    float vy_target;  // Lateral velocity (normalized)
    float wz_target;  // Rotation velocity (normalized)
} SwerveChassisController;

/**
 * @brief Initialize swerve chassis controller
 * @param controller Swerve chassis controller pointer
 */
void SwerveChassisController_Init(SwerveChassisController *controller);

/**
 * @brief Update swerve chassis control logic
 * @param controller Swerve chassis controller pointer
 * @param sensor_data Sensor data pointer
 */
void SwerveChassisController_Update(SwerveChassisController *controller, SensorData* sensor_data);

/**
 * @brief Compute swerve module motor currents
 * @param controller Swerve chassis controller pointer
 * @param current_tick Current timestamp
 */
void SwerveChassisController_ComputeCurrents(SwerveChassisController *controller, uint32_t current_tick);

/**
 * @brief Stop swerve chassis
 * @param controller Swerve chassis controller pointer
 */
void SwerveChassisController_Stop(SwerveChassisController *controller);

/**
 * @brief Check if swerve chassis is running
 * @param controller Swerve chassis controller pointer
 * @return true if chassis is running
 */
bool SwerveChassisController_IsRunning(const SwerveChassisController *controller);

/**
 * @brief Update motor feedback (to be called from CAN receive path)
 * @param controller Swerve chassis controller pointer
 * @param motor_id Motor ID
 * @param angle Encoder angle
 * @param speed Speed (RPM)
 * @param current Current
 * @param temp Temperature
 * @param current_tick Current timestamp
 */
void SwerveChassisController_UpdateMotorFeedback(SwerveChassisController *controller,
                                                  uint8_t motor_id,
                                                  uint16_t angle,
                                                  int16_t speed,
                                                  int16_t current,
                                                  uint8_t temp,
                                                  uint32_t current_tick);

/**
 * @brief Initialize swerve chassis app
 */
void SwerveChassisApp_Init(void);

/**
 * @brief Get swerve chassis controller instance
 * @return Pointer to controller
 */
SwerveChassisController* SwerveChassisApp_GetController(void);

#endif // SWERVE_CHASSIS_CONTROLLER_H
