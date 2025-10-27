#ifndef YAW_STABILIZER_H
#define YAW_STABILIZER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "remote_control.h"
#include "motor_feedback.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"

 // Target angular velocity (w) when stabilizing
#define TARGET_W_HEAD 0.0f
// Chassis control parameters
#define CHASSIS_DEMO_TARGET_SPEED 7000
#define CHASSIS_RAMP_STEP 50.0f

// Yaw stabilizer structure
typedef struct {
    //w of the head
    float w_gimble;
    //w of the chassis
    float w_chassis;
    //Target speed for yaw stabilization
    float targetspeed;
    // PID controller for yaw stabilization
    PID_Controller yaw_pid;
    // Motor feedback for yaw control
    Motor_Feedback yaw_motor_feedback;
    // Output current for yaw motor
    int16_t output_current;
    // Stabilization enabled flag
    bool spinning;
} YawStabilizer;


/**
 * @brief Initialize yaw stabilizer
 * @param stabilizer yaw stabilizer pointer
 */
void YawStabilizer_Init(YawStabilizer *stabilizer);

/**
 * @brief Enable or disable yaw stabilization
 * @param stabilizer yaw stabilizer pointer
 */
void YawStabilizer_Switch(YawStabilizer *stabilizer, bool enable);

/**
 * @brief Get angular velocity of the gimble from BMI088
 * @return angular velocity around Z axis (yaw) in deg/s
 */
float Gimble_GetGyroZ()

/**
 * @brief Get angular velocity of the chasis from......
 * @return angular velocity around Z axis (yaw) in deg/s
 */
float Chasis_GetGyroZ()

/**
 * @brief Update yaw status and do initial stabilization.
 * @param stabilizer Yaw stabilizer pointer.
 * @param current_tick Current timestamp (ms).
 */
void YawStabilizer_Update(YawStabilizer *stabilizer, uint32_t current_tick);


/**
 * @brief Compute yaw motor current using PID and send via CAN
 * @param stabilizer Yaw stabilizer pointer.
 */
void YawStabilizer_ComputeCurrent(YawStabilizer *stabilizer);

/**
 * @brief Handle the bias using gimble gyro data
 * @param stabilizer Yaw stabilizer pointer.
 */
void YawStabilizer_Correction(YawStabilizer *stabilizer);


/**
 * @brief Update one motor's feedback from CAN receive path.
 * @param stabilizer Yaw stabilizer pointer.
 * @param motor_id Motor index in range 0..3.
 * @param angle Encoder angle.
 * @param speed Motor speed (RPM).
 * @param current Motor current.
 * @param temp Motor temperature.
 * @param current_tick Timestamp when feedback was received (ms).
 */
void Yawstabilizer_UpdateMotorFeedback(YawStabilizer *stabilizer , uint8_t motor_id,
     uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (stabilizer == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
    
    controller->motor_feedbacks[motor_id].angle = angle;
    controller->motor_feedbacks[motor_id].speed = speed;
    controller->motor_feedbacks[motor_id].current = current;
    controller->motor_feedbacks[motor_id].temp = temp;
    controller->motor_feedbacks[motor_id].last_update_time = current_tick;
}


#endif 