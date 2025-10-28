#ifndef YAW_STABILIZER_H
#define YAW_STABILIZER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "remote_control.h"
#include "motor_feedback.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "main.h"
#include "shooter_controller.h"

 // Target angular velocity (w) when stabilizing
#define TARGET_W_GIMBLE 0.0f
#define MAX_BIAS 0.01f
//The yaw ratio
#define yaw_ratio 0.0f//!!!!!! Replace with actual ratio value!!!!!!
//PID
#define YAW_PID_KP (5.0f)
#define YAW_PID_KI (0.5f)
#define YAW_PID_KD (0.1f)
#define YAW_PID_OUTPUT_MAX (15000)
#define YAW_PID_INTEGRAL_MAX (7500)//!!!!!! Replace with actual value!!!!!!

// Yaw stabilizer structure
typedef struct {
    //w of the head
    float w_gimble_raw;
    //w of the chassis
    float w_chassis_raw;
    //Target speed for yaw stabilization
    float yaw_targetspeed;
    // PID controller for yaw stabilization
    PID_Controller yaw_pid;
    // Motor feedback for yaw control
    float yaw_motor_feedback;
    // Output current for yaw motor
    int16_t output_current;
    // Stabilization enabled flag
    bool stabilizer_enabled;
    
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
void YawStabilizer_Switch(YawStabilizer *stabilizer, bool stabilizer_enable);


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
 * @brief Get angular velocity of the gimble from BMI088
 * @return angular velocity around Z axis (yaw) in deg/s
 */
float Gimble_GetGyroZ()


/**
 * @brief Handle the bias using gimble gyro data
 * @param stabilizer Yaw stabilizer pointer.
 */
void YawStabilizer_Correction(YawStabilizer *stabilizer);


#endif 