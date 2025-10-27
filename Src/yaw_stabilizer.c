#include "yaw_stabilizer.h"
#include "can.h"
#include "can_manager.h"
#include <string.h>

/**
 * @brief Initialize yaw stabilizer
 * @param stabilizer yaw stabilizer pointer
 */
void YawStabilizer_Init(YawStabilizer *stabilizer);
{
    if (stabilizer == NULL) return;
    //initialize all members to zero
    memset(stabilizer, 0, sizeof(YawStabilizer));

    // Initialize PID controller for yaw stabilization
    PID_Init(&stabilizer->yaw_pid, SPEED_PID_KP, SPEED_PID_KI,
         SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);

    // Initialize motor feedback
    memset(&stabilizer->yaw_motor_feedback, 0, sizeof(Motor_Feedback));

    // Initialize w records
    stabilizer->w_gimble = 0.0f;
    stabilizer->w_chassis = 0.0f;
}


/**
 * @brief Get angular velocity of the gimble from BMI088
 * @return angular velocity around Z axis (yaw) in deg/s
 */
float Gimble_GetGyroZ()
{
    // Placeholder: Replace with actual BMI088 reading code
    return ; // Replace with actual value
}




/**
 * @brief Get angular velocity of the chasis from......
 * @return angular velocity around Z axis (yaw) in deg/s
 */
float Chasis_GetGyroZ(){
    // Placeholder: Replace with actual chassis gyro reading code
    return ; // Replace with actual value 
}




/**
 * @brief Update yaw status and do initial stabilization.
 * @param stabilizer Yaw stabilizer pointer.
 * @param current_tick Current timestamp (ms).
 */
void YawStabilizer_Update(YawStabilizer*stabilizer, uint32_t current_tick);
{
    if (stabilizer == NULL) return;

    // Read angular velocity from BMI088 gyro on the chasis(Z axis)
    (int16_t) &stabilizer->w_chasis = Gimble_GetGyroZ();
    // Scale to target speed units
    const float scale = (float)CHASSIS_DEMO_TARGET_SPEED / (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET); // 7000/660(supposed)
    (int16_t) &stabilizer->targetspeed= (-w_chasis * scale)

}
    

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
