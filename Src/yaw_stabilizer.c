#include "yaw_stabilizer.h"
#include "shooter_controller.h"
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
    PID_Init(&stabilizer->yaw_pid, YAW_PID_KP, YAW_PID_KI,
         YAW_PID_KD, YAW_PID_OUTPUT_MAX, YAW_PID_INTEGRAL_MAX);

    // Initialize motor feedback
    memset(&stabilizer->yaw_motor_feedback, 0, sizeof(Motor_Feedback));

    // Initialize w records
    stabilizer->w_gimble_raw = 0.0f;
    stabilizer->w_chassis_raw = 0.0f;
}



/**
 * @brief Get angular velocity of the gimble from BMI088
 * @return angular velocity around Z axis (yaw) in deg/s
 */
void Chasis_GetGyroZ(YawStabilizer *stabilizer)
{
   &stabilizer->w_chasis_raw=BMI088_read(gyro[3])
    return ;
}


/**
 * @brief Update yaw status and do initial stabilization.
 * @param stabilizer Yaw stabilizer pointer.
 * @param current_tick Current timestamp (ms).
 */
void YawStabilizer_Update(YawStabilizer*stabilizer, uint32_t current_tick,bool stabilizer_enable,const RC_ctrl_t *rc_data);
{
    
//Enable or disable yaw stabilization
 
if (rc_data==NULL)
    {
        stabilizer->stabilizer_enabled = false;
        return;
    }
 if(rc_data!=NULL)
 {  // Check left switch position: down/off, mid/preheat (shooters only), up/all on
    bool right_switch_up = false;
    bool right_switch_mid = false;
    bool right_switch_down = false;
        right_switch_up = switch_is_up(rc_data->rc.s[0]);//!!!!!! Replace with actual switch index!!!!!!
        right_switch_mid = switch_is_mid(rc_data->rc.s[0]);
        right_switch_down = switch_is_down(rc_data->rc.s[0]);
 }
    // Enabled if any shooter/turntable should run (mid or up)
    stabilizer->stabilizer_enabled = right_switch_mid; 
    &stabilizer->yaw_targetspeed=stabilizer->w_chasis_raw * yaw_ratio
    &stabilizer->yaw_motor_feedback =motor_feedbacks[motor_id].speed ;//!!!!!!Replce motor_id with yaw motor id!!!!!!

}

/**
 * @brief Compute yaw motor current using PID 
 * @param stabilizer Yaw stabilizer pointer.
 */
void YawStabilizer_ComputeCurrent(YawStabilizer *stabilizer)
{
    if (stabilizer == NULL) return;
    
    // Compute yaw motor current using PID
    stabilizer->output_current = (int16_t)PID_Calculate(
        &stabilizer->yaw_pid,
        stabilizer->yaw_targetspeed,
        stabilizer->yaw_motor_feedback
        &controller->gimbal_yaw_current=stabilizer->output_current
    );

}
