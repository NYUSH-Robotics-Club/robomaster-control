#include "shooter_controller.h"
#include "can.h"
#include "can_manager.h"
#include <string.h>


// External CAN handles
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;

// PID parameters
#define SPEED_PID_KP (5.0f)
#define SPEED_PID_KI (0.5f)
#define SPEED_PID_KD (0.1f)
#define YAW_KP (10.0f)
#define YAW_KI (0.05f)
#define YAW_KD (0.1f)
#define PITCH_KP (11.0f)
#define PITCH_KI (0.0f)
#define PITCH_KD (0.1f)
#define INTIAL_PITCH_ANGLE (2500.0f)
#define INTIAL_YAW_ANGLE (0.0f)
#define SPEED_PID_OUTPUT_MAX (15000)
#define SPEED_PID_INTEGRAL_MAX (7500)


// Motor feedback timeout
#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

// Shooter system motor ID definition
#define MOTOR_STDID_1_4 (0x1FFU)
#define MOTOR_STDID_5_8 (0x2FFU)

/**
 * @brief Smoothly ramp a value towards a target by a fixed step.
 * @param current Current value.
 * @param target Target value to approach.
 * @param step Maximum change per call.
 * @return New value after applying the ramp step toward target.
 */
static float RampTowards(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}

/**
 * @brief Compute one motor's output current using PID based on feedback.
 * @param pid PID controller pointer.
 * @param target Target speed for this motor.
 * @param feedback Motor feedback pointer.
 * @param current_tick Current timestamp (ms).
 * @return Output current command (int16).
 */
static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        return 0;
    }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}



/**
 * @brief Initialize shooter controller and its PIDs; init gimbal pitch.
 * @param controller Shooter controller pointer.
 */
void ShooterController_Init(ShooterController *controller)
{
    if (controller == NULL) return;
    
    // Initialize all members to 0
    memset(controller, 0, sizeof(ShooterController));
    
    // Initialize PID controllers
    PID_Init(&controller->turntable_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 
             SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    PID_Init(&controller->shooter1_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 
             SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    PID_Init(&controller->shooter2_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 
             SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    
    // Initialize gimbal pitch

    Motor_Init(6, YAW_KP, YAW_KI, YAW_KD, 6300.0f); // yaw
    Motor_Init(7, PITCH_KP, PITCH_KI, PITCH_KD, INTIAL_PITCH_ANGLE); // pitch
}

/**
 * @brief Update shooter state and smooth targets from RC input.
 * @param controller Shooter controller pointer.
 * @param rc_data Remote control data pointer (can be NULL).
 * @param current_tick Current timestamp (ms).
 */
void ShooterController_Update(ShooterController *controller, const RC_ctrl_t *rc_data, uint32_t current_tick, SensorData* sensor_data)
{
    if (controller == NULL) return;
    
    // Check left switch position: down/off, mid/preheat (shooters only), up/all on
    bool right_switch_up = false;
    bool right_switch_mid = false;
    bool right_switch_down = false;
    bool left_switch_up = false;

    int16_t yaw = 0;
    if (rc_data != NULL)
    {
        right_switch_up = switch_is_up(rc_data->rc.s[0]);
        right_switch_mid = switch_is_mid(rc_data->rc.s[0]);
        right_switch_down = switch_is_down(rc_data->rc.s[0]);
        
    }
    
    left_switch_up = switch_is_up(rc_data->rc.s[1]);
    if (left_switch_up) {
        controller->gimbal_yaw_current = Joystick_control(6, 660, sensor_data);
    }
    
    
    // Enabled if any shooter/turntable should run (mid or up)
    controller->enabled = (right_switch_up || right_switch_mid);
    
    // Targets based on mode
    float turntable_target = right_switch_up ? MOTOR5_CONST_SPEED : 0.0f;
    float shooter1_target = (right_switch_up || right_switch_mid) ? -SHOOTER_CONST_SPEED : 0.0f;
    float shooter2_target = (right_switch_up || right_switch_mid) ?  SHOOTER_CONST_SPEED : 0.0f;
    
    // Apply ramping
    controller->ramped_turntable = RampTowards(controller->ramped_turntable, turntable_target, SHOOTER_RAMP_STEP);
    controller->ramped_shooter1 = RampTowards(controller->ramped_shooter1, shooter1_target, SHOOTER_RAMP_STEP);
    controller->ramped_shooter2 = RampTowards(controller->ramped_shooter2, shooter2_target, SHOOTER_RAMP_STEP);


    // Gimbal pitch control
    controller->gimbal_enabled = (rc_data != NULL);
    if (controller->gimbal_enabled)
    {
        controller->gimbal_current = Joystick_control(7, rc_data->rc.ch[1], sensor_data);
        controller->gimbal_yaw_current = Yaw_Control_With_Compensation(rc_data->rc.ch[0], sensor_data);

    }
    else
    {
        controller->gimbal_current = 0;
        controller->gimbal_yaw_current = 0;
    }
}

/**
 * @brief Compute all shooter currents and send via CAN.
 * @param controller Shooter controller pointer.
 * @param current_tick Current timestamp (ms).
 */
void ShooterController_ComputeCurrents(ShooterController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;
    
    // Compute turntable current (motor 5)
    controller->output_currents[0] = ComputeSingleMotorCurrent(
        &controller->turntable_pid, 
        controller->ramped_turntable, 
        &controller->turntable_feedback, 
        current_tick
    );
    
    // Compute shooter wheel 1 current (motor 6)
    controller->output_currents[1] = ComputeSingleMotorCurrent(
        &controller->shooter1_pid, 
        controller->ramped_shooter1, 
        &controller->shooter1_feedback, 
        current_tick
    );
    
    // Motor 7 (GM6020 gimbal)
    controller->output_currents[2] = controller->gimbal_current;
    
    // Compute shooter wheel 2 current (motor 8)
    controller->output_currents[3] = ComputeSingleMotorCurrent(
        &controller->shooter2_pid, 
        controller->ramped_shooter2, 
        &controller->shooter2_feedback, 
        current_tick
    );

    
    
    // Send CAN commands
    // 1) Send motors 5,6,8 on CAN2 (StdId 0x1FF). Leave slot for motor 7 empty here.
    CAN_Manager_SendMotorCurrents4(
        &hcan2,
        MOTOR_STDID_1_4,
        controller->output_currents[0],  // Turntable (motor 5)
        controller->output_currents[1],  // Shooter wheel 1 (motor 6)
        0,                               // Skip GM6020 here (motor 7 sent on CAN1/0x2FF)
        controller->output_currents[3]   // Shooter wheel 2 (motor 8)
    );

    // 2) Send GM6020 currents: pitch on CAN2 (id=7), yaw on CAN1 (id=3)
    CAN_Manager_SendGM6020Current(&hcan2, 7, controller->gimbal_current);
    CAN_Manager_SendGM6020Current(&hcan1, 6, controller->gimbal_yaw_current);
    
}

/**
 * @brief Set turntable target speed.
 * @note Currently not used by main loop; reserved for future control inputs.
 * @param controller Shooter controller pointer.
 * @param speed Target speed.
 */
void ShooterController_SetTurntableSpeed(ShooterController *controller, float speed)
{
    if (controller == NULL) return;
    controller->turntable_target = speed;
}

/**
 * @brief Set shooter wheels target speeds.
 * @note Currently not used by main loop; reserved for future control inputs.
 * @param controller Shooter controller pointer.
 * @param shooter1_speed Target speed for shooter1.
 * @param shooter2_speed Target speed for shooter2.
 */
void ShooterController_SetShooterSpeeds(ShooterController *controller, float shooter1_speed, float shooter2_speed)
{
    if (controller == NULL) return;
    controller->shooter1_target = shooter1_speed;
    controller->shooter2_target = shooter2_speed;
}

/**
 * @brief Stop shooter system and reset integrals/targets.
 * @note Currently not used by main loop; can be used by higher-level logic.
 * @param controller Shooter controller pointer.
 */
void ShooterController_Stop(ShooterController *controller)
{
    if (controller == NULL) return;
    
    controller->enabled = false;
    controller->turntable_target = 0.0f;
    controller->shooter1_target = 0.0f;
    controller->shooter2_target = 0.0f;
    controller->gimbal_current = 0;
    controller->gimbal_yaw_current = 0;

    
    // Reset PID integrals
    controller->turntable_pid.integral = 0.0f;
    controller->shooter1_pid.integral = 0.0f;
    controller->shooter2_pid.integral = 0.0f;
}

/**
 * @brief Get latest shooter output currents.
 * @note Currently not used externally; handy for diagnostics/telemetry.
 * @param controller Shooter controller pointer.
 * @return Pointer to 4-element int16 array or NULL.
 */
const int16_t* ShooterController_GetOutputCurrents(const ShooterController *controller)
{
    if (controller == NULL) return NULL;
    return controller->output_currents;
}

/**
 * @brief Check whether shooter has any active demand.
 * @param controller Shooter controller pointer.
 * @return true if any demand/current is non-zero.
 */
bool ShooterController_IsRunning(const ShooterController *controller)
{
    if (controller == NULL) return false;
    
    return controller->enabled || 
           controller->ramped_turntable != 0 || 
           controller->ramped_shooter1 != 0 || 
           controller->ramped_shooter2 != 0 ||
           controller->gimbal_current != 0 ||
           controller->gimbal_yaw_current != 0;
}

/**
 * @brief Update feedback for one shooter motor from CAN receive path.
 * @param controller Shooter controller pointer.
 * @param motor_id Motor ID (4=turntable, 5=shooter1, 7=shooter2).
 * @param angle Encoder angle.
 * @param speed Speed (RPM).
 * @param current Motor current.
 * @param temp Temperature.
 * @param current_tick Timestamp in ms.
 */
void ShooterController_UpdateMotorFeedback(ShooterController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL) return;
    
    Motor_Feedback *feedback = NULL;
    
    switch (motor_id)
    {
        case 4: // Turntable
            feedback = &controller->turntable_feedback;
            break;
        case 5: // Shooter wheel 1
            feedback = &controller->shooter1_feedback;
            break;
        case 7: // Shooter wheel 2
            feedback = &controller->shooter2_feedback;
            break;
        default:
            return;
    }
    
    if (feedback != NULL)
    {
        feedback->angle = angle;
        feedback->speed = speed;
        feedback->current = current;
        feedback->temp = temp;
        feedback->last_update_time = current_tick;
    }
}
