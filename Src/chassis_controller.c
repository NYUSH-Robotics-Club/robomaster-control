#include "chassis_controller.h"
#include "can.h"
#include "can_manager.h"
#include <string.h>
#include <math.h>

extern CAN_HandleTypeDef hcan1;

// PID parameters
#define SPEED_PID_KP (5.0f)
#define SPEED_PID_KI (0.5f)
#define SPEED_PID_KD (0.1f)
#define SPEED_PID_OUTPUT_MAX (15000)
#define SPEED_PID_INTEGRAL_MAX (7500)

// Motor feedback timeout
#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

// Chassis motor ID definition
#define MOTOR_STDID_1_4 (0x200U)

typedef struct {
    float x;
    float y;
} Pair;

Pair to_real_speed(Pair speed, float angle, float w) {
    const float k = 0.01;
    angle += k * w;
    angle = 0;

    Pair result;
    result.x = speed.x * cos(angle) - speed.y * sin(angle);
    result.y = speed.x * sin(angle) + speed.y * cos(angle);
    return result;
}

// Motor direction: +1 for left side, -1 for right side (aligns physical forward)
static const int8_t MOTOR_DIR[CHASSIS_MOTOR_COUNT] = {
    -1, // RR (ID1) - right side
    +1, // LR (ID2) - left side
    +1, // LF (ID3) - left side
    -1  // RF (ID4) - right side
};

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
 * @brief Reset the integral term of all wheel speed PID controllers.
 * @param controller Chassis controller pointer (must not be NULL).
 */
static void ResetPidIntegrals(ChassisController *controller)
{
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->speed_pids[i].integral = 0.0f;
    }
}

/**
 * @brief Compute one motor's output current using PID based on feedback.
 * @param pid PID controller pointer.
 * @param target Target speed for this motor.
 * @param feedback Motor feedback pointer (angle/speed/current/temp/timestamp).
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
 * @brief Initialize chassis controller: zero fields and setup PID controllers.
 * @param controller Chassis controller pointer.
 */
void ChassisController_Init(ChassisController *controller)
{
    if (controller == NULL) return;
    
    // Initialize all members to 0
    memset(controller, 0, sizeof(ChassisController));
    
    // Initialize PID controllers
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        PID_Init(&controller->speed_pids[i], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 
                 SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    }
    
    // Default targets to zero
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->target_speeds[i] = 0.0f;
        controller->ramped_targets[i] = 0.0f;
    }
}

/**
 * @brief Update chassis state machine and smooth target speeds.
 * @param controller Chassis controller pointer.
 * @param rc_data Remote control data pointer (can be NULL).
 * @param current_tick Current timestamp (ms).
 */
void ChassisController_Update(ChassisController *controller, const RC_ctrl_t *rc_data, uint32_t current_tick, SensorData sensor_data)
{
    if (controller == NULL) return;
    
    // Map sticks to chassis velocities
    // Left stick: ch[3] vertical (forward/back), ch[2] horizontal (strafe)
    // Right stick: ch[0] horizontal (yaw rotation)
    int16_t vx_raw = 0; // forward/backward
    int16_t vy_raw = 0; // right/left strafe
    int16_t wz_raw = 0; // yaw rotation (right positive: CCW)
    if (rc_data != NULL)
    {
        vx_raw = (int16_t)(rc_data->rc.ch[3]);
        vy_raw = (int16_t)(rc_data->rc.ch[2]);
        wz_raw = (int16_t)(-rc_data->rc.ch[4]);
        // deadband
        const int16_t deadband = 10;
        if (vx_raw > -deadband && vx_raw < deadband) vx_raw = 0;
        if (vy_raw > -deadband && vy_raw < deadband) vy_raw = 0;
        if (wz_raw > -deadband && wz_raw < deadband) wz_raw = 0;
    }

    // Scale to target speed units
    const float scale = (float)CHASSIS_DEMO_TARGET_SPEED / (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET); // 7000/660
    // float vx = (-(float)vx_raw * scale) / 3.0f; // forward + (1/3 sensitivity)
    // float vy = (-(float)vy_raw * scale) / 3.0f; // left + (1/3 sensitivity)

    float omega = (-(float)wz_raw * scale) / 3.0f; // CCW + (1/3 sensitivity)

    Pair _speed = (Pair){-(float)vx_raw * scale / 3.0f,
        -(float)vy_raw * scale / 3.0f};
    Pair speed = to_real_speed(_speed, sensor_data.yaw, omega);
    float vx = speed.x, vy = speed.y;

    // Mecanum kinematics with rotation
    // Motor order by CAN IDs: 1: Right Rear, 2: Left Rear, 3: Left Front, 4: Right Front
    // RR: vx - vy + omega; LR: vx + vy - omega; LF: vx - vy - omega; RF: vx + vy + omega
    controller->target_speeds[0] = MOTOR_DIR[0] * (vx - vy + omega); // RR (ID1)
    controller->target_speeds[1] = MOTOR_DIR[1] * (vx + vy - omega); // LR (ID2)
    controller->target_speeds[2] = MOTOR_DIR[2] * (vx - vy - omega); // LF (ID3)
    controller->target_speeds[3] = MOTOR_DIR[3] * (vx + vy + omega); // RF (ID4)

    // Running state based on stick activity
    controller->running = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);

    // Smooth target speeds
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->ramped_targets[i] = RampTowards(controller->ramped_targets[i], controller->target_speeds[i], CHASSIS_RAMP_STEP);
    }
}

/**
 * @brief Compute all motor currents and send them via CAN.
 * @param controller Chassis controller pointer.
 * @param current_tick Current timestamp (ms).
 */
void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;
    
    // Compute current for each motor
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        int16_t motor_current = ComputeSingleMotorCurrent(
            &controller->speed_pids[i],
            controller->ramped_targets[i],
            &controller->motor_feedbacks[i],
            current_tick
        );
        controller->output_currents[i] = motor_current;
    }
    
    // Send CAN command
    CAN_Manager_SendMotorCurrents4(
        &hcan1,
        MOTOR_STDID_1_4,
        controller->output_currents[0],
        controller->output_currents[1],
        controller->output_currents[2],
        controller->output_currents[3]
    );
}

/**
 * @brief Set target speeds for the 4 chassis motors.
 * @note Currently not used by main loop; reserved for future control inputs.
 * @param controller Chassis controller pointer.
 * @param speeds Array of 4 target speeds.
 */
void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT])
{
    if (controller == NULL || speeds == NULL) return;
    
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->target_speeds[i] = speeds[i];
    }
}

/**
 * @brief Stop the chassis: clear running state, reset integrals and targets.
 * @note Currently not used by main loop; can be used by higher-level logic.
 * @param controller Chassis controller pointer.
 */
void ChassisController_Stop(ChassisController *controller)
{
    if (controller == NULL) return;
    
    controller->running = false;
    ResetPidIntegrals(controller);
    
    // Clear target speeds
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->target_speeds[i] = 0.0f;
    }
}

/**
 * @brief Get the pointer to the latest computed output currents.
 * @note Currently not used externally; handy for diagnostics/telemetry.
 * @param controller Chassis controller pointer.
 * @return Pointer to 4-element int16 current array, or NULL if controller NULL.
 */
const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller)
{
    if (controller == NULL) return NULL;
    return controller->output_currents;
}

/**
 * @brief Check if any motor target is non-zero (chassis considered running).
 * @param controller Chassis controller pointer.
 * @return true if any target after ramping is non-zero.
 */
bool ChassisController_IsRunning(const ChassisController *controller)
{
    if (controller == NULL) return false;
    
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        if (controller->ramped_targets[i] != 0)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Update one motor's feedback from CAN receive path.
 * @param controller Chassis controller pointer.
 * @param motor_id Motor index in range 0..3.
 * @param angle Encoder angle.
 * @param speed Motor speed (RPM).
 * @param current Motor current.
 * @param temp Motor temperature.
 * @param current_tick Timestamp when feedback was received (ms).
 */
void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
    
    controller->motor_feedbacks[motor_id].angle = angle;
    controller->motor_feedbacks[motor_id].speed = speed;
    controller->motor_feedbacks[motor_id].current = current;
    controller->motor_feedbacks[motor_id].temp = temp;
    controller->motor_feedbacks[motor_id].last_update_time = current_tick;
}
