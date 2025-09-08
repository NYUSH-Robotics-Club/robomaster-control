#include "chassis_controller.h"
#include "can.h"
#include <string.h>

// External CAN handle
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

// Smoothing function
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

// Reset PID integrals
static void ResetPidIntegrals(ChassisController *controller)
{
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->speed_pids[i].integral = 0.0f;
    }
}

// Compute single motor current
static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        return 0;
    }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

// CAN send function
static HAL_StatusTypeDef CAN_SendMotorCurrents4(int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint8_t d[8];
    uint32_t mb;

    tx.StdId = MOTOR_STDID_1_4;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    d[0] = (uint8_t)(i1 >> 8); d[1] = (uint8_t)i1;
    d[2] = (uint8_t)(i2 >> 8); d[3] = (uint8_t)i2;
    d[4] = (uint8_t)(i3 >> 8); d[5] = (uint8_t)i3;
    d[6] = (uint8_t)(i4 >> 8); d[7] = (uint8_t)i4;

    return HAL_CAN_AddTxMessage(&hcan1, &tx, d, &mb);
}

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
    
    // Set default target speeds (Mecanum wheel configuration)
    controller->target_speeds[0] = CHASSIS_DEMO_TARGET_SPEED;   // Right front
    controller->target_speeds[1] = -CHASSIS_DEMO_TARGET_SPEED;  // Left front
    controller->target_speeds[2] = -CHASSIS_DEMO_TARGET_SPEED;  // Left rear
    controller->target_speeds[3] = CHASSIS_DEMO_TARGET_SPEED;   // Right rear
}

void ChassisController_Update(ChassisController *controller, const RC_ctrl_t *rc_data, uint32_t current_tick)
{
    if (controller == NULL) return;
    
    // Check remote control data
    bool left_switch_up = false;
    if (rc_data != NULL)
    {
        left_switch_up = switch_is_up(rc_data->rc.s[0]);
    }
    
    // State machine: control running state based on left switch
    static bool last_left_switch_up = false;
    if (left_switch_up && !last_left_switch_up)
    {
        // Start running
        controller->running = true;
    }
    else if (!left_switch_up && last_left_switch_up)
    {
        // Stop running
        controller->running = false;
        ResetPidIntegrals(controller);
    }
    last_left_switch_up = left_switch_up;
    
    // Smooth target speeds
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        float target = controller->running ? controller->target_speeds[i] : 0.0f;
        controller->ramped_targets[i] = RampTowards(controller->ramped_targets[i], target, CHASSIS_RAMP_STEP);
    }
}

void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;
    
    // Compute current for each motor
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->output_currents[i] = ComputeSingleMotorCurrent(
            &controller->speed_pids[i], 
            controller->ramped_targets[i], 
            &controller->motor_feedbacks[i], 
            current_tick
        );
    }
    
    // Send CAN command
    CAN_SendMotorCurrents4(
        controller->output_currents[0],
        controller->output_currents[1],
        controller->output_currents[2],
        controller->output_currents[3]
    );
}

void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT])
{
    if (controller == NULL || speeds == NULL) return;
    
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->target_speeds[i] = speeds[i];
    }
}

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

const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller)
{
    if (controller == NULL) return NULL;
    return controller->output_currents;
}

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

// Motor feedback update function (for external calls)
void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
    
    controller->motor_feedbacks[motor_id].angle = angle;
    controller->motor_feedbacks[motor_id].speed = speed;
    controller->motor_feedbacks[motor_id].current = current;
    controller->motor_feedbacks[motor_id].temp = temp;
    controller->motor_feedbacks[motor_id].last_update_time = current_tick;
}
