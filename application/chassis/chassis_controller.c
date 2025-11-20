#include "chassis_controller.h"
#include "can.h"
#include "can_manager.h"
#include <string.h>
#include <math.h>
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "can_comm.h"
#include "printing.h"
#include "cmd_controller.h"

extern CAN_HandleTypeDef hcan1;

#define SPEED_PID_KP (5.0f)
#define SPEED_PID_KI (0.5f)
#define SPEED_PID_KD (0.1f)
#define SPEED_PID_OUTPUT_MAX (15000)
#define SPEED_PID_INTEGRAL_MAX (7500)
#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)
#define MOTOR_STDID_1_4 (0x200U)

typedef struct { float x; float y; } Pair;

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static SensorData sensor_data;
static ChassisController controller;

static Pair to_real_speed(Pair speed, float angle, float w) {
    const float k = 0.01f;
    angle += k * w;
    angle = 0.0f;
    Pair result;
    result.x = speed.x * cosf(angle) - speed.y * sinf(angle);
    result.y = speed.x * sinf(angle) + speed.y * cosf(angle);
    return result;
}

static const int8_t MOTOR_DIR[CHASSIS_MOTOR_COUNT] = { -1, +1, +1, -1 };

static float RampTowards(float current, float target, float step)
{
    if (current < target) { current += step; if (current > target) current = target; }
    else if (current > target) { current -= step; if (current < target) current = target; }
    return current;
}

static void ResetPidIntegrals()
{
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller.speed_pids[i].integral = 0.0f; }
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback )
{
    if (HAL_GetTick() - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) { return 0; }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate (pid, target, current_speed);
}

void ChassisController_Init()
{
    
    memset(&controller, 0, sizeof(ChassisController));
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        PID_Init(&controller.speed_pids[i], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 
                 SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    }
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        controller.target_speeds[i] = 0.0f;
        controller.ramped_targets[i] = 0.0f;
    }
    controller.initialized = true;
}

void ChassisController_Update()
{
    
    if (!controller.initialized) return;
    
    float vx_norm = s_last_cmd.vx;
    float vy_norm = s_last_cmd.vy;
    float wz_norm = s_last_cmd.wz;
    
    float scale = (float)CHASSIS_DEMO_TARGET_SPEED / 2.0f;
    float omega = wz_norm * scale;
    Pair _speed = (Pair){vx_norm * scale, vy_norm * scale};
    Pair speed = to_real_speed(_speed, sensor_data.c_yaw, omega);
    float vx = speed.x, vy = speed.y;

    controller.target_speeds[0] = MOTOR_DIR[0] * (vx - vy + omega);
    controller.target_speeds[1] = MOTOR_DIR[1] * (vx + vy - omega);
    controller.target_speeds[2] = MOTOR_DIR[2] * (vx - vy - omega);
    controller.target_speeds[3] = MOTOR_DIR[3] * (vx + vy + omega);
    
    controller.running = s_last_cmd.enabled;
    
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        controller.ramped_targets[i] = RampTowards(controller.ramped_targets[i], controller.target_speeds[i], CHASSIS_RAMP_STEP);
    }
}

void ChassisController_ComputeCurrents()
{
   
    if (!controller.initialized) return;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        int16_t motor_current = ComputeSingleMotorCurrent(
            &controller.speed_pids[i],
            controller.ramped_targets[i],
            &controller.motor_feedbacks[i]
        );
        controller.output_currents[i] = motor_current;
    }
    CAN_Manager_SendMotorCurrents4(&hcan1, MOTOR_STDID_1_4,
        controller.output_currents[0], controller.output_currents[1], controller.output_currents[2], controller.output_currents[3]);
}

void ChassisController_SetTargetSpeeds(const float speeds[CHASSIS_MOTOR_COUNT])
{
    if (!controller.initialized || speeds == NULL) return;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller.target_speeds[i] = speeds[i]; }
}

void ChassisController_Stop()
{
    if (!controller.initialized) return;
    controller.running = false;
    ResetPidIntegrals();
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller.target_speeds[i] = 0.0f; }
}

const int16_t* ChassisController_GetOutputCurrents()
{
    if(!controller.initialized) return NULL;
    return controller.output_currents;
    
}

bool ChassisController_IsRunning()
{
    if (!controller.initialized) return false;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { if (controller.ramped_targets[i] != 0) { return true; } }
    return false;
}

void ChassisController_UpdateMotorFeedback(uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (!controller.initialized || motor_id >= CHASSIS_MOTOR_COUNT) return;
    controller.motor_feedbacks[motor_id].angle = angle;
    controller.motor_feedbacks[motor_id].speed = speed;
    controller.motor_feedbacks[motor_id].current = current;
    controller.motor_feedbacks[motor_id].temp = temp;
    controller.motor_feedbacks[motor_id].last_update_time = current_tick;
}

// Subscription callbacks
static void on_chassis_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(ChassisCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
        // Update controller and compute currents when command arrives
        ChassisController_Update();
        ChassisController_ComputeCurrents();
    }
}

static void on_imu_update(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(SensorData)) {
        memcpy(&sensor_data, ev->data, sizeof(SensorData));
    }
}

static void on_motor_feedback(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
        // Only process chassis motor feedback (id < 4)
        if (m->id < 4) {
            ChassisController_UpdateMotorFeedback( m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}

void ChassisApp_Init(void) {
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&sensor_data, 0, sizeof(sensor_data));
    ChassisController_Init();
    (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
}

ChassisController* ChassisApp_GetController(void) {
    return &controller;
}


