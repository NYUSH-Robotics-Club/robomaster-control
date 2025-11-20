#include "shooter_controller.h"
#include "can.h"
#include "can_manager.h"
#include <string.h>
#include <math.h>
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "can_comm.h"
#include "cmd_controller.h"

extern CAN_HandleTypeDef hcan2;

#define SPEED_PID_KP            (5.0f)
#define SPEED_PID_KI            (0.5f)
#define SPEED_PID_KD            (0.1f)
#define SPEED_PID_OUTPUT_MAX    (15000)
#define SPEED_PID_INTEGRAL_MAX  (7500)
#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)
#define MOTOR_STDID_1_4         (0x1FFU)

// Static variables for app wrapper
static ShootCmd        s_last_cmd;
static SensorData      s_last_sensor;
static ShooterController controller;

static float RampTowards(float current, float target, float step)
{
    if (current < target) {
        current += step;
        if (current > target) current = target;
    } else if (current > target) {
        current -= step;
        if (current < target) current = target;
    }
    return current;
}

static void ResetPidIntegrals(void)
{
    controller.turntable_pid.integral = 0.0f;
    controller.shooter1_pid.integral  = 0.0f;
    controller.shooter2_pid.integral  = 0.0f;
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback)
{
    if (HAL_GetTick() - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) {
        return 0;
    }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

/* ------------------- Controller core API (no pointers, like chassis) ------------------- */

void ShooterController_Init(void)
{
    memset(&controller, 0, sizeof(ShooterController));

    PID_Init(&controller.turntable_pid,
             SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);

    PID_Init(&controller.shooter1_pid,
             SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);

    PID_Init(&controller.shooter2_pid,
             SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);

    controller.initialized = true;
}

void ShooterController_Update(void)
{
    if (!controller.initialized) return;

    // Use standardized command from cmd_controller
    controller.enabled = s_last_cmd.friction_enabled;

    // Turntable: only spin when feed is enabled
    float turntable_target = s_last_cmd.feed_enabled ? MOTOR5_CONST_SPEED : 0.0f;

    // Shooter wheels: spin when friction is enabled
    float shooter1_target = s_last_cmd.friction_enabled ? -SHOOTER_CONST_SPEED : 0.0f;
    float shooter2_target = s_last_cmd.friction_enabled ?  SHOOTER_CONST_SPEED : 0.0f;

    // Save raw targets if you want them
    controller.turntable_target = turntable_target;
    controller.shooter1_target  = shooter1_target;
    controller.shooter2_target  = shooter2_target;

    // Apply ramping
    controller.ramped_turntable = RampTowards(controller.ramped_turntable,
                                              turntable_target,
                                              SHOOTER_RAMP_STEP);
    controller.ramped_shooter1  = RampTowards(controller.ramped_shooter1,
                                              shooter1_target,
                                              SHOOTER_RAMP_STEP);
    controller.ramped_shooter2  = RampTowards(controller.ramped_shooter2,
                                              shooter2_target,
                                              SHOOTER_RAMP_STEP);
}

void ShooterController_ComputeCurrents(void)
{
    if (!controller.initialized) return;

    controller.output_currents[0] = ComputeSingleMotorCurrent(
        &controller.turntable_pid,
        controller.ramped_turntable,
        &controller.turntable_feedback
    );

    controller.output_currents[1] = ComputeSingleMotorCurrent(
        &controller.shooter1_pid,
        controller.ramped_shooter1,
        &controller.shooter1_feedback
    );

    controller.output_currents[2] = 0;  // Not used / reserved

    controller.output_currents[3] = ComputeSingleMotorCurrent(
        &controller.shooter2_pid,
        controller.ramped_shooter2,
        &controller.shooter2_feedback
    );

    CAN_Manager_SendMotorCurrents4(&hcan2, MOTOR_STDID_1_4,
        controller.output_currents[0],
        controller.output_currents[1],
        controller.output_currents[2],
        controller.output_currents[3]);
}

void ShooterController_SetTurntableSpeed(float speed)
{
    if (!controller.initialized) return;
    controller.turntable_target = speed;
}

void ShooterController_SetShooterSpeeds(float shooter1_speed, float shooter2_speed)
{
    if (!controller.initialized) return;
    controller.shooter1_target = shooter1_speed;
    controller.shooter2_target = shooter2_speed;
}

void ShooterController_Stop(void)
{
    if (!controller.initialized) return;

    controller.enabled = false;
    controller.turntable_target = 0.0f;
    controller.shooter1_target  = 0.0f;
    controller.shooter2_target  = 0.0f;

    controller.ramped_turntable = 0.0f;
    controller.ramped_shooter1  = 0.0f;
    controller.ramped_shooter2  = 0.0f;

    ResetPidIntegrals();
}

const int16_t* ShooterController_GetOutputCurrents(void)
{
    if (!controller.initialized) return NULL;
    return controller.output_currents;
}

bool ShooterController_IsRunning(void)
{
    if (!controller.initialized) return false;
    return controller.enabled || controller.ramped_turntable != 0.0f || controller.ramped_shooter1  != 0.0f || controller.ramped_shooter2  != 0.0f;
}

void ShooterController_UpdateMotorFeedback(uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (!controller.initialized) return;

    Motor_Feedback *feedback = NULL;
    switch (motor_id) {
        case 4: feedback = &controller.turntable_feedback; break;
        case 5: feedback = &controller.shooter1_feedback;  break;
        case 7: feedback = &controller.shooter2_feedback;  break;
        default: return; // Not a shooter motor
    }

    feedback->angle = angle;
    feedback->speed = speed;
    feedback->current = current;
    feedback->temp = temp;
    feedback->last_update_time = current_tick;
}

/* ------------------------ App wrapper (MsgCenter callbacks) ------------------------ */

static void on_shoot_cmd(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(ShootCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ShootCmd));
        ShooterController_Update();
        ShooterController_ComputeCurrents();
    }
}

static void on_imu_update(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
        // currently unused, but kept for future sensor-aware logic
    }
}

static void on_motor_feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
        // Shooter uses ids 4, 5, and 7
        if (m->id == 4 || m->id == 5 || m->id == 7) {
            ShooterController_UpdateMotorFeedback(m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}

void ShooterApp_Init(void)
{
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));

    ShooterController_Init();

    (void)MsgCenter_Subscribe(TOPIC_SHOOT_CMD, on_shoot_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK,  on_motor_feedback, NULL);
}

ShooterController* ShooterApp_GetController(void)
{
    return &controller;
}
