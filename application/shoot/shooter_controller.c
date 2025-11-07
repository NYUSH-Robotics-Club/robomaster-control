#include "shooter_controller.h"
#include "can.h"
#include "can_manager.h"
#include <string.h>
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "can_comm.h"
#include "gimbal_controller.h"

extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;

#define SPEED_PID_KP (5.0f)
#define SPEED_PID_KI (0.5f)
#define SPEED_PID_KD (0.1f)
#define YAW_KP (10.0f)
#define YAW_KI (0.05f)
#define YAW_KD (0.1f)
#define PITCH_KP (11.0f)
#define PITCH_KI (0.0f)
#define PITCH_KD (0.1f)
#define INTIAL_PITCH_ANGLE (-1.0f)
#define INTIAL_YAW_ANGLE (0.0f)
#define SPEED_PID_OUTPUT_MAX (15000)
#define SPEED_PID_INTEGRAL_MAX (7500)
#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)
#define MOTOR_STDID_1_4 (0x1FFU)
#define MOTOR_STDID_5_8 (0x2FFU)

static float RampTowards(float current, float target, float step)
{
    if (current < target) { current += step; if (current > target) current = target; }
    else if (current > target) { current -= step; if (current < target) current = target; }
    return current;
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) { return 0; }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

void ShooterController_Init(ShooterController *controller)
{
    if (controller == NULL) return;
    memset(controller, 0, sizeof(ShooterController));
    PID_Init(&controller->turntable_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    PID_Init(&controller->shooter1_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    PID_Init(&controller->shooter2_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    GimbalController_Init(YAW_KP, YAW_KI, YAW_KD, 6300.0f,
                          PITCH_KP, PITCH_KI, PITCH_KD, INTIAL_PITCH_ANGLE);
}

void ShooterController_Update(ShooterController *controller, const RC_ctrl_t *rc_data, uint32_t current_tick, SensorData* sensor_data)
{
    if (controller == NULL) return;
    bool right_switch_up = false;
    bool right_switch_mid = false;
    bool right_switch_down = false;
    bool left_switch_up = false;
    int16_t yaw = 0; (void)yaw;
    if (rc_data != NULL)
    {
        right_switch_up = switch_is_up(rc_data->rc.s[0]);
        right_switch_mid = switch_is_mid(rc_data->rc.s[0]);
        right_switch_down = switch_is_down(rc_data->rc.s[0]);
    }
    left_switch_up = switch_is_up(rc_data->rc.s[1]);
    if (left_switch_up) {
        controller->gimbal_yaw_current = GimbalController_JoystickControl(6, 660, sensor_data);
    }
    controller->enabled = (right_switch_up || right_switch_mid);
    float turntable_target = right_switch_up ? MOTOR5_CONST_SPEED : 0.0f;
    float shooter1_target = (right_switch_up || right_switch_mid) ? -SHOOTER_CONST_SPEED : 0.0f;
    float shooter2_target = (right_switch_up || right_switch_mid) ?  SHOOTER_CONST_SPEED : 0.0f;
    controller->ramped_turntable = RampTowards(controller->ramped_turntable, turntable_target, SHOOTER_RAMP_STEP);
    controller->ramped_shooter1 = RampTowards(controller->ramped_shooter1, shooter1_target, SHOOTER_RAMP_STEP);
    controller->ramped_shooter2 = RampTowards(controller->ramped_shooter2, shooter2_target, SHOOTER_RAMP_STEP);
    controller->gimbal_enabled = (rc_data != NULL);
    if (controller->gimbal_enabled) {
        // TODO trash
        controller->gimbal_current = GimbalController_JoystickControl(7, rc_data->rc.ch[1], sensor_data);
        controller->gimbal_yaw_current = GimbalController_YawControlWithCompensation(rc_data->rc.ch[0], sensor_data);
    } else {
        controller->gimbal_current = 0;
        controller->gimbal_yaw_current = 0;
    }
}

void ShooterController_ComputeCurrents(ShooterController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;
    controller->output_currents[0] = ComputeSingleMotorCurrent(&controller->turntable_pid, controller->ramped_turntable, &controller->turntable_feedback, current_tick);
    controller->output_currents[1] = ComputeSingleMotorCurrent(&controller->shooter1_pid, controller->ramped_shooter1, &controller->shooter1_feedback, current_tick);
    controller->output_currents[2] = controller->gimbal_current;
    controller->output_currents[3] = ComputeSingleMotorCurrent(&controller->shooter2_pid, controller->ramped_shooter2, &controller->shooter2_feedback, current_tick);
    CAN_Manager_SendMotorCurrents4(&hcan2, MOTOR_STDID_1_4,
        controller->output_currents[0], controller->output_currents[1], 0, controller->output_currents[3]);
    CAN_Manager_SendGM6020Current(&hcan2, 7, controller->gimbal_current);
    CAN_Manager_SendGM6020Current(&hcan1, 6, controller->gimbal_yaw_current);
}

void ShooterController_SetTurntableSpeed(ShooterController *controller, float speed)
{ if (controller == NULL) return; controller->turntable_target = speed; }

void ShooterController_SetShooterSpeeds(ShooterController *controller, float shooter1_speed, float shooter2_speed)
{ if (controller == NULL) return; controller->shooter1_target = shooter1_speed; controller->shooter2_target = shooter2_speed; }

void ShooterController_Stop(ShooterController *controller)
{
    if (controller == NULL) return;
    controller->enabled = false;
    controller->turntable_target = 0.0f;
    controller->shooter1_target = 0.0f;
    controller->shooter2_target = 0.0f;
    controller->gimbal_current = 0;
    controller->gimbal_yaw_current = 0;
    controller->turntable_pid.integral = 0.0f;
    controller->shooter1_pid.integral = 0.0f;
    controller->shooter2_pid.integral = 0.0f;
}

const int16_t* ShooterController_GetOutputCurrents(const ShooterController *controller)
{ if (controller == NULL) return NULL; return controller->output_currents; }

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

void ShooterController_UpdateMotorFeedback(ShooterController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL) return;
    Motor_Feedback *feedback = NULL;
    switch (motor_id)
    {
        case 4: feedback = &controller->turntable_feedback; break;
        case 5: feedback = &controller->shooter1_feedback; break;
        case 7: feedback = &controller->shooter2_feedback; break;
        default: return;
    }
    if (feedback != NULL) {
        feedback->angle = angle;
        feedback->speed = speed;
        feedback->current = current;
        feedback->temp = temp;
        feedback->last_update_time = current_tick;
    }
}

// Subscription-driven wrapper
static RC_ctrl_t s_last_rc;
static SensorData s_last_sensor;
static ShooterController s_ctrl;

static void on_rc_update(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(RC_ctrl_t)) {
        memcpy(&s_last_rc, ev->data, sizeof(RC_ctrl_t));
    }
}

static void on_imu_update(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

static void on_motor_feedback(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
        if (m->id >= 4) {
            ShooterController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}

void ShooterApp_Init(void) {
    memset(&s_last_rc, 0, sizeof(s_last_rc));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));
    ShooterController_Init(&s_ctrl);
    (void)MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
}

void ShooterApp_Tick(uint32_t tick_ms) {
    ShooterController_Update(&s_ctrl, &s_last_rc, tick_ms, &s_last_sensor);
    ShooterController_ComputeCurrents(&s_ctrl, tick_ms);
}

ShooterController* ShooterApp_GetController(void) {
    return &s_ctrl;
}


