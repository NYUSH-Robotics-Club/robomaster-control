#include "dm10010l_motor.h"
#include "pid.h"
#include "can.h"
#include "can_manager.h"
#include <math.h>
#include "printing.h"
#include "message_center.h"
#include "can_comm.h"

// Forward declarations for subscription callback and guard flag
static void on_dm10010l_feedback(const MsgEvent *ev, void *user);
static uint8_t g_subscribed = 0;

#define DM10010L_MAX_TARGET_VEL         (10.0f)  // Max velocity in rad/s
#define DM10010L_MAX_TARGET_POS         (2.0f * M_PI)  // Max position in rad

static DM10010L_MotorContext g_ctx[8];

typedef struct {
    uint8_t id;
    float kp;
    float ki;
    float kd;
    float initial_position;
} DM10010L_Init_Config_s;

DM10010L_MotorContext* DM10010L_GetContext(uint8_t id)
{
    if (id < 1 || id > 7) return NULL;
    return &g_ctx[id-1];
}

bool DM10010L_IsInitialized(uint8_t id)
{
    if (id < 1 || id > 7) return false;
    return g_ctx[id-1].position_inited != 0;
}

void DM10010L_Init(uint8_t id, float KP, float KI, float KD, float initial_position, float output_max, float integral_max)
{
    if (!g_subscribed) {
        (void)MsgCenter_Subscribe(TOPIC_DM10010L_FEEDBACK, on_dm10010l_feedback, NULL);
        g_subscribed = 1;
    }
    if (id < 1 || id > 7) return;
    DM10010L_MotorContext *c = &g_ctx[id-1];
    c->id = id;
    c->position = 0.0f;
    c->velocity = 0.0f;
    c->torque = 0.0f;
    c->position_target = initial_position;
    c->velocity_target = 0.0f;
    if (initial_position < 0.0f) {
        c->position_inited = 0;
    } else {
        c->position_inited = 1;
    }

    // Configure limits (adjust as needed)
    c->position_min = -DM10010L_MAX_TARGET_POS;
    c->position_max = DM10010L_MAX_TARGET_POS;
    c->velocity_max = DM10010L_MAX_TARGET_VEL;

    PID_Init(&c->position_pid, KP, KI, KD, output_max, integral_max);
    PID_Reset(&c->position_pid);
}

void DM10010L_Motor_Feedback(uint8_t motor_id, uint8_t err, int16_t pos, int16_t vel, int16_t torque, uint8_t t_mos, uint32_t timestamp)
{
    if (motor_id < 1 || motor_id > 7) return;
    DM10010L_MotorContext *c = &g_ctx[motor_id-1];
    c->err = err;
    c->position = pos / 1000.0f;  // Assuming pos is in millirad or similar, convert to rad
    c->velocity = vel / 1000.0f;  // Convert to rad/s
    c->torque = torque / 1000.0f;  // Convert to Nm or appropriate unit
    c->t_mos = t_mos;
    c->last_update = timestamp;

    if (!c->position_inited) {
        c->position_target = c->position;
        c->position_inited = 1;
    }
}

static void on_dm10010l_feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(DM10010LFeedbackEvent)) {
        const DM10010LFeedbackEvent *m = (const DM10010LFeedbackEvent *)ev->data;
        DM10010L_Motor_Feedback(m->motor_id, m->err, m->pos, m->vel, m->torque, m->t_mos, m->timestamp);
    }
}

void DM10010L_SetPositionTarget(uint8_t id, float position)
{
    if (id < 1 || id > 7) return;
    DM10010L_MotorContext *c = &g_ctx[id-1];
    if (position < c->position_min) position = c->position_min;
    if (position > c->position_max) position = c->position_max;
    c->position_target = position;
}

void DM10010L_SetVelocityTarget(uint8_t id, float velocity)
{
    if (id < 1 || id > 7) return;
    DM10010L_MotorContext *c = &g_ctx[id-1];
    if (velocity < -c->velocity_max) velocity = -c->velocity_max;
    if (velocity > c->velocity_max) velocity = c->velocity_max;
    c->velocity_target = velocity;
}

void DM10010L_UpdateControl(uint8_t id)
{
    if (id < 1 || id > 7) return;
    DM10010L_MotorContext *c = &g_ctx[id-1];
    if (!c->position_inited) return;

    // Position control (if needed)
    float pos_error = c->position_target - c->position;
    float vel_cmd = PID_Update(&c->position_pid, pos_error, 0.01f);  // Assuming 10ms dt

    // Velocity control
    if (vel_cmd < -c->velocity_max) vel_cmd = -c->velocity_max;
    if (vel_cmd > c->velocity_max) vel_cmd = c->velocity_max;

    // Send command
    CAN_Manager_SendDM10010LCurrent(&hcan1, id, c->position_target, vel_cmd);
}