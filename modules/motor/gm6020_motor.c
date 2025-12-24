#include "gm6020_motor.h"
#include "pid.h"
#include "can.h"
#include "can_manager.h"
#include <math.h>
#include "printing.h"
#include "message_center.h"
#include "can_comm.h"

// TODO: trash here, why there is only gm6020 in motor layer, but where is the 3508, refactor!

// Forward declarations for subscription callback and guard flag
static void on_gm6020_feedback(const MsgEvent *ev, void *user);
static uint8_t g_subscribed = 0;

#define GM6020_MAX_TARGET_RPM           (50.0f)
#define GM6020_JOYSTICK_DEADZONE        (30)
#define GM6020_JOYSTICK_FULL_SCALE      (660.0f)
#define GM6020_ANGLE_HOLD_KP_RPM_PER_DEG   (100.0f)
#define GM6020_ANGLE_HOLD_MIN_RPM          (120.0f)
#define PITCH_ID 7
#define YAW_ID 6
#define YAW_RAMP_STEP (25.0f)

static GM6020_MotorContext g_ctx[8];

float tar_rpm = 0.0f;
float cur_rpm = 0.0f;

#define send_current_by_id(id, cur) CAN_Manager_SendGM6020Current(&hcan2, (id), (cur))

typedef struct {
    uint8_t id;
    float kp;
    float ki;
    float kd;
    float initial_angle;
} GM6020_Init_Config_s;


GM6020_MotorContext* GM6020_GetContext(uint8_t id)
{
    if (id < 1 || id > 7) return NULL;
    return &g_ctx[id-1];
}

bool GM6020_IsInitialized(uint8_t id)
{
    if (id < 1 || id > 7) return false;
    return g_ctx[id-1].angle_inited != 0;
}

void Motor_Init(uint8_t id, float KP, float KI, float KD, float initial_angle, float output_max, float integral_max)
{
  if (!g_subscribed) {
    (void)MsgCenter_Subscribe(TOPIC_GM6020_FEEDBACK, on_gm6020_feedback, NULL);
    g_subscribed = 1;
  }
  if (id < 1 || id > 7) return;
  GM6020_MotorContext *c = &g_ctx[id-1];
  c->id = id;
  c->angle_raw = 0.0f;
  c->speed_rpm = 0;
  c->angle_target = initial_angle;
  if(initial_angle < 0.0f){
    c->angle_inited = 0;
  }else{
    c->angle_inited = 1;
  }

  if(id == PITCH_ID){
    c->angle_min = 1000.0f;
    c->angle_max = 4000.0f;
    c->max_encoder = 8192.0f;
    c->pitch_direction = -1.0f;
    c->gravity_effort = 5000.0f;
  } else {
    c->angle_min = 0.0f;
    c->angle_max = 8192.0f;
    c->max_encoder = 8192.0f;
  }
  PID_Init(&c->angle_pid, KP, KI, KD, output_max, integral_max);
  PID_Reset(&c->angle_pid);
  //PID_Reset(&c->speed_pid);
}

void Yaw_Speed_PID_Init(uint8_t id, float KP, float KI, float KD)
{
  if (id < 1 || id > 7) return;
  GM6020_MotorContext *c = &g_ctx[id-1];
  PID_Init(&c->speed_pid, KP, KI, KD, 30000.0f, 25000.0f);
  PID_Reset(&c->speed_pid);
}

void GM6020_Motor_Feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm, int16_t current)
{
  if (id < 1 || id > 7) return;
  GM6020_MotorContext *c = &g_ctx[id-1];
  c->angle_raw = angle_raw;
  c->feedback_current = current;
  if (c->max_encoder > 0.0f) {
    c->target_angle_rad = (float)c->angle_raw / c->max_encoder * 2.0f * M_PI;
  }
  c->speed_rpm = speed_rpm;
  if(!c->angle_inited){
    c->angle_target = (float)angle_raw;
    c->angle_inited = 1;
  }
}

static void on_gm6020_feedback(const MsgEvent *ev, void *user)
{
  (void)user;
  if (ev->size == sizeof(GM6020FeedbackEvent)) {
    const GM6020FeedbackEvent *m = (const GM6020FeedbackEvent *)ev->data;
    if (m->id >= 1 && m->id <= 7) {
      GM6020_Motor_Feedback(m->id, m->angle, m->speed, m->current);
    }
  }
}

