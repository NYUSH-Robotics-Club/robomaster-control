#include "GM6020_Motor.h"
#include "pid.h"
#include "can.h"
#include "can_manager.h"
#include <math.h>





#define GM6020_MAX_TARGET_RPM           (50.0f)
#define GM6020_JOYSTICK_DEADZONE        (30)
#define GM6020_JOYSTICK_FULL_SCALE      (660.0f)
#define GM6020_ANGLE_HOLD_KP_RPM_PER_DEG   (100.0f)
#define GM6020_ANGLE_HOLD_MIN_RPM          (120.0f)
#define PITCH_ID 7
typedef struct {
  uint8_t   id;
  uint16_t  angle_raw;
  int16_t   speed_rpm;
  int32_t   hold_angle_raw;
  uint8_t   hold_inited;
  PID_Controller speed_pid;
  PID_Controller angle_pid;
  
  float target_angle_rad;
  float phase_offset_rad;   // encoder value when the barrel points "forward"
  float pitch_direction;     // +1 or -1 (mechanical sign)
  float gravity_effort;      // current units needed to hold at 90° (tune this)
  float max_encoder;         // e.g. 8192.0f for GM6020
} gm6020_ctx_t;

static gm6020_ctx_t g_ctx[8];

float tar_rpm = 0.0f;
float cur_rpm = 0.0f;

#define send_current_by_id(id, cur) CAN_Manager_SendGM6020Current(&hcan2, (id), (cur))

void Motor_Init(uint8_t id, float KP, float KI, float KD)
{
  if (id < 1 || id > 7) return;
  gm6020_ctx_t *c = &g_ctx[id-1];
  c->id = id;
  c->angle_raw = 0;
  c->speed_rpm = 0;
  c->hold_angle_raw = 0;
  c->hold_inited = 0;
  PID_Init(&c->speed_pid, KP, KI, KD, 30000.0f, 25000.0f);
  PID_Init(&c->angle_pid, KP, KI, KD, 30000.0f, 25000.0f);

}

void GM6020_Motor_Feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm)
{
  if (id < 1 || id > 7) return;
  gm6020_ctx_t *c = &g_ctx[id-1];
  c->angle_raw = angle_raw;
  c->target_angle_rad = (float)c->angle_raw / c->max_encoder * 2.0f * M_PI;
  c->speed_rpm = speed_rpm;
  if (!c->hold_inited) {
    c->hold_angle_raw = angle_raw;
    c->hold_inited = 1;
  }
}

/**
 * @brief Joystick control: deadzone => zero target speed via PID, beyond deadzone => constant speed.
 * @param id Motor id (1..7).
 * @param joystick_ch1 Joystick raw value.
 * @return Current command for GM6020 (int16).
 */
int16_t Joystick_control(uint8_t id, int16_t joystick_ch1)
{
    if (id < 1 || id > 7) return 0;
    gm6020_ctx_t *c = &g_ctx[id-1];

    float current_angle = (float)c->angle_raw / c->max_encoder * 2.0f * M_PI;
    float current_rpm   = (float)c->speed_rpm;

    // (1) Adjust target angle using joystick input
    float delta_angle = ((float)joystick_ch1 / GM6020_JOYSTICK_FULL_SCALE) * 0.02f; // radians per tick
    c->target_angle_rad += delta_angle;

    // Wrap target angle to [-π, π]
    if (c->target_angle_rad >  M_PI) c->target_angle_rad -= 2.0f * M_PI;
    if (c->target_angle_rad < -M_PI) c->target_angle_rad += 2.0f * M_PI;

    // (2) Outer loop: Position PID → target RPM
    float angle_error = fmodf(c->target_angle_rad - current_angle + M_PI, 2.0f * M_PI) - M_PI;
    float target_rpm = PID_Calculate(&c->angle_pid, 0.0f, -angle_error);

    // Clamp target RPM
    if (target_rpm >  GM6020_MAX_TARGET_RPM) target_rpm =  GM6020_MAX_TARGET_RPM;
    if (target_rpm < -GM6020_MAX_TARGET_RPM) target_rpm = -GM6020_MAX_TARGET_RPM;

    // (3) Inner loop: Speed PID → torque/current command
    int16_t cmd = (int16_t)PID_Calculate(&c->speed_pid, target_rpm, current_rpm);

    // (4) Clamp final current output
    if (id == PITCH_ID) {  // whichever motor ID handles vertical tilt
    float ang01 = ((float)c->angle_raw) / c->max_encoder;  // 0..1
    float ang_rad = ang01 * (2.0f * M_PI) + c->phase_offset_rad;
    float gravity_ff = c->pitch_direction * c->gravity_effort * sinf(ang_rad);
    cmd += gravity_ff;
}
    float max_abs = 30000.0f;
    if (cmd >  max_abs) cmd =  max_abs;
    if (cmd < -max_abs) cmd = -max_abs;

    USB_CDC_Printf("yaw angle=%.2f target=%.2f err=%.2f target_rpm=%.2f\n",
       current_angle, c->target_angle_rad,
       c->target_angle_rad - current_angle, target_rpm);
    return cmd;
}






