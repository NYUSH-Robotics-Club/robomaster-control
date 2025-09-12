#include "gimbal.h"
#include "pid.h"
#include "can.h"
#include "can_manager.h"

#define GM6020_MAX_TARGET_RPM           (50.0f)
#define GM6020_JOYSTICK_DEADZONE        (30)
#define GM6020_JOYSTICK_FULL_SCALE      (660.0f)
#define GM6020_ANGLE_HOLD_KP_RPM_PER_DEG   (100.0f)
#define GM6020_ANGLE_HOLD_MIN_RPM          (120.0f)

typedef struct {
  uint8_t   id;
  uint16_t  angle_raw;
  int16_t   speed_rpm;
  int32_t   hold_angle_raw;
  uint8_t   hold_inited;
  PID_Controller speed_pid;
} gm6020_ctx_t;

static gm6020_ctx_t g_ctx[8];

#define send_current_by_id(id, cur) CAN_Manager_SendGM6020Current(&hcan1, (id), (cur))

void pitch_init(uint8_t id)
{
  if (id < 1 || id > 7) return;
  gm6020_ctx_t *c = &g_ctx[id-1];
  c->id = id;
  c->angle_raw = 0;
  c->speed_rpm = 0;
  c->hold_angle_raw = 0;
  c->hold_inited = 0;
  PID_Init(&c->speed_pid, 10.0f, 0.3f, 0.05f, 30000.0f, 25000.0f);
}

void pitch_on_feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm)
{
  if (id < 1 || id > 7) return;
  gm6020_ctx_t *c = &g_ctx[id-1];
  c->angle_raw = angle_raw;
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
int16_t pitch_control_from_joystick(uint8_t id, int16_t joystick_ch1)
{
  if (id < 1 || id > 7) return 0;
  gm6020_ctx_t *c = &g_ctx[id-1];

  int16_t raw = joystick_ch1;
  float current_rpm = (float)c->speed_rpm;

  // Within deadzone: target speed = 0
  if (raw > -GM6020_JOYSTICK_DEADZONE && raw < GM6020_JOYSTICK_DEADZONE)
  {
    // Immediate stop: zero output, clear integral to avoid residual torque
    c->speed_pid.integral = 0.0f;
    return 0;
  }

  // Beyond deadzone: run at constant speed toward stick direction
  float target_rpm = ((float)raw / GM6020_JOYSTICK_FULL_SCALE) * GM6020_MAX_TARGET_RPM;
  if (target_rpm >  GM6020_MAX_TARGET_RPM) target_rpm =  GM6020_MAX_TARGET_RPM;
  if (target_rpm < -GM6020_MAX_TARGET_RPM) target_rpm = -GM6020_MAX_TARGET_RPM;
  int16_t cmd = (int16_t)PID_Calculate(&c->speed_pid, target_rpm, current_rpm);
  return cmd;
}


