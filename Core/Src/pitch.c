#include "gimbal.h"
#include "pid.h"
#include "can.h"
#include "can_manager.h"
#include <math.h>

#define GM6020_MAX_TARGET_RPM              (50.0f)
#define GM6020_JOYSTICK_DEADZONE           (30)
#define GM6020_JOYSTICK_FULL_SCALE         (660.0f)
#define GM6020_ANGLE_HOLD_KP_RPM_PER_DEG   (100.0f)
#define GM6020_ANGLE_HOLD_MIN_RPM          (10.0f)

// Encoder and geometry
#define GM6020_ENCODER_CPR                 (8192.0f)
#define PITCH_PI                           (3.14159265358979323846f)

// Gravity feedforward parameters (tunable)
#define GM6020_PITCH_ZERO_RAW_DEFAULT      (0)     // set to horizontal encoder raw
#define GM6020_PITCH_DIR_SIGN              (1)     // +1 or -1 depending on mounting
#define GM6020_GRAVITY_K                   (1500.0f)
#define GM6020_STATIC_FRICTION_CURRENT     (200.0f)
#define GM6020_STATIC_SPEED_EPS_RPM        (5.0f)

// Hold behavior
#define GM6020_ANGLE_HOLD_DEG_EPS          (0.3f)
#define GM6020_HOLD_MAX_RPM                (GM6020_MAX_TARGET_RPM)
#define GM6020_DECAY_INTEGRAL              (0.90f)

// Software clockwise limit relative to power-on angle
#define GM6020_CW_LIMIT_DEG                 (60.0f)
#define GM6020_CW_SIGN                      (1)    // +1 if encoder counts increase when rotating clockwise, else -1
#define GM6020_POSITIVE_CMD_INCREASES_COUNTS (1)   // +1 if positive command drives counts increasing

typedef struct {
  uint8_t   id;
  uint16_t  angle_raw;
  int16_t   speed_rpm;
  int32_t   hold_angle_raw;
  uint8_t   hold_inited;
  int32_t   zero_raw;     // encoder raw value at horizontal
  int8_t    dir_sign;     // +1 or -1
  int32_t   home_angle_raw; // power-on reference angle
  PID_Controller speed_pid;
} gm6020_ctx_t;

static gm6020_ctx_t g_ctx[8];

#define send_current_by_id(id, cur) CAN_Manager_SendGM6020Current(&hcan1, (id), (cur))

static inline int32_t wrap_counts_diff(int32_t diff)
{
  if (diff > 4096) diff -= 8192;
  if (diff < -4096) diff += 8192;
  return diff;
}

static inline float raw_to_theta_rad(uint16_t angle_raw, int32_t zero_raw, int8_t dir_sign)
{
  int32_t diff = (int32_t)angle_raw - zero_raw;
  diff = wrap_counts_diff(diff);
  float revolutions = (float)diff / GM6020_ENCODER_CPR;
  return (float)dir_sign * (revolutions * 2.0f * PITCH_PI);
}

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int32_t deg_to_counts(float deg)
{
  return (int32_t)(deg * (GM6020_ENCODER_CPR / 360.0f));
}

void pitch_init(uint8_t id)
{
  if (id < 1 || id > 7) return;
  gm6020_ctx_t *c = &g_ctx[id-1];
  c->id = id;
  c->angle_raw = 0;
  c->speed_rpm = 0;
  c->hold_angle_raw = 0;
  c->hold_inited = 0;
  c->zero_raw = GM6020_PITCH_ZERO_RAW_DEFAULT;
  c->dir_sign = (int8_t)GM6020_PITCH_DIR_SIGN;
  c->home_angle_raw = 0;
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
    c->home_angle_raw = angle_raw;  // record power-on/home angle
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

  // Gravity feedforward (based on current angle)
  float theta = raw_to_theta_rad(c->angle_raw, c->zero_raw, c->dir_sign);
  float I_g = GM6020_GRAVITY_K * sinf(theta);
  if (fabsf(current_rpm) < GM6020_STATIC_SPEED_EPS_RPM) {
    I_g += (sinf(theta) >= 0.0f ? 1.0f : -1.0f) * GM6020_STATIC_FRICTION_CURRENT;
  }

  // Compute clockwise delta from home (in encoder counts)
  int32_t cw_delta_counts = GM6020_CW_SIGN * wrap_counts_diff((int32_t)c->angle_raw - (int32_t)c->home_angle_raw);
  int32_t cw_limit_counts = deg_to_counts(GM6020_CW_LIMIT_DEG);

  // Within deadzone: position hold + gravity feedforward
  if (raw > -GM6020_JOYSTICK_DEADZONE && raw < GM6020_JOYSTICK_DEADZONE)
  {
    if (!c->hold_inited) {
      c->hold_angle_raw = c->angle_raw;
      c->hold_inited = 1;
    }

    int32_t diff_counts = wrap_counts_diff((int32_t)c->hold_angle_raw - (int32_t)c->angle_raw);
    float error_deg = ((float)diff_counts) * (360.0f / GM6020_ENCODER_CPR);
    float target_rpm = GM6020_ANGLE_HOLD_KP_RPM_PER_DEG * error_deg;
    target_rpm = clampf(target_rpm, -GM6020_HOLD_MAX_RPM, GM6020_HOLD_MAX_RPM);

    if (fabsf(error_deg) > GM6020_ANGLE_HOLD_DEG_EPS) {
      float min_mag = GM6020_ANGLE_HOLD_MIN_RPM;
      if (fabsf(target_rpm) < min_mag) {
        target_rpm = (target_rpm >= 0.0f ? 1.0f : -1.0f) * min_mag;
      }
    } else {
      // near target, soften integral to avoid residual torque
      c->speed_pid.integral *= GM6020_DECAY_INTEGRAL;
    }

    int16_t cmd = (int16_t)PID_Calculate(&c->speed_pid, target_rpm, current_rpm);
    int32_t cmd_ff = (int32_t)cmd + (int32_t)I_g;
    // Enforce CW software limit: block commands that would further increase CW delta beyond limit
    if (cw_delta_counts >= cw_limit_counts && (cmd_ff * GM6020_POSITIVE_CMD_INCREASES_COUNTS * GM6020_CW_SIGN) > 0) {
      cmd_ff = 0;
    }
    if (cmd_ff > 30000) cmd_ff = 30000;
    if (cmd_ff < -30000) cmd_ff = -30000;
    return (int16_t)cmd_ff;
  }

  // Beyond deadzone: run at constant speed toward stick direction + gravity feedforward
  float target_rpm = ((float)raw / GM6020_JOYSTICK_FULL_SCALE) * GM6020_MAX_TARGET_RPM;
  if (target_rpm >  GM6020_MAX_TARGET_RPM) target_rpm =  GM6020_MAX_TARGET_RPM;
  if (target_rpm < -GM6020_MAX_TARGET_RPM) target_rpm = -GM6020_MAX_TARGET_RPM;

  // If at CW limit and command requests further CW motion, block it
  if (cw_delta_counts >= cw_limit_counts && (target_rpm * GM6020_POSITIVE_CMD_INCREASES_COUNTS * GM6020_CW_SIGN) > 0.0f) {
    target_rpm = 0.0f;
    c->speed_pid.integral *= GM6020_DECAY_INTEGRAL;
  }

  // Update hold angle to current so re-entering deadzone does not jerk back
  c->hold_angle_raw = c->angle_raw;

  int16_t cmd = (int16_t)PID_Calculate(&c->speed_pid, target_rpm, current_rpm);
  int32_t cmd_ff = (int32_t)cmd + (int32_t)I_g;
  // Enforce CW software limit for final command as well
  if (cw_delta_counts >= cw_limit_counts && (cmd_ff * GM6020_POSITIVE_CMD_INCREASES_COUNTS * GM6020_CW_SIGN) > 0) {
    cmd_ff = 0;
  }
  if (cmd_ff > 30000) cmd_ff = 30000;
  if (cmd_ff < -30000) cmd_ff = -30000;
  return (int16_t)cmd_ff;
}


