#include "gm6020.h"
#include "pid.h"
#include "can.h"

#define GM6020_MAX_TARGET_RPM           (200.0f)
#define GM6020_JOYSTICK_DEADZONE        (80)
#define GM6020_JOYSTICK_FULL_SCALE      (660.0f)
#define GM6020_ANGLE_HOLD_KP_RPM_PER_DEG   (50.0f)

typedef struct {
  uint8_t   id;
  uint16_t  angle_raw;
  int16_t   speed_rpm;
  int32_t   hold_angle_raw;
  uint8_t   hold_inited;
  PID_Controller speed_pid;
} gm6020_ctx_t;

static gm6020_ctx_t g_ctx[8];

static HAL_StatusTypeDef send_current_by_id(uint8_t id, int16_t cur)
{
  if (id < 1 || id > 7) return HAL_ERROR;
  if (cur >  30000) cur =  30000;
  if (cur < -30000) cur = -30000;

  uint16_t stdId = (id <= 4) ? 0x1FF : 0x2FF;
  uint8_t  slot  = (id <= 4) ? (uint8_t)(id - 1) : (uint8_t)(id - 5);

  CAN_TxHeaderTypeDef tx = {0};
  uint8_t d[8] = {0};
  uint32_t mb;

  tx.StdId = stdId;
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = 8;

  d[slot*2 + 0] = (uint8_t)((cur >> 8) & 0xFF);
  d[slot*2 + 1] = (uint8_t)( cur       & 0xFF);

  return HAL_CAN_AddTxMessage(&hcan1, &tx, d, &mb);
}

void gm6020_init(uint8_t id)
{
  if (id < 1 || id > 7) return;
  gm6020_ctx_t *c = &g_ctx[id-1];
  c->id = id;
  c->angle_raw = 0;
  c->speed_rpm = 0;
  c->hold_angle_raw = 0;
  c->hold_inited = 0;
  PID_Init(&c->speed_pid, 6.0f, 0.1f, 0.1f, 20000.0f, 15000.0f);
}

void gm6020_on_feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm)
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

void gm6020_control_from_joystick(uint8_t id, int16_t joystick_ch1)
{
  if (id < 1 || id > 7) return;
  gm6020_ctx_t *c = &g_ctx[id-1];

  int16_t raw = joystick_ch1;
  float current_rpm = (float)c->speed_rpm;

  if (raw > -GM6020_JOYSTICK_DEADZONE && raw < GM6020_JOYSTICK_DEADZONE)
  {
    // center hold by angle
    int32_t cur = c->angle_raw;
    int32_t err = cur - c->hold_angle_raw;
    if (err >  (int32_t)(8192/2)) err -= 8192;
    if (err < -(int32_t)(8192/2)) err += 8192;
    float err_deg = (float)err * (360.0f / 8192.0f);
    float target_rpm = -err_deg * GM6020_ANGLE_HOLD_KP_RPM_PER_DEG;
    if (target_rpm >  GM6020_MAX_TARGET_RPM) target_rpm =  GM6020_MAX_TARGET_RPM;
    if (target_rpm < -GM6020_MAX_TARGET_RPM) target_rpm = -GM6020_MAX_TARGET_RPM;
    int16_t cmd = (int16_t)PID_Calculate(&c->speed_pid, target_rpm, current_rpm);
    (void)send_current_by_id(id, cmd);
  }
  else
  {
    float x = (float)raw / GM6020_JOYSTICK_FULL_SCALE; // -1..1
    if (x >  1.0f) x =  1.0f;
    if (x < -1.0f) x = -1.0f;
    float shaped = x * x * x;
    float target_rpm = shaped * GM6020_MAX_TARGET_RPM;
    c->hold_angle_raw = c->angle_raw;
    c->hold_inited = 1;
    int16_t cmd = (int16_t)PID_Calculate(&c->speed_pid, target_rpm, current_rpm);
    (void)send_current_by_id(id, cmd);
  }
}


