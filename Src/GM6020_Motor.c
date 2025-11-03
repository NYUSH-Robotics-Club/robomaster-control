#include "gm6020_motor.h"
#include "pid.h"
#include "can.h"
#include "can_manager.h"
#include <math.h>
#include "printing.h"




#define GM6020_MAX_TARGET_RPM           (50.0f)
#define GM6020_JOYSTICK_DEADZONE        (30)
#define GM6020_JOYSTICK_FULL_SCALE      (660.0f)
#define GM6020_ANGLE_HOLD_KP_RPM_PER_DEG   (100.0f)
#define GM6020_ANGLE_HOLD_MIN_RPM          (120.0f)
#define PITCH_ID 7
#define YAW_ID 6
typedef struct {
  uint8_t   id;
  uint16_t  angle_raw;         // current encoder ticks (0–8191)
  int16_t   speed_rpm;         // motor feedback speed (RPM)
  
  // --- Position Hold Tracking ---
  float     angle_target;      // target position in encoder ticks
  uint8_t   angle_inited;
  float     w_chasis_raw;
  float     angle_correction;     // flag: 0 = not initialized, 1 = initialized

  // --- PID Controllers ---
  PID_Controller speed_pid;    // optional (for cascade or debugging)
  PID_Controller angle_pid;    // primary position-hold PID

  // --- Mechanical & Physical Parameters ---
  float     phase_offset_rad;  // encoder offset when barrel points forward
  float     pitch_direction;   // +1 or -1 (mechanical sign)
  float     gravity_effort;    // torque offset to counter gravity
  float     max_encoder;       // encoder ticks per revolution (e.g. 8192)
  
  // --- Position Control Config ---
  float     target_angle_rad;  // same as angle_target but in radians (optional)
  float     angle_min;         // lower mechanical limit (ticks)
  float     angle_max;         // upper mechanical limit (ticks)
  float     joystick_sensitivity; // scale factor for joystick to tick delta
} gm6020_ctx_t;


static gm6020_ctx_t g_ctx[8];

float tar_rpm = 0.0f;
float cur_rpm = 0.0f;

#define send_current_by_id(id, cur) CAN_Manager_SendGM6020Current(&hcan2, (id), (cur))

void Motor_Init(uint8_t id, float KP, float KI, float KD, float initial_angle)
{
  if (id < 1 || id > 7) return;
  gm6020_ctx_t *c = &g_ctx[id-1];
  c->id = id;
  c->angle_raw = 0.0f;
  c->speed_rpm = 0;
  c->angle_target = initial_angle/360.0f * 8192.0f; // convert degrees to encoder ticks
  c->angle_inited = 0;
  if(id == PITCH_ID){
    c->angle_min = 1000.0f;
    c->angle_max = 4000.0f;

  } else {
    c->angle_min = 0.0f;
    c->angle_max = 8192.0f;
  }
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
  
}

void Target_Angle_Correction(SensorData sensor_data)//A function to keep the yaw stable
{
  gm6020_ctx_t *c = &g_ctx[YAW_ID-1];
  c->w_chasis_raw = sensor_data.gz;

  c->angle_correction = c->w_chasis_raw / 900 / (2 * M_PI) * c->angle_max; //900 is the weird unit for chasis gyro
  
  USB_CDC_Printf("Chasis Wz: %d | Angle Corr: %d\r\n", (int)c->w_chasis_raw, (int)c->angle_correction);
}

/**
 * @brief Joystick control: deadzone => zero target speed via PID, beyond deadzone => constant speed.
 * @param id Motor id (1..7).
 * @param joystick_ch1 Joystick raw value.
 * @return Current command for GM6020 (int16).
 */
int16_t Joystick_control(uint8_t id, int16_t joystick_ch1, SensorData sensor_data)
{
    if (id < 1 || id > 7) return 0;
    gm6020_ctx_t *c = &g_ctx[id-1];

    int16_t raw = joystick_ch1;
    c->max_encoder = 8192.0f;     // GM6020 encoder ticks per revolution
    c->pitch_direction = -1.0f;    // +1 = normal, -1 = inverted
    c->gravity_effort = 5000.0f;  // feed-forward magnitude
    float sensitivity = 15.0f;     // joystick sensitivity in ticks per input step



    // --- Incremental Target Update (Position-Hold Style) ---
    if (raw > GM6020_JOYSTICK_DEADZONE || raw < -GM6020_JOYSTICK_DEADZONE)
    {
        // Move target proportionally to joystick input
       
        c->angle_target += c->pitch_direction * sensitivity * ((float)raw / GM6020_JOYSTICK_FULL_SCALE);
    }
    
    if(id == YAW_ID){
        Target_Angle_Correction(sensor_data);
         c->angle_target += c->angle_correction;
    }
    
    // --- Wrap Around Encoder Range (0..8192) ---
    if(id == PITCH_ID){
      
        if (c->angle_target > c->angle_max)
        c->angle_target = c->angle_max;
        if (c->angle_target < c->angle_min)
        c->angle_target = c->angle_min;
    
    } else {
      
        if (c->angle_target >= c->angle_max)
            c->angle_target = c->angle_min;
        else if (c->angle_target < c->angle_min)
            c->angle_target = c->angle_max;
    }

    
    

    // --- Run Position PID ---
    float current_angle = (float)c->angle_raw;

    // Calculate shortest path error considering encoder wrap-around
    float error = c->angle_target - current_angle;
    if (error > c->max_encoder / 2.0f)
        error -= c->max_encoder;
    else if (error < -c->max_encoder / 2.0f)
        error += c->max_encoder;

    float cmd = PID_Calculate(&c->angle_pid, 0.0f, -error);

    if(id == PITCH_ID){
        float ang01 = current_angle / c->max_encoder; // 0..1 fraction of revolution
        float ang_rad = ang01 * (2.0f * (float)M_PI);
        float gravity_ff = c->pitch_direction * c->gravity_effort * sinf(ang_rad);
        cmd += gravity_ff;
    }
    // --- Clamp Output ---
    float max_abs = 25000.0f;
    if (cmd >  max_abs) cmd =  max_abs;
    if (cmd < -max_abs) cmd = -max_abs;
    // USB_CDC_Printf("GM6020 ID=%d | Target=%d | Current=%d | Cmd=%d\r\n",
    //     c->id, (int)c->angle_target, (int)current_angle, (int)cmd);

    
    return (int16_t)cmd;
}



 




