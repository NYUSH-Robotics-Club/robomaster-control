#include "gimbal_controller.h"
#include "gm6020_motor.h"
#include "pid.h"
#include <math.h>
#include "printing.h"

// Control parameters
#define GM6020_JOYSTICK_DEADZONE        (30)
#define GM6020_JOYSTICK_FULL_SCALE      (660.0f)
#define PITCH_ID 7
#define YAW_ID 6

// Yaw control parameters
#define YAW_CONTROL_DT              (0.005f)
#define YAW_CONTROL_ENC_MAX         (8192.0f)
#define YAW_CONTROL_TICKS_PER_RAD  (YAW_CONTROL_ENC_MAX / (2.0f * (float)M_PI))
#define YAW_CONTROL_COUNTER_GAIN   (1.4f)
#define YAW_CONTROL_RATE_FEEDBACK  (0.8f)
#define YAW_CONTROL_GYRO_LPF_ALPHA (0.3f)
#define YAW_CONTROL_JOY_SENSITIVITY (20.0f)
#define YAW_CONTROL_JOY_RAMP_ALPHA  (0.10f)

void GimbalController_Init(float yaw_kp, float yaw_ki, float yaw_kd, float yaw_initial_angle,
                           float pitch_kp, float pitch_ki, float pitch_kd, float pitch_initial_angle)
{
    // Motor_Init already initializes motor parameters in module layer
    Motor_Init(GIMBAL_YAW_ID, yaw_kp, yaw_ki, yaw_kd, yaw_initial_angle);
    Motor_Init(GIMBAL_PITCH_ID, pitch_kp, pitch_ki, pitch_kd, pitch_initial_angle);
}

int16_t GimbalController_JoystickControl(uint8_t id, int16_t joystick_ch1, SensorData* sensor_data)
{
    if (id < 1 || id > 7) return 0;
    GM6020_MotorContext *c = GM6020_GetContext(id);
    if (!c || !c->angle_inited) {
        return 0;
    }

    int16_t raw = joystick_ch1;
    float sensitivity = 15.0f;

    if (raw > GM6020_JOYSTICK_DEADZONE || raw < -GM6020_JOYSTICK_DEADZONE)
    {
        c->angle_target += c->pitch_direction * sensitivity * ((float)raw / GM6020_JOYSTICK_FULL_SCALE);
    }

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

    float current_angle = (float)c->angle_raw;
    float error = c->angle_target - current_angle;
    if (error > c->max_encoder / 2.0f)
        error -= c->max_encoder;
    else if (error < -c->max_encoder / 2.0f)
        error += c->max_encoder;

    float cmd = PID_Calculate(&c->angle_pid, error, 0.0f);

    if(id == PITCH_ID){
        float ang01 = current_angle / c->max_encoder;
        float ang_rad = ang01 * (2.0f * (float)M_PI);
        float gravity_ff = c->pitch_direction * c->gravity_effort * sinf(ang_rad);
        cmd += gravity_ff;
    }
    float max_abs = 25000.0f;
    if (cmd >  max_abs) cmd =  max_abs;
    if (cmd < -max_abs) cmd = -max_abs;
    if(id == YAW_ID){
        USB_CDC_Printf("GM6020 ID=%d | Target=%d | Current=%d | Cmd=%d\r\n",
         c->id, (int)c->angle_target, (int)current_angle, (int)cmd);
    }
    return (int16_t)cmd;
}

int16_t GimbalController_YawControlWithCompensation(int16_t joystick_yaw, SensorData* sensor_data)
{
    GM6020_MotorContext *yaw = GM6020_GetContext(GIMBAL_YAW_ID);
    if (!yaw || !yaw->angle_inited) return 0;

    static float joy_smoothed = 0.0f;
    float joy_input = 0.0f;

    if (fabsf((float)joystick_yaw) > GM6020_JOYSTICK_DEADZONE)
        joy_input = (float)joystick_yaw / GM6020_JOYSTICK_FULL_SCALE;

    joy_smoothed = YAW_CONTROL_JOY_RAMP_ALPHA * joy_input + (1.0f - YAW_CONTROL_JOY_RAMP_ALPHA) * joy_smoothed;

    yaw->angle_target += YAW_CONTROL_JOY_SENSITIVITY * joy_smoothed;

    static float g_gz_filt = 0.0f;
    const float g_gz = sensor_data->g_gz;
    const float c_gz = sensor_data->c_gz * (float)M_PI / 180.0f;

    g_gz_filt = YAW_CONTROL_GYRO_LPF_ALPHA * g_gz + (1.0f - YAW_CONTROL_GYRO_LPF_ALPHA) * g_gz_filt;

    float dYaw_ticks = (-YAW_CONTROL_COUNTER_GAIN * c_gz - YAW_CONTROL_RATE_FEEDBACK * g_gz_filt)
                       * YAW_CONTROL_TICKS_PER_RAD * YAW_CONTROL_DT;
    yaw->angle_target += dYaw_ticks;

    if (yaw->angle_target >= YAW_CONTROL_ENC_MAX)
        yaw->angle_target -= YAW_CONTROL_ENC_MAX;
    else if (yaw->angle_target < 0)
        yaw->angle_target += YAW_CONTROL_ENC_MAX;

    const float current = (float)yaw->angle_raw;
    float raw_err = yaw->angle_target - current;

    if (raw_err >  YAW_CONTROL_ENC_MAX / 2.0f) raw_err -= YAW_CONTROL_ENC_MAX;
    if (raw_err < -YAW_CONTROL_ENC_MAX / 2.0f) raw_err += YAW_CONTROL_ENC_MAX;

    float cmd = PID_Calculate(&yaw->angle_pid, raw_err, 0.0f);

    if (cmd >  25000.0f) cmd =  25000.0f;
    if (cmd < -25000.0f) cmd = -25000.0f;

    return (int16_t)cmd;
}

void GimbalController_TargetAngleCorrection(SensorData* sensor_data)
{
    GM6020_MotorContext *c = GM6020_GetContext(GIMBAL_YAW_ID);
    if (!c) return;
    
    c->w_chasis_raw = sensor_data->c_gz;
    c->angle_correction = c->w_chasis_raw / 900.0f / (2.0f * (float)M_PI) * c->angle_max / 120.0f;
    USB_CDC_Printf("Chasis Wz: %d | Angle Corr: %d|Head Wz: %f\r\n", 
                   (int)c->w_chasis_raw, (int)c->angle_correction, (float)sensor_data->g_gz);
}
