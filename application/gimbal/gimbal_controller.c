#include "gimbal_controller.h"
#include "gm6020_motor.h"
#include "pid.h"
#include "message_center.h"
#include "can.h"
#include "can_manager.h"
#include <math.h>
#include <string.h>
#include "printing.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// Control parameters (local aliases for readability)
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

// PID parameters
#define YAW_KP (10.0f)
#define YAW_KI (0.05f)
#define YAW_KD (0.1f)
#define PITCH_KP (11.0f)
#define PITCH_KI (0.0f)
#define PITCH_KD (0.1f)
#define INITIAL_PITCH_ANGLE (-1.0f)
#define INITIAL_YAW_ANGLE (0.0f)

// Static state for application
static GimbalCmd s_last_cmd;
static SensorData s_last_sensor;
static bool s_initialized = false;

void GimbalController_Init(float yaw_kp, float yaw_ki, float yaw_kd, float yaw_initial_angle,
                           float pitch_kp, float pitch_ki, float pitch_kd, float pitch_initial_angle)
{
    // Motor_Init already initializes motor parameters in module layer
    Motor_Init(GIMBAL_YAW_ID, yaw_kp, yaw_ki, yaw_kd, yaw_initial_angle);
    Motor_Init(GIMBAL_PITCH_ID, pitch_kp, pitch_ki, pitch_kd, pitch_initial_angle);
}

int16_t GimbalController_PitchControl(uint8_t id, float rate_normalized, SensorData* sensor_data)
{
    (void)sensor_data;  // Not used for pitch
    if (id < 1 || id > 7) return 0;
    GM6020_MotorContext *c = GM6020_GetContext(id);
    if (!c || !c->angle_inited) {
        return 0;
    }

    // Sensitivity: how much angle to add per control cycle for full stick deflection
    float sensitivity = 15.0f;

    // Update target angle based on normalized rate command (-1.0 to 1.0)
    c->angle_target += c->pitch_direction * sensitivity * rate_normalized;

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

int16_t GimbalController_YawControlWithCompensation(float rate_normalized, SensorData* sensor_data)
{
    GM6020_MotorContext *yaw = GM6020_GetContext(GIMBAL_YAW_ID);
    if (!yaw || !yaw->angle_inited) return 0;

    // Apply low-pass filter to smooth joystick input (already normalized -1.0 to 1.0)
    static float joy_smoothed = 0.0f;
    joy_smoothed = YAW_CONTROL_JOY_RAMP_ALPHA * rate_normalized + (1.0f - YAW_CONTROL_JOY_RAMP_ALPHA) * joy_smoothed;

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

// Application layer: Message subscription callbacks
static void on_gimbal_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(GimbalCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(GimbalCmd));
        
        // Execute gimbal control when command arrives
        if (s_last_cmd.enabled) {
            int16_t pitch_current = GimbalController_PitchControl(
                GIMBAL_PITCH_ID, 
                s_last_cmd.pitch_rate, 
                &s_last_sensor
            );
            int16_t yaw_current = GimbalController_YawControlWithCompensation(
                s_last_cmd.yaw_rate, 
                &s_last_sensor
            );
            
            // Send CAN commands
            CAN_Manager_SendGM6020Current(&hcan2, GIMBAL_PITCH_ID, pitch_current);
            CAN_Manager_SendGM6020Current(&hcan1, GIMBAL_YAW_ID, yaw_current);
        } else {
            // Gimbal disabled, send zero current
            CAN_Manager_SendGM6020Current(&hcan2, GIMBAL_PITCH_ID, 0);
            CAN_Manager_SendGM6020Current(&hcan1, GIMBAL_YAW_ID, 0);
        }
    }
}

static void on_imu_update(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

void GimbalApp_Init(void) {
    if (s_initialized) {
        return;
    }
    
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));
    
    // Initialize gimbal controller
    GimbalController_Init(
        YAW_KP, YAW_KI, YAW_KD, INITIAL_YAW_ANGLE,
        PITCH_KP, PITCH_KI, PITCH_KD, INITIAL_PITCH_ANGLE
    );
    
    // Subscribe to messages
    (void)MsgCenter_Subscribe(TOPIC_GIMBAL_CMD, on_gimbal_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    
    s_initialized = true;
}
