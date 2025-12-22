#include "gimbal_controller.h"
#include "gm6020_motor.h"
#include "pid.h"
#include "message_center.h"
#include "can.h"
#include "can_manager.h"
#include <math.h>
#include <string.h>
#include "printing.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// Control parameters (local aliases for readability)
#define PITCH_ID 7
#define YAW_ID 6

// Yaw control parameters

#define YAW_CONTROL_ENC_MAX         (8192.0f)
#define YAW_CONTROL_GYRO_LPF_ALPHA (0.3f)
#define YAW_CONTROL_JOY_SENSITIVITY (50.0f)





// PID parameters
#define YAW_KP        (0.25f)      // was 0.5f
#define YAW_KI        (0.0008f)   // was 0.002f (x ~13 smaller)
#define YAW_KD        (0.04f)
#define YAW_SPEED_KP  (30.0f)
#define YAW_SPEED_KI  (0.10f)
#define YAW_SPEED_KD  (3.0f)
#define CURRENT_LIMIT (25000.0f)
#define PITCH_KP (20.0f)
#define PITCH_KI (0.0f)
#define PITCH_KD (2.0f)
#define INITIAL_PITCH_ANGLE (-1.0f)
#define INITIAL_YAW_ANGLE (0.0f)
#define YAW_FF_JOY_RPM_GAIN   (140.0f) 


#define YAW_SETTLING_THRESHOLD (200.0f)  // Error threshold for "near target"
#define YAW_SETTLING_SPEED_LIMIT (10.0f) // Max speed when near target


#define YAW_RPM_MAX               (220.0f)    
#define YAW_RPM_MIN               (25.0f)    
#define YAW_ERROR_FOR_FULL_SPEED  (1200.0f) 
// Static state for application
static GimbalCmd s_last_cmd;
static SensorData s_last_sensor;
static bool s_initialized = false;

void last_data(float last_yaw_rate, float last_yaw_target)
{
   s_last_cmd.yaw_rate_memo = last_yaw_rate;
   s_last_cmd.yaw_target_memo = last_yaw_target;
}

void GimbalController_Init(float yaw_kp, float yaw_ki, float yaw_kd, float yaw_initial_angle,
                           float pitch_kp, float pitch_ki, float pitch_kd, float pitch_initial_angle)
{
    // Motor_Init already initializes motor parameters in module layer
    Motor_Init(GIMBAL_YAW_ID, yaw_kp, yaw_ki, yaw_kd, yaw_initial_angle, 300.0f, 150.0f);
    Motor_Init(GIMBAL_PITCH_ID, pitch_kp, pitch_ki, pitch_kd, pitch_initial_angle, 30000.0f, 25000.0f);
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
    float sensitivity = 40.0f;

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

    USB_CDC_Printf("PITCH_CSV,%lu,%.2f,%.2f,%d,%.2f,%.2f,%.2f\r\n",
                   HAL_GetTick(),
                   c->angle_target,
                   current_angle,
                   c->speed_rpm,
                   cmd,
                   error,
                   rate_normalized * 300.0f);


    return (int16_t)cmd;
}

int16_t GimbalController_YawControlWithCompensation(float rate_normalized, SensorData* sensor_data)
{
    GM6020_MotorContext *yaw = GM6020_GetContext(GIMBAL_YAW_ID);
    if (!yaw || !yaw->angle_inited) return 0;

    // Filter gyro
    

    // Joystick → target angle
    yaw->angle_target += YAW_CONTROL_JOY_SENSITIVITY * rate_normalized;

    // Counter-rotation compensation
    //yaw->angle_target -= sensor_data->c_gz / 4.75f;

    // Wrap target into encoder range
    if (yaw->angle_target >= YAW_CONTROL_ENC_MAX)
        yaw->angle_target -= YAW_CONTROL_ENC_MAX;
    else if (yaw->angle_target < 0)
        yaw->angle_target += YAW_CONTROL_ENC_MAX;

    float current = yaw->angle_raw;
    float angle_error = yaw->angle_target - current;

    // small deadband
    if (fabsf(angle_error) < 1.0f)
        angle_error = 0.0f;

    // wrap error into [-ENC_MAX/2, ENC_MAX/2]
    if (angle_error >  YAW_CONTROL_ENC_MAX / 2.0f) angle_error -= YAW_CONTROL_ENC_MAX;
    if (angle_error < -YAW_CONTROL_ENC_MAX / 2.0f) angle_error += YAW_CONTROL_ENC_MAX;

    // ==========================
    // OUTER LOOP: angle → speed
    // ==========================
    float cmd_angle_to_speed = PID_Calculate(&yaw->angle_pid, 0.0f, -angle_error);
    float ff_scale;
    float abs_err = fabsf(angle_error);
    if (abs_err >= YAW_ERROR_FOR_FULL_SPEED) {
        ff_scale = 1.0f;                      // far away: full FF
    } else {
        ff_scale = abs_err / YAW_ERROR_FOR_FULL_SPEED;  // 0..1
    }

    float speed_ff = rate_normalized * YAW_FF_JOY_RPM_GAIN * ff_scale;
    cmd_angle_to_speed += speed_ff;

    // ---- error-based max rpm (far → fast, near → slow) ----
    float rpm_limit;
    if (abs_err >= YAW_ERROR_FOR_FULL_SPEED) {
        rpm_limit = YAW_RPM_MAX;
    } else {
        float t = abs_err / YAW_ERROR_FOR_FULL_SPEED;       // 0..1
        rpm_limit = YAW_RPM_MIN + t * (YAW_RPM_MAX - YAW_RPM_MIN);
    }

    // Soft stop zone: when very close, fade speed to zero
    const float SOFT_STOP_ERR = 300.0f; // ticks
    if (abs_err < SOFT_STOP_ERR) {
        float soft = abs_err / SOFT_STOP_ERR;   // 0..1
        cmd_angle_to_speed *= soft;            // shrink command as we approach
    }

    // Apply signed clamp
    if (cmd_angle_to_speed >  rpm_limit) cmd_angle_to_speed =  rpm_limit;
    if (cmd_angle_to_speed < -rpm_limit) cmd_angle_to_speed = -rpm_limit;
    float cmd_speed_to_current =
        PID_Calculate(&yaw->speed_pid, cmd_angle_to_speed, yaw->speed_rpm);

    // Clamp current
    if (cmd_speed_to_current >  CURRENT_LIMIT) cmd_speed_to_current =  CURRENT_LIMIT;
    if (cmd_speed_to_current < -CURRENT_LIMIT) cmd_speed_to_current = -CURRENT_LIMIT;

    // LOGGING (unchanged format)
    uint32_t timestamp = HAL_GetTick();
    last_data(rate_normalized, yaw->angle_target);

    float g_gz_filt = sensor_data->g_gz * YAW_CONTROL_GYRO_LPF_ALPHA +
                      s_last_sensor.g_gz * (1.0f - YAW_CONTROL_GYRO_LPF_ALPHA);
    USB_CDC_Printf("YAW_CSV,%lu,%.2f,%.2f,%d,%.2f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f\r\n",
                   timestamp,
                   yaw->angle_target,
                   current,
                   yaw->speed_rpm,
                   cmd_speed_to_current,
                   cmd_angle_to_speed,
                   rate_normalized * 300,
                   angle_error,
                   g_gz_filt,
                   sensor_data->c_gz);

    return (int16_t)cmd_speed_to_current;
}


// Application layer: Message subscription callbacks
static void on_gimbal_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(GimbalCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(GimbalCmd));
        
        // Execute gimbal control when command arrives
        if (s_last_cmd.enabled) {
            static bool s_yaw_vision_active = false;
            bool use_vision_target = s_last_cmd.vision_valid;

            // Continuous angle control: update target angle every cycle when vision is valid
            if (use_vision_target) {
                GM6020_MotorContext *yaw = GM6020_GetContext(GIMBAL_YAW_ID);
                GM6020_MotorContext *pitch = GM6020_GetContext(GIMBAL_PITCH_ID);

                if (yaw && yaw->angle_inited && yaw->max_encoder > 0.0f) {
                    const float ticks_per_rad = yaw->max_encoder / (2.0f * (float)M_PI);
                    float err_ticks = s_last_cmd.vision_yaw_err_rad * ticks_per_rad;
                    // Update target angle continuously based on current angle + vision error
                    yaw->angle_target = (float)yaw->angle_raw + err_ticks;
                    while (yaw->angle_target >= yaw->max_encoder) yaw->angle_target -= yaw->max_encoder;
                    while (yaw->angle_target < 0.0f) yaw->angle_target += yaw->max_encoder;
                    
                    if (!s_yaw_vision_active) {
                        PID_Reset(&yaw->angle_pid);
                        PID_Reset(&yaw->speed_pid);
                    }
                    s_yaw_vision_active = true;
                }

                (void)pitch;
            } else {
                s_yaw_vision_active = false;
            }

            int16_t pitch_current = GimbalController_PitchControl(
                GIMBAL_PITCH_ID, 
                use_vision_target ? 0.0f : s_last_cmd.pitch_rate, 
                &s_last_sensor
            );
            int16_t yaw_current = GimbalController_YawControlWithCompensation(
                use_vision_target ? 0.0f : s_last_cmd.yaw_rate, 
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
   
    Motor_Init(GIMBAL_YAW_ID, YAW_KP, YAW_KI, YAW_KD, INITIAL_YAW_ANGLE, 300.0f, 16000.0f);
    Motor_Init(GIMBAL_PITCH_ID, PITCH_KP, PITCH_KI, PITCH_KD, INITIAL_PITCH_ANGLE, 30000.0f, 25000.0f);



    Yaw_Speed_PID_Init(GIMBAL_YAW_ID, YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_KD);
    
    // Subscribe to messages
    (void)MsgCenter_Subscribe(TOPIC_GIMBAL_CMD, on_gimbal_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    
    s_initialized = true;
}
