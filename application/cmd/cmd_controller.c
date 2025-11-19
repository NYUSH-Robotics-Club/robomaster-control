#include "cmd_controller.h"
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "vision_comm.h"
#include "printing.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#define SAMPLE_COUNT 50
#define REFRESH_HZ   200
#define REFRESH_DT   (1.0 / REFRESH_HZ)

static float x_buf[SAMPLE_COUNT];
static float y_buf[SAMPLE_COUNT];
static int index_linear= 0;
static int filled = 0;

static float new_y = 0.0f;  // 现在外部只输入 y
static unsigned long tick = 0;
static float yaw_raw= 0.0f; 
static float yaw_corrected=0.0f; // 固定时间序号，用来生成 x

// Local state storage
static RC_ctrl_t s_last_rc;
static SensorData s_last_sensor;
static Vision_Recv_s s_last_vision;
static bool s_initialized = false;

// Command messages to publish
static ChassisCmd s_chassis_cmd;
static ShootCmd s_shoot_cmd;
static GimbalCmd s_gimbal_cmd;

// Deadband for joystick input
#define JOYSTICK_DEADBAND 10

// Callback for RC update
static void on_rc_update(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(RC_ctrl_t)) {
        memcpy(&s_last_rc, ev->data, sizeof(RC_ctrl_t));
    }
}

// Callback for IMU update
static void on_imu_update(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

// Callback for vision data update
static void on_vision_update(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(Vision_Recv_s)) {
        memcpy(&s_last_vision, ev->data, sizeof(Vision_Recv_s));
    }
}

// Apply deadband to joystick input
static int16_t apply_deadband(int16_t value, int16_t deadband) {
    if (value > -deadband && value < deadband) {
        return 0;
    }
    return value;
}

// Process chassis control commands
static void process_chassis_command(const RC_ctrl_t *rc) {
    if (rc == NULL) {
        // RC disconnected, stop chassis
        s_chassis_cmd.vx = 0.0f;
        s_chassis_cmd.vy = 0.0f;
        s_chassis_cmd.wz = 0.0f;
        s_chassis_cmd.enabled = false;
        return;
    }

    // Extract joystick values with deadband
    int16_t vx_raw = apply_deadband((int16_t)(rc->rc.ch[3]), JOYSTICK_DEADBAND);
    int16_t vy_raw = apply_deadband((int16_t)(rc->rc.ch[2]), JOYSTICK_DEADBAND);
    int16_t wz_raw = apply_deadband((int16_t)(rc->rc.ch[4]), JOYSTICK_DEADBAND);

    // Convert to normalized values (-1.0 to 1.0)
    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    s_chassis_cmd.vx = (float)vx_raw / max_input;
    s_chassis_cmd.vy = (float)vy_raw / max_input;
    s_chassis_cmd.wz = (float)wz_raw / max_input;
    
    // Enable chassis if any joystick is moved
    s_chassis_cmd.enabled = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);
}

// Process shooter control commands
static void process_shooter_command(const RC_ctrl_t *rc) {
    if (rc == NULL) {
        // RC disconnected, disable shooter
        s_shoot_cmd.friction_enabled = false;
        s_shoot_cmd.feed_enabled = false;
        return;
    }

    // Right switch controls shooter
    // Up: friction + feed enabled
    // Mid: friction enabled only
    // Down: all disabled
    bool right_switch_up = switch_is_up(rc->rc.s[0]);
    bool right_switch_mid = switch_is_mid(rc->rc.s[0]);
    
    s_shoot_cmd.friction_enabled = (right_switch_up || right_switch_mid);
    s_shoot_cmd.feed_enabled = right_switch_up;
}

// Process gimbal control commands
static void process_gimbal_command(const RC_ctrl_t *rc) {
    if (rc == NULL) {
        // RC disconnected, disable gimbal
        s_gimbal_cmd.enabled = false;
        s_gimbal_cmd.pitch_rate = 0.0f;
        s_gimbal_cmd.yaw_rate = 0.0f;
        return;
    }
    
    // Gimbal always enabled
    s_gimbal_cmd.enabled = true;
    
    // Right stick controls gimbal (ch0=yaw, ch1=pitch)
    // Apply deadband and normalize to -1.0 to 1.0
    int16_t yaw_raw_fluctuated = apply_deadband((int16_t)(-rc->rc.ch[0]), JOYSTICK_DEADBAND);
    int16_t pitch_raw = apply_deadband((int16_t)(rc->rc.ch[1]), JOYSTICK_DEADBAND);

    if (yaw_raw_fluctuated!=0){
        //Work as a filter to remove zero
        yaw_raw = yaw_raw_fluctuated;
    }
    


// 简单的线性拟合：y = a*x + b （保持不变）
void linear_fit(float *x, float *y, int n, float *a, float *b)
{
    float sumx = 0, sumy = 0, sumxy = 0, sumxx = 0;
    for (int i = 0; i < n; i++) {
        sumx  += x[i];
        sumy  += y[i];
        sumxy += x[i] * y[i];
        sumxx += x[i] * x[i];
    }
    float denom = n * sumxx - sumx * sumx;
    if (denom != 0) {
        *a = (n * sumxy - sumx * sumy) / denom;
        *b = (sumy - (*a) * sumx) / n;
    }
}


float linear_method_calculator( )
{   float k=0.0f;
    float c=0.0f;
    float predicted_value=0.0f;
    

    while (1)
    {   
        new_y = yaw_raw;


            // x 由固定时间序号 tick 决定
        float new_x = tick * REFRESH_DT;  // Δt = 1/200 s
        tick++;  // 时间序号 +1

            // 写入循环缓冲区（不变）
        x_buf[index_linear] = new_x;
        y_buf[index_linear] = new_y;

        index_linear = (index_linear + 1) % SAMPLE_COUNT;
        if (filled < SAMPLE_COUNT) filled++;

            // 拟合 50 点（不变）
        if (filled == SAMPLE_COUNT) {
            float a, b;
            linear_fit(x_buf, y_buf, SAMPLE_COUNT, &a, &b);
            //printf("Fit: y = %.3f*x + %.3f\n", a, b);
            k=a;
            c=b;

          
        }
        predicted_value= k*50+c;
    }
    return predicted_value;
}
    yaw_corrected=linear_method_calculator( );
    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    s_gimbal_cmd.yaw_rate = (float)yaw_corrected / max_input;
    s_gimbal_cmd.pitch_rate = (float)pitch_raw / max_input;
    
   
    if (s_last_vision.updated && s_last_vision.target_state != NO_TARGET) {
    const float vision_gain = 5.0f;
        
    float yaw_addition = s_last_vision.yaw * vision_gain;
    float pitch_addition = s_last_vision.pitch * vision_gain;
        
    s_gimbal_cmd.yaw_rate += yaw_addition;
    s_gimbal_cmd.pitch_rate += pitch_addition;
        
    if (s_gimbal_cmd.yaw_rate > 1.0f) s_gimbal_cmd.yaw_rate = 1.0f;
    if (s_gimbal_cmd.yaw_rate < -1.0f) s_gimbal_cmd.yaw_rate = -1.0f;
    if (s_gimbal_cmd.pitch_rate > 1.0f) s_gimbal_cmd.pitch_rate = 1.0f;
    if (s_gimbal_cmd.pitch_rate < -1.0f) s_gimbal_cmd.pitch_rate = -1.0f;
    
     }
    
    //plot
    uint32_t timestamp = HAL_GetTick();
    USB_CDC_Printf("YAW_CSV,%lu,%.2f,%.2f,%.2f,%.2f\r\n",
                   timestamp,
                   yaw_raw,
                   s_last_vision.yaw,
                   s_gimbal_cmd.yaw_rate
                 );
}

void CmdController_Init(void) {
    if (s_initialized) {
        return;
    }

    memset(&s_last_rc, 0, sizeof(s_last_rc));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));
    memset(&s_last_vision, 0, sizeof(s_last_vision));
    memset(&s_chassis_cmd, 0, sizeof(s_chassis_cmd));
    memset(&s_shoot_cmd, 0, sizeof(s_shoot_cmd));
    memset(&s_gimbal_cmd, 0, sizeof(s_gimbal_cmd));

    (void)MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_VISION_DATA, on_vision_update, NULL);

    s_initialized = true;
}

void CmdController_Task(uint32_t current_tick) {
    (void)current_tick;

    if (!s_initialized) {
        return;
    }

    // Process control input
    process_chassis_command(&s_last_rc);
    process_shooter_command(&s_last_rc);
    process_gimbal_command(&s_last_rc);

    // Publish commands to message center
    (void)MsgCenter_Publish(TOPIC_CHASSIS_CMD, &s_chassis_cmd, sizeof(s_chassis_cmd));
    (void)MsgCenter_Publish(TOPIC_SHOOT_CMD, &s_shoot_cmd, sizeof(s_shoot_cmd));
    (void)MsgCenter_Publish(TOPIC_GIMBAL_CMD, &s_gimbal_cmd, sizeof(s_gimbal_cmd));

    
}

