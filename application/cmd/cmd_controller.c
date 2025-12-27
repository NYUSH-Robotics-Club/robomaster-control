#include "cmd_controller.h"
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "vision_comm.h"
#include "printing.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define SAMPLE_COUNT 10
#define REFRESH_HZ   200
#define REFRESH_DT   (1.0 / REFRESH_HZ)
#define VISION_CMD_TIMEOUT_MS 80u

// ==========================
// Small gyro (spinning) mode
// ==========================
// Trigger: right switch in MID position.
// Behavior:
//  - Chassis: constant spin + allow translation (field-oriented control).
//  - Gimbal yaw: hold gimbal IMU absolute yaw steady (controlled by gimbal module's internal PID).
//
// NOTE: Tune these parameters on robot if needed.
#define SPIN_WZ_NORM                 (0.33f)   // chassis spin rate command (normalized, 0-1)
#define SPIN_TRANSLATE_LIMIT_NORM    (1.00f)   // max translation velocity in spin mode (normalized)
#define SPIN_GIMBAL_YAW_ADJ_DEG_PER_S (120.0f) // manual yaw adjustment rate when in spin mode (deg/s)

static bool  s_spin_mode = false;
static float s_spin_hold_yaw_deg = 0.0f;       // target absolute yaw (deg, gimbal IMU yaw_total_angle)





static float yaw_storage=0.0f;
static uint32_t s_last_spin_dbg_tick = 0; // rate limiter for SPINDBG prints (tagged)

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

// Normalize angle to [-180, 180] range
static float normalize_angle_180(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

// Rotate a 2D vector from gimbal frame to chassis frame
// offset_angle_deg: gimbal yaw - chassis yaw (how much gimbal is rotated relative to chassis)
// Uses counter-clockwise rotation matrix R(θ)
static void gimbal_to_chassis_frame(float vx_g, float vy_g, float offset_angle_deg, float *vx_c, float *vy_c)
{
    const float angle_rad = offset_angle_deg * (float)M_PI / 180.0f;
    const float c = cosf(angle_rad);
    const float s = sinf(angle_rad);
    // R(θ) = [cos(θ)  -sin(θ)]
    //        [sin(θ)   cos(θ)]
    *vx_c = c * vx_g - s * vy_g;
    *vy_c = s * vx_g + c * vy_g;
}

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
static void process_chassis_command(const RC_ctrl_t *rc, const SensorData *sensor, bool spin_mode) {
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
    float vx_f = -(float)vx_raw / max_input;
    float vy_f = -(float)vy_raw / max_input;
    float wz_n = (float)wz_raw / max_input;

    if (spin_mode && sensor != NULL) {
        // Spin mode: gimbal stays stable, chassis rotates, joystick input is in gimbal frame.
        // Need to convert joystick input from gimbal frame to chassis frame.

        // Calculate offset angle: gimbal yaw - chassis yaw
        // Both angles need to be normalized to same range for correct subtraction
        float gimbal_yaw_norm = normalize_angle_180(sensor->yaw_total_angle);
        float chassis_yaw_norm = normalize_angle_180(sensor->c_yaw);
        float offset_angle = normalize_angle_180(- gimbal_yaw_norm + chassis_yaw_norm);

        // Rotate joystick input from gimbal frame to chassis frame
        float vx_c = 0.0f, vy_c = 0.0f;
        gimbal_to_chassis_frame(vx_f, vy_f, offset_angle, &vx_c, &vy_c);

        // Set chassis spin rate
        const float omega = SPIN_WZ_NORM;

        // Limit translation velocity to prevent wheel saturation
        // Use L2 norm (magnitude) instead of L1 norm for better control
        float mag = sqrtf(vx_c * vx_c + vy_c * vy_c);
        if (mag > SPIN_TRANSLATE_LIMIT_NORM) {
            float scale = SPIN_TRANSLATE_LIMIT_NORM / mag;
            vx_c *= scale;
            vy_c *= scale;
        }

        s_chassis_cmd.vx = vx_c;
        s_chassis_cmd.vy = vy_c;
        s_chassis_cmd.wz = omega;
        // In spin mode we always enable chassis so it keeps rotating even with sticks centered.
        s_chassis_cmd.enabled = true;
    } else {
        // Normal (original) behavior
        s_chassis_cmd.vx = vx_f;
        s_chassis_cmd.vy = vy_f;
        s_chassis_cmd.wz = wz_n;
        // Enable chassis if any joystick is moved
        s_chassis_cmd.enabled = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);
    }
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
    bool right_switch_up = switch_is_up(rc->rc.s[1]);
    bool right_switch_mid = switch_is_mid(rc->rc.s[1]);
    
    s_shoot_cmd.friction_enabled = (right_switch_up || right_switch_mid);
    s_shoot_cmd.feed_enabled = right_switch_up;
}

// Process gimbal control commands
static void process_gimbal_command(const RC_ctrl_t *rc, const SensorData *sensor, bool spin_mode) {
    if (rc == NULL) {
        // RC disconnected, disable gimbal
        s_gimbal_cmd.enabled = false;
        s_gimbal_cmd.pitch_rate = 0.0f;
        s_gimbal_cmd.yaw_rate = 0.0f;
        s_gimbal_cmd.vision_valid = false;
        s_gimbal_cmd.vision_yaw_err_rad = 0.0f;
        s_gimbal_cmd.vision_pitch_err_rad = 0.0f;
        s_gimbal_cmd.vision_ts_ms = 0;
        return;
    }
    
    // Gimbal always enabled
    s_gimbal_cmd.enabled = true;
    
    // Right stick controls gimbal (ch0=yaw, ch1=pitch)
    // Apply deadband and normalize to -1.0 to 1.0
    int16_t yaw_raw = apply_deadband((int16_t)(-rc->rc.ch[0]), JOYSTICK_DEADBAND);
    int16_t pitch_raw = apply_deadband((int16_t)(rc->rc.ch[1]), JOYSTICK_DEADBAND);

    if (abs(yaw_storage-yaw_raw)>1000){
        yaw_raw=yaw_storage;
    }

          
    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    float yaw_rate_manual = (float)yaw_raw / max_input;
    s_gimbal_cmd.pitch_rate = (float)pitch_raw / max_input;

    if (spin_mode && sensor != NULL) {
        // Allow manual yaw adjustment by shifting hold target (deg/s).
        s_spin_hold_yaw_deg += yaw_rate_manual * SPIN_GIMBAL_YAW_ADJ_DEG_PER_S * (float)REFRESH_DT;

        // Provide absolute yaw hold target to gimbal controller.
        // We reuse existing memo fields to avoid changing message struct.
        // - yaw_rate_memo: acts as a boolean flag (1.0 = spin hold active)
        // - yaw_target_memo: hold target absolute yaw (deg, gimbal IMU yaw_total_angle frame)
        s_gimbal_cmd.yaw_rate_memo = 1.0f;
        s_gimbal_cmd.yaw_target_memo = s_spin_hold_yaw_deg;

        // Keep yaw_rate at 0 in spin mode; gimbal controller will compute angle_target directly.
        s_gimbal_cmd.yaw_rate = 0.0f;
    } else {
        // Normal (original) behavior
        s_gimbal_cmd.yaw_rate = yaw_rate_manual;
        s_gimbal_cmd.yaw_rate_memo = 0.0f;
        s_gimbal_cmd.yaw_target_memo = 0.0f;
    }
    
    if (s_last_vision.updated) {
        s_last_vision.updated = 0;
        s_gimbal_cmd.vision_ts_ms = HAL_GetTick();
        if (s_last_vision.target_state != NO_TARGET) {
            s_gimbal_cmd.vision_valid = true;
            s_gimbal_cmd.vision_yaw_err_rad = s_last_vision.yaw;
            s_gimbal_cmd.vision_pitch_err_rad = s_last_vision.pitch;
        } else {
            s_gimbal_cmd.vision_valid = false;
            s_gimbal_cmd.vision_yaw_err_rad = 0.0f;
            s_gimbal_cmd.vision_pitch_err_rad = 0.0f;
        }
    } else {
        uint32_t now = HAL_GetTick();
        if (s_gimbal_cmd.vision_valid && (now - s_gimbal_cmd.vision_ts_ms > VISION_CMD_TIMEOUT_MS)) {
            s_gimbal_cmd.vision_valid = false;
        }
    }
    yaw_storage=yaw_raw; 
    // NOTE: Tagged debug printing is done in CmdController_Task() to keep one place for mode diagnostics.
    
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

    // Small gyro mode gating: right switch in MID position.
    bool spin_now = switch_is_mid(s_last_rc.rc.s[0]);
    if (spin_now && !s_spin_mode) {
        // Rising edge: latch current gimbal absolute yaw as hold target.
        s_spin_hold_yaw_deg = s_last_sensor.yaw_total_angle;
    }
    s_spin_mode = spin_now;

    // Process control input
    process_chassis_command(&s_last_rc, &s_last_sensor, s_spin_mode);
    process_shooter_command(&s_last_rc);
    process_gimbal_command(&s_last_rc, &s_last_sensor, s_spin_mode);

    // Tagged debug prints for spin mode (10 Hz), to avoid mixing with other logs.
    // Format:
    // SPINDBG,ts_ms,spin,sw0,sw1,c_yaw_deg,g_yaw_total_deg,hold_yaw_deg,yaw_err_deg,yaw_rate_cmd,vx_cmd,vy_cmd,wz_cmd
    if (HAL_GetTick() - s_last_spin_dbg_tick >= 100) {
        s_last_spin_dbg_tick = HAL_GetTick();

        float yaw_err_deg = s_spin_hold_yaw_deg - s_last_sensor.yaw_total_angle;
        USB_CDC_Printf("SPINDBG,%lu,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\r\n",
                       (unsigned long)s_last_spin_dbg_tick,
                       (unsigned int)(s_spin_mode ? 1U : 0U),
                       (unsigned int)((uint8_t)s_last_rc.rc.s[0]),
                       (unsigned int)((uint8_t)s_last_rc.rc.s[1]),
                       s_last_sensor.c_yaw,
                       s_last_sensor.yaw_total_angle,
                       s_spin_hold_yaw_deg,
                       yaw_err_deg,
                       s_gimbal_cmd.yaw_rate,
                       s_chassis_cmd.vx,
                       s_chassis_cmd.vy,
                       s_chassis_cmd.wz);
    }

    // Publish commands to message center
    (void)MsgCenter_Publish(TOPIC_CHASSIS_CMD, &s_chassis_cmd, sizeof(s_chassis_cmd));
    (void)MsgCenter_Publish(TOPIC_SHOOT_CMD, &s_shoot_cmd, sizeof(s_shoot_cmd));
    (void)MsgCenter_Publish(TOPIC_GIMBAL_CMD, &s_gimbal_cmd, sizeof(s_gimbal_cmd));

    
}

