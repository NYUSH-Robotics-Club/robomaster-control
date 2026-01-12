#include "cmd_controller.h"
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "vision_comm.h"
#include "radar_comm.h"
#include "chassis_controller.h"
#include "motor_driver.h"
#include "printing.h"
#include "logger.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define SAMPLE_COUNT 10
#define REFRESH_HZ 200
#define REFRESH_DT (1.0 / REFRESH_HZ)
#define VISION_CMD_TIMEOUT_MS 80u

// Radar smoothing parameters
#define RADAR_SMOOTH_ALPHA 0.20f    // one-pole low-pass alpha (0..1)
#define RADAR_MAX_DELTA_V 0.05f     // max m/s change per cycle for vx,vy
#define RADAR_MAX_DELTA_W 0.10f     // max rad/s change per cycle for wz

// ==========================
// Small gyro (spinning) mode
// ==========================
// Trigger: left switch in MID position.
// Behavior:
//  - Chassis: constant spin + allow translation (field-oriented control).
//  - Gimbal yaw: hold gimbal IMU absolute yaw steady (controlled by gimbal module's internal PID).
//
// NOTE: Tune these parameters on robot if needed.
#define SPIN_WZ_NORM (0.33f)                   // chassis spin rate command (normalized, 0-1)
#define SPIN_TRANSLATE_LIMIT_NORM (1.00f)      // max translation velocity in spin mode (normalized)
#define SPIN_GIMBAL_YAW_ADJ_DEG_PER_S (120.0f) // manual yaw adjustment rate when in spin mode (deg/s)

static bool s_spin_mode = false;
static float s_spin_hold_yaw_deg = 0.0f; // target absolute yaw (deg, gimbal IMU yaw_total_angle)

// ==========================
// Gimbal-oriented follow mode
// ==========================
// Trigger: left switch in UP position.
// Behavior:
//  - Chassis: movement direction follows gimbal orientation (field-oriented control).
//  - Chassis does NOT auto-spin (wz controlled manually by joystick).
//  - Gimbal: normal manual control.
static bool s_gimbal_follow_mode = false;

static float yaw_storage = 0.0f;

// Local state storage
static RC_ctrl_t s_last_rc;
static SensorData s_last_sensor;
static Vision_Recv_s s_last_vision;
static Radar_Recv_s s_last_radar;
static bool s_initialized = false;

// Filtered radar outputs (internal state)
static float s_filtered_vx = 0.0f;
static float s_filtered_vy = 0.0f;
static float s_filtered_wz = 0.0f;

// Control mode flags
#define CONTROL_MODE_RC 0      // Remote control (RC)
#define CONTROL_MODE_RADAR 1   // Radar/NUC autonomous
static uint8_t s_control_mode = CONTROL_MODE_RC;  // Default to RC

// Command messages to publish
static ChassisCmd s_chassis_cmd;
static ShootCmd s_shoot_cmd;
static GimbalCmd s_gimbal_cmd;

// Deadband for joystick input
#define JOYSTICK_DEADBAND 10

// Normalize angle to [-180, 180] range
static float normalize_angle_180(float angle_deg)
{
    while (angle_deg > 180.0f)
        angle_deg -= 360.0f;
    while (angle_deg < -180.0f)
        angle_deg += 360.0f;
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

/**
 * @brief Map radar velocity (vx, vy, wz) to swerve/mecanum wheel speeds
 * 
 * NOTE: This function is sentry_swerve specific.
 * For infantry mecanum, use different mapping (TODO if needed).
 */
#ifdef ROBOT_TYPE_sentry_swerve
static void radar_cmd_to_wheel_speeds(float vx, float vy, float wz, uint32_t now)
{
    // Timeout protection: if radar data is stale (>500ms), fall back to RC or stop
    if (!s_last_radar.valid || (now - s_last_radar.ts_ms > 500u)) {
        // Data invalid, switch back to RC or stop
        s_control_mode = CONTROL_MODE_RC;
        return;
    }
    
    // Radar data valid, use it
    s_control_mode = CONTROL_MODE_RADAR;
    
    // Swerve wheel mapping (simplified)
    // Wheel positions relative to chassis center:
    //  Front-Left: (-L, -W),  Front-Right: (-L, W)
    //  Rear-Left:  (L, -W),   Rear-Right:  (L, W)
    // For 2-module swerve (front & rear): use average positions
    
    const float L = 0.15f;  // Front-to-rear half-length
    const float W = 0.15f;  // Left-to-right half-width
    const float r = 0.05f;  // Wheel radius
    
    // Module 0 (front): position (-L, 0) -> average of front wheels
    float v0_x = vx - wz * 0.0f;  // wz * (-y_pos) = wz * 0
    float v0_y = vy + wz * (-L);
    float theta0 = atan2f(v0_y, v0_x);
    float speed0 = sqrtf(v0_x * v0_x + v0_y * v0_y) / r;
    
    // Module 1 (rear): position (L, 0) -> average of rear wheels
    float v1_x = vx - wz * 0.0f;
    float v1_y = vy + wz * L;
    float theta1 = atan2f(v1_y, v1_x);
    float speed1 = sqrtf(v1_x * v1_x + v1_y * v1_y) / r;
    
    // Dispatch: set steer target angles and drive target speeds
    // Convert theta (rad) -> encoder ticks as used by GM6020 (8192 ticks/rev)
    extern ChassisController* ChassisApp_GetController(void);
    extern void ChassisController_SetSteerTargetAngles(ChassisController *controller, const float angles[CHASSIS_STEER_COUNT]);

    ChassisController *ctrl = ChassisApp_GetController();
    if (ctrl) {
        // Compute canonical tick value from radians
        float ticks_per_rev = 8192.0f;
        float rad_to_ticks = ticks_per_rev / (2.0f * (float)M_PI);

        float tick0 = theta0 * rad_to_ticks;
        float tick1 = theta1 * rad_to_ticks;

        // Normalize to [-4096,4096) then canonicalize to +/-90deg logic similar to sentry
        float canonical0 = tick0;
        if (canonical0 > 2048.0f) canonical0 -= 4096.0f;
        if (canonical0 < -2048.0f) canonical0 += 4096.0f;

        float canonical1 = tick1;
        if (canonical1 > 2048.0f) canonical1 -= 4096.0f;
        if (canonical1 < -2048.0f) canonical1 += 4096.0f;

        // Find steer motors and drive motors
        uint8_t steer_ids[CHASSIS_STEER_COUNT] = {0};
        uint8_t drive_ids[CHASSIS_MOTOR_COUNT] = {0};
        uint8_t steer_count = MotorDriver_FindByRole(MOTOR_ROLE_CHASSIS_STEER, steer_ids, CHASSIS_STEER_COUNT);
        uint8_t drive_count = MotorDriver_FindByRole(MOTOR_ROLE_CHASSIS_DRIVE, drive_ids, CHASSIS_MOTOR_COUNT);

        // Determine final angle ticks using initial offset and shortest-path heuristics
        float final_ticks = canonical0; // default
        if (steer_count > 0) {
            MotorContext_t *ctx0 = MotorDriver_GetContext(steer_ids[0]);
            if (ctx0 && ctx0->config && ctx0->angle_initialized) {
                float initial0 = (float)ctx0->config->limits.gm6020.initial_angle;
                float last_target0 = ctrl->steer_target_angles[0];

                float option1 = initial0 + canonical0;
                while (option1 >= ticks_per_rev) option1 -= ticks_per_rev;
                while (option1 < 0.0f) option1 += ticks_per_rev;

                float option2 = initial0 + canonical0 + 4096.0f;
                while (option2 >= ticks_per_rev) option2 -= ticks_per_rev;
                while (option2 < 0.0f) option2 += ticks_per_rev;

                float diff1 = option1 - last_target0;
                if (diff1 > 4096.0f) diff1 -= 8192.0f;
                if (diff1 < -4096.0f) diff1 += 8192.0f;

                float diff2 = option2 - last_target0;
                if (diff2 > 4096.0f) diff2 -= 8192.0f;
                if (diff2 < -4096.0f) diff2 += 8192.0f;

                if (fabsf(diff1) <= fabsf(diff2)) {
                    final_ticks = canonical0;
                } else {
                    final_ticks = canonical0 + 4096.0f;
                    while (final_ticks >= 4096.0f) final_ticks -= 8192.0f;
                    while (final_ticks < -4096.0f) final_ticks += 8192.0f;
                }
            }
        }

        // Prepare steer angles array (ticks + initial offsets)
        float steer_angles[CHASSIS_STEER_COUNT] = {0};
        for (uint8_t i = 0; i < CHASSIS_STEER_COUNT; i++) {
            MotorContext_t *ctx = MotorDriver_GetContext(steer_ids[i]);
            if (ctx && ctx->config) {
                float initial = (float)ctx->config->limits.gm6020.initial_angle;
                float target_angle = initial + final_ticks;
                while (target_angle >= ticks_per_rev) target_angle -= ticks_per_rev;
                while (target_angle < 0.0f) target_angle += ticks_per_rev;
                steer_angles[i] = target_angle;
            }
        }

        // Drive speeds: assign module speeds to front (0,1) and rear (2,3)
        float drive_speeds[CHASSIS_MOTOR_COUNT] = {0};
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            if (i < 2) {
                // front module
                MotorContext_t *dctx = MotorDriver_GetContext(drive_ids[i]);
                int8_t dir = (dctx && dctx->config) ? dctx->config->direction : 1;
                drive_speeds[i] = dir * speed0;
            } else {
                // rear module
                MotorContext_t *dctx = MotorDriver_GetContext(drive_ids[i]);
                int8_t dir = (dctx && dctx->config) ? dctx->config->direction : 1;
                drive_speeds[i] = dir * speed1;
            }
        }

        // Commit to controller state and compute currents immediately
        ChassisController_SetSteerTargetAngles(ctrl, steer_angles);
        ChassisController_SetTargetSpeeds(ctrl, drive_speeds);
        ChassisController_ComputeCurrents(ctrl, HAL_GetTick());
    }
    // Safety checks: discard obviously-bad values
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz)) {
        s_control_mode = CONTROL_MODE_RC;
        return;
    }

    // Simple sanity limits (prevent absurd commands)
    const float MAX_REASONABLE_V = 10.0f; // m/s
    const float MAX_REASONABLE_W = 10.0f; // rad/s
    if (fabsf(vx) > MAX_REASONABLE_V || fabsf(vy) > MAX_REASONABLE_V || fabsf(wz) > MAX_REASONABLE_W) {
        s_control_mode = CONTROL_MODE_RC;
        return;
    }

    // One-pole low-pass filter then per-cycle delta cap
    float prev_vx = s_filtered_vx;
    float prev_vy = s_filtered_vy;
    float prev_wz = s_filtered_wz;

    // Low-pass
    float lp_vx = prev_vx + RADAR_SMOOTH_ALPHA * (vx - prev_vx);
    float lp_vy = prev_vy + RADAR_SMOOTH_ALPHA * (vy - prev_vy);
    float lp_wz = prev_wz + RADAR_SMOOTH_ALPHA * (wz - prev_wz);

    // Delta cap
    float dvx = lp_vx - prev_vx;
    if (dvx > RADAR_MAX_DELTA_V) dvx = RADAR_MAX_DELTA_V;
    if (dvx < -RADAR_MAX_DELTA_V) dvx = -RADAR_MAX_DELTA_V;
    s_filtered_vx = prev_vx + dvx;

    float dvy = lp_vy - prev_vy;
    if (dvy > RADAR_MAX_DELTA_V) dvy = RADAR_MAX_DELTA_V;
    
    if (dvy < -RADAR_MAX_DELTA_V) dvy = -RADAR_MAX_DELTA_V;
    s_filtered_vy = prev_vy + dvy;

    float dwz = lp_wz - prev_wz;
    if (dwz > RADAR_MAX_DELTA_W) dwz = RADAR_MAX_DELTA_W;
    if (dwz < -RADAR_MAX_DELTA_W) dwz = -RADAR_MAX_DELTA_W;
    s_filtered_wz = prev_wz + dwz;

    // Apply filtered values to chassis command (safe to publish)
    s_chassis_cmd.vx = s_filtered_vx;
    s_chassis_cmd.vy = s_filtered_vy;
    s_chassis_cmd.wz = s_filtered_wz;
    s_chassis_cmd.enabled = true;

    // Debug log: original vs filtered speeds
    LOG_CSV(LOG_TAG_CMD, "RADAR,IN:%.3f,%.3f,%.3f,OUT:%.3f,%.3f,%.3f,sp0:%.2f,sp1:%.2f",
            vx, vy, wz, s_filtered_vx, s_filtered_vy, s_filtered_wz, speed0, speed1);
}
#else
// Infantry (mecanum/2WD) radar mapping - placeholder for future implementation
static void radar_cmd_to_wheel_speeds(float vx, float vy, float wz, uint32_t now)
{
    // Timeout protection
    if (!s_last_radar.valid || (now - s_last_radar.ts_ms > 500u)) {
        s_control_mode = CONTROL_MODE_RC;
        return;
    }
    
    s_control_mode = CONTROL_MODE_RADAR;
    
    // For infantry: direct velocity pass-through (chassis_controller handles mapping)
    s_chassis_cmd.vx = vx;
    s_chassis_cmd.vy = vy;
    s_chassis_cmd.wz = wz;
    s_chassis_cmd.enabled = true;
    
    LOG_CSV(LOG_TAG_CMD, "RADAR,%.3f,%.3f,%.3f", vx, vy, wz);
}
#endif

// Callback for RC update
static void on_rc_update(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(RC_ctrl_t))
    {
        memcpy(&s_last_rc, ev->data, sizeof(RC_ctrl_t));
    }
}

// Callback for IMU update
static void on_imu_update(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(SensorData))
    {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

// Callback for vision data update
static void on_vision_update(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(Vision_Recv_s))
    {
        memcpy(&s_last_vision, ev->data, sizeof(Vision_Recv_s));
    }
}

// Callback for radar data update
static void on_radar_update(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(Radar_Recv_s))
    {
        memcpy(&s_last_radar, ev->data, sizeof(Radar_Recv_s));
    }
}

// Apply deadband to joystick input
static int16_t apply_deadband(int16_t value, int16_t deadband)
{
    if (value > -deadband && value < deadband)
    {
        return 0;
    }
    return value;
}

// Process chassis control commands
static void process_chassis_command(const RC_ctrl_t *rc, const SensorData *sensor, bool spin_mode, bool gimbal_follow_mode)
{
    if (rc == NULL)
    {
        // RC disconnected, stop chassis
        s_chassis_cmd.vx = 0.0f;
        s_chassis_cmd.vy = 0.0f;
        s_chassis_cmd.wz = 0.0f;
        s_chassis_cmd.enabled = false;
        return;
    }

    // Extract joystick values with deadband
    // ch[2]: left stick X -> vy, ch[3]: left stick Y -> vx, ch[4]: dial/wheel -> wz
    int16_t vx_raw = apply_deadband((int16_t)(rc->rc.ch[3]), JOYSTICK_DEADBAND);
    int16_t vy_raw = apply_deadband((int16_t)(rc->rc.ch[2]), JOYSTICK_DEADBAND);
    int16_t wz_raw = apply_deadband((int16_t)(rc->rc.ch[4]), JOYSTICK_DEADBAND);

    // Convert to normalized values (-1.0 to 1.0)
    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    float vx_f = -(float)vx_raw / max_input;
    float vy_f = -(float)vy_raw / max_input;
    float wz_n = (float)wz_raw / max_input;

    if ((spin_mode || gimbal_follow_mode) && sensor != NULL)
    {
        // Spin mode OR Gimbal-follow mode: joystick input is in gimbal frame.
        // Need to convert joystick input from gimbal frame to chassis frame.

        // Calculate offset angle: chassis yaw - gimbal yaw
        // Both angles need to be normalized to same range for correct subtraction
        float gimbal_yaw_norm = normalize_angle_180(sensor->yaw_total_angle);
        float chassis_yaw_norm = normalize_angle_180(sensor->c_yaw);
        float offset_angle = normalize_angle_180(chassis_yaw_norm - gimbal_yaw_norm);

        // Rotate joystick input from gimbal frame to chassis frame
        float vx_c = 0.0f, vy_c = 0.0f;
        gimbal_to_chassis_frame(vx_f, vy_f, offset_angle, &vx_c, &vy_c);

        if (spin_mode)
        {
            // Spin mode: chassis auto-rotates at constant speed
            const float omega = SPIN_WZ_NORM;

            // Limit translation velocity to prevent wheel saturation
            // Use L2 norm (magnitude) instead of L1 norm for better control
            float mag = sqrtf(vx_c * vx_c + vy_c * vy_c);
            if (mag > SPIN_TRANSLATE_LIMIT_NORM)
            {
                float scale = SPIN_TRANSLATE_LIMIT_NORM / mag;
                vx_c *= scale;
                vy_c *= scale;
            }

            // Swap vx_c and vy_c to match chassis coordinate system, negate vy for correct direction
            s_chassis_cmd.vx = vy_c;
            s_chassis_cmd.vy = -vx_c;
            s_chassis_cmd.wz = omega;
            // In spin mode we always enable chassis so it keeps rotating even with sticks centered.
            s_chassis_cmd.enabled = true;
        }
        else
        {
            // Gimbal-follow mode: chassis does NOT auto-rotate, manual wz control
            // Swap vx_c and vy_c to match chassis coordinate system, negate vy for correct direction
            s_chassis_cmd.vx = vy_c;
            s_chassis_cmd.vy = -vx_c;
            s_chassis_cmd.wz = wz_n;
            // Enable chassis if any joystick is moved
            s_chassis_cmd.enabled = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);
        }
    }
    else
    {
        // Normal (original) behavior
        s_chassis_cmd.vx = vx_f;
        s_chassis_cmd.vy = vy_f;
        s_chassis_cmd.wz = wz_n;
        // Enable chassis if any joystick is moved
        s_chassis_cmd.enabled = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);
    }
}

// Process shooter control commands
static void process_shooter_command(const RC_ctrl_t *rc)
{
    if (rc == NULL)
    {
        // RC disconnected, disable shooter
        s_shoot_cmd.friction_enabled = false;
        s_shoot_cmd.feed_enabled = false;
        return;
    }

    // Right switch (s[0]) controls shooter
    // Up: friction + feed enabled
    // Mid: friction enabled only
    // Down: all disabled
    bool right_switch_up = switch_is_up(rc->rc.s[0]);
    bool right_switch_mid = switch_is_mid(rc->rc.s[0]);

    s_shoot_cmd.friction_enabled = (right_switch_up || right_switch_mid);
    s_shoot_cmd.feed_enabled = right_switch_up;
}

// Process gimbal control commands
static void process_gimbal_command(const RC_ctrl_t *rc, const SensorData *sensor, bool spin_mode)
{
    if (rc == NULL)
    {
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

    // Gimbal controls:
    //   ch[0]: yaw via right stick X
    //   ch[1]: pitch via right stick Y
    // Apply deadband and normalize to -1.0 to 1.0
    int16_t yaw_raw = apply_deadband((int16_t)(-rc->rc.ch[0]), JOYSTICK_DEADBAND);
    int16_t pitch_raw = apply_deadband((int16_t)(rc->rc.ch[1]), JOYSTICK_DEADBAND);

    // RC signal glitch filter: reject sudden jumps >1000 units (likely signal noise/interference)
    if (abs(yaw_storage - yaw_raw) > 1000)
    {
        yaw_raw = yaw_storage;
    }

    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    float yaw_rate_manual = (float)yaw_raw / max_input;
    s_gimbal_cmd.pitch_rate = (float)pitch_raw / max_input;

    if (spin_mode && sensor != NULL)
    {
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
    }
    else
    {
        // Normal (original) behavior
        s_gimbal_cmd.yaw_rate = yaw_rate_manual;
        s_gimbal_cmd.yaw_rate_memo = 0.0f;
        s_gimbal_cmd.yaw_target_memo = 0.0f;
    }

    if (s_last_vision.updated)
    {
        s_last_vision.updated = 0;
        s_gimbal_cmd.vision_ts_ms = HAL_GetTick();
        if (s_last_vision.target_state != NO_TARGET)
        {
            s_gimbal_cmd.vision_valid = true;
            s_gimbal_cmd.vision_yaw_err_rad = s_last_vision.yaw;
            s_gimbal_cmd.vision_pitch_err_rad = s_last_vision.pitch;
        }
        else
        {
            s_gimbal_cmd.vision_valid = false;
            s_gimbal_cmd.vision_yaw_err_rad = 0.0f;
            s_gimbal_cmd.vision_pitch_err_rad = 0.0f;
        }
    }
    else
    {
        uint32_t now = HAL_GetTick();
        if (s_gimbal_cmd.vision_valid && (now - s_gimbal_cmd.vision_ts_ms > VISION_CMD_TIMEOUT_MS))
        {
            s_gimbal_cmd.vision_valid = false;
        }
    }
    yaw_storage = yaw_raw;
}

void CmdController_Init(void)
{
    if (s_initialized)
    {
        return;
    }

    memset(&s_last_rc, 0, sizeof(s_last_rc));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));
    memset(&s_last_vision, 0, sizeof(s_last_vision));
    memset(&s_last_radar, 0, sizeof(s_last_radar));
    memset(&s_chassis_cmd, 0, sizeof(s_chassis_cmd));
    memset(&s_shoot_cmd, 0, sizeof(s_shoot_cmd));
    memset(&s_gimbal_cmd, 0, sizeof(s_gimbal_cmd));

    /* Initialize radar filter state to zero (or last known) */
    s_filtered_vx = 0.0f;
    s_filtered_vy = 0.0f;
    s_filtered_wz = 0.0f;

    (void)MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_VISION_DATA, on_vision_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_RADAR_CMD, on_radar_update, NULL);

    s_initialized = true;
}

void CmdController_Task(uint32_t current_tick)
{
    (void)current_tick;

    if (!s_initialized)
    {
        return;
    }

    // ===== CONTROL MODE PRIORITY =====
    // 1. If radar data is valid and fresh, use radar autonomous mode (sentry only for now)
    // 2. Otherwise, fall back to RC manual control
    
    uint32_t now = HAL_GetTick();
    
#ifdef ROBOT_TYPE_sentry_swerve
    // Sentry: support radar autonomous mode
    if (s_last_radar.valid && (now - s_last_radar.ts_ms <= 500u)) {
        // Radar mode: autonomous motion from external controller
        radar_cmd_to_wheel_speeds(s_last_radar.vx, s_last_radar.vy, s_last_radar.wz, now);
        // Still process gimbal and shooter from RC/other sources
        process_shooter_command(&s_last_rc);
        process_gimbal_command(&s_last_rc, &s_last_sensor, false);  // No spin mode in radar autonomous
    } else {
        // RC mode (fallback)
        bool gimbal_follow_now = switch_is_mid(s_last_rc.rc.s[1]);
        bool spin_now = switch_is_up(s_last_rc.rc.s[1]);

        // Spin mode rising edge: latch current gimbal absolute yaw as hold target
        if (spin_now && !s_spin_mode)
        {
            s_spin_hold_yaw_deg = s_last_sensor.yaw_total_angle;
        }

        s_gimbal_follow_mode = gimbal_follow_now;
        s_spin_mode = spin_now;

        // Process control input (RC mode)
        process_chassis_command(&s_last_rc, &s_last_sensor, s_spin_mode, s_gimbal_follow_mode);
        process_shooter_command(&s_last_rc);
        process_gimbal_command(&s_last_rc, &s_last_sensor, s_spin_mode);
    }
#else
    // Infantry: support both RC and radar control
    // Priority: radar (if valid and fresh) > RC (fallback)
    if (s_last_radar.valid && (now - s_last_radar.ts_ms <= 500u)) {
        // Radar mode: autonomous motion from external controller (ROS2/Jetson)
        radar_cmd_to_wheel_speeds(s_last_radar.vx, s_last_radar.vy, s_last_radar.wz, now);
        // Still process gimbal and shooter from RC
        process_shooter_command(&s_last_rc);
        process_gimbal_command(&s_last_rc, &s_last_sensor, false);  // No spin mode in radar autonomous
    } else {
        // RC mode (fallback when radar not available)
        bool gimbal_follow_now = switch_is_mid(s_last_rc.rc.s[1]);
        bool spin_now = switch_is_up(s_last_rc.rc.s[1]);

        // Spin mode rising edge: latch current gimbal absolute yaw as hold target
        if (spin_now && !s_spin_mode)
        {
            s_spin_hold_yaw_deg = s_last_sensor.yaw_total_angle;
        }

        s_gimbal_follow_mode = gimbal_follow_now;
        s_spin_mode = spin_now;

        // Process control input (RC mode)
        process_chassis_command(&s_last_rc, &s_last_sensor, s_spin_mode, s_gimbal_follow_mode);
        process_shooter_command(&s_last_rc);
        process_gimbal_command(&s_last_rc, &s_last_sensor, s_spin_mode);
    }
#endif

    // Tagged debug prints for control mode (10 Hz)
    // Format: CMD,ctrl_mode(RC/RADAR),vx_cmd,vy_cmd,wz_cmd
    const char *mode_str = (s_control_mode == CONTROL_MODE_RADAR) ? "RADAR" : "RC";
    LOG_CSV(LOG_TAG_CMD, "%s,%.3f,%.3f,%.3f",
            mode_str,
            s_chassis_cmd.vx,
            s_chassis_cmd.vy,
            s_chassis_cmd.wz);

    // Publish commands to message center
    (void)MsgCenter_Publish(TOPIC_CHASSIS_CMD, &s_chassis_cmd, sizeof(s_chassis_cmd));
    (void)MsgCenter_Publish(TOPIC_SHOOT_CMD, &s_shoot_cmd, sizeof(s_shoot_cmd));
    (void)MsgCenter_Publish(TOPIC_GIMBAL_CMD, &s_gimbal_cmd, sizeof(s_gimbal_cmd));
}
