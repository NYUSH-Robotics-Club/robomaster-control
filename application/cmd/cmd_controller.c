#include "cmd_controller.h"
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include <string.h>

// Local state storage
static RC_ctrl_t s_last_rc;
static SensorData s_last_sensor;
static bool s_initialized = false;

// Command messages to publish
static ChassisCmd s_chassis_cmd;
static ShootCmd s_shoot_cmd;

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
    int16_t wz_raw = apply_deadband((int16_t)(-rc->rc.ch[4]), JOYSTICK_DEADBAND);

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
        s_shoot_cmd.gimbal_enabled = false;
        s_shoot_cmd.gimbal_pitch_input = 0;
        s_shoot_cmd.gimbal_yaw_input = 0;
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
    
    // Right switch controls gimbal mode
    bool left_switch_up = switch_is_up(rc->rc.s[1]);
    s_shoot_cmd.gimbal_enabled = true;
    
    // Gimbal joystick inputs (right stick controls gimbal)
    s_shoot_cmd.gimbal_yaw_input = rc->rc.ch[0];
    s_shoot_cmd.gimbal_pitch_input = rc->rc.ch[1];
}

void CmdController_Init(void) {
    if (s_initialized) {
        return;
    }

    memset(&s_last_rc, 0, sizeof(s_last_rc));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));
    memset(&s_chassis_cmd, 0, sizeof(s_chassis_cmd));
    memset(&s_shoot_cmd, 0, sizeof(s_shoot_cmd));

    (void)MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

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

    // Publish commands to message center
    (void)MsgCenter_Publish(TOPIC_CHASSIS_CMD, &s_chassis_cmd, sizeof(s_chassis_cmd));
    (void)MsgCenter_Publish(TOPIC_SHOOT_CMD, &s_shoot_cmd, sizeof(s_shoot_cmd));
}

