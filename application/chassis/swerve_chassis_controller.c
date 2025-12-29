#include "swerve_chassis_controller.h"
#include "motor_driver.h"
#include <string.h>
#include <math.h>
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "can_comm.h"
#include "printing.h"
#include "cmd_controller.h"

#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

// Chassis geometry (meters, adjust based on your robot)
#define WHEELBASE_LENGTH (0.4f)   // Distance between front and back wheels
#define WHEELBASE_WIDTH  (0.4f)   // Distance between left and right wheels

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static SensorData s_last_sensor;
static SwerveChassisController s_ctrl;

// Swerve module motor IDs (dynamically assigned during init)
static uint8_t s_module_steering_ids[SWERVE_MODULE_COUNT];
static uint8_t s_module_drive_ids[SWERVE_MODULE_COUNT];

/**
 * @brief Normalize angle to [0, 8192) range
 */
static float NormalizeAngle(float angle, float max_encoder)
{
    while (angle < 0.0f) angle += max_encoder;
    while (angle >= max_encoder) angle -= max_encoder;
    return angle;
}

/**
 * @brief Calculate shortest angular distance between two angles
 * @return Shortest distance in encoder units (can be negative)
 */
static float ShortestAngularDistance(float target, float current, float max_encoder)
{
    float diff = target - current;
    float half_range = max_encoder / 2.0f;

    if (diff > half_range) {
        diff -= max_encoder;
    } else if (diff < -half_range) {
        diff += max_encoder;
    }

    return diff;
}

/**
 * @brief Compute PID with timeout check
 */
static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) {
        return 0;
    }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

/**
 * @brief Swerve drive inverse kinematics for diagonal layout
 *
 * For a diagonal swerve drive with modules at FL and BR:
 * - Module 0 (FL): Front-left position
 * - Module 1 (BR): Back-right position
 *
 * This function calculates steering angle and drive speed for each module
 * based on desired chassis velocity (vx, vy, wz).
 */
static void SwerveKinematics(float vx, float vy, float wz,
                              float *fl_angle, float *fl_speed,
                              float *br_angle, float *br_speed)
{
    // Position vectors for each module relative to robot center
    // Front-left: (+width/2, +length/2)
    float fl_x = WHEELBASE_WIDTH / 2.0f;
    float fl_y = WHEELBASE_LENGTH / 2.0f;

    // Back-right: (-width/2, -length/2)
    float br_x = -WHEELBASE_WIDTH / 2.0f;
    float br_y = -WHEELBASE_LENGTH / 2.0f;

    // Calculate wheel velocities using: v_wheel = v_chassis + w × r
    // Front-left
    float fl_vx = vx - wz * fl_y;
    float fl_vy = vy + wz * fl_x;

    // Back-right
    float br_vx = vx - wz * br_y;
    float br_vy = vy + wz * br_x;

    // Calculate steering angles (in radians)
    float fl_angle_rad = atan2f(fl_vy, fl_vx);
    float br_angle_rad = atan2f(br_vy, br_vx);

    // Convert to encoder units (0-8192 for GM6020)
    const float encoder_max = 8192.0f;
    *fl_angle = (fl_angle_rad / (2.0f * M_PI)) * encoder_max;
    *br_angle = (br_angle_rad / (2.0f * M_PI)) * encoder_max;

    // Normalize angles to [0, 8192)
    *fl_angle = NormalizeAngle(*fl_angle, encoder_max);
    *br_angle = NormalizeAngle(*br_angle, encoder_max);

    // Calculate drive speeds (magnitude of velocity vector)
    *fl_speed = sqrtf(fl_vx * fl_vx + fl_vy * fl_vy);
    *br_speed = sqrtf(br_vx * br_vx + br_vy * br_vy);

    // Scale speeds to RPM range
    const float speed_scale = SWERVE_DEMO_TARGET_SPEED;
    *fl_speed *= speed_scale;
    *br_speed *= speed_scale;
}

void SwerveChassisController_Init(SwerveChassisController *controller)
{
    if (controller == NULL) return;
    memset(controller, 0, sizeof(SwerveChassisController));

    // Find drive motors (M3508)
    uint8_t drive_motor_ids[4];
    uint8_t drive_count = MotorDriver_FindByRole(MOTOR_ROLE_CHASSIS_DRIVE, drive_motor_ids, 4);

    // Find steering motors (GM6020)
    uint8_t steer_motor_ids[4];
    uint8_t steer_count = MotorDriver_FindByRole(MOTOR_ROLE_CHASSIS_STEER, steer_motor_ids, 4);

    USB_CDC_Printf("[SwerveChassisController] Found %d drive, %d steer motors\r\n", drive_count, steer_count);

    // Assign motors to modules (assuming first 2 of each type)
    for (uint8_t i = 0; i < SWERVE_MODULE_COUNT && i < drive_count && i < steer_count; i++) {
        s_module_drive_ids[i] = drive_motor_ids[i];
        s_module_steering_ids[i] = steer_motor_ids[i];

        controller->modules[i].drive_motor_id = drive_motor_ids[i];
        controller->modules[i].steering_motor_id = steer_motor_ids[i];

        // Get motor contexts to initialize PIDs
        MotorContext_t *drive_ctx = MotorDriver_GetContext(drive_motor_ids[i]);
        MotorContext_t *steer_ctx = MotorDriver_GetContext(steer_motor_ids[i]);

        if (drive_ctx && drive_ctx->config) {
            PID_Init(&controller->modules[i].drive_speed_pid,
                     drive_ctx->config->pid_outer.kp,
                     drive_ctx->config->pid_outer.ki,
                     drive_ctx->config->pid_outer.kd,
                     drive_ctx->config->pid_outer.output_max,
                     drive_ctx->config->pid_outer.integral_max);
        }

        if (steer_ctx && steer_ctx->config) {
            PID_Init(&controller->modules[i].steering_angle_pid,
                     steer_ctx->config->pid_outer.kp,
                     steer_ctx->config->pid_outer.ki,
                     steer_ctx->config->pid_outer.kd,
                     steer_ctx->config->pid_outer.output_max,
                     steer_ctx->config->pid_outer.integral_max);
        }

        USB_CDC_Printf("[SwerveModule %d] Drive=%d, Steer=%d\r\n", i, drive_motor_ids[i], steer_motor_ids[i]);
    }
}

void SwerveChassisController_Update(SwerveChassisController *controller, SensorData* sensor_data)
{
    if (controller == NULL) return;

    controller->vx_target = s_last_cmd.vx;
    controller->vy_target = s_last_cmd.vy;
    controller->wz_target = s_last_cmd.wz;
    controller->running = s_last_cmd.enabled;

    // Compute swerve kinematics
    float fl_angle, fl_speed, br_angle, br_speed;
    SwerveKinematics(controller->vx_target,
                     controller->vy_target,
                     controller->wz_target,
                     &fl_angle, &fl_speed,
                     &br_angle, &br_speed);

    // Update module targets
    if (SWERVE_MODULE_COUNT >= 1) {
        controller->modules[0].steering_angle_target = fl_angle;
        controller->modules[0].drive_speed_target = fl_speed;
    }

    if (SWERVE_MODULE_COUNT >= 2) {
        controller->modules[1].steering_angle_target = br_angle;
        controller->modules[1].drive_speed_target = br_speed;
    }
}

void SwerveChassisController_ComputeCurrents(SwerveChassisController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;

    for (uint8_t i = 0; i < SWERVE_MODULE_COUNT; i++) {
        SwerveModule_t *module = &controller->modules[i];

        // Compute steering angle current using GM6020 angle control
        MotorContext_t *steer_ctx = MotorDriver_GetContext(module->steering_motor_id);
        if (steer_ctx && steer_ctx->config) {
            float current_angle = (float)module->steering_feedback.angle;
            float target_angle = module->steering_angle_target;
            float max_encoder = steer_ctx->config->limits.gm6020.angle_max;

            // Calculate shortest path to target
            float angle_error = ShortestAngularDistance(target_angle, current_angle, max_encoder);
            float speed_target = PID_Calculate(&module->steering_angle_pid, 0.0f, -angle_error);

            // Send steering current
            module->steering_current = (int16_t)speed_target;
            MotorDriver_SendCurrent(module->steering_motor_id, module->steering_current);
        }

        // Compute drive speed current
        module->drive_current = ComputeSingleMotorCurrent(
            &module->drive_speed_pid,
            module->drive_speed_target,
            &module->drive_feedback,
            current_tick
        );

        // Send drive current
        MotorDriver_SendCurrent(module->drive_motor_id, module->drive_current);
    }

    // Flush all CAN commands
    MotorDriver_FlushAll();
}

void SwerveChassisController_Stop(SwerveChassisController *controller)
{
    if (controller == NULL) return;

    controller->running = false;
    controller->vx_target = 0.0f;
    controller->vy_target = 0.0f;
    controller->wz_target = 0.0f;

    for (uint8_t i = 0; i < SWERVE_MODULE_COUNT; i++) {
        controller->modules[i].drive_speed_target = 0.0f;
        PID_Reset(&controller->modules[i].drive_speed_pid);
        PID_Reset(&controller->modules[i].steering_angle_pid);
    }
}

bool SwerveChassisController_IsRunning(const SwerveChassisController *controller)
{
    if (controller == NULL) return false;
    return controller->running;
}

void SwerveChassisController_UpdateMotorFeedback(SwerveChassisController *controller,
                                                  uint8_t motor_id,
                                                  uint16_t angle,
                                                  int16_t speed,
                                                  int16_t current,
                                                  uint8_t temp,
                                                  uint32_t current_tick)
{
    if (controller == NULL) return;

    // Find which module this motor belongs to
    for (uint8_t i = 0; i < SWERVE_MODULE_COUNT; i++) {
        Motor_Feedback *feedback = NULL;

        if (motor_id == s_module_drive_ids[i]) {
            feedback = &controller->modules[i].drive_feedback;
        } else if (motor_id == s_module_steering_ids[i]) {
            feedback = &controller->modules[i].steering_feedback;
        }

        if (feedback != NULL) {
            feedback->angle = angle;
            feedback->speed = speed;
            feedback->current = current;
            feedback->temp = temp;
            feedback->last_update_time = current_tick;
            return;
        }
    }
}

// ========== App Wrapper ==========

static void on_chassis_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(ChassisCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
        SwerveChassisController_Update(&s_ctrl, &s_last_sensor);
        SwerveChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
    }
}

static void on_imu_update(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

static void on_motor_feedback(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;

        // Check if this motor is a swerve chassis motor
        bool is_swerve_motor = false;
        for (uint8_t i = 0; i < SWERVE_MODULE_COUNT; i++) {
            if (m->id == s_module_drive_ids[i] || m->id == s_module_steering_ids[i]) {
                is_swerve_motor = true;
                break;
            }
        }

        if (is_swerve_motor) {
            SwerveChassisController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}

static void on_gm6020_feedback(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(GM6020FeedbackEvent)) {
        const GM6020FeedbackEvent *m = (const GM6020FeedbackEvent *)ev->data;

        // Check if this motor is a steering motor
        for (uint8_t i = 0; i < SWERVE_MODULE_COUNT; i++) {
            if (m->id == s_module_steering_ids[i]) {
                SwerveChassisController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, 0, m->tick_ms);
                break;
            }
        }
    }
}

void SwerveChassisApp_Init(void) {
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));

    SwerveChassisController_Init(&s_ctrl);

    (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
    (void)MsgCenter_Subscribe(TOPIC_GM6020_FEEDBACK, on_gm6020_feedback, NULL);
}

SwerveChassisController* SwerveChassisApp_GetController(void) {
    return &s_ctrl;
}
