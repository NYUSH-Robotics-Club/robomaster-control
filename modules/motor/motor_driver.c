#include "motor_driver.h"
#include "message_center.h"
#include "can_comm.h"
#include <math.h>
#include <string.h>

// Global motor contexts (one per motor)
static MotorContext_t g_motor_contexts[MOTOR_DRIVER_MAX_MOTORS];

// Subscription flags
static bool g_module_initialized = false;

// Forward declarations for callbacks
static void on_gm6020_feedback(const MsgEvent *ev, void *user);
static void on_motor_feedback(const MsgEvent *ev, void *user);

/**
 * @brief Initialize motor driver module
 */
void MotorDriver_ModuleInit(void)
{
    if (g_module_initialized) {
        return;  // Already initialized
    }

    // Clear all motor contexts
    memset(g_motor_contexts, 0, sizeof(g_motor_contexts));

    // Subscribe to CAN feedback topics
    (void)MsgCenter_Subscribe(TOPIC_GM6020_FEEDBACK, on_gm6020_feedback, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);

    g_module_initialized = true;
}

/**
 * @brief Initialize a motor from configuration
 */
bool MotorDriver_Init(uint8_t motor_id, const MotorConfig_t *config)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS || config == NULL) {
        return false;
    }

    // Ensure module is initialized
    if (!g_module_initialized) {
        MotorDriver_ModuleInit();
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];

    // Clear context
    memset(ctx, 0, sizeof(MotorContext_t));

    // Store motor identification
    ctx->motor_id = motor_id;
    ctx->type = config->type;
    ctx->role = config->role;
    ctx->config = config;

    // Initialize target angle from configuration
    if (config->type == MOTOR_TYPE_GM6020) {
        ctx->angle_target = config->limits.gm6020.initial_angle;
        // If initial angle < 0, mark as not initialized (will auto-init on first feedback)
        ctx->angle_initialized = (config->limits.gm6020.initial_angle >= 0.0f);
    } else {
        ctx->angle_target = 0.0f;
        ctx->angle_initialized = false;
    }

    // Initialize PID controllers from configuration
    PID_Init(&ctx->pid_outer,
             config->pid_outer.kp,
             config->pid_outer.ki,
             config->pid_outer.kd,
             config->pid_outer.output_max,
             config->pid_outer.integral_max);

    // Inner loop PID (if configured)
    if (config->pid_inner.kp != 0.0f || config->pid_inner.output_max != 0.0f) {
        PID_Init(&ctx->pid_inner,
                 config->pid_inner.kp,
                 config->pid_inner.ki,
                 config->pid_inner.kd,
                 config->pid_inner.output_max,
                 config->pid_inner.integral_max);
    }

    ctx->initialized = true;
    return true;
}

/**
 * @brief Update motor feedback
 */
void MotorDriver_UpdateFeedback(uint8_t motor_id,
                                uint16_t angle_raw,
                                int16_t speed_rpm,
                                int16_t current,
                                uint32_t timestamp)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (!ctx->initialized) {
        return;  // Motor not initialized yet
    }

    // Update feedback data
    ctx->angle_raw = angle_raw;
    ctx->speed_rpm = speed_rpm;
    ctx->feedback_current = current;
    ctx->last_feedback_time = timestamp;

    // Compute angle in radians (for GM6020 gimbal control)
    if (ctx->type == MOTOR_TYPE_GM6020) {
        float max_encoder = (ctx->config->limits.gm6020.angle_max > 0.0f) ?
                           ctx->config->limits.gm6020.angle_max : 8192.0f;
        ctx->target_angle_rad = (float)angle_raw / max_encoder * 2.0f * M_PI;
    }

    // Auto-initialize angle target on first feedback
    if (!ctx->angle_initialized) {
        ctx->angle_target = (float)angle_raw;
        ctx->angle_initialized = true;
    }
}

/**
 * @brief Compute motor current command
 */
int16_t MotorDriver_ComputeCurrent(uint8_t motor_id,
                                   float target,
                                   bool use_angle_control)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return 0;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (!ctx->initialized || !ctx->angle_initialized) {
        return 0;  // Not ready
    }

    int16_t output_current = 0;

    if (use_angle_control) {
        // Outer loop: Angle control
        ctx->angle_target = target;

        // Apply angle limits (for GM6020 gimbal motors)
        if (ctx->type == MOTOR_TYPE_GM6020) {
            if (ctx->angle_target < ctx->config->limits.gm6020.angle_min) {
                ctx->angle_target = ctx->config->limits.gm6020.angle_min;
            }
            if (ctx->angle_target > ctx->config->limits.gm6020.angle_max) {
                ctx->angle_target = ctx->config->limits.gm6020.angle_max;
            }
        }

        // PID: angle error -> speed target
        float angle_error = ctx->angle_target - (float)ctx->angle_raw;
        float speed_target = PID_Calculate(&ctx->pid_outer, ctx->angle_target, (float)ctx->angle_raw);

        // Inner loop: Speed control (if configured)
        if (ctx->config->pid_inner.kp != 0.0f) {
            output_current = (int16_t)PID_Calculate(&ctx->pid_inner, speed_target, (float)ctx->speed_rpm);
        } else {
            // Direct output from angle PID
            output_current = (int16_t)speed_target;
        }

        // Add gravity compensation (for pitch motors)
        if (ctx->type == MOTOR_TYPE_GM6020 && ctx->role == MOTOR_ROLE_GIMBAL_PITCH) {
            float gravity_comp = ctx->config->limits.gm6020.gravity_compensation;
            float direction = (float)ctx->config->direction;
            output_current += (int16_t)(gravity_comp * direction);
        }

    } else {
        // Direct speed control
        output_current = (int16_t)PID_Calculate(&ctx->pid_outer, target, (float)ctx->speed_rpm);
    }

    // Apply direction correction
    output_current *= ctx->config->direction;

    // Clamp current based on motor type
    int16_t max_current;
    if (ctx->type == MOTOR_TYPE_GM6020) {
        max_current = 25000;
    } else {  // M3508/M2006
        max_current = 16384;
    }

    if (output_current > max_current) output_current = max_current;
    if (output_current < -max_current) output_current = -max_current;

    return output_current;
}

/**
 * @brief Get motor context
 */
MotorContext_t* MotorDriver_GetContext(uint8_t motor_id)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return NULL;
    }
    return &g_motor_contexts[motor_id];
}

/**
 * @brief Check if motor is initialized
 */
bool MotorDriver_IsInitialized(uint8_t motor_id)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return false;
    }
    return g_motor_contexts[motor_id].initialized &&
           g_motor_contexts[motor_id].angle_initialized;
}

/**
 * @brief Set motor angle target
 */
void MotorDriver_SetAngleTarget(uint8_t motor_id, float target_angle)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (ctx->initialized) {
        ctx->angle_target = target_angle;
    }
}

/**
 * @brief Reset motor PID integrals
 */
void MotorDriver_ResetPID(uint8_t motor_id)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (ctx->initialized) {
        PID_Reset(&ctx->pid_outer);
        PID_Reset(&ctx->pid_inner);
    }
}

// ========== Message Center Callbacks ==========

/**
 * @brief GM6020 feedback callback
 */
static void on_gm6020_feedback(const MsgEvent *ev, void *user)
{
    (void)user;

    if (ev->size == sizeof(GM6020FeedbackEvent)) {
        const GM6020FeedbackEvent *m = (const GM6020FeedbackEvent *)ev->data;
        MotorDriver_UpdateFeedback(m->id, m->angle, m->speed, m->current, m->tick_ms);
    }
}

/**
 * @brief M3508/M2006 feedback callback
 */
static void on_motor_feedback(const MsgEvent *ev, void *user)
{
    (void)user;

    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
        MotorDriver_UpdateFeedback(m->id, m->angle, m->speed, m->current, m->tick_ms);
    }
}
