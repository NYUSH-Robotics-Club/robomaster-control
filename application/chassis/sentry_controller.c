#include "sentry_controller.h"

#include <string.h>
#include <math.h>

#include "printing.h"   // USB_CDC_Printf (optional)
#include "stm32f4xx_hal.h"

static void reset_integrals(SentryController *c)
{
    for (int i = 0; i < SENTRY_DRIVE_MOTOR_COUNT; i++) {
        c->speed_pid[i].integral = 0.0f;
    }
}

static int16_t compute_single_current(PID_Controller *pid,
                                      float target,
                                      Motor_Feedback *fb,
                                      uint32_t now_ms)
{
    if (now_ms - fb->last_update_time > SENTRY_FEEDBACK_TIMEOUT_MS) {
        return 0;
    }
    return (int16_t)PID_Calculate(pid, target, (float)fb->speed);
}

/**
 * X-drive mixing (4 wheels).
 *
 * Wheel order here is logical indices 0..3 in the controller (not physical placement).
 * You MUST ensure your config/registry role ordering matches your wiring expectation.
 *
 * Common X-drive mapping (one of the standard forms):
 *   w0 = +vx - vy + wz
 *   w1 = +vx + vy - wz
 *   w2 = +vx - vy - wz
 *   w3 = +vx + vy + wz
 *
 * This is intentionally identical to the mixing you used in chassis_controller.c,
 * which already behaves like an X/mecanum-style mixer for 4 drive motors.
 */
static void xdrive_mix(float vx, float vy, float wz, float w[4])
{
    w[0] = (vx - vy + wz);
    w[1] = (vx + vy - wz);
    w[2] = (vx - vy - wz);
    w[3] = (vx + vy + wz);
}

void SentryController_Init(SentryController *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));

    // discover drive motors by role
    c->motor_count = MotorDriver_FindByRole(MOTOR_ROLE_CHASSIS_DRIVE,
                                           c->motor_ids,
                                           SENTRY_DRIVE_MOTOR_COUNT);

    // init from config
    for (uint8_t i = 0; i < c->motor_count; i++) {
        MotorContext_t *ctx = MotorDriver_GetContext(c->motor_ids[i]);
        if (ctx && ctx->config) {
            c->motor_dirs[i] = ctx->config->direction;

            // Use OUTER PID (speed PID in your current chassis code)
            PID_Init(&c->speed_pid[i],
                     ctx->config->pid_outer.kp,
                     ctx->config->pid_outer.ki,
                     ctx->config->pid_outer.kd,
                     ctx->config->pid_outer.output_max,
                     ctx->config->pid_outer.integral_max);

            c->target_speed[i] = 0.0f;
        }
    }

    // Optional debug print
    USB_CDC_Printf("[SentryController] drive motors found=%d\r\n", c->motor_count);
    for (uint8_t i = 0; i < c->motor_count; i++) {
        USB_CDC_Printf("  idx=%d id=%d dir=%d\r\n", i, c->motor_ids[i], c->motor_dirs[i]);
    }
}

void SentryController_SetCmd(SentryController *c, bool enabled, float vx, float vy, float wz)
{
    if (!c) return;
    c->enabled = enabled;
    c->vx = vx;
    c->vy = vy;
    c->wz = wz;
}

void SentryController_Update(SentryController *c)
{
    if (!c) return;

    // If disabled, just zero targets (but do not spam prints)
    if (!c->enabled) {
        for (uint8_t i = 0; i < c->motor_count; i++) {
            c->target_speed[i] = 0.0f;
        }
        return;
    }

    // Scale normalized input to target speed units
    const float scale = (float)SENTRY_TARGET_SPEED / 2.0f;
    const float vx = c->vx * scale;
    const float vy = c->vy * scale;
    const float wz = c->wz * scale * (float)SENTRY_OMEGA_SCALE;

    float w[4] = {0};
    xdrive_mix(vx, vy, wz, w);

    // apply direction and assign
    for (uint8_t i = 0; i < c->motor_count && i < 4; i++) {
        c->target_speed[i] = (float)c->motor_dirs[i] * w[i];
    }
}

void SentryController_ComputeCurrents(SentryController *c, uint32_t current_tick_ms)
{
    if (!c) return;

    // Safety: if not enabled, send zero currents
    if (!c->enabled) {
        for (uint8_t i = 0; i < c->motor_count; i++) {
            c->out_current[i] = 0;
            MotorDriver_SendCurrent(c->motor_ids[i], 0);
        }
        MotorDriver_FlushAll();
        return;
    }

    // compute and send
    for (uint8_t i = 0; i < c->motor_count; i++) {
        int16_t cur = compute_single_current(&c->speed_pid[i],
                                             c->target_speed[i],
                                             &c->motor_fb[i],
                                             current_tick_ms);

        c->out_current[i] = cur;
        MotorDriver_SendCurrent(c->motor_ids[i], cur);
    }

    MotorDriver_FlushAll();

    // Optional: periodic debug print of the exact command being sent
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (now - last > 200) {
        last = now;
        USB_CDC_Printf("[SENTRY_CUR] en=%d\r\n", (int)c->enabled);
        for (uint8_t i = 0; i < c->motor_count; i++) {
            uint32_t dt = now - c->motor_fb[i].last_update_time;
            USB_CDC_Printf("  i=%d id=%d tgt=%.1f fb=%.1f dt=%lu out=%d\r\n",
                           i, c->motor_ids[i], c->target_speed[i],
                           (float)c->motor_fb[i].speed,
                           (unsigned long)dt, (int)c->out_current[i]);
        }
    }
}

void SentryController_Stop(SentryController *c)
{
    if (!c) return;
    c->enabled = false;
    reset_integrals(c);
    for (uint8_t i = 0; i < c->motor_count; i++) {
        c->target_speed[i] = 0.0f;
        c->out_current[i] = 0;
    }
}

void SentryController_UpdateMotorFeedback(
    SentryController *c,
    uint8_t motor_index,
    uint16_t angle,
    int16_t speed,
    int16_t current,
    uint8_t temp,
    uint32_t tick_ms)
{
    if (!c) return;
    if (motor_index >= c->motor_count) return;

    c->motor_fb[motor_index].angle = angle;
    c->motor_fb[motor_index].speed = speed;
    c->motor_fb[motor_index].current = current;
    c->motor_fb[motor_index].temp = temp;
    c->motor_fb[motor_index].last_update_time = tick_ms;
}
