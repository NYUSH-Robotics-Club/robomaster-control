#include "gimbal_controller.h"
#include "message_center.h"
#include "motor_driver.h"
#include "pid.h"
#include "printing.h"
#include "logger.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

// Motor IDs (dynamically assigned during init)
static uint8_t s_pitch_motor_id = 0xFF;
static uint8_t s_yaw_motor_id = 0xFF;

// Yaw control parameters
#define YAW_CONTROL_ENC_MAX (8192.0f)
#define YAW_CONTROL_GYRO_LPF_ALPHA (0.3f)
#define CURRENT_LIMIT (30000.0f)

// Legacy defines (not used anymore - values from config)
#define YAW_RPM_MAX (220.0f) * 2.0f
#define YAW_RPM_MIN 0.0f
#define YAW_ERROR_FOR_FULL_SPEED (1200.0f)
// Static state for application
static GimbalCmd s_last_cmd;
static SensorData s_last_sensor;
static bool s_initialized = false;

int16_t GimbalController_PitchControl(uint8_t id, float rate_normalized,
                                      SensorData *sensor_data) {
  (void)sensor_data; // Not used for pitch
  MotorContext_t *c = MotorDriver_GetContext(id);
  if (!c || !c->angle_initialized) {
    return 0;
  }

  // Joystick control with sensitivity scaling
  float sensitivity = 40.0f;
  c->angle_target += c->config->direction * sensitivity * rate_normalized;

  // Determine if this is pitch motor (has angle limits)
  bool is_pitch_motor = (c->role == MOTOR_ROLE_GIMBAL_PITCH);
  float max_encoder = (c->config->limits.gm6020.angle_max > 0.0f)
                          ? c->config->limits.gm6020.angle_max
                          : 8192.0f;

  if (is_pitch_motor) {
    if (c->angle_target > c->config->limits.gm6020.angle_max)
      c->angle_target = c->config->limits.gm6020.angle_max;
    if (c->angle_target < c->config->limits.gm6020.angle_min)
      c->angle_target = c->config->limits.gm6020.angle_min;
  } else {
    if (c->angle_target >= max_encoder)
      c->angle_target = c->config->limits.gm6020.angle_min;
    else if (c->angle_target < c->config->limits.gm6020.angle_min)
      c->angle_target = max_encoder;
  }

  float current_angle = (float)c->angle_raw;
  float error = c->angle_target - current_angle;
  if (error > max_encoder / 2.0f)
    error -= max_encoder;
  else if (error < -max_encoder / 2.0f)
    error += max_encoder;

  float cmd = PID_Calculate(&c->pid_outer, error, 0.0f);

  if (is_pitch_motor) {
    float ang01 = current_angle / max_encoder;
    float ang_rad = ang01 * (2.0f * (float)M_PI);
    float gravity_ff = c->config->direction *
                       c->config->limits.gm6020.gravity_compensation *
                       sinf(ang_rad);
    cmd += gravity_ff;
  }
  float max_abs = 25000.0f;
  if (cmd > max_abs)
    cmd = max_abs;
  if (cmd < -max_abs)
    cmd = -max_abs;

  // Pitch PID tuning CSV (20Hz rate limited in main.c) - DISABLED for clean output
  // Format: GIM,timestamp_ms,angle_target,angle_current,speed_rpm,cmd,error,rate_scaled
  // LOG_CSV(LOG_TAG_GIM, "PITCH,%.2f,%.2f,%d,%.2f,%.2f,%.2f",
  //         c->angle_target,
  //         current_angle,
  //         c->speed_rpm,
  //         cmd,
  //         error,
  //         rate_normalized * 300.0f);

  return (int16_t)cmd;
}

// Test mode: generates step signal for tuning
#define YAW_TEST_MODE 0

#if YAW_TEST_MODE
static int16_t test_counter = 0;
static int16_t test_target = 0;
#endif

int16_t GimbalController_YawControlWithCompensation(float rate_normalized,
                                                    SensorData *sensor_data,
                                                    bool use_imu_feedback) {
  MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
  if (!yaw || !yaw->angle_initialized)
    return 0;

#if YAW_TEST_MODE
  // Generate step signal for testing (full 360° rotation over 18 seconds)
  // Increments by 8192/18 ≈ 455 ticks every 1 second (200 cycles @ 5ms)
  if (test_counter++ % 200 == 0) {
    test_target += 8192 / 18;
    if (test_target >= 8192)
      test_target = 0;
  }
  yaw->angle_target = (float)test_target;
#else
  // Joystick control
  yaw->angle_target += 50.0f * rate_normalized;
#endif

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
  if (angle_error > YAW_CONTROL_ENC_MAX / 2.0f)
    angle_error -= YAW_CONTROL_ENC_MAX;
  if (angle_error < -YAW_CONTROL_ENC_MAX / 2.0f)
    angle_error += YAW_CONTROL_ENC_MAX;

  // ==========================
  // OUTER LOOP: angle → speed
  // ==========================
  float cmd_angle_to_speed = PID_Calculate(&yaw->pid_outer, 0.0f, -angle_error);

  // Speed limit for yaw control stability (from working commit 22e9ff0e7a)
  float rpm_limit = 440.0f;

  // Apply signed clamp
  if (cmd_angle_to_speed > rpm_limit)
    cmd_angle_to_speed = rpm_limit;
  if (cmd_angle_to_speed < -rpm_limit)
    cmd_angle_to_speed = -rpm_limit;

  // Speed feedback source selection:
  // - Spin mode: Use IMU gyro for absolute yaw stability
  // - Normal mode: Use motor encoder for better response
  float speed_feedback;
  if (use_imu_feedback) {
    // IMU gyro feedback (for spin mode)
    // Convert IMU gyro (rad/s) to RPM: 1 rad/s = 30/π RPM ≈ 9.549 RPM
    speed_feedback = -sensor_data->g_gz * 30.0f /
                     (float)M_PI; // negative because IMU z-axis convention
  } else {
    // Motor encoder speed feedback (for normal mode)
    speed_feedback = (float)yaw->speed_rpm;
  }

  float cmd_speed_to_current =
      PID_Calculate(&yaw->pid_inner, cmd_angle_to_speed, speed_feedback);

  // Clamp current
  if (cmd_speed_to_current > CURRENT_LIMIT)
    cmd_speed_to_current = CURRENT_LIMIT;
  if (cmd_speed_to_current < -CURRENT_LIMIT)
    cmd_speed_to_current = -CURRENT_LIMIT;

  // Yaw PID tuning CSV (20Hz rate limited in main.c) - DISABLED for clean output
  // Format: GIM,timestamp_ms,mode,angle_target,angle_current,speed_rpm,cmd_current,cmd_speed,rate,error,g_gz_filtered,c_gz
  // LOG_CSV(LOG_TAG_GIM, "YAW,%.2f,%.2f,%d,%.2f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f",
  //         yaw->angle_target,
  //         current,
  //         yaw->speed_rpm,
  //         cmd_speed_to_current,
  //         cmd_angle_to_speed,
  //         rate_normalized * 300.0f,
  //         angle_error,
  //         sensor_data->g_gz * YAW_CONTROL_GYRO_LPF_ALPHA +
  //             s_last_sensor.g_gz * (1.0f - YAW_CONTROL_GYRO_LPF_ALPHA),
  //         sensor_data->c_gz);

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
      static bool s_yaw_spin_hold_active = false;
      bool use_vision_target = s_last_cmd.vision_valid;
      bool use_spin_hold =
          (!use_vision_target) && (s_last_cmd.yaw_rate_memo > 0.5f);

      // Continuous angle control: update target angle every cycle when vision
      // is valid
      if (use_vision_target) {
        MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
        MotorContext_t *pitch = MotorDriver_GetContext(s_pitch_motor_id);

        if (yaw && yaw->angle_initialized) {
          float max_encoder = (yaw->config->limits.gm6020.angle_max > 0.0f)
                                  ? yaw->config->limits.gm6020.angle_max
                                  : 8192.0f;
          const float ticks_per_rad = max_encoder / (2.0f * (float)M_PI);
          float err_ticks = s_last_cmd.vision_yaw_err_rad * ticks_per_rad;
          // Update target angle continuously based on current angle + vision
          // error
          yaw->angle_target = (float)yaw->angle_raw + err_ticks;
          while (yaw->angle_target >= max_encoder)
            yaw->angle_target -= max_encoder;
          while (yaw->angle_target < 0.0f)
            yaw->angle_target += max_encoder;

          if (!s_yaw_vision_active) {
            PID_Reset(&yaw->pid_outer);
            PID_Reset(&yaw->pid_inner);
          }
          s_yaw_vision_active = true;
        }

        (void)pitch;
      } else {
        s_yaw_vision_active = false;
      }

      // Spin-hold mode: hold gimbal absolute yaw (deg) using gimbal IMU
      // yaw_total_angle. Target is carried via yaw_target_memo; enable flag via
      // yaw_rate_memo.
      if (use_spin_hold) {
        MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
        if (yaw && yaw->angle_initialized) {
          float max_encoder = (yaw->config->limits.gm6020.angle_max > 0.0f)
                                  ? yaw->config->limits.gm6020.angle_max
                                  : 8192.0f;

          // Calculate angle error with proper wrapping to [-180, 180] range
          float yaw_err_deg =
              s_last_cmd.yaw_target_memo - s_last_sensor.yaw_total_angle;

          // Wrap error to shortest path
          while (yaw_err_deg > 180.0f)
            yaw_err_deg -= 360.0f;
          while (yaw_err_deg < -180.0f)
            yaw_err_deg += 360.0f;

          // Convert to encoder ticks
          float ticks_per_deg = max_encoder / 360.0f;
          float err_ticks = yaw_err_deg * ticks_per_deg;

          // Set target angle in encoder space
          yaw->angle_target = (float)yaw->angle_raw + err_ticks;
          while (yaw->angle_target >= max_encoder)
            yaw->angle_target -= max_encoder;
          while (yaw->angle_target < 0.0f)
            yaw->angle_target += max_encoder;

          if (!s_yaw_spin_hold_active) {
            PID_Reset(&yaw->pid_outer);
            PID_Reset(&yaw->pid_inner);
          }
          s_yaw_spin_hold_active = true;
        } else {
          s_yaw_spin_hold_active = false;
        }
      } else {
        s_yaw_spin_hold_active = false;
      }

      int16_t pitch_current = GimbalController_PitchControl(
          s_pitch_motor_id, use_vision_target ? 0.0f : s_last_cmd.pitch_rate,
          &s_last_sensor);
      int16_t yaw_current = GimbalController_YawControlWithCompensation(
          (use_vision_target || use_spin_hold) ? 0.0f : s_last_cmd.yaw_rate,
          &s_last_sensor,
          use_spin_hold // Use IMU feedback only in spin mode
      );

      // Send motor currents (module layer handles CAN)
      MotorDriver_SendCurrent(s_pitch_motor_id, pitch_current);
      MotorDriver_SendCurrent(s_yaw_motor_id, yaw_current);
      MotorDriver_FlushAll();
    } else {
      // Gimbal disabled, send zero current
      MotorDriver_SendCurrent(s_pitch_motor_id, 0);
      MotorDriver_SendCurrent(s_yaw_motor_id, 0);
      MotorDriver_FlushAll();
    }

    // Gimbal position logging (20Hz rate limited in main.c)
    // Format: GIM,timestamp_ms,ENCODER,yaw_raw,pitch_raw,yaw_tgt,pitch_tgt
    // Log regardless of enabled state to monitor motor positions
    MotorContext_t *yaw_ctx = MotorDriver_GetContext(s_yaw_motor_id);
    MotorContext_t *pitch_ctx = MotorDriver_GetContext(s_pitch_motor_id);
    if (yaw_ctx && pitch_ctx) {
      LOG_CSV(LOG_TAG_GIM, "ENCODER,%.2f,%.2f,%.2f,%.2f",
              (float)yaw_ctx->angle_raw,
              (float)pitch_ctx->angle_raw,
              yaw_ctx->angle_target,
              pitch_ctx->angle_target);
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

  // Find gimbal motors by role (module layer handles config)
  uint8_t pitch_motors[1];
  uint8_t yaw_motors[1];

  if (MotorDriver_FindByRole(MOTOR_ROLE_GIMBAL_PITCH, pitch_motors, 1) > 0) {
    s_pitch_motor_id = pitch_motors[0];
  }

  if (MotorDriver_FindByRole(MOTOR_ROLE_GIMBAL_YAW, yaw_motors, 1) > 0) {
    s_yaw_motor_id = yaw_motors[0];
  }

  // Subscribe to messages
  (void)MsgCenter_Subscribe(TOPIC_GIMBAL_CMD, on_gimbal_cmd, NULL);
  (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

  s_initialized = true;
}

/**
 * @brief Wait for gimbal to reach initial alignment position
 * @note This function sends gimbal commands and waits for both yaw and pitch
 *       to reach their initial positions before returning.
 */
void Gimbal_WaitForAlignment(void) {
  const float ALIGNMENT_THRESHOLD = 50.0f; // encoder ticks
  const uint32_t TIMEOUT_MS = 10000;       // 10 seconds timeout
  const uint32_t CHECK_INTERVAL_MS = 100;

  USB_CDC_Printf("[Gimbal] Waiting for gimbal alignment...\r\n");

  // Create gimbal command to enable gimbal and hold initial position
  GimbalCmd cmd = {.enabled = true,
                   .pitch_rate = 0.0f,
                   .yaw_rate = 0.0f,
                   .yaw_rate_memo = 0.0f,
                   .yaw_target_memo = 0.0f,
                   .vision_valid = false,
                   .vision_yaw_err_rad = 0.0f,
                   .vision_pitch_err_rad = 0.0f,
                   .vision_ts_ms = 0};

  uint32_t start_time = HAL_GetTick();

  while (HAL_GetTick() - start_time < TIMEOUT_MS) {
    // Continuously send command to keep gimbal control
    MsgCenter_Publish(TOPIC_GIMBAL_CMD, &cmd, sizeof(cmd));
    MsgCenter_Dispatch(); // Dispatch messages immediately

    HAL_Delay(CHECK_INTERVAL_MS);

    // Check if gimbal has reached target position
    MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
    MotorContext_t *pitch = MotorDriver_GetContext(s_pitch_motor_id);

    if (yaw && pitch && yaw->angle_initialized && pitch->angle_initialized) {
      float yaw_error = fabsf(yaw->angle_target - (float)yaw->angle_raw);
      float pitch_error = fabsf(pitch->angle_target - (float)pitch->angle_raw);

      // Handle yaw wraparound (0-8192 encoder range)
      if (yaw_error > 4096.0f) {
        yaw_error = 8192.0f - yaw_error;
      }

      USB_CDC_Printf("[Gimbal] Yaw error: %.1f, Pitch error: %.1f\r\n",
                     yaw_error, pitch_error);

      if (yaw_error < ALIGNMENT_THRESHOLD &&
          pitch_error < ALIGNMENT_THRESHOLD) {
        USB_CDC_Printf("[Gimbal] Alignment complete!\r\n");
        return;
      }
    }
  }

  USB_CDC_Printf(
      "[Gimbal] Warning: Alignment timeout, continuing anyway...\r\n");
}
