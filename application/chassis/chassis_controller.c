#include "chassis_controller.h"
#include "can.h"
#include "can_manager.h"
#include <string.h>
#include <math.h>
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "can_comm.h"
#include "printing.h"
#include "cmd_controller.h"
#include "robot_config.h"
#include "arm_math.h"

extern CAN_HandleTypeDef hcan1;

#define SPEED_PID_KP (5.0f)
#define SPEED_PID_KI (0.5f)
#define SPEED_PID_KD (0.1f)
#define SPEED_PID_OUTPUT_MAX (15000)
#define SPEED_PID_INTEGRAL_MAX (7500)
#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)
#define MOTOR_STDID_1_4 (0x200U)

typedef struct { float x; float y; } Pair;

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static SensorData s_last_sensor;
static ChassisController s_ctrl;

/**
 * @brief Transform velocity from gimbal frame to chassis frame
 * @param gimbal_speed Velocity in gimbal frame (x: forward, y: left)
 * @param gimbal_chassis_angle Gimbal yaw angle relative to chassis (radians)
 * @return Velocity in chassis frame
 *
 * Coordinate transformation matrix:
 * [chassis_vx]   [cos(θ)  -sin(θ)] [gimbal_vx]
 * [chassis_vy] = [sin(θ)   cos(θ)] [gimbal_vy]
 *
 * Where θ is the gimbal's yaw angle relative to chassis frame
 */
static Pair transform_gimbal_to_chassis(Pair gimbal_speed, float gimbal_chassis_angle) {
    Pair chassis_speed;

#if ENABLE_GIMBAL_FOLLOWING
    // Perform coordinate transformation when gimbal-following is enabled
    float cos_theta = arm_cos_f32(gimbal_chassis_angle);
    float sin_theta = arm_sin_f32(gimbal_chassis_angle);

    chassis_speed.x = gimbal_speed.x * cos_theta - gimbal_speed.y * sin_theta;
    chassis_speed.y = gimbal_speed.x * sin_theta + gimbal_speed.y * cos_theta;
#else
    // No transformation - directly use gimbal frame velocities
    chassis_speed.x = gimbal_speed.x;
    chassis_speed.y = gimbal_speed.y;
#endif

    return chassis_speed;
}

// Motor direction configuration (from robot_config.h)
static const int8_t MOTOR_DIR[CHASSIS_MOTOR_COUNT] = {
    MOTOR_DIR_LF,  // Motor 0: Left-front
    MOTOR_DIR_RF,  // Motor 1: Right-front
    MOTOR_DIR_LB,  // Motor 2: Left-back
    MOTOR_DIR_RB   // Motor 3: Right-back
};

static float RampTowards(float current, float target, float step)
{
    if (current < target) { current += step; if (current > target) current = target; }
    else if (current > target) { current -= step; if (current < target) current = target; }
    return current;
}

static void ResetPidIntegrals(ChassisController *controller)
{
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->speed_pids[i].integral = 0.0f; }
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) { return 0; }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

void ChassisController_Init(ChassisController *controller)
{
    if (controller == NULL) return;
    memset(controller, 0, sizeof(ChassisController));
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        PID_Init(&controller->speed_pids[i], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 
                 SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    }
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        controller->target_speeds[i] = 0.0f;
        controller->ramped_targets[i] = 0.0f;
    }
}

/**
 * @brief Update chassis controller with improved mecanum wheel kinematics
 * @param controller Chassis controller pointer
 * @param sensor_data Sensor data pointer (currently unused, kept for API compatibility)
 *
 * Mecanum wheel kinematics with gimbal offset compensation:
 *
 * For a mecanum wheel chassis, each wheel velocity is computed as:
 * v_wheel = v_x ± v_y ± ω * r
 *
 * Where:
 * - v_x, v_y are chassis linear velocities (after coordinate transformation)
 * - ω is chassis angular velocity
 * - r is the distance from wheel to rotation center (gimbal position)
 * - ± signs depend on wheel position and mecanum roller orientation
 *
 * Wheel arrangement (top view):
 *     Front
 *   LF    RF
 *     [G]      <- Gimbal (rotation center)
 *   LB    RB
 *     Back
 */
void ChassisController_Update(ChassisController *controller, SensorData* sensor_data)
{
    (void)sensor_data; // Currently unused, kept for API compatibility

    if (controller == NULL) return;

    // Step 1: Extract normalized command velocities (-1.0 to 1.0)
    float vx_norm = s_last_cmd.vx;  // Forward/backward (in gimbal frame)
    float vy_norm = s_last_cmd.vy;  // Left/right (in gimbal frame)
    float wz_norm = s_last_cmd.wz;  // Rotation

    // Step 2: Scale normalized velocities to actual motor RPM
    // Using CHASSIS_DEMO_TARGET_SPEED as the maximum speed reference
    const float speed_scale = (float)CHASSIS_DEMO_TARGET_SPEED / 2.0f;

    // Scale linear velocities
    Pair gimbal_speed;
    gimbal_speed.x = vx_norm * speed_scale;
    gimbal_speed.y = vy_norm * speed_scale;

    // Scale angular velocity
    // Note: Angular velocity needs to be scaled by wheel distance to center
    // This is applied per-wheel below using LF_CENTER, RF_CENTER, etc.
    float omega = wz_norm * speed_scale;

    // Step 3: Transform from gimbal coordinate frame to chassis coordinate frame
    // This enables "gimbal-following mode" where joystick input is relative to gimbal
    Pair chassis_speed = transform_gimbal_to_chassis(gimbal_speed, s_last_cmd.offset_angle);
    float chassis_vx = chassis_speed.x;
    float chassis_vy = chassis_speed.y;

    // Step 4: Compute individual wheel velocities using mecanum kinematics
    // Each wheel's contribution to chassis motion:
    // - Positive vx: all wheels rotate to move chassis forward
    // - Positive vy: LF/RB rotate forward, RF/LB rotate backward (chassis moves left)
    // - Positive omega: all wheels rotate to turn chassis counterclockwise
    //
    // The omega term is scaled by each wheel's distance to rotation center (gimbal)
    // This ensures accurate rotation about the gimbal position

    // Left-Front (LF): -vx - vy - omega*r
    float vt_lf = -chassis_vx - chassis_vy - omega * LF_CENTER;
    controller->target_speeds[0] = MOTOR_DIR[0] * vt_lf;

    // Right-Front (RF): -vx + vy - omega*r
    float vt_rf = -chassis_vx + chassis_vy - omega * RF_CENTER;
    controller->target_speeds[1] = MOTOR_DIR[1] * vt_rf;

    // Left-Back (LB): vx - vy - omega*r
    float vt_lb = chassis_vx - chassis_vy - omega * LB_CENTER;
    controller->target_speeds[2] = MOTOR_DIR[2] * vt_lb;

    // Right-Back (RB): vx + vy - omega*r
    float vt_rb = chassis_vx + chassis_vy - omega * RB_CENTER;
    controller->target_speeds[3] = MOTOR_DIR[3] * vt_rb;

    // Step 5: Update running state
    controller->running = s_last_cmd.enabled;

    // Step 6: Apply ramping for smooth acceleration
    // This prevents sudden motor current spikes and improves stability
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        controller->ramped_targets[i] = RampTowards(
            controller->ramped_targets[i],
            controller->target_speeds[i],
            CHASSIS_RAMP_STEP
        );
    }
}

void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        int16_t motor_current = ComputeSingleMotorCurrent(
            &controller->speed_pids[i],
            controller->ramped_targets[i],
            &controller->motor_feedbacks[i],
            current_tick
        );
        controller->output_currents[i] = motor_current;
    }
    CAN_Manager_SendMotorCurrents4(&hcan1, MOTOR_STDID_1_4,
        controller->output_currents[0], controller->output_currents[1], controller->output_currents[2], controller->output_currents[3]);
}

void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT])
{
    if (controller == NULL || speeds == NULL) return;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->target_speeds[i] = speeds[i]; }
}

void ChassisController_Stop(ChassisController *controller)
{
    if (controller == NULL) return;
    controller->running = false;
    ResetPidIntegrals(controller);
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->target_speeds[i] = 0.0f; }
}

const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller)
{
    if (controller == NULL) return NULL;
    return controller->output_currents;
}

bool ChassisController_IsRunning(const ChassisController *controller)
{
    if (controller == NULL) return false;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { if (controller->ramped_targets[i] != 0) { return true; } }
    return false;
}

void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
    controller->motor_feedbacks[motor_id].angle = angle;
    controller->motor_feedbacks[motor_id].speed = speed;
    controller->motor_feedbacks[motor_id].current = current;
    controller->motor_feedbacks[motor_id].temp = temp;
    controller->motor_feedbacks[motor_id].last_update_time = current_tick;
}

// Subscription callbacks
static void on_chassis_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(ChassisCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
        // Update controller and compute currents when command arrives
        ChassisController_Update(&s_ctrl, &s_last_sensor);
        ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
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
        // Only process chassis motor feedback (id < 4)
        if (m->id < 4) {
            ChassisController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}

void ChassisApp_Init(void) {
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));
    ChassisController_Init(&s_ctrl);
    (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
}

ChassisController* ChassisApp_GetController(void) {
    return &s_ctrl;
}


