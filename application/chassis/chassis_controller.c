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

extern CAN_HandleTypeDef hcan1;

#define SPEED_PID_KP (10.0f)
#define SPEED_PID_KI (0.0f)
#define SPEED_PID_KD (0.1f)
#define SPEED_PID_OUTPUT_MAX (15000)
#define SPEED_PID_INTEGRAL_MAX (7500)
#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)
#define MOTOR_STDID_1_4 (0x200U)

typedef struct
{
    float x;
    float y;
} Pair;

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static SensorData s_last_sensor;
static ChassisController s_ctrl;

// Motor direction correction array for mecanum wheel kinematics
// Indices: [front-left, front-right, back-left, back-right]
static const int8_t MOTOR_DIR[CHASSIS_MOTOR_COUNT] = {-1, +1, +1, -1};

static void ResetPidIntegrals(ChassisController *controller)
{
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->speed_pids[i].integral = 0.0f;
    }
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        return 0;
    }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

void ChassisController_Init(ChassisController *controller)
{
    if (controller == NULL)
        return;
    memset(controller, 0, sizeof(ChassisController));
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        PID_Init(&controller->speed_pids[i], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
                 SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
    }
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->target_speeds[i] = 0.0f;
    }
}
/* ================== 参数定义 ================== */

// 舵轮几何参数（不用太准，后期能调）
#define STEER_WHEEL_TRACK 0.3f // 车宽 (m)
#define STEER_WHEEL_BASE 0.3f  // 轴距 (m)

// 小速度死区，防止乱转舵
#define SPEED_DEADZONE 0.01f

/* ================== 电机接口（你替换成自己的） ================== */

// 舵轮ID定义
#define LF_STEER_ID 1
#define LF_DRIVE_ID 2
#define RB_STEER_ID 3
#define RB_DRIVE_ID 4

// 舵向电机（角度控制）——实现：保存目标角并（TODO）发送到舵向控制器
static float steer_targets[2] = {0.0f, 0.0f}; // [0]=LF, [1]=RB

void SteerMotor_SetAngle(uint8_t id, float angle_rad)
{
    // Store target angle for later processing (replace with CAN or dedicated steer driver)
    if (id == LF_STEER_ID)
    {
        steer_targets[0] = angle_rad;
    }
    else if (id == RB_STEER_ID)
    {
        steer_targets[1] = angle_rad;
    }
#if CHASSIS_DEBUG
    USB_CDC_Printf("[CHASSIS_DEBUG] Steer id=%u angle=%.3f\r\n", (unsigned)id, angle_rad);
#endif
    // TODO: send steer command to hardware (CAN or other)
}

// 驱动电机（速度控制）——实现：写入 chassis controller 的 target_speeds
void DriveMotor_SetSpeed(uint8_t id, float speed)
{
    // Map drive IDs to controller->target_speeds indices:
    // LF_DRIVE_ID maps to chassis motor index 0 (front-left)
    // RB_DRIVE_ID maps to chassis motor index 3 (back-right)
    if (id == LF_DRIVE_ID)
    {
        s_ctrl.target_speeds[0] = MOTOR_DIR[0] * speed;
    }
    else if (id == RB_DRIVE_ID)
    {
        s_ctrl.target_speeds[3] = MOTOR_DIR[3] * speed;
    }
    // Note: other drive motors (RF/LB) are disabled in this mode (set to 0 elsewhere)
}

/* ================== 状态变量 ================== */

static float last_steer_angle = 0.0f;

/* ================== 核心运动学 ================== */

/**
 * @brief 对角线双舵轮（同向转向）运动学
 * @param vx 前后速度
 * @param vy 左右速度
 * @param wz 旋转速度
 */
static void TwoDiagonalSteerCalculate(float vx, float vy, float wz)
{
    float steer_angle;
    float base_speed;

    /* ---------- 1. 舵轮转向角 ---------- */
    if (fabsf(vx) < SPEED_DEADZONE && fabsf(vy) < SPEED_DEADZONE)
    {
        // 小速度时保持上一次角度
        steer_angle = last_steer_angle;
    }
    else
    {
        steer_angle = atan2f(vy, vx);
        last_steer_angle = steer_angle;
    }

    /* ---------- 2. 平移基础速度 ---------- */
    base_speed = sqrtf(vx * vx + vy * vy);

    /* ---------- 3. 旋转引起的差速 ---------- */
    // 左前和右后对角线差速
    float rot_comp = wz * (STEER_WHEEL_BASE + STEER_WHEEL_TRACK) * 0.5f;

    float speed_lf = base_speed - rot_comp;
    float speed_rb = base_speed + rot_comp;

#if CHASSIS_DEBUG
    USB_CDC_Printf("[CHASSIS_DEBUG] steer_angle=%.3f base_speed=%.3f speed_lf=%.3f speed_rb=%.3f\r\n",
                   steer_angle, base_speed, speed_lf, speed_rb);
#endif

    /* ---------- 4. 输出到电机 ---------- */

    // 舵向电机：两个同角度
    SteerMotor_SetAngle(LF_STEER_ID, steer_angle);
    SteerMotor_SetAngle(RB_STEER_ID, steer_angle);

    // 驱动电机
    DriveMotor_SetSpeed(LF_DRIVE_ID, speed_lf);
    DriveMotor_SetSpeed(RB_DRIVE_ID, speed_rb);

    // 另外两个全向轮：不管（或置 0）
    // DriveMotor_SetSpeed(RF_ID, 0);
    // DriveMotor_SetSpeed(LB_ID, 0);
}

/* ================== 底盘主任务 ================== */

void ChassisTask(float vx, float vy, float wz)
{
    // 后续你可以在这里加：
    // - 急停判断
    // - 模式切换
    // - 功率限制

    TwoDiagonalSteerCalculate(vx, vy, wz);
}

void ChassisController_Update(ChassisController *controller, SensorData *sensor_data)
{
    if (controller == NULL)
        return;

    float vx_norm = s_last_cmd.vx;
    float vy_norm = s_last_cmd.vy;
    float wz_norm = s_last_cmd.wz;
    float scale = (float)CHASSIS_DEMO_TARGET_SPEED / 2.0f;
    float omega = wz_norm * scale;

    // Coordinate transformation is already done in cmd_controller for spin mode
    // So we directly use the vx/vy from command without additional transformation
    float vx = vx_norm * scale;
    float vy = vy_norm * scale;

#if CHASSIS_MODE_TWO_DIAG_STEER
    // Two-diagonal steer mode: use LF & RB steer+drive
    TwoDiagonalSteerCalculate(vx, vy, omega);
    // Ensure disabled motors have zero target speeds (front-right and back-left)
    controller->target_speeds[1] = 0.0f;
    controller->target_speeds[2] = 0.0f;
#else
    controller->target_speeds[0] = MOTOR_DIR[0] * (vx - vy + omega);
    controller->target_speeds[1] = MOTOR_DIR[1] * (vx + vy - omega);
    controller->target_speeds[2] = MOTOR_DIR[2] * (vx - vy - omega);
    controller->target_speeds[3] = MOTOR_DIR[3] * (vx + vy + omega);
#endif

    controller->running = s_last_cmd.enabled;
}

void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
{
    if (controller == NULL)
        return;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        int16_t motor_current = ComputeSingleMotorCurrent(
            &controller->speed_pids[i],
            controller->target_speeds[i],
            &controller->motor_feedbacks[i],
            current_tick);
        controller->output_currents[i] = motor_current;
    }
    CAN_Manager_SendMotorCurrents4(&hcan1, MOTOR_STDID_1_4,
                                   controller->output_currents[0], controller->output_currents[1], controller->output_currents[2], controller->output_currents[3]);
}

void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT])
{
    if (controller == NULL || speeds == NULL)
        return;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->target_speeds[i] = speeds[i];
    }
}

void ChassisController_Stop(ChassisController *controller)
{
    if (controller == NULL)
        return;
    controller->running = false;
    ResetPidIntegrals(controller);
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        controller->target_speeds[i] = 0.0f;
    }
}

const int16_t *ChassisController_GetOutputCurrents(const ChassisController *controller)
{
    if (controller == NULL)
        return NULL;
    return controller->output_currents;
}

bool ChassisController_IsRunning(const ChassisController *controller)
{
    if (controller == NULL)
        return false;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        if (controller->target_speeds[i] != 0)
        {
            return true;
        }
    }
    return false;
}

void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT)
        return;
    controller->motor_feedbacks[motor_id].angle = angle;
    controller->motor_feedbacks[motor_id].speed = speed;
    controller->motor_feedbacks[motor_id].current = current;
    controller->motor_feedbacks[motor_id].temp = temp;
    controller->motor_feedbacks[motor_id].last_update_time = current_tick;
}

// Subscription callbacks
static void on_chassis_cmd(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(ChassisCmd))
    {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
        // Update controller and compute currents when command arrives
        ChassisController_Update(&s_ctrl, &s_last_sensor);
        ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
    }
}

static void on_imu_update(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(SensorData))
    {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

static void on_motor_feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(MotorFeedbackEvent))
    {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
        // Only process chassis motor feedback (id < 4)
        if (m->id < 4)
        {
            ChassisController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}

void ChassisApp_Init(void)
{
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));
    ChassisController_Init(&s_ctrl);
    (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
}

ChassisController *ChassisApp_GetController(void)
{
    return &s_ctrl;
}
