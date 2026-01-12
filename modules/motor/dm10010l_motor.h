#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdint.h>
#include <stddef.h>
#include "pid.h"

/* Minimal compatibility layer for HNU-style types used in original DM module.
   These are local definitions so the module can be integrated with this project
   without requiring the original HNU platform headers. They intentionally only
   include fields used by the DM implementation. */

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

typedef struct {
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
} DM_Motor_Measure_s;

typedef struct {
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;

/* Minimal motor control setting used by the HNU code. Only fields referenced
   by the dm module are present here. */
typedef enum { MOTOR_DIRECTION_FORWARD = 0, MOTOR_DIRECTION_REVERSE = 1 } Motor_Direction_e;
typedef enum { MOTOR_STOP = 0, MOTOR_ENALBED = 1 } Motor_Working_Type_e;

typedef struct {
    uint8_t motor_reverse_flag;
    int outer_loop_type;
} Motor_Control_Setting_s;

/* Simplified PID holder compatible with project PID_Controller naming */
typedef PID_Controller PIDInstance;

/* Lightweight placeholders for CAN/Daemon abstractions used by original code.
   We keep pointers as void* so the module can compile even if the original
   infrastructure is not present; dm module will call project CAN manager APIs
   instead of using these pointers directly. */
typedef struct {
    uint8_t tx_buff[8];  // 发送缓冲区（用于 DMMotorSetMode）
    uint8_t rx_buff[8];  // 接收缓冲区（用于 DMMotorDecode）
    void* id;
} CANInstance;
typedef void DaemonInstance;

typedef enum {
    CURRENT_LOOP = 0,  // 示例：电流环
    SPEED_LOOP,        // 速度环
    ANGLE_LOOP         // 角度环
} Closeloop_Type_e;

typedef struct {
    DM_Motor_Measure_s measure;
    uint8_t id;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace; /* unused in project-adapted path */
    DaemonInstance* motor_daemon;   /* unused if original daemon missing */
    uint32_t lost_cnt;
} DMMotorInstance;

typedef enum {
    DM_CMD_MOTOR_MODE = 0xfc,   // enable
    DM_CMD_RESET_MODE = 0xfd,   // stop
    DM_CMD_ZERO_POSITION = 0xfe,// set current encoder as zero
    DM_CMD_CLEAR_ERROR = 0xfb   // clear error
} DMMotor_Mode_e;

/* Motor init config is HNU-specific; provide an opaque forward-declaration so
   code that still constructs one can pass it. Integration code can build
   DMMotorInstance manually instead if desired. */
typedef struct Motor_Init_Config_s Motor_Init_Config_s;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorOuterLoop(DMMotorInstance *motor, int closeloop_type);
void DMMotorEnable(DMMotorInstance *motor);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit(void);

#endif // DMMOTOR_H