#ifndef INFANTRY_STANDARD_H
#define INFANTRY_STANDARD_H

#include "config_types.h"

/**
 * @brief Standard Infantry Robot Configuration
 *
 * This configuration matches the current hardcoded values exactly:
 * - 4x M3508 chassis motors (mecanum wheels)
 * - 2x GM6020 gimbal motors (pitch + yaw)
 * - 3x M3508 shooter motors (turntable + 2x friction wheels)
 */

// Motor configuration array
static const MotorConfig_t g_motor_configs_infantry_standard[] = {
    // ========== CHASSIS MOTORS (4x M3508) ==========
    // Front-left chassis motor (ID 0)
    {
        .motor_id = 0,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = -1,  // Mecanum kinematics correction
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },  // Speed PID
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }           // Not used
    },

    // Front-right chassis motor (ID 1)
    {
        .motor_id = 1,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x202,
        .can_tx_id = 0x200,
        .tx_slot = 1,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Back-left chassis motor (ID 2)
    {
        .motor_id = 2,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x203,
        .can_tx_id = 0x200,
        .tx_slot = 2,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Back-right chassis motor (ID 3)
    {
        .motor_id = 3,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x204,
        .can_tx_id = 0x200,
        .tx_slot = 3,
        .direction = -1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // ========== SHOOTER MOTORS (3x M3508) ==========
    // Turntable/feed motor (ID 4)
    {
        .motor_id = 4,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FEED,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x205,
        .can_tx_id = 0x1FF,
        .tx_slot = 0,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 3.0f, 0.0f, 0.0f, 15000.0f, 7500.0f },  // Shooter feed PID (reduced Kp, increased Kd for stability)
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Friction wheel 1 (ID 5)
    {
        .motor_id = 5,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x206,
        .can_tx_id = 0x1FF,
        .tx_slot = 1,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 5.0f, 0.5f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Friction wheel 2 (ID 8, CAN RX 0x207)
    // Note: motor_id 8 != CAN RX mapping (0x207-0x201=6), but avoids conflict with pitch motor_id 7
    {
        .motor_id = 8,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x207,
        .can_tx_id = 0x1FF,
        .tx_slot = 3,  // Slot 3 in TX frame
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 5.0f, 0.5f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // ========== GIMBAL MOTORS (2x GM6020) ==========
    // Yaw gimbal motor (ID 6)
    {
        .motor_id = 6,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x20A,  // GM6020: 0x204 + motor_id
        .can_tx_id = 0x2FF,  // Motors 5-7 use 0x2FF
        .tx_slot = 1,        // Motor 6 -> slot 1 (motor_id - 5)
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = 2183.0f  // Calibration value from 2025-12-25
        },
        .pid_outer = { 0.70f, 0.045f, 0.04f, 30000.0f, 15000.0f },    // Yaw angle PID
        .pid_inner = { 30.0f, 0.01f, 3.0f, 30000.0f, 15000.0f }        // Yaw speed PID
    },

    // Pitch gimbal motor (ID 7)
    {
        .motor_id = 7,  // GM6020 hardware motor ID 7 (CAN RX 0x20B = 0x204 + 7)
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_PITCH,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x20B,  // GM6020: 0x204 + 7
        .can_tx_id = 0x2FF,
        .tx_slot = 2,        // Motor 7 -> slot 2 (motor_id - 5)
        .direction = -1,     // Pitch direction correction
        .limits.gm6020 = {
            .angle_min = 1000.0f,
            .angle_max = 4000.0f,
            .gravity_compensation = 5000.0f,  // Gravity compensation for pitch
            .initial_angle = 3370.0f  // Calibration value from 2025-12-25
        },
        .pid_outer = { 20.0f, 0.0f, 2.0f, 30000.0f, 25000.0f },  // Pitch PID
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }            // Not used for pitch
    }
};

// Robot configuration structure
static const RobotConfig_t g_robot_config_infantry_standard = {
    .name = "Infantry Standard",
    .chassis_motor_count = 4,
    .gimbal_motor_count = 2,
    .shooter_motor_count = 3,
    .motor_configs = g_motor_configs_infantry_standard,
    .total_motor_count = 9
};

#endif // INFANTRY_STANDARD_H
