#ifndef SENTRY_SWERVE_H
#define SENTRY_SWERVE_H

#include "config_types.h"

/**
 * @brief Sentry Robot with Diagonal Swerve Drive Configuration
 *
 * This configuration is for a sentry robot with diagonal swerve drive:
 * - 2x active swerve modules (front-left + back-right, diagonal layout)
 * - 2x passive/omni wheels (front-right + back-left)
 * - Each swerve module: 1x M3508 drive + 1x GM6020 steering
 * - Total 4 chassis motors on CAN1
 * - 2x GM6020 gimbal motors on CAN2
 * - 3x M3508 shooter motors on CAN2
 */

// Motor configuration array
static const MotorConfig_t g_motor_configs_sentry_swerve[] = {
    // ========== CHASSIS MOTORS (4 motors on CAN1) ==========

    // Front-left swerve drive motor (M3508, ID 0)
    {
        .motor_id = 0,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x201,  // M3508: 0x200 + motor_id
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },  // Speed PID
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Back-right swerve drive motor (M3508, ID 1)
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

    // Front-left swerve steering motor (GM6020, ID 4)
    {
        .motor_id = 4,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_CHASSIS_STEER,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x208,  // GM6020: 0x204 + motor_id
        .can_tx_id = 0x1FF,  // Motors 1-4 use 0x1FF
        .tx_slot = 0,        // Motor 4 -> slot 0
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = -1.0f  // Auto-initialize from current position
        },
        .pid_outer = { 15.0f, 0.0f, 1.0f, 20000.0f, 10000.0f },  // Angle PID for steering
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }            // Not used
    },

    // Back-right swerve steering motor (GM6020, ID 5)
    {
        .motor_id = 5,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_CHASSIS_STEER,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x209,  // GM6020: 0x204 + motor_id
        .can_tx_id = 0x1FF,
        .tx_slot = 1,        // Motor 5 -> slot 1
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = -1.0f  // Auto-initialize from current position
        },
        .pid_outer = { 15.0f, 0.0f, 1.0f, 20000.0f, 10000.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // ========== SHOOTER MOTORS (3x M3508 on CAN2) ==========
    // Same as infantry_standard

    // Turntable/feed motor (ID 8)
    {
        .motor_id = 8,
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
        .pid_outer = { 3.0f, 0.0f, 0.0f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Friction wheel 1 (ID 9)
    {
        .motor_id = 9,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x206,
        .can_tx_id = 0x1FF,
        .tx_slot = 1,
        .direction = -1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 5.0f, 0.5f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Friction wheel 2 (ID 11)
    {
        .motor_id = 11,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x208,
        .can_tx_id = 0x1FF,
        .tx_slot = 3,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 5.0f, 0.5f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // ========== GIMBAL MOTORS (2x GM6020 on CAN2) ==========
    // Same as infantry_standard

    // Yaw gimbal motor (ID 6)
    {
        .motor_id = 6,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x20A,  // GM6020: 0x204 + motor_id
        .can_tx_id = 0x2FF,
        .tx_slot = 1,
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = 2183.0f
        },
        .pid_outer = { 0.70f, 0.045f, 0.04f, 300.0f, 300.0f },
        .pid_inner = { 30.0f, 0.01f, 3.0f, 30000.0f, 4000.0f }
    },

    // Pitch gimbal motor (ID 7)
    {
        .motor_id = 7,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_PITCH,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x20B,
        .can_tx_id = 0x2FF,
        .tx_slot = 2,
        .direction = -1,
        .limits.gm6020 = {
            .angle_min = 1000.0f,
            .angle_max = 4000.0f,
            .gravity_compensation = 5000.0f,
            .initial_angle = 3370.0f
        },
        .pid_outer = { 20.0f, 0.0f, 2.0f, 30000.0f, 25000.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    }
};

// Robot configuration structure
static const RobotConfig_t g_robot_config_sentry_swerve = {
    .name = "Sentry Swerve (Diagonal)",
    .chassis_type = CHASSIS_TYPE_SWERVE,  // Swerve drive chassis
    .chassis_motor_count = 4,  // 2 drive + 2 steer
    .gimbal_motor_count = 2,
    .shooter_motor_count = 3,
    .motor_configs = g_motor_configs_sentry_swerve,
    .total_motor_count = 9
};

#endif // SENTRY_SWERVE_H
