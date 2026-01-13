#ifndef HERO_STANDARD_H
#define HERO_STANDARD_H

#include "config_types.h"

/**
 * Hero Standard Robot Configuration
 *
 * CAN2:
 * - 4x M3508 (ID 1-4): chassis drive motors
 * - 1x M3508 (ID 5): shooter feed motor
 * - 1x GM6020 (ID 7): gimbal yaw motor
 */

// ========== MOTOR LIST ==========
static const MotorConfig_t g_motor_configs_hero_standard[] = {

    // --------------------------
    // CHASSIS DRIVE (M3508) - CAN2
    // --------------------------
    // Front-left chassis motor (hardware ID 1)
    {
        .motor_id = 0,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = -1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // Front-right chassis motor (hardware ID 3)
    {
        .motor_id = 1,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x203,
        .can_tx_id = 0x200,
        .tx_slot = 2,
        .direction = +1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // Back-left chassis motor (hardware ID 4)
    {
        .motor_id = 2,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x204,
        .can_tx_id = 0x200,
        .tx_slot = 3,
        .direction = +1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // Back-right chassis motor (hardware ID 2)
    {
        .motor_id = 3,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x202,
        .can_tx_id = 0x200,
        .tx_slot = 1,
        .direction = -1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // --------------------------
    // SHOOTER FEED (M3508) - CAN2
    // --------------------------
    // Feed motor (hardware ID 5)
    {
        .motor_id = 4,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FEED,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x205,
        .can_tx_id = 0x1FF,
        .tx_slot = 0,
        .direction = +1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {1.0f, 0.0f, 0.0f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // --------------------------
    // GIMBAL YAW (GM6020) - CAN2
    // --------------------------
    // Yaw gimbal motor (hardware ID 7)
    {
        .motor_id = 7,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x20B, // 0x204 + 7
        .can_tx_id = 0x2FF, // GM6020 ID 5-7 use 0x2FF
        .tx_slot = 2,       // motor_id - 5 = 7 - 5 = 2
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = -1.0f // Auto-init: use current angle on first feedback
        },
        .pid_outer = {1.5f, 0.03f, 0.03f, 400.0f, 300.0f},
        .pid_inner = {20.0f, 0.2f, 4.0f, 20000.0f, 6000.0f}
    }
};

// ========== ROBOT CONFIG ==========
static const RobotConfig_t g_robot_config_hero_standard = {
    .name = "Hero Standard",
    .chassis_motor_count = 4,
    .gimbal_motor_count = 1,
    .shooter_motor_count = 1,
    .motor_configs = g_motor_configs_hero_standard,
    .total_motor_count = 6,
    .enable_imu_calibration = 1
};

#endif // HERO_STANDARD_H
