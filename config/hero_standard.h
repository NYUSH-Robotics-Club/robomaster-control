#ifndef HERO_STANDARD_H
#define HERO_STANDARD_H

#include "config_types.h"

/**
 * @brief Standard Hero Robot Configuration
 *
 * This configuration is for Hero robot with:
 * - 4x M3508 chassis motors (mecanum wheels) - ALL ON CAN2
 * - 1x M3508 shooter feed motor (bullet dial) - ON CAN2
 * - 1x GM6020 gimbal yaw motor - ON CAN2
 *
 * IMPORTANT: All devices use CAN_CHANNEL_2
 */

// Motor configuration array
static const MotorConfig_t g_motor_configs_hero_standard[] = {
    // ========== CHASSIS MOTORS (4x M3508) - CAN2 ==========
    // Front-left chassis motor (ID 1)
    {
        .motor_id = 1,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = -1, // Mecanum kinematics correction
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f}, // Speed PID
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}          // Not used
    },

    // Front-right chassis motor (ID 2)
    {
        .motor_id = 2,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x202,
        .can_tx_id = 0x200,
        .tx_slot = 1,
        .direction = +1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // Back-left chassis motor (ID 3)
    {
        .motor_id = 3,
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

    // Back-right chassis motor (ID 4)
    {
        .motor_id = 4,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x204,
        .can_tx_id = 0x200,
        .tx_slot = 3,
        .direction = -1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // ========== SHOOTER MOTOR (1x M3508) - CAN2 ==========
    // Bullet dial/feed motor (ID 5)
    {
        .motor_id = 5,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FEED,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x205,
        .can_tx_id = 0x1FF,
        .tx_slot = 0,
        .direction = +1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {1.0f, 0.0f, 0.0f, 15000.0f, 7500.0f}, // Feed PID
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // ========== GIMBAL MOTOR (1x GM6020) - CAN2 ==========
    // Yaw gimbal motor (ID 6)
    {
        .motor_id = 6,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x20A, // GM6020: 0x204 + motor_id (0x204 + 6)
        .can_tx_id = 0x2FF, // GM6020 motors 5-7 use 0x2FF
        .tx_slot = 1,       // Motor 6 -> slot 1 (motor_id - 5)
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = 2183.0f // Calibration value (update after testing)
        },
        .pid_outer = {0.70f, 0.045f, 0.04f, 300.0f, 300.0f}, // Yaw angle PID
        .pid_inner = {30.0f, 0.01f, 3.0f, 30000.0f, 4000.0f} // Yaw speed PID
    }
};

// Robot configuration structure
static const RobotConfig_t g_robot_config_hero_standard = {
    .name = "Hero Standard",
    .chassis_motor_count = 4,
    .gimbal_motor_count = 1,  // 1 yaw motor
    .shooter_motor_count = 1, // 1 feed motor
    .motor_configs = g_motor_configs_hero_standard,
    .total_motor_count = 6,   // 4 chassis + 1 shooter + 1 gimbal
    .enable_imu_calibration = 1 // Hero needs IMU calibration
};

#endif // HERO_STANDARD_H
