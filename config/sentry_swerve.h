#ifndef SENTRY_SWERVE_STANDARD_H
#define SENTRY_SWERVE_STANDARD_H

#include "config_types.h"

/**
 * Sentry / Swerve Chassis Config
 *
 * CAN1:
 * - 4x M3508 (ID 1-4): wheel drive motors
 * - 2x GM6020 (ID 5-6): wheel steering motors
 * - 1x GM6020 (ID 7): gimbal yaw motor
 *
 * CAN2:
 * - 1x M3508 (ID 1): shooter feed motor
 * - 1x GM6020 (ID 5): gimbal pitch motor
 * - 2x M3508 (ID 6, 8): friction wheels
 */

// ========== MOTOR LIST ==========
static const MotorConfig_t g_motor_configs_sentry_swerve[] = {

    // --------------------------
    // DRIVE (M3508) — CAN1
    // --------------------------
    // Wheel 0 drive (right-omniwheel)
    {
        .motor_id = 0,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = 1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // Wheel 1 drive (front normal wheel)
    {.motor_id = 1, .type = MOTOR_TYPE_M3508, .role = MOTOR_ROLE_CHASSIS_DRIVE, .can_channel = CAN_CHANNEL_1, .can_rx_id = 0x202, .can_tx_id = 0x200, .tx_slot = 1, .direction = -1, .limits.m3508 = {.speed_limit = 10000.0f}, .pid_outer = {8.0f, 0.0f, 0.1f, 15000.0f, 7500.0f}, .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // Wheel 2 drive (left-omniwheel)
    {.motor_id = 2, .type = MOTOR_TYPE_M3508, .role = MOTOR_ROLE_CHASSIS_DRIVE, .can_channel = CAN_CHANNEL_1, .can_rx_id = 0x203, .can_tx_id = 0x200, .tx_slot = 2, .direction = 1, .limits.m3508 = {.speed_limit = 10000.0f}, .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f}, .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // Wheel 3 drive (back normal wheel)
    {.motor_id = 3, .type = MOTOR_TYPE_M3508, .role = MOTOR_ROLE_CHASSIS_DRIVE, .can_channel = CAN_CHANNEL_1, .can_rx_id = 0x204, .can_tx_id = 0x200, .tx_slot = 3, .direction = -1, .limits.m3508 = {.speed_limit = 10000.0f}, .pid_outer = {8.0f, 0.0f, 0.005f, 15000.0f, 7500.0f}, .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // --------------------------
    // STEERING (GM6020) — CAN1
    // --------------------------
    //  (GM6020 ID 5 => RX 0x209)
    {
        .motor_id = 5, .type = MOTOR_TYPE_GM6020, .role = MOTOR_ROLE_CHASSIS_STEER, .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x209, // 0x204 + 5
        .can_tx_id = 0x2FF, // GM6020 ID 5-7 use 0x2FF
        .tx_slot = 0,       // For 0x2FF, slot = (motor_id - 5)
        .direction = 1,
        .limits.gm6020 = {.angle_min = 0.0f, .angle_max = 8192.0f, .gravity_compensation = 0.0f, .initial_angle = 1084.0f},
        .pid_outer = {0.7f, 0.045f, 0.018f, 300.0f, 300.0f}, // Yaw angle PID
        .pid_inner = {22.0f, 0.01f, 3.0f, 30000.0f, 4000.0f} // Yaw speed PID
    },

    // Steer motor B (GM6020 ID 6 => RX 0x20A)
    {
        .motor_id = 6, .type = MOTOR_TYPE_GM6020, .role = MOTOR_ROLE_CHASSIS_STEER, .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x20A, // 0x204 + 6
        .can_tx_id = 0x2FF,
        .tx_slot = 1,
        .direction = 1,
        .limits.gm6020 = {.angle_min = 0.0f, .angle_max = 8192.0f, .gravity_compensation = 0.0f, .initial_angle = 2434.0f},
        .pid_outer = {0.7f, 0.045f, 0.018f, 300.0f, 300.0f},
        .pid_inner = {22.0f, 0.01f, 3.0f, 30000.0f, 4000.0f}},

    // --------------------------
    // GIMBAL YAW (GM6020) — CAN1
    // --------------------------
    // Yaw gimbal motor (GM6020 ID 7 => RX 0x20B)
    // DUAL-LOOP CONTROL: angle → speed → current (same as infantry)
    {
        .motor_id = 7,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x20B, // 0x204 + 7
        .can_tx_id = 0x2FF, // GM6020 ID 5-7 use 0x2FF
        .tx_slot = 2,       // motor_id - 5 = 7 - 5 = 2
        .direction = 1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = -1.0f // Auto-init: use current angle on first feedback
        },
        .pid_outer = {3.0f, 0.06f, 0.055f, 500.0f, 400.0f},  // Angle → Speed (aggressive)
        .pid_inner = {35.0f, 0.7f, 7.0f, 30000.0f, 8000.0f} // Speed → Current (very aggressive)
    },

    // --------------------------
    // SHOOTER FEED (M3508) — CAN2
    // --------------------------
    // Feed/turntable motor (M3508 ID 1 => RX 0x201)
    {
        .motor_id = 4,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FEED,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x201, // Hardware ID 1
        .can_tx_id = 0x200, // M3508 ID 1-4 use 0x200
        .tx_slot = 0,
        .direction = +1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {1.0f, 0.0f, 0.0f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // --------------------------
    // GIMBAL PITCH (GM6020) — CAN2
    // --------------------------
    // Pitch gimbal motor (GM6020 ID 5 => RX 0x209)
    // Uses CAN2 with ID 5 to avoid confusion with Yaw (CAN1 ID 7)
    {
        .motor_id = 8,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_PITCH,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x209, // GM6020: 0x204 + 5
        .can_tx_id = 0x2FF, // GM6020 ID 5-7 use 0x2FF
        .tx_slot = 0,       // Motor 5 -> slot 0 (motor_id - 5)
        .direction = -1,    // Pitch direction correction
        .limits.gm6020 = {
            .angle_min = 1000.0f,
            .angle_max = 4000.0f,
            .gravity_compensation = 5000.0f, // Gravity compensation for pitch
            .initial_angle = -1.0f // Auto-init: use current angle on first feedback
        },
        .pid_outer = {20.0f, 0.0f, 2.0f, 30000.0f, 25000.0f}, // Pitch angle PID
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f} // Not used for pitch (single-loop control)
    },

    // --------------------------
    // SHOOTER FRICTION WHEELS (M3508) — CAN2
    // --------------------------
    // Friction wheel 1 (M3508 hardware ID 6 => RX 0x206) - Left/Upper wheel
    {
        .motor_id = 9,  // FIXED: Use motor_id 9 (was 5, conflicted with Steer A)
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x206, // Hardware ID 6
        .can_tx_id = 0x1FF, // M3508 ID 5-8 use 0x1FF
        .tx_slot = 1,       // Slot 1 in 0x1FF frame (hardware_id - 5 = 6 - 5 = 1)
        .direction = +1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {5.0f, 0.5f, 0.1f, 15000.0f, 7500.0f}, // Friction wheel PID
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // Friction wheel 2 (M3508 hardware ID 8 => RX 0x208) - Right/Lower wheel
    {
        .motor_id = 10,  // Use motor_id 10 (next available)
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x208, // Hardware ID 8
        .can_tx_id = 0x1FF, // M3508 ID 5-8 use 0x1FF
        .tx_slot = 3,       // Slot 3 in 0x1FF frame (hardware_id - 5 = 8 - 5 = 3)
        .direction = +1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {5.0f, 0.5f, 0.1f, 15000.0f, 7500.0f}, // Friction wheel PID
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    }};

// ========== ROBOT CONFIG ==========
static const RobotConfig_t g_robot_config_sentry_swerve = {
    .name = "Sentry Swerve Standard",
    .chassis_motor_count = 6, // 4 drive + 2 steer
    .gimbal_motor_count = 2,  // 1 yaw motor (CAN1) + 1 pitch motor (CAN2)
    .shooter_motor_count = 3, // 1 feed motor + 2 friction wheels (CAN2)
    .motor_configs = g_motor_configs_sentry_swerve,
    .total_motor_count = 11,  // 6 chassis + 2 gimbal + 3 shooter
    .enable_imu_calibration = 0 // Sentry does not need IMU calibration
};

#endif // SENTRY_SWERVE_STANDARD_H
