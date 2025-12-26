/**
 * @file robot_config.h
 * @brief Centralized robot physical parameters and configuration
 * @date 2025-12-27
 *
 * This file contains all robot-specific physical parameters, motor configurations,
 * and kinematic constants. Modify these values to match your specific robot setup.
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ROBOT PHYSICAL PARAMETERS
 * ============================================================================ */

/**
 * Chassis dimensions (in meters)
 * These values should match your robot's actual measurements
 */
#define WHEEL_BASE           (0.375f)    // 轴距: 375mm (distance between front and rear axles)
#define TRACK_WIDTH          (0.325f)    // 轮距: 325mm (distance between left and right wheels)
#define WHEEL_RADIUS         (0.0760f)   // 轮半径: 76mm (mecanum wheel radius)

/**
 * Gimbal mounting offset from chassis geometric center (in meters)
 * Positive X = forward, Positive Y = left (following right-hand rule)
 *
 * Current configuration: Gimbal centered on chassis (no offset)
 * If your gimbal is offset from chassis center, adjust these values accordingly
 */
#define CENTER_GIMBAL_OFFSET_X  (0.0f)   // Forward/backward offset (currently centered)
#define CENTER_GIMBAL_OFFSET_Y  (0.0f)   // Left/right offset (currently centered)

/* ============================================================================
 * MOTOR CONFIGURATION
 * ============================================================================ */

/**
 * Motor rotation directions
 * Values: +1 = counterclockwise (CCW), -1 = clockwise (CW) when viewed from motor shaft
 *
 * These directions should match your motor installation orientation.
 * Adjust if wheels rotate in unexpected directions during testing.
 */
#define MOTOR_DIR_LF  (-1)   // Left-front motor direction
#define MOTOR_DIR_RF  (+1)   // Right-front motor direction
#define MOTOR_DIR_LB  (+1)   // Left-back motor direction
#define MOTOR_DIR_RB  (-1)   // Right-back motor direction

/* ============================================================================
 * KINEMATIC CALCULATIONS
 * ============================================================================ */

/**
 * Helper macros for kinematic calculations
 */
#define HALF_WHEEL_BASE      (WHEEL_BASE / 2.0f)
#define HALF_TRACK_WIDTH     (TRACK_WIDTH / 2.0f)

/**
 * Distance from each wheel to the rotation center (gimbal position)
 *
 * These values represent the moment arm for each wheel when chassis rotates.
 * They are calculated based on wheel base, track width, and gimbal offset.
 *
 * Formula: Distance = sqrt((dx)^2 + (dy)^2)
 * where dx and dy are offsets from rotation center to each wheel
 *
 * For mecanum wheels, we use the simplified form:
 * distance = |x_offset| + |y_offset|
 *
 * Wheel positions (top view):
 *     Front (X+)
 *   LF        RF
 *       [G]          <- Gimbal (rotation center)
 *   LB        RB
 *     Back (X-)
 *
 * Left (Y+)  Right (Y-)
 */
#define LF_CENTER  (HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y)
#define RF_CENTER  (HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y)
#define LB_CENTER  (HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y)
#define RB_CENTER  (HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y)

/* ============================================================================
 * CHASSIS CONTROL PARAMETERS
 * ============================================================================ */

/**
 * Speed scaling and limits
 */
#define CHASSIS_MAX_LINEAR_SPEED_MPS   (3.0f)     // Maximum linear speed (m/s)
#define CHASSIS_MAX_ANGULAR_SPEED_RPS  (3.14159f) // Maximum angular speed (rad/s) ~ 180 deg/s

/**
 * Default speed for demo/testing (RPM)
 * This value is used as the maximum speed reference for normalized commands
 */
#define CHASSIS_DEMO_TARGET_SPEED      (7000)

/**
 * Ramping for smooth acceleration
 * Speed increment per control cycle to prevent sudden motor current spikes
 */
#define CHASSIS_RAMP_STEP              (50.0f)

/* ============================================================================
 * COORDINATE TRANSFORMATION
 * ============================================================================ */

/**
 * Enable/disable gimbal-following coordinate transformation
 *
 * When enabled (1):
 *   - Joystick input is interpreted relative to gimbal orientation
 *   - Forward on joystick always moves robot in direction gimbal is pointing
 *   - Requires valid IMU data for gimbal-chassis angle
 *
 * When disabled (0):
 *   - Joystick input is interpreted relative to chassis orientation
 *   - Forward on joystick always moves robot in chassis forward direction
 */
#define ENABLE_GIMBAL_FOLLOWING  (1)

/* ============================================================================
 * MATHEMATICAL CONSTANTS
 * ============================================================================ */

#ifndef PI
#define PI  (3.14159265358979323846f)
#endif

#define DEGREE_TO_RAD  (PI / 180.0f)
#define RAD_TO_DEGREE  (180.0f / PI)

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONFIG_H
