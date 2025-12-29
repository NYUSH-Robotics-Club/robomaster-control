#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "config_types.h"

/**
 * @brief Robot Configuration Selector
 *
 * This file selects the appropriate robot configuration based on
 * compile-time definitions set by CMake.
 *
 * To build for a specific robot type:
 *   cmake -DROBOT_TYPE=infantry_standard -S . -B build
 *   cmake -DROBOT_TYPE=infantry_swerve -S . -B build
 */

// Select configuration based on compile-time definition
#if defined(ROBOT_TYPE_infantry_swerve)
    #include "infantry_swerve.h"
    #define ACTIVE_ROBOT_CONFIG g_robot_config_infantry_swerve
#elif defined(ROBOT_TYPE_infantry_standard)
    #include "infantry_standard.h"
    #define ACTIVE_ROBOT_CONFIG g_robot_config_infantry_standard
#else
    // Default to standard infantry if no type specified
    #include "infantry_standard.h"
    #define ACTIVE_ROBOT_CONFIG g_robot_config_infantry_standard
#endif

/**
 * @brief Get the active robot configuration
 * @return Pointer to the active robot configuration structure
 */
const RobotConfig_t* RobotConfig_Get(void);

#endif // ROBOT_CONFIG_H
