#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "config_types.h"

/**
 * Robot Configuration Selector
 *
 * This file selects the appropriate robot configuration based on
 * compile-time definitions set by CMake:
 *
 *   cmake -S . -B build -DROBOT_TYPE=infantry_standard
 *   cmake -S . -B build -DROBOT_TYPE=infantry_swerve
 *   cmake -S . -B build -DROBOT_TYPE=sentry_swerve
 *   cmake -S . -B build -DROBOT_TYPE=sentry_standard   (if you still use it)
 */

#if defined(ROBOT_TYPE_infantry_swerve)
  #include "infantry_swerve.h"
  #define ACTIVE_ROBOT_CONFIG g_robot_config_infantry_swerve

#elif defined(ROBOT_TYPE_infantry_standard)
  #include "infantry_standard.h"
  #define ACTIVE_ROBOT_CONFIG g_robot_config_infantry_standard

#elif defined(ROBOT_TYPE_hero_standard)
  #include "hero_standard.h"
  #define ACTIVE_ROBOT_CONFIG g_robot_config_hero_standard

#elif defined(ROBOT_TYPE_sentry_swerve)
  #include "sentry_swerve.h"
  #define ACTIVE_ROBOT_CONFIG g_robot_config_sentry_swerve


#elif defined(ROBOT_TYPE_sentry_standard)
  #include "sentry_standard.h"
  #define ACTIVE_ROBOT_CONFIG g_robot_config_sentry_standard

#else
  #error "Unknown or missing ROBOT_TYPE_*. Set -DROBOT_TYPE=infantry_standard|infantry_swerve|hero_standard|sentry_swerve|sentry_standard"
#endif

/**
 * @brief Get the active robot configuration
 * @return Pointer to the active robot configuration structure
 */
const RobotConfig_t* RobotConfig_Get(void);

#endif // ROBOT_CONFIG_H
