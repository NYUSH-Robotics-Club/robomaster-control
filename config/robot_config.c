#include "robot_config.h"

/**
 * @brief Get the active robot configuration
 *
 * This function returns a pointer to the robot configuration structure
 * selected at compile time. The configuration is immutable and stored
 * in flash memory.
 *
 * @return Pointer to the active robot configuration structure
 */
const RobotConfig_t* RobotConfig_Get(void)
{
    return &ACTIVE_ROBOT_CONFIG;
}
