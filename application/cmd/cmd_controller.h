#ifndef CMD_CONTROLLER_H
#define CMD_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "gimbal_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// Chassis command structure
typedef struct {
    float vx;           // X-axis velocity (forward/backward, in gimbal frame)
    float vy;           // Y-axis velocity (left/right, in gimbal frame)
    float wz;           // Rotation velocity
    float offset_angle; // Gimbal yaw angle relative to chassis (radians), for coordinate transformation
    bool enabled;       // Chassis enabled flag
} ChassisCmd;

// Shooter command structure  
typedef struct {
    bool friction_enabled;      // Friction wheel enabled
    bool feed_enabled;          // Feed mechanism enabled
} ShootCmd;

/**
 * @brief Initialize command controller
 */
void CmdController_Init(void);

/**
 * @brief Process remote control input and publish commands
 * @param current_tick Current timestamp in ms
 */
void CmdController_Task(uint32_t current_tick);

#ifdef __cplusplus
}
#endif

#endif // CMD_CONTROLLER_H

