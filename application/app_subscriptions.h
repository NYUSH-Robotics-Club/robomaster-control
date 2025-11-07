#ifndef APP_SUBSCRIPTIONS_H
#define APP_SUBSCRIPTIONS_H

#include <stdint.h>
#include "chassis_controller.h"
#include "shooter_controller.h"

void ChassisApp_Init(void);
void ChassisApp_Tick(uint32_t tick_ms);

void ShooterApp_Init(void);
void ShooterApp_Tick(uint32_t tick_ms);

// Expose controller instances for CAN feedback wiring
ChassisController* ChassisApp_GetController(void);
ShooterController* ShooterApp_GetController(void);

#endif // APP_SUBSCRIPTIONS_H

