// sentry_controller.h
// Note: This file uses chassis_controller.h interfaces for Sentry/Swerve chassis
// The actual implementation is in sentry_controller.c

#ifndef SENTRY_CONTROLLER_H
#define SENTRY_CONTROLLER_H

#include "chassis_controller.h"

// App wrapper functions (implementation uses ChassisController internally)
void ChassisApp_Init(void);
ChassisController* ChassisApp_GetController(void);

// Sentry-specific initialization helper
void Sentry_WaitForSteerAlignment(void);

#endif // SENTRY_CONTROLLER_H
