#ifndef PRINTING_H
#define PRINTING_H

#include "usb_device.h"
#include "can_manager.h"
#include "remote_control.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "can.h"

// Debug info interval
#define USB_DEBUG_INTERVAL_MS (1000U)

void USB_CDC_SendString(const char* message);
void USB_CDC_Printf(const char *fmt, ...);
void Debug_PrintCANStatus(uint32_t current_tick, const CAN_Manager_t *can1_manager, const CAN_Manager_t *can2_manager);
void Debug_PrintCANDiag(void);
void Debug_PrintRCDiagnostics(uint32_t current_tick, uint32_t *last_debug_time, uint32_t *last_frame_count);

#endif


