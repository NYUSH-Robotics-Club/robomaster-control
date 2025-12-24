
#include "printing.h"

// Current print mode (default to USB)
static PrintMode_t current_print_mode = PRINT_MODE_USB;

/*
Set the print mode (USB or UART)
*/
void Debug_SetPrintMode(PrintMode_t mode)
{
    current_print_mode = mode;
}

/*
Get the current print mode
*/
PrintMode_t Debug_GetPrintMode(void)
{
    return current_print_mode;
}

/*
Unified send string function - supports both USB CDC and UART
*/
void Debug_SendString(const char* message)
{
    if (message == NULL)
    {
        return;
    }

    if (current_print_mode == PRINT_MODE_USB)
    {
        // Send via USB CDC
        CDC_Transmit_FS((uint8_t*)message, strlen(message));
    }
    else
    {
        // Send via USART6
        USART6_SendString(message);
    }
}

/*
Unified formatted print function - supports both USB CDC and UART
*/
void Debug_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);

    if (current_print_mode == PRINT_MODE_USB)
    {
        // Send via USB CDC
        CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n);
    }
    else
    {
        // Send via USART6
        HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
    }
}

/*
Legacy USB CDC send string function (for backward compatibility)
*/
#if defined(__GNUC__)
__attribute__((unused))
#endif
void USB_CDC_SendString(const char* message)
{
    if (message != NULL)
    {
        CDC_Transmit_FS((uint8_t*)message, strlen(message));
    }
}

/*
Legacy USB CDC formatted print function (for backward compatibility)
*/
void USB_CDC_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n);
}


 void Debug_PrintCANStatus(uint32_t current_tick, const CAN_Manager_t *can1_manager, const CAN_Manager_t *can2_manager)
{
  Debug_Printf("CAN1 tx_ok=%lu tx_err=%lu rx=%lu last_rx_id=0x%03lX last_tx=%lums last_rx=%lums\r\n",
      (unsigned long)CAN_Manager_GetTxOk(can1_manager),
      (unsigned long)CAN_Manager_GetTxErr(can1_manager),
      (unsigned long)CAN_Manager_GetRxFrames(can1_manager),
      (unsigned long)CAN_Manager_GetLastRxId(can1_manager),
      (unsigned long)(current_tick - CAN_Manager_GetLastTxTime(can1_manager)),
      (unsigned long)(current_tick - CAN_Manager_GetLastRxTime(can1_manager)));

  Debug_Printf("CAN2 tx_ok=%lu tx_err=%lu rx=%lu last_rx_id=0x%03lX last_tx=%lums last_rx=%lums\r\n",
      (unsigned long)CAN_Manager_GetTxOk(can2_manager),
      (unsigned long)CAN_Manager_GetTxErr(can2_manager),
      (unsigned long)CAN_Manager_GetRxFrames(can2_manager),
      (unsigned long)CAN_Manager_GetLastRxId(can2_manager),
      (unsigned long)(current_tick - CAN_Manager_GetLastTxTime(can2_manager)),
      (unsigned long)(current_tick - CAN_Manager_GetLastRxTime(can2_manager)));
}

 void Debug_PrintCANDiag(void)
{
  uint32_t can2_err  = HAL_CAN_GetError(&hcan2);
  uint32_t can2_esr  = hcan2.Instance->ESR;
  uint32_t can2_tsr  = hcan2.Instance->TSR;
  uint32_t can2_rf0r = hcan2.Instance->RF0R;
  uint32_t can2_tx_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
  Debug_Printf("CAN2 diag err=0x%08lX ESR=0x%08lX TSR=0x%08lX RF0R=0x%08lX TXMB_FREE=%lu\r\n",
    (unsigned long)can2_err,
    (unsigned long)can2_esr,
    (unsigned long)can2_tsr,
    (unsigned long)can2_rf0r,
    (unsigned long)can2_tx_free);
}

/*
Print RC and control path diagnostics
Usage: Called periodically in main loop for debugging
*/
void Debug_PrintRCDiagnostics(uint32_t current_tick, uint32_t *last_debug_time, uint32_t *last_frame_count)
{
  if (current_tick - *last_debug_time < USB_DEBUG_INTERVAL_MS)
  {
    return;
  }

  *last_debug_time = current_tick;
  uint32_t fc = RC_GetFrameCount();
  (void)(*last_frame_count);

  // RC and control path quick diagnostics
  {
    // Print sanitized channels (after baseline removal)
    const RC_ctrl_t *raw_rc = get_remote_control_point();
    int16_t ch0 = 0, ch2 = 0, ch3 = 0, ch4 = 0; 
    uint8_t swl = 0;
    
    if (raw_rc) 
    { 
      ch0 = raw_rc->rc.ch[0]; 
      ch2 = raw_rc->rc.ch[2]; 
      ch3 = raw_rc->rc.ch[3]; 
      ch4 = raw_rc->rc.ch[4]; 
      swl = (uint8_t)raw_rc->rc.s[0]; 
    }
    Debug_Printf("RC fc=%lu ch0=%d ch2=%d ch3=%d ch4=%d swL=%u\r\n",
      (unsigned long)fc, (int)ch0, (int)ch2, (int)ch3, (int)ch4, (unsigned int)swl);
  }
  
  *last_frame_count = fc;
}

