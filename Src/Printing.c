
#include "printing.h"

/*
Send string message via USB CDC
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
Formatted print over USB CDC (non-blocking best-effort)
*/
 void USB_CDC_Printf(const char *fmt, ...)
{
  static uint32_t last_output_time = 0;
  static uint32_t frame_count = 0;
  uint32_t now = HAL_GetTick();

  frame_count++;
  if(now - last_output_time >100) {
    last_output_time = now;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n);
  }
  
}


 void Debug_PrintCANStatus(uint32_t current_tick, const CAN_Manager_t *can1_manager, const CAN_Manager_t *can2_manager)
{
  USB_CDC_Printf("CAN1 tx_ok=%lu tx_err=%lu rx=%lu last_rx_id=0x%03lX last_tx=%lums last_rx=%lums\r\n",
      (unsigned long)CAN_Manager_GetTxOk(can1_manager),
      (unsigned long)CAN_Manager_GetTxErr(can1_manager),
      (unsigned long)CAN_Manager_GetRxFrames(can1_manager),
      (unsigned long)CAN_Manager_GetLastRxId(can1_manager),
      (unsigned long)(current_tick - CAN_Manager_GetLastTxTime(can1_manager)),
      (unsigned long)(current_tick - CAN_Manager_GetLastRxTime(can1_manager)));

  USB_CDC_Printf("CAN2 tx_ok=%lu tx_err=%lu rx=%lu last_rx_id=0x%03lX last_tx=%lums last_rx=%lums\r\n",
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
  USB_CDC_Printf("CAN2 diag err=0x%08lX ESR=0x%08lX TSR=0x%08lX RF0R=0x%08lX TXMB_FREE=%lu\r\n",
    (unsigned long)can2_err,
    (unsigned long)can2_esr,
    (unsigned long)can2_tsr,
    (unsigned long)can2_rf0r,
    (unsigned long)can2_tx_free);
}