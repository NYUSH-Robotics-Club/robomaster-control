#include "can_manager.h"
#include <string.h>

// External CAN handles
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// CAN configuration parameters
#define CAN1_FILTER_BANK    0
#define CAN2_FILTER_BANK    14
#define CAN_FILTER_MODE     CAN_FILTERMODE_IDMASK
#define CAN_FILTER_SCALE    CAN_FILTERSCALE_32BIT
#define CAN_FIFO_ASSIGNMENT CAN_FILTER_FIFO0
#define CAN_IT_TYPE         CAN_IT_RX_FIFO0_MSG_PENDING

/**
 * @brief Initialize a CAN manager instance and bind it to a CAN handle.
 * @param manager CAN manager pointer (output).
 * @param channel Logical CAN channel selector (CAN1 or CAN2).
 * @param hcan Pointer to HAL CAN handle to use.
 * @param chassis_controller Optional chassis controller to forward feedback (can be NULL).
 * @param shooter_controller Optional shooter controller to forward feedback (can be NULL).
 * @return HAL_OK on success, HAL_ERROR on invalid args.
 */
HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager, CAN_Channel_t channel, CAN_HandleTypeDef *hcan, 
                                   ChassisController *chassis_controller, ShooterController *shooter_controller)
{
    if (manager == NULL || hcan == NULL) return HAL_ERROR;
    
    // Initialize manager structure
    memset(manager, 0, sizeof(CAN_Manager_t));
    manager->hcan = hcan;
    manager->chassis_controller = chassis_controller;
    manager->shooter_controller = shooter_controller;
    
    // Set filter bank based on channel
    switch (channel)
    {
        case CAN_CHANNEL_1:
            manager->filter_bank = CAN1_FILTER_BANK;
            break;
        case CAN_CHANNEL_2:
            manager->filter_bank = CAN2_FILTER_BANK;
            break;
        default:
            return HAL_ERROR;
    }
    
    manager->initialized = 1;
    return HAL_OK;
}

/**
 * @brief Configure filters, start CAN peripheral, and enable RX interrupts.
 * @param manager CAN manager pointer.
 * @return HAL status from filter/start/activate operations.
 */
HAL_StatusTypeDef CAN_Manager_Start(CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return HAL_ERROR;
    
    // Configure CAN filter
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = manager->filter_bank;
    filter.SlaveStartFilterBank = (manager->filter_bank == CAN1_FILTER_BANK) ? 14 : 0;
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTER_MODE;
    filter.FilterScale = CAN_FILTER_SCALE;
    filter.FilterFIFOAssignment = CAN_FIFO_ASSIGNMENT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    
    // Configure filter
    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(manager->hcan, &filter);
    if (status != HAL_OK) return status;
    
    // Start CAN
    status = HAL_CAN_Start(manager->hcan);
    if (status != HAL_OK) return status;
    
    // Activate notification
    status = HAL_CAN_ActivateNotification(manager->hcan, CAN_IT_TYPE);
    if (status != HAL_OK) return status;
    
    return HAL_OK;
}

/**
 * @brief Disable RX interrupts and stop the CAN peripheral.
 * @param manager CAN manager pointer.
 * @return HAL status from stop operation.
 */
HAL_StatusTypeDef CAN_Manager_Stop(CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return HAL_ERROR;
    
    // Deactivate notification
    HAL_CAN_DeactivateNotification(manager->hcan, CAN_IT_TYPE);
    
    // Stop CAN
    HAL_StatusTypeDef status = HAL_CAN_Stop(manager->hcan);
    if (status != HAL_OK) return status;
    
    return HAL_OK;
}

/**
 * @brief Check whether the manager has been initialized.
 * @param manager CAN manager pointer.
 * @return true if initialized; false otherwise.
 */
bool CAN_Manager_IsInitialized(const CAN_Manager_t *manager)
{
    if (manager == NULL) return false;
    return manager->initialized;
}

/**
 * @brief Get the underlying HAL CAN handle bound to this manager.
 * @param manager CAN manager pointer.
 * @return HAL CAN handle pointer or NULL if not initialized.
 */
CAN_HandleTypeDef* CAN_Manager_GetHandle(const CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return NULL;
    return manager->hcan;
}

/**
 * @brief Process a CAN RX pending callback for this manager's handle.
 * @note Should be called from the global callback when FIFO0 has data.
 * @param manager CAN manager pointer.
 * @param hcan The HAL CAN handle that triggered the callback.
 */
void CAN_Manager_ProcessCallback(CAN_Manager_t *manager, CAN_HandleTypeDef *hcan)
{
    if (manager == NULL || !manager->initialized || hcan != manager->hcan) return;
    
    CAN_RxHeaderTypeDef rx;
    uint8_t d[8];
    uint32_t current_tick = HAL_GetTick();

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, d) != HAL_OK) return;
    
    // Gimbal pitch feedback (CAN1 only)
    if (manager->filter_bank == CAN1_FILTER_BANK && rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x205 && rx.StdId<=0x20B) {
        uint8_t gid = (uint8_t)(rx.StdId - 0x204);
        if (gid >= 1 && gid <= 7) {
            uint16_t angle_raw = (uint16_t)((d[0]<<8) | d[1]);
            int16_t  speed_rpm = (int16_t)((d[2]<<8) | d[3]);
            pitch_on_feedback(gid, angle_raw, speed_rpm);
        }
    }
    // Motor feedback (both CAN1 and CAN2)
    else if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x201 && rx.StdId<=0x208) {
        uint8_t  mid   = rx.StdId - 0x201;
        if (mid < 8) {
            uint16_t angle = (d[0]<<8) | d[1];
            int16_t  speed = (int16_t)((d[2]<<8) | d[3]);
            int16_t  current = (int16_t)((d[4]<<8) | d[5]);
            uint8_t  temp = d[6];
            
            // Update chassis motor feedback (0-3)
            if (mid < 4 && manager->chassis_controller != NULL) {
                ChassisController_UpdateMotorFeedback(manager->chassis_controller, mid, angle, speed, current, temp, current_tick);
            }
            // Update shooter system motor feedback (4-7)
            else if (mid >= 4 && manager->shooter_controller != NULL) {
                ShooterController_UpdateMotorFeedback(manager->shooter_controller, mid, angle, speed, current, temp, current_tick);
            }
        }
    }
}

// Global CAN managers (defined in main.c)
extern CAN_Manager_t can1_manager;
extern CAN_Manager_t can2_manager;

/**
 * @brief Global RX callback router; forwards to the correct manager instance.
 * @note Call this from HAL_CAN_RxFifo0MsgPendingCallback.
 * @param hcan HAL CAN handle that has pending RX message on FIFO0.
 */
void CAN_Manager_GlobalCallback(CAN_HandleTypeDef *hcan)
{
    // Check if this is CAN1
    if (hcan == CAN_Manager_GetHandle(&can1_manager)) {
        CAN_Manager_ProcessCallback(&can1_manager, hcan);
    }
    // Check if this is CAN2
    else if (hcan == CAN_Manager_GetHandle(&can2_manager)) {
        CAN_Manager_ProcessCallback(&can2_manager, hcan);
    }
}

/**
 * @brief Send 4 motor currents in one CAN frame (StdId 0x200/0x1FF pattern).
 * @param hcan HAL CAN handle to send on.
 * @param std_id Standard ID to use (0x200 for 1-4, 0x1FF for 5-8).
 * @param i1 Current for slot 1.
 * @param i2 Current for slot 2.
 * @param i3 Current for slot 3.
 * @param i4 Current for slot 4.
 * @return HAL status from transmit.
 */
HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(CAN_HandleTypeDef *hcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    if (hcan == NULL) return HAL_ERROR;
    CAN_TxHeaderTypeDef tx = (CAN_TxHeaderTypeDef){0};
    uint8_t d[8];
    uint32_t mb;

    tx.StdId = std_id;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    d[0] = (uint8_t)(i1 >> 8); d[1] = (uint8_t)i1;
    d[2] = (uint8_t)(i2 >> 8); d[3] = (uint8_t)i2;
    d[4] = (uint8_t)(i3 >> 8); d[5] = (uint8_t)i3;
    d[6] = (uint8_t)(i4 >> 8); d[7] = (uint8_t)i4;

    return HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
}

/**
 * @brief Send a single GM6020 (gimbal pitch) current by logical motor id (1..7).
 * @param hcan HAL CAN handle to send on (typically CAN1).
 * @param motor_id GM6020 id in 1..7; selects StdId 0x1FF (1..4) or 0x2FF (5..7) and slot.
 * @param current Desired current; clamped to [-30000, 30000].
 * @return HAL status from transmit.
 */
HAL_StatusTypeDef CAN_Manager_SendGM6020Current(CAN_HandleTypeDef *hcan, uint8_t motor_id, int16_t current)
{
    if (hcan == NULL) return HAL_ERROR;
    if (motor_id < 1 || motor_id > 7) return HAL_ERROR;

    if (current >  30000) current =  30000;
    if (current < -30000) current = -30000;

    uint16_t stdId = (motor_id <= 4) ? 0x1FF : 0x2FF;
    uint8_t  slot  = (motor_id <= 4) ? (uint8_t)(motor_id - 1) : (uint8_t)(motor_id - 5);

    CAN_TxHeaderTypeDef tx = (CAN_TxHeaderTypeDef){0};
    uint8_t d[8] = {0};
    uint32_t mb;

    tx.StdId = stdId;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    d[slot*2 + 0] = (uint8_t)((current >> 8) & 0xFF);
    d[slot*2 + 1] = (uint8_t)( current       & 0xFF);

    return HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
}
