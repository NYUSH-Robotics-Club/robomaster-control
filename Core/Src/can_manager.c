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

bool CAN_Manager_IsInitialized(const CAN_Manager_t *manager)
{
    if (manager == NULL) return false;
    return manager->initialized;
}

CAN_HandleTypeDef* CAN_Manager_GetHandle(const CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return NULL;
    return manager->hcan;
}

void CAN_Manager_ProcessCallback(CAN_Manager_t *manager, CAN_HandleTypeDef *hcan)
{
    if (manager == NULL || !manager->initialized || hcan != manager->hcan) return;
    
    CAN_RxHeaderTypeDef rx;
    uint8_t d[8];
    uint32_t current_tick = HAL_GetTick();

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, d) != HAL_OK) return;
    
    // GM6020 gimbal feedback (CAN1 only)
    if (manager->filter_bank == CAN1_FILTER_BANK && rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x205 && rx.StdId<=0x208) {
        uint8_t gid = (uint8_t)(rx.StdId - 0x204);
        if (gid >= 1 && gid <= 7) {
            uint16_t angle_raw = (uint16_t)((d[0]<<8) | d[1]);
            int16_t  speed_rpm = (int16_t)((d[2]<<8) | d[3]);
            gm6020_on_feedback(gid, angle_raw, speed_rpm);
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
