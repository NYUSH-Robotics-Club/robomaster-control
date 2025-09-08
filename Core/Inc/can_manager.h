#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "can.h"
#include "chassis_controller.h"
#include "shooter_controller.h"
#include "gm6020.h"

// CAN channel enumeration
typedef enum {
    CAN_CHANNEL_1 = 0,
    CAN_CHANNEL_2 = 1,
    CAN_CHANNEL_COUNT
} CAN_Channel_t;

// CAN manager structure
typedef struct {
    CAN_HandleTypeDef *hcan;
    uint32_t filter_bank;
    uint8_t initialized;
    ChassisController *chassis_controller;
    ShooterController *shooter_controller;
} CAN_Manager_t;

/**
 * @brief Initialize CAN manager
 * @param manager CAN manager pointer
 * @param channel CAN channel (CAN_CHANNEL_1 or CAN_CHANNEL_2)
 * @param hcan CAN handle pointer
 * @param chassis_controller Chassis controller pointer
 * @param shooter_controller Shooter controller pointer
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager, CAN_Channel_t channel, CAN_HandleTypeDef *hcan, 
                                   ChassisController *chassis_controller, ShooterController *shooter_controller);

/**
 * @brief Start CAN communication
 * @param manager CAN manager pointer
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_Start(CAN_Manager_t *manager);

/**
 * @brief Stop CAN communication
 * @param manager CAN manager pointer
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_Stop(CAN_Manager_t *manager);

/**
 * @brief Check if CAN is initialized
 * @param manager CAN manager pointer
 * @return true if initialized
 */
bool CAN_Manager_IsInitialized(const CAN_Manager_t *manager);

/**
 * @brief Get CAN handle
 * @param manager CAN manager pointer
 * @return CAN handle pointer
 */
CAN_HandleTypeDef* CAN_Manager_GetHandle(const CAN_Manager_t *manager);

/**
 * @brief Process CAN receive callback
 * @param manager CAN manager pointer
 * @param hcan CAN handle that triggered the callback
 */
void CAN_Manager_ProcessCallback(CAN_Manager_t *manager, CAN_HandleTypeDef *hcan);

/**
 * @brief Global CAN callback function (to be called from HAL_CAN_RxFifo0MsgPendingCallback)
 * @param hcan CAN handle that triggered the callback
 */
void CAN_Manager_GlobalCallback(CAN_HandleTypeDef *hcan);

#endif // CAN_MANAGER_H
