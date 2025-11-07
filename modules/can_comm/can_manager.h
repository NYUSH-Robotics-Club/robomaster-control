#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "can.h"

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
    // Debug counters
    uint32_t tx_ok;
    uint32_t tx_err;
    uint32_t rx_frames;
    uint32_t last_rx_id;
    uint32_t last_tx_time;
    uint32_t last_rx_time;
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
HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager, CAN_Channel_t channel, CAN_HandleTypeDef *hcan);

/**
 * @brief Start CAN communication
 * @param manager CAN manager pointer
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_Start(CAN_Manager_t *manager);

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
 * @brief Send 4 motor currents in one CAN frame to StdId 0x200/0x1FF
 * @param hcan CAN handle
 * @param std_id Standard ID (0x200 for motors 1-4 on CAN1, 0x1FF for 5-8 on CAN2)
 * @param i1 Current for slot 1 (motor 1 or 5)
 * @param i2 Current for slot 2 (motor 2 or 6)
 * @param i3 Current for slot 3 (motor 3 or 7)
 * @param i4 Current for slot 4 (motor 4 or 8)
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(CAN_HandleTypeDef *hcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4);

/**
 * @brief Send GM6020 current by motor id (1..7) using StdId 0x1FF/0x2FF layout
 * @param hcan CAN handle (typically hcan1)
 * @param motor_id GM6020 id in 1..7
 * @param current current value, will be clamped to [-30000,30000]
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_SendGM6020Current(CAN_HandleTypeDef *hcan, uint8_t motor_id, int16_t current);

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

// Debug accessor helpers
uint32_t CAN_Manager_GetTxOk(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetTxErr(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetRxFrames(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastRxId(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastTxTime(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastRxTime(const CAN_Manager_t *m);

#endif // CAN_MANAGER_H
