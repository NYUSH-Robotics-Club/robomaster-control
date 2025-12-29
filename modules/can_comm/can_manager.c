#include "can_manager.h"
#include "motor_registry.h"
#include "robot_config.h"
#include "message_center.h"
#include "can_comm.h"
#include <string.h>
#include "printing.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CAN1_FILTER_BANK    0
#define CAN2_FILTER_BANK    14
#define CAN_FILTER_MODE     CAN_FILTERMODE_IDMASK
#define CAN_FILTER_SCALE    CAN_FILTERSCALE_32BIT
#define CAN_FIFO_ASSIGNMENT CAN_FILTER_FIFO0
#define CAN_IT_TYPE         CAN_IT_RX_FIFO0_MSG_PENDING

HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager,
                                  CAN_Channel_t channel,
                                  CAN_HandleTypeDef *hcan,
                                  const RobotConfig_t *robot_config,
                                  MotorRegistry_t *registry_storage)
{
    if (manager == NULL || hcan == NULL || robot_config == NULL || registry_storage == NULL) {
        return HAL_ERROR;
    }

    // Clear manager structure
    memset(manager, 0, sizeof(CAN_Manager_t));
    manager->hcan = hcan;
    manager->channel = channel;

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

    // Initialize motor registry
    manager->registry = registry_storage;
    MotorRegistry_Init(manager->registry, robot_config, channel);

    // Initialize TX frame buffers
    manager->tx_frames[0].std_id = 0x200;
    manager->tx_frames[1].std_id = 0x1FF;
    manager->tx_frames[2].std_id = 0x2FF;

    manager->initialized = 1;
    return HAL_OK;
}

HAL_StatusTypeDef CAN_Manager_Start(CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return HAL_ERROR;
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
    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(manager->hcan, &filter);
    if (status != HAL_OK) return status;
    status = HAL_CAN_Start(manager->hcan);
    if (status != HAL_OK) return status;
    status = HAL_CAN_ActivateNotification(manager->hcan, CAN_IT_TYPE);
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
    manager->rx_frames++;
    manager->last_rx_id = rx.StdId;
    manager->last_rx_time = current_tick;

    // Publish raw CAN RX frame to message center
    CanRxFrame f = { (uint16_t)rx.StdId, rx.DLC, {0} };
    if (rx.DLC <= 8) { memcpy(f.data, d, rx.DLC); }
    (void)MsgCenter_Publish(TOPIC_CAN_RX, &f, sizeof(f));

    // ========== NEW: Dynamic motor feedback processing using registry ==========
    // Only process standard ID frames with 8 bytes of data
    if (rx.IDE != CAN_ID_STD || rx.DLC != 8) {
        return;
    }

    // Look up motor configuration by RX ID
    const MotorConfig_t *motor = MotorRegistry_FindByRxId(manager->registry, rx.StdId);
    if (motor == NULL) {
        // Not a registered motor - ignore
        return;
    }

    // Parse feedback based on motor type
    if (motor->type == MOTOR_TYPE_M3508 || motor->type == MOTOR_TYPE_M2006) {
        // M3508/M2006 feedback format:
        // Bytes [0-1]: Encoder angle (0-8191)
        // Bytes [2-3]: Speed (RPM, signed)
        // Bytes [4-5]: Current (signed)
        // Byte  [6]:   Temperature (degrees C)
        uint16_t angle   = (uint16_t)((d[0] << 8) | d[1]);
        int16_t  speed   = (int16_t)((d[2] << 8) | d[3]);
        int16_t  current = (int16_t)((d[4] << 8) | d[5]);
        uint8_t  temp    = d[6];

        // Publish to message center
        MotorFeedbackEvent ev = {
            .id = motor->motor_id,
            .angle = angle,
            .speed = speed,
            .current = current,
            .temp = temp,
            .tick_ms = current_tick
        };
        (void)MsgCenter_Publish(TOPIC_MOTOR_FEEDBACK, &ev, sizeof(ev));
    }
    else if (motor->type == MOTOR_TYPE_GM6020) {
        // GM6020 feedback format:
        // Bytes [0-1]: Encoder angle (0-8191)
        // Bytes [2-3]: Speed (RPM, signed)
        // Bytes [4-5]: Current (signed)
        // Byte  [6]:   Temperature (degrees C)
        uint16_t angle_raw = (uint16_t)((d[0] << 8) | d[1]);
        int16_t  speed_rpm = (int16_t)((d[2] << 8) | d[3]);
        int16_t  current   = (int16_t)((d[4] << 8) | d[5]);

        // Publish to message center
        GM6020FeedbackEvent gev = {
            .id = motor->motor_id,
            .angle = angle_raw,
            .speed = speed_rpm,
            .tick_ms = current_tick,
            .current = current
        };
        (void)MsgCenter_Publish(TOPIC_GM6020_FEEDBACK, &gev, sizeof(gev));
    }
    // ========== END: Dynamic feedback processing ==========
}

extern CAN_Manager_t can1_manager;
extern CAN_Manager_t can2_manager;

void CAN_Manager_GlobalCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == CAN_Manager_GetHandle(&can1_manager)) {
        CAN_Manager_ProcessCallback(&can1_manager, hcan);
    } else if (hcan == CAN_Manager_GetHandle(&can2_manager)) {
        CAN_Manager_ProcessCallback(&can2_manager, hcan);
    }
}

HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(CAN_HandleTypeDef *hcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    if (hcan == NULL) return HAL_ERROR;
    static uint32_t last_tx_tick = 0;   
    uint32_t now = HAL_GetTick();
    if (now - last_tx_tick < 4) {
        return HAL_OK;
    }
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
    HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
    CAN_Manager_t *m = NULL;
    if (hcan == can1_manager.hcan) m = &can1_manager; else if (hcan == can2_manager.hcan) m = &can2_manager;
    if (m) {
        if (st == HAL_OK) m->tx_ok++; else m->tx_err++;
        m->last_tx_time = HAL_GetTick();
    }
    return st;
}

HAL_StatusTypeDef CAN_Manager_SendGM6020Current(CAN_HandleTypeDef *hcan, uint8_t motor_id, int16_t current)
{
    if (hcan == NULL) return HAL_ERROR;
    static uint32_t last_tx_tick = 0;   
    uint32_t now = HAL_GetTick();
    if (now - last_tx_tick < 1.5) {
        return HAL_OK;
    }
    if (motor_id < 1 || motor_id > 7) return HAL_ERROR;
    if (current >  25000) current =  25000;
    if (current < -25000) current = -25000;
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
    HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
    CAN_Manager_t *m = NULL;
    if (hcan == can1_manager.hcan) m = &can1_manager; else if (hcan == can2_manager.hcan) m = &can2_manager;
    if (m) {
        if (st == HAL_OK) m->tx_ok++; else m->tx_err++;
        m->last_tx_time = HAL_GetTick();
    }
    return st;
}

/**
 * @brief Send motor current by motor ID (new configurable API)
 */
HAL_StatusTypeDef CAN_Manager_SendMotorCurrent(CAN_Manager_t *manager,
                                              uint8_t motor_id,
                                              int16_t current)
{
    if (manager == NULL || !manager->initialized || manager->registry == NULL) {
        return HAL_ERROR;
    }

    // Find motor configuration
    const MotorConfig_t *motor = MotorRegistry_FindByMotorId(manager->registry, motor_id);
    if (motor == NULL) {
        return HAL_ERROR;  // Motor not found in this CAN channel
    }

    // Clamp current based on motor type
    if (motor->type == MOTOR_TYPE_GM6020) {
        if (current >  25000) current =  25000;
        if (current < -25000) current = -25000;
    } else {  // M3508/M2006
        if (current >  16384) current =  16384;
        if (current < -16384) current = -16384;
    }

    // Find appropriate TX frame
    CANTxFrame_t *tx_frame = NULL;
    for (uint8_t i = 0; i < CAN_TX_FRAME_COUNT; i++) {
        if (manager->tx_frames[i].std_id == motor->can_tx_id) {
            tx_frame = &manager->tx_frames[i];
            break;
        }
    }

    if (tx_frame == NULL) {
        return HAL_ERROR;  // Unsupported TX ID
    }

    // Aggregate current into appropriate slot
    if (motor->tx_slot < 4) {
        tx_frame->currents[motor->tx_slot] = current;
        tx_frame->pending = 1;  // Mark frame as pending
    } else {
        return HAL_ERROR;  // Invalid slot
    }

    return HAL_OK;
}

/**
 * @brief Flush all pending TX frames
 */
HAL_StatusTypeDef CAN_Manager_FlushTx(CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef result = HAL_OK;

    // Send all pending frames
    for (uint8_t i = 0; i < CAN_TX_FRAME_COUNT; i++) {
        CANTxFrame_t *tx_frame = &manager->tx_frames[i];

        if (tx_frame->pending) {
            // Prepare CAN message
            CAN_TxHeaderTypeDef tx_header = {0};
            uint8_t data[8] = {0};
            uint32_t mailbox;

            tx_header.StdId = tx_frame->std_id;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = 8;

            // Pack currents into data buffer (big-endian)
            for (uint8_t slot = 0; slot < 4; slot++) {
                data[slot * 2 + 0] = (uint8_t)((tx_frame->currents[slot] >> 8) & 0xFF);
                data[slot * 2 + 1] = (uint8_t)(tx_frame->currents[slot] & 0xFF);
            }

            // Send CAN message
            HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(manager->hcan, &tx_header, data, &mailbox);
            if (status == HAL_OK) {
                manager->tx_ok++;
            } else {
                manager->tx_err++;
                result = HAL_ERROR;  // Mark as error but continue sending other frames
            }
            manager->last_tx_time = HAL_GetTick();

            // Clear frame for next cycle
            memset(tx_frame->currents, 0, sizeof(tx_frame->currents));
            tx_frame->pending = 0;
        }
    }

    return result;
}

/**
 * @brief Get CAN manager from handle (for reverse lookup)
 */
CAN_Manager_t* CAN_Manager_FromHandle(CAN_HandleTypeDef *hcan)
{
    extern CAN_Manager_t can1_manager;
    extern CAN_Manager_t can2_manager;

    if (hcan == can1_manager.hcan) {
        return &can1_manager;
    } else if (hcan == can2_manager.hcan) {
        return &can2_manager;
    }
    return NULL;
}

uint32_t CAN_Manager_GetTxOk(const CAN_Manager_t *m){ return m?m->tx_ok:0; }
uint32_t CAN_Manager_GetTxErr(const CAN_Manager_t *m){ return m?m->tx_err:0; }
uint32_t CAN_Manager_GetRxFrames(const CAN_Manager_t *m){ return m?m->rx_frames:0; }
uint32_t CAN_Manager_GetLastRxId(const CAN_Manager_t *m){ return m?m->last_rx_id:0; }
uint32_t CAN_Manager_GetLastTxTime(const CAN_Manager_t *m){ return m?m->last_tx_time:0; }
uint32_t CAN_Manager_GetLastRxTime(const CAN_Manager_t *m){ return m?m->last_rx_time:0; }
