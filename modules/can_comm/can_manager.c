#include "can_manager.h"
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

HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager, CAN_Channel_t channel, CAN_HandleTypeDef *hcan)
{
    if (manager == NULL || hcan == NULL) return HAL_ERROR;
    memset(manager, 0, sizeof(CAN_Manager_t));
    manager->hcan = hcan;
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

    // Process M3508/M2006 motor feedback (0x201-0x207): chassis (0x201-0x204) and shooter (0x205-0x207)
    // These motors have full feedback: angle, speed, current, temp
    // 0x205-0x207 are M3508 shooter motors
    if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x201 && rx.StdId<=0x207) {
        uint8_t mid = rx.StdId - 0x201;
        // Map CAN IDs to motor IDs: 0x201-0x204 -> 0-3 (chassis), 0x205-0x207 -> 4,5,7 (shooter)
        if (rx.StdId == 0x207) {
            mid = 7;  // Map 0x207 to motor_id 7 (shooter2), not 6
        }
        uint16_t angle = (d[0]<<8) | d[1];
        int16_t  speed = (int16_t)((d[2]<<8) | d[3]);
        int16_t  current = (int16_t)((d[4]<<8) | d[5]);
        uint8_t  temp = d[6];
        MotorFeedbackEvent ev = { mid, angle, speed, current, temp, current_tick };
        (void)MsgCenter_Publish(TOPIC_MOTOR_FEEDBACK, &ev, sizeof(ev));
    }
    
    // Process GM6020 motor feedback (0x20A-0x20B): gimbal motors (Yaw=6, Pitch=7)
    // GM6020 feedback format: angle (d[0-1]), speed (d[2-3]), current (d[4-5]), temp (d[6])
    // But we only extract angle and speed for gimbal control
    // Note: GM6020 CAN ID = 0x204 + motor_id, so motor_id 6->0x20A, 7->0x20B
    if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x20A && rx.StdId<=0x20B) {
        uint8_t gid = (uint8_t)(rx.StdId - 0x204);  // GM6020 motor_id: 0x20A->6, 0x20B->7
        if (gid >= 6 && gid <= 7) {
            uint16_t angle_raw = (uint16_t)((d[0]<<8) | d[1]);
            int16_t  speed_rpm = (int16_t)((d[2]<<8) | d[3]);
            int16_t  current = (int16_t)((d[4]<<8) | d[5]);
            GM6020FeedbackEvent gev = { gid, angle_raw, speed_rpm, current_tick, current };
            (void)MsgCenter_Publish(TOPIC_GM6020_FEEDBACK, &gev, sizeof(gev));
        }
    }
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

uint32_t CAN_Manager_GetTxOk(const CAN_Manager_t *m){ return m?m->tx_ok:0; }
uint32_t CAN_Manager_GetTxErr(const CAN_Manager_t *m){ return m?m->tx_err:0; }
uint32_t CAN_Manager_GetRxFrames(const CAN_Manager_t *m){ return m?m->rx_frames:0; }
uint32_t CAN_Manager_GetLastRxId(const CAN_Manager_t *m){ return m?m->last_rx_id:0; }
uint32_t CAN_Manager_GetLastTxTime(const CAN_Manager_t *m){ return m?m->last_tx_time:0; }
uint32_t CAN_Manager_GetLastRxTime(const CAN_Manager_t *m){ return m?m->last_rx_time:0; }
