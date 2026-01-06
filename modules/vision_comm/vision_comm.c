/**
 * @file vision_comm.c
 * @brief Vision communication module - using USB CDC
 */

#include "vision_comm.h"
#include "seasky_protocol.h"
#include "message_center.h"
#include "gyro_data.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;

// USB CDC receive processing buffer
static uint8_t cdc_recv_processing[VISION_RECV_SIZE];

// Vision send frequency control (100Hz = 10ms interval)
#define VISION_SEND_INTERVAL_MS 10
static uint32_t last_send_time = 0;

/**
 * @brief USB CDC receive callback function (called from CDC_Receive_FS)
 */
void VisionComm_RxCallback(uint8_t *buf, uint32_t len)
{
    uint16_t flag_register;

    // Copy data to processing buffer
    if (len >= 18 && len <= VISION_RECV_SIZE) {
        memcpy(cdc_recv_processing, buf, len);

        // Parse protocol
        uint16_t cmd_id = get_protocol_info(cdc_recv_processing,
                                            &flag_register,
                                            (uint8_t *)&recv_data.pitch);

        if (cmd_id == 0x0001) {
            // Parse flags
            recv_data.fire_mode = (Fire_Mode_e)(flag_register & 0x03);
            recv_data.target_state = (Target_State_e)((flag_register >> 2) & 0x03);
            recv_data.target_type = (Target_Type_e)((flag_register >> 4) & 0x0F);

            // Mark data as updated
            recv_data.updated = 1;

            // Publish vision data to message center
            (void)MsgCenter_Publish(TOPIC_VISION_DATA, &recv_data, sizeof(Vision_Recv_s));
        }
    }

    // Note: USB CDC reception is handled automatically by the USB stack
    // No need to manually restart reception like with UART
}

/**
 * @brief Start USB CDC reception for vision communication
 * @note USB CDC reception is handled automatically by the USB stack.
 *       This function is kept for compatibility but does nothing.
 */
void VisionComm_StartReceive(void)
{
    // USB CDC reception is handled automatically by the USB stack
    // No action needed - the CDC_Receive_FS callback will be called automatically
}

/**
 * @brief IMU update callback - sends vision data at controlled frequency
 */
static void on_imu_update(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    
    if (ev->size == sizeof(SensorData)) {
        const SensorData *sensor_data = (const SensorData *)ev->data;
        uint32_t current_time = HAL_GetTick();
        
        // Control send frequency (100Hz)
        if (current_time - last_send_time >= VISION_SEND_INTERVAL_MS) {
            // Set attitude data from gimbal IMU
            VisionComm_SetAltitude(sensor_data->g_gz, sensor_data->g_gx, sensor_data->g_gy);
            VisionComm_SetFlag(COLOR_BLUE, VISION_MODE_AIM, SMALL_AMU_15);
            VisionComm_Send();
            
            last_send_time = current_time;
        }
    }
}

/**
 * @brief Initialize vision communication
 */
Vision_Recv_s *VisionComm_Init(void)
{
    // Clear receive and send data
    memset(&recv_data, 0, sizeof(Vision_Recv_s));
    memset(&send_data, 0, sizeof(Vision_Send_s));
    memset(cdc_recv_processing, 0, sizeof(cdc_recv_processing));

    // Subscribe to IMU updates for sending vision data
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

    // Note: USB CDC reception is handled automatically by the USB stack
    // No need to manually start reception

    return &recv_data;
}

/**
 * @brief Set flags
 */
void VisionComm_SetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

/**
 * @brief Set attitude data
 */
void VisionComm_SetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief Send vision data
 */
void VisionComm_Send(void)
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;

    // Set flag register (example)
    flag_register = 30 << 8 | 0b00000001;

    // Convert data to seasky protocol packet
    get_protocol_send_data(0x02,                // cmd_id = 0x0002 (attitude data)
                          flag_register,
                          &send_data.yaw,       // 3 floats: yaw, pitch, roll
                          3,
                          send_buff,
                          &tx_len);

    // Send via USB CDC
    CDC_Transmit_FS(send_buff, tx_len);
}

/**
 * @brief Get receive data
 */
Vision_Recv_s *VisionComm_GetData(void)
{
    return &recv_data;
}

