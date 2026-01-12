#ifndef RADAR_COMM_H
#define RADAR_COMM_H

#include "main.h"
#include <stdint.h>

#pragma pack(1)

// Radar receive data structure
typedef struct {
    float vx;        // m/s
    float vy;        // m/s
    float wz;        // rad/s
    uint32_t ts_ms;  // HAL_GetTick() timestamp when frame was parsed
    uint8_t valid;   // 1 = valid, 0 = timeout or error
} Radar_Recv_s;

#// Frame format: [SYNC1=0xA5] [SYNC2=0x5A] [vx:4B] [vy:4B] [wz:4B] [CRC8:1B]
#// Total: 15 bytes (2 + 12 + 1)
#define RADAR_FRAME_SYNC1 0xA5u
#define RADAR_FRAME_SYNC2 0x5Au
#define RADAR_FRAME_SIZE 15u
#define RADAR_FRAME_DATA_SIZE 12u  // 3 floats
#define RADAR_RX_BUFFER_SIZE 128u  // Circular buffer for USB CDC data

#pragma pack()

/** Initialize radar comm (USB CDC). Returns pointer to receive struct. */
Radar_Recv_s *RadarComm_Init(void);

/** Start reception (no-op for USB CDC). */
void RadarComm_StartReceive(void);

/** USB CDC receive callback (called from CDC receive wrapper) - writes to ring buffer ISR-safe */
void RadarComm_RxCallback(uint8_t *buf, uint32_t len);

/** Process pending data in ring buffer. Call periodically in main task. */
void RadarComm_Task(void);

/** Get pointer to latest data */
Radar_Recv_s *RadarComm_GetData(void);

#endif // RADAR_COMM_H
