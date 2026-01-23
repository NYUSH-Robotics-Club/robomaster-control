/**
 * @file radar_comm.c
 * @brief Radar communication module (USB CDC) with framing, CRC, and circular buffer
 * 
 * Protocol:
 *  [SYNC1=0xA5] [SYNC2=0x5A] [vx:4B] [vy:4B] [wz:4B] [CRC8:1B]
 *  Total: 15 bytes (2 + 12 + 1)
 * 
 * Framing & Safety:
 *  - Circular ring buffer to handle USB packet fragmentation
 *  - Sync byte detection to recover from data corruption
 *  - CRC8 checksum to detect transmission errors (EMI)
 *  - Main task processes frames (not ISR) to avoid overload
 */

#include "radar_comm.h"
#include "message_center.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include "logger.h"
#include "printing.h"
#include <string.h>

// ============================================
// CRC8 Checksum
// ============================================
static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x07;  // Polynomial: x^8 + x^2 + x + 1
        } else {
            crc = (crc << 1);
        }
    }
    return crc;
}

static uint8_t crc8_calc(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc = crc8_update(crc, data[i]);
    }
    return crc;
}

// ============================================
// Circular Ring Buffer
// ============================================
typedef struct {
    uint8_t buffer[RADAR_RX_BUFFER_SIZE];
    volatile uint32_t write_idx;  // Updated in ISR
    uint32_t read_idx;            // Updated in task
} RingBuffer_t;

static RingBuffer_t ring_buffer;
static Radar_Recv_s recv_data;
static uint32_t last_valid_time_ms = 0u;
static uint32_t frame_count = 0u;      // Total frames received
static uint32_t crc_error_count = 0u;  // CRC errors
static uint32_t last_debug_time_ms = 0u;

#define RADAR_DATA_TIMEOUT_MS 1000u  // Mark invalid if no new data for 1000ms
#define RADAR_DEBUG_INTERVAL_MS 500u  // Print debug info every 500ms

/**
 * @brief Write data to ring buffer (ISR-safe, only write pointer updated atomically)
 */
static void ring_buffer_write(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0) return;
    
    for (uint32_t i = 0; i < len; i++) {
        // If buffer full, overwrite oldest data (lose old data to prevent deadlock)
        uint32_t next_idx = (ring_buffer.write_idx + 1) % RADAR_RX_BUFFER_SIZE;
        ring_buffer.buffer[ring_buffer.write_idx] = data[i];
        ring_buffer.write_idx = next_idx;
    }
}

/**
 * @brief Read one byte from ring buffer
 */
static int ring_buffer_read_one(uint8_t *byte)
{
    if (ring_buffer.read_idx == ring_buffer.write_idx) {
        return -1;  // Empty
    }
    *byte = ring_buffer.buffer[ring_buffer.read_idx];
    ring_buffer.read_idx = (ring_buffer.read_idx + 1) % RADAR_RX_BUFFER_SIZE;
    return 0;
}

/**
 * @brief Peek N bytes from ring buffer without consuming
 */
static int ring_buffer_peek(uint32_t offset, uint8_t *byte)
{
    if (offset >= RADAR_RX_BUFFER_SIZE) return -1;
    uint32_t idx = (ring_buffer.read_idx + offset) % RADAR_RX_BUFFER_SIZE;
    if (idx == ring_buffer.write_idx && offset > 0) return -1;  // Would overflow
    *byte = ring_buffer.buffer[idx];
    return 0;
}

/**
 * @brief Skip N bytes in ring buffer
 */
static void ring_buffer_skip(uint32_t count)
{
    ring_buffer.read_idx = (ring_buffer.read_idx + count) % RADAR_RX_BUFFER_SIZE;
}

/**
 * @brief USB CDC receive callback (called from CDC_Receive_FS)
 * 
 * Strategy: Just write bytes to ring buffer. Don't parse here (ISR context).
 * Main task will handle frame parsing and CRC verification.
 */
void RadarComm_RxCallback(uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return;
    ring_buffer_write(buf, len);
    // Debug: print raw bytes received (rate limited in main task, not here)
}

/**
 * @brief Process frames in the ring buffer
 * 
 * This function should be called periodically (e.g., 200Hz in main task).
 * It searches for sync bytes, validates frames with CRC, and updates recv_data.
 */
void RadarComm_Task(void)
{
    uint8_t byte = 0;
    
    // Search for frame sync (0xA5 0x5A)
    while (ring_buffer_read_one(&byte) == 0) {
        if (byte != RADAR_FRAME_SYNC1) {
            continue;  // Not sync byte 1, keep searching
        }
        
        // Found potential SYNC1, check SYNC2
        if (ring_buffer_peek(0, &byte) != 0) {
            break;  // Not enough data yet
        }
        
        if (byte != RADAR_FRAME_SYNC2) {
            continue;  // False alarm, keep searching
        }
        
        // Found SYNC1 SYNC2, now try to read full frame
        if (ring_buffer.read_idx == ring_buffer.write_idx) {
            // Undo the SYNC1 read to retry next time
            ring_buffer.read_idx = (ring_buffer.read_idx - 1 + RADAR_RX_BUFFER_SIZE) % RADAR_RX_BUFFER_SIZE;
            break;  // Not enough data
        }
        
        // Check if we have enough bytes for full frame (14 total)
        uint32_t available = ring_buffer.write_idx >= ring_buffer.read_idx
            ? (ring_buffer.write_idx - ring_buffer.read_idx + 1)  // +1 for already consumed SYNC1
            : (RADAR_RX_BUFFER_SIZE - ring_buffer.read_idx + ring_buffer.write_idx + 1);
        
        if (available < RADAR_FRAME_SIZE) {
            // Put SYNC1 back
            ring_buffer.read_idx = (ring_buffer.read_idx - 1 + RADAR_RX_BUFFER_SIZE) % RADAR_RX_BUFFER_SIZE;
            break;  // Wait for more data
        }
        
        // Read full frame
        uint8_t frame[RADAR_FRAME_SIZE];
        frame[0] = RADAR_FRAME_SYNC1;
        frame[1] = RADAR_FRAME_SYNC2;
        
        // Skip SYNC2 that we already peeked
        ring_buffer_skip(1);
        
        // Read remaining 12 bytes (data + CRC)
        for (int i = 2; i < RADAR_FRAME_SIZE; i++) {
            if (ring_buffer_read_one(&frame[i]) != 0) {
                // Should not happen given available check, but be safe
                return;
            }
        }
        
        // Validate CRC (CRC8 of first 14 bytes)
        uint8_t crc_calc = crc8_calc(frame, RADAR_FRAME_SIZE - 1);
        uint8_t crc_recv = frame[RADAR_FRAME_SIZE - 1];
        
        if (crc_calc != crc_recv) {
            // CRC mismatch, frame corrupted
            crc_error_count++;
            LOG_DEBUG(LOG_TAG_DEBUG, "RADAR CRC ERR: calc=0x%02X recv=0x%02X", crc_calc, crc_recv);
            continue;  // Discard and search for next frame
        }
        
        // Frame valid! Extract floats
        float f_vx = 0.0f, f_vy = 0.0f, f_wz = 0.0f;
        memcpy(&f_vx, &frame[2], sizeof(float));
        memcpy(&f_vy, &frame[6], sizeof(float));
        memcpy(&f_wz, &frame[10], sizeof(float));
        
        // Update recv_data
        recv_data.vx = f_vx;
        recv_data.vy = f_vy;
        recv_data.wz = f_wz;
        recv_data.ts_ms = HAL_GetTick();
        recv_data.valid = 1;
        last_valid_time_ms = recv_data.ts_ms;
        frame_count++;
        
        // Publish to message center
        (void)MsgCenter_Publish(TOPIC_RADAR_CMD, &recv_data, sizeof(Radar_Recv_s));
        
        // Debug output (rate limited)
        uint32_t now = HAL_GetTick();
        if (now - last_debug_time_ms >= RADAR_DEBUG_INTERVAL_MS) {
            LOG_INFO(LOG_TAG_DEBUG, "RADAR OK: vx=%.3f vy=%.3f wz=%.3f frames=%lu errors=%lu", 
                     f_vx, f_vy, f_wz, (unsigned long)frame_count, (unsigned long)crc_error_count);
            last_debug_time_ms = now;
        }
        
        // Found and processed one frame, continue searching for next
    }
    
    // Timeout check: if no valid data for > RADAR_DATA_TIMEOUT_MS, mark as invalid
    uint32_t now = HAL_GetTick();
    if (recv_data.valid && (now - last_valid_time_ms > RADAR_DATA_TIMEOUT_MS)) {
        recv_data.valid = 0;
        LOG_DEBUG(LOG_TAG_DEBUG, "RADAR TIMEOUT: no data for %lums", (unsigned long)(now - last_valid_time_ms));
    }
    
    // Periodic status report (even when no frames)
    static uint32_t last_status_time_ms = 0u;
    if (now - last_status_time_ms >= 2000u) {  // Every 2 seconds
        uint32_t buffer_used = (ring_buffer.write_idx >= ring_buffer.read_idx) 
            ? (ring_buffer.write_idx - ring_buffer.read_idx)
            : (RADAR_RX_BUFFER_SIZE - ring_buffer.read_idx + ring_buffer.write_idx);
        LOG_INFO(LOG_TAG_DEBUG, "RADAR STATUS: buf_used=%lu/%lu frames=%lu errors=%lu valid=%d", 
                 (unsigned long)buffer_used, (unsigned long)RADAR_RX_BUFFER_SIZE,
                 (unsigned long)frame_count, (unsigned long)crc_error_count, recv_data.valid);
        last_status_time_ms = now;
    }
}

void RadarComm_StartReceive(void)
{
    // USB CDC reception is handled by the USB stack; no manual start needed
}

Radar_Recv_s *RadarComm_Init(void)
{
    memset(&ring_buffer, 0, sizeof(ring_buffer));
    memset(&recv_data, 0, sizeof(recv_data));
    return &recv_data;
}

Radar_Recv_s *RadarComm_GetData(void)
{
    return &recv_data;
}
