/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wt61c.c
  * @brief   WT61C-TTL IMU sensor driver implementation
  ******************************************************************************
  * @attention
  *
  * Driver for WitMotion WT61C-TTL 6-axis IMU sensor
  * Protocol: 11-byte frames starting with 0x55
  * Frame format: [0x55] [ID] [8 data bytes] [checksum]
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "wt61c.h"
#include <string.h>

#ifndef GRAVITY
#define GRAVITY 9.80665f
#endif

/* Private variables */
static UART_HandleTypeDef *s_huart = NULL;
static WT61C_Data s_data;

/* Parser state machine */
typedef enum {
    ST_WAIT_55 = 0,  // Waiting for frame header 0x55
    ST_WAIT_ID,      // Waiting for frame ID
    ST_PAYLOAD       // Receiving payload bytes
} ParseState;

static struct {
    ParseState st;
    uint8_t buf[11];
    uint8_t idx;
} s_parser;

/* Private function prototypes */
static int16_t to_s16(uint8_t lo, uint8_t hi);
static void parse_one_frame(const uint8_t *f);
static HAL_StatusTypeDef wit_cmd5(UART_HandleTypeDef *huart, uint8_t reg, uint16_t val);

/**
 * @brief Convert two bytes to signed 16-bit integer (little-endian)
 */
static int16_t to_s16(uint8_t lo, uint8_t hi) {
    return (int16_t)((hi << 8) | lo);
}

/**
 * @brief Parse one complete 11-byte frame from WT61C
 * @param f Pointer to 11-byte frame buffer
 */
static void parse_one_frame(const uint8_t *f) {
    // Frame format: f[0]=0x55, f[1]=ID, f[2..9]=payload, f[10]=checksum
    uint8_t sum = 0;
    for (int i = 0; i < 10; ++i) {
        sum += f[i];
    }

    // Verify checksum
    if (sum != f[10]) {
        return;
    }

    const uint8_t id = f[1];
    const uint8_t *p = &f[2];

    switch (id) {
    case 0x51: { // Acceleration + Temperature
        int16_t ax_raw = to_s16(p[0], p[1]);
        int16_t ay_raw = to_s16(p[2], p[3]);
        int16_t az_raw = to_s16(p[4], p[5]);
        int16_t temp_raw = to_s16(p[6], p[7]);

        // Convert to m/s^2 (±16g range)
        s_data.ax = (ax_raw / 32768.0f) * 16.0f * GRAVITY;
        s_data.ay = (ay_raw / 32768.0f) * 16.0f * GRAVITY;
        s_data.az = (az_raw / 32768.0f) * 16.0f * GRAVITY;

        // Temperature in Celsius
        s_data.temperature = temp_raw / 100.0f;
        break;
    }

    case 0x52: { // Angular velocity (Gyroscope)
        int16_t gx_raw = to_s16(p[0], p[1]);
        int16_t gy_raw = to_s16(p[2], p[3]);
        int16_t gz_raw = to_s16(p[4], p[5]);

        // Convert to deg/s (±2000 deg/s range)
        s_data.gx = (gx_raw / 32768.0f) * 2000.0f;
        s_data.gy = (gy_raw / 32768.0f) * 2000.0f;
        s_data.gz = (gz_raw / 32768.0f) * 2000.0f;
        break;
    }

    case 0x53: { // Euler angles
        int16_t roll_raw = to_s16(p[0], p[1]);
        int16_t pitch_raw = to_s16(p[2], p[3]);
        int16_t yaw_raw = to_s16(p[4], p[5]);

        // Convert to degrees (±180 degrees)
        s_data.roll = (roll_raw / 32768.0f) * 180.0f;
        s_data.pitch = (pitch_raw / 32768.0f) * 180.0f;
        s_data.yaw = (yaw_raw / 32768.0f) * 180.0f;
        break;
    }

    default:
        // Unknown frame ID, ignore
        break;
    }

    // Update timestamp
    s_data.last_update_ms = HAL_GetTick();
}

/**
 * @brief Process incoming bytes from UART
 */
void WT61C_ProcessBytes(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        uint8_t b = data[i];

        switch (s_parser.st) {
        case ST_WAIT_55:
            if (b == 0x55) {
                s_parser.buf[0] = b;
                s_parser.idx = 1;
                s_parser.st = ST_WAIT_ID;
            }
            break;

        case ST_WAIT_ID:
            s_parser.buf[1] = b;
            s_parser.idx = 2;
            s_parser.st = ST_PAYLOAD;
            break;

        case ST_PAYLOAD:
            s_parser.buf[s_parser.idx++] = b;
            if (s_parser.idx >= 11) {
                parse_one_frame(s_parser.buf);
                s_parser.st = ST_WAIT_55;
                s_parser.idx = 0;
                // Notify application only when a complete frame is parsed
                WT61C_OnNewData(&s_data);
            }
            break;
        }
    }
}

/**
 * @brief Initialize WT61C driver
 */
void WT61C_Init(UART_HandleTypeDef *huart) {
    s_huart = huart;
    memset(&s_data, 0, sizeof(s_data));
    s_parser.st = ST_WAIT_55;
    s_parser.idx = 0;
}

/**
 * @brief Get pointer to latest sensor data
 */
const WT61C_Data* WT61C_GetData(void) {
    return &s_data;
}

/* Configuration functions */

/**
 * @brief Send 5-byte configuration command to WT61C
 */
static HAL_StatusTypeDef wit_cmd5(UART_HandleTypeDef *huart, uint8_t reg, uint16_t val) {
    uint8_t pkt[5] = {
        0xFF,
        0xAA,
        reg,
        (uint8_t)(val & 0xFF),
        (uint8_t)(val >> 8)
    };
    return HAL_UART_Transmit(huart, pkt, sizeof(pkt), 20);
}

/**
 * @brief Unlock sensor for configuration
 */
HAL_StatusTypeDef WT61C_Unlock(UART_HandleTypeDef *huart) {
    uint8_t unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    return HAL_UART_Transmit(huart, unlock, sizeof(unlock), 20);
}

/**
 * @brief Set sensor data return rate
 * Rate values:
 * - 0x06: 10Hz
 * - 0x07: 20Hz
 * - 0x08: 50Hz
 * - 0x09: 100Hz
 * - 0x0B: 200Hz
 */
HAL_StatusTypeDef WT61C_SetReturnRate(UART_HandleTypeDef *huart, uint8_t rateEnum) {
    return wit_cmd5(huart, 0x03, rateEnum);
}

/**
 * @brief Save configuration to flash
 */
HAL_StatusTypeDef WT61C_Save(UART_HandleTypeDef *huart) {
    return wit_cmd5(huart, 0x00, 0x0000);
}

/**
 * @brief Weak callback for new data (can be overridden by user)
 */
__weak void WT61C_OnNewData(const WT61C_Data *d) {
    // Default implementation does nothing
    // User can override this in their application code
    (void)d;
}
