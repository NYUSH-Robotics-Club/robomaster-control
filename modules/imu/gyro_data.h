#ifndef GYRO_DATA_H

#define GYRO_DATA_H


#include <stdint.h>
#include "main.h"   
#include "bmi088driver.h"

#define Initial_Tick 6300.0f// the tick when gimbal pointing forward
#define Max_Tick  8192.0f
#define delta_t 0.005f//5ms


/**
 * @brief Sensor data structure for gyroscope and accelerometer readings
 * a is accelerometer, g is gyroscope
 * c_ means chassis sensor, g_ means gimbal sensor
 */
typedef struct {
    int c_ax, c_ay, c_az;
    int c_gx, c_gy, c_gz;
    int c_roll, c_pitch, c_yaw;
    float g_gx, g_gy, g_gz;
    float g_ax, g_ay, g_az;
    float absolute_angle;

} SensorData;

void gyro_data_init(void);
void gyro_data_update(SensorData *sensor_data);
float gimbal_absolute_angle(SensorData* sensor_data);

#endif

