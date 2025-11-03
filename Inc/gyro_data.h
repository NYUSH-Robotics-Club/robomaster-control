#ifndef GYRO_DATA_H

#define GYRO_DATA_H


#include <stdint.h>
#include "main.h"   
#include "bmi088driver.h"



typedef struct {
    int c_ax, c_ay, c_az;
    int c_gx, c_gy, c_gz;
    int c_roll, c_pitch, c_yaw;
    float g_gx, g_gy, g_gz;
    float g_ax, g_ay, g_az;

} SensorData;

void gyro_data_init(void);
void gyro_data_update(SensorData *sensor_data);

#endif