#include "gyro_data.h"
#include "bmi088driver.h"
#include "wt61c.h"
#include "printing.h"
#include "message_center.h"

static float gyro[3];
static float accel[3];
static float temp;

SensorData* sensor_data;

void gyro_data_init(void)
{
    BMI088_init();
    // Any calibration or startup routines
}

void gyro_data_update()
{
    BMI088_read(gyro, accel, &temp);

    sensor_data->g_gx = gyro[0];
    sensor_data->g_gy = gyro[1];
    sensor_data->g_gz = gyro[2];
    sensor_data->g_ax   = accel[0];
    sensor_data->g_ay   = accel[1];
    sensor_data->g_az   = accel[2];

    WT61C_Data const* d = WT61C_GetData();
    sensor_data->c_ax = (int)(d->ax * 1000);
    sensor_data->c_ay = (int)(d->ay * 1000);
    sensor_data->c_az = (int)(d->az * 1000);
    sensor_data->c_gx = (int)(d->gx );
    sensor_data->c_gy = (int)(d->gy );
    sensor_data->c_gz = (int)(d->gz );
    sensor_data->c_roll = (int)(d->roll * 10);
    sensor_data->c_pitch = (int)(d->pitch * 10);
    sensor_data->c_yaw = (int)(d->yaw * 10);
    (void)MsgCenter_Publish(TOPIC_IMU_UPDATE, sensor_data, sizeof(*sensor_data));
}

void WT61C_OnNewData(const WT61C_Data *d)
{
  (void)d;
}


