#include "gyro_data.h"
#include "bmi088driver.h"
#include "wt61c.h"
#include "printing.h"

static float gyro[3];
static float accel[3];
static float temp;


void gyro_data_init(void)
{
    BMI088_init();
    // Any calibration or startup routines
}
void gyro_data_update(SensorData *sensor_data)
{
    BMI088_read(gyro, accel, &temp);

    sensor_data->g_gx = gyro[0];
    sensor_data->g_gy = gyro[1];
    sensor_data->g_gz = gyro[2];
    sensor_data->g_ax   = accel[0];
    sensor_data->g_ay   = accel[1];
    sensor_data->g_az   = accel[2];


    WT61C_Data const* d = WT61C_GetData();
    sensor_data->c_ax = (int)(d->ax * 1000);  // m/s^2 * 1000
    sensor_data->c_ay = (int)(d->ay * 1000);
    sensor_data->c_az = (int)(d->az * 1000);
    sensor_data->c_gx = (int)(d->gx * 10);    // deg/s * 10
    sensor_data->c_gy = (int)(d->gy * 10);
    sensor_data->c_gz = (int)(d->gz * 10);
    sensor_data->c_roll = (int)(d->roll * 10);
    sensor_data->c_pitch = (int)(d->pitch * 10);
    sensor_data->c_yaw = (int)(d->yaw * 10);
    int temp_i = (int)(d->temperature * 10);
}

/*
WT61C new data callback - sends JSON formatted data via USB CDC
*/
void WT61C_OnNewData(const WT61C_Data *d)
{
  // Throttle output heavily to avoid USB buffer overflow
  static uint32_t last_output_time = 0;
  static uint32_t frame_count = 0;
  uint32_t now = HAL_GetTick();

  frame_count++;

  // Output data only every 100ms to avoid overflow
  if (now - last_output_time < 100) {
    return; // Skip this update
  }
  last_output_time = now;

  // Convert floats to integers for printf (workaround for missing float support)
  
  

  // USB_CDC_Printf("{\"ax\":%d,\"ay\":%d,\"az\":%d,"
  //                "\"gx\":%d,\"gy\":%d,\"gz\":%d,"
  //                "\"roll\":%d,\"pitch\":%d,\"yaw\":%d,"
  //                "\"T\":%d}\r\n",
  //                (int)ax_i, (int)ay_i, (int)az_i,
  //                (int)gx_i, (int)gy_i, (int)gz_i,
  //                (int)roll_i, (int)pitch_i, (int)yaw_i,
  //                (int)temp_i);

  //USB_CDC_Printf("\"gz\":%d", (int)gz_i, "\n");


}