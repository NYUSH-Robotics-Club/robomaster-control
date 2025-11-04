#ifndef GM6020_MOTOR_H
#define GM6020_MOTOR_H

#include <stdint.h>
#include "gyro_data.h"

void Motor_Init(uint8_t id, float KP, float KI, float KD, float initial_angle );
void GM6020_Motor_Feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm);
void Target_Angle_Correction(SensorData* sensor_data);
int16_t Joystick_control(uint8_t id, int16_t joystick_ch1, SensorData *sensor_data);
int16_t Yaw_Control_With_Compensation(int16_t joystick_yaw, SensorData* sensor_data);

#endif


