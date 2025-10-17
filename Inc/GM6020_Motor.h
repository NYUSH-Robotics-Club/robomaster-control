#ifndef GM6020_MOTOR_H
#define GM6020_MOTOR_H

#include <stdint.h>

void Motor_Init(uint8_t id, float KP, float KI, float KD);
void GM6020_Motor_Feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm);
int16_t Joystick_control(uint8_t id, int16_t joystick_ch1);


#endif


