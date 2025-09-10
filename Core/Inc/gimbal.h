#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

void pitch_init(uint8_t id);
void pitch_on_feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm);
int16_t pitch_control_from_joystick(uint8_t id, int16_t joystick_ch1);
int16_t pitch_control_from_dial(uint8_t id, int16_t dial_ch4);

#endif


