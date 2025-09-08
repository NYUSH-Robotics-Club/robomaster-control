#ifndef GM6020_H
#define GM6020_H

#include <stdint.h>

void gm6020_init(uint8_t id);
void gm6020_on_feedback(uint8_t id, uint16_t angle_raw, int16_t speed_rpm);
int16_t gm6020_control_from_joystick(uint8_t id, int16_t joystick_ch1);

#endif // GM6020_H


