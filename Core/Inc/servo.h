#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

void servo_init(uint8_t channel);
void servo_set_angle(uint8_t channel, uint8_t angle);

#endif /* __SERVO_H__ */
