// This IR control ONLY supports NEC protocol.
#ifndef __IR_H__
#define __IR_H__

#include "main.h"

void IR_SetPower(uint8_t on_off);
void IR_SetTemperature(uint8_t temperature);
void IR_SetMode(uint8_t mode);
void IR_SetFanSpeed(uint8_t fan_speed);

#endif // __IR_H__
