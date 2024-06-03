#ifndef __GY302_H__
#define __GY302_H__

#include "main.h"

#ifdef __cplusplus
#define EXPORT extern "C"
#else
#define EXPORT
#endif

uint8_t gy302_read_id(I2C_HandleTypeDef *hi2c);
uint32_t gy302_read_lux(I2C_HandleTypeDef *hi2c, uint16_t *lux);

void getLightStrength(I2C_HandleTypeDef *hi2c, uint16_t *lux);

#endif // __GY302_H__
