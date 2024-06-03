#include "GY302.h"

uint8_t gy302_read_id(I2C_HandleTypeDef *hi2c)
{
  uint8_t data[1];
  uint8_t command = 0x92;

  uint32_t res =  HAL_I2C_Master_Transmit(hi2c, GY302_ADDRESS_WRITE, &command, 1, 100);
  if (res != HAL_OK) {
    return 0;
  }
  res = HAL_I2C_Master_Receive(hi2c, GY302_ADDRESS_READ, data, 1, 100);
  if (res != HAL_OK) {
    return 0;
  }

  return data[0];
}

uint32_t gy302_read_lux(I2C_HandleTypeDef *hi2c, uint16_t *lux)
{
  uint8_t data[2];
  uint8_t command = 0x20;

  uint32_t res =  HAL_I2C_Master_Transmit(hi2c, GY302_ADDRESS_WRITE, &command, 1, 100);
  if (res != HAL_OK) {
    return 0;
  }
  res = HAL_I2C_Master_Receive(hi2c, GY302_ADDRESS_READ, data, 2, 100);
  if (res != HAL_OK) {
    return 0;
  }

  *lux = data[0] << 8 | data[1];
  return 1;
}

void getLightStrength(I2C_HandleTypeDef *hi2c, uint16_t *lux)
{
  gy302_read_lux(hi2c, lux);
}