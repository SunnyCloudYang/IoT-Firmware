#include "servo.h"

extern TIM_HandleTypeDef htim1;

void servo_init(uint8_t channel)
{
  if (channel == 1)
  {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  }
  else if (channel == 2)
  {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  }
  else if (channel == 3)
  {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  }
  else if (channel == 4)
  {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  }
  else
  {
    Error_Handler();
  }
}

void servo_set_angle(uint8_t channel, uint8_t angle)
{
    if (angle > 180)
    {
        angle = 180;
    }
    uint8_t cnt = 50 + angle * 20 / 18;
    if (channel == 1)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cnt);
    }
    else if (channel == 2)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cnt);
    }
    else if (channel == 3)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cnt);
    }
    else if (channel == 4)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, cnt);
    }
    else
    {
        Error_Handler();
    }
}
