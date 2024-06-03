#include "servo.h"

extern TIM_HandleTypeDef htim1;

void servo_init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void servo_set_angle(uint8_t angle)
{
    if (angle > 180)
    {
        angle = 180;
    }
    uint8_t cnt = 50 + angle * 20 / 18;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cnt);
}
