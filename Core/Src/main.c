/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AHT20.h"
#include "bmp280.h"
#include "IR.h"
#include "servo.h"
#include "SHTC3.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char rx_buffer[256];
uint8_t MSG_LEN = 7;
uint8_t light_state = 0;
uint8_t door_state = 0;
uint8_t aircon_state = 0;
uint8_t aircon_temp = 26;
uint8_t aircon_mode = 0;
uint32_t interval = 1*1000;
struct sensor_equipment sensor;
struct actuator_equipment actuator;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void getLightStrength(uint32_t *light);
void sensor_init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  servo_init();
  sensor_init();
  /* USER CODE END 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buffer, MSG_LEN);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float temperature = 0;
  int32_t humidity = 0;
  uint32_t pressure = 0;
  uint32_t light = 0;
  uint8_t human = 0;
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    getLightStrength(&light);
    getTempAndHumid(&hi2c1, &temperature, &humidity);
    getPressure(&hi2c1, &pressure);
    char message[100];
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
    {
      human = 1;
    }
    else
    {
      human = 0;
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)message, sprintf(message, "{T: %.2fC, H: %d%%, L: %dlx, M: %d, I: %d}\n", temperature, humidity, light, human, light_state), 1000);
    HAL_Delay(interval);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void sensor_init()
{
  if (shtc3_read_id(&hi2c1) == HAL_OK)
  {
    sensor.SHTC3 = 1;
  }
  if (AHT20_Init(&hi2c1) == HAL_OK)
  {
    sensor.AHT20 = 1;
  }
  bmp280_params_t params;
  bmp280_init_default_params(&params);
  if (bmp280_init(&hi2c1, &params) == HAL_OK)
  {
    sensor.BMP280 = 1;
  }
  if (sgp30_init() == HAL_OK)
  {
    sensor.SGP30 = 1;
  }
  actuator.servo = 0; // if there is a servo (not a switch)
  actuator.IR = 0;    // if there is an IR sensor
  actuator.light_switch = 0;
  actuator.door_lock = 1;
}

void getLightStrength(uint32_t *light)
{
  uint32_t digits = 12;
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  // *light = HAL_ADC_GetValue(&hadc1);
  *light = (1 << digits) - HAL_ADC_GetValue(&hadc1);
  // *light = (uint32_t)(*light * 1.0 / (1<<digits));
  HAL_ADC_Stop(&hadc1);
}

void getTempAndHumid(I2C_HandleTypeDef *hi2c, float *temp, int32_t *humid)
{
  if (sensor.SHTC3)
  {
    int32_t temp_raw, humid_raw;
    if (!shtc3_read_id(hi2c))
    {
      *temp = 0;
      *humid = 0;
    }
    else
    {
      if (shtc3_perform_measurements(hi2c, &temp_raw, &humid_raw))
      {
        *temp = temp_raw / 100.0;
        *humid = humid_raw;
      }
      else
      {
        *temp = 0;
        *humid = 0;
      }
    }
  }
  else if (sensor.AHT20)
  {
    AHT20_Measure();
    *temp = AHT20_Temp();
    *humid = AHT20_Humid();
  }
  else
  {
    *temp = 0;
    *humid = 0;
  }
}

void getPressure(I2C_HandleTypeDef *hi2c, uint32_t *pressure)
{
  if (sensor.BMP280)
  {
    float temp, pres, hum;
    bmp280_read_float(&hi2c1, &temp, &pres, &hum);
    *pressure = (uint32_t)(pres * 100);
  }
  else
  {
    *pressure = 0;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // format: "%3d %3d" (angle, interval)
    char message[] = "\"%%3d %%3d %%3d\" (door, aircon, interval)";
    if (rx_buffer[6] == '?') 
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)message, 27, 1000);
      HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buffer, MSG_LEN);
      return;
    }
    uint8_t recv_angle = atoi(rx_buffer);
    uint8_t recv_aircon_power = rx_buffer[4] - '0';
    uint8_t recv_aircon_temp = rx_buffer[5] - '0';  // 0 down, 1 stay, 2 up
    uint8_t recv_aircon_mode = rx_buffer[6] - '0';
    uint16_t recv_interval = atoi(rx_buffer + 7);
    HAL_UART_Transmit(&huart1, (uint8_t *)rx_buffer, MSG_LEN, 1000);

    servo_set_angle(recv_angle);

    if (recv_aircon_power == 1)
    {
      aircon_state = 1;
      IR_SetPower(recv_aircon_power);
      IR_SetTemperature(recv_aircon_temp + aircon_temp - 1);
      IR_SetMode(recv_aircon_mode);
    }
    else
    {
      aircon_state = 0;
      IR_SetPower(recv_aircon_power);
    }

    if (recv_interval == 0)
    {
      recv_interval = 1;
    }
    interval = recv_interval * 1000;
    if (recv_angle < 90)
    {
      light_state = 0;
    }
    else
    {
      light_state = 1;
    }
  }
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buffer, MSG_LEN);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
