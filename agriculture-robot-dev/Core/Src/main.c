/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body integrating motor control and ultrasonic sensor.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;  // For motor PWM
TIM_HandleTypeDef htim4;  // For ultrasonic sensor delay
UART_HandleTypeDef huart2;

const float speedOfSound = 0.0343f/2;  // in cm/usec
float distance;
char uartBuf[100];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
void Error_Handler(void);
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif

// This function uses TIM4 for microsecond delays.
void usDelay(uint32_t uSec)
{
  if(uSec < 2) uSec = 2;
  TIM4->ARR = uSec - 1;    /* Set auto-reload value */
  TIM4->EGR = 1;           /* Update registers */
  TIM4->SR &= ~1;          /* Reset the update flag */
  TIM4->CR1 |= 1;          /* Enable the counter */
  while((TIM4->SR & 0x0001) != 1);
  TIM4->SR &= ~(0x0001);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t numTicks = 0;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* Start PWM channels for motor control */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  /* Main loop */
  while (1)
  {
    /* --- Ultrasonic sensor measurement --- */
    // Set TRIG low for a short period
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    usDelay(3);
    // Output 10 µsec pulse on TRIG
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    usDelay(10);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    // Wait for ECHO rising edge
    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);
    numTicks = 0;
    // Measure the pulse width on ECHO pin
    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
    {
      numTicks++;
      usDelay(2);
    }
    // Calculate distance in cm
    distance = numTicks * 2.8f * speedOfSound;
    sprintf(uartBuf, "Distance (cm) = %.1f\r\n", distance);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);

    /* --- Decision making based on distance --- */
    if(distance <= 10.0f)  // Obstacle detected within 10 cm
    {
      if(distance < 5.0f)  // If very close, perform a turning maneuver
      {
        // Alternate between turning left and right
        static uint8_t turnDirection = 0;  // 0: turn left, 1: turn right
        // Set motor directions: both motors forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);  // Right motor forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);  // Left motor forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);

        if(turnDirection == 0)
        {
          // Turn left: right motor full speed, left motor slower
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);  // Left motor reduced speed
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2000);  // Right motor full speed
        }
        else
        {
          // Turn right: left motor full speed, right motor slower
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000);  // Left motor full speed
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);  // Right motor reduced speed
        }
        HAL_Delay(1000);
        turnDirection = !turnDirection;  // Toggle turning direction
      }
      else  // Obstacle is detected but not too close; move straight forward
      {
        // Set motor directions: both motors forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);  // Right motor forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);  // Left motor forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
        // Both motors at full speed
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2000);
        HAL_Delay(1000);
      }
    }
    else  // No obstacle within 10 cm: stop motors
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
      HAL_Delay(1000);
    }
    HAL_Delay(100);
  }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM2 Initialization Function for PWM (Motor control) */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/* TIM4 Initialization Function for microsecond delay */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART2 Initialization Function */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO Ports Clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure motor control pins */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

  /* Configure button pin if used */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Configure output pins for motor direction */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure ultrasonic sensor pins */
  // TRIG and LED pins on GPIOA are configured as output.
  GPIO_InitStruct.Pin = TRIG_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);
  // ECHO pin configured as input.
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add implementation here */
}
#endif
