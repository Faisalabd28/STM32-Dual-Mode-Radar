/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN 0 */
// Volatile flags for hardware interrupts.
// State changes are deferred to the main loop to keep ISR execution time minimal.
volatile uint8_t pb12_interrupt_flag = 0;
volatile uint8_t pb13_interrupt_flag = 0;
/* USER CODE END 0 */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  // Initialize Hardware Timers
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Servo Motor PWM
  HAL_TIM_Base_Start(&htim1);               // Ultrasonic Microsecond Counter

  // Servo Control Parameters (PWM limits for 180-degree sweep)
  uint16_t pulse_min = 500;
  uint16_t pulse_max = 2500;
  uint16_t pulse_width = 1500; // Initialize at 90 degrees (center)
  uint16_t step_size = 2;
  uint16_t delay_ms = 5;

  // Sensor & Data Pipeline Variables
  uint32_t echo_time = 0;
  uint16_t distance = 0;
  char uart_buffer[50];
  uint32_t timeout = 0;

  // System State Management
  int8_t transmit_method = 1;  // 1 = Wired (UART1), -1 = Wireless (UART2)
  int8_t movement_method = 1;  // 1 = Auto-Sweep, -1 = Manual Joystick
  /* USER CODE END 2 */

  while (1)
  {
      // --- Process Pending Hardware Interrupts ---

      // Toggle Data Transmission Route
      if (pb12_interrupt_flag == 1)
      {
          transmit_method = transmit_method * -1;
          pb12_interrupt_flag = 0;
      }

      // Toggle Movement Control Mode
      if (pb13_interrupt_flag == 1)
      {
          movement_method = movement_method * -1;
          pb13_interrupt_flag = 0;
      }

      // ==========================================
      // MODE 1: AUTOMATED RADAR SWEEP
      // ==========================================
      if (movement_method == 1)
      {
          // Sweep UP (0 to 180 degrees)
          for (; pulse_width <= pulse_max; pulse_width += step_size)
          {
              // Check for state changes mid-sweep
              if (pb12_interrupt_flag == 1) { transmit_method *= -1; pb12_interrupt_flag = 0; }
              if (pb13_interrupt_flag == 1) { movement_method *= -1; pb13_interrupt_flag = 0; break; }

              // Move Servo
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);
              HAL_Delay(delay_ms);

              // Transmit 10us Ultrasonic Trigger Pulse
              HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
              HAL_Delay(1);
              HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

              // Wait for Echo line to go HIGH
              timeout = 0;
              while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET && timeout < 30000) { timeout++; }

              // Reset timer and measure Echo pulse duration
              __HAL_TIM_SET_COUNTER(&htim1, 0);

              timeout = 0;
              while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET && timeout < 30000) { timeout++; }

              echo_time = __HAL_TIM_GET_COUNTER(&htim1);
              distance = echo_time * 0.0343 / 2; // Convert raw time to distance in cm

              // Package data into CSV format: [Pulse, Distance, Time]
              if (echo_time == 0 || echo_time > 30000) {
                  snprintf(uart_buffer, sizeof(uart_buffer), "error\r\n");
              } else {
                  snprintf(uart_buffer, sizeof(uart_buffer), "%u,%u,%lu\r\n", pulse_width, distance, echo_time);
              }

              // Route data payload to the active transmission port
              if (transmit_method == 1) {
                  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
              } else if (transmit_method == -1) {
                  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
              } else {
                  snprintf(uart_buffer, sizeof(uart_buffer), "error transmitting\r\n");
                  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
                  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
              }
          }

          // Sweep DOWN (180 to 0 degrees)
          if (movement_method == 1)
          {
              for (; pulse_width >= pulse_min; pulse_width -= step_size)
              {
                  // Check for state changes mid-sweep
                  if (pb12_interrupt_flag == 1) { transmit_method *= -1; pb12_interrupt_flag = 0; }
                  if (pb13_interrupt_flag == 1) { movement_method *= -1; pb13_interrupt_flag = 0; break; }

                  // Move Servo
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);
                  HAL_Delay(delay_ms);

                  // Transmit 10us Ultrasonic Trigger Pulse
                  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
                  HAL_Delay(1);
                  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

                  // Wait for Echo line to go HIGH
                  timeout = 0;
                  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET && timeout < 30000) { timeout++; }

                  // Reset timer and measure Echo pulse duration
                  __HAL_TIM_SET_COUNTER(&htim1, 0);

                  timeout = 0;
                  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET && timeout < 30000) { timeout++; }

                  echo_time = __HAL_TIM_GET_COUNTER(&htim1);
                  distance = echo_time * 0.0343 / 2; // Convert raw time to distance in cm

                  // Package data into CSV format: [Pulse, Distance, Time]
                  if (echo_time == 0 || echo_time > 30000) {
                      snprintf(uart_buffer, sizeof(uart_buffer), "error\r\n");
                  } else {
                      snprintf(uart_buffer, sizeof(uart_buffer), "%u,%u,%lu\r\n", pulse_width, distance, echo_time);
                  }

                  // Route data payload to the active transmission port
                  if (transmit_method == 1) {
                      HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
                  } else if (transmit_method == -1) {
                      HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
                  } else {
                      snprintf(uart_buffer, sizeof(uart_buffer), "error transmitting\r\n");
                      HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
                      HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
                  }
              }
          }
      }
      // ==========================================
      // MODE -1: MANUAL ANALOG JOYSTICK OVERRIDE
      // ==========================================
      else if (movement_method == -1)
      {
          // Poll ADC to read current joystick voltage
          HAL_ADC_Start(&hadc1);
          HAL_ADC_PollForConversion(&hadc1, 10);
          uint32_t joystick_val = HAL_ADC_GetValue(&hadc1);

          // Calibrated joystick deadzones (Center resting value is ~3145)
          int32_t deadzone_left = 2900;
          int32_t deadzone_right = 3400;

          // Asymmetric sensitivity scaling to balance hardware voltage offsets
          int32_t sensitivity_left = 150;
          int32_t sensitivity_right = 37;

          int32_t dynamic_step = 0;
          int32_t difference = 0;

          // Process Left Deflection
          if (joystick_val < deadzone_left)
          {
              difference = deadzone_left - joystick_val;
              dynamic_step = 1 + (difference / sensitivity_left);

              if (pulse_width > pulse_min) {
                  if (pulse_width > (pulse_min + dynamic_step)) {
                      pulse_width -= dynamic_step;
                  } else {
                      pulse_width = pulse_min;
                  }
              }
          }
          // Process Right Deflection
          else if (joystick_val > deadzone_right)
          {
              difference = joystick_val - deadzone_right;
              dynamic_step = 1 + (difference / sensitivity_right);

              if (pulse_width < pulse_max) {
                  if (pulse_width < (pulse_max - dynamic_step)) {
                      pulse_width += dynamic_step;
                  } else {
                      pulse_width = pulse_max;
                  }
              }
          }

          // Apply calculated dynamic step to servo
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);
          HAL_Delay(delay_ms);

          // Continuous Radar Ping for live environment monitoring during manual override
          HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
          HAL_Delay(1);
          HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

          timeout = 0;
          while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET && timeout < 30000) { timeout++; }

          __HAL_TIM_SET_COUNTER(&htim1, 0);

          timeout = 0;
          while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET && timeout < 30000) { timeout++; }

          echo_time = __HAL_TIM_GET_COUNTER(&htim1);
          distance = echo_time * 0.0343 / 2;

          if (echo_time == 0 || echo_time > 30000) {
              snprintf(uart_buffer, sizeof(uart_buffer), "error\r\n");
          } else {
              snprintf(uart_buffer, sizeof(uart_buffer), "%u,%u,%lu\r\n", pulse_width, distance, echo_time);
          }

          // Route data payload to the active transmission port
          if (transmit_method == 1) {
              HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
          } else if (transmit_method == -1) {
              HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
          } else {
              snprintf(uart_buffer, sizeof(uart_buffer), "error transmitting\r\n");
              HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
              HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
          }
      }
  }
}

// ---------------------------------------------------------
// INITIALIZATION FUNCTIONS (Auto-Generated by STM32CubeMX)
// ---------------------------------------------------------
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8MHz Crystal * 9 = 72MHz System Clock
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
// Software debounce algorithm and hardware state verification
// to filter out noisy contacts and fix phantom button presses
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_time_pb12 = 0;
    static uint32_t last_time_pb13 = 0;
    uint32_t current_time = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_12)
    {
        // 300ms debounce window + physical high-state verification
        if ((current_time - last_time_pb12 > 300) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET))
        {
            pb12_interrupt_flag = 1;
            last_time_pb12 = current_time;
        }
    }

    if (GPIO_Pin == GPIO_PIN_13)
    {
        // 300ms debounce window + physical high-state verification
        if ((current_time - last_time_pb13 > 300) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET))
        {
            pb13_interrupt_flag = 1;
            last_time_pb13 = current_time;
        }
    }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */
