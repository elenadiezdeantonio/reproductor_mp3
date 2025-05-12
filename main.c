/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "i2c_lcd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

I2C_LCD_HandleTypeDef lcd1;

GPIO_PinState play_button_state, next_button_state, prev_button_state;
uint8_t is_playing = 0; // 0: pausado, 1: reproduciendo

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Función para enviar comandos al módulo DY-SV5W
static HAL_StatusTypeDef dfplayer_send_command(uint8_t command, uint8_t data_len, uint8_t* data) {
    /* Calculate checksum */
    unsigned checksum = 0xAA + command + data_len;
    if (data_len) {
        uint8_t* pdata = data;
        for (int remaining = data_len; remaining; remaining--)
            checksum += *pdata++;
    }
    checksum &= 0xFF;

    /* Send command */
    uint8_t buffer[3] = {0xAA, command, data_len};
    HAL_StatusTypeDef rc = HAL_UART_Transmit(&huart2, buffer, 3, HAL_MAX_DELAY);
    if (rc != HAL_OK)
        return rc;

    /* Send data (if needed) */
    if (data_len) {
        rc = HAL_UART_Transmit(&huart2, data, data_len, HAL_MAX_DELAY);
        if (rc != HAL_OK)
            return rc;
    }

    /* Send checksum */
    buffer[0] = (uint8_t)checksum;
    HAL_UART_Transmit(&huart2, buffer, 1, HAL_MAX_DELAY);

    return rc;
}

HAL_StatusTypeDef dfplayer_play(void) {
    return dfplayer_send_command(0x02, 0, NULL);
}

HAL_StatusTypeDef dfplayer_pause(void) {
    return dfplayer_send_command(0x03, 0, NULL);
}

HAL_StatusTypeDef dfplayer_next(void) {
    return dfplayer_send_command(0x06, 0, NULL);
}

HAL_StatusTypeDef dfplayer_previous(void) {
    return dfplayer_send_command(0x05, 0, NULL);
}

HAL_StatusTypeDef dfplayer_set_volume(uint8_t vol) {
    /* Sanitize inputs */
    if (vol > 30)
        vol = 30;

    /* Send command */
    return dfplayer_send_command(0x13, 1, &vol);
}

HAL_StatusTypeDef dfplayer_play_song(uint16_t number) {
    uint8_t buffer[2] = {(uint8_t)(number >> 8), (uint8_t)(number & 0xFF)};
    return dfplayer_send_command(0x07, 2, buffer);
}

uint32_t Leer_Potenciometro(void){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	uint32_t adc_value=HAL_ADC_GetValue(&hadc1);
	return adc_value;
}
uint8_t Calcular_Volumen(uint32_t adc_value){
	return (adc_value * 30)/4095;
}
void Display_Volumen_Play(void){
	char buffer[16];
	uint32_t adc_value=Leer_Potenciometro();
	uint8_t volumen=Calcular_Volumen(adc_value);
	sprintf(buffer,"Vol: %2d%",volumen);
	lcd_clear(&lcd1);
	lcd_gotoxy(&lcd1, 0, 0);
	lcd_puts(&lcd1,"Reproduciendo");
	lcd_gotoxy(&lcd1, 0, 1);
	lcd_puts(&lcd1,buffer);
}
void Display_Volumen_Pausa(void){
	char buffer[16];
	uint32_t adc_value=Leer_Potenciometro();
	uint8_t volumen=Calcular_Volumen(adc_value);
	sprintf(buffer,"Vol: %2d%",volumen);
	lcd_clear(&lcd1);
	lcd_gotoxy(&lcd1, 0, 0);
	lcd_puts(&lcd1,"Pausa");
	lcd_gotoxy(&lcd1, 0, 1);
	lcd_puts(&lcd1,buffer);
}
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd1.hi2c = &hi2c1;
  lcd1.address = 0x4E;
  lcd_init(&lcd1);
  lcd_clear(&lcd1);
  HAL_Delay(100);
  lcd_gotoxy(&lcd1, 0, 0);
  lcd_puts(&lcd1,"Pulsa Play");

  HAL_StatusTypeDef rc = HAL_UART_Init(&huart2);  /* Inicia la UART */
  rc = dfplayer_set_volume(15);  /* Ajusta el volumen */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      play_button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
      next_button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
      prev_button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
      // Leer y actualizar el volumen continuamente
      uint32_t adc_value = Leer_Potenciometro();
      uint8_t volumen = Calcular_Volumen(adc_value);
      rc = dfplayer_set_volume(volumen); // Ajustar el volumen en tiempo real

      char buffer[16];
      sprintf(buffer, "Vol: %2d%", volumen);
      lcd_gotoxy(&lcd1, 0, 1);
      lcd_puts(&lcd1, buffer);

      if (play_button_state == GPIO_PIN_RESET){
          HAL_Delay(50);
          while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
          HAL_Delay(50);
          if (is_playing){
        	  HAL_Delay(50);
        	  Display_Volumen_Pausa();
        	  HAL_Delay(50);
              rc = dfplayer_pause();
              is_playing = 0;
          } else{
        	  HAL_Delay(50);
        	  Display_Volumen_Play();
        	  HAL_Delay(50);
              rc = dfplayer_play();
              is_playing = 1;
          }
      }

      if (next_button_state == GPIO_PIN_RESET){
          HAL_Delay(50);
          while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET);
          HAL_Delay(50);
          lcd_clear(&lcd1);
          HAL_Delay(50);
          lcd_gotoxy(&lcd1, 0, 0);
          lcd_puts(&lcd1,"Siguiente");
          HAL_Delay(2000);
          Display_Volumen_Play();
          rc = dfplayer_next();
      }

      if (prev_button_state == GPIO_PIN_RESET){
          HAL_Delay(50);
          while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET);
          HAL_Delay(50);
          lcd_clear(&lcd1);
          HAL_Delay(50);
          lcd_gotoxy(&lcd1, 0, 0);
          lcd_puts(&lcd1,"Anterior");
          HAL_Delay(2000);
          Display_Volumen_Play();
          rc = dfplayer_previous();
      }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PLAY_PAUSA_Pin NEXT_Pin PREV_Pin */
  GPIO_InitStruct.Pin = PLAY_PAUSA_Pin|NEXT_Pin|PREV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
