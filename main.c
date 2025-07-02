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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RC522.h"
#include "string.h"

// KOD LCD
#include "lcd.h"
//#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GPIO_Port GPIOA   // Port diody
#define LED_Pin GPIO_PIN_1    // Pin diody

//#define UART_RX_BUFFER_SIZE 10 // Odbór z UART1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t status;
uint8_t str[16];
uint8_t sNum[5];

// KOD MQTT
uint8_t send_uid_to_esp = 1;  // 1 = wysyłaj UID do ESP32 przez UART

// KOD SERWO
uint8_t tag_triggered = 0;
uint32_t tag_trigger_time = 0;

char *msg11 = "Card 1\r\n";
char *msg12 = "Card 2\r\n";
char *msg21 = "TAG 1\r\n";
char *msg22 = "TAG 2\r\n";
char *msg3 = "Legitymacja\r\n";
char *msg4 = "Phone\r\n";

char uart_buffer[100];  // Bufor UART (globalna zmienna)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// KOD SERWO
void Servo_SetAngle(uint8_t angle)
{
    // Pulse min i max zgodnie z konfiguracją timera
    uint32_t pulse_min = 500;   // 0.5 ms
    uint32_t pulse_max = 2500;  // 2.5 ms

    // Oblicz puls szerokości impulsu odpowiadający kątowi
    uint32_t pulse_length = pulse_min + ((pulse_max - pulse_min) * angle) / 180;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_length);
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();

  // KOD LCD
  LCD_Init();
  LCD_Clear();

  // KOD SERWO
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  Servo_SetAngle(90);  // Domyślnie na 90°
  LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_Print("Door closed");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	// Wysyłanie komendy do MFRC522
	uint8_t status = MFRC522_Request(PICC_REQIDL, str);

	if (status == MI_OK)
	{

		// Antykolizja i odczyt numeru karty
		status = MFRC522_Anticoll(str);
		if (status == MI_OK)
		{
			memcpy(sNum, str, 5);

			// Wysałnie numeru karty na UART
			sprintf(uart_buffer, "UID: %02X %02X %02X %02X %02X\r\n", str[0], str[1], str[2], str[3], str[4]);
			HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);


			if ((str[0]==35) && (str[1]==39) && (str[2]==15) && (str[3]==40) && (str[4]==35)){
				HAL_UART_Transmit(&huart1, (uint8_t*)msg11, strlen(msg11), HAL_MAX_DELAY);

				// Wyświetlanie LCD
			    LCD_Clear();
			    LCD_SetCursor(0,0);
			    LCD_Print(msg11);
			    LCD_SetCursor(1,0);
			    LCD_Print("Door open");

	            if (tag_triggered == 0)  // Tylko jeśli nie było aktywne
	            {
	                Servo_SetAngle(180);  // Przekręć na 180°
	                tag_trigger_time = HAL_GetTick();  // Zapisz czas
	                tag_triggered = 1;
	            }
			}
			if ((str[0]==21) && (str[1]==156) && (str[2]==222) && (str[3]==0) && (str[4]==87)){
				HAL_UART_Transmit(&huart1, (uint8_t*)msg12, strlen(msg12), HAL_MAX_DELAY);

				// Wyświetlanie LCD
			    LCD_Clear();
			    LCD_SetCursor(0,0);
			    LCD_Print(msg12);
			    LCD_SetCursor(1,0);
			    LCD_Print("Door open");

	            if (tag_triggered == 0)  // Tylko jeśli nie było aktywne
	            {
	                Servo_SetAngle(180);  // Przekręć na 180°
	                tag_trigger_time = HAL_GetTick();  // Zapisz czas
	                tag_triggered = 1;
	            }
			}
			if ((str[0]==105) && (str[1]==151) && (str[2]==174) && (str[3]==2) && (str[4]==82)){
				HAL_UART_Transmit(&huart1, (uint8_t*)msg21, strlen(msg21), HAL_MAX_DELAY);

				// Wyświetlanie LCD
			    LCD_Clear();
			    LCD_SetCursor(0,0);
			    LCD_Print(msg21);
			    LCD_SetCursor(1,0);
			    LCD_Print("Door open");

	            if (tag_triggered == 0)  // Tylko jeśli nie było aktywne
	            {
	                Servo_SetAngle(180);  // Przekręć na 180°
	                tag_trigger_time = HAL_GetTick();  // Zapisz czas
	                tag_triggered = 1;
	            }
			}
			if ((str[0]==66) && (str[1]==144) && (str[2]==253) && (str[3]==0) && (str[4]==47)){
				HAL_UART_Transmit(&huart1, (uint8_t*)msg22, strlen(msg22), HAL_MAX_DELAY);

				// Wyświetlanie LCD
			    LCD_Clear();
			    LCD_SetCursor(0,0);
			    LCD_Print(msg22);
			    LCD_SetCursor(1,0);
			    LCD_Print("Door open");

	            if (tag_triggered == 0)  // Tylko jeśli nie było aktywne
	            {
	                Servo_SetAngle(180);  // Przekręć na 180°
	                tag_trigger_time = HAL_GetTick();  // Zapisz czas
	                tag_triggered = 1;
	            }
			}
			if ((str[0]==13) && (str[1]==54) && (str[2]==99) && (str[3]==147) && (str[4]==203)){
				HAL_UART_Transmit(&huart1, (uint8_t*)msg3, strlen(msg3), HAL_MAX_DELAY);

				// Wyświetlanie LCD
			    LCD_Clear();
			    LCD_SetCursor(0,0);
			    LCD_Print(msg3);
			    LCD_SetCursor(1,0);
			    LCD_Print("Door open");

	            if (tag_triggered == 0)  // Tylko jeśli nie było aktywne
	            {
	                Servo_SetAngle(180);  // Przekręć na 180°
	                tag_trigger_time = HAL_GetTick();  // Zapisz czas
	                tag_triggered = 1;
	            }
			}
			if (str[0]==8){
				HAL_UART_Transmit(&huart1, (uint8_t*)msg4, strlen(msg4), HAL_MAX_DELAY);

				// Wyświetlanie LCD
			    LCD_Clear();
			    LCD_SetCursor(0,0);
			    LCD_Print(msg4);
			    LCD_SetCursor(1,0);
			    LCD_Print("Door open");

	            if (tag_triggered == 0)  // Tylko jeśli nie było aktywne
	            {
	                Servo_SetAngle(180);  // Przekręć na 180°
	                tag_trigger_time = HAL_GetTick();  // Zapisz czas
	                tag_triggered = 1;
	            }
			}
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);  // Włącz diodę
			HAL_Delay(4000); // Dioda świeci przez 4s
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // Wyłącz diodę
		}
	}
	if (tag_triggered && (HAL_GetTick() - tag_trigger_time >= 4000))
	{
	    Servo_SetAngle(90);
	    tag_triggered = 0;
	}

    // KOD MQTT - Otwieranie z aplikacji
    uint8_t rx_data;
    if (HAL_UART_Receive(&huart1, &rx_data, 1, 10) == HAL_OK) {
        if (rx_data == 'o') {	//Odbiór polecenia z aplikacji
            Servo_SetAngle(180);	//Otwarcie zamka
            HAL_UART_Transmit(&huart1, (uint8_t*)"_o_", strlen("_o_") + 1, HAL_MAX_DELAY); // Wysłanie potwierdzenia otwarcia
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);  // Włącz diodę

			// Wyświetlanie LCD
		    LCD_Clear();
		    LCD_SetCursor(0,0);
		    LCD_Print("Admin");
		    LCD_SetCursor(1,0);
		    LCD_Print("Door open");

            HAL_Delay(4000);
            Servo_SetAngle(90);	// Zamknięcie zamka
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // Wyłącz diodę
        }
    }

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("Door closed");

	HAL_Delay(200); // Małe opóźnienie, żeby uniknąć spamowania UART

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB3 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
