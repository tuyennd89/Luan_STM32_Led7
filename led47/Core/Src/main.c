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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void clock_signal(void){
	HAL_GPIO_WritePin(GPIOB,CLCOK_Pin,1);
//   digitalWrite(CLCOK_pin, HIGH);
//	HAL_Delay(1);
//    delayMicroseconds(500);
	HAL_GPIO_WritePin(GPIOB,CLCOK_Pin,0);
//   digitalWrite(CLCOK_pin, LOW);
//	HAL_Delay(1);
//    delayMicroseconds(500);
}
void latch_enable(void)
   {
	HAL_GPIO_WritePin(GPIOB,LATCH_Pin,1);
//    digitalWrite(LATCH_pin, HIGH);
//	HAL_Delay(1);
//    delayMicroseconds(500);
	HAL_GPIO_WritePin(GPIOB,LATCH_Pin,0);
//    digitalWrite(LATCH_pin, LOW);
    }
void send_data(unsigned int data_out)
{
    int i;
    for (i=0 ; i<8 ; i++)
    {
        if ((data_out >> i) & (0x01))
        {
        	HAL_GPIO_WritePin(GPIOB,DATA_Pin,1);
//        digitalWrite(DATA_pin,HIGH);
        }
        else
        {
        	HAL_GPIO_WritePin(GPIOB,DATA_Pin,0);
//        digitalWrite(DATA_pin,LOW);
        }

        clock_signal();
    }
//    latch_enable(); // Data finally submitted
}

void Display_led_number(int number, int times){
//	uint8_t index = 16;
	uint8_t nghin = number/1000;
	uint8_t tram = (number%1000)/100;
	uint8_t chuc = (number%100)/10;
	uint8_t donvi = number%10;

	for(int i = 0; i < times; i++ ){
		Display_led_Dec(1, nghin);
		Display_led_Dec(2, tram);
		Display_led_Dec(3, chuc);
		Display_led_Dec(4, donvi);
	}
}

void Display_led_Dec(uint8_t led_index, uint8_t number){
	uint8_t led_codes[] = {3, 159, 37, 13, 153, 73, 65, 31, 1, 9};

	uint8_t index = 16;
	for(uint8_t i = 1; i < led_index; i++ ){
		index *= 2;
	}

	Display_Led(index,led_codes[number]);
}
void Display_Led(uint8_t Led,uint8_t number)
{
	send_data(number);
	send_data(Led);
	latch_enable(); // Data finally submitted
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int number = 0;
  GPIO_PinState preStatus = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  GPIO_PinState curStatus = preStatus;
  while (1)
  {
	  curStatus = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if(curStatus != preStatus && curStatus == GPIO_PIN_SET){
		  number += 1;
	  }
	  preStatus = curStatus;
	  Display_led_number(number, 1);

//	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
//		  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
//			  Display_led_number(number, 1);
//		  }
//		  number += 1;
//	  }
//	  Display_led_number(number, 1000);

    /* USER CODE END WHILE */

//	  for(uint8_t led_index = 1; led_index < 5; led_index++ ){
//		  for(uint8_t number = 0; number < 10; number++ ){
//			  Display_led_Dec(led_index, number);
//			  HAL_Delay(300);
//		  }
//	  }


//	  for(int number = 0; number < 10000; number++ ){
//		  Display_led_number(number);
//	  }





//	  Display_Led(0xFF,0x00);
//	  Display_Led(48,0xC0);
//	  Display_Led(0x10,0xFC);
//	  Display_Led(0x10,0x02);
//	  Display_Led(0x10,0x03);
//	  Display_Led(0x10,0x04);
//	  Display_Led(0x10,0x05);
//	  Display_Led(0x10,0x06);
//	  Display_Led(0x10,0x07);
//	  Display_Led(0x10,0x08);
//	  Display_Led(0x10,0x09);
//	  Display_Led(0x10,0x0A);
//	  Display_Led(0x10,0x0B);
//	  Display_Led(0x10,0x0C);
//	  Display_Led(0x10,0x0D);
//	  Display_Led(0x10,0x0F);
//	  Display_Led(0x10,0x10);
//	  Display_Led(0x10,0x11);
//	  Display_Led(0x20,0xF1);
//	  Display_Led(0x40,0xF4);
//	  Display_Led(0x80,0xF8);

//	  Display_Led(0xF0,0xFD);
//
//	  Display_Led(0xF0,0x7F);

//	  Display_Led(9,1);
//	  send_data(0b00000000);
//	  HAL_Delay(1000);
//	  send_data(0b10000000);
//	  HAL_Delay(1000);
//	  send_data(0b01000000);
//	  HAL_Delay(1000);
//	  send_data(0b00100000);
//	  HAL_Delay(1000);
//	  send_data(0b00010000);
//	  HAL_Delay(1000);
//	  send_data(0b00001000);
//	  HAL_Delay(1000);
//	  send_data(0b00000100);
//	  HAL_Delay(1000);
//	  send_data(0b00000010);
//	  HAL_Delay(1000);
//	  send_data(0b00000001);
//	  HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DATA_Pin|CLCOK_Pin|LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DATA_Pin CLCOK_Pin LATCH_Pin */
  GPIO_InitStruct.Pin = DATA_Pin|CLCOK_Pin|LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //    __GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin   = GPIO_PIN_13;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
