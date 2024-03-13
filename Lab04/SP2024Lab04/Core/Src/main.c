/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Lab04 - UART
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void transmitCharacter(char c);
void redLED(void);

//char stringToSend[] = "Hello";
char stringToSend[2] = {'c'};
//int stringToSend[3] = {65, 66,'\0'};
int send = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

	redLED();
	
	// choose the pins to use for the USART - PC4 & PC5 (4.1 Q1 & Q2)
	// set the pins to alternate function mode (4.1 Q4)
	GPIOC->MODER |= GPIO_MODER_MODER4_1;		// TX, BROWN WIRE -> RX on USART
	GPIOC->MODER |= GPIO_MODER_MODER5_1;		// RX, RED WIRE -> TX on USART
	
	// set their alternate functions; AF1 on both pins (4.1 Q4)
	GPIOC->AFR[0] |= (0x01 << GPIO_AFRL_AFRL4_Pos);
	GPIOC->AFR[0] |= (0x01 << GPIO_AFRL_AFRL5_Pos);	

	// enable the RCC clock for USART3, using the system clock (4.9.2 Q1)
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// calculate the baud rate for 115200 bits/second (4.9.2 Q2)
	// BRR = processor clock / target baud rate 
	//	MAKE SURE TO USE SPEED = BRR IN THE PUTTY CONNECTION; otherwise it won't work :T
	USART3->BRR = HAL_RCC_GetHCLKFreq()/115200;
	
	// enable the transmitter and receiver hardware (4.9.2 Q3)
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE ;
	
	// enable the USART via UE bit in CR1 (4.9.2 Q4)
	USART3->CR1 |= USART_CR1_UE;

  while (1) {
		HAL_Delay(200);

		transmitCharacter('c');

		// read the character
/*		if ((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
			char chartoreceive = (uint8_t)(USART1->RDR); 		// Receive data, clear flag
		}		*/
		GPIOC->ODR ^= GPIO_ODR_6;
  }
}


/*
  Transmit a single character onto the USART (4.9.2)
	Pulled example from peripheral manual
 */
void transmitCharacter(char c){	
	// empty while loop
	while ((USART3->ISR & USART_ISR_TXE) != USART_ISR_TXE);

	USART3->TDR = c;
}


/*
	Turn on and toggle the red LED to see if it works (?)
*/
void redLED() {
	// enable the clock for the pins
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// LED modifications from lab1
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);

	GPIOC->ODR |= GPIO_ODR_6;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


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
