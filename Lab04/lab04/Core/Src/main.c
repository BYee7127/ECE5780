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
#include "main.h"

void SystemClock_Config(void);
void transmitCharacter(char c);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

	// choose the pins to use for the USART - PC4 & PC5 (4.1 Q1 & Q2)
	// set these pins to alternate function mode 
	GPIOC->MODER |= GPIO_MODER_MODER4_1;		// TX, brown wire -> RX on USART
	GPIOC->MODER |= GPIO_MODER_MODER5_1;		// RX, red wire -> TX on USART
	
	// set them to their alternate functions; AF1 on both pins (4.1 Q4)
	GPIOC->AFR[0] |= (0X01 << GPIO_AFRL_AFRL4_Pos); 
	GPIOC->AFR[0] |= (0X01 << GPIO_AFRL_AFRL5_Pos); 
	
	// enable the RCC block for USART3, using the system clock (4.9.3 Q1)
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// calculate the baud rate for 115200 bits/second (4.9.2 Q2)
	// BRR = processor clock/target baud rate
	USART3->BRR = HAL_RCC_GetHCLKFreq()/115200;
	
	// enable the transmitter and the receiver hardware (4.9.2 Q3)
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
	
	// enable the USART via UE bit in CR1 (4.9.2 Q4)
	USART3->CR1 |= USART_CR1_UE;
	
  while (1)
  {
		// call the character transmit function with a character constant
		transmitCharacter(65);
		
		// add a delay
		HAL_Delay(300);
  }
}

/*
	Transmits a single character on the USART
	Checks and waits on the USART status flag and transmits a single character
	when the register is empty
*/
void transmitCharacter(char c) {
	while((USART3->ISR & USART_ISR_TXE) != USART_ISR_TXE){
		// do not use the busy bit 
		// exit the loop once the flag is set
	}
	
	// flag is set, transmit the character
	USART3->TDR = c;
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