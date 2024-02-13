/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stm32f072xb.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void EXTI0_1_IRQHandler(void);
void TIM2_IRQHandler(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init(); 								// Reset of all peripherals, init the Flash and Systick 
	SystemClock_Config(); 			//Configure the system clock

	__HAL_RCC_GPIOC_CLK_ENABLE();			// Enable the GPIOC clock in the RCC
	__HAL_RCC_SYSCFG_CLK_ENABLE();		// enable the peripheral clock for the syscfg
	__HAL_RCC_GPIOA_CLK_ENABLE();			// enable the GPIOA clock to start configuring PA0

	// Set up a configuration struct to pass to the initialization function 
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7
														, GPIO_MODE_OUTPUT_PP
														, GPIO_SPEED_FREQ_LOW
														, GPIO_NOPULL};	
	HAL_GPIO_Init(GPIOC, &initStr); 													// Initialize pins 
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); 			// Start PC9 (orange) high 

	// configure PA0 to input mode at low-speed with an internal pulldown resister
	GPIO_InitTypeDef userInit;
	userInit.Pin = GPIO_PIN_0;
	userInit.Mode = GPIO_MODE_INPUT;
	userInit.Speed = GPIO_SPEED_FREQ_LOW;
	userInit.Pull = GPIO_PULLDOWN;
	// initialize it
	HAL_GPIO_Init(GPIOA, &userInit);
	
	// now work with EXTI0
	// enable/unmask = EXTI->IMR	
	EXTI->IMR |= EXTI_IMR_IM0;
	// rising edge trigger = EXTI_RTSR
	EXTI->RTSR |= EXTI_RTSR_RT0;
	
	// find the multiplexer that connects PA0 to EXTI0
	// EXTICR[0] accesses the EXTICR0 register
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

	// find the IRQn_Type for EXTI0 = EXTI0_1_IRQn
	// enable it
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	// set interrupt priority
	NVIC_SetPriority(EXTI0_1_IRQn, 3);
	
//-------------------------------------------------------------------	
		
	// enable timers (3.1 Q1)
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	
	// configure timer to trigger an update event at 4Hz (default processor frequency = 8MHz)
	TIM2->PSC |= 0x1F3F;			// prescaler value = 7999 (3.1 Q2)
	TIM2->ARR |= 0x00FA;			// auto-reload value = 250 (3.1 Q2)
	TIM2->DIER |= TIM_DIER_UIE;			// enable the update interrupt (3.1 Q3)
	
	TIM2->CR1 |= TIM_CR1_CEN;			// enable the timer (3.1 Q4)
	
	// find the IRQn_Type for TIM2_IRQn, enable it (3.1 Q5)
	NVIC_EnableIRQ(TIM2_IRQn);
	
	// set interrupt priority
	NVIC_SetPriority(TIM2_IRQn, 2);
	
//-------------------------------------------------------------------	
	
	NVIC_SetPriority(SysTick_IRQn, 2);

	while (1) {
		HAL_Delay(500); 			// Delay 200ms
		
		// Toggle the output state of PC6; this shows if the interrupt is working
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	}
}

/*
 * timer2 interrupt handler (3.1 Q5)
 */
void TIM2_IRQHandler(void){
	// clear the pending flag in SR (3.1 Q6)
	TIM2->SR = ~TIM_SR_UIF;

	// toggle the LEDs when the interrupt is triggered
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);

}

/*
 */
void EXTI0_1_IRQHandler(void){
	// clear the flag for EXTI0 
	EXTI->PR = EXTI_PR_PR0;

	// toggle the LEDs when the interrupt is triggered
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
	
	int count = 0;
	int limit = 2000000;
	while(count < limit){
		// delay loop
		count++;
	}
	
	// delay loop is finished, toggle again
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);

	// set count to 0
	count = 0;
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

