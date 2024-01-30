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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	HAL_Init();		 // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config();		 //Configure the system clock

	// enable the GPIOC clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// enable the GPIOA clock for the user button
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// set up red led (PC6) to low-speed, push-pull output, no push/pull resistor
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
	
	// set up blue led (PC7) to low-speed, push-pull output, no push/pull resistor
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);

	// set up green led (PC8) to low-speed, push-pull output, no push/pull resistor
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);

	// set up orange led (PC9) to low-speed, push-pull output, no push/pull resistor
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);
	
	// set up the user button, labeled B1 on the board, PA0 in the manual
	// user button needs to be in input-mode, low-speed, and pull-down register
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
	
	// write 1 pin high, write other pin low using either ODR or BSRR register
	// start with 6 high and the others down
	GPIOC->ODR |= GPIO_ODR_6;
	GPIOC->ODR &= ~(GPIO_ODR_7);
	GPIOC->ODR &= ~(GPIO_ODR_8);
	GPIOC->ODR &= ~(GPIO_ODR_9);
	
	// debounce variable to prevent the system from reading "multiple" presses
	uint32_t debouncer = 0;
	
	while (1) {
		// following code provided in the lab about the debouncer
		debouncer = (debouncer << 1); // Always shift every loop iteration
		
		// check the IDR register of the button
		if (GPIOA->IDR & 0x1) {
			// necessary to use the 0x1 part???
			// when the IDR register is set, set the lowest bit of the bit-vector
			debouncer |= 0x01;
		}
		
		// only trigger the switch once
		if(debouncer == 0x7FFFFFFF) {
			// if it's either all high or all low, then it will be triggered repeatedly
			// toggle the states of the LEDs
			switch(GPIOC->ODR) {
				case GPIO_ODR_6:
					GPIOC->ODR &= ~(GPIO_ODR_6);
					GPIOC->ODR |= GPIO_ODR_7;
					GPIOC->ODR &= ~(GPIO_ODR_8);
					GPIOC->ODR &= ~(GPIO_ODR_9);
					break;
				case GPIO_ODR_7:
					GPIOC->ODR &= ~(GPIO_ODR_6);
					GPIOC->ODR &= ~(GPIO_ODR_7);
					GPIOC->ODR |= GPIO_ODR_8;
					GPIOC->ODR &= ~(GPIO_ODR_9);
					break;
				case GPIO_ODR_8:
					GPIOC->ODR &= ~(GPIO_ODR_6);
					GPIOC->ODR &= ~(GPIO_ODR_7);
					GPIOC->ODR &= ~(GPIO_ODR_8);
					GPIOC->ODR |= GPIO_ODR_9;
					break;
				case GPIO_ODR_9:
					GPIOC->ODR |= GPIO_ODR_6;
					GPIOC->ODR &= ~(GPIO_ODR_7);
					GPIOC->ODR &= ~(GPIO_ODR_8);
					GPIOC->ODR &= ~(GPIO_ODR_9);
					break;
			}
		}
	}
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
