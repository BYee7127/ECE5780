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
void SetPins();
void SetTimers();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock

	SetPins();

	SetTimers();
	
	// find the IRQn_Type for TIM2_IRQn, enable it (3.1 Q5)
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1);
	
  while (1)
  {
//		HAL_Delay(400);		// delay in ms
		
		// toggle red LED
		//GPIOC->ODR ^= GPIO_ODR_6;
  }
}

/* timer setup function */
void SetTimers(void){
	// enable the timer clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// configure timer to trigger an update event at 4Hz (default processor frequency = 8MHz)
	TIM2->PSC |= 0x1F3F;			// prescaler value = 7999 (3.1 Q2)
	TIM2->ARR |= 0x00FA;			// auto-reload value = 250 (3.1 Q2)
	TIM2->DIER |= 0x00000001;			// enable the update interrupt (3.1 Q3)
	
	TIM2->CR1 |= 0x00000001;			// enable the timer (3.1 Q4)

	// find the IRQn_Type for TIM2_IRQn, enable it (3.1 Q5)
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1);
}

/*
 * timer2 interrupt handler (3.1 Q5)
 */
void TIM2_IRQHandler(void){
		// toggle green and orange LEDs when the interrupt is triggered
	GPIOC->ODR ^= GPIO_ODR_8;
	GPIOC->ODR ^= GPIO_ODR_9;

	// clear the pending flag in SR (3.1 Q6)
	TIM2->SR &= ~0x00000001;
}

/**
  * Set the pins according to the lab instructions
  * Helper method so the main method doesn't look overcrowded
  */
void SetPins(void){
	// enable the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// enable the GPIOA clock for the user button
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// set the general purpose output for the LEDs
	// bits = 01
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;

	// LEDs have push-pull output type = both bits cleared
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);

	// low speed = both bits cleared
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);

	// no pull-up/down resistors = both bits cleared
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);
	
	// set green LED (PC9) to high
	GPIOC->ODR &= GPIO_ODR_6;
	//GPIOC->ODR &= ~(GPIO_ODR_7);
	GPIOC->ODR &= ~(GPIO_ODR_8);
	GPIOC->ODR |= GPIO_ODR_9;	
	
	// configure the user button, input-mode @ low speed and pull down resistor
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
