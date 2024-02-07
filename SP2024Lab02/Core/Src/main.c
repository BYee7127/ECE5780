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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock

	SetPins();

	// set green LED (PC9) to high
	GPIOC->ODR |= GPIO_ODR_6;
	GPIOC->ODR &= ~(GPIO_ODR_7);
	GPIOC->ODR &= ~(GPIO_ODR_8);
	GPIOC->ODR |= GPIO_ODR_9;
	
	// now work with EXTI0, interrupt connected to PA0
	// enable/unmask = EXTI->IMR	
	EXTI->IMR |= EXTI_IMR_IM0;
	// rising edge trigger = EXTI_RTSR
	EXTI->RTSR |= EXTI_RTSR_RT0;
	
	// use RCC to enable the peripheral for the system clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	// find the multiplexer that connects PA0 to EXTI0
	// EXTICR[0] accesses the EXTICR0 register
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	
	// find the IRQn_Type for EXTI0 = EXTI0_1_IRQn
	// enable it
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
	// set interrupt priority
	NVIC_SetPriority(EXTI0_1_IRQn, 3);
	//NVIC_SetPriority(EXTI0_1_IRQn, 1);
	NVIC_SetPriority(SysTick_IRQn, 2);
	
  while (1)
  {
		HAL_Delay(400);		// delay in ms
		
		// toggle red LED
		GPIOC->ODR ^= GPIO_ODR_6;
  }
}

/*
 * Interrupt handler for EXTI0
 */
void EXTI0_1_IRQHandler(void){
	// toggle green and orange LEDs when the interrupt is triggered
	GPIOC->ODR ^= GPIO_ODR_8;
	GPIOC->ODR ^= GPIO_ODR_9;
	
	// clear the flag for EXTI0 
	EXTI->PR = EXTI_PR_PR0;
	
	// add a loop to delay cause a long interrupt
	int delay = 0;
	
	while(delay < 2000000) {
		// iterate for 1-2 seconds
		delay++;
	}
	
	// toggle the LEDs again after the delay
	GPIOC->ODR ^= GPIO_ODR_8;
	GPIOC->ODR ^= GPIO_ODR_9;
	
	// clear count
	delay = 0;
}

/**
  * Set the pins according to the lab instructions
  * Helper method so the main method doesn't look overcrowded
  */
void SetPins(){
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
