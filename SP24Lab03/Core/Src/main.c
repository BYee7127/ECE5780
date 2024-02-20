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
void SetPins(void);
void SetTimers(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	SetTimers();

	SetPins();

  /* Infinite loop */
  while (1)
  {
  }
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

/* timer setup function */
void SetTimers(void){
	// enable the timer clocks
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// configure timer2 to trigger an update event at 4Hz (default processor frequency = 8MHz)
	TIM2->PSC = 7999;			// prescaler value = 7999 (3.1 Q2)
	TIM2->ARR = 250;			// auto-reload value = 250 (3.1 Q2)
	TIM2->DIER |= 0x00000001;			// enable the update interrupt (3.1 Q3)
	
	TIM2->CR1 |= TIM_CR1_CEN_Msk;			// enable the timer (3.1 Q4)

	// find the IRQn_Type for TIM2_IRQn, enable it (3.1 Q5)
	NVIC_EnableIRQ(TIM2_IRQn);
	
	// timer 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 		//(3.2 Q1)
	
	int targetClk = 800;
	int timerClk = 8000000;
	int prescl = 4;
	int autoreload = timerClk/((prescl + 1) * targetClk);
	
	TIM3->PSC = prescl;				// make it a reasonable range between 0-limit
	TIM3->ARR = autoreload;
	// do not enable update interrupt or setup the interrupt handler
	
	// 0x6878
	TIM3->CCMR1 |= 0x0008			// CC1S to ouptut & output compare preload enabled
							|  0x0070			// OC1M to PWM mode 2
							|  0x0800			// CC2S to output & output compare preload enabled
							|  0x6000;		// OC2M to PWM mode 1

	// enable the channel registers
	TIM3->CCER |= 0x01				// enable channel 1
						 |  0x10;				// enable channel 2
	
	// set channels to 20% of ARR value
	float counter = autoreload * 0.2;
	TIM3->CCR1 = counter;
	TIM3->CCR2 = counter;

	TIM3->CR1 |= TIM_CR1_CEN_Msk;
}

/**
  * Set the pins according to the lab instructions
  * Helper method so the main method doesn't look overcrowded
  */
void SetPins(void){
	// enable the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// set the general purpose output for the LEDs
	// bits = 01
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	// configure red and blue pins to be in alternate function mode
	GPIOC->MODER |= GPIO_MODER_MODER6_1;
	GPIOC->MODER |= GPIO_MODER_MODER7_1;

	// LEDs have push-pull output type = both bits cleared
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);

	// low speed = both bits cleared
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7);

	// no pull-up/down resistors = both bits cleared
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);

	// set green LED (PC9) to high
	GPIOC->ODR &= ~(GPIO_ODR_8);
	GPIOC->ODR |= GPIO_ODR_9;	
	
	// select the appropriate function numbers for said pins
	GPIOC->AFR[0] |= (0x00 << GPIO_AFRL_AFRL6_Pos) | (0x00 << GPIO_AFRL_AFRL7_Pos);	
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
