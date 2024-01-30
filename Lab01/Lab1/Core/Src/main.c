/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stm32f072xb.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SetPins();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock

	SetPins();

	// write 1 pin high, write other pin low using either ODR or BSRR register
	GPIOC->ODR |= GPIO_ODR_6;
	GPIOC->ODR &= ~(GPIO_ODR_7);
	GPIOC->ODR &= ~(GPIO_ODR_8);
	GPIOC->ODR &= ~(GPIO_ODR_9);
	
	uint32_t debounce = 0;
	while (1) {
		// monitor the button pin input state using IDR register
		// the button pin is on 0
		
		// make sure to do the debouncing as well
		debounce = (debounce << 1);			// shift every loop iteration
		
		// if the input signal is set/high
		if(GPIOA->IDR & 0x1) {
			// set the lowest bit of bit-vector
			debounce |= 0x01;
		}
		
		// this number only triggers once while 0xffffffff and 0x00000000 triggers repeatedly
		if(debounce == 0x7FFFFFFF) {
			// if the button is pressed, switch the LED 
			if(GPIOC->ODR & GPIO_ODR_6){ 
				GPIOC->ODR |= GPIO_ODR_7;
				GPIOC->ODR &= ~(GPIO_ODR_6);
				GPIOC->ODR &= ~(GPIO_ODR_8);
				GPIOC->ODR &= ~(GPIO_ODR_9);
			}
			else if(GPIOC->ODR & GPIO_ODR_7){ 
				GPIOC->ODR |= GPIO_ODR_8;
				GPIOC->ODR &= ~(GPIO_ODR_7);
				GPIOC->ODR &= ~(GPIO_ODR_6);
				GPIOC->ODR &= ~(GPIO_ODR_9);
			}
			else if(GPIOC->ODR & GPIO_ODR_8){ 
				GPIOC->ODR |= GPIO_ODR_9;
				GPIOC->ODR &= ~(GPIO_ODR_8);
				GPIOC->ODR &= ~(GPIO_ODR_6);
				GPIOC->ODR &= ~(GPIO_ODR_7);
			}
			else if(GPIOC->ODR & GPIO_ODR_9){ 
				GPIOC->ODR |= GPIO_ODR_6;
				GPIOC->ODR &= ~(GPIO_ODR_9);
				GPIOC->ODR &= ~(GPIO_ODR_7);
				GPIOC->ODR &= ~(GPIO_ODR_8);
			}
		}
	}
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

	// set the general purpose output for blue and red LEDs, PC6 and PC7 respectively
	// bits = 01
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;

	// both LEDs have push-pull output type = both bits cleared
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
	
	// configure the user button
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

