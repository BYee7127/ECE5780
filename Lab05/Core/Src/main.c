/**
******************************************************************************
* @file : main.c
* @brief : Main program body
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

void SystemClock_Config(void);
void setupLEDs(void);
void setupPins(void);
void setupI2C(void);
void partOne(void);
void flashLEDs(int times);


void setWrite(int byte) {
	// clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));	

	// set the slave address in CR2 (5.4 Q1)
	I2C2->CR2 = (byte << 16)			// number of bytes to transmit
						| (0x69 << 1);			// slave address
	
	I2C2->CR2 &= ~(1 << 10); 		// write transfer
	I2C2->CR2 |= I2C_CR2_START;		// start bit
}

/*
 * Wait for the TXIS or NACKF flag is set.
 * If the NACKF flag is set, then there is a wiring issue.
 * If the function leaves the while loop, then set the register
 */
int checkTXIS(char reg){
	while(1) {
		// exit when the TXIS flag is set
		if((I2C2->ISR & I2C_ISR_TXIS) == I2C_ISR_TXIS) {
			break;
		}
		else if((I2C2->ISR & I2C_ISR_NACKF) == I2C_ISR_NACKF) {
			return 1;
		}
	}
	
	I2C2->TXDR = reg;		// set the register
	return 0;
}

/*
 * Wait for the RXNE or NACKF flag is set.
 * If the NACKF flag is set, then there is a wiring issue.
 */
void setRead(int byte) {
	// clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));	

	// set the slave address in CR2 (5.4 Q1)
	I2C2->CR2 = (byte << 16)			// number of bytes to transmit
						| (0x69 << 1);			// slave address
	
	I2C2->CR2 |= (1 << 10); 		// enable read
	I2C2->CR2 |= I2C_CR2_START;		// start bit
}

int checkRXNE(){	
	while(1) {
		// exit when the RXNE flag is set
		if((I2C2->ISR & I2C_ISR_RXNE) == I2C_ISR_RXNE) {
			break;
		}
		else if((I2C2->ISR & I2C_ISR_NACKF) == I2C_ISR_NACKF) {
			return 1;
		}
	}

	return 0;
}

/*
 * Wait until the transfer is complete.
 * If it is not complete, then it hangs here.
 */
void checkTC(){
	while(1) {
		if((I2C2->ISR & I2C_ISR_TC) == I2C_ISR_TC) {
			break;
		}
	}
}

/*--main function-----------------------------------------------------------------------------*/
/**
* @brief The application entry point.
* @retval int
*/
int main(void) {
	HAL_Init();
	SystemClock_Config();

	setupLEDs();
	setupPins();
	setupI2C();
	
	partOne();

}

void partOne() {
	setWrite(1);
	
	// wait until the TXIS flag is set (5.4 Q2)
	// write the address for the WHO_AM_I register (5.4 Q3)
	if(checkTXIS(0x0F) == 1) {
		// error was returned, flash all LEDs then exit
		flashLEDs(2);
		return;
	}
	
	// wait until TC flag is set (5.4 Q4)
	checkTC();
	
	setRead(1);
	// reload the CR2 register to read and start it again (5.4 Q5)
	// wait until the RXNE flag is set (5.4 Q6)
	if(checkRXNE() == 1) {
		// error was returned, flash all LEDs then exit
		flashLEDs(3);
		return;
	}
	
	// wait until TC flag is set (5.4 Q7)
	checkTC();
	
	// check the RXDR contents to see if it matches the address WHO_AM_I register (5.4 Q8)
	if(I2C2->RXDR != 0xD3) {
		// if it's not right, flash LEDs
		flashLEDs(4);
	}
	
	// set the stop bit and release the I2C bus (5.4 Q9)
	I2C2->CR2 |= I2C_CR2_STOP;
	
	// flash once when successfully reaching here.
	flashLEDs(1);
}

/*--helper functions--------------------------------------------------------------------------*/
/*
 * Flashes all 4 LEDs as a means of debugging.
 *
 * It takes in an integer that tells the while loop how many times to
 * flash the LED.
 * 1 = everything good
 * 2+ = something went wrong
 */
void flashLEDs(int times) {
	int count = 0;
	while(count < times){
		GPIOC->ODR |= (GPIO_ODR_6);
		GPIOC->ODR |= (GPIO_ODR_7);
		GPIOC->ODR |= (GPIO_ODR_8);
		GPIOC->ODR |= (GPIO_ODR_9);

		HAL_Delay(200);
		
		GPIOC->ODR &= ~(GPIO_ODR_6);
		GPIOC->ODR &= ~(GPIO_ODR_7);
		GPIOC->ODR &= ~(GPIO_ODR_8);
		GPIOC->ODR &= ~(GPIO_ODR_9);

		count++;
		HAL_Delay(200);
	}
}

/*--setup functions----------------------------------------------------------------------------*/
/**/
void setupLEDs(){
	// enable the clock for the pins
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// LED modifications from lab1
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);

	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);

	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);
	
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);	

	GPIOC->ODR &= ~(GPIO_ODR_6);
	GPIOC->ODR &= ~(GPIO_ODR_7);
	GPIOC->ODR &= ~(GPIO_ODR_8);
	GPIOC->ODR &= ~(GPIO_ODR_9);
}

/**/
void setupPins() {
	// enable GPIOB and GPIOC in RCC (5.2 Q1)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

	// Set PB11 to alternate function/open-drain output type, I2C2_SDA (5.2 Q2)
	// AF1
	GPIOB->AFR[1] |= (0x01 << GPIO_AFRH_AFSEL11_Pos);
	GPIOB->MODER |= GPIO_MODER_MODER11_1; 		// set the alternate function mode
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;

	// AF5 PB13
	// Set PB13 to alternate function/open-drain output type, I2C2_SCL (5.2 Q3)
	GPIOB->AFR[1] |= (0x05 << GPIO_AFRH_AFSEL13_Pos);
	GPIOB->MODER |= GPIO_MODER_MODER13_1; 		// set the alternate function mode
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	
	// set PB14 to output mode, push-pull output type, set to high (5.2 Q4)
	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);
	GPIOB->ODR |= GPIO_ODR_14;

	// set PC0 to output mode, push-pull output type, set to high (5.2 Q5)
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0);
	GPIOC->ODR |= GPIO_ODR_0;
}

/**/
void setupI2C() {
	// enable i2c2 in RCC (5.3 Q1)
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// set timing parameters (5.3 Q2)
	I2C2->TIMINGR =  (0x1 << 28)		// prescaler = 1
								|  (0x4 << 20)		// SCLDEL = 0x4
								|  (0x2 << 16)		// SDADEL = 0x2
								|  (0xF << 8)		// SCLH = 0xF
								|  (0x13);				// CSLL = 0x13

	// Enable the I2C peripheral using the PE bit in the CR1 register (5.3 Q3)
	I2C2->CR1 |= I2C_CR1_PE;
}


/*--auto-generated functions-------------------------------------------------------------------*/
/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) 	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
															| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}


/**
* @brief This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void) {
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
}

#ifdef USE_FULL_ASSERT
/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
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
