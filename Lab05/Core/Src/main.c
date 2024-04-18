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


int doublei2c(char reg, volatile int16_t* read){ //collects two nums

	I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16));
	I2C2->CR2 |= (1<<16) //numbytes == 1
						| (0x69<<1); //slave address = 0x69
	I2C2->CR2 &= ~(1<<10); //write transfer (bit 10 is 0)
	I2C2->CR2 |= (1<<13);//start bit
		
	while(1){ //wait for TXIS
		GPIOC->ODR ^= (1<<7);//blue
		if ((I2C2->ISR & (1<<1))){ break;}
		else if (I2C2->ISR & I2C_ISR_NACKF) {
			//error
			return 1;
		}
		GPIOC->ODR ^= (1<<7);//blue
	}

	I2C2->TXDR = reg; //addr
	while (1){
		GPIOC->ODR ^= (1<<8);
		if (I2C2->ISR & I2C_ISR_TC) {break;} //wait until TC flag is set
		GPIOC->ODR ^= (1<<8);
	}

	//READ NOW.
	I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16));
	I2C2->CR2 |= (2<<16) //numbytes == 1
						| (0x69<<1); //slave address = 0x69
	I2C2->CR2 |= (1<<10); //READ transfer (bit 10 is 1)
	I2C2->CR2 |= (1<<13);//start bit set

	int8_t h, l;
	int16_t result;
	
	for(uint32_t i = 0; i < 2; i++){
		GPIOC->ODR ^= (1<<9);
		while (1){
			GPIOC->ODR ^= (1<<6);
			if (I2C2->ISR & I2C_ISR_RXNE){break;}
			else if (I2C2->ISR & I2C_ISR_NACKF) {
				//error
				return 1;
			}
		}
		if (i == 0){
			l = I2C2->RXDR;
		}
		else{
			h = I2C2->RXDR;
		}
	}

	while (1){
		GPIOC->ODR ^= (1<<9);
		if (I2C2->ISR & I2C_ISR_TC) {break;} //wait until TC flag is set
	}
	I2C2->CR2 |= (1<<14);//STOP
	GPIOC->ODR ^= (1<<6);
	
	GPIOC->ODR ^= (1<<6);
	GPIOC->ODR ^= (1<<7);
	GPIOC->ODR ^= (1<<8);
	GPIOC->ODR ^= (1<<9);
	
	result = (h << 8) | (l);
	*(read) = result;
	
	return 0;
}

/**/
int i2ctransfer(char reg, char info , volatile char* read){ //slave address = 0x69
	I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16));
	I2C2->CR2 |= (2<<16) //numbytes == 1
						| (0x69<<1); //slave address = 0x69
	I2C2->CR2 &= ~(1<<10); //write transfer (bit 10 is 0)
	I2C2->CR2 |= (1<<13);//start bit

	while(1){ //wait for TXIS
		GPIOC->ODR ^= (1<<7);//blue 
		if ((I2C2->ISR & (1<<1))){ break;}
		else if (I2C2->ISR & I2C_ISR_NACKF) {
			//error
			return 1;
		}
		HAL_Delay(200);
		GPIOC->ODR ^= (1<<7);//blue
	}

	I2C2->TXDR = reg; //addr

	while(1){ //wait for TXIS
		GPIOC->ODR |= (1<<8);
		if ((I2C2->ISR & (1<<1))){ break;}
		else if (I2C2->ISR & I2C_ISR_NACKF) {
			//error
			return 1;
		}
		HAL_Delay(200);
		GPIOC->ODR |= (1<<8);
	}
	
	I2C2->TXDR = info;
	
	while (1){
		GPIOC->ODR |= (1<<9);
		if (I2C2->ISR & I2C_ISR_TC) {break;} //wait until TC flag is set
		HAL_Delay(200);
		GPIOC->ODR |= (1<<9);
	}

	I2C2->CR2 &= ~((0x3FF << 0) | (0x7F << 16));
	I2C2->CR2 |= (1<<16) //numbytes == 1
						| (0x69<<1); //slave address = 0x69
	I2C2->CR2 |= (1<<10); //READ transfer (bit 10 is 1)
	I2C2->CR2 |= (1<<13);//start bit set
	
	while (1){
		GPIOC->ODR |= (1<<6);
		if (I2C2->ISR & I2C_ISR_RXNE){break;}
		else if (I2C2->ISR & I2C_ISR_NACKF) {
			//error
			return 1;
		}
		HAL_Delay(200);
		GPIOC->ODR |= (1<<6);
	}

	while (1){
		GPIOC->ODR ^= (1<<7);
		if (I2C2->ISR & I2C_ISR_TC){break;}
		HAL_Delay(200);
		//GPIOC->ODR ^= (1<<7);
	}
	
	*(read) = I2C2->RXDR;
	I2C2->CR2 |= (1<<14);//STOP

	GPIOC->ODR ^= (1<<6);
	GPIOC->ODR ^= (1<<8);
	GPIOC->ODR ^= (1<<9);
	
	return 0;
}

/**/
void initleds(){
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

void initgyro(){
	//gyro
	GPIOC->BSRR = (1<<0);
}

/**
* @brief The application entry point.
* @retval int
*/
int main(void) {
	HAL_Init();
	SystemClock_Config();

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	initleds();

	//pin b11
	GPIOB->MODER |= (1<<23 | 0<<22); //alternate function mode for pin B11
	GPIOB->OTYPER |= (1<<11);//open drain output on B11
	GPIOB->AFR[1] |= (1<<12); //set pin B11 to AF 1 (I2C2_SDA)

	//pin b13
	GPIOB->MODER |= (1<<27); //alternate function mode for pin b13
	GPIOB->OTYPER |= (1<<13);//open drain B13
	GPIOB->AFR[1] |= (1<<22 | 1<<20);//AF5 for pin b13 (i2c2_SCL)

	//pin b14
	GPIOB->MODER |= (1<<28);//output mode
	//pushpull is default
	GPIOB->ODR |= (1<<14);//initialize high

	GPIOB->PUPDR |= (1<<22 | 1<<26);

	//pin PC0
	GPIOC->MODER |= (1<<0); //PC0 output mode
	//pushpull is default
	GPIOC->ODR |= (1<<0);//initialize high


	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	//i2c2
	I2C2->TIMINGR = (1<< 28)//PRESCALER == 1
								| (0x4 << 20) // SCLDEL == 0x4
								| (0x2<<16) //SDADEL == 0x2
								| (0xF<<8)//SCLH == 0xF
								| (0x13<<0);//8SCLL == 0x13
	I2C2->CR1 |= (1<<0); //PE bit

	//initgyro();

	volatile char read, info, reg;

	reg = 0x20;
	info = 0x0B; 

	//i2ctransfer(reg, info, &read);//reg info read

//////////////////////////////////
	//main loop
	while (1)
	{
		HAL_Delay(100);
		volatile int16_t x, y;
		//collect x
		doublei2c(0xA8, &x); //reg read
		//collect y
		doublei2c(0xAA, &y);
		//decide lights
		const int16_t thresh = 0x01FF;
		if (x < 0-thresh) {GPIOC->ODR |= (1<<8);} else {GPIOC->ODR &= ~(1<<8);} //orange
		if (y < 0-thresh) {GPIOC->ODR |= (1<<7);} else {GPIOC->ODR &= ~(1<<7);} //blue
		if (y > thresh) {GPIOC->ODR |= (1<<6);} else {GPIOC->ODR &= ~(1<<6);} //red
		if (x > thresh) {GPIOC->ODR |= (1<<9);} else {GPIOC->ODR &= ~(1<<9);} //green
	}
}



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
