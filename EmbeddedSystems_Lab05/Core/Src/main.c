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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
void Write_Setup(void){
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// set timingr to 100khz
	// I2C2->TIMINGR = 0x13;
	I2C2->TIMINGR = (1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);
	// set I2C2 to CR1
	I2C2->CR1 |= I2C_CR1_PE;
	I2C2->CR2 |= (0x69<<1);
	I2C2->CR2 |= (2<<16); // nbytes is 16-23, set bit to 1
	I2C2->CR2 &=~ (1<<10); // RD_wrn , write = 0
	I2C2->CR2 |= (1<<13); // enable start bit
}

void Read_Setup(void){
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// set timingr to 100khz
	// I2C2->TIMINGR = 0x13;
	I2C2->TIMINGR = (1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);
	// set I2C2 to CR1
	I2C2->CR1 |= I2C_CR1_PE;
	I2C2->CR2 |= (0x69<<1);
	I2C2->CR2 |= (2<<16); // nbytes is 16-23, set bit to 1
	I2C2->CR2 |= (1<<10); // RD_wrn , write = 0
	I2C2->CR2 |= (1<<13); // enable start bit
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int test;
int Angle;
int x_low;
int x_high; 
int x; 
int y_low;
int y_high;
int y;
/* USER CODE END PD */

void TXIS_Flag(void){
	while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))){   // Wait until either TXIS or NACKF flags are set
    // GPIOC->ODR ^= (1<<7);    
		if(I2C2->ISR & I2C_ISR_NACKF) {      
            // ErrorLoopGreen(100); // Error condition, no ACK
					GPIOC->ODR |= (1<<6);
        }
			}
}

void TC_Flag(void){
	while (1){ // transfer complete flag
		if (I2C2->ISR & (1<<6)){
			break;
		}
	}
}

void RXNE_Flag(void){
	// test = 1;
	while (1){
		if (I2C2->ISR & (1<<4)){ // NACKF flag - bad
			//  (if this happens wires are probably bad) 		
		}
		if (I2C2->ISR & (1<<2)){ // RXNE flag - good
			break;
		}	
	}
}
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	void ErrorLoop(uint16_t rate) {
     while(1) {
        GPIOC->ODR ^= (1 << 6);    // Flash RED LED infinitely 
        HAL_Delay(rate);
    }
}

int main(void)
{
	//system code
  HAL_Init();
  SystemClock_Config();
	// enable GPIOB and C
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// enable the I2C2 in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	// set PB11 to alternate function mode, output drain function type, I2C2_SDA
	GPIOB->MODER |= (1<<23); // alternate function mode = 10
	GPIOB->MODER &=~ (1<<22); // PB11 = 22,23
	GPIOB->OTYPER |= (1<<11); // output drain = 1, PB11 = 11
	GPIOB->AFR[1] |= (1<<12);// AF1 -> 0001, 15 14 13 12
	GPIOB->AFR[1] &=~ ((1<<15)|(1<<14)|(1<<13));
	
	// set PB13 to alternate function mode, output drain function type, I2C2_SDL
	GPIOB->MODER |=   (1<<27); // alternate function mode = 10
	GPIOB->MODER &=~  (1<<26); // PB13 = 26,27
	GPIOB->OTYPER |=  (1<<13); // output drain = 1, PB13 = 13	
	GPIOB->AFR[1] |=  ((1<<22)|(1<<20)); // AF5 -> 0101, 23 22 21 20
	GPIOB->AFR[1] &=~ ((1<<23)|(1<<21));
	
	// set PB14 to output mode, push pull output, high 
	GPIOB->MODER |= (1<<28); // output mode = 01
	GPIOB->MODER &=~ (1<<29); // PB14 = 28,29
	GPIOB->OTYPER &=~ (1<<14); // push pull = 0, PB14 = 14
	GPIOB->ODR |= (1<<14); // set pin to high 
	
	// set PC0 to output mode, push pull output, high 
	GPIOC->MODER |= (1<<0); // output mode = 01
	GPIOC->MODER &=~ (1<<1); // PC0 = 0,1
	GPIOC->OTYPER &=~ (1<<0); // push pull = 0, PC0 = 0
	GPIOC->ODR |= (1<<0);	// set pin to high 
		// led testing 
	GPIOC->MODER &=~(1<<15); // clear
	GPIOC->MODER |= (1<<14); // enable 
	GPIOC->MODER &=~(1<<13); // clear
	GPIOC->MODER |= (1<<12); // enable
	GPIOC->MODER &=~(1<<19); // clear
	GPIOC->MODER |= (1<<18); // enable 
	GPIOC->MODER &=~(1<<17); // clear
	GPIOC->MODER |= (1<<16); // enable
	// led: 6-red 7-blue 8-orange 9-green
	// GPIOC->ODR |= (1<<6); //visual indictation 
	

	// set timingr to 100khz
	// I2C2->TIMINGR = 0x13;
	I2C2->TIMINGR = (1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);    // Set the timing register from table
	// set I2C2 to CR1
	I2C2->CR1 |= I2C_CR1_PE;
	// set up transcation params
	I2C2->CR2 |= (0x69<<1);
	I2C2->CR2 |= (1<<16); // nbytes is 16-23, set bit to 1
	I2C2->CR2 &=~ (1<<10); // RD_wrn , write = 0
	I2C2->CR2 |= (1<<13); // enable start bit

	
	// Initializing the gyroscope 
	// enable the x and y sensing in the CTRL_REG1 register 
	// default address is 00000111
	// set up transcation params
	Write_Setup();
	TXIS_Flag();
	// write the control register 1 address 
	I2C2->TXDR = 0x20;
	TXIS_Flag();
	// write the control register 1 address 
	I2C2->TXDR = 0x0B; // 0000 1011 
	TC_Flag();
	I2C2->CR2 |= (1<<14);
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		x = 0;
		HAL_Delay(100); // gyroscope reads at 95hz
		// write a transaction to I2C2->CR2
		Write_Setup(); // write setup
		TXIS_Flag(); // check for Txis flag
		I2C2->TXDR = 0xA8; // write to the 0xA8 for two byte x  
		TC_Flag(); // check for transfer complete
		// breaking here, TC flag not setting
		GPIOC->ODR |=  (1<<8);
		Read_Setup(); // read setup
		RXNE_Flag(); // check for RXNE flag
		x_low = I2C2->RXDR;	// check the RXDR register for x data 
		RXNE_Flag(); // check for RXNE flag
		x = (I2C2->RXDR << 8) | x_low; // Save upper byte and add to lower byte
		TC_Flag();

		
		// write to the 0xAA for y  
		/*
		Write_Setup();  
		I2C2->TXDR = 0xAA;
		TXIS_Flag();
		// step 3 start a new read transaction 
		Read_Setup();
		// step 4 read the data in the IC2C->RXDR register 
		RXNE_Flag();
		GPIOC->ODR |=  (1<<8);
		// GPIOC->ODR |=  (1<<6);
		// check the RXDR register for y data 
		y_low = I2C2->RXDR;
		// check the RXDR register for x data 
		//Read_Setup();
		RXNE_Flag();
		//x_high = I2C2->RXDR;
		y = (I2C2->RXDR << 8) | y_low; // Save upper byte
		TC_Flag();		
	*/
	
		// set the stop bit 
		I2C2->CR2 |= (1<<14);
		// rotation LEDS
		if (x>1000){
			GPIOC->ODR |=  (1<<7);
			GPIOC->ODR &=~ (1<<6);
		}
		else if (x<-1000) {
			GPIOC->ODR |=  (1<<6);
			GPIOC->ODR &=~ (1<<7);
		}
		//else{
			//GPIOC->ODR &=~ (1<<6);
			//GPIOC->ODR &=~ (1<<7);
		//}
		/*
		if (y>0){
			GPIOC->ODR |=  (1<<8);
			GPIOC->ODR &=~ (1<<9);
		}
		else if (y<0) { 
			GPIOC->ODR |=  (1<<9);
			GPIOC->ODR &=~ (1<<8);
		}
		else{
			GPIOC->ODR &=~ (1<<6);
			GPIOC->ODR &=~ (1<<7);
		}
		*/
		// set the stop bit 
		// I2C2->CR2 |= (1<<14);
		// I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		
  }
  /* USER CODE END 3 */
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
