/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

#include "HD44780i2c.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

//Pin map of PCF8574A 8-Bit I/O-Expander
#define slave_adress_read 0x4F //adress to read from slave / 01001111
#define slave_adress_write 0x4E //adress to write to slave 	/ 01001110


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  	//show LCD id on screen
  	lcd_detect(&hi2c1);

  	uint8_t testaddress = 0x4E;
  	//initialize LCD
  	lcd_init(&hi2c1,testaddress);
  	lcd_setSize(2,16);

  	//testprint on LCD and set cursor start position
  	lcd_print2("Knuti",1,6);


   	HAL_Delay(1000); //1s delay to let lcd start up
   	//create data to set

   	uint16_t timeout = 0xFF;
   	uint8_t data1 = 0x07;
   	uint8_t data2 = 0xff;
   	uint8_t data[1];
   	uint8_t data_out[1];
   	uint8_t data_out2[2];
   	data[0] = 0xAA;
   	data_out[0] = 0x05;
   	data_out2[0] = 0x00;
   	data_out2[1] = 0x00;
   	uint8_t data_rec = 0xAA;
   	//write to register
   	HAL_Delay(1000);

   	//---------------------------------------------------------------------------------------------
   	init_lcd();
   	float temp = 23.8;


   	char buffer[16];
   	sprintf(buffer,"%f",temp);

   	char str[16];
   	float adc_read = 23.8;

   	char *tmpSign = (adc_read < 0) ? "-" : "";
   	float tmpVal = (adc_read < 0) ? -adc_read : adc_read;

   	int tmpInt1 = tmpVal;                  // Get the integer (678).
   	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
   	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

   	// Print as parts, note that you need 0-padding for fractional bit.

   	//sprintf (str, "Temp =%s%d.%02d\ntest", tmpSign, tmpInt1, tmpInt2);


   	lcd_print("Schaaaatz",0,4);

   	lcd_print("Lovilove  <3",1,0);
   	//---------------------------------------------------------------------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
  {
//	  	//reset readout values
//	  	data[0] = 0xAA;
//	  	result = HAL_ERROR;
//	  	result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, &data1, 1, timeout);
//	 	result = HAL_ERROR;
//	   	result = HAL_I2C_Master_Receive(&hi2c1, slave_adress_write, data, 1, timeout);
//
//	   	HAL_Delay(1000);
//
//		 result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, &data2, 1, timeout);
//
//		 HAL_Delay(1000);
//		 result = HAL_I2C_Master_Receive(&hi2c1, slave_adress_write, data, 1, timeout);
//
//		 result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, data_out, 1, timeout);
//		 result = HAL_I2C_Master_Receive(&hi2c1, slave_adress_write, data, 1, timeout);





	   	//result = HAL_ERROR;
	   	//result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, &data2, 1, timeout);

	    //result = HAL_ERROR;
	   	//result = HAL_I2C_Master_Receive(&hi2c1, slave_adress_write, data_out2, 2, timeout);


	   	//HAL_Delay(2000);
  /* USER CODE END WHILE */
	  //send command via I2C
//	  HAL_I2C_Master_Transmit (&hi2c1, i2c_adress , (uint8_t *) data_u, i2c_nBytes, i2c_timeout);
//
//	  HAL_Delay(2000);	// microcontroller wait 2 sec;
//
//	  HAL_I2C_Master_Transmit (&hi2c1, i2c_adress , (uint8_t *) data_l, i2c_nBytes, i2c_timeout);
//
//	  HAL_Delay(2000);
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  //Initialize struct
  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
//void LCD_init()
//{
//
//	//Reset from any state into 8-Bit mode
//	HAL_Delay(20);		//wait 50 ms after Power on
//	HAL_I2C_Master_Transmit(&hi2c1, 0x4E, 0x33, 1, 0xFF);
//	HAL_Delay(5);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x4E, 0x33, 1, 0xFF);
//	HAL_Delay(5);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x4E, 0x32, 1, 0xFF);
//	HAL_Delay(5);
//
//	// Change to 4-bit mode
//	HAL_I2C_Master_Transmit(&hi2c1, 0x4E, 0x28, 1, 0xFF);
//	HAL_Delay(5);
//
//	// Clear the LCD
//	HAL_I2C_Master_Transmit(&hi2c1, 0x4E, 0x0E, 1, 0xFF);
//
//	//Display on & Show cursor
//	HAL_I2C_Master_Transmit(&hi2c1, 0x4E, 0x01, 1, 0xFF);
//
//	//Increment Cursor
//	HAL_I2C_Master_Transmit(&hi2c1, 0x4E, 0x06, 1, 0xFF);
//
//	//Set Display (rows, columns) -> 1 row, 1 column
//	HAL_I2C_Master_Transmit(&hi2c1, 0x4E, 0x80, 1, 0xFF);
//}

void writeByteWithClock(uint8_t RS, uint8_t RW, uint8_t BL, uint8_t data)
{
   	uint8_t temp 		= data;    	//init temp = data and overwrite lower nibble
   	uint16_t timeout 	= 0xFF; 	//i2c timeout 255ms
   	uint16_t nBytes		= 1;		//number of bytes to send over i2c-bus
   	HAL_StatusTypeDef result = HAL_ERROR;

   	//get upper nibble from data
   	temp &= 0xF0;
   	//set non data pins
   	temp |= ( (RS<<Pin_RS) | (RW<<Pin_RW) | (BL<<Pin_BL) | (EN_UP<<Pin_EN) ); //set as sent RS, RW, BL, and CLK high

   	//send temp-byte containing upper nibble
   	result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, &temp, nBytes, timeout);
	HAL_Delay(5);

   	//send upper nibble again with CLK-pin pulled down
   	temp = temp & ~(1<<Pin_EN);
	result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, &temp, nBytes, timeout);
	HAL_Delay(5);

   	//get lower nibble from data
   	temp &= 0x0F;                               //clear upper nibble and retain lower nibble (RS, RW, etc)
   	temp |= ( (data<<4) | (1<<Pin_EN) ) ;      //set upper nibble same as data && drive CLK high

   	//send upper nibble
	result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, &temp, nBytes, timeout);
	HAL_Delay(5);
	//resend lower nibble with CLK-Pin pulled down
   	temp = temp & ~(1<<Pin_EN);
	result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, &temp, nBytes, timeout);
	HAL_Delay(5);


}

void writeNibbleWithClock(uint8_t RS, uint8_t RW, uint8_t BL, uint8_t data)
{
	//uint8_t temp = data;    //init temp = data and overwrite lower nibble
	uint16_t timeout 	= 0xFF; 	//i2c timeout 255ms
	uint16_t nBytes		= 1;		//number of bytes to send over i2c-bus
   	HAL_StatusTypeDef result = HAL_ERROR;

   	uint8_t lo_nib;
   	uint8_t hi_nib;
   	uint8_t combined_nib = 0x00;

   	lo_nib &= 0x0f; 														//clear lower nibble
   	hi_nib &= 0xf0;															//clear upper nibble
   	hi_nib = (data<<4);														//shift data to upper nibble
   	lo_nib |= (RS<<Pin_RS) | (RW<<Pin_RW) | (1<<Pin_EN) | (BL<<Pin_BL); 	//set bits in lower nibble

   	//combined_nib = (data << 4) | lo_nib;
   	combined_nib = hi_nib | lo_nib;											//combine nibbles to byte

   	uint16_t slave_adress_write2  = 0x4E;

	result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write2, &combined_nib, nBytes, timeout);
	HAL_Delay(5);
	combined_nib &= ~(1<<Pin_EN);
	result = HAL_I2C_Master_Transmit(&hi2c1, slave_adress_write, &combined_nib, nBytes, timeout);
	HAL_Delay(5);
}

void init_lcd()
{

	//Pin B5 connected to VDD on LCD
	//power lcd down and wait
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	//power lcd up and wait
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

	//Wait for stable VDD-voltage and device power up
	HAL_Delay(1000);

	//Note on HD44780 with I2C-expander the pins (D0,D1,D2,D3 = GND -> 0)

	//Reset lcd mode from 8- or 4-bit mode						//8-bit mode									//4-bit mode waiting for first nibble			//4-bit mode waiting for second nibble
	writeNibbleWithClock(COMMAND, LCD_WRITE, BL_ON, 0x3);		//0b0011000 -> reset to 8 bit mode				// 0b0011 = first nibble						// 0b0011 = second nibble
	HAL_Delay(500);																																				// 0b****0011 -> some random command

	writeNibbleWithClock(COMMAND, LCD_WRITE, BL_ON, 0x3);		//0b0011000 -> reset to 8 bit mode				// 0b0011 = second nibble 						// 0b0011 = first nibble
	HAL_Delay(200);																								// 0b00110011 combined -> go to 8-bit mode
	writeNibbleWithClock(COMMAND, LCD_WRITE, BL_ON, 0x3);		//0b0011000 -> reset to 8 bit mode				//0b0011000 -> reset to 8 bit mode				// 0b0011 = second nibble
	HAL_Delay(500);																																				// 0b00110011 combined -> go to 8-bit mode


	//all three possible states have been reset to 8-bit mode
	writeNibbleWithClock(COMMAND, LCD_WRITE, BL_ON, 0x2);		//0b0010000 -> 8-bit command to switch to 4-bit mode
	HAL_Delay(500);

	//entered 4-bit mode
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x28); 		//enter into 4-Bit mode
	HAL_Delay(500);
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x28); 		//enter into 4-Bit mode
	HAL_Delay(500);
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x28); 		//enter into 4-Bit mode
	HAL_Delay(500);

	//Set lcd parameters in 4-bit mode
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x01); 		//clear lcd go to pos 0
	HAL_Delay(500);
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x06); 		//increment left, no shift
	HAL_Delay(500);
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x0D); 		//display on, hide cursor, blink on
	HAL_Delay(500);

/*
	// entered 4-Bit mode from 4- or 8-bit mode


	//Function set (LCD Dimensions)
	// 	0 0 1 DL N F * * -> DL = 0(4-Bit interface) 1(8-Bit interface) -> N = 0(1 Line) 1(2 Lines) -> F = 0(5x8 dots per character) 1(5x10 dots per character)
	//0b0 0 1 0  1 0 0 0 -> 0x28
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x28);           //N=1 (2 line), F=0 (5x8)
	HAL_Delay(100);
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x28);           //N=1 (2 line), F=0 (5x8)
	HAL_Delay(100);


	//clear display
	// 0 0 0 0 0 0 0 0 1
	// 0b00000001 -> 0x01
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x01);           //clear
	HAL_Delay(10);

	//Entry mode set
	// 0 0 0 0 0 0 0 1 I/D S -> I/D = 0(decrement left) 1(increment right) -> S = 0(display shift off) 1(display shift on)
	// 0b00000110 -> 0x06
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x06);           //ID=1(increment), S=0 (no shift)
	HAL_Delay(10);

	//display on/of control:
	// 0 0 0 0 0 0 1 D C B  -> D = 0(display off) 1(display on) -> C = 0(hide cursor) 1(show cursor) -> B = 1(blink on) 0(blink off)
	// 0b0000001101 -> 0x08
	//writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, 0x0D);           //Display on, Cursor on, Blink off
	HAL_Delay(10);
*/
}


extern void lcd_print(char string[], uint8_t y, uint8_t x)
{
	//16x2LCD
	uint8_t lcd_format_y = 2;
	uint8_t lcd_format_x = 16;

	//check if y exceeds lcd bounds
	if(y<0 || y > (lcd_format_y -1))
	{
		y=0;
	}

	//check if x exceeds lcd bounds
	if(x<0 || x> (lcd_format_x -1))
	{
		x=0;
	}

	//adresses to first characters in line
	uint8_t line_start_adress[4];
	line_start_adress[0]= 0b10000000;
	line_start_adress[1]= 0b11000000;
	line_start_adress[2]= 0b10010100;
	line_start_adress[3]= 0b11010100;

	//calculate cursor adress from (x / y)
	uint8_t cursor_adress = line_start_adress[y] + x;

	//set cursor position
	//writeNibbleWithClock(COMMAND, LCD_WRITE, BL_ON, 0b1100);
	//writeNibbleWithClock(COMMAND, LCD_WRITE, BL_ON, 0b0000);
	writeByteWithClock(COMMAND, LCD_WRITE, BL_ON, cursor_adress);

	//write string to that position
    for(uint8_t i = 0; string[i] != 0; i++)
    {
    		writeByteWithClock(DATA, LCD_WRITE, BL_ON, string[i]);
    }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
