/*
 * HD44780_i2c.c
 */

#ifndef HD44780I2C_C_
#define HD44780I2C_C_

#include "HD44780i2c.h"

void lcd_setSize(uint8_t x_max, uint8_t y_max)
{
	_x_max = x_max;
	_y_max = y_max;
}

void lcd_clock(uint8_t nibble)
{

}

void lcd_writeNibble(uint8_t RS, uint8_t RW, uint8_t BL, uint8_t nibble)
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
   	hi_nib = (nibble<<4);													//shift data to upper nibble
   	lo_nib |= (RS<<Pin_RS) | (RW<<Pin_RW) | (1<<Pin_EN) | (BL<<Pin_BL); 	//set bits in lower nibble

   	//combined_nib = (data << 4) | lo_nib;
   	combined_nib = hi_nib | lo_nib;											//combine nibbles to byte

   	uint16_t slave_adress_write2  = 0x4E;

	result = HAL_I2C_Master_Transmit(hi2c, dev_address, &combined_nib, nBytes, timeout);
	HAL_Delay(5);
	combined_nib &= ~(1<<Pin_EN);
	result = HAL_I2C_Master_Transmit(hi2c, dev_address, &combined_nib, nBytes, timeout);
	HAL_Delay(5);
}

void lcd_writeByte(uint8_t RS, uint8_t RW, uint8_t BL, uint8_t data)
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
   	result = HAL_I2C_Master_Transmit(hi2c, dev_address, &temp, nBytes, timeout);
	HAL_Delay(5);

   	//send upper nibble again with CLK-pin pulled down
   	temp = temp & ~(1<<Pin_EN);
	result = HAL_I2C_Master_Transmit(hi2c, dev_address, &temp, nBytes, timeout);
	HAL_Delay(5);

   	//get lower nibble from data
   	temp &= 0x0F;                               //clear upper nibble and retain lower nibble (RS, RW, etc)
   	temp |= ( (data<<4) | (1<<Pin_EN) ) ;      //set upper nibble same as data && drive CLK high

   	//send upper nibble
	result = HAL_I2C_Master_Transmit(hi2c, dev_address, &temp, nBytes, timeout);
	HAL_Delay(5);
	//resend lower nibble with CLK-Pin pulled down
   	temp = temp & ~(1<<Pin_EN);
	result = HAL_I2C_Master_Transmit(hi2c, dev_address, &temp, nBytes, timeout);
	HAL_Delay(5);
}

void lcd_init(I2C_HandleTypeDef *i2c, uint8_t address)
{
	dev_address = address;
	hi2c		= i2c;
	HAL_Delay(1000);	//1000ms to let VDD stabalize and LCD to power up
	//Note on HD44780 with I2C-expander the pins (D0,D1,D2,D3 = GND -> 0)

	//wait for i2c ready
	HAL_I2C_StateTypeDef status = HAL_ERROR;
	while(status != HAL_OK)
	{
		HAL_Delay(5);
		status =  HAL_I2C_IsDeviceReady(hi2c, dev_address, 2, 200);
	}

	//Reset lcd mode from 8- or 4-bit mode						//8-bit mode									//4-bit mode waiting for first nibble			//4-bit mode waiting for second nibble
	lcd_writeNibble(COMMAND, LCD_WRITE, BL_ON, 0x3);		//0b0011000 -> reset to 8 bit mode				// 0b0011 = first nibble						// 0b0011 = second nibble
	HAL_Delay(500);																																				// 0b****0011 -> some random command

	lcd_writeNibble(COMMAND, LCD_WRITE, BL_ON, 0x3);		//0b0011000 -> reset to 8 bit mode				// 0b0011 = second nibble 						// 0b0011 = first nibble
	HAL_Delay(200);
	// 0b00110011 combined -> go to 8-bit mode
	lcd_writeNibble(COMMAND, LCD_WRITE, BL_ON, 0x3);		//0b0011000 -> reset to 8 bit mode				//0b0011000 -> reset to 8 bit mode				// 0b0011 = second nibble
	HAL_Delay(500);																																				// 0b00110011 combined -> go to 8-bit mode


	//all three possible states have been reset to 8-bit mode
	lcd_writeNibble(COMMAND, LCD_WRITE, BL_ON, 0x2);		//0b0010000 -> 8-bit command to switch to 4-bit mode
	HAL_Delay(500);

	//Has entered 4-bit mode
	lcd_writeByte(COMMAND, LCD_WRITE, BL_ON, 0x28); 		//Data entry mode
	HAL_Delay(500);
	lcd_writeByte(COMMAND, LCD_WRITE, BL_ON, 0x28); 		//
	HAL_Delay(500);
	lcd_writeByte(COMMAND, LCD_WRITE, BL_ON, 0x28); 		//
	HAL_Delay(500);

	//Set lcd parameters in 4-bit mode
	lcd_writeByte(COMMAND, LCD_WRITE, BL_ON, 0x01); 		//clear lcd go to pos 0
	HAL_Delay(5);
	lcd_writeByte(COMMAND, LCD_WRITE, BL_ON, 0x06); 		//increment left, no shift
	HAL_Delay(5);
	lcd_writeByte(COMMAND, LCD_WRITE, BL_ON, 0x0F); 		//display on, hide cursor, blink on
	HAL_Delay(5);
}



//requires the i2c handle to decide witch i2c-bus should be searched
void lcd_detect(I2C_HandleTypeDef *hi2c)
{

  	HAL_StatusTypeDef result;
	HAL_Delay(500);	//500ms to let VDD stabilize and LCD to power up

  	//searching 7-Bit address space
   	for (uint8_t i=0; i<128; i++)
   	{
   	  /*
   	   * retries 2
   	   * timeout 200ms
   	   */

   		result = HAL_I2C_IsDeviceReady(hi2c, (uint16_t) i, 2, 200);
   		if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
   		{
   			//do nothing
   		}
   		if (result == HAL_OK)
   		{
   			uint8_t address = i;
   			lcd_init(hi2c, address);

   			binary_to_string(address);
   			i++; //skip the read adress of same device
   		}
   	}
}

void binary_to_string(uint8_t binary)
{

	for(uint8_t i=0; i<8; i++)
	{

		if( binary & (1 << i))
		{
			lcd_writeByte(DATA, LCD_WRITE, BL_ON, 0b00110001);  //1
		}
		else
		{
			lcd_writeByte(DATA, LCD_WRITE, BL_ON, 0b00110000);  //0
		}

	}

}

void lcd_clear()
{
	lcd_writeByte(COMMAND, LCD_WRITE, BL_ON, 0x01);
}

void lcd_print2(char string[], uint8_t y, uint8_t x)
{
	//16x2LCD


	//check if y exceeds lcd bounds
	if(y<0 || y > (_y_max-1))
	{
		y=0;
	}

	//check if x exceeds lcd bounds
	if(x<0 || x> (_y_max-1))
	{
		x=0;
	}

	//whats down?

	//adresses to first characters in line
	uint8_t line_start_address[4];
	line_start_address[0]= 0b10000000;
	line_start_address[1]= 0b11000000;
	line_start_address[2]= 0b10010100;
	line_start_address[3]= 0b11010100;

	//calculate cursor adress from (x / y)
	uint8_t cursor_address = line_start_address[y] + x;

	//set cursor position
	//writeNibbleWithClock(COMMAND, LCD_WRITE, BL_ON, 0b1100);
	//writeNibbleWithClock(COMMAND, LCD_WRITE, BL_ON, 0b0000);
	lcd_writeByte(COMMAND, LCD_WRITE, BL_ON, cursor_address);

	//write string to that position
    for(uint8_t i = 0; string[i] != 0; i++)
    {
    		lcd_writeByte(DATA, LCD_WRITE, BL_ON, string[i]);
    }

}
#endif /* HD44780I2C_C_ */
