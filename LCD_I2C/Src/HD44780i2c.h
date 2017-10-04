/*
 * HD44780i2c.h
 *
 *  Created on: 28.09.2017
 *      Author: gordon
 */

#ifndef HD44780I2C_H_
#define HD44780I2C_H_

//include STM HAL_Drivers
#include <stdint.h>
#include "stm32f1xx_hal.h"

//MACROS
//-----------------------------------------------------------------------------

//Pin layout on I2C 8-Bit I/O-Expander
#define Pin_RS	0	//Register select
#define Pin_RW 	1	//Read/Write
#define Pin_EN	2	//Enable / Clock pin
#define Pin_BL	3  	//LCDbacklight
#define Pin_D4	4	//DataPin 4
#define Pin_D5 	5	//DataPin 5
#define Pin_D6 	6	//DataPin 6
#define Pin_D7 	7	//DataPin 7

//Commands
//RS
#define COMMAND     0
#define DATA        1
//RW For Read/Write to LCD
#define LCD_READ    1
#define LCD_WRITE	0
//EN for accepting new input from ioexpander
#define EN_DOWN		0
#define EN_UP		1
//BL for backlight
#define BL_OFF      0
#define BL_ON       1

//FUNCTIONS
//-----------------------------------------------------------------------------

//will detect lcd hooked up to the i2c-bus and display its 8 bit adresses
void lcd_detect(I2C_HandleTypeDef *hi2c);
void lcd_setSize(uint8_t rows, uint8_t lines);
void lcd_init(I2C_HandleTypeDef *hi2c, uint8_t dev_address);
void lcd_write(char* str[]);
void lcd_print2(char string[], uint8_t y, uint8_t x);
void lcd_clear();

void lcd_clock(uint8_t nibble);
void lcd_writeNibble(uint8_t RS, uint8_t RW, uint8_t BL, uint8_t nibble);
void lcd_writeByte(uint8_t RS, uint8_t RW, uint8_t BL, uint8_t data);
void binary_to_string(uint8_t binary);
//VARIABLES
//-----------------------------------------------------------------------------
//LCD format eg: LCD16x2: x=16, y=2
 uint8_t _x_max;
 uint8_t _y_max;
 uint8_t dev_address;
 I2C_HandleTypeDef *hi2c;
#endif /* HD44780I2C_H_ */
