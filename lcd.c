/*
 * lcd.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * Grove 16x2 LCD, HD44780 chip with JHD1804 I2C interface
 * Display RAM addresses 0x00-0x1f for top row and 0x40-0x5f for bottom row
 * Character Generator addresses are 0x00-0x07
 *
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "lcd.h"





// commands
#define LCD_CLEARDISPLAY 	0x01
#define LCD_RETURNHOME 		0x02
#define LCD_ENTRYMODESET 	0x04
#define LCD_DISPLAYCONTROL 	0x08
#define LCD_CURSORSHIFT 	0x10
#define LCD_FUNCTIONSET 	0x20
#define LCD_SETCGRAMADDR 	0x40
#define LCD_SETDDRAMADDR 	0x80

// flags for display entry mode: LCD_ENTRYMODESET
#define LCD_ENTRYRIGHT		0x00
#define LCD_ENTRYLEFT		0x02
#define LCD_ENTRYSHIFTINC	0x01
#define LCD_ENTRYSHIFTDEC	0x00

// flags for display on/off control: LCD_DISPLAYCONTROL
#define LCD_DISPLAYON 		0x04
#define LCD_DISPLAYOFF 		0x00
#define LCD_CURSORON 		0x02
#define LCD_CURSOROFF 		0x00
#define LCD_BLINKON 		0x01
#define LCD_BLINKOFF 		0x00

// flags for display/cursor shift: LCD_CURSORSHIFT
#define LCD_DISPLAYMOVE 	0x08
#define LCD_CURSORMOVE 		0x00
#define LCD_MOVERIGHT 		0x04
#define LCD_MOVELEFT 		0x00

// flags for function set: LCD_FUNCTIONSET
#define LCD_8BITMODE 		0x10
#define LCD_4BITMODE 		0x00
#define LCD_2LINE 			0x08
#define LCD_1LINE 			0x00
#define LCD_5x10DOTS 		0x04
#define LCD_5x8DOTS 		0x00


/* I2C address and pins */
#define I2C_LCD 0x3E
#define I2C0_SDA 16
#define I2C0_SCL 17

uint8_t lcd_buf[2][16];								// Buffer, y={0,1}, x={0..15}
uint8_t lcd_x, lcd_y;

void lcd_init(void)
{ 
	uint8_t txdata[8];
		
	/* I2C0 initialisation at 400Khz. */
	i2c_init(i2c0, 400*1000);
	gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C0_SDA);
	gpio_pull_up(I2C0_SCL);
	
	txdata[0] = 0x80;
	txdata[1] = LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS;
	i2c_write_blocking(i2c0, I2C_LCD, txdata, 2, false);
	sleep_us(39);
	txdata[1] = LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	i2c_write_blocking(i2c0, I2C_LCD, txdata, 2, false);
	sleep_us(39);
}

void lcd_ctrl(uint8_t cmd, uint8_t x, uint8_t y)
{
	uint8_t txdata[8];
	
	txdata[0] = 0x80;
	switch(cmd)
	{
	case LCD_CLEAR:
		for (lcd_x=0; lcd_x<16; lcd_x++)
		{
			lcd_buf[0][lcd_x] = ' ';
			lcd_buf[1][lcd_x] = ' ';
		}
		lcd_y = 0x00;
		lcd_x = 0x00;		
		txdata[1] = LCD_CLEARDISPLAY;
		i2c_write_blocking(i2c0, I2C_LCD, txdata, 2, false);
		sleep_us(1530);
		break;
	case LCD_HOME:
		lcd_y = 0x00;
		lcd_x = 0x00;		
		txdata[1] = LCD_RETURNHOME;
		i2c_write_blocking(i2c0, I2C_LCD, txdata, 2, false);
		sleep_us(39);
		break;
	case LCD_CURSOR:
		if (x==1)
			txdata[1] = 0x0e;
		else
			txdata[1] = 0x0c;
		i2c_write_blocking(i2c0, I2C_LCD, txdata, 2, false);
		sleep_us(39);
		break;
	case LCD_GOTO: 					// 2-row is 0x00-0x27 per row, only 0x00-0x1F are visible
		lcd_y = y&0x01;
		lcd_x = x&0x0f;
		if (y==1)
			txdata[1] = lcd_x | 0xc0;
		else
			txdata[1] = lcd_x | 0x80;
		i2c_write_blocking(i2c0, I2C_LCD, txdata, 2, false);
		sleep_us(39);
		break;
	}
}

void lcd_put(uint8_t c)
{
	uint8_t txdata[3];

	lcd_buf[lcd_y][lcd_x]=c;
	lcd_x = (lcd_x<15)?(lcd_x + 1):lcd_x;
	txdata[0] = 0x40;
	txdata[1] = c;
	i2c_write_blocking(i2c0, I2C_LCD, txdata, 2, false);
	sleep_us(43);
}

void lcd_write(uint8_t *s)
{
	uint8_t i, len;
	uint8_t txdata[18];

	len = strlen(s);
	len = (len>(16-lcd_x))?(16-lcd_x):len;
	txdata[0] = 0x40;
	for(i=0; i<len; i++)
	{
		lcd_buf[lcd_y][lcd_x++]=s[i];
		txdata[i+1]=s[i];
	}
	i2c_write_blocking(i2c0, I2C_LCD, txdata, len+1, false);
	sleep_us(43);
}


void lcd_test(void)
{
	uint8_t chr[17];
	int i, j;
	
	chr[16] = 0; 
	lcd_ctrl(LCD_CLEAR,0,0);
	for (i=0; i<16; i++)
	{
		for(j=0; j<16; j++) chr[j] = (uint8_t)(16*i+j);
		lcd_ctrl(LCD_GOTO,0,0);
		lcd_write(chr);
		sleep_ms(800);
		lcd_ctrl(LCD_GOTO,0,1);
		lcd_write(chr);
	}
	lcd_ctrl(LCD_CLEAR,0,0);
}