/*
 * lcd.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * => LCD Address and Type are chosen in uSDR.h!
 *
 * This file contains the driver for 16x2 HD44780 based LCD displays.
 * Many different types exist, so you may need to adapt some of the code.
 * Most notably the startup sequence and the way bytes are sent over the I2C interface. 
 * But also the register mappings as described below.
 *
 * LCD_1804:
 * ---------
 * Grove 16x2 LCD HD44780, with integrated JHD1804 I2C bridge (@ 0x3E)
 * 2 byte interface, 
 * byte0 contains coomand/data, 
 * byte1 contains 8-bit command or data word
 *
 * LCD_8574_ADA:
 * -------------
 * Standard 16x2 LCD HD44780, with PCF8574 based Adafruit backpack I2C bridge (@ 0x27)
 * Same registers, but interface uses 4-bit transfers for data/comand, in bits 3..6
 * bit 0 is unused
 * bit 1 is Register Select (0 for command, 1 for data)
 * bit 2 is Enable, data/command is transferred on falling edge
 * bit 3..6 data or command nibble (write high nibble first)
 * bit 7 is backlight (1 for on)
 *
 * LCD_8574_GEN:
 * -------------
 * Standard 16x2 LCD HD44780, with PCF8574 based Generic backpack I2C bridge (@ 0x27)
 * Same registers, but interface uses 4-bit transfers for data/comand, in bits 4..7
 * bit 0 is Register Select (0 for command, 1 for data)
 * bit 1 is unused
 * bit 2 is Enable, data/command is transferred on falling edge
 * bit 3 is backlight (1 for on)
 * bit 4..7 data or command nibble (write high nibble first)
 * 
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "uSDR.h"
#include "lcd.h"


/** Generic HD44780 interface **/
// commands
#define LCD_CLEARDISPLAY 	0x01											// Note: LCD_ENTRYINC is set
#define LCD_RETURNHOME 		0x02
#define LCD_ENTRYMODESET 	0x04
#define LCD_DISPLAYCONTROL 	0x08
#define LCD_CURSORSHIFT 	0x10
#define LCD_FUNCTIONSET 	0x20
#define LCD_SETCGRAMADDR 	0x40
#define LCD_SETDDRAMADDR 	0x80

// flags for display entry mode: LCD_ENTRYMODESET
#define LCD_ENTRYSHIFT		0x01
#define LCD_ENTRYNOSHIFT	0x00
#define LCD_ENTRYINC		0x02											// Also applies to CGRAM writes
#define LCD_ENTRYDEC		0x00											// Also applies to CGRAM writes

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

/** I2C interface specific mappings **/
// 1804-based specific bitmasks (Seeed/Grove)
#define LCD_COMMAND			0x80
#define LCD_DATA			0x40
#define LCD_INIT_1804		(LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS)


// 8574-based specific bitmasks (Adafruit)
#define LCD_COMMAND_ADA		0x00
#define LCD_DATA_ADA		0x02
#define LCD_BACKLIGHT_ADA	0x80
#define LCD_ENABLE_ADA		0x04
#define LCD_INIT_ADA		(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS)

// 8574-based specific bitmasks (Generic)
#define LCD_COMMAND_GEN		0x00
#define LCD_DATA_GEN		0x01
#define LCD_ENABLE_GEN		0x04
#define LCD_BACKLIGHT_GEN	0x08
#define LCD_INIT_GEN		(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS)

#if (LCD_TYPE == LCD_1804)
#define LCD_START			LCD_INIT_1804
#define LCD_INIT_FUNCTION 	LCD_INIT_1804
#elif (LCD_TYPE == LCD_8574_ADA)
#define LCD_START			0x30
#define LCD_INIT_FUNCTION 	LCD_INIT_ADA
#elif (LCD_TYPE == LCD_8574_GEN)
#define LCD_START			0x30
#define LCD_INIT_FUNCTION 	LCD_INIT_GEN
#endif



/*
 * User defined (CGRAM) characters
 * Display RAM addresses 0x00-0x1f for top row and 0x40-0x5f for bottom row
 * Available character Generator addresses are 0x00-0x07
 */
uint8_t cgram[8][8] = 
{ 
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// 0x00: blank
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00},	// 0x01: Level 1
	{0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x00, 0x00},	// 0x02: Level 2
	{0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x00, 0x00},	// 0x03: Level 3
	{0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x00, 0x00},	// 0x04: Level 4
	{0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x00, 0x00},	// 0x05: Level 5
	{0x00, 0x04, 0x04, 0x04, 0x15, 0x0e, 0x04, 0x00},	// 0x06: Receive arrow down
	{0x04, 0x0e, 0x15, 0x04, 0x04, 0x04, 0x00, 0x00}	// 0x07: Transmit arrow up
};

/*
 * Transfer 1 byte to LCD
 * --> this function is interface dependent
 */
void lcd_sendbyte(uint8_t command, uint8_t data)
{

#if LCD_TYPE == LCD_1804
	uint8_t txdata[2];
	// Write command/data flag and data byte
	txdata[0] = (command?LCD_COMMAND:LCD_DATA); 
	txdata[1] = data;
	i2c_put_data(i2c1, I2C_LCD, txdata, 2, false);
#endif
		
#if LCD_TYPE == LCD_8574_ADA
	uint8_t high, low;
	high = (command?LCD_COMMAND_ADA:LCD_DATA_ADA)|((data>>1)&0x78)|LCD_BACKLIGHT_ADA;
	low  = (command?LCD_COMMAND_ADA:LCD_DATA_ADA)|((data<<3)&0x78)|LCD_BACKLIGHT_ADA;

	// Write high nibble
	high |= LCD_ENABLE_ADA;
	i2c_put_data(i2c1, I2C_LCD, &high, 1, false);
	high &= ~LCD_ENABLE_ADA;
	i2c_put_data(i2c1, I2C_LCD, &high, 1, false);
	
	// Write low nibble
	low |= LCD_ENABLE_ADA;
	i2c_put_data(i2c1, I2C_LCD, &low, 1, false);
	low &= ~LCD_ENABLE_ADA;
	i2c_put_data(i2c1, I2C_LCD, &low, 1, false);
#endif

#if LCD_TYPE == LCD_8574_GEN
	uint8_t high, low;
	high = (command?LCD_COMMAND_GEN:LCD_DATA_GEN)|((data   )&0xf0)|LCD_BACKLIGHT_GEN;
	low  = (command?LCD_COMMAND_GEN:LCD_DATA_GEN)|((data<<4)&0xf0)|LCD_BACKLIGHT_GEN;

	// Write high nibble
	high |= LCD_ENABLE_GEN;
	i2c_put_data(i2c1, I2C_LCD, &high, 1, false);
	high &= ~LCD_ENABLE_GEN;
	i2c_put_data(i2c1, I2C_LCD, &high, 1, false);
	
	// Write low nibble
	low |= LCD_ENABLE_GEN;
	i2c_put_data(i2c1, I2C_LCD, &low, 1, false);
	low &= ~LCD_ENABLE_GEN;
	i2c_put_data(i2c1, I2C_LCD, &low, 1, false);
#endif
}

/*
 * It seems that there is too much init here, but it doesn't harm either.
 */
void lcd_init(void)
{ 
	uint8_t i;
	
	/* HD44780 start sequence */
	sleep_ms(500);
	i = LCD_START;
	lcd_sendbyte(true, i);
	sleep_us(4500);
	lcd_sendbyte(true, i);
	sleep_us(100);
	lcd_sendbyte(true, i);

	/* Initialize function set */
	lcd_sendbyte(true, LCD_INIT_FUNCTION);

	/* Initialize display control */
	lcd_sendbyte(true, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);

	/* Initialize entry mode set */ 
	lcd_sendbyte(true, LCD_ENTRYMODESET | LCD_ENTRYINC | LCD_ENTRYNOSHIFT);
	
	/* Initialize display */
//	lcd_sendbyte(true, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
	lcd_sendbyte(true, LCD_CLEARDISPLAY);
	sleep_ms(2);
	lcd_sendbyte(true, LCD_RETURNHOME);
	sleep_ms(2);
	
	/* Load CGRAM */
	for (i=0; i<8; i++)
	{
		lcd_sendbyte(true, LCD_SETCGRAMADDR | (i<<3));						//Set CGRAM address
		for (int j=0; j<8; j++)
			lcd_sendbyte(false, cgram[i][j]);								// One byte at a time
	}
	
}

void lcd_clear(void)
{
	lcd_sendbyte(true, LCD_CLEARDISPLAY);
	sleep_ms(2);
}

void lcd_curxy(uint8_t x, uint8_t y, bool on)
{
	uint8_t txdata[3];

	x &= 0x0f;																// Clip range
	y &= 0x01;
	lcd_sendbyte(true, (x | 0x80 | (y==1?0x40:0x00)));
	lcd_sendbyte(true, LCD_DISPLAYCONTROL | LCD_DISPLAYON | (on?LCD_CURSORON:LCD_CURSOROFF) | LCD_BLINKOFF);
}

void lcd_putxy(uint8_t x, uint8_t y, uint8_t c)
{
	x &= 0x0f;																// Clip range
	y &= 0x01;
	lcd_sendbyte(true, (x | 0x80 | (y==1?0x40:0x00)));
	lcd_sendbyte(false, c);
}

void lcd_writexy(uint8_t x, uint8_t y, uint8_t *s)
{
	uint8_t i, len;

	x &= 0x0f;																// Clip range
	y &= 0x01;
	lcd_sendbyte(true, (x | 0x80 | (y==1?0x40:0x00)));

	len = strlen(s);
	len = (len>(16-x))?(16-x):len;											// Clip range
	for(i=0; i<len; i++)
		lcd_sendbyte(false, s[i]);
}


void lcd_test(void)
{
	uint8_t chr[17];
	int i, j;
	
	chr[16] = 0; 
	lcd_clear();
	for (i=0; i<16; i++)
	{
		for(j=0; j<16; j++) 
			chr[j] = (uint8_t)(16*i+j);
		lcd_writexy(0, 0, chr);
		sleep_ms(800);
		lcd_writexy(0, 1, chr);
	}
	lcd_clear();
}