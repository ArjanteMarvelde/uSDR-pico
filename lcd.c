/*
 * lcd.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * Select LCD_TYPE:
 * 
 * Grove 16x2 LCD HD44780, with integrated JHD1804 I2C bridge (@ 0x3E)
 * Display RAM addresses 0x00-0x1f for top row and 0x40-0x5f for bottom row
 * Available character Generator addresses are 0x00-0x07
 *
 * Standard 16x2 LCD HD44780, with backpack PCF8574 based I2C bridge (@ 0x27)
 * Same registers, but interface uses 4 bits for data/comand, in bits 3..7
 * bit 0 is unused
 * bit 1 is Register Select (0 for command, 1 for data)
 * bit 2 is Enable, data/command is transferred on falling edge
 * bit 3..6 data or command nibble (write high nibble first)
 * bit 8 is backlight (1 for on)
 * 
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "lcd.h"

/* Select LCD type matching your HW */
#define LCD_1804			0
#define LCD_8574			1
#define LCD_TYPE			LCD_8574

/* I2C address */
//#define I2C_LCD 			0x3E
#define I2C_LCD			0x27

/* HD44780 interface */
// commands
#define LCD_CLEARDISPLAY 	0x01					// Note: LCD_ENTRYINC is set
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
#define LCD_ENTRYINC		0x02					// Also applies to CGRAM writes
#define LCD_ENTRYDEC		0x00					// Also applies to CGRAM writes

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

#define LCD_DELAY			100							// Delay for regular write
#define LCD_COMMAND			0x80
#define LCD_DATA			0x40

/* 8574-based specific bitmasks */
#define LCD2_BACKLIGHT		0x80
#define LCD2_DATA			0x02
#define LCD2_ENABLE			0x04


/*
 * User defined (CGRAM) characters
 */
uint8_t cgram[8][8] = 
{ 
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},	// 0x00: blank
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00},	// 0x01: Level 1
	{0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x00, 0x00},	// 0x02: Level 2
	{0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x00, 0x00},	// 0x03: Level 3
	{0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x00, 0x00},	// 0x04: Level 4
	{0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x00, 0x00},	// 0x05: Level 5
	{0x00, 0x04, 0x04, 0x04, 0x1f, 0x0e, 0x04, 0x00},	// 0x06: Receive arrow down
	{0x04, 0x0e, 0x1f, 0x04, 0x04, 0x04, 0x00, 0x00}	// 0x07: Transmit arrow up
};

/*
 * Transfer 1 byte to LCD
 * This function is interface dependent
 */
void lcd_sendbyte(uint8_t command, uint8_t data)
{

#if LCD_TYPE == LCD_1804

	uint8_t txdata[2];
	// Write command/data flag and data byte
	txdata[0] = (command?LCD_COMMAND:LCD_DATA); 
	txdata[1] = data;
	i2c_write_blocking(i2c1, I2C_LCD, txdata, 2, false);
	sleep_us(LCD_DELAY);
		
#elif LCD_TYPE == LCD_8574

	uint8_t high, low;
	high = (command?0:LCD2_DATA)|LCD2_ENABLE|((data&0xf0)>>1)|LCD2_BACKLIGHT;
	low  = (command?0:LCD2_DATA)|LCD2_ENABLE|((data&0x0f)<<3)|LCD2_BACKLIGHT;
	
	// Write high nibble
	i2c_write_blocking(i2c1, I2C_LCD, &high, 1, false);	sleep_us(LCD_DELAY);
	high &= ~LCD2_ENABLE;
	i2c_write_blocking(i2c1, I2C_LCD, &high, 1, false);	sleep_us(LCD_DELAY);
	
	// Write low nibble
	i2c_write_blocking(i2c1, I2C_LCD, &low, 1, false); sleep_us(LCD_DELAY);
	low &= ~LCD2_ENABLE;
	i2c_write_blocking(i2c1, I2C_LCD, &low, 1, false); sleep_us(LCD_DELAY);

#endif
}

void lcd_init(void)
{ 
	uint8_t i;
	
	sleep_ms(100);

#if LCD_TYPE == LCD_1804
	/* Initialize function set (see datasheet fig 23)*/
	lcd_sendbyte(true, LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS);
	sleep_us(4500);
	lcd_sendbyte(true, LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS);
	lcd_sendbyte(true, LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS);
	lcd_sendbyte(true, LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS);
#elif LCD_TYPE == LCD_8574
	lcd_sendbyte(true, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
	sleep_us(4500);
	lcd_sendbyte(true, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
	lcd_sendbyte(true, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
	lcd_sendbyte(true, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
#endif

	/* Initialize display control */
	lcd_sendbyte(true, LCD_DISPLAYCONTROL | LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF);
	
	/* Display clear */
	lcd_sendbyte(true, LCD_CLEARDISPLAY);
	sleep_ms(2);

	/* Initialize entry mode set */ 
	lcd_sendbyte(true, LCD_ENTRYMODESET | LCD_ENTRYINC | LCD_ENTRYNOSHIFT);
	
	/* Load CGRAM */
	for (i=0; i<8; i++)
	{
		lcd_sendbyte(true, LCD_SETCGRAMADDR | (i<<3));		//Set CGRAM address
		for (int j=0; j<8; j++)
			lcd_sendbyte(false, cgram[i][j]);				// One byte at a time
	}
	
	/* Initialize display control */
	lcd_sendbyte(true, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
	
	/* Display clear once more */
	lcd_sendbyte(true, LCD_CLEARDISPLAY);
	sleep_ms(2);
}

void lcd_clear(void)
{
	lcd_sendbyte(true, LCD_CLEARDISPLAY);
	sleep_ms(2);
}

void lcd_curxy(uint8_t x, uint8_t y, bool on)
{
	uint8_t txdata[3];

	x &= 0x0f;											// Clip range
	y &= 0x01;
	lcd_sendbyte(true, (x | 0x80 | (y==1?0x40:0x00)));
	lcd_sendbyte(true, LCD_DISPLAYCONTROL | LCD_DISPLAYON | (on?LCD_CURSORON:LCD_CURSOROFF) | LCD_BLINKOFF);
}

void lcd_putxy(uint8_t x, uint8_t y, uint8_t c)
{
	x &= 0x0f;											// Clip range
	y &= 0x01;
	lcd_sendbyte(true, (x | 0x80 | (y==1?0x40:0x00)));
	lcd_sendbyte(false, c);
}

void lcd_writexy(uint8_t x, uint8_t y, uint8_t *s)
{
	uint8_t i, len;

	x &= 0x0f;											// Clip range
	y &= 0x01;
	lcd_sendbyte(true, (x | 0x80 | (y==1?0x40:0x00)));

	len = strlen(s);
	len = (len>(16-x))?(16-x):len;						// Clip range
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