#ifndef __LCD_H__
#define __LCD_H__
/* 
 * lcd.h
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 *
 * See lcd.c for more information 
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define LCD_CLEAR		0
#define LCD_HOME		1
#define LCD_BLINK		2
#define LCD_GOTO		3
#define LCD_CURSOR		4

void lcd_init(void);
void lcd_ctrl(uint8_t cmd, uint8_t x, uint8_t y);
void lcd_write(uint8_t *s, uint8_t len);
void lcd_test(void);


#endif
