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

void lcd_init(void);
void lcd_clear(void);
void lcd_curxy(uint8_t x, uint8_t y, bool on);
void lcd_putxy(uint8_t x, uint8_t y, uint8_t c);
void lcd_writexy(uint8_t x, uint8_t y, uint8_t *s);
void lcd_test(void);

#endif
