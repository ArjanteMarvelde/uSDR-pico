/*
 * uSDR.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * The main loop of the application.
 * This initializes the units that do the actual work, and then loops in the background. 
 * Other units are:
 * - dsp.c, containing all signal processing in RX and TX branches. This part runs on the second processor core.
 * - si5351.c, containing all controls for setting up the si5351 clock generator.
 * - lcd.c, contains all functions to put something on the LCD
 * - hmi.c, contains all functions that handle user inputs
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "lcd.h"
#include "dsp.h"
#include "si5351.h"
#include "monitor.h"




/* 
 * LED TIMER definition and callback routine
 */
struct repeating_timer led_timer;
bool led_callback(struct repeating_timer *t) 
{
	static bool led_state;
	
	gpio_put(PICO_DEFAULT_LED_PIN, led_state);
	led_state = (led_state?false:true);
	return true;
}


int main()
{
	/* Initialize LED pin output */
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN, true);			// Set LED on
	add_repeating_timer_ms(-1000, led_callback, NULL, &led_timer);

	/* Initialize units */
	si_init();										// VFO control unit
	lcd_init();										// LCD output unit
	dsp_init();										// Signal processing unit
	mon_init();										// Monitor shell on stdio
	
	SI_SETFREQ(0, 2*7074000UL);						// Set freq to 2*7074 kHz
	SI_SETPHASE(0,2);								// Set phase to 180deg
	si_evaluate();									// Commit setting

	//lcd_test();									// Test LCD character set
	
	lcd_ctrl(LCD_GOTO, 0, 0);						// Go to (col, row)
	lcd_write(" 7074.0 kHz  USB");					// Max 16 char per line!
	lcd_ctrl(LCD_GOTO, 1, 0);
	lcd_ctrl(LCD_CURSOR, 1, 0);						// Switch cursor on
	
	while (1) 
	{
		mon_read(100000L);							// Check monitor input, wait max 100msec
		si_evaluate();								// Check VFO settings
	}

    return 0;
}
