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
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "hmi.h"
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
	mon_init();										// Monitor shell on stdio
	si_init();										// VFO control unit
	dsp_init();										// Signal processing unit
	lcd_init();										// LCD output unit
	hmi_init();										// HMI user inputs
	
	while (1) 
	{
		mon_evaluate(10000L);						// Check monitor input, wait max 10000 usec
		si_evaluate();								// Refresh VFO settings
		hmi_evaluate();								// Refresh HMI
	}

    return 0;
}
