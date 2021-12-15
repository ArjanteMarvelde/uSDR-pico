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
#include "pico/sem.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "hmi.h"
#include "lcd.h"
#include "dsp.h"
#include "si5351.h"
#include "monitor.h"
#include "relay.h"

#define LED_MS		1000
#define LOOP_MS		100

#define I2C0_SDA	16
#define I2C0_SCL	17
#define I2C1_SDA	18
#define I2C1_SCL	19

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

/*
 * Scheduler callback function.
 * This executes every LOOP_MS.
 */
semaphore_t loop_sem;
struct repeating_timer loop_timer;
bool loop_callback(struct repeating_timer *t)
{
	sem_release(&loop_sem);
	return(true);
}


int main()
{
	/* Initialize LED pin output */
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN, true);			// Set LED on
	add_repeating_timer_ms(-LED_MS, led_callback, NULL, &led_timer);

	/*
	 * i2c0 is used for the si5351 interface
	 * i2c1 is used for the LCD and all other interfaces
	 */

	/* i2c0 initialisation at 400Khz. */
	i2c_init(i2c0, 400*1000);
	gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C0_SDA);
	gpio_pull_up(I2C0_SCL);
	
	/* i2c1 initialisation at 400Khz. */
	i2c_init(i2c1, 100*1000);
	gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C1_SDA);
	gpio_pull_up(I2C1_SCL);
	

	/* Initialize units */
	mon_init();										// Monitor shell on stdio
	si_init();										// VFO control unit
	dsp_init();										// Signal processing unit
	relay_init();
	lcd_init();										// LCD output unit
	hmi_init();										// HMI user inputs
	
	/* A simple round-robin scheduler */
	sem_init(&loop_sem, 1, 1) ;	
	add_repeating_timer_ms(-LOOP_MS, loop_callback, NULL, &loop_timer);
	while (1) 										
	{
		sem_acquire_blocking(&loop_sem);			// Wait until timer callback releases sem
		mon_evaluate();								// Check monitor input
		si_evaluate();								// Refresh VFO settings
		hmi_evaluate();								// Refresh HMI
	}

    return 0;
}
