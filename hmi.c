/*
 * hmi.c
 *
 * Created: Apr 2021
 * Author: Arjan te Marvelde
 * 
 * This file contains the HMI driver, processing user inputs.
 * It will also do the logic behind these, and write feedback to the LCD.
 *
 * The 4 auxiliary buttons have the following functions:
 * GP6 - Enter, confirm : Used to select menu items or make choices from a list
 * GP7 - Escape, cancel : Used to exit a (sub)menu or cancel the current action
 * GP8 - Left           : Used to move left, e.g. to select a digit
 * GP9 - Right			: Used to move right, e.g. to select a digit
 *
 * The rotary encoder (GP2, GP3) controls an up/down counter connected to some field. 
 * It may be that the encoder has a bushbutton as well, this can be connected to GP4.
 *
 * The PTT is connected to GP15 and will always be active, even when VOX is used.
 *
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "lcd.h"
#include "hmi.h"
#include "dsp.h"
#include "si5351.h"

/*
 * GPIO assignments
 */
#define GP_ENC_A	2
#define GP_ENC_B	3
#define GP_AUX_0	6
#define GP_AUX_1	7
#define GP_AUX_2	8
#define GP_AUX_3	9
#define GP_PTT		15
#define GP_MASK_IN	((1<<GP_ENC_A)|(1<<GP_ENC_B)|(1<<GP_AUX_0)|(1<<GP_AUX_1)|(1<<GP_AUX_2)|(1<<GP_AUX_3)|(1<<GP_PTT))

/*
 * Events: GPIO_IRQ_LEVEL_LOW, GPIO_IRQ_LEVEL_HIGH, GPIO_IRQ_EDGE_FALL, GPIO_IRQ_EDGE_RISE
 */
#define GPIO_IRQ_ALL		(GPIO_IRQ_LEVEL_LOW|GPIO_IRQ_LEVEL_HIGH|GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)
#define GPIO_IRQ_EDGE_ALL	(GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)

/*
 * Display layout: +----------------+
 *                 |USB 14074.1  920| --> USB mode, 14074.1 kHz, S9+20dB
 *                 |Fast  -20      5| --> Fast AGC, -20dB preamp, volume level 5
 *                 +----------------+
 * Menu statemachine: array of states and what to do per input event:
 * Menu								Encoder 	Enter		Escape		Left		Right
 * 0: Tune		Frequency			<value>		Menu 1					<dig		dig>
 * 1: Mode		USB, LSB, AM, CW	Menu 2
 * 2: AGC		Fast, Slow, Off		Menu 3
 * 3: Pre		+20dB, 0, -20dB		Menu 4
 * 4: Vol 		5, 4, 3, 2, 1
 */

uint8_t hmi_agc;
uint8_t hmi_mode;
uint8_t hmi_preamp;
uint8_t hmi_vol;


/*
 * GPIO IRQ callback routine
 */
volatile int count=0;
void hmi_callback(uint gpio, uint32_t events)
{
	switch (gpio)
	{
	case GP_ENC_A:
		if ((events&GPIO_IRQ_EDGE_FALL)&&gpio_get(GP_ENC_B))
			count++;
		else
			count--;
		break;
	case GP_ENC_B:
		break;
	case GP_AUX_0:
		break;
	case GP_AUX_1:
		break;
	case GP_AUX_2:
		break;
	case GP_AUX_3:
		break;
	case GP_PTT:
		if (events&GPIO_IRQ_EDGE_FALL)
			dsp_ptt(true);
		else
			dsp_ptt(false);
		printf("PTT\n");
		break;
	default:
		break;
	}
	
}


void hmi_init(void)
{
	/*
	 * Notes on using GPIO interrupts: 
	 * The callback handles interrupts for all GPIOs with IRQ enabled.
	 * Level interrupts don't seem to work properly.
	 * For debouncing, the GPIO pins should be pulled-up and connected to gnd with 100nF.
	 */
	// Init input GPIOs
	gpio_init_mask(GP_MASK_IN);
	
	// Enable pull-ups
	gpio_pull_up(GP_ENC_A);
	gpio_pull_up(GP_ENC_B);
	gpio_pull_up(GP_AUX_0);
	gpio_pull_up(GP_AUX_1);
	gpio_pull_up(GP_AUX_2);
	gpio_pull_up(GP_AUX_3);
	gpio_pull_up(GP_PTT);
	
	// Enable interrupt on level low
	gpio_set_irq_enabled(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_0, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_1, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_2, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_3, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_PTT, GPIO_IRQ_EDGE_ALL, true);

	// Set callback, one for all GPIO, not sure about correctness!
	gpio_set_irq_enabled_with_callback(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true, hmi_callback);
	
	// Initialize LCD and set VFO
	lcd_ctrl(LCD_CLEAR, 0, 0);
	lcd_write("USB  7074.0  920");
	lcd_ctrl(LCD_GOTO, 0, 1);
	lcd_write("Fast  -20      5");
	SI_SETFREQ(0, 2*7074000UL);						// Set freq to 2*7074 kHz
	SI_SETPHASE(0, 2);								// Set phase to 180deg

}

