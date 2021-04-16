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
 *     ___     ___
 * ___|   |___|   |___  A
 *   ___     ___     _
 * _|   |___|   |___|   B
 *
 * Encoder channel A triggers on falling edge. 
 * Depending on B level, count is incremented or decremented.
 * 
 * The PTT is connected to GP15 and will be active, except when VOX is used.
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
#define GP_AUX_0	6								// Enter, Confirm
#define GP_AUX_1	7								// Escape, Cancel
#define GP_AUX_2	8								// Left move
#define GP_AUX_3	9								// Right move
#define GP_PTT		15
#define GP_MASK_IN	((1<<GP_ENC_A)|(1<<GP_ENC_B)|(1<<GP_AUX_0)|(1<<GP_AUX_1)|(1<<GP_AUX_2)|(1<<GP_AUX_3)|(1<<GP_PTT))

/*
 * Events: GPIO_IRQ_LEVEL_LOW, GPIO_IRQ_LEVEL_HIGH, GPIO_IRQ_EDGE_FALL, GPIO_IRQ_EDGE_RISE
 */
#define GPIO_IRQ_ALL		(GPIO_IRQ_LEVEL_LOW|GPIO_IRQ_LEVEL_HIGH|GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)
#define GPIO_IRQ_EDGE_ALL	(GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)

/*
 * Display layout: +----------------+
 *                 |USB 14074.0  920| --> USB mode, 14074.0 kHz, S9+20dB
 *                 |Tune  Att   Fast| --> Menu:Tune, Attenuator, Fast AGC
 *                 +----------------+
 * LEFT and RIGHT buttons (or encoder) are used to navigate sub-menus such as {tune,mode,agc,pre}.
 * ENTER is used to get into the sub-menu.
 * ENTER is used again to exit and accept changes or ESCAPE to exit without changes. 
 *
 * When entered in a submenu:
 * Menu		Values				Encoder 	Enter		Escape		Left		Right
 * -------------------------------------------------------------------------------------
 * Mode		USB, LSB, AM, CW	<value>		Accept		Exit		<value>		<value>
 * Tune		Frequency (digit)	<value>		Accept		Exit		<=dig		dig=>
 * AGC		Fast, Slow, Off		<value>		Accept		Exit		<value>		<value>
 * Pre		+20dB, 0, -20dB		<value>		Accept		Exit		<value>		<value>
 */
 
/* State definitions */
#define HMI_S_MENU		0
#define HMI_S_TUNE		1
#define HMI_S_MODE		2
#define HMI_S_AGC		3
#define HMI_S_PRE		4

/* Sub menu string sets */
char hmi_s_menu[5][8] = {"Menu","Tune","Mode","AGC ","Pre "};
char hmi_s_mode[4][8] = {"USB", "LSB", " AM", " CW"};
char hmi_s_agc [3][8] = {"NoGC", "Slow", "Fast"};
char hmi_s_pre [3][8] = {"Off", "Amp", "Att"};

uint8_t  hmi_state, hmi_mode, hmi_agc, hmi_pre;
uint32_t hmi_freq;
uint8_t  hmi_sub, hmi_option;

/*
 * Redraw the LCD
 */
void hmi_evaluate(void)
{
	char s[20];
	
	sprintf(s, "%s %7.1f  %3d", hmi_s_mode[hmi_mode], (double)hmi_freq/1000.0, 920);
	lcd_writexy(0,0,s);
	switch (hmi_state)
	{
	case HMI_S_MENU:
		sprintf(s, "%s   %s  %s", hmi_s_menu[hmi_sub], hmi_s_pre[hmi_pre], hmi_s_agc[hmi_agc]);
		lcd_writexy(0,1,s);	
		lcd_curxy(0, 1, true);
		break;
	case HMI_S_TUNE:
		sprintf(s, "%s   %s  %s", hmi_s_menu[HMI_S_TUNE], hmi_s_pre[hmi_pre], hmi_s_agc[hmi_agc]);
		lcd_writexy(0,1,s);	
		lcd_curxy(4+(hmi_option>4?6:hmi_option), 0, true);
		break;
	case HMI_S_MODE:
		sprintf(s, "%s: %s       ", hmi_s_menu[HMI_S_MODE], hmi_s_mode[hmi_option]);
		lcd_writexy(0,1,s);	
		lcd_curxy(6, 1, true);
		break;
	case HMI_S_AGC:
		sprintf(s, "%s: %s      ", hmi_s_menu[HMI_S_AGC], hmi_s_agc[hmi_option]);
		lcd_writexy(0,1,s);	
		lcd_curxy(6, 1, true);
		break;
	case HMI_S_PRE:
		sprintf(s, "%s: %s       ", hmi_s_menu[HMI_S_PRE], hmi_s_pre[hmi_option]);
		lcd_writexy(0,1,s);	
		lcd_curxy(6, 1, true);
		break;
	default:
		break;
	}
	

}


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
	case GP_AUX_0:									// Enter
		break;
	case GP_AUX_1:									// Escape
		break;
	case GP_AUX_2:									// Previous
		break;
	case GP_AUX_3:									// Next
		break;
	case GP_PTT:									// PTT
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
	lcd_clear();
	
	hmi_state = HMI_S_TUNE;
	hmi_sub = 1;
	hmi_option = 1;
	hmi_mode = 0;
	hmi_agc = 0;
	hmi_pre = 0;
	hmi_freq = 7074000UL;
	
	hmi_evaluate();

	SI_SETFREQ(0, 2*hmi_freq);						// Set freq to 2*7074 kHz
	SI_SETPHASE(0, 2);								// Set phase to 180deg
}

