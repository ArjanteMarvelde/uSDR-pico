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
#define HMI_S_MENU			0
#define HMI_S_TUNE			1
#define HMI_S_MODE			2
#define HMI_S_AGC			3
#define HMI_S_PRE			4
#define HMI_NSTATES			5

/* Event definitions */
#define HMI_E_NOEVENT		0
#define HMI_E_INCREMENT		1
#define HMI_E_DECREMENT		2
#define HMI_E_ENTER			3
#define HMI_E_ESCAPE		4
#define HMI_E_LEFT			5
#define HMI_E_RIGHT			6
#define HMI_E_PTTON			7
#define HMI_E_PTTOFF		8
#define HMI_NEVENTS			9

/* Sub menu option string sets */
#define HMI_NMODE	4
#define HMI_NAGC	3
#define HMI_NPRE	3
char hmi_o_menu[HMI_NSTATES][8] = {"Menu","Tune","Mode","AGC ","Pre "};	// Selected by hmi_state
char hmi_o_mode[HMI_NMODE][8] = {"USB", "LSB", "AM ", "CW "};			// Selected by hmi_option/hmi_mode
char hmi_o_agc [HMI_NAGC][8] = {"NoGC", "Slow", "Fast"};				// Selected by hmi_option/hmi_agc
char hmi_o_pre [HMI_NPRE][8] = {"Off", "Amp", "Att"};					// Selected by hmi_option/hmi_pre

uint8_t  hmi_state, hmi_option;											// Current state and option
uint8_t  hmi_sub[HMI_NSTATES] = {0,4,0,0,0};							// Stored option per state
uint32_t hmi_freq;														// Frequency from Tune state
uint32_t hmi_step[6] = {10000000, 1000000, 100000, 10000, 1000, 100};	// Frequency digit increments
#define HMI_MAXFREQ		30000000
#define HMI_MINFREQ		     100

/*
 * Finite State Machine,
 * Handle event according to current state
 */
void hmi_handler(uint8_t event)
{
	switch(hmi_state)
	{
	case HMI_S_MENU:
		if ((event==HMI_E_INCREMENT)||(event==HMI_E_RIGHT))
			hmi_option = (hmi_option<HMI_NSTATES-1)?hmi_option+1:1;
		if ((event==HMI_E_DECREMENT)||(event==HMI_E_LEFT))
			hmi_option = (hmi_option>1)?hmi_option-1:HMI_NSTATES-1;
		if (event==HMI_E_ENTER)
		{
			hmi_state = hmi_option;										// Enter new submenu
			hmi_option = hmi_sub[hmi_state];							// Restore option
		}
		break;
	case HMI_S_TUNE:
		if (event==HMI_E_ENTER)
		{
			hmi_sub[hmi_state] = hmi_option;							// Store option
			SI_SETFREQ(0, 2*hmi_freq);									// Commit frequency
		}
		if (event==HMI_E_ESCAPE)
		{
			hmi_sub[hmi_state] = hmi_option;							// Store option
			hmi_option = hmi_state;
			hmi_state = HMI_S_MENU;										// Leave submenu
		}
		if (event==HMI_E_INCREMENT)
		{
			hmi_freq += hmi_step[hmi_option];
			if (hmi_freq > HMI_MAXFREQ) hmi_freq = HMI_MAXFREQ;
		}
		if (event==HMI_E_DECREMENT)
		{
			hmi_freq -= hmi_step[hmi_option];
			if (hmi_freq < HMI_MINFREQ) hmi_freq = HMI_MINFREQ;
		}
		if (event==HMI_E_RIGHT)
			hmi_option = (hmi_option<6)?hmi_option+1:0;
		if (event==HMI_E_LEFT)
			hmi_option = (hmi_option>0)?hmi_option-1:6;
		break;	
	case HMI_S_MODE:
		if (event==HMI_E_ENTER)
		{
			// Set Mode
			hmi_sub[hmi_state] = hmi_option;							// Store option	
			hmi_option = hmi_state;
			hmi_state = HMI_S_MENU;										// Leave submenu
		}
		if (event==HMI_E_ESCAPE)
		{
			hmi_option = hmi_state;
			hmi_state = HMI_S_MENU;										// Leave submenu
		}
		if ((event==HMI_E_INCREMENT)||(event==HMI_E_RIGHT))
			hmi_option = (hmi_option<HMI_NMODE-1)?hmi_option+1:0;
		if ((event==HMI_E_DECREMENT)||(event==HMI_E_LEFT))
			hmi_option = (hmi_option>0)?hmi_option-1:HMI_NMODE-1;
		break;
	case HMI_S_AGC:
		if (event==HMI_E_ENTER)
		{
			// Set AGC
			hmi_sub[hmi_state] = hmi_option;							// Store option	
			hmi_option = hmi_state;
			hmi_state = HMI_S_MENU;										// Leave submenu
		}
		if (event==HMI_E_ESCAPE)
		{
			hmi_option = hmi_state;
			hmi_state = HMI_S_MENU;										// Leave submenu
		}
		if ((event==HMI_E_INCREMENT)||(event==HMI_E_RIGHT))
			hmi_option = (hmi_option<HMI_NAGC-1)?hmi_option+1:0;
		if ((event==HMI_E_DECREMENT)||(event==HMI_E_LEFT))
			hmi_option = (hmi_option>0)?hmi_option-1:HMI_NAGC-1;
		break;
	case HMI_S_PRE:
		if (event==HMI_E_ENTER)
		{
			// Set Preamp
			hmi_sub[hmi_state] = hmi_option;							// Store option	
			hmi_option = hmi_state;
			hmi_state = HMI_S_MENU;										// Leave submenu
		}
		if (event==HMI_E_ESCAPE)
		{
			hmi_option = hmi_state;
			hmi_state = HMI_S_MENU;										// Leave submenu
		}
		if ((event==HMI_E_INCREMENT)||(event==HMI_E_RIGHT))
			hmi_option = (hmi_option<HMI_NPRE-1)?hmi_option+1:0;
		if ((event==HMI_E_DECREMENT)||(event==HMI_E_LEFT))
			hmi_option = (hmi_option>0)?hmi_option-1:HMI_NPRE-1;
		break;
	}
}

/*
 * GPIO IRQ callback routine
 */
void hmi_callback(uint gpio, uint32_t events)
{
	uint8_t evt=HMI_E_NOEVENT;

	switch (gpio)
	{
	case GP_ENC_A:									// Encoder
		if (events&GPIO_IRQ_EDGE_FALL)
			evt = gpio_get(GP_ENC_B)?HMI_E_INCREMENT:HMI_E_DECREMENT;
		break;
	case GP_AUX_0:									// Enter
		if (events&GPIO_IRQ_EDGE_FALL)
			evt = HMI_E_ENTER;
		break;
	case GP_AUX_1:									// Escape
		if (events&GPIO_IRQ_EDGE_FALL)
			evt = HMI_E_ESCAPE;
		break;
	case GP_AUX_2:									// Previous
		if (events&GPIO_IRQ_EDGE_FALL)
			evt = HMI_E_LEFT;
		break;
	case GP_AUX_3:									// Next
		if (events&GPIO_IRQ_EDGE_FALL)
			evt = HMI_E_RIGHT;
		break;
	case GP_PTT:									// PTT
		if (events&GPIO_IRQ_EDGE_FALL)
			dsp_ptt(true);
		else
			dsp_ptt(false);
		return;
	default:
		return;
	}
	
	hmi_handler(evt);
}

/*
 * Initialize the User interface
 */
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
	hmi_option = 4;									// kHz digit
	hmi_freq = 7074000UL;
	
	hmi_evaluate();

	SI_SETFREQ(0, 2*hmi_freq);						// Set freq to 2*7074 kHz
	SI_SETPHASE(0, 2);								// Set phase to 180deg
}

/*
 * Redraw the LCD, representing current state
 */
void hmi_evaluate(void)
{
	char s[20];
	
	sprintf(s, "%s %7.1f  %3d", hmi_o_mode[hmi_sub[HMI_S_MODE]], (double)hmi_freq/1000.0, 920);
	lcd_writexy(0,0,s);
	switch (hmi_state)
	{
	case HMI_S_MENU:
		sprintf(s, "=> %s         ", hmi_o_menu[hmi_option]);
		lcd_writexy(0,1,s);	
		lcd_curxy(2, 1, false);
		break;
	case HMI_S_TUNE:
		sprintf(s, "%s   %s  %s", hmi_o_menu[HMI_S_TUNE], hmi_o_pre[hmi_sub[HMI_S_PRE]], hmi_o_agc[hmi_sub[HMI_S_AGC]]);
		lcd_writexy(0,1,s);	
		lcd_curxy(4+(hmi_option>4?6:hmi_option), 0, true);
		break;
	case HMI_S_MODE:
		sprintf(s, "=> Mode: %s         ", hmi_o_mode[hmi_option]);
		lcd_writexy(0,1,s);	
		lcd_curxy(9, 1, false);
		break;
	case HMI_S_AGC:
		sprintf(s, "=> AGC: %s        ", hmi_o_agc[hmi_option]);
		lcd_writexy(0,1,s);	
		lcd_curxy(8, 1, false);
		break;
	case HMI_S_PRE:
		sprintf(s, "=> Pre: %s         ", hmi_o_pre[hmi_option]);
		lcd_writexy(0,1,s);	
		lcd_curxy(8, 1, false);
		break;
	default:
		break;
	}
}

