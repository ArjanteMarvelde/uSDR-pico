#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "lcd.h"
#include "dsp.h"
#include "si5351.h"



/* Monitor definitions */
#define ENDSTDIN	255
#define CR			13
#define LF			10
#define CMD_LEN		32

uint8_t display1[16];
uint8_t display2[16];



/* LED TIMER definition and callback */
struct repeating_timer led_timer;
bool led_callback(struct repeating_timer *t) 
{
	static bool led_state;
	
	gpio_put(PICO_DEFAULT_LED_PIN, led_state);
	led_state = (led_state?false:true);
	return true;
}


uint8_t si5351_reg[200];
char delim[] = " ";
#define NCMD	3
char *shell[NCMD] = {"si", "fa", "fb"};
void mon_parse(char* s)
{
	char *p;
	int base, nreg, i;

	p = strtok(s, delim); 							// Get command part of string
	for (i=0; i<NCMD; i++)
		if (strcmp(p, shell[i]) == 0) break;
	switch(i)
	{
	case 0:
		// Next: p = strtok(NULL, delim); (returns NULL if none left)
		for (i=0; i<nreg; i++) si5351_reg[i] = 0xaa;
		si_getreg(si5351_reg, (uint8_t)base, (uint8_t)nreg);
		for (i=0; i<nreg; i++) printf("%02x ",(int)(si5351_reg[i]));
		printf("\n");
		break;
	case 1:
		printf("%s\n", p);
		break;
	case 2:
		printf("%s\n", p);
		break;
	default:
		break;
	}
}



int main()
{
	/* Initialize IOs */
    stdio_init_all();

	/* Initialize LED pin output */
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN, true);			// Set LED on
	add_repeating_timer_ms(-1000, led_callback, NULL, &led_timer);

	/* Initialize PWM */
	dsp_init();
	
	/* Initialize Si5351 vfo */
	vfo_init();

	/* Initialize LCD */
	lcd_init();
	lcd_ctrl(LCD_GOTO, 0, 0);
	sprintf(display1, "A: 7074.0 kHz");
	lcd_write(display1,13);
	
	SI_SETFREQ(0, 2*7074000UL);						// Set freq to 2*7074 kHz
	
	/* Initialize monitor terminal */
	printf("Pico> ");
	int c, i=0;
	char mon_cmd[CMD_LEN+1];
	while (1) 
	{
		/* Check for monitor input */
		c = getchar_timeout_us(100000);				// 1 try per 100 msec
		switch (c)
		{
		case PICO_ERROR_TIMEOUT:					// just go-on
			break;
		case CR:									// CR or LF:
		case LF:									// 		need to parse command string
			putchar((char)c);						// echo character
			if (i==0) break;						// already did a parse, only do it once
			mon_cmd[i] = 0;							// terminate command string
			i=0;									// reset index
			mon_parse(mon_cmd);						// process command
			printf("Pico> ");						// prompt
			break;
		default:
			if ((c<32)||(c>=128)) break;			// Alfanumeric?
			putchar((char)c);						// echo character
			mon_cmd[i] = (char)c;					// store in command string
			if (i<CMD_LEN) i++;						// check range and increment
			break;
		}
		
		/* Check whether VFO settings have changed */
		vfo_evaluate();		
	}

    return 0;
}
