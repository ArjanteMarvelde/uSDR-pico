/*
 * monitor.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * Command shell on stdin/stdout.
 * Collects characters and parses commandstring.
 */ 

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "lcd.h"
#include "si5351.h"
#include "dsp.h"
#include "monitor.h"

/* Monitor definitions */
#define ENDSTDIN	255
#define CR			13
#define LF			10
#define CMD_LEN		32


char mon_cmd[CMD_LEN+1];

uint8_t si5351_reg[200];
bool ptt = false;


/* Commandstring parser */
char delim[] = " ";
#define NCMD	4
char shell[NCMD][3] = {"si", "lt", "fb", "pt"};
void mon_parse(char* s)
{
	char *p;
	int base, nreg, i;

	p = s; //strtok(s, delim); 							// Get command part of string
	for (i=0; i<NCMD; i++)
		if (strncmp(p, shell[i], 2) == 0) break;
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
		lcd_test();
		break;
	case 2:
		printf("%s\n", p);
		break;
	case 3:
		if (ptt)
		{
			ptt = false;
			printf("PTT released\n");
		}
		else
		{
			ptt = true;
			printf("PTT active\n");
		}
		dsp_ptt(ptt);
		break;
	default:
		printf("??\n");
		break;
	}
}

void mon_init()
{
    stdio_init_all();								// Initialize Standard IO
	mon_cmd[CMD_LEN] = '\0';						// Termination to be sure
}

/*
 * This function collects characters from stdin until CR or LF
 * Then the command is send to a parser and executed.
 */
void mon_evaluate(uint32_t timeout)
{
	static int i = 0;
	int c = getchar_timeout_us(timeout);			// This is the only SDK way to read from stdin
	if (c==PICO_ERROR_TIMEOUT) return;				// Early bail out
	
	switch (c)
	{
	case CR:										// CR : need to parse command string
		putchar((char)c);							// Echo character
		mon_cmd[i] = '\0';							// Terminate command string		
		if (i>0)									// something to parse?
			mon_parse(mon_cmd);						// process command
		i=0;										// reset index
		printf("Pico> ");							// prompt
		break;
	case LF:
		putchar((char)c);							// Echo character
		break;										// Further ignore, assume CR as terminator
	default:
		if ((c<32)||(c>=128)) break;				// Only allow alfanumeric
		putchar((char)c);							// Echo character
		mon_cmd[i] = (char)c;						// store in command string
		if (i<CMD_LEN) i++;							// check range and increment
		break;
	}
}
