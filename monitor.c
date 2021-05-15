/*
 * monitor.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * Command shell on stdin/stdout.
 * Collects characters and parses commandstring.
 * Additional commands can easily be added.
 */ 

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "lcd.h"
#include "si5351.h"
#include "dsp.h"
#include "monitor.h"

/* Monitor definitions */
#define CR			13
#define LF			10
#define CMD_LEN		32

char mon_cmd[CMD_LEN+1];



typedef struct 
{
	char *cmdstr;									// Command string
	int   cmdlen;									// Command string length
	void (*cmd)(char* par);							// Command executive
	char *cmdsyn;									// Command syntax
	char *help;										// Command help text
} shell_t;


/* ------------------------------------------------------------- */
/* Below the definitions of the shell commands, add where needed */
/* ------------------------------------------------------------- */

/* 
 * Dumps a defined range of Si5351 registers 
 */
uint8_t si5351_reg[200];
void mon_si(char *par)
{
	int base=0, nreg=200, i;

	// Next: p = strtok(NULL, delim); (returns NULL if none left)
	for (i=0; i<nreg; i++) si5351_reg[i] = 0xaa;
	si_getreg(si5351_reg, (uint8_t)base, (uint8_t)nreg);
	for (i=0; i<nreg; i++) printf("%02x ",(int)(si5351_reg[i]));
	printf("\n");
}


/* 
 * Dumps the complete built-in and programmed characterset on the LCD 
 */
void mon_lt(char *par)
{
	printf("Check LCD...");
	lcd_test();
	printf("\n");
}


/* 
 * Checks for inter-core fifo overruns 
 */
extern volatile uint32_t fifo_overrun, fifo_rx, fifo_tx, fifo_xx, fifo_incnt;
void mon_fo(char *par)
{
	printf("Fifo input: %lu\n", fifo_incnt);
	printf("Fifo rx: %lu\n", fifo_rx);
	printf("Fifo tx: %lu\n", fifo_tx);
	printf("Fifo unknown: %lu\n", fifo_xx);
	printf("Fifo overruns: %lu\n", fifo_overrun);
}


/*
 * Toggles the PTT status, overriding the HW signal
 */
bool ptt = false;
void mon_pt(char *par)
{
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
	tx_enabled = ptt;
}


#define NCMD	4
shell_t shell[NCMD]=
{
	{"si", 2, &mon_si, "si <start> <nr of reg>", "Dumps Si5351 registers"},
	{"lt", 2, &mon_lt, "lt (no parameters)", "LCD test, dumps characterset on LCD"},
	{"fo", 2, &mon_fo, "fo (no parameters)", "Returns inter core fifo overruns"},
	{"pt", 2, &mon_pt, "pt (no parameters)", "Toggles PTT status"}
};





/* Commandstring parser, checks commandstring and invokes shellcommand */
char delim[] = " ";
void mon_parse(char* s)
{
	char *p;
	int  i;

	p = s;											// Get command part of string
	for (i=0; i<NCMD; i++)
		if (strncmp(p, shell[i].cmdstr, shell[i].cmdlen) == 0) break;
	if (i<NCMD)
		(*shell[i].cmd)(p);
	else
	{
		for (i=0; i<NCMD; i++)
			printf("%s\n   %s\n", shell[i].cmdsyn, shell[i].help);
	}
}

void mon_init()
{
    stdio_init_all();								// Initialize Standard IO
	mon_cmd[CMD_LEN] = '\0';						// Termination to be sure
	printf("\n");
	printf("=============\n");
	printf(" uSDR-Pico   \n");
	printf(" PE1ATM      \n");
	printf(" 2021, Udjat \n");
	printf("=============\n");
	printf("Pico> ");								// prompt
}

/*
 * This function collects characters from stdin until CR
 * Then the command is send to a parser and executed.
 */
void mon_evaluate(uint32_t timeout)
{
	static int i = 0;
	int c = getchar_timeout_us(timeout);			// NOTE: this is the only SDK way to read from stdin
	if (c==PICO_ERROR_TIMEOUT) return;				// Early bail out
	
	switch (c)
	{
	case CR:										// CR : need to parse command string
		putchar('\n');								// Echo character, assume terminal appends CR
		mon_cmd[i] = '\0';							// Terminate command string		
		if (i>0)									// something to parse?
			mon_parse(mon_cmd);						// --> process command
		i=0;										// reset index
		printf("Pico> ");							// prompt
		break;
	case LF:
		break;										// Ignore, assume CR as terminator
	default:
		if ((c<32)||(c>=128)) break;				// Only allow alfanumeric
		putchar((char)c);							// Echo character
		mon_cmd[i] = (char)c;						// store in command string
		if (i<CMD_LEN) i++;							// check range and increment
		break;
	}
}
