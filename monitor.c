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
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"

#include "lcd.h"
#include "si5351.h"
#include "dsp.h"
#include "relay.h"
#include "monitor.h"


#define CR			13
#define LF			10
#define SP			32
#define CMD_LEN		80
#define CMD_ARGS	16

char mon_cmd[CMD_LEN+1];							// Command string buffer
char *argv[CMD_ARGS];								// Argument pointers
int nargs;											// Nr of arguments

typedef struct 
{
	char *cmdstr;									// Command string
	int   cmdlen;									// Command string length
	void (*cmd)(void);								// Command executive
	char *cmdsyn;									// Command syntax
	char *help;										// Command help text
} shell_t;




/*** Initialisation, called at startup ***/
void mon_init()
{
    stdio_init_all();								// Initialize Standard IO
	mon_cmd[CMD_LEN] = '\0';						// Termination to be sure
	printf("\n");
	printf("=============\n");
	printf(" uSDR-Pico   \n");
	printf("  PE1ATM     \n");
	printf(" 2021, Udjat \n");
	printf("=============\n");
	printf("Pico> ");								// prompt
}



/*** ------------------------------------------------------------- ***/
/*** Below the definitions of the shell commands, add where needed ***/
/*** ------------------------------------------------------------- ***/

/* 
 * Dumps a defined range of Si5351 registers 
 */
uint8_t si5351_reg[200];
void mon_si(void)
{
	int base=0, nreg=200, i;

	for (i=0; i<nreg; i++) si5351_reg[i] = 0xaa;
	si_getreg(si5351_reg, (uint8_t)base, (uint8_t)nreg);
	for (i=0; i<nreg; i++) printf("%02x ",(int)(si5351_reg[i]));
	printf("\n");
}


/* 
 * Dumps the entire built-in and programmed characterset on the LCD 
 */
void mon_lt(void)
{
	printf("Check LCD...");
	lcd_test();
	printf("\n");
}


/* 
 * Checks for inter-core fifo overruns 
 */
extern volatile uint32_t fifo_overrun, fifo_rx, fifo_tx, fifo_xx, fifo_incnt;
void mon_fo(void)
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
void mon_pt(void)
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

/*
 * Relay read or write
 */
void mon_bp(void)
{
	int ret;
	
	if (*argv[1]=='w')
	{
		if (nargs>=2) 
		{
			ret = atoi(argv[2]);
			relay_setband((uint8_t)ret);
		}
	}
	ret = relay_getband();
	if (ret<0)
		printf ("I2C read error\n");
	else
		printf("%02x\n",  ret);
}

/*
 * Relay read or write
 */
void mon_rx(void)
{
	int ret;
	
	if (*argv[1]=='w')
	{
		if (nargs>=2) 
		{
			ret = atoi(argv[2]);
			relay_setattn((uint8_t)ret);
		}
	}
	ret = relay_getattn();
	if (ret<0)
		printf ("I2C read error\n");
	else
		printf("%02x\n", ret);
	
}

/*
 * Command shell table, organize the command functions above
 */
#define NCMD	6
shell_t shell[NCMD]=
{
	{"si", 2, &mon_si, "si <start> <nr of reg>", "Dumps Si5351 registers"},
	{"lt", 2, &mon_lt, "lt (no parameters)", "LCD test, dumps characterset on LCD"},
	{"fo", 2, &mon_fo, "fo (no parameters)", "Returns inter core fifo overruns"},
	{"pt", 2, &mon_pt, "pt (no parameters)", "Toggles PTT status"},
	{"bp", 2, &mon_bp, "bp {r|w} <value>", "Read or Write BPF relays"},
	{"rx", 2, &mon_rx, "rx {r|w} <value>", "Read or Write RX relays"}
};



/*** ---------------------------------------- ***/
/*** Commandstring parser and monitor process ***/
/*** ---------------------------------------- ***/

/*
 * Command line parser
 */
void mon_parse(char* s)
{
	char *p;
	int  i;

	p = s;											// Set to start of string
	nargs = 0;
	while (*p!='\0')								// Assume stringlength >0 
	{
		while (*p==' ') p++;						// Skip whitespace
		if (*p=='\0') break;						// String might end in spaces
		argv[nargs++] = p;							// Store first valid char loc after whitespace
		while ((*p!=' ')&&(*p!='\0')) p++;			// Skip non-whitespace
	}
	if (nargs==0) return;							// No command or parameter
	for (i=0; i<NCMD; i++)							// Lookup shell command
		if (strncmp(argv[0], shell[i].cmdstr, shell[i].cmdlen) == 0) break;
	if (i<NCMD)
		(*shell[i].cmd)();
	else											// Unknown command
	{
		for (i=0; i<NCMD; i++)						// Print help if no match
			printf("%s\n   %s\n", shell[i].cmdsyn, shell[i].help);
	}
}

/*
 * Monitor process 
 * This function collects characters from stdin until CR
 * Then the command is send to a parser and executed.
 */
void mon_evaluate(void)
{
	static int i = 0;
	int c = getchar_timeout_us(10L);				// NOTE: this is the only SDK way to read from stdin
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
