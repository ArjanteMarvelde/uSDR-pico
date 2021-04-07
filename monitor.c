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
#include "monitor.h"

/* Monitor definitions */
#define ENDSTDIN	255
#define CR			13
#define LF			10
#define CMD_LEN		32


char mon_cmd[CMD_LEN+1];


uint8_t si5351_reg[200];

/* Commandstring parser */
char delim[] = " ";
#define NCMD	4
char shell[NCMD][3] = {"si", "lt", "fb", "xx"};
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
	case 3:
		printf("%s\n", p);
		break;
	default:
		printf("??\n");
		break;
	}
}

void mon_init()
{
	/* Initialize IOs */
    stdio_init_all();
}

void mon_read(uint32_t timeout)
{
	int i = 0;
	int c = getchar_timeout_us(timeout);
	switch (c)
	{
	case PICO_ERROR_TIMEOUT:						// just go-on
		break;
	case CR:										// CR or LF:
	case LF:										// 		need to parse command string
		putchar((char)c);							// echo character
		if (i==0) break;							// already did a parse, only do it once
		mon_cmd[i] = '\0';							// terminate command string
		i=0;										// reset index
		mon_parse(mon_cmd);							// process command
		printf("Pico> ");							// prompt
		break;
	default:
		if ((c<32)||(c>=128)) break;				// Alfanumeric?
		putchar((char)c);							// echo character
		mon_cmd[i] = (char)c;						// store in command string
		if (i<CMD_LEN) i++;							// check range and increment
		break;
	}
}
