#ifndef _SI5351_H
#define _SI5351_H
/*
 * si5351.h
 *
 * Created: March 2021
 * Author: Arjan
 *
 * Driver for Si5351A chip.
 * VFO 0 is a quadrature clock on outputs 0 and 1, 
 * VFO 1 is a regular clock on output 2.
 *
 * VFO 0 allows to set frequency and phase offsets of 0-90-180-270 deg (delay clk1 wrt clk0)
 * VFO 1 just allows to set frequency, phase is ignored
 * 
 * Use the 'set' functions to change VFO settings.
 * Make regular calls to the 'evaluate' function to commit the changes (if any).
 * To get smooth tuning, a suggested interval is 0.1sec, e.g. from a timer loop
 *
 * See si5351.c for more information 
 *
 */ 

// Phase definitions for si_setphase()
#define PH000	0
#define PH090	1
#define PH180	2
#define PH270	3

typedef struct
{
	uint32_t freq;		// type can hold up to 4GHz
	uint8_t  phase;		// in quarter waves (0, 1, 2, 3)
	uint8_t  ri;		// Ri (1 .. 128), but should be 1 for VFO 0
	uint8_t  msi;		// MSi parameter a (4, 6, 8 .. 126)
	double   msn;		// MSN (24.0 .. 35.9999)
} vfo_t;


int  si_getreg(uint8_t *buffer, uint8_t reg, uint8_t len);
int  si_getvfo(int i, vfo_t *v);
void si_setphase(int i, uint8_t p);
void si_enable(int i, bool en);
void si_init(void);
void si_evaluate(int i, uint32_t freq);


#endif /* _SI5351_H */