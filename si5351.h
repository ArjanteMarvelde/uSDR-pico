#ifndef _SI5351_H
#define _SI5351_H
/*
 * si5351.h
 *
 * Created: 13 March 2021
 * Author: Arjan
 */ 



typedef struct
{
	uint32_t freq;		// type can hold up to 4GHz
	uint8_t  flag;		// flag != 0 when update needed
	uint8_t  phase;		// in quarter waves (0, 1, 2, 3)
	uint8_t  ri;		// Ri (1 .. 128)
	uint8_t  msi;		// MSi parameter a (4, 6, 8 .. 126)
	float    msn;		// MSN (24.0 .. 35.9999)
} vfo_t;
extern vfo_t vfo[2];	// Table contains all control data for three clk outputs, but 0 and 1 are coupled in vfo[0]

int  si_getreg(uint8_t *data, uint8_t reg, uint8_t len);
void vfo_init(void);
void vfo_evaluate(void);


#define SI_GETFREQ(i)		((((i)>=0)&&((i)<2))?vfo[(i)].freq:0)
#define SI_INCFREQ(i, d)	if ((((i)>=0)&&((i)<2))&&((vfo[(i)].freq)<(150000000-(d)))) { vfo[(i)].freq += (d); vfo[(i)].flag = 1;}
#define SI_DECFREQ(i, d)	if ((((i)>=0)&&((i)<2))&&((vfo[(i)].freq)>(d))) { (vfo[(i)].freq) -= (d); vfo[(i)].flag = 1;}
#define SI_SETFREQ(i, f)	if ((((i)>=0)&&((i)<2))&&((f)<150000000)) { vfo[(i)].freq = (f); vfo[(i)].flag = 1;}

#endif /* _SI5351_H */