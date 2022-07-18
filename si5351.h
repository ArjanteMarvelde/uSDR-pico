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
 * The quadrature clock allows to set shared frequency and phase offsets of 0, 90, 180 and 270 deg
 * The regular clock just allows to set frequency, phase is ignored
 * 
 * See si5351.c for more information 
 *
 */ 

#define PH000	0
#define PH090	1
#define PH180	2
#define PH270	3

int  si_getreg(uint8_t *data, uint8_t reg, uint8_t len);
void si_init(void);
void si_evaluate(void);
void si_setfreq(int i, uint32_t f);
void si_setphase(int i, uint8_t p);
void si_enable(int i, bool en);


#endif /* _SI5351_H */