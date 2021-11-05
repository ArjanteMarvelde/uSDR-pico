#ifndef __DSP_H__
#define __DSP_H__
/* 
 * dsp.h
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 *
 * See dsp.c for more information 
 */

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"


void dsp_setagc(int agc);
void dsp_setmode(int mode);

extern volatile bool tx_enabled;
#define DSP_SETPTT(on)			tx_enabled = (on)


void dsp_init();



#endif
