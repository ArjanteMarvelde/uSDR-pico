#ifndef __DSP_FFT_H__
#define __DSP_FFT_H__
/* 
 * dsp.h
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 *
 * See dsp.c for more information 
 *
 * HERE THE SELECTION BETWEEN TIME OR FREQUENCY DOMAIN PROCESSING IS MADE
 * DO THIS BY SETTING THE #define DSP_FFT TO 0 OR TO 1 RESPECTIVELY
 *
 */


/* 
 * Callback timeout is TIM_US, value in usec
 * The carrier offset is !=0 only in FFT case.
 */
 
#if DSP_FFT == 1

#define TIM_US		   64
#define S_RATE		15625					// 1e6/TIM_US
#define FC_OFFSET	 3906  					// RX carrier in bin FFT_SIZE/4 ==> S_RATE/4

#else
	
#define TIM_US		   64
#define S_RATE		15625					// 1e6/TIM_US
#define FC_OFFSET 	    0					// Must be 0 for time-domain DSP

#endif


/** DSP module interface **/

extern volatile uint32_t s_rssi;
int get_sval(void);

extern volatile bool tx_enabled;			// Determined by (vox_active || ptt_active)

#define VOX_OFF			0
#define VOX_LOW			1
#define VOX_MEDIUM		2
#define VOX_HIGH		3
void dsp_setvox(int vox);

#define MODE_USB		0
#define MODE_LSB		1
#define MODE_AM			2
#define MODE_CW			3
void dsp_setmode(int mode);

#define AGC_NONE		0
#define AGC_SLOW		1
#define AGC_FAST		2
void dsp_setagc(int agc);

void dsp_init();

#endif
