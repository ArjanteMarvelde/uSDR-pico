/*
 * dsp.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * Signal processing of RX and TX branch, to be run on the second processor core.
 * Each branch has a dedicated routine that must run on set times.
 * The period is determined by reads from the inter-core fifo, by the dsp_loop() routine. 
 * This fifo is written from core0 from a 16us timer callback routine (i.e. 62.5kHz)
 *
 * The RX branch:
 * - Sample I and Q QSD channels, and shift into I and Q delay line (62.5 kHz per channel)
 * - Low pass filter: Fc=4kHz
 * - Quarter rate (15.625 kHz) to improve low F behavior of Hilbert transform
 * - Calculate 15 tap Hilbert transform on Q
 * - Demodulate, SSB: Q - I, taking proper delays into account
 * - Push to Audio output DAC
 *
 * The TX branch:
 * - Sample the Audio input channel (62.5 kHz)
 * - Low pass filter: Fc=3kHz
 * - Eight rate (7.8125 kHz) to improve low F behavior of Hilbert transform
 * - Generate Q samples by doing a Hilbert transform
 * - Push I and Q to QSE output DACs
 *
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "dsp.h"

/* 
 * DAC_RANGE defines PWM cycle, determining DAC resolution and PWM frequency.
 * DAC resolution = Vcc / DAC_RANGE
 * PWM frequency = Fsys / DAC_RANGE
 * A value of 250 means 125MHz/250=500kHz
 * ADC is 12 bit, so resolution is by definition 4096
 * To eliminate undefined behavior we clip off the upper 4 sample bits.
 */
#define DAC_RANGE	250
#define DAC_BIAS	DAC_RANGE/2
#define ADC_RANGE	4096
#define ADC_BIAS	ADC_RANGE/2

/* 
 * Callback timeout and inter-core FIFO commands. 
 * The timer value in usec determines frequency of TX and RX loops
 * Exact time is obtained by passing the value as negative
 * Here we use 16us (62.5 kHz == PWM freq/4) 
 */
#define DSP_US		16
#define DSP_TX		1
#define DSP_RX		2


/* 
 * Low pass filters Fc=3, 7 and 15 kHz (see http://t-filter.engineerjs.com/)
 * for sample rates 62.500 , 31.250 or 15.625 kHz , stopband is appr -40dB
 * 8 bit precision, so divide sum by 256 
 */
int16_t lpf3_62[15] =  {  3,  3,  5,  7,  9, 10, 11, 11, 11, 10,  9,  7,  5,  3,  3};	// Pass: 0-3000, Stop: 6000-31250
int16_t lpf3_31[15] =  { -2, -3, -3,  1, 10, 21, 31, 35, 31, 21, 10,  1, -3, -3, -2};	// Pass: 0-3000, Stop: 6000-15625
int16_t lpf3_15[15] =  {  3,  4, -3,-14,-15,  6, 38, 53, 38,  6,-15,-14, -3,  4,  3};	// Pass: 0-3000, Stop: 4500-7812
int16_t lpf7_62[15] =  { -2, -1,  1,  7, 16, 26, 33, 36, 33, 26, 16,  7,  1, -1, -2};	// Pass: 0-7000, Stop: 10000-31250
int16_t lpf7_31[15] =  { -1,  4,  9,  2,-12, -2, 40, 66, 40, -2,-12,  2,  9,  4, -1};	// Pass: 0-7000, Stop: 10000-15625
int16_t lpf15_62[15] = { -1,  3, 12,  6,-12, -4, 40, 69, 40, -4,-12,  6, 12,  3, -1};	// Pass: 0-15000, Stop: 20000-31250

volatile uint16_t dac_iq, dac_audio;
volatile bool tx_enabled;

void dsp_ptt(bool active)
{
	tx_enabled = active;
}
	

/* 
 * CORE1: 
 * Execute RX branch signal processing
 * No interleaving, since only 2us per conversion
 */
volatile int16_t i_s_raw[15], q_s_raw[15];			// Raw I/Q samples minus DC bias
volatile int16_t i_s[15], q_s[15];					// Filtered I/Q samples
volatile int16_t i_dc, q_dc; 						// DC bias for I/Q channel
volatile int dec_cnt=0;								// Decimation counter
bool rx(void) 
{
	int16_t q_sample, i_sample;
	int32_t q_accu, i_accu;
	int16_t qh;
	int i;
		
	adc_select_input(1);							// Q channel ADC 
	q_sample = (int16_t)adc_read();					// Take Q sample
	adc_select_input(0);							// I channel ADC 
	i_sample = (int16_t)adc_read();					// Take I sample

	/* 
	 * Shift in I and Q raw samples 
	 */
	for (i=0; i<14; i++)
	{
		q_s_raw[i] = q_s_raw[i+1];					// Q raw samples shift register
		i_s_raw[i] = i_s_raw[i+1];					// I raw samples shift register
	}
		
	/*
	 * Remove DC and store new sample
	 *  w(t) = x(t) + a*w(t-1) (use a=7/8, ca 0.87)
	 *  y(t) = w(t) - w(t-1) 
	 */
//	q_sample += (((q_dc<<3)-q_dc)>>3);				// Use sample as temporary q_dc
//	q_s_raw[14] = q_sample - q_dc;					// Calculate output
//	q_dc = q_sample;								// Store new q_dc
	q_s_raw[14] = (q_sample&0x0fff) - ADC_BIAS; 		//     just clip and subtract mid-level
		
//	-_sample += (((i_dc<<3)-i_dc)>>3);				// Use sample as temporary i_dc
//	i_s_raw[14] = i_sample - i_dc;					// Calculate output
//	i_dc = i_sample;								// Store new i_dc
	i_s_raw[14] = (i_sample&0x0fff) - ADC_BIAS; 		//     just clip and subtract mid-level
		
	/*
	 * Low pass filter + decimation
	 */
	dec_cnt = (dec_cnt+1)&0x03;						// Use only every fourth sample
	if (dec_cnt>0) return true;
	
	for (i=0; i<14; i++) 							// Shift samples
	{
		q_s[i] = q_s[i+1];
		i_s[i] = i_s[i+1];
	}
	q_accu = 0;
	i_accu = 0;
	for (i=0; i<15; i++)							// Low pass FIR filter
	{
		q_accu += (int32_t)q_s_raw[i]*lpf3_62[i];	// Fc=3kHz, at 62.5 kHz sampling		
		i_accu += (int32_t)i_s_raw[i]*lpf3_62[i];	// Fc=3kHz, at 62.5 kHz sampling	
	}
	q_s[14] = q_accu >> 8;
	i_s[14] = q_accu >> 8;

	/* 
	 * Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills)
	 */	
	q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[ 8])*2202L;
	qh = q_accu >> 12;	 
	
	/* 
	 * SSB demodulate: I[7] - Qh
	 * Range should be within DAC_RANGE
	 * Add 250 offset and send to audio DAC output
	 */
	q_sample = (i_s[7] - qh)/8;
	pwm_set_chan_level(dac_audio, PWM_CHAN_A, DAC_BIAS + q_sample);

	return true;
}


/* 
 * CORE1: 
 * Execute TX branch signal processing
 */
volatile int16_t a_s_pre[15]; 						// Raw samples, minus DC bias
volatile int16_t a_s[15];							// Filtered and decimated samples
volatile int16_t a_dc;								// DC level
bool tx(void) 
{
	static int tx_phase = 0;
	int16_t sample;
	int32_t accu;
	int16_t qh;
	int i;
		
	/*
	 * Get sample and shift into delay line
	 */
	adc_select_input(2);							// Audio channel ADC 
	for (i=0; i<14; i++) 
		a_s_pre[i] = a_s_pre[i+1];					// Audio samples delay line
	a_s_pre[14] = (int16_t)adc_read()-ADC_RANGE/2;	// Subtract half range (is appr. dc bias)
	
	tx_phase = (tx_phase+1)&0x01;					// Count to 2
	if (tx_phase != 0)								// 
		return true;								//   early bail out 1 out of 2 times
	
	/*
	 * Downsample and low pass
	 */
	for (i=0; i<14; i++) 							// Shift decimated samples
		a_s[i] = a_s[i+1];
	accu = 0;
	for (i=0; i<15; i++)							// Low pass FIR filter
		accu += (int32_t)a_s_pre[i]*lpf3_62[i];		// 3kHz, at 62.5 kHz sampling
	a_s[14] = accu / 256;

	
	/*
	 * Remove DC and store new sample
	 *  w(t) = x(t) + a*w(t-1) (use a=31/32, ca 0.97)
	 *  y(t) = w(t) - w(t-1) 
	 */
	//temp = a_dc;									// a_dc is w(t-1)
	//sample += (int16_t)(((temp<<5)-temp)>>5);		// Use sample as w(t)
	//a_s[14] = sample - a_dc;						// Calculate output
	//a_dc = sample;								// Store new w(t)
		
	/* 
	 * Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills):
	 */	
	accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
	qh = accu / 4096;	 

	/* 
	 * Write I and Q to QSE DACs, phase is 7 back.
	 * Need to multiply AC with DAC_RANGE/ADC_RANGE (appr 1/16, but compensate for losses)
	 */
	pwm_set_chan_level(dac_iq, PWM_CHAN_A, DAC_BIAS + (a_s[7]/8));
	pwm_set_chan_level(dac_iq, PWM_CHAN_B, DAC_BIAS + (qh/8));

	return true;
}


/* 
 * CORE1: 
 * Timing loop, triggered through inter-core fifo 
 */
void dsp_loop()
{
	uint32_t cmd;
	
    while(1) 
	{
        cmd = multicore_fifo_pop_blocking();		// Wait for fifo output
		if (cmd == DSP_TX)							// Change to switch(cmd) when more commands added
			tx();
		else
			rx();
    }
}


/* 
 * CORE0: 
 * Timer callback, triggers core1 through inter-core fifo.
 * Either TX or RX, but could do both when testing in loopback on I+Q channels.
 */
struct repeating_timer dsp_timer;
bool dsp_callback(struct repeating_timer *t) 
{
	if (tx_enabled)
		multicore_fifo_push_blocking(DSP_TX);		// Write TX in fifo to core 1
	else
		multicore_fifo_push_blocking(DSP_RX);		// Write RX in fifo to core 1
	
	return true;
}


/* 
 * CORE0: 
 * Initialize dsp context and spawn core1 process 
 */
void dsp_init() 
{
	uint16_t slice_num;
	
	/* Initialize DACs */
	gpio_set_function(20, GPIO_FUNC_PWM);			// GP20 is PWM for I DAC (Slice 2, Channel A)
	gpio_set_function(21, GPIO_FUNC_PWM);			// GP21 is PWM for Q DAC (Slice 2, Channel B)
	dac_iq = pwm_gpio_to_slice_num(20);				// Get PWM slice for GP20 (Same for GP21)
	pwm_set_clkdiv_int_frac (dac_iq, 1, 0);			// clock divide by 1
	pwm_set_wrap(dac_iq, DAC_RANGE);				// Set cycle length
	pwm_set_enabled(dac_iq, true); 					// Set the PWM running
	
	gpio_set_function(22, GPIO_FUNC_PWM);			// GP22 is PWM for Audio DAC (Slice 3, Channel A)
	dac_audio = pwm_gpio_to_slice_num(22);			// Find PWM slice for GP22
	pwm_set_clkdiv_int_frac (dac_audio, 1, 0);		// clock divide by 1
	pwm_set_wrap(dac_audio, DAC_RANGE);				// Set cycle length
	pwm_set_enabled(dac_audio, true); 				// Set the PWM running

	/* Initialize ADCs */
	adc_init();
	adc_gpio_init(26);								// GP26 is ADC 0 for I channel
	adc_gpio_init(27);								// GP27 is ADC 1 for Q channel
	adc_gpio_init(28);								// GP28 is ADC 2 for Audio channel
	adc_select_input(0);							// Select ADC 0 

	tx_enabled = false;								// RX mode

    multicore_launch_core1(dsp_loop);				// Start processing on core1
	add_repeating_timer_us(-DSP_US, dsp_callback, NULL, &dsp_timer);
}

