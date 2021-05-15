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

#define ADC0_IRQ_FIFO 22

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
 * Note: 8 bit precision, so divide sum by 256 
 */
int16_t lpf3_62[15] =  {  3,  3,  5,  7,  9, 10, 11, 11, 11, 10,  9,  7,  5,  3,  3};	// Pass: 0-3000, Stop: 6000-31250
int16_t lpf3_31[15] =  { -2, -3, -3,  1, 10, 21, 31, 35, 31, 21, 10,  1, -3, -3, -2};	// Pass: 0-3000, Stop: 6000-15625
int16_t lpf3_15[15] =  {  3,  4, -3,-14,-15,  6, 38, 53, 38,  6,-15,-14, -3,  4,  3};	// Pass: 0-3000, Stop: 4500-7812
int16_t lpf7_62[15] =  { -2, -1,  1,  7, 16, 26, 33, 36, 33, 26, 16,  7,  1, -1, -2};	// Pass: 0-7000, Stop: 10000-31250
int16_t lpf7_31[15] =  { -1,  4,  9,  2,-12, -2, 40, 66, 40, -2,-12,  2,  9,  4, -1};	// Pass: 0-7000, Stop: 10000-15625
int16_t lpf15_62[15] = { -1,  3, 12,  6,-12, -4, 40, 69, 40, -4,-12,  6, 12,  3, -1};	// Pass: 0-15000, Stop: 20000-31250

volatile uint16_t dac_iq, dac_audio;
volatile uint32_t fifo_overrun, fifo_rx, fifo_tx, fifo_xx, fifo_incnt;
volatile bool tx_enabled;




/* 
 * CORE1: 
 * ADC IRQ handler.
 * Fills the results array in RR fashion, for 3 channels (~2usec per channel).
 */
volatile uint16_t adc_result[3];
volatile int adc_next;								// Remember which ADC the result is from
uint32_t adc_count=0;
void adcfifo_handler(void)
{
	adc_result[adc_next] = adc_fifo_get();
	adc_next = adc_hw->cs;
	adc_next = (adc_next >> ADC_CS_AINSEL_LSB) & 3;	// Only 0, 1 and  are valid
	adc_count++;
}
	

/* 
 * CORE1: 
 * Execute RX branch signal processing, max time to spend is <16us !
 * No ADC sample interleaving, take both I and Q channels.
 * The delay is only 2us per conversion, which causes less distortion than interpolation of samples.
 */
volatile int16_t i_s_raw[15], q_s_raw[15];			// Raw I/Q samples minus DC bias
volatile int16_t i_s[15], q_s[15];					// Filtered I/Q samples
volatile int16_t i_dc, q_dc; 						// DC bias for I/Q channel
volatile int rx_cnt=0;								// Decimation counter
bool rx(void) 
{
	int16_t q_sample, i_sample;
	int32_t q_accu, i_accu;
	int16_t qh;
	int i;

	i_sample = adc_result[0];						// Take last ADC 0 result
	q_sample = adc_result[1];						// Take last ADC 1 result

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
	 * IIR filter: dc = a*sample + (1-a)*dc  where a = 1/128
	 */
	q_sample = (q_sample&0x0fff) - ADC_BIAS;		// Clip and subtract mid-range
	q_dc += (q_sample>>7) - (q_dc>>7);				//   then IIR running average
	q_s_raw[14] = q_sample - q_dc;					//   and subtract DC, store in shift register
	i_sample = (i_sample&0x0fff) - ADC_BIAS;
	i_dc += (i_sample>>7) - (i_dc>>7);
	i_s_raw[14] = i_sample - i_dc;
		
	/*
	 * Low pass filter + decimation
	 */
	rx_cnt = (rx_cnt+1)&3;							// Calculate only every fourth sample
	if (rx_cnt>0) return true;
	
	for (i=0; i<14; i++) 							// Shift decimated samples
	{
		q_s[i] = q_s[i+1];
		i_s[i] = i_s[i+1];
	}
	q_accu = 0;										// Initialize accumulators
	i_accu = 0;
	for (i=0; i<15; i++)							// Low pass FIR filter
	{
		q_accu += (int32_t)q_s_raw[i]*lpf3_62[i];	// Fc=3kHz, at 62.5 kHz sampling		
		i_accu += (int32_t)i_s_raw[i]*lpf3_62[i];	// Fc=3kHz, at 62.5 kHz sampling	
	}
	q_s[14] = q_accu >> 8;
	i_s[14] = q_accu >> 8;

	/*
	 * From here things get dependent on receive mode.
	 * We assume USB for now, e.g. supporting FT8
	 */

	/* 
	 * Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills)
	 */	
	q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[ 8])*2202L;
	qh = q_accu >> 12;	 
	
	/* 
	 * SSB demodulate: I[7] - Qh
	 * Add DAC_BIAS offset, Clip to DAC_RANGE and send to audio DAC output
	 */
	q_sample = ((i_s[7] - qh)/2)+DAC_BIAS;
	q_sample = (q_sample>DAC_RANGE)?DAC_RANGE:((q_sample<0)?0:q_sample);
	pwm_set_chan_level(dac_audio, PWM_CHAN_A, q_sample);


/* AGC ALGORITHM
if(m_Peak>m_AttackAve) 								//if power is rising (use m_AttackRiseAlpha time constant)
	m_AttackAve = (1.0-m_AttackRiseAlpha)*m_AttackAve + m_AttackRiseAlpha*m_Peak;
else 												//else magnitude is falling (use m_AttackFallAlpha time constant)
	m_AttackAve = (1.0-m_AttackFallAlpha)*m_AttackAve + m_AttackFallAlpha*m_Peak;
if(m_Peak>m_DecayAve) 								//if magnitude is rising (use m_DecayRiseAlpha time constant)
{
	m_DecayAve = (1.0-m_DecayRiseAlpha)*m_DecayAve + m_DecayRiseAlpha*m_Peak;
	m_HangTimer = 0; 								//reset hang timer
}
else
{ 													//here if decreasing signal
if(m_HangTimer<m_HangTime)
	m_HangTimer++; 									//just inc and hold current m_DecayAve
else 												//else decay with m_DecayFallAlpha which is RELEASE_TIMECONST
	m_DecayAve = (1.0-m_DecayFallAlpha)*m_DecayAve + m_DecayFallAlpha*m_Peak;
}
*/
	return true;
}


/* 
 * CORE1: 
 * Execute TX branch signal processing
 */
volatile int16_t a_s_raw[15]; 						// Raw samples, minus DC bias
volatile int16_t a_s[15];							// Filtered and decimated samples
volatile int16_t a_dc;								// DC level
volatile int tx_cnt=0;								// Decimation counter
bool tx(void) 
{
	static int tx_phase = 0;
	int16_t a_sample;
	int32_t a_accu;
	int16_t qh;
	int i;
		
	/*
	 * Get sample and shift into delay line
	 */
	a_sample = adc_result[2];						// Take last ADC 2 result
	
	for (i=0; i<14; i++) 
		a_s_raw[i] = a_s_raw[i+1];					// Audio raw samples shift register

	/*
	 * Remove DC and store new sample
	 * IIR filter: dc = a*sample + (1-a)*dc  where a = 1/128
	 */
	a_sample = (a_sample&0x0fff) - ADC_BIAS;		// Clip and subtract mid-range
	a_dc = (a_sample>>7) + a_dc - (a_dc>>7);		//   then IIR running average
	a_s_raw[14] = a_sample - a_dc;					//   and subtract DC, store in shift register

	/*
	 * Low pass filter + decimation
	 */
	tx_cnt = (tx_cnt+1)&3;							// Calculate only every fourth sample
	if (tx_cnt>0) return true;
	
	for (i=0; i<14; i++) 							// Shift decimated samples
		a_s[i] = a_s[i+1];

	a_accu = 0;										// Initialize accumulator
	for (i=0; i<15; i++)							// Low pass FIR filter
		a_accu += (int32_t)a_s_raw[i]*lpf3_62[i];	// Fc=3kHz, at 62.5 kHz sampling		
	a_s[14] = a_accu >> 8;

	/*
	 * From here things get dependent on transmit mode.
	 * We assume USB for now, e.g. supporting FT8
	 */
		
	/* 
	 * Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills):
	 */	
	a_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
	qh = (int16_t)(a_accu >> 12);	 

	/* 
	 * Write I and Q to QSE DACs, phase is 7 back.
	 * Need to multiply AC with DAC_RANGE/ADC_RANGE (appr 1/16, but compensate for losses)
	 */
	pwm_set_chan_level(dac_iq, PWM_CHAN_A, DAC_BIAS + (a_s[7]/4));
	pwm_set_chan_level(dac_iq, PWM_CHAN_B, DAC_BIAS + (qh/4));

	return true;
}


/* 
 * CORE1: 
 * Timing loop, triggered through inter-core fifo 
 */
void dsp_loop()
{
	uint32_t cmd;
	uint16_t slice_num;
	
	tx_enabled = false;
	fifo_overrun = 0;	
	fifo_rx = 0;	
	fifo_tx = 0;	
	fifo_xx = 0;
	fifo_incnt++;

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
	adc_init();										// Initialize ADC to known state
	adc_set_clkdiv(0);								// Fastest clock (500 kSps)
	adc_gpio_init(26);								// GP26 is ADC 0 for I channel
	adc_gpio_init(27);								// GP27 is ADC 1 for Q channel
	adc_gpio_init(28);								// GP28 is ADC 2 for Audio channel
	adc_select_input(0);							// Start with ADC0
	adc_next = 0;
	
	adc_set_round_robin(1+2+4);						// Sequence ADC 0-1-2 free running
	adc_fifo_setup(true,false,1,false,false);		// IRQ for every result (threshold = 1)
    irq_set_exclusive_handler(ADC0_IRQ_FIFO, adcfifo_handler);
	adc_irq_set_enabled(true);
	irq_set_enabled(ADC0_IRQ_FIFO, true);
	adc_run(true);
	
	// Consider using alarm_pool_add_repeating_timer_us() for a core1 associated timer
	// First create an alarm pool on core1: alarm_pool_create(HWalarm, Ntimers)
	// For the core1 alarm pool don't use default HWalarm (usually 3) but e.g. 1
	
    while(1) 
	{
        cmd = multicore_fifo_pop_blocking();		// Wait for fifo output
		if (cmd == DSP_RX) 
		{
			fifo_rx++;
			rx();
		}
		else if (cmd == DSP_TX)
		{
			fifo_tx++;
			tx();
		}
		else
			fifo_xx++;
 		if (multicore_fifo_rvalid()) 
			fifo_overrun++;							// Check for missed events
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
		multicore_fifo_push_blocking(DSP_TX);		// Send TX to core 1 through fifo 
	else
		multicore_fifo_push_blocking(DSP_RX);		// Send RX to core 1 through fifo
	fifo_incnt++;
	return true;
}


/* 
 * CORE0: 
 * Initialize dsp context and spawn core1 process 
 */
void dsp_init() 
{
    multicore_launch_core1(dsp_loop);				// Start processing on core1
	add_repeating_timer_us(-DSP_US, dsp_callback, NULL, &dsp_timer);
}

