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
 * - Quarter rate (15.625 kHz) to improve low freq behavior of Hilbert transform
 * - Calculate 15 tap Hilbert transform on Q
 * - Demodulate, taking proper delays into account
 * - Push to Audio output DAC
 *
 * Always perform audio sampling (62.5kHz) and level detections, in case of VOX active
 *
 * The TX branch (if VOX or PTT):
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

#define ADC0_IRQ_FIFO 		22		// FIFO IRQ number
#define GP_PTT				15		// PTT pin 20 (GPIO 15)

#include "dsp.h"
#include "hmi.h"


/* 
 * DAC_RANGE defines PWM cycle, determining DAC resolution and PWM frequency.
 * DAC resolution = Vcc / DAC_RANGE
 * PWM frequency = Fsys / DAC_RANGE
 * A value of 250 means 125MHz/250=500kHz
 * ADC is 12 bit, so resolution is by definition 4096
 * To eliminate undefined behavior we clip off the upper 4 sample bits.
 */
#define DAC_RANGE	256
#define DAC_BIAS	(DAC_RANGE/2)
#define ADC_RANGE	4096
#define ADC_BIAS	(ADC_RANGE/2)

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
 * AGC reference level is log2(0x40) = 6, where 0x40 is the MSB of half DAC_RANGE
 * 1/AGC_DECAY and 1/AGC_ATTACK are multipliers before agc_gain value integrator
 * These values should ultimately be set by the HMI.
 * The time it takes to a gain change is the ( (Set time)/(signal delta) ) / samplerate
 * So when delta is 1, and attack is 64, the time is 64/15625 = 4msec (fast attack)
 * The decay time is about 100x this value
 * Slow attack would be about 4096
 */
#define AGC_REF		6
#define AGC_DECAY	8192
#define AGC_FAST	64
#define AGC_SLOW	4096
#define AGC_OFF		32766
volatile uint16_t agc_decay  = AGC_OFF;
volatile uint16_t agc_attack = AGC_OFF;
void dsp_setagc(int agc)
{
	switch(agc)
	{
	case 1:		//SLOW, for values see hmi.c
		agc_attack = AGC_SLOW;
		agc_decay  = AGC_DECAY;
		break;
	case 2:		//FAST
		agc_attack = AGC_FAST;
		agc_decay  = AGC_DECAY;
		break;
	default: 	//OFF
		agc_attack = AGC_OFF;
		agc_decay  = AGC_OFF;
		break;
	}
}

/*
 * MODE is modulation/demodulation 
 * This setting steers the signal processing branch chosen
 */
volatile uint16_t dsp_mode;				// For values see hmi.c, assume {USB,LSB,AM,CW}
void dsp_setmode(int mode)
{
	dsp_mode = (uint16_t)mode;
}




/*
 * VOX LINGER is the number of 16us cycles to wait before releasing TX mode
 * The level of detection is related to the maximum ADC range.
 */
#define VOX_LINGER		500000/16
#define VOX_HIGH		ADC_BIAS/2
#define VOX_MEDIUM		ADC_BIAS/4
#define VOX_LOW			ADC_BIAS/16
#define VOX_OFF			0
volatile uint16_t vox_count;
volatile uint16_t vox_level = VOX_OFF;
void dsp_setvox(int vox)
{
	switch(vox)
	{
	case 1:
		vox_level = VOX_LOW;
		break;
	case 2:
		vox_level = VOX_MEDIUM;
		break;
	case 3:
		vox_level = VOX_HIGH;
		break;
	default: 
		vox_level = VOX_OFF;
		vox_count = 0;
		break;
	}
}


/* 
 * Low pass filters Fc=3, 7 and 15 kHz (see http://t-filter.engineerjs.com/)
 * Settings: sample rates 62500, 31250 or 15625 Hz, stopband -40dB, passband ripple 5dB
 * Note: 8 bit precision, so divide sum by 256 (this could be improved when 32bit accumulator)
 */
int16_t lpf3_62[15] =  {  3,  3,  5,  7,  9, 10, 11, 11, 11, 10,  9,  7,  5,  3,  3};	// Pass: 0-3000, Stop: 6000-31250
int16_t lpf3_31[15] =  { -2, -3, -3,  1, 10, 21, 31, 35, 31, 21, 10,  1, -3, -3, -2};	// Pass: 0-3000, Stop: 6000-15625
int16_t lpf3_15[15] =  {  3,  4, -3,-14,-15,  6, 38, 53, 38,  6,-15,-14, -3,  4,  3};	// Pass: 0-3000, Stop: 4500-7812
int16_t lpf7_62[15] =  { -2, -1,  1,  7, 16, 26, 33, 36, 33, 26, 16,  7,  1, -1, -2};	// Pass: 0-7000, Stop: 10000-31250
int16_t lpf7_31[15] =  { -1,  4,  9,  2,-12, -2, 40, 66, 40, -2,-12,  2,  9,  4, -1};	// Pass: 0-7000, Stop: 10000-15625
int16_t lpf15_62[15] = { -1,  3, 12,  6,-12, -4, 40, 69, 40, -4,-12,  6, 12,  3, -1};	// Pass: 0-15000, Stop: 20000-31250

volatile uint16_t dac_iq, dac_audio;
volatile uint32_t fifo_overrun, fifo_rx, fifo_tx, fifo_xx, fifo_incnt;
volatile bool tx_enabled, vox_active;



/*
 * Some macro's
 * See Alpha Max plus Beta Min algorithm for MAG (vector length)
 */
#define ABS(x)		((x)<0?-(x):(x))
#define MAG(i,q)	(ABS(i)>ABS(q) ? ABS(i)+((3*ABS(q))>>3) : ABS(q)+((3*ABS(i))>>3))


/* 
 * CORE1: 
 * ADC IRQ handler.
 * Fills the results array in RR fashion, for 3 channels (~2usec per channel).
 */
volatile uint16_t adc_result[3];
volatile int adc_next;								// Remember which ADC the result is from
uint32_t adc_count=0;								// Debugging
void adcfifo_handler(void)
{
	adc_result[adc_next] = adc_fifo_get();			// Get result from fifo
	adc_next = adc_hw->cs;							// Update adc_next with HW CS register
	adc_next = (adc_next >> ADC_CS_AINSEL_LSB) & 3;	// Shift and Mask: Only 0, 1 and 2 are valid numbers
	adc_count++;
}
	

/* 
 * CORE1: 
 * Execute RX branch signal processing, max time to spend is <16us, i.e. rate is 62.5 kHz
 * No ADC sample interleaving, read both I and Q channels.
 * The delay is only 2us per conversion, which causes less distortion than interpolation of samples.
 */
volatile int16_t i_s_raw[15], q_s_raw[15];			// Raw I/Q samples minus DC bias
volatile uint16_t peak=0;							// Peak detector running value
volatile int16_t agc_gain=0;	       				// AGC gain (left-shift value)
volatile int16_t agc_accu=0;	       				// Log peak level integrator
volatile int16_t i_s[15], q_s[15];					// Filtered I/Q samples
volatile int16_t i_dc, q_dc; 						// DC bias for I/Q channel
volatile int rx_cnt=0;								// Decimation counter

bool rx(void) 
{
	int16_t q_sample, i_sample, a_sample;
	int32_t q_accu, i_accu;
	int16_t qh;
	uint16_t i;
	int16_t k;

	/*** SAMPLING ***/
	
	q_sample = adc_result[0];						// Take last ADC 0 result, connected to Q input
	i_sample = adc_result[1];						// Take last ADC 1 result, connected to I input
		
	/*
	 * Remove DC and store new sample
	 * IIR filter: dc = a*sample + (1-a)*dc  where a = 1/128
	 * Amplitude of samples should fit inside [-2048, 2047]
	 */
	q_sample = (q_sample&0x0fff) - ADC_BIAS;		// Clip to 12 bits and subtract mid-range
	q_dc += q_sample/128 - q_dc/128;				//   then IIR running average
	q_sample -= q_dc;								//   and subtract DC
	i_sample = (i_sample&0x0fff) - ADC_BIAS;		// Same for I sample
	i_dc += i_sample/128 - i_dc/128;
	i_sample -= i_dc;

	/*
	 * Shift with AGC feedback from AUDIO GENERATION stage
	 * Note: bitshift does not work with negative numbers, so need to MPY/DIV
	 * This behavior in essence is exponential, complementing the logarithmic peak detector
	 */
	if (agc_gain > 0)
	{
		q_sample = q_sample * (1<<agc_gain);
		i_sample = i_sample * (1<<agc_gain);
	}
	else if (agc_gain < 0)
	{
		q_sample = q_sample / (1<<(-agc_gain));
		i_sample = i_sample / (1<<(-agc_gain));
	}
	 
	/* 
	 * Shift-in I and Q raw samples 
	 */
	for (i=0; i<14; i++)
	{
		q_s_raw[i] = q_s_raw[i+1];					// Q raw samples shift register
		i_s_raw[i] = i_s_raw[i+1];					// I raw samples shift register
	}
	q_s_raw[14] = q_sample;							// Store in shift registers
	i_s_raw[14] = i_sample;
	
	
	/*
	 * Low pass filter + decimation
	 */
	rx_cnt = (rx_cnt+1)&3;							// Calculate only every fourth sample
	if (rx_cnt>0) return (true);					// So net sample time is 64us or 15.625 kHz
	
	for (i=0; i<14; i++) 							// Shift decimated samples
	{
		q_s[i] = q_s[i+1];
		i_s[i] = i_s[i+1];
	}
	q_accu = 0;										// Initialize accumulators
	i_accu = 0;
	for (i=0; i<15; i++)							// Low pass FIR filter
	{
		q_accu += (int32_t)q_s_raw[i]*lpf3_62[i];	// Fc=3kHz, at 62.5 kHz raw sampling		
		i_accu += (int32_t)i_s_raw[i]*lpf3_62[i];	// Fc=3kHz, at 62.5 kHz raw sampling	
	}
	q_accu = q_accu/256;
	i_accu = i_accu/256;
	
	q_s[14] = q_accu;
	i_s[14] = i_accu;
	

	/*** DEMODULATION ***/
	switch (dsp_mode)
	{
	case 0:											//USB
		/* 
		 * USB demodulate: I[7] - Qh,
		 * Qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
		 */	
		q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[ 8])*2202L;
		qh = q_accu / 4096L;	
		a_sample = i_s[7] - qh;
		break;
	case 1:											//LSB
		/* 
		 * LSB demodulate: I[7] + Qh,
		 * Qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
		 */	
		q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[ 8])*2202L;
		qh = q_accu / 4096L;	
		a_sample = i_s[7] + qh;
		break;
	case 2:											//AM
		/*
		 * AM demodulate: sqrt(sqr(i)+sqr(q))
		 * Approximated with MAG(i,q)
		 */
		a_sample = MAG(i_s[14], q_s[14]);
		break;
	default:
		break;
	}
	
	/*** AUDIO GENERATION ***/
	/*
	 * AGC, peak detector
	 * Sample speed is 15625 per second
	 */
	peak += (ABS(a_sample))/128 - peak/128;			// Running average level detect, a=1/128
	k=0; i=peak;									// Logarithmic peak detection
	if (i&0xff00) {k+=8; i>>=8;}					// k=log2(peak), find highest bit set
	if (i&0x00f0) {k+=4; i>>=4;}
	if (i&0x000c) {k+=2; i>>=2;}
	if (i&0x0002) {k+=1;}
	agc_accu += (k - AGC_REF);						// Add difference with target to integrator (Acc += Xn - R)
	if (agc_accu > agc_attack)						// Attack time, gain correction in case of high level
	{
		agc_gain--;									// Decrease gain
		agc_accu -= agc_attack;						// Reset integrator
	} else if (agc_accu < -(agc_decay))				// Decay time, gain correction in case of low level
	{
		agc_gain++;									// Increase gain
		agc_accu += agc_decay;						// Reset integrator
	}

	/*
	 * Scale and clip output,  
	 * Send to audio DAC output
	 */
	a_sample += DAC_BIAS;							// Add bias level
	if (a_sample > DAC_RANGE)						// Clip to DAC range
		a_sample = DAC_RANGE;
	else if (a_sample<0)
		a_sample = 0;
	pwm_set_chan_level(dac_audio, PWM_CHAN_A, a_sample);


	return true;
}


/* 
 * CORE1: 
 * The VOX function is called separately every cycle, to check audio level.
 * Execute TX branch signal processing when tx enabled
 */
volatile int16_t a_s_raw[15]; 						// Raw samples, minus DC bias
volatile int16_t a_level=0;							// Average level of raw sample stream
volatile int16_t a_s[15];							// Filtered and decimated samples
volatile int16_t a_dc;								// DC level
volatile int tx_cnt=0;								// Decimation counter

bool vox(void)
{
	int16_t a_sample;
	int i;

	/*
	 * Get sample and shift into delay line
	 */
	a_sample = adc_result[2];						// Get latest ADC 2 result
	

	/*
	 * Remove DC and store new raw sample
	 * IIR filter: dc = a*sample + (1-a)*dc  where a = 1/128
	 */
	a_sample = (a_sample&0x0fff) - ADC_BIAS;		// Clip and subtract mid-range
	a_dc += (a_sample - a_dc)/128;					//   then IIR running average
	a_sample -= a_dc;								//   subtract DC
	for (i=0; i<14; i++) 							//   and store in shift register
		a_s_raw[i] = a_s_raw[i+1];
	a_s_raw[14] = a_sample;
	
	/*
	 * Detect level of audio signal
	 * Return true if VOX enabled and:
	 * - Audio level higher than threshold 
	 * - Linger time sill active 
	 */
	if (a_sample<0) a_sample = -a_sample;			// Absolute value
	a_level += (a_sample - a_level)/128;			//   running average, 16usec * 128 = 2msec

	if (vox_level != VOX_OFF)						// Only when VOX is enabled
	{
		if (a_level > vox_level)
		{
			vox_count = VOX_LINGER;					// While audio present, reset linger timer
			return(true);							//  and keep TX active
		}
		if (vox_count>0)
		{
			vox_count--;							// No audio; decrement linger timer
			return(true);							//  but keep TX active
		}
	}
	return(false);									// All other cases: no TX
}

bool tx(void) 
{
	int32_t a_accu, q_accu;
	int16_t qh;
	int i;
	uint16_t i_dac, q_dac;
		
	/*** RAW Audio SAMPLES from VOX function ***/
	/*** Low pass filter + decimation ***/
	tx_cnt = (tx_cnt+1)&3;							// Calculate only every fourth sample
	if (tx_cnt>0) return true;						//   So effective sample rate will be 15625Hz

	a_accu = 0;										// Initialize accumulator
	for (i=0; i<15; i++)							// Low pass FIR filter, using raw samples
		a_accu += (int32_t)a_s_raw[i]*lpf3_62[i];	//   Fc=3kHz, at 62.5 kHz sampling		
		
	for (i=0; i<14; i++) 							// Shift decimated samples
		a_s[i] = a_s[i+1];
	a_s[14] = a_accu / 256;							// Store rescaled accumulator

	/*** MODULATION ***/
	switch (dsp_mode)
	{
	case 0:											// USB
		/* 
		 * qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
		 */	
		q_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
		qh = -(q_accu / 4096L);						// USB: sign is negative
		break;
	case 1:											// LSB
		/* 
		 * qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
		 */	
		q_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
		qh = q_accu / 4096L;						// LSB: sign is positive
		break;
	case 2:											// AM
		/*
		 * I and Q values are identical
		 */
		qh = a_s[7];
		break;
	default:
		break;
	}

	/* 
	 * Write I and Q to QSE DACs, phase is 7 samples back.
	 * Need to multiply AC with DAC_RANGE/ADC_RANGE (appr 1/8)
	 * Any case: clip to range
	 */
	a_accu = DAC_BIAS - (qh/8);
	if (a_accu<0)
		q_dac = 0;
	else if (a_accu>(DAC_RANGE-1))
		q_dac = DAC_RANGE-1;
	else
		q_dac = a_accu;
	
	a_accu = DAC_BIAS + (a_s[7]/8);
	if (a_accu<0)
		i_dac = 0;
	else if (a_accu>(DAC_RANGE-1))
		i_dac = DAC_RANGE-1;
	else
		i_dac = a_accu;
		
	// pwm_set_both_levels(dac_iq, q_dac, i_dac);		// Set both channels of the IQ slice simultaneously
	// pwm_set_chan_level(dac_iq, PWM_CHAN_A, q_dac);
	// pwm_set_chan_level(dac_iq, PWM_CHAN_B, i_dac);
	pwm_set_gpio_level(21, i_dac);
	pwm_set_gpio_level(20, q_dac);
	
	
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

	/* Initialize DACs, default mode is free running, A and B pins are output */
	gpio_set_function(20, GPIO_FUNC_PWM);			// GP20 is PWM for Q DAC (Slice 2, Channel A)
	gpio_set_function(21, GPIO_FUNC_PWM);			// GP21 is PWM for I DAC (Slice 2, Channel B)
	dac_iq = pwm_gpio_to_slice_num(20);				// Get PWM slice for GP20 (Same for GP21)
	pwm_set_clkdiv_int_frac (dac_iq, 1, 0);			// clock divide by 1: 125MHz
	pwm_set_wrap(dac_iq, DAC_RANGE-1);				// Set cycle length; nr of counts until wrap, i.e. 125/DAC_RANGE MHz
	pwm_set_enabled(dac_iq, true); 					// Set the PWM running
	
	gpio_set_function(22, GPIO_FUNC_PWM);			// GP22 is PWM for Audio DAC (Slice 3, Channel A)
	dac_audio = pwm_gpio_to_slice_num(22);			// Find PWM slice for GP22
	pwm_set_clkdiv_int_frac (dac_audio, 1, 0);		// clock divide by 1: 125MHz
	pwm_set_wrap(dac_audio, DAC_RANGE-1);			// Set cycle length; nr of counts until wrap, i.e. 125/DAC_RANGE MHz
	pwm_set_enabled(dac_audio, true); 				// Set the PWM running

	/* Initialize ADCs */
	adc_init();										// Initialize ADC to known state
	adc_set_clkdiv(0);								// Fastest clock (500 kSps)
	adc_gpio_init(26);								// GP26 is ADC 0 for Q channel
	adc_gpio_init(27);								// GP27 is ADC 1 for I channel
	adc_gpio_init(28);								// GP28 is ADC 2 for Audio channel
	adc_select_input(0);							// Start with ADC0
	adc_next = 0;
	
	adc_set_round_robin(0x01+0x02+0x04);			// Sequence ADC 0-1-2 (GP 26, 27, 28) free running
	adc_fifo_setup(true,false,1,false,false);		// IRQ for every result (fifo threshold = 1)
    irq_set_exclusive_handler(ADC0_IRQ_FIFO, adcfifo_handler);
	adc_irq_set_enabled(true);
	irq_set_enabled(ADC0_IRQ_FIFO, true);
	adc_run(true);
	
	// Consider using alarm_pool_add_repeating_timer_us() for a core1 associated timer
	// First create an alarm pool on core1: alarm_pool_create(HWalarm, Ntimers)
	// For the core1 alarm pool don't use default HWalarm (usually 3) but e.g. 1
	// Timer callback signals semaphore, while loop blocks on getting it
	
    while(1) 
	{
        cmd = multicore_fifo_pop_blocking();		// Wait for fifo output

		tx_enabled = ptt_active || vox();			// Sample audio and check level	
		if (tx_enabled)
		{
			if (vox_level != VOX_OFF) 				// Only when vox is enabled
				gpio_put(GP_PTT, false);			//     drive PTT low (active)
			tx();
		}
		else
		{
			if (vox_level != VOX_OFF)  				// Only when vox is enabled
				gpio_put(GP_PTT, true);				//     drive PTT high (inactive)
			rx();
		}
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
 * Initialize dsp context and spawn CORE1 process 
 *
 * Some CORE1 code parts should not run from Flash, but be loaded in SRAM at boot time
 * See platform.h for function qualifier macro's
 * for example: 
 * void __not_in_flash_func(funcname)(int arg1, float arg2)
 * {
 * }
 *
 * Need to set BUS_PRIORITY of Core 1 to high
 * #include bus_ctrl.h
 * bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS; // Set Core 1 prio high
 */
void dsp_init() 
{
    multicore_launch_core1(dsp_loop);				// Start processing on core1
	add_repeating_timer_us(-DSP_US, dsp_callback, NULL, &dsp_timer);
}

