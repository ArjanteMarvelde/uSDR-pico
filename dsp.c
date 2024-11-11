/*
 * dsp.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * Signal processing of RX and TX branch, to be run on the second processor core (CORE1).
 * 
 * The actual DSP engine can be either FFT based in the frequency domain, or in the time domain.
 * In dsp.h this can be selected compile-time, by defining the environment variable DSP_FFT.
 *
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "pico/sem.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "uSDR.h"
#include "dsp.h"
#include "hmi.h"
#include "fix_fft.h"


volatile bool    tx_enabled;												// TX branch active
volatile int32_t dsp_overrun;												// Overrun counter (could be underrun too)


/* 
 * DAC_RANGE defines PWM cycle, determining DAC resolution and PWM frequency.
 * DAC resolution = Vcc / DAC_RANGE
 * PWM frequency = Fsys / DAC_RANGE
 * A value of 250 means 125MHz/250=500kHz
 * ADC is 12 bit, so resolution is by definition 4096
 */
#define DAC_RANGE	256
#define DAC_BIAS	(DAC_RANGE/2)
#define ADC_RANGE	4096
#define ADC_BIAS	(ADC_RANGE/2)


volatile uint16_t dac_iq, dac_audio;


/*** External Interfaces, mostly used by hmi.c ***/

/*
 * MODE is modulation/demodulation 
 * This setting steers the signal processing branch chosen
 */
volatile int dsp_mode;
void dsp_setmode(int mode)
{
	dsp_mode = mode;
}


/*
 * S-Meter is for now based on RSSI, which is in fact the signal level in the preprocessor, 
 * in uV equivalent. The S level makes 6dB steps, i.e. factor 2 in voltage. The length of 
 * the (I,Q) vector (amplitude) of Rx signal is taken as reference for RSSI. This covers the 
 * whole sampled RX band, and is not specific to a particular station. To realize that, the 
 * S value must be calculated after FFT and for the specific tuning frequency.
 * Returned S value is highest bit set, i.e. RSSI of 512 corresponds with S9 (S=log2(RSSI))
 * This value was calibrated roughly by using the sam antenna and 
 *   comparing an IC R71-E with my uSDR HW implementation and ADC_INT=8.
 * +20dB means 10x the S-9 RSSI level, or >5120
 * +40dB means 100x the S-9 RSSI level, or >51200
 */
#define S940	51200
#define S930	16180
#define S920	5120
#define S910	1618
#define S9		512
#define S8		256
#define S7		128
#define S6		64
#define S5		32
#define S4		16
#define S3		8
#define S2		4
#define S1		2

#define LSH 	8															// Level SHift for level LPF: 2^LSH

volatile uint16_t dsp_rssi, dsp_vox;										// Signal levels for IF and Audio ADC channels, fixed point (<<16)
int get_sval(void)
{
	uint32_t sval = GET_RSSI_LEVEL;
	if (sval>S940) return(94);												// Return max 2 digits!
	if (sval>S930) return(93);
	if (sval>S920) return(92);
	if (sval>S910) return(91);
	if (sval>S9)   return(9);
	if (sval>S8)   return(8);
	if (sval>S7)   return(7);
	if (sval>S6)   return(6);
	if (sval>S5)   return(5);
	if (sval>S4)   return(4);
	if (sval>S3)   return(3);
	if (sval>S2)   return(2);
	return(1);
}

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
#define AGC_SHORT	64
#define AGC_LONG	4096
#define AGC_DIS		32766
#define RXAGC_TOP	2047
#define TXAGC_TOP	2047
volatile uint16_t agc_decay  = AGC_DIS;
volatile uint16_t agc_attack = AGC_DIS;

void dsp_setagc(int agc)
{
	switch(agc)
	{
	case AGC_SLOW:
		agc_attack = AGC_LONG;
		agc_decay  = AGC_DECAY;
		break;
	case AGC_FAST:
		agc_attack = AGC_SHORT;
		agc_decay  = AGC_DECAY;
		break;
	default:
		agc_attack = AGC_DIS;
		agc_decay  = AGC_DIS;
		break;
	}
}



/*
 * VOX LINGER is the number msec to wait before releasing TX mode
 * The level of detection is derived from the maximum ADC range.
 */
#define VOX_LINGER		500													// 500msec

volatile uint16_t vox_count = 0;
volatile uint16_t vox_level = 0;
volatile bool	  vox_active;												// Is set when audio energy > vox level (and not OFF)
void dsp_setvox(int vox)
{
	switch(vox)
	{
	case VOX_HIGH:
		vox_level = ADC_BIAS/2;
		break;
	case VOX_MEDIUM:
		vox_level = ADC_BIAS/4;
		break;
	case VOX_LOW:
		vox_level = ADC_BIAS/16;
		break;
	default: 
		vox_level = 0;
		vox_count = 0;
		break;
	}
}


/*** Some handy macro's ***/

#define ABS(x)		( (x)<0   ? -(x) : (x) )
 
/*
 * Calculation of vector length:
 * Z = alpha*max(i,q) + beta*min(i,q); 
 * alpha = 1/1, beta = 3/8 (error<6.8%)
 * alpha = 15/16, beta = 15/32 (error<6.25%)
 * Better algorithm:
 * Z = max( max(i,q), alpha*max(i,q)+beta*min(i,q) )
 * alpha = 29/32, beta = 61/128 (error<2.4%)
 */
inline uint16_t mag(int16_t i, int16_t q)
{
	i = ABS(i); q = ABS(q);
	if (i>q)
		return (MAX(i,((29*i/32) + (61*q/128))));
	else
		return (MAX(q,((29*q/32) + (61*i/128))));
}

/* 
 * Note: A simple regression IIR single pole low pass filter could be made for anti-aliasing.
 *  y(n) = (1-a)*y(n-1) + a*x(n) = y(n-1) + a*(x(n) - y(n-1))
 * in this a = T / (T + R*C)  
 * Example:
 *    T is sample period (e.g. 64usec) 
 *    RC the desired RC time: T*(1-a)/a.
 *    example: a=1/256 : RC = 255*64usec = 16msec (65Hz)
 * Alternative faster implementation with higher accuracy
 *  y(n) = y(n-1) + (x(n) - y(n-1)>>b)
 * Here the filtered value is maintained in higher accuracy, i.e. left shifted by b bits.
 * Before using the value: y >> b. 
 * Also, for RC value 1/a = 1<<b, or RC = ((1<<b)-1)*64us
 */



/*** Include the desired DSP engine ***/

#if DSP_FFT == 1
#include "dsp_fft.c"
#else
#include "dsp_tim.c"
#endif




/** CORE1: ADC IRQ handler **/
/** CORE1: DMA IRQ handler **/
/*
 * The IRQ handling is redirected to a DMA channel
 * This will transfer ADC_INT samples per channel, ADC_INT maximum is 10 (would take 60usec) but safer to use 8
 * These are all registers used for the sample acquisition process
 * A sample is a value between 0..4095, in a 16 bit unsigned integer
 * The DC bias value is somewhere around 2048, but will depend on the analogue circuits
 * The DC bias value is stored left shifted (by 16 as 32bit unsigned integer), to maintain precision
 */
 
// From RP2040 datasheet, DMA Status/Control register layout
// 0x80000000 [31]    : AHB_ERROR (0): Logical OR of the READ_ERROR and WRITE_ERROR flags
// 0x40000000 [30]    : READ_ERROR (0): If 1, the channel received a read bus error
// 0x20000000 [29]    : WRITE_ERROR (0): If 1, the channel received a write bus error
// 0x01000000 [24]    : BUSY (0): This flag goes high when the channel starts a new transfer sequence, and low when the...
// 0x00800000 [23]    : SNIFF_EN (0): If 1, this channel's data transfers are visible to the sniff hardware, and each...
// 0x00400000 [22]    : BSWAP (0): Apply byte-swap transformation to DMA data
// 0x00200000 [21]    : IRQ_QUIET (0): In QUIET mode, the channel does not generate IRQs at the end of every transfer block
// 0x001f8000 [20:15] : TREQ_SEL (0): Select a Transfer Request signal
// 0x00007800 [14:11] : CHAIN_TO (0): When this channel completes, it will trigger the channel indicated by CHAIN_TO
// 0x00000400 [10]    : RING_SEL (0): Select whether RING_SIZE applies to read or write addresses
// 0x000003c0 [9:6]   : RING_SIZE (0): Size of address wrap region
// 0x00000020 [5]     : INCR_WRITE (0): If 1, the write address increments with each transfer
// 0x00000010 [4]     : INCR_READ (0): If 1, the read address increments with each transfer
// 0x0000000c [3:2]   : DATA_SIZE (0): Set the size of each bus transfer (byte/halfword/word)
// 0x00000002 [1]     : HIGH_PRIORITY (0): HIGH_PRIORITY gives a channel preferential treatment in issue scheduling: in...
// 0x00000001 [0]     : EN (0): DMA Channel Enable

// 0x00120027 (IRQ_QUIET=0x0, TREQ_SEL=0x24, CHAIN_TO=0, INCR_WRITE=1, INCR_READ=0, DATA_SIZE=1, HIGH_PRIORITY=1, EN=1)	

/*
 * The dma_handler is called when the sample buffer adc_sample[][] is full.
 * It only stops ADC conversions and resets DMA interrupt flag, samples are processed in timeout dsp_callback routine.
 */
#define CH0			0
#define DMA_CTRL0	0x00120027
volatile int      adccnt = 0;												// Sampling overflow indicator, negative when timeout is too soon
void __not_in_flash_func(dma_handler)(void)
{
	adccnt++;																// ADC overrun indicator increment
	adc_run(false);															// Stop freerunning ADC
	dma_hw->ints0 = 1u << CH0;												// Clear the interrupt request.
	//while (!adc_fifo_is_empty()) adc_fifo_get();							// Empty leftovers from fifo
}




/** CORE1: Timer callback routine **/

/*
 * This runs every TIM_US, i.e. 64usec, and hence determines the actual sample rate
 * The filtered samples are set aside, so a new ADC cycle can be started. 
 * One ADC cycle takes 6usec to complete (3x ADC0..2) + 1x 2usec stray ADC0 conversion.
 * The timing is critical, it assumes that the ADC is finished.
 * --> Do not put any other stuff in this callback routine that affects timing
 */

#define ADC_INT		8														// Nr of samples for integration (use 8=2^3)
#define BSH			8														// Bias SHift for moving average; 2^BSH
#define DC_LEN		(1<<BSH)												// Length of DC level delay line (initially all ADC_BIAS)
#define SUM_BIAS	(ADC_BIAS*DC_LEN)										// Sum of samples in delay line (initially DC_LEN*ADC_BIAS)
volatile uint16_t adc_sample[ADC_INT][3];									// ADC sample buffers, filled by DMA (one per channel)
volatile uint16_t adc_movavg[DC_LEN][3];									// ADC DC level running average sample delay lines
volatile uint32_t adc_sumbias[3] = {SUM_BIAS, SUM_BIAS, SUM_BIAS};			// ADC dynamic bias (DC) level, summed delay line
volatile uint16_t adc_bias[3] = {ADC_BIAS, ADC_BIAS, ADC_BIAS};				// ADC dynamic bias (DC) level
volatile int16_t  adc_result[3];											// Pre-processed sample, for each channel
volatile int      adc_i = 0;												// Points into delay line
volatile uint16_t rx_agc = 1, tx_agc = 1;									// Factor as AGC

semaphore_t dsp_sem;														// Semaphore to trigger dsp loop
repeating_timer_t dsp_timer;												// TIM_US timer
bool __not_in_flash_func(dsp_callback)(repeating_timer_t *t) 				// Timer callback routine
{
	uint32_t temp;
	
	/** Here the rate is: S_RATE=1/TIM_US, --> 15625Hz **/

	// Add up the ADC_INT samples for each channel, after removing DC
	// Increase dynamic range, implicit LPF, maybe better use proper filter coefficients
	adc_result[CH_Q] = 0;
	adc_result[CH_I] = 0;
	adc_result[CH_A] = 0;
	for (temp = 0; temp<ADC_INT; temp++)
	{
		adc_result[CH_Q] += (int16_t)(adc_sample[temp][CH_Q]) - adc_bias[CH_Q];
		adc_result[CH_I] += (int16_t)(adc_sample[temp][CH_I]) - adc_bias[CH_I];
		adc_result[CH_A] += (int16_t)(adc_sample[temp][CH_A]) - adc_bias[CH_A];
	}
	
	// Calculate new bias / sumbias values and replace sample in delay line
	adc_sumbias[CH_Q] += adc_sample[0][CH_Q] - adc_movavg[adc_i][CH_Q]; adc_bias[CH_Q] = adc_sumbias[CH_Q]>>BSH;
	adc_sumbias[CH_I] += adc_sample[0][CH_I] - adc_movavg[adc_i][CH_I]; adc_bias[CH_I] = adc_sumbias[CH_I]>>BSH;
	adc_sumbias[CH_A] += adc_sample[0][CH_A] - adc_movavg[adc_i][CH_A]; adc_bias[CH_A] = adc_sumbias[CH_A]>>BSH;
	adc_movavg[adc_i][CH_Q] = adc_sample[0][CH_Q];
	adc_movavg[adc_i][CH_I] = adc_sample[0][CH_I];
	adc_movavg[adc_i][CH_A] = adc_sample[0][CH_A];
	if (++adc_i >= DC_LEN) adc_i = 0;

	// Kick-off a new acquisition phase
	// So restart ADCs and DMA channel
	adc_select_input(0);													// Start with ADC0
	while (!adc_fifo_is_empty()) adc_fifo_get();							// Empty leftovers from fifo, if any

	dma_hw->ch[CH0].read_addr = (io_rw_32)&adc_hw->fifo;					// Read from ADC FIFO
	dma_hw->ch[CH0].write_addr = (io_rw_32)&adc_sample[0][0];				// Write to sample buffer
	dma_hw->ch[CH0].transfer_count = ADC_INT * 3;							// Nr of 16 bit words to transfer
	dma_hw->ch[CH0].ctrl_trig = DMA_CTRL0;									// Write ctrl word while starting the DMA
	adc_run(true);															// Start the ADC too
	adccnt--;																// ADC overrun indicator decrement

	// Derive RSSI value from RX vector length
	// Crude AGC mechanism **NEEDS TO BE IMPROVED**
	if (!tx_enabled)	
	{
		temp = mag(adc_result[CH_I], adc_result[CH_Q]);						// Approximate amplitude, with alpha max + beta min function
		temp = MAX(1,temp);													// Prevent 0 level
		dsp_rssi += (temp - dsp_rssi) >> LSH;								// Promote temp to fixed point, then LPF
		
		rx_agc = RXAGC_TOP/GET_RSSI_LEVEL;									// Calculate scaling factor (max level/actual level)
		if (rx_agc==0) rx_agc=1;											// Shouldn't ever happen
	}
	
	// Calculate VOX level
	dsp_vox += (ABS(adc_result[CH_A]) - dsp_vox) >> LSH;					// Running average of audio input level
	tx_agc = TXAGC_TOP/GET_VOX_LEVEL;										// Calculate scaling factor (max level/actual level)
	if (tx_agc==0) tx_agc=1;												// Shouldn't ever happen
		
#if DSP_FFT == 1

	// Copy samples from/to the right buffers
	if (tx_enabled)
	{								
		A_buf[dsp_active][dsp_tick] = (int16_t)(tx_agc*adc_result[CH_A]);	// Copy A sample to A buffer
		pwm_set_gpio_level(DAC_I, I_buf[dsp_active][dsp_tick] + DAC_BIAS);	// Output I to DAC
		pwm_set_gpio_level(DAC_Q, Q_buf[dsp_active][dsp_tick] + DAC_BIAS);	// Output Q to DAC
	}
	else
	{
		I_buf[dsp_active][dsp_tick] = (int16_t)(rx_agc*adc_result[CH_I]);	// Copy I sample to I buffer
		Q_buf[dsp_active][dsp_tick] = (int16_t)(rx_agc*adc_result[CH_Q]);	// Copy Q sample to Q buffer
		pwm_set_gpio_level(DAC_A, A_buf[dsp_active][dsp_tick] + DAC_BIAS);	// Output A to DAC
	}
	
	// When buffers are full, move pointer to the next and signal the DSP loop
	if (++dsp_tick >= BUFSIZE)												// Increment tick and check range
	{
		dsp_overrun++;														// Increment overrun counter
		dsp_tick = 0;														// Reset counter
		if (++dsp_active > 2) dsp_active = 0;								// Point to next buffer
		sem_release(&dsp_sem);												// Signal background processing
	}
	
#else

	// Copy samples from/to the right buffers
	if (tx_enabled)
	{
		a_sample = tx_agc * adc_result[CH_A];								// Store A sample for background processing
		pwm_set_gpio_level(DAC_I, i_sample);								// Output calculated I sample to DAC
		pwm_set_gpio_level(DAC_Q, q_sample);								// Output calculated Q sample to DAC
	}
	else
	{							
		q_sample = rx_agc * adc_result[CH_Q];								// Store Q sample for background processing
		i_sample = rx_agc * adc_result[CH_I];								// Store I sample for background processing
		pwm_set_gpio_level(DAC_A, a_sample);								// Output calculated A sample to DAC
	}
	dsp_overrun++;															// Increment overrun counter
	sem_release(&dsp_sem);													// Signal background processing

#endif
		

	return true;
}


/** CORE1: DSP loop **/
/*
 * Background signal processing, 
 * triggered through repeating timer (dsp_callback) and semaphore
 * This also initializes all DSP environment
 */
void __not_in_flash_func(dsp_loop)()
{
	alarm_pool_t *ap;
	int i;
	
	tx_enabled = false;	
	vox_active = false;
	
	/* 
	 * Initialize DACs, 
	 * default mode is free running, 
	 * A and B pins are output 
	 */
	gpio_set_function(DAC_Q, GPIO_FUNC_PWM);								// GP20 is PWM for Q DAC (Slice 2, Channel A)
	gpio_set_function(DAC_I, GPIO_FUNC_PWM);								// GP21 is PWM for I DAC (Slice 2, Channel B)
	dac_iq = pwm_gpio_to_slice_num(DAC_Q);									// Get PWM slice for GP20 (Same for GP21)
	pwm_set_clkdiv_int_frac (dac_iq, 1, 0);									// clock divide by 1: full system clock
	pwm_set_wrap(dac_iq, DAC_RANGE-1);										// Set cycle length; nr of counts until wrap, i.e. 125/DAC_RANGE MHz
	pwm_set_enabled(dac_iq, true); 											// Set the PWM running
	
	gpio_set_function(DAC_A, GPIO_FUNC_PWM);								// GP22 is PWM for Audio DAC (Slice 3, Channel A)
	dac_audio = pwm_gpio_to_slice_num(DAC_A);								// Find PWM slice for GP22
	pwm_set_clkdiv_int_frac (dac_audio, 1, 0);								// clock divide by 1: full system clock
	pwm_set_wrap(dac_audio, DAC_RANGE-1);									// Set cycle length; nr of counts until wrap, i.e. 125/DAC_RANGE MHz
	pwm_set_enabled(dac_audio, true); 										// Set the PWM running

	/* 
	 * Initialize ADCs, use in round robin mode (3 channels)
	 * samples are stored in array through IRQ callback
	 * Initialize ADC delay lines for DC moving average
	 */
	adc_init();																// Initialize ADC to known state
	adc_gpio_init(ADC_Q);													// ADC GPIO for Q channel
	adc_gpio_init(ADC_I);													// ADC GPIO for I channel
	adc_gpio_init(ADC_A);													// ADC GPIO for Audio channel
	adc_set_round_robin(0x01+0x02+0x04);									// Sequence ADC 0-1-2 (GP 26, 27, 28) free running
	adc_select_input(0);													// Start with ADC0
	adc_fifo_setup(true,true,3,false,false);								// IRQ result, DMA req, fifo thr=3: xfer per 3 x 16 bits
	adc_set_clkdiv(0);														// Fastest clock (500 kSps)
	for (i=0; i<DC_LEN; i++)
	{
		adc_movavg[i][CH_Q] = ADC_BIAS;
		adc_movavg[i][CH_I] = ADC_BIAS;
		adc_movavg[i][CH_A] = ADC_BIAS;
	}

	/*
	 * Setup and start DMA channel CH0
	 */
	dma_channel_set_irq0_enabled(CH0, true);								// Raise IRQ line 0 when the channel finishes a block
	irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);						// Install IRQ handler
	irq_set_enabled(DMA_IRQ_0, true);										// Enable it
	irq_set_priority(DMA_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);					// Prevent race condition with timer
	
	dma_hw->ch[CH0].read_addr = (io_rw_32)&adc_hw->fifo;					// Read from ADC FIFO
	dma_hw->ch[CH0].write_addr = (io_rw_32)&adc_sample[0][0];				// Write to sample buffer
	dma_hw->ch[CH0].transfer_count = ADC_INT * 3;							// Nr of 16 bit words to transfer (interrupt when done)
	dma_hw->ch[CH0].ctrl_trig = DMA_CTRL0;									// Write ctrl word and start the DMA

	adc_run(true);															// Also start the ADC

	
	/*
	 * Use alarm_pool_add_repeating_timer_us() for a core1 associated timer
	 * First create an alarm pool on core1:
	 * alarm_pool_t *alarm_pool_create( uint hardware_alarm_num, 
	 *                                  uint max_timers);
	 * For the core1 alarm pool don't use the default alarm_num (usually 3) but e.g. 1
	 * Timer callback signals semaphore, while loop blocks on getting it.
	 * Initialize repeating timer on core1:
	 * bool alarm_pool_add_repeating_timer_us( alarm_pool_t *pool, 
	 *                                         int64_t delay_us, 
	 *                                         repeating_timer_callback_t callback, 
	 *                                         void *user_data, 
	 *                                         repeating_timer_t *out);
	 */
	sem_init(&dsp_sem, 0, 1);
	ap = alarm_pool_create(1, 4);
	alarm_pool_add_repeating_timer_us( ap, -TIM_US, dsp_callback, NULL, &dsp_timer);

	dsp_overrun = 0;
	
	// Background processing loop
    while(1) 
	{
		sem_acquire_blocking(&dsp_sem);										// Wait until timer-callback releases sem

		// Use dsp_vox value
		if (vox_level == 0)													// Only when VOX is enabled
			vox_active = false;												// De-activate in case it was active
		else
		{
			if ((dsp_vox>>LSH) > vox_level)									// AND actual level > limit level
			{
				vox_count = S_RATE * VOX_LINGER / 1000;						// While audio present, reset linger timer
				vox_active = true;											//  and keep TX active
			}
			else if (--vox_count>0)											// else decrement linger counter
				vox_active = true;											//  and keep TX active until 0
		}


		if (tx_enabled)														// Use previous setting
		{
			gpio_put(GP_PTT_OUT, true); 									// Drive PTT high (active)  
			tx();															// Do TX signal processing (Freq or Time domain)
		}
		else
		{
			gpio_put(GP_PTT_OUT, false);									// Drive PTT low (inactive)   
			rx();															// Do RX signal processing (Freq or Time domain)
		}
		
		/** Activate transmission **/
		tx_enabled = vox_active || ptt_active;								// Either VOX or PTT 	

		dsp_overrun--;														// Decrement overrun counter
		
#if DSP_FFT == 1
		dsp_tickx = dsp_tick;												// Capture how far we are in sampling a FFT buffer
#endif
	}
}




/** CORE0: Initialize dsp context and spawn CORE1 process **/
void dsp_init() 
{
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS; 				// Set Core 1 prio on bus to high
	multicore_launch_core1(dsp_loop);										// Start processing on Core 1
}




