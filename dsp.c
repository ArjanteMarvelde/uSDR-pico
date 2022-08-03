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

#include "dsp.h"
#include "hmi.h"
#include "fix_fft.h"

#define GP_PTT				15												// PTT pin 20 (GPIO 15)

volatile bool     tx_enabled;												// TX branch active
volatile uint32_t dsp_overrun;												// Overrun counter


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
 * S-Meter is based on RSSI, which is in fact the signal level in the preprocessor.
 * The length of the (I,Q) vector is taken as reference for RSSI, where
 * S value is highest bit set, i.e. RSSI of 512 corresponds with S-9 (S=log2(RSSI))
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
volatile uint32_t s_rssi;													// 1.. >51200
int get_sval(void)
{
	uint32_t s_val = s_rssi;
	if (s_val>S940) return(94);												// Return max 2 digits!
	if (s_val>S930) return(93);
	if (s_val>S920) return(92);
	if (s_val>S910) return(91);
	if (s_val>S9)   return(9);
	if (s_val>S8)   return(8);
	if (s_val>S7)   return(7);
	if (s_val>S6)   return(6);
	if (s_val>S5)   return(5);
	if (s_val>S4)   return(4);
	if (s_val>S3)   return(3);
	if (s_val>S2)   return(2);
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
inline int16_t mag(int16_t i, int16_t q)
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



volatile int32_t q_sample, i_sample, a_sample;								// Latest processed sample values

/*** Include the desired DSP engine ***/

#if DSP_FFT == 1
#define AGC_TOP		 2047L
#include "dsp_fft.c"
#else
#define AGC_TOP		 2047L
#include "dsp_tim.c"
#endif



/** CORE1: ADC IRQ handler **/
/*
 * The IRQ handling is redirected to a DMA channel
 * This will transfer ADC_INT samples per channel, ADC_INT maximum is 10 (would take 60usec)
 */
#define LSH 		8														// Shift for higher accuracy of level, also LPF
#define ADC_LEVELS	(ADC_BIAS/2)<<LSH										// Shifted initial ADC level
#define BSH			8														// Shift for higher accuracy of bias, also LPF
#define ADC_BIASS	ADC_BIAS<<BSH											// Shifted initial ADC bias
#define ADC_INT		8														// Nr of samples for integration (max 8)
volatile int16_t  adc_sample[ADC_INT][3];									// ADC samples collection
volatile int32_t  adc_bias[3] = {ADC_BIASS, ADC_BIASS, ADC_BIASS};			// ADC dynamic bias level
volatile int32_t  adc_result[3];											// ADC filtered result for further processing
volatile uint32_t adc_level[3] = {ADC_LEVELS, ADC_LEVELS, ADC_LEVELS};		// Levels for ADC channels
volatile int adccnt = 0;													// Sampling overflow indicator



/** CORE1: DMA IRQ handler **/

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
 * The DMA handler only stops conversions and resets interrupt flag, data is processed in timer callback.
 */
#define CH0			0
#define DMA_CTRL0	0x00120027
void __not_in_flash_func(dma_handler)(void)
{
	adc_run(false);															// Stop freerunning ADC
	dma_hw->ints0 = 1u << CH0;												// Clear the interrupt request.
	//while (!adc_fifo_is_empty()) adc_fifo_get();							// Empty leftovers from fifo
	adccnt++;																// ADC overrun indicator increment
}


/** CORE1: Timer callback routine **/

/*
 * This runs every TIM_US, i.e. 64usec, and hence determines the actual sample rate
 * The filtered samples are set aside, so a new ADC cycle can be started. 
 * One ADC cycle takes 6usec to complete (3x ADC0..2) + 1x 2usec stray ADC0 conversion.
 * The timing is critical, it assumes that the ADC is finished.
 * Do not put any other stuff in this callback routine.
 */
semaphore_t dsp_sem;
repeating_timer_t dsp_timer;
volatile int cnt = 4;
volatile int32_t rx_agc = 1, tx_agc = 1;									// Factor as AGC
bool __not_in_flash_func(dsp_callback)(repeating_timer_t *t) 
{
	int32_t temp;
	
	/* 
	 * Here the rate is 15625Hz
	 */

	// Get samples and correct for DC bias
	// RC: ((1<<BSH)-1)*64usec = 16msec
	adc_result[0] = 0;
	adc_result[1] = 0;
	adc_result[2] = 0;
	for (temp = 0; temp<ADC_INT; temp++)
	{
		adc_bias[0]   += (int32_t)(adc_sample[temp][0]) - (adc_bias[0]>>BSH);
		adc_result[0] += (int32_t)(adc_sample[temp][0]) - (adc_bias[0]>>BSH);
		adc_bias[1]   += (int32_t)(adc_sample[temp][1]) - (adc_bias[1]>>BSH);
		adc_result[1] += (int32_t)(adc_sample[temp][1]) - (adc_bias[1]>>BSH);
		adc_bias[2]   += (int32_t)(adc_sample[temp][2]) - (adc_bias[2]>>BSH);
		adc_result[2] += (int32_t)(adc_sample[temp][2]) - (adc_bias[2]>>BSH);
	}
		
	// Resstart ADCs and DMA
	adccnt--;																// ADC overrun indicator decrement
	adc_select_input(0);													// Start with ADC0
	while (!adc_fifo_is_empty()) adc_fifo_get();							// Empty leftovers from fifo, if any

	dma_hw->ch[CH0].read_addr = (io_rw_32)&adc_hw->fifo;					// Read from ADC FIFO
	dma_hw->ch[CH0].write_addr = (io_rw_32)&adc_sample[0][0];				// Write to sample buffer
	dma_hw->ch[CH0].transfer_count = ADC_INT * 3;							// Nr of 16 bit words to transfer
	dma_hw->ch[CH0].ctrl_trig = DMA_CTRL0;									// Write ctrl word without starting the DMA

	adc_run(true);															// Start ADC again

	// Calculate and save level, value is left shifted by LSH = 8
	// RC: ((1<<LSH)-1)*64usec = 16msec
	adc_level[0] += (ABS(adc_result[0]))-(adc_level[0]>>LSH);
	adc_level[1] += (ABS(adc_result[1]))-(adc_level[1]>>LSH);
	adc_level[2] += (ABS(adc_result[2]))-(adc_level[2]>>LSH);

	// Derive RSSI value from RX vector length
	// Crude AGC mechanism **TO BE IMPROVED**
	if (!tx_enabled)	
	{
		// Approximate amplitude, with alpha max + beta min function
		uint32_t i=adc_level[1],q=adc_level[0];
		if (i>q)
			temp = (MAX(i,((29*i/32) + (61*q/128))))>>LSH;
		else
			temp = (MAX(q,((29*q/32) + (61*i/128))))>>LSH;
		s_rssi = MAX(1,temp);
		rx_agc = AGC_TOP/s_rssi;											// calculate scaling factor
		if (rx_agc==0) rx_agc=1;
	}
		
#if DSP_FFT == 1

	if (tx_enabled)
	{								
		A_buf[dsp_active][dsp_tick] = (int16_t)(tx_agc*adc_result[2]);
		pwm_set_gpio_level(21, I_buf[dsp_active][dsp_tick] + DAC_BIAS);		// Output I to DAC
		pwm_set_gpio_level(20, Q_buf[dsp_active][dsp_tick] + DAC_BIAS);		// Output Q to DAC
	}
	else
	{
		I_buf[dsp_active][dsp_tick] = (int16_t)(rx_agc*adc_result[1]);
		Q_buf[dsp_active][dsp_tick] = (int16_t)(rx_agc*adc_result[0]);
		pwm_set_gpio_level(22, A_buf[dsp_active][dsp_tick] + DAC_BIAS);		// Output A to DAC
	}
	
	// When sample buffer is full, move pointer to next and signal the DSP loop
	if (++dsp_tick >= BUFSIZE)												// Increment tick and check range
	{
		dsp_tick = 0;														// Reset counter
		if (++dsp_active > 2) dsp_active = 0;								// Rotate offset
		dsp_overrun++;														// Increment overrun counter
		sem_release(&dsp_sem);												// Signal DSP loop semaphore
	}
	
#else

	if (tx_enabled)
	{
		a_sample = tx_agc * adc_result[2];									// Store A for DSP use
		pwm_set_gpio_level(21, i_sample);									// Output I to DAC
		pwm_set_gpio_level(20, q_sample);									// Output Q to DAC
	}
	else
	{							
		pwm_set_gpio_level(22, a_sample);									// Output Q to DAC
		q_sample = rx_agc * adc_result[0];									// Store Q for DSP use
		i_sample = rx_agc * adc_result[1];									// Store I for DSP use
	}
	dsp_overrun++;															// Increment overrun counter
	sem_release(&dsp_sem);													// Signal DSP loop semaphore

#endif
		

	return true;
}
	


/** CORE1: DSP loop, triggered through repeating timer/semaphore **/
void __not_in_flash_func(dsp_loop)()
{
	uint32_t cmd;
	uint16_t slice_num;
	alarm_pool_t *ap;
	
	tx_enabled = false;	
	vox_active = false;
	
	/* 
	 * Initialize DACs, 
	 * default mode is free running, 
	 * A and B pins are output 
	 */
	gpio_set_function(20, GPIO_FUNC_PWM);									// GP20 is PWM for Q DAC (Slice 2, Channel A)
	gpio_set_function(21, GPIO_FUNC_PWM);									// GP21 is PWM for I DAC (Slice 2, Channel B)
	dac_iq = pwm_gpio_to_slice_num(20);										// Get PWM slice for GP20 (Same for GP21)
	pwm_set_clkdiv_int_frac (dac_iq, 1, 0);									// clock divide by 1: full system clock
	pwm_set_wrap(dac_iq, DAC_RANGE-1);										// Set cycle length; nr of counts until wrap, i.e. 125/DAC_RANGE MHz
	pwm_set_enabled(dac_iq, true); 											// Set the PWM running
	
	gpio_set_function(22, GPIO_FUNC_PWM);									// GP22 is PWM for Audio DAC (Slice 3, Channel A)
	dac_audio = pwm_gpio_to_slice_num(22);									// Find PWM slice for GP22
	pwm_set_clkdiv_int_frac (dac_audio, 1, 0);								// clock divide by 1: full system clock
	pwm_set_wrap(dac_audio, DAC_RANGE-1);									// Set cycle length; nr of counts until wrap, i.e. 125/DAC_RANGE MHz
	pwm_set_enabled(dac_audio, true); 										// Set the PWM running

	/* 
	 * Initialize ADCs, use in round robin mode (3 channels)
	 * samples are stored in array through IRQ callback
	 */
	adc_init();																// Initialize ADC to known state
	adc_gpio_init(26);														// GP26 is ADC 0 for Q channel
	adc_gpio_init(27);														// GP27 is ADC 1 for I channel
	adc_gpio_init(28);														// GP28 is ADC 2 for Audio channel
	adc_set_round_robin(0x01+0x02+0x04);									// Sequence ADC 0-1-2 (GP 26, 27, 28) free running
	adc_select_input(0);													// Start with ADC0
	adc_fifo_setup(true,true,3,false,false);								// IRQ result, DMA req, fifo thr=3: xfer per 3 x 16 bits
	adc_set_clkdiv(0);														// Fastest clock (500 kSps)

	/*
	 * Setup and start DMA channel CH0
	 */
	dma_channel_set_irq0_enabled(CH0, true);								// Raise IRQ line 0 when the channel finishes a block
	irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);						// Install IRQ handler
	irq_set_enabled(DMA_IRQ_0, true);										// Enable it
	irq_set_priority(DMA_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);					// Prevent race condition with timer
	
	dma_hw->ch[CH0].read_addr = (io_rw_32)&adc_hw->fifo;					// Read from ADC FIFO
	dma_hw->ch[CH0].write_addr = (io_rw_32)&adc_sample[0][0];				// Write to sample buffer
	dma_hw->ch[CH0].transfer_count = ADC_INT * 3;							// Nr of 16 bit words to transfer
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
	 *                                          int64_t delay_us, 
	 *                                         repeating_timer_callback_t callback, 
	 *                                         void *user_data, 
	 *                                         repeating_timer_t *out);
	 */
	sem_init(&dsp_sem, 0, 1);
	ap = alarm_pool_create(1, 4);
	alarm_pool_add_repeating_timer_us( ap, -TIM_US, dsp_callback, NULL, &dsp_timer);

	dsp_overrun = 0;
	
    while(1) 
	{
		sem_acquire_blocking(&dsp_sem);										// Wait until timer callback releases sem
		dsp_overrun--;														// Decrement overrun counter

		// Use adc_level[2] for VOX
		if (vox_level == 0)													// Only when VOX is enabled
			vox_active = false;												// Normally false
		else
		{
			if ((adc_level[2]>>LSH) > vox_level)							// AND level > limit
			{
				vox_count = S_RATE * VOX_LINGER / 1000;						// While audio present, reset linger timer
				vox_active = true;											//  and keep TX active
			}
			else if (--vox_count>0)											// else decrement linger counter
				vox_active = true;											//  and keep TX active until 0
		}


		if (tx_enabled)														// Use previous setting
		{
			gpio_put(GP_PTT, false); 										// Drive PTT low (active)  
			tx();															// Do TX signal processing
		}
		else
		{
			gpio_put(GP_PTT, true);											// Drive PTT high (inactive)   
			rx();															// Do RX signal processing
		}
		
/////// This is a trap, ptt remains active after once asserted: to be checked!
		tx_enabled = vox_active || ptt_active;								// Check RX or TX	
		
#if DSP_FFT == 1
		dsp_tickx = dsp_tick;
#endif
	}
}




/** CORE0: Initialize dsp context and spawn CORE1 process **/
void dsp_init() 
{
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS; 				// Set Core 1 prio on bus to high
	multicore_launch_core1(dsp_loop);										// Start processing on Core 1
}



/* DMA EXAMPLE, should convert to chained DMA to reload after 3 words
    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    adc_set_clkdiv(0);

    printf("Arming DMA\n");
    sleep_ms(1000);
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        capture_buf,    // dst
        &adc_hw->fifo,  // src
        CAPTURE_DEPTH,  // transfer count
        true            // start immediately
    );

    printf("Starting capture\n");
    adc_run(true);

    // Once DMA finishes, stop any new conversions from starting, and clean up
    // the FIFO in case the ADC was still mid-conversion.
    dma_channel_wait_for_finish_blocking(dma_chan);
    printf("Capture finished\n");
    adc_run(false);
    adc_fifo_drain();

    // Print samples to stdout so you can display them in pyplot, excel, matlab
    for (int i = 0; i < CAPTURE_DEPTH; ++i) {
        printf("%-3d, ", capture_buf[i]);
        if (i % 10 == 9)
            printf("\n");
    }
*/
