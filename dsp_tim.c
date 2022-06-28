/*
 * dsp_tim.c
 * ==>TO BE INCLUDED IN dsp.c
 *
 * Created: May 2022
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


/* 
 * Low pass FIR filters Fc=3, 7 and 15 kHz (see http://t-filter.engineerjs.com/)
 * Settings: sample rates 62500, 31250 or 15625 Hz, stopband -40dB, passband ripple 5dB
 * Note: 8 bit precision, so divide sum by 256 (this could be improved when 32bit accumulator)
 */
int16_t lpf3_62[15] =  {  3,  3,  5,  7,  9, 10, 11, 11, 11, 10,  9,  7,  5,  3,  3};	// Pass: 0-3000, Stop: 6000-31250
int16_t lpf3_31[15] =  { -2, -3, -3,  1, 10, 21, 31, 35, 31, 21, 10,  1, -3, -3, -2};	// Pass: 0-3000, Stop: 6000-15625
int16_t lpf3_15[15] =  {  3,  4, -3,-14,-15,  6, 38, 53, 38,  6,-15,-14, -3,  4,  3};	// Pass: 0-3000, Stop: 4500-7812
int16_t lpf7_62[15] =  { -2, -1,  1,  7, 16, 26, 33, 36, 33, 26, 16,  7,  1, -1, -2};	// Pass: 0-7000, Stop: 10000-31250
int16_t lpf7_31[15] =  { -1,  4,  9,  2,-12, -2, 40, 66, 40, -2,-12,  2,  9,  4, -1};	// Pass: 0-7000, Stop: 10000-15625
int16_t lpf15_62[15] = { -1,  3, 12,  6,-12, -4, 40, 69, 40, -4,-12,  6, 12,  3, -1};	// Pass: 0-15000, Stop: 20000-31250




/** CORE1: RX Branch **/

/* 
 * Execute RX branch signal processing
 */
volatile int32_t i_s_raw[15], q_s_raw[15];									// Raw I/Q samples minus DC bias
volatile uint16_t peak=0;													// Peak detector running value
volatile int16_t agc_gain=0;	       										// AGC gain (left-shift value)
volatile int16_t agc_accu=0;	       										// Log peak level integrator
volatile int32_t i_s[15], q_s[15];											// Filtered I/Q samples
volatile int32_t i2, q2;													// Squared samples
bool __not_in_flash_func(rx)(void) 
{
	int32_t q_accu, i_accu;
	int32_t qh;
	uint16_t i;
	int16_t k;
	
	/*** SAMPLING ***/
	/* 
	 * Shift-in I and Q raw samples 
	 */
	for (i=0; i<14; i++)
	{
		q_s_raw[i] = q_s_raw[i+1];
		i_s_raw[i] = i_s_raw[i+1];
	}
	q_s_raw[14] = q_sample;													// Store decimated samples in shift registers
	i_s_raw[14] = i_sample;
	
	
	/*
	 * Low pass FIR filter
	 */
	q_accu = 0;																// Initialize accumulators
	i_accu = 0;
	for (i=0; i<15; i++)													// Low pass FIR filter
	{
		q_accu += (int32_t)q_s_raw[i]*lpf3_15[i];							// Fc=3kHz, at 15625 Hz sampling		
		i_accu += (int32_t)i_s_raw[i]*lpf3_15[i];							// Fc=3kHz, at 15625 Hz sampling	
	}
	q_accu = q_accu/256;
	i_accu = i_accu/256;
	
	for (i=0; i<14; i++) 													// Shift filtered samples
	{
		q_s[i] = q_s[i+1];
		i_s[i] = i_s[i+1];
	}
	q_s[14] = q_accu;
	i_s[14] = i_accu;
	

	/*** DEMODULATION ***/
	switch (dsp_mode)
	{
	case MODE_USB:
		/* 
		 * USB demodulate: I[7] - Qh,
		 * Qh is Classic Hilbert transform 15 taps, 12 bits 
		 * (see Iowa Hills calculator)
		 */	
		q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + 
		         (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[ 8])*2202L;
		qh = q_accu / 4096L;	
		a_sample = i_s[7] - qh;
		break;
	case MODE_LSB:
		/* 
		 * LSB demodulate: I[7] + Qh,
		 * Qh is Classic Hilbert transform 15 taps, 12 bits 
		 * (see Iowa Hills calculator)
		 */	
		q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + 
		         (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[ 8])*2202L;
		qh = q_accu / 4096L;	
		a_sample = i_s[7] + qh;
		break;
	case MODE_AM:
		/*
		 * AM demodulate: sqrt(sqr(i)+sqr(q))
		 * Approximated with mag(i,q)
		 */
		a_sample = mag(i_s[7], q_s[7]);
		break;
	default:
		break;
	}
	
	/*** AUDIO GENERATION ***/

	/*
	 * Scale and clip output,  
	 * Send to audio DAC output
	 */
	a_sample = (a_sample/32) + DAC_BIAS;									// -15dB and add bias level
	if (a_sample > DAC_RANGE)												// Clip to DAC range
		a_sample = DAC_RANGE;
	else if (a_sample<0)
		a_sample = 0;

	return true;
}


/** CORE1: TX branch **/
/*
 * Execute TX branch signal processing, 
 * max time to spend is <16us, i.e. rate is 62.5 kHz
 * The audio sampling has already been done in vox()
 */
volatile int16_t a_s_raw[15]; 												// Raw samples, minus DC bias
volatile int16_t a_s[15];													// Filtered and decimated samplesvolatile int16_t a_dc;														// DC level
bool __not_in_flash_func(tx)(void) 
{
	int32_t a_accu, q_accu;
	int16_t qh;
	int i;
	uint16_t i_dac, q_dac;
		
	/*** RAW Audio ***/
	for (i=0; i<14; i++) 													//   and store in shift register
		a_s_raw[i] = a_s_raw[i+1];
	a_s_raw[14] = a_sample;
	
	/*** Low pass filter ***/
	a_accu = 0;																// Initialize accumulator
	for (i=0; i<15; i++)													// Low pass FIR filter, using raw samples
		a_accu += (int32_t)a_s_raw[i]*lpf3_15[i];							//   Fc=3kHz, at 15.625 kHz sampling		
		
	for (i=0; i<14; i++) 													// Shift decimated samples
		a_s[i] = a_s[i+1];
	a_s[14] = a_accu / 256;													// Store rescaled accumulator

	/*** MODULATION ***/
	switch (dsp_mode)
	{
	case 0:																	// USB
		/* 
		 * qh is Classic Hilbert transform 15 taps, 12 bits 
		 * (see Iowa Hills calculator)
		 */	
		q_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + 
		         (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
		qh = -(q_accu / 4096L);												// USB: sign is negative
		break;
	case 1:																	// LSB
		/* 
		 * qh is Classic Hilbert transform 15 taps, 12 bits 
		 * (see Iowa Hills calculator)
		 */	
		q_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + 
		         (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
		qh = q_accu / 4096L;												// LSB: sign is positive
		break;
	case 2:																	// AM
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
		q_sample = 0;
	else if (a_accu>(DAC_RANGE-1))
		q_sample = DAC_RANGE-1;
	else
		q_sample = a_accu;
	
	a_accu = DAC_BIAS + (a_s[7]/8);
	if (a_accu<0)
		i_sample = 0;
	else if (a_accu>(DAC_RANGE-1))
		i_sample = DAC_RANGE-1;
	else
		i_sample = a_accu;
		
	
	return true;
}



