/*
 * dsp_fft.c
 * ==>TO BE INCLUDED IN dsp.c
 *
 * Created: May 2022
 * Author: Arjan te Marvelde
 * 
 * Signal processing of RX and TX branch, to be run on the second processor core (CORE1).
 * A branch has a dedicated routine that must run on set times.
 * In this case it runs when half FFT_SIZE of samples is ready to be processed.
 *
 *
 * The pace for sampling is set by a timer at 64usec (15.625 kHz)
 * The associated timer callback routine:
 * - handles data transfer to/from physical interfaces
 * - starts a new ADC conversion sequence 
 * - maintains dsp_tick counter
 * - when dsp_tick == FFT_SIZE/2 (one buffer), the dsp-loop is triggered.
 *
 * The ADC functions in round-robin and fifo mode, triggering IRQ after 3 conversions (ADC[0..2])
 * The ADC FIFO IRQ handler reads the 3 samples from the fifo after stopping the ADC
 *
 * Buffer structure, built from half FFT_SIZE buffers.
 * The I, Q and A external interfaces communicate each through 3x buffers.
 * One buffer is being filled or emptied, depending on data direction.
 * The other two are swapped with the FFT signal processing buffers.
 * Since we use complex FFT, the algorithm uses 4x buffers.
 *
 * I, Q and A buffers used as queues. RX case looks like:
 *
 *        +--+--+--+
 *  i --> |  |  |  |
 *        +--+--+--+
 *            \  \  \     +--+--+
 *              --------> |  |  |                      +--+--+--+
 *                        +--+--+    FFT-DSP-iFFT  --> |  |  |  | --> a
 *              --------> |  |  |                      +--+--+--+
 *            /  /  /     +--+--+
 *        +--+--+--+
 *  q --> |  |  |  |
 *        +--+--+--+
 *
 * RX, when triggered by timer callback:
 * - The oldest real FFT buffer is moved to the output queue (check this)
 * - The oldest two I and Q buffers are copied into the FFT buffers
 * - FFT is executed
 * - Signal processing is done
 * - iFFT is executed
 *
 * The bin step is the sampling frequency divided by the FFT_SIZE.
 * So for S_RATE=15625 and FFT_SIZE=1024 this step is 15625/1024=15.259 Hz
 * The Carrier offset (Fc) is at about half the Nyquist frequency: bin 256 or 3906 Hz
 *
 */



/*
 * FFT buffer allocation
 * Buffer size is FFT_SIZE/2 (see fix_fft.h).
 * In case FFT_SIZE of 1024, a buffer is 1kB
 *  RX:  3 buffers for I samples, 3 buffers for Q samples, 3 buffers for Audio
 *  DSP: 4 buffers for FFT, complex samples and these have to be consecutive!
 *  TX:  re-use RX buffers in reverse order
 * Total of 13kByte RAM is required.
 * Samples are 16 bit signed integer, but align buffers on 32bit boundaries
 * dsp_tick points into I, Q and A buffers, so wrap once per two FFTs
 * When tick==FFT_SIZE/2: do buffer copy
 */ 
#define BUFSIZE		FFT_SIZE/2
int16_t  I_buf[3][BUFSIZE] __attribute__((aligned(4)));						// I sample queue, 3x buffer of FFT_SIZE/2
int16_t  Q_buf[3][BUFSIZE] __attribute__((aligned(4)));						// Q sample queue, 3x buffer of FFT_SIZE/2
int16_t  A_buf[3][BUFSIZE] __attribute__((aligned(4)));						// A sample queue, 3x buffer of FFT_SIZE/2
int16_t XI_buf[FFT_SIZE] __attribute__((aligned(4)));						// Re FFT buffer, 1x buffer of FFT_SIZE
int16_t XQ_buf[FFT_SIZE] __attribute__((aligned(4)));						// Im FFT buffer, 1x buffer of FFT_SIZE

// Sample buffer indexes, updated by timer callback
volatile int      dsp_active = 0;											// I, Q, A active buffer number (0..2)
volatile uint32_t dsp_tick   = 0;											// Index in active buffer
volatile uint32_t dsp_tickx  = 0;											// Load indicator DSP loop

// Spectrum bins for a frequency
#define BIN(f)			(int)(((f)*FFT_SIZE+S_RATE/2)/S_RATE)
#define BIN_FC			  256
#define BIN_100       	    7
#define BIN_300		 	   20
#define BIN_900		 	   59
#define BIN_3000		  197

/*
 * This applies a bandpass filter to XI and XQ buffers
 * lowbin and highbin edges must be between 3 and FFT_SIZE/2 - 3
 * Edge is a 7 bin raised cosine flank, i.e. 100Hz wide
 * Coefficients are: 0, 0.067, 0.25, 0.5, 0.75, 0.933, 1
 * where the edge bin is in the center of this flank
 * Note: maybe make slope less steep, e.g. 9 or 11 bins
 */
inline void dsp_bandpass(int lowbin, int highbin)
{
	int i;
	
	if ((lowbin<3)||(highbin>(FFT_SIZE/2-3))||(highbin-lowbin<6)) return;
	
	XI_buf[0] = 0; XQ_buf[0] = 0; 	
	for (i=1; i<lowbin-2; i++) 
	{ 
		XI_buf[i] = 0; XI_buf[FFT_SIZE-i] = 0;
		XQ_buf[i] = 0; XQ_buf[FFT_SIZE-i] = 0; 
	}
	for (i=highbin+3; i<FFT_SIZE-highbin-2; i++) 
	{ 
		XI_buf[i] = 0; 
		XQ_buf[i] = 0; 
	}

	// Note: There is not much difference between using or discarding Q bins 
	i=lowbin-2;
	XI_buf[i] = XI_buf[i]*0.067; XQ_buf[i] = XQ_buf[i]*0.067; i++;
	XI_buf[i] = XI_buf[i]*0.250; XQ_buf[i] = XQ_buf[i]*0.250; i++;
	XI_buf[i] = XI_buf[i]*0.500; XQ_buf[i] = XQ_buf[i]*0.500; i++;
	XI_buf[i] = XI_buf[i]*0.750; XQ_buf[i] = XQ_buf[i]*0.750; i++;
	XI_buf[i] = XI_buf[i]*0.933; XQ_buf[i] = XQ_buf[i]*0.933; 
	i=highbin-2;
	XI_buf[i] = XI_buf[i]*0.933; XQ_buf[i] = XQ_buf[i]*0.933; i++;
	XI_buf[i] = XI_buf[i]*0.250; XQ_buf[i] = XQ_buf[i]*0.250; i++;
	XI_buf[i] = XI_buf[i]*0.500; XQ_buf[i] = XQ_buf[i]*0.500; i++;
	XI_buf[i] = XI_buf[i]*0.750; XQ_buf[i] = XQ_buf[i]*0.750; i++;
	XI_buf[i] = XI_buf[i]*0.067; XQ_buf[i] = XQ_buf[i]*0.067; 
	i=FFT_SIZE-highbin-2;
	XI_buf[i] = XI_buf[i]*0.067; XQ_buf[i] = XQ_buf[i]*0.067; i++;
	XI_buf[i] = XI_buf[i]*0.250; XQ_buf[i] = XQ_buf[i]*0.250; i++;
	XI_buf[i] = XI_buf[i]*0.500; XQ_buf[i] = XQ_buf[i]*0.500; i++;
	XI_buf[i] = XI_buf[i]*0.750; XQ_buf[i] = XQ_buf[i]*0.750; i++;
	XI_buf[i] = XI_buf[i]*0.933; XQ_buf[i] = XQ_buf[i]*0.933; 
	i=FFT_SIZE-lowbin-2;
	XI_buf[i] = XI_buf[i]*0.933; XQ_buf[i] = XQ_buf[i]*0.933; i++;
	XI_buf[i] = XI_buf[i]*0.250; XQ_buf[i] = XQ_buf[i]*0.250; i++;
	XI_buf[i] = XI_buf[i]*0.500; XQ_buf[i] = XQ_buf[i]*0.500; i++;
	XI_buf[i] = XI_buf[i]*0.750; XQ_buf[i] = XQ_buf[i]*0.750; i++;
	XI_buf[i] = XI_buf[i]*0.067; XQ_buf[i] = XQ_buf[i]*0.067; 
}



/** CORE1: RX branch **/
/*
 * Execute RX branch signal processing
 */
volatile int scale0;
volatile int scale1; 
bool __not_in_flash_func(rx)(void) 
{
	int b;
	int i;
	int16_t *ip, *qp, *ap, *xip, *xqp;
	int16_t peak;
		
	b = dsp_active;															// Point to Active sample buffer
	
	/*** Copy saved I/Q buffers to FFT buffer ***/
	if (++b > 2) b = 0;														// Point to Old Saved sample buffer
	ip = &I_buf[b][0]; xip = &XI_buf[0];
	qp = &Q_buf[b][0]; xqp = &XQ_buf[0];
	for (i=0; i<BUFSIZE; i++)
	{
		*xip++ = *ip++;
		*xqp++ = *qp++;
	}
	if (++b > 2) b = 0;														// Point to New Saved sample buffer
	ip = &I_buf[b][0]; xip = &XI_buf[BUFSIZE];
	qp = &Q_buf[b][0]; xqp = &XQ_buf[BUFSIZE];
	for (i=0; i<BUFSIZE; i++)
	{
		*xip++ = *ip++;
		*xqp++ = *qp++;
	}

	
	/*** Execute FFT ***/
	scale0 = fix_fft(&XI_buf[0], &XQ_buf[0], false);	
	
	
	/*** Shift and filter sidebands ***/
	XI_buf[0] = 0;
	XQ_buf[0] = 0;
	switch (dsp_mode)
	{
	case MODE_USB:
		// Shift Fc to 0Hz
		for (i=1; i<BIN_3000; i++)
		{
			XI_buf[i]          = XI_buf[i+BIN_FC]; 
			XI_buf[FFT_SIZE-i] = XI_buf[FFT_SIZE-BIN_FC-i];
			XQ_buf[i]          = XQ_buf[i+BIN_FC]; 
			XQ_buf[FFT_SIZE-i] = XQ_buf[FFT_SIZE-BIN_FC-i];
		}
		// Bandpass DSB (2x USB)
		dsp_bandpass(BIN_100, BIN_3000);
		break;
	case MODE_LSB:
		// Shift Fc to 0Hz, i.e. swap buffers
		for (i=1; i<BIN_3000; i++)
		{
			XI_buf[BUFSIZE-i]  = XI_buf[BIN_FC-i]; 
			XI_buf[i]          = XI_buf[FFT_SIZE-BIN_FC+i];
			XI_buf[FFT_SIZE-i] = XI_buf[BUFSIZE-i];
			XQ_buf[BUFSIZE-i]  = XQ_buf[BIN_FC-i]; 
			XQ_buf[i]          = XQ_buf[FFT_SIZE-BIN_FC+i];
			XQ_buf[FFT_SIZE-i] = XQ_buf[BUFSIZE-i];
		}
		// Bandpass DSB (2x LSB)
		dsp_bandpass(BIN_100, BIN_3000);
		break;
	case MODE_AM:
		// Shift the rest to the right place
		for (i=1; i<BIN_3000; i++)
		{
			XI_buf[FFT_SIZE-i] = XI_buf[BIN_FC-i]; 
			XI_buf[i]          = XI_buf[BIN_FC+i];
			XQ_buf[FFT_SIZE-i] = XQ_buf[BIN_FC-i]; 
			XQ_buf[i]          = XQ_buf[BIN_FC+i];
		}
		// Bandpass DSB (LSB + USB)
		dsp_bandpass(BIN_100, BIN_3000);
		break;
	case MODE_CW:
		// Shift carrier from Fc to 900Hz 
		for (i=-BIN_900+1; i<BIN_900-1; i++) 
		{
			XI_buf[i+BIN_900]          = XI_buf[BIN_FC+i]; 
			XI_buf[FFT_SIZE-i-BIN_900] = XI_buf[FFT_SIZE-BIN_FC-i];
			XQ_buf[i+BIN_900]          = XQ_buf[BIN_FC+i]; 
			XQ_buf[FFT_SIZE-i-BIN_900] = XQ_buf[FFT_SIZE-BIN_FC-i];
		}
		// Bandpass CW
		dsp_bandpass(BIN_900-BIN_300, BIN_900+BIN_300);
		break;
	}

	
	/*** Execute inverse FFT ***/
	scale1 = fix_fft(&XI_buf[0], &XQ_buf[0], true);


	/*** Export FFT buffer to A ***/
	b = dsp_active;															// Assume active buffer not changed, i.e. no overruns
	if (++b > 2) b = 0;														// Point to oldest (will be next for output)
	ap = &A_buf[b][0]; xip = &XI_buf[BUFSIZE];
	for (i=0; i<BUFSIZE; i++)
	{
		*ap++ = *xip++;														// Copy newest results
	}


	/*** Scale down into DAC_RANGE! ***/	
	peak = 256;
	for (i=0; i<BUFSIZE; i++)									
	{
		A_buf[b][i] /= peak;
	}
		
	return true;
}


/** CORE1: TX branch **/
/*
 * Execute TX branch signal processing
 */
bool __not_in_flash_func(tx)(void) 
{
	// Export FFT buffers to I/Q
	
	// Import A buffers
	
	// FFT
	
	// Filter
	
	// iFFT
	
	return true;
}





