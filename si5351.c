/*
 * si5351.c
 *
 * Created: Jan 2020
 * Author: Arjan
 *
 * Driver for the SI5351A VCO
 *
 
Si5351 principle of operation:
==============================
Crystal frequency Fxtal (usually 25MHz) is multiplied in a PLL by MSN to obtain Fvco.
PLLs A and B have independent MSN values, leading to two Fvco frequencies.
The output Fout on each channel (i) can be derived from either Fvco.
Fvco must be between 600MHz and 900MHz, but the spec is more relaxed in reality.
In the second stage, an Fvco is divided by MSi and Ri to obtain the output frequency Fout.
Only certain values of MSi and Ri are allowed when quadrature output is required.

Tuning stragtegy:
MSi and Ri are selected to be in the ballpark of desired frequency range.
MSN is then used for tuning Fout.

Details:
========

             +-------+             +-------+     +------+
   Fxtal --> | * MSN | -- Fvco --> | / MSi | --> | / Ri | -- Fout -->
             +-------+             +-------+     +------+

 MSN determines:          Fvco = Fxtal * (MSN)      , where MSN = a + b/c
 MSi and Ri determine:    Fout = Fvco / (Ri*MSi)    , where MSi = a + b/c	(note: different a, b and c)
 
 
 ---Derivation of the register values, that determine MSN and MSi---
 P1 = 128*a + Floor(128*b/c) - 512		(P1 = calculated for MSN tuning,  P1 = 750MHz/Fout for MSi integer mode)
 P2 = 128*b - c*Floor(128*b/c)			(P2 = calculated for MSN tuning,  P2 = 0 for MSi integer mode)
 P3 = c									(P3 = c = 1000000 for MSN tuning, P3 = 1 for MSi integer mode)
 
 
 This VFO implementation assumes PLLA is used for VFO 0 (clk0 and clk1), and PLLB is used for VFO 1 (clk2)
 
 The algorithm to get from required Fout to synthesizer settings:
 | calculate new <MSN> from the desired <Fout>, based on the current <Ri> and <MSi>
 | if MSN is still inside [600/Fxtal, 900/Fxtal]
 | then
 |   just update the MSN related registers
 | else 
 |   re-calculate <MSi>, <Ri> and <MSN> from desired <Fout>
 |   write the <MSi> and <Ri> parameter registers, including phase offset (MSi equals phase offset for 90 deg, use INV to shift 180deg more)
 |   write the <MSN> parameter registers
 |   reset PLL
 (this all assumes that the current settings are consistent, i.e. must be initialized at startup)

Calculate MSi:
	MSi = 750MHz/(Fout*Ri)	// Target for mid-band, i.e. Fvco=750MHz
	MSi &= 0xfe				// Make it even, minimum is 4 in case of integer mode

Calculate MSN:
	MSN = MSi*Ri*Fout/Fxtal (spec mandates between 24 and 36, but could be stretched)
	
	
Some boundary values, assuming 600M < Fvco < 900M.
With low end 400M, Low MHz is multiplied with 2/3.
------------+--------------+-----------------------+
Ri		MSi	:	a	b	c  |    Low MHz	  High MHz
1		  4	:	4	0	1  | 150.000000	225.000000	
1		126	: 126	0	1  |   4.761905	  7.142857
32		  4	:	4	0	1  |   4.687500	  7.031250
32		126	:  126	0	1  |   0.148810	  0.223214
128		  4	:	4	0	1  |   1.171875	  1.757813
128		126	:  126	0	1  |   0.037202	  0.055804

NOTE: Phase offsets
The PHOFF is given as a multiple of 1/(4*Fvco), so when MSi == PHOFF the shift will be 90deg.
It also implies that MSi must be an integer, and Ri == 1.

Only use MSi even-integers, i.e. a=[4, 6, 8..126], b=0 and c=100000, and set INT bits in reg 22, 23. 
Quadrature Phase offsets (i.e. delay): 
- Phase offset for MS0 (reg 165) must be 0 (cos: delay = 0), 
- Phase offset for MS1 (reg 166) must be equal to divider MS1 for 90 deg (sin: delay = MSi / 4*Fvco),
- Set INV bit (reg 17) to add 180 deg.

This implies that minimum Fout is 4.762MHz at Fvco = 600MHz. 
Additional flip/flop dividers are needed to get down to 80m band frequencies, or Fvco must be tuned below spec.

 
Control Si5351 (see AN619):
===========================
----+---------+---------+---------+---------+---------+---------+---------+---------+
 @	|    7    |    6    |    5    |    4    |    3    |    2    |    1    |    0    |
----+---------+---------+---------+---------+---------+---------+---------+---------+
  0	|SYS_INIT |  LOL_B  |  LOL_A  |   LOS   |      Reserved     |      REVID[1:0]   |
  1	|SYS_INIT |  LOS_B  |  LOL_A  |   LOS   |                Reserved               |
	|    _STKY|    _STKY|    _STKY|    _STKY|                Reserved               |
  2	|SYS_INIT |  LOS_B  |  LOL_A  |   LOS   |                Reserved               |
	|    _MASK|    _MASK|    _MASK|    _MASK|                Reserved               |
  3	|                   Reserved                      | CLK2_EN | CLK1_EN | CLK0_EN |
====
  9	|                   Reserved                      |OEB_MASK2|OEB_MASK1|OEB_MASK0|
====
 15 |  CLKIN_DIV[2:0]   |    0    |    0    | PLLB_SRC| PLLA_SRC|    0    |    0    |
 16 | CLK0_PDN| MS0_INT | MS0_SRC | CLK0_INV|   CLK0_SRC[1:0]   |   CLK0_IDRV[1:0]  |
 17 | CLK1_PDN| MS1_INT | MS1_SRC | CLK1_INV|   CLK1_SRC[1:0]   |   CLK1_IDRV[1:0]  |
 18 | CLK2_PDN| MS2_INT | MS2_SRC | CLK2_INV|   CLK2_SRC[1:0]   |   CLK2_IDRV[1:0]  |
====
 22 | Reserved| FBA_INT |                          Reserved                         |
 23 | Reserved| FBB_INT |                          Reserved                         |
 24 |      Reserved     |   CLK2_DIS_STATE  |   CLK1_DIS_STATE  |   CLK0_DIS_STATE  |
====
 26 |                                  MSNA_P3[15:8]                                |
 27 |                                  MSNA_P3[7:0]                                 |
 28	|                         Reserved                          |   MSNA_P1[17:16]  |
 29 |                                  MSNA_P1[15:8]                                |
 30 |                                  MSNA_P1[7:0]                                 |
 31 |             MSNA_P3[19:16]            |             MSNA_P2[19:16]            |
 32 |                                  MSNA_P2[15:8]                                |
 33 |                                  MSNA_P2[7:0]                                 |
 
 34: Same pattern for PLLB

 42 |                                  MS0_P3[15:8]                                 |
 43 |                                  MS0_P3[7:0]                                  |
 44 | Reserved|      R0_DIV[2:0]            |  MS0_DIVBY4[1:0]  |   MS0_P1[17:16]   |
 45 |                                  MS0_P1[15:8]                                 |
 46 |                                  MS0_P1[7:0]                                  |
 47 |              MS0_P3[19:16]            |              MS0_P2[19:16]            |
 48 |                                  MS0_P2[15:8]                                 |
 49 |                                  MS0_P2[7:0]                                  |

 50: Same pattern for CLK1
 
 58: Same pattern for CLK2

====
165	| Reserved|                          CLK0_PHOFF[6:0]                            |
166	| Reserved|                          CLK1_PHOFF[6:0]                            |
167	| Reserved|                          CLK2_PHOFF[6:0]                            |
====
177	| PLLB_RST| Reserved| PLLA_RST|                     Reserved                    |
====
183	|       XTAL_CL     |                         Reserved                          |
====
 *
 */ 

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "si5351.h"

#define I2C_VFO		0x60													// I2C address

// SI5351 register address definitions
#define SI_CLK_OE		3     
#define SI_CLK0_CTL		16
#define SI_CLK1_CTL		17
#define SI_CLK2_CTL		18
#define SI_SYNTH_PLLA	26
#define SI_SYNTH_PLLB	34
#define SI_SYNTH_MS0	42
#define SI_SYNTH_MS1	50
#define SI_SYNTH_MS2	58
#define SI_SS_EN		149
#define SI_CLK0_PHOFF	165
#define SI_CLK1_PHOFF	166
#define SI_CLK2_PHOFF	167
#define SI_PLL_RESET	177
#define SI_XTAL_LOAD	183

// CLK_OE register 3 masks
#define SI_CLK0_ENABLE	0b00000001											// Enable clock 0 output
#define SI_CLK1_ENABLE	0b00000010											// Enable clock 1 output
#define SI_CLK2_ENABLE	0b00000100											// Enable clock 2 output
#define SI_VFO0_DISABLE	0b00000011											// Set bits to disable
#define SI_VFO1_DISABLE	0b00000100											// Set bits to disable

// CLKi_CTL register 16, 17, 18 values
// Normally 0x4f for clk 0 and 1, 0x6f for clk 2
#define SI_CLK_INT		0b01000000  										// Set integer mode 
#define SI_CLK_PLL 		0b00100000											// Select PLL B as MS source (default 0 = PLL A)
#define SI_CLK_INV		0b00010000											// Invert output (i.e. phase + 180deg)
#define SI_CLK_SRC		0b00001100											// Select output source: 11=MS, 00=XTAL direct
#define SI_CLK_DRV		0b00000011											// Select output drive, increasingly: 2-4-6-8 mA 
																			// Play with these to get a nice block output

// PLL_RESET register 177 values
#define SI_PLLB_RST		0b10000000											// Reset PLL B
#define SI_PLLA_RST		0b00100000											// Reset PLL A



#define SI_XTAL_FREQ	25001414UL											// Replace with measured crystal frequency of XTAL for CL = 10pF (default)
#define SI_MSN_LO		((0.4e9)/SI_XTAL_FREQ)								// Should be 600M, but 400MHz works too
#define SI_MSN_HI		((0.9e9)/SI_XTAL_FREQ)
#define SI_PLL_C		1000000UL											// Parameter c for PLL-A and -B setting


typedef struct
{
	uint32_t freq;		// type can hold up to 4GHz
	uint8_t  flag;		// flag != 0 when update needed
	uint8_t  phase;		// in quarter waves (0, 1, 2, 3)
	uint8_t  ri;		// Ri (1 .. 128), but should be 1 for VFO 0
	uint8_t  msi;		// MSi parameter a (4, 6, 8 .. 126)
	double   msn;		// MSN (24.0 .. 35.9999)
} vfo_t;
vfo_t vfo[2];																// 0: clk0 / clk1     1: clk2


void si_setfreq(int i, uint32_t f)
{
	if ((i<0)||(i>1)) return;												// Check VFO range
	if (f>150000000) return;												// Check frequency range
	if (vfo[i].freq == f) return;											// Anything to set at all?
	
	vfo[i].freq = f; 														// Entry checks pass, so do the actual setting
	vfo[i].flag = 1;
}

void si_setphase(int i, uint8_t p)
{
	if (i!=0) return;														// Check VFO range
	if (p>3) return;														// Check phase range
	if (vfo[i].phase == p) return;											// Anything to set at all?
	
	vfo[i].phase = p; 														// Entry checks pass, so do the actual setting
	vfo[i].flag = 1;
}

void si_enable(int i, bool en)
{
	uint8_t data[2];
	
	if ((i<0)||(i>1)) return;												// Check VFO range
	
	data[0] = SI_CLK_OE;													// Read OE register
	i2c_write_blocking(i2c0, I2C_VFO, &data[0], 1, true);
	i2c_read_blocking(i2c0, I2C_VFO, &data[1], 1, false);

	data[0] = SI_CLK_OE;
	if (i==0)
	{
		data[1] = en ? data[1]&~SI_VFO0_DISABLE : data[1]|SI_VFO0_DISABLE;	// clk0 and clk1
	}
	else
	{
		data[1] = en ? data[1]&~SI_VFO1_DISABLE : data[1]|SI_VFO1_DISABLE;	// clk2
	}
	i2c_write_blocking(i2c0, I2C_VFO, &data[0], 2, false);
}

/* 
 * read contents of SI5351 registers, from reg to reg+len-1, output in data array 
 */
int si_getreg(uint8_t *data, uint8_t reg, uint8_t len)
{
	int ret;
	
	ret = i2c_write_blocking(i2c0, I2C_VFO, &reg, 1, true);
	if (ret<0) printf ("I2C write error\n");
	ret = i2c_read_blocking(i2c0, I2C_VFO, data, len, false);
	if (ret<0) printf ("I2C read error\n");
	return(len);
}


/*
 * Set up MSN PLL divider for vfo[i], assuming MSN has been set in vfo[i]
 * Optimize for speed, this may be called with short intervals
 * See also SiLabs AN619 section 3.2
 * VFO 0 refers to PLL a, VFO 1 refers to PLL B

 MSN = a + b/c
 c = 1000000							(Fxtal/c step size)
 P1 = 128*a + Floor(128*b/c) - 512
 P2 = 128*b - c*Floor(128*b/c)
 P3 = c

 */
void si_setmsn(int i)
{
	uint8_t  data[16];														// I2C trx buffer
	uint32_t P1, P2;														// MSN parameters, P3 is SI_PLL_C
	uint32_t A, B;

	if ((i<0)||(i>1)) return;												// Check VFO range
	
	A  = (uint32_t)(floor(vfo[i].msn));										// A is integer part of MSN
	B  = (uint32_t)((vfo[i].msn - (double)A) * SI_PLL_C);					// B is C * fraction part of MSN (C is a constant)
	P2 = (uint32_t)(floor((double)(128 * B) / (double)SI_PLL_C));			// use P2 for intermediate result..
	P1 = (uint32_t)(128 * A + P2 - 512);
	P2 = (uint32_t)(128 * B - SI_PLL_C * P2);
	
	// transfer PLL A or PLL B registers
	if (i==0)
		data[0] = SI_SYNTH_PLLA;
	else
		data[0] = SI_SYNTH_PLLB;
	data[1] = (SI_PLL_C & 0x0000FF00) >> 8;
	data[2] = (SI_PLL_C & 0x000000FF);
	data[3] = (P1 & 0x00030000) >> 16;
	data[4] = (P1 & 0x0000FF00) >> 8;
	data[5] = (P1 & 0x000000FF);
	data[6] = ((SI_PLL_C & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
	data[7] = (P2 & 0x0000FF00) >> 8;
	data[8] = (P2 & 0x000000FF);
	i2c_write_blocking(i2c0, I2C_VFO, data, 9, false);
}

/*
 * Set up registers with MS and R divider for vfo[i], assuming values have been set in vfo[i]
 * In this implementation we only use integer mode, i.e. b=0 and P3=1

 MSi = a + b/c
 c = 1, b=0
 P1 = 128*a - 512
 P2 = 0
 P3 = c

 */
void si_setmsi(uint8_t i)
{
	uint8_t data[16];														// I2C trx buffer
	uint32_t P1;
	uint8_t  R;

	if ((i<0)||(i>1)) return;												// Check VFO range
	
	P1 = (uint32_t)(128*(uint32_t)floor(vfo[i].msi) - 512);
	R  = vfo[i].ri;
	R  = (R&0xf0) ? ((R&0xc0)?((R&0x80)?7:6):(R&0x20)?5:4) : ((R&0x0c)?((R&0x08)?3:2):(R&0x02)?1:0); // quick log2(r)
	
	if (i==0)
		data[0] = SI_SYNTH_MS0;
	else
		data[0] = SI_SYNTH_MS2;
	data[1] = 0x00;
	data[2] = 0x01;
	data[3] = ((P1 & 0x00030000) >> 16) | (R << 4 );
	data[4] = (P1 & 0x0000FF00) >> 8;
	data[5] = (P1 & 0x000000FF);
	data[6] = 0x00;
	data[7] = 0x00;
	data[8] = 0x00;
	i2c_write_blocking(i2c0, I2C_VFO, data, 9, false);

	// If vfo[0] also set clk 1	and phase offset, (integer mode and high drive current for low phase noise).
	if (i==0)
	{
		data[0] = SI_SYNTH_MS1;												// Same data in synthesizer
		i2c_write_blocking(i2c0, I2C_VFO, data, 9, false);
		
		if (vfo[0].phase&(PH090|PH270))										// Phase is 90 or 270 deg?
		{
			data[0] = SI_CLK1_PHOFF;
			data[1] = vfo[0].msi;											// offset == MSi for 90deg
			i2c_write_blocking(i2c0, I2C_VFO, data, 2, false);
		}
		else																// Phase is 0 or 180 deg
		{
			data[0] = SI_CLK1_PHOFF;
			data[1] = 0;													// offset == 0 for 0deg
			i2c_write_blocking(i2c0, I2C_VFO, data, 2, false);
		}
		if (vfo[0].phase&(PH180|PH270))										// Phase is 180 or 270 deg?
		{
			data[0] = SI_CLK1_CTL;											// set the invert flag
			data[1] = 0x1d;													// CLK1: nonINT, PLLA, INV, MS, 4mA
			i2c_write_blocking(i2c0, I2C_VFO, data, 2, false);
		}
		else
		{
			data[0] = SI_CLK1_CTL;											// clear the invert flag
			data[1] = 0x0d;													// CLK1: nonINT, PLLA, nonINV, MS, 4mA
			i2c_write_blocking(i2c0, I2C_VFO, data, 2, false);
		}
	}
	else
	{
		data[0] = SI_CLK2_CTL;												// set the invert flag
		data[1] = 0x2d;														// CLK2: nonINT, PLLB, nonINV, MS, 4mA
		i2c_write_blocking(i2c0, I2C_VFO, data, 2, false);
	}
		
	// Reset associated PLL
	data[0] = SI_PLL_RESET;
	data[1] = (i==1)?0x80:0x20;
	i2c_write_blocking(i2c0, I2C_VFO, data, 2, false);
}


/*
 * This function needs to be invoked at regular intervals, e.g. 10x per sec. See hmi.c
 * For each vfo, calculate required MSN setting, MSN = MSi*Ri*Fout/Fxtal
 * If still in range, 
 *	then just set MSN registers
 * else, 
 *	recalculate MSi and Ri as well
 *	set MSN, MSi and Ri registers (implicitly resets PLL)
 */
void si_evaluate(void)
{
	double msn;

	if (vfo[0].flag)
	{
		msn = (double)(vfo[0].msi); 										// Re-calculate MSN
		msn = msn * (double)(vfo[0].ri);
		msn = msn * (double)(vfo[0].freq) / SI_XTAL_FREQ;
		
		if ((msn>=SI_MSN_LO)&&(msn<SI_MSN_HI))								// Check MSN range
		{
			vfo[0].msn = msn;
			si_setmsn(0);
		}
		else
		{
			// Pre-scale Ri, stretch down Ri=1 range to 3MHz
			// Otherwise use just 32 and 128
			if (vfo[0].freq>3000000)
				vfo[0].ri  = 1;
			else if (vfo[0].freq>1000000)
				vfo[0].ri  = 32;
			else
				vfo[0].ri  = 128;
			
			// Set MSi
			if ((vfo[0].freq >= 3000000)&&(vfo[0].freq < 6000000))			// Handle Low end of Ri=1 range
				vfo[0].msi = (uint8_t)126;									// Maximum MSi on Fvco=(4x126)MHz
			else															// Or calculate MSi on Fvco=750MHz
				vfo[0].msi = (uint8_t)(750000000UL / (vfo[0].freq * vfo[0].ri)) & 0x000000fe;

			msn = (double)(vfo[0].msi); 									// Re-calculate MSN
			msn = msn * (double)(vfo[0].ri);
			msn = msn * (double)(vfo[0].freq) / SI_XTAL_FREQ;
			vfo[0].msn = msn;
			
			si_setmsn(0);
			si_setmsi(0);
		}
		vfo[0].flag = 0;
	}
	if (vfo[1].flag)
	{
		msn = (double)(vfo[1].msi); 										// Re-calculate MSN
		msn = msn * (double)(vfo[1].ri);
		msn = msn * (double)(vfo[1].freq) / SI_XTAL_FREQ;
		
		if ((msn>=SI_MSN_LO)&&(msn<SI_MSN_HI))								// Check MSN range
		{
			vfo[1].msn = msn;
			si_setmsn(1);
		}
		else
		{
			// Pre-scale Ri, stretch down Ri=1 range to 3MHz
			// Otherwise use just 32 and 128
			if (vfo[1].freq>3000000)
				vfo[1].ri  = 1;
			else if (vfo[1].freq>1000000)
				vfo[1].ri  = 32;
			else
				vfo[1].ri  = 128;
			
			// Set MSi
			if ((vfo[1].freq >= 3000000)&&(vfo[1].freq < 6000000))			// Handle Low end of Ri=1 range
				vfo[1].msi = (uint8_t)126;									// Maximum MSi on Fvco=(4x126)MHz
			else															// Or calculate MSi on Fvco=750MHz
				vfo[1].msi = (uint8_t)(750000000UL / (vfo[1].freq * vfo[1].ri)) & 0x000000fe;

			msn = (double)(vfo[1].msi); 									// Re-calculate MSN
			msn = msn * (double)(vfo[1].ri);
			msn = msn * (double)(vfo[1].freq) / SI_XTAL_FREQ;
			vfo[1].msn = msn;
			
			si_setmsn(1);
			si_setmsi(1);
		}
		vfo[1].flag = 0;
	}
}


/*
 * Initialize the Si5351 VFO registers
 */
void si_init(void)
{
	uint8_t data[16];														// I2C trx buffer

	// Disable spread spectrum (startup state is undefined)	
	data[0] = SI_SS_EN;
	data[1] = 0x00;
	i2c_write_blocking(i2c0, I2C_VFO, data, 2, false);
	
	// First time init of clock control registers
	data[0] = SI_CLK0_CTL;
	data[1] = 0x0d;															// CLK0: nonINT, PLLA, nonINV, MS, 4mA
	data[2] = 0x0d;															// CLK1: nonINT, PLLA, nonINV, MS, 4mA
	data[3] = 0x2d;															// CLK2: nonINT, PLLB, nonINV, MS, 4mA
	i2c_write_blocking(i2c0, I2C_VFO, data, 4, false);
			
	// Initialize VFO values
	vfo[0].freq  = 7074000;
	vfo[0].flag  = 0;
	vfo[0].phase = PH090;
	vfo[0].ri    = 1;
	vfo[0].msi   = 106;
	vfo[0].msn   = ((double)vfo[0].freq*vfo[0].msi)/(double)SI_XTAL_FREQ;
	
	vfo[1].freq  = 10000000;
	vfo[1].flag  = 0;
	vfo[1].phase = PH000;
	vfo[1].ri    = 1;
	vfo[1].msi   = 76;
	vfo[1].msn   = ((double)vfo[1].freq*vfo[1].msi)/(double)SI_XTAL_FREQ;
	
	// Commit settings
	si_setmsn(0);
	si_setmsi(0);	
	si_setmsn(1);
	si_setmsi(1);	

	// Enable only VFO 0 outputs	
	si_enable(0, true);
	si_enable(1, false);
}

