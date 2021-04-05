/*
 * si5351.c
 *
 * Created: Jan 2020
 * Author: Arjan
 
Si5351 principle:
=================
             +-------+             +-------+     +------+
 - Fxtal --> | * MSN | -- Fvco --> | / MSi | --> | / Ri | -- Fout -->
             +-------+             +-------+     +------+
 
 ---Derivation of Fout---
 MSN determines:          Fvco = Fxtal * (MSN)      , where MSN = a + b/c
 MSi and Ri determine:    Fout = Fvco / (Ri*MSi)    , where MSi = a + b/c	(different a, b and c)
 
 ---Derivation of register values, for MSN and MSi---
 P1 = 128*a + Floor(128*b/c) - 512
 P2 = 128*b - c*Floor(128*b/c)			(P2 = 0 for MSi integer mode, or calculated for MSN tuning)
 P3 = c									(P3 = 1 for MSi integer mode, or P3 = 1000000 for MSN tuning)
 
 This VFO implementation assumes PLLA is used for clk0 and clk1, PLLB is used for clk2
 
 Algorithm to get from frequency to synthesizer settings:
 (this assumes that the current settings are consistent, i.e. must be initialized at startup)
 | calculate new MSN from the desired Fout, based on current Ri and MSi
 | if MSN is still inside [600/Fxtal, 900/Fxtal]
 | then
 |   just write the MSN parameter registers
 | else 
 |   re-calculate MSi, Ri and MSN from desired Fout
 |   write the MSi and Ri parameter registers, including phase offset (MSi equals phase offset for 90 deg, use INV to shift 180deg more)
 |   write the MSN parameter registers
 |   reset PLL

Ri=128 for Fout   <1 MHz
Ri= 32 for Fout  1-6 MHz
Ri=  1 for Fout   >6 MHz

Some boundary values:
Ri		MSi		Lo MHz	    Hi MHz
1		  4		150.000000	225.000000
1		126		  4.761905	  7.142857
32		  4		  4.687500	  7.031250
32		126		  0.148810	  0.223214
128		  4		  1.171875	  1.757813
128		126		  0.037202	  0.055804

MSi: target for mid-band, i.e. Fvco=750MHz
     MSi = 750MHz/(Fout*Ri)
	 MSi &= 0xfe	// Make it even

MSN = MSi*Ri*Fout/Fxtal (should be between 24 and 36)

Only use MSi even-integers, i.e. d=[4, 6, 8..126], e=0 and f=100000, and set INT bits in reg 22, 23. 
Phase offset MS0 (reg 165) must be 0, MS1 (reg 166) must be equal to MS1 for 90 deg. Set INV bit in reg 17 to add 180 deg.
NOTE: Phase offsets only work when Ri = 1, this means minimum Fout is 4.762MHz at Fvco = 600MHz


 
Control Si5351:
================
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

 */ 

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "si5351.h"

#define I2C_VFO		0x60	// I2C address

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

// CLK_OE register 3 values
#define SI_CLK0_ENABLE	0b00000001	// Enable clock 0 output
#define SI_CLK1_ENABLE	0b00000010	// Enable clock 1 output
#define SI_CLK2_ENABLE	0b00000100	// Enable clock 2 output

// CLKi_CTL register 16, 17, 18 values
// Normally 0x4f for clk 0 and 1, 0x6f for clk 2
#define SI_CLK_INT		0b01000000  // Set integer mode 
#define SI_CLK_PLL 		0b00100000	// Select PLL B as MS source (default 0 = PLL A)
#define SI_CLK_INV		0b00010000	// Invert output (i.e. phase + 180deg)
#define SI_CLK_SRC		0b00001100	// Select output source: 11=MS, 00=XTAL direct
#define SI_CLK_DRV		0b00000011	// Select output drive, increasingly: 2-4-6-8 mA (best risetime, use max = 11)

// PLL_RESET register 177 values
#define SI_PLLB_RST		0b10000000	// Reset PLL B
#define SI_PLLA_RST		0b00100000	// Reset PLL A



#define SI_XTAL_FREQ	24998851UL	// Replace with measured crystal frequency of XTAL for CL = 10pF (default)
#define SI_MSN_LO		((0.6e9)/SI_XTAL_FREQ)
#define SI_MSN_HI		((0.9e9)/SI_XTAL_FREQ)
#define SI_PLL_C		1000000UL		// Parameter c for PLL-A and -B setting


/* I2C1 pins */
#define I2C1_SDA 10
#define I2C1_SCL 11

vfo_t vfo[2];				// 0: clk0 and clk1     1: clk2

/* read contents of SI5351 registers, from reg to reg+len-1, output in data array */
int si_getreg(uint8_t *data, uint8_t reg, uint8_t len)
{
	int ret;
	
	ret = i2c_write_blocking(i2c1, I2C_VFO, &reg, 1, true);
	if (ret<0) printf ("I2C write error\n");
	ret = i2c_read_blocking(i2c1, I2C_VFO, data, len, false);
	if (ret<0) printf ("I2C read error\n");
	return(len);
}


// Set up MSN PLL divider for vfo[i], assuming MSN has been set in vfo[i]
// Optimize for speed, this may be called with short intervals
// See also SiLabs AN619 section 3.2
void si_setmsn(uint8_t i)
{
	uint8_t  data[16];		// I2C trx buffer
	uint32_t P1, P2;		// MSN parameters
	uint32_t A;
	uint32_t B;

	i=(i>0?1:0);
/*
 P1 = 128*a + Floor(128*b/c) - 512
 P2 = 128*b - c*Floor(128*b/c)
 P3 = c									(P3 = 1000000 for MSN tuning)
*/	
	A  = (uint32_t)(floor(vfo[i].msn));						// A is integer part of MSN
	B  = (uint32_t)((vfo[i].msn - (float)A) * SI_PLL_C);	// B is C * fraction part of MSN (C is a constant)
	P2 = (uint32_t)(floor((float)(128 * B) / (float)SI_PLL_C));
	P1 = (uint32_t)(128 * A + P2 - 512);
	P2 = (uint32_t)(128 * B - SI_PLL_C * P2);
	
	// transfer registers
	data[0] = (i==0?SI_SYNTH_PLLA:SI_SYNTH_PLLB);
	data[1] = (SI_PLL_C & 0x0000FF00) >> 8;
	data[2] = (SI_PLL_C & 0x000000FF);
	data[3] = (P1 & 0x00030000) >> 16;
	data[4] = (P1 & 0x0000FF00) >> 8;
	data[5] = (P1 & 0x000000FF);
	data[6] = ((SI_PLL_C & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
	data[7] = (P2 & 0x0000FF00) >> 8;
	data[8] = (P2 & 0x000000FF);
	i2c_write_blocking(i2c1, I2C_VFO, data, 9, false);
}

// Set up registers with MS and R divider for vfo[i], assuming values have been set in vfo[i]
// In this implementation we only use integer mode, i.e. b=0 and P3=1
// See also SiLabs AN619 section 4.1
void si_setmsi(uint8_t i)
{
	uint8_t data[16];		// I2C trx buffer
	uint32_t P1;
	uint8_t  R;

	i=(i>0?1:0);
/*
 P1 = 128*a + Floor(128*b/c) - 512
 P2 = 128*b - c*Floor(128*b/c)			(P2 = 0 for MSi integer mode)
 P3 = c									(P3 = 1 for MSi integer mode)
*/	
	P1 = (uint32_t)(128*(uint32_t)floor(vfo[i].msi) - 512);
	R  = vfo[i].ri;
	R  = (R&0xf0) ? ((R&0xc0)?((R&0x80)?7:6):(R&0x20)?5:4) : ((R&0x0c)?((R&0x08)?3:2):(R&0x02)?1:0); // quick log2(r)
	
	data[0] = (i==0?SI_SYNTH_MS0:SI_SYNTH_MS2);
	data[1] = 0x00;
	data[2] = 0x01;
	data[3] = ((P1 & 0x00030000) >> 16) | (R << 4 );
	data[4] = (P1 & 0x0000FF00) >> 8;
	data[5] = (P1 & 0x000000FF);
	data[6] = 0x00;
	data[7] = 0x00;
	data[8] = 0x00;
	i2c_write_blocking(i2c1, I2C_VFO, data, 9, false);

	// If vfo[0] also set clk 1	
	if (i==0)
	{
		data[0] = SI_SYNTH_MS1;						// Same data in synthesizer
		i2c_write_blocking(i2c1, I2C_VFO, data, 9, false);
		
		if (vfo[0].phase&1)							// Phase is either 90 or 270 deg?
		{
			data[0] = SI_CLK1_PHOFF;
			data[1] = vfo[0].msi;
			i2c_write_blocking(i2c1, I2C_VFO, data, 2, false);
		}
		else
		{
			data[0] = SI_CLK1_PHOFF;
			data[1] = 0;
			i2c_write_blocking(i2c1, I2C_VFO, data, 2, false);
		}
		if (vfo[0].phase&2)							// Phase is 180 or 270 deg?
		{
			data[0] = SI_CLK1_CTL;
			data[1] = 0x5f;							// CLK1: INT, PLLA, INV, MS, 8mA
			i2c_write_blocking(i2c1, I2C_VFO, data, 2, false);
		}
	}
		
	// Reset associated PLL
	data[0] = SI_PLL_RESET;
	data[1] = (i==1)?0x80:0x20;
	i2c_write_blocking(i2c1, I2C_VFO, data, 2, false);
}


// For each vfo, calculate required MSN setting, MSN = MSi*Ri*Fout/Fxtal
// If in range, just set MSN registers
// If not in range, recalculate MSi and Ri and also MSN
// Set MSN, MSi and Ri registers (implicitly resets PLL)
void si_evaluate(void)
{
	float msn;

	if (vfo[0].flag)
	{
		msn = (float)(vfo[0].msi); 														// Re-calculate MSN
		msn = msn * (float)(vfo[0].ri);
		msn = msn * (float)(vfo[0].freq) / SI_XTAL_FREQ;
		if ((msn>=SI_MSN_LO)&&(msn<SI_MSN_HI))
		{
			vfo[0].msn = msn;
			si_setmsn(0);
		}
		else
		{
			vfo[0].ri  = (vfo[0].freq<1000000)?128:((vfo[0].freq<3000000)?32:1);		// Pre-scale Ri, stretch down Ri=1 range
			if ((vfo[0].freq >= 3000000)&&(vfo[0].freq < 6000000))						// Low end of Ri=1 range
				vfo[0].msi = (uint8_t)126;												// Maximum MSi on Fvco=(4x126)MHz
			else
				vfo[0].msi = (uint8_t)(750000000UL / (vfo[0].freq * vfo[0].ri)) & 0xfe;	// Calculate MSi on Fvco=750MHz
			msn = (float)(vfo[0].msi); 													// Re-calculate MSN
			msn = msn * (float)(vfo[0].ri);
			msn = msn * (float)(vfo[0].freq) / SI_XTAL_FREQ;
			vfo[0].msn = msn;
			si_setmsn(0);
			si_setmsi(0);
		}
		vfo[0].flag = 0;
	}
	if (vfo[1].flag)
	{
		vfo[1].flag = 0;
	}
}


// Initialize the Si5351 VFO registers
void si_init(void)
{
	uint8_t data[16];		// I2C trx buffer

	i2c_init(i2c1, 400*1000);
	gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C1_SDA);
	gpio_pull_up(I2C1_SCL);

	// Hard initialize Synth registers: all 10MHz, CLK1 90 deg ahead, PLLA for CLK 0&1, PLLB for CLK2
	// Ri=1,
	// MSi=68,    P1=8192, P2=0,      P3=1
	// MSN=27.2   P1=2969, P2=600000, P3=1000000
	vfo[0].freq  = 10000000;
	vfo[0].flag  = 0;
	vfo[0].phase = 1;
	vfo[0].ri    = 1;
	vfo[0].msi   = 68;
	vfo[0].msn   = 27.2;
	vfo[1].freq  = 10000000;
	vfo[1].flag  = 0;
	vfo[1].phase = 0;
	vfo[1].ri    = 1;
	vfo[1].msi   = 68;
	vfo[1].msn   = 27.2;

	// PLLA: MSN P1=0x00000b99, P2=0x000927c0, P3=0x000f4240
	data[0] = SI_SYNTH_PLLA;
	data[1] = 0x42;		// MSNA_P3[15:8]
	data[2] = 0x40;		// MSNA_P3[7:0]
	data[3] = 0x00;		// 0b000000 , MSNA_P1[17:16]
	data[4] = 0x0b;		// MSNA_P1[15:8]
	data[5] = 0x99;		// MSNA_P1[7:0]
	data[6] = 0xf9;		// MSNA_P3[19:16] , MSNA_P2[19:16]
	data[7] = 0x27;		// MSNA_P2[15:8]
	data[8] = 0xc0;		// MSNA_P2[7:0]
	i2c_write_blocking(i2c1, I2C_VFO, data, 9, false);

	
	// PLLB: MSN P1=0x00000b99, P2=0x000927c0, P3=0x000f4240
	data[0] = SI_SYNTH_PLLB;		// Same content
	i2c_write_blocking(i2c1, I2C_VFO, data, 9, false);

	// MS0 P1=0x00002000, P2=0x00000000, P3=0x00000001, R=1
	data[0] = SI_SYNTH_MS0;
	data[1] = 0x00;		// MS0_P3[15:8]
	data[2] = 0x01;		// MS0_P3[7:0]
	data[3] = 0x00;		// 0b0, R0_DIV[2:0] , MS0_DIVBY4[1:0] , MS0_P1[17:16] 
	data[4] = 0x20;		// MS0_P1[15:8]
	data[5] = 0x00;		// MS0_P1[7:0]
	data[6] = 0x00;		// MS0_P3[19:16] , MS0_P2[19:16]
	data[7] = 0x00;		// MS0_P2[15:8]
	data[8] = 0x00;		// MS0_P2[7:0]
	i2c_write_blocking(i2c1, I2C_VFO, data, 9, false);

	// MS1 P1=0x00002000, P2=0x00000000, P3=0x00000001, R=1
	data[0] = SI_SYNTH_MS1;		// Same content
	i2c_write_blocking(i2c1, I2C_VFO, data, 9, false);

	// MS2 P1=0x00002000, P2=0x00000000, P3=0x00000001, R=1
	data[0] = SI_SYNTH_MS2;		// Same content
	i2c_write_blocking(i2c1, I2C_VFO, data, 9, false);

	// Phase offsets for 3 clocks
	data[0] = SI_CLK0_PHOFF;
	data[1] = 0x00;		// CLK0: phase 0 deg
	data[2] = 0x44;		// CLK1: phase 90 deg (=MSi)
	data[3] = 0x00;		// CLK2: phase 0 deg
	i2c_write_blocking(i2c1, I2C_VFO, data, 4, false);

	// Output port settings for 3 clocks
	data[0] = SI_CLK0_CTL;
	data[1] = 0x4f;		// CLK0: INT, PLLA, nonINV, MS, 8mA
	data[2] = 0x4f;		// CLK1: INT, PLLA, nonINV, MS, 8mA
	data[3] = 0x6f;		// CLK2: INT, PLLB, nonINV, MS, 8mA
	i2c_write_blocking(i2c1, I2C_VFO, data, 4, false);

	// Disable spread spectrum (startup state is undefined)	
	data[0] = SI_SS_EN;
	data[1] = 0x00;
	i2c_write_blocking(i2c1, I2C_VFO, data, 2, false);
	
	// Reset both PLL
	data[0] = SI_PLL_RESET;
	data[1] = 0xa0;
	i2c_write_blocking(i2c1, I2C_VFO, data, 2, false);

	// Enable all outputs	
	data[0] = SI_CLK_OE;
	data[1] = 0x00;
	i2c_write_blocking(i2c1, I2C_VFO, data, 2, false);
}

