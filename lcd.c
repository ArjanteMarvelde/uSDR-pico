/*
 * lcd.c
 *
 * Created: Sep 2024
 * Author: Arjan te Marvelde
 * 
 *
 * This file contains the driver for a Waveshare 240x320 colour LCD display, based on a 
 * IL9341 controller. In contrast with the previous 16x2 alphanumeric display, this one
 * is graphical and uses SPI as interface. For this reason the driver has been rewritten.
 * For alphanumeric solutions please download a previous version.

   LCD_BL					10			// Pin 14: LCD backlight, on
   LCD_RST					11			// Pin 15: LCD reset, on
   LCD_DC					12			// Pin 16: LCD data/control, on
   LCD_CS					13			// Pin 17: Not connected, do not use
   LCD_CLK					14			// Pin 19: LCD SPI clock
   LCD_MOSI					15			// Pin 20: LCD SPI data
   
 * The Waveshare TFT display and the ILI9341 controller have been configured for a portrait 
 * orientation by default. The controller contains a GRAM memory that refreshes the display 
 * in a certain order determined by some settings. The writing of pixels from MCU into the GRAM
 * also follows a certain preset order. This writing is sequential, without specific addressing. 
 * The default order is line (page) after line. 
 * The order refresh and GRAM writing can be set with the Memory Access Control command (0x36). 
 * 
 * This command has the following bits in the data field:
 * 7 - MY	Row address order, how pixels are transferred from MCU to GRAM
 * 6 - MX	Column address order, how pixels are transferred from MCU to GRAM
 * 5 - MV	Row/Column swap, write columns instead of rows. This is handy when display in landscape.
 * 4 - ML	Vertical refresh order, how pixels are transferred from GRAM to display
 * 3 - BGR	BGR color order instead of RGB
 * 2 - MH	Horizontal refresh order, how pixels are transferred from GRAM to display
 * 1 - X
 * 0 - X
 * You need to play with bits 7, 6 and 5 to get the proper orientation. 
 * In my case 101 worked for a 320 wide x 240 high display with origin in the upper left corner.
 * Changing the refresh order (ML or MH) will flip the display vertically or horizontally respectively.
 *
 * Transferring actual pixel data from MCU to GRAM can be initiated by either Memory write (0x2C) 
 * or Memory write continue (0x3C) commands. The first command resets the page and column pointers 
 * to the page origin, while the other command continues from the last position. 
 * 
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "uSDR.h"
#include "lcd.h"
#include "fonts.h"

#define LCD_MASK_OUT			((1<<LCD_BL)|(1<<LCD_RST)|(1<<LCD_DC)|(1<<LCD_CS))


/** ILI9341 commands **/
#define LCD_NOP			0x00		// No Operation
#define LCD_RESET		0x01		// Software reset
#define LCD_SLPON		0x10		// Sleep mode ON
#define LCD_SLPOFF		0x11		// Sleep mode OFF
#define LCD_PTLON		0x12		// Set partial mode
#define LCD_NORON		0x13		// Set normal mode
#define LCD_INVOFF		0x20		// Turn inverse mode OFF
#define LCD_INVON		0x21		// Turn inverse mode ON
#define LCD_GAMMASET	0x26		// Set gamma
#define LCD_DISPOFF		0x28		// Turn display OFF
#define LCD_DISPON		0x29		// Turn display ON

#define LCD_COLSET		0x2A		// Set column address
#define LCD_PAGSET		0x2B		// Set page address
#define LCD_MEMWR		0x2C		// Memory write, after reset window start location
#define LCD_COLOR		0x2D		// Color set
#define LCD_PTLAREA		0x30		// Partial area set
#define LCD_VERSCR		0x33		// Vertical scrolling area
#define LCD_TEAROFF		0x34		// Tear effect line OFF
#define LCD_TEARON		0x35		// Tear effect line ON
#define LCD_MACTL		0x36		// Memory access control
#define LCD_VERSCRST	0x37		// Vertical scroll start address
#define LCD_IDLEOFF		0x38		// Idle mode off, full color range
#define LCD_IDLEON		0x39		// Idle mode on, reduced color range
#define LCD_PIXFMT		0x3A		// Pixel format set
#define LCD_MEMWRCONT	0x3C		// Memory write, continuing from last location
#define LCD_TEARSCANL	0x44		// Set tear scanline
#define LCD_DISPBR		0x51		// Set display brightness
#define LCD_CTRLDIS		0x53		// Write CTRL display
#define LCD_ADAPBR		0x55		// Set adaptive brightness control
#define LCD_MINBR		0x5E		// Set CABC minimum brightness

#define LCD_RGBCTL		0xB0		// RGB interface signal control
#define LCD_FRMCTL1		0xB1		// Frame control (normal mode)
#define LCD_FRMCTL2		0xB2		// Frame control (idle mode)
#define LCD_FRMCTL3		0xB3		// Frame control (partial mode)
#define LCD_INVCTL		0xB4		// Display inversion control
#define LCD_BLANKCTL	0xB5		// Blanking porch control
#define LCD_DFUNCTL		0xB6		// Display function control

#define LCD_PWCTL1		0xC0		// Power control 1
#define LCD_PWCTL2		0xC1		// Power control 2
#define LCD_VCOMCTL1	0xC5		// VCOM control 1
#define LCD_VCOMCTL2	0xC7		// VCOM control 2

#define LCD_PGAMCOR		0xE0		// Positive gamma correction
#define LCD_NGAMCOR		0xE1		// Negative gamma correction


/*
 * Inline definitions for SPI transfers 
 * Parameters: char *x and size_t n
 */
#define SPI_CMD(x)   { gpio_put(LCD_DC, 0); spi_write_blocking(spi1, x, 1); }
#define SPI_DAT(x,n) { gpio_put(LCD_DC, 1); spi_write_blocking(spi1, x, n); }


/*
 * Reset the ILI9341 TFT display driver
 */
void lcd_reset()
{
	sleep_ms(20);
	gpio_put(LCD_RST, 0);
	sleep_ms(20);
	gpio_put(LCD_RST, 1);
	sleep_ms(100);								// Let ILI9341 finish (20ms does not work!)
}


/*
 * Set window (column, page, width, height)
 */
int lcd_window(uint16_t c, uint16_t p, uint16_t w, uint16_t  h)
{
	uint8_t x[5];
	
	// Range check
	if ((c>(LCD_WIDTH-1))||(p>(LCD_HEIGHT-1))||(w>(LCD_WIDTH-c))||(h>(LCD_HEIGHT-p)))
		return(-1);
	
	// Set column start and end addresses
	x[0] = LCD_COLSET;  			   			// CASET
	x[1] = (c>>8);								// Start column
	x[2] = (c&0xff);
	x[3] = ((c+w-1)>>8);						// End column
	x[4] = ((c+w-1)&0xff); 	
	SPI_CMD(x); SPI_DAT(x+1, 4);

	// Set page start and end addresses
	x[0] = LCD_PAGSET;     						// PASET
	x[1] = (p>>8);								// Start page
	x[2] = (p&0xff);
	x[3] = ((p+h-1)>>8);						// End page
	x[4] = ((p+h-1)&0xff); 
	SPI_CMD(x); SPI_DAT(x+1, 4);
	
	x[0] = LCD_MEMWR;							// Point at window start address
	SPI_CMD(x);
	
	return(0);
}
	
	
/*
 * Clear complete display with specified colour
 */
int lcd_clear(uint16_t c, uint16_t p, uint16_t w, uint16_t h, uint16_t color)
{
	uint32_t len;
	uint8_t x[2*LCD_WIDTH];
	
	if (0>lcd_window(c, p, w, h)) return(-1);	// Define window
	for (len=0; len<2*w; len+=2)		// Define one display line
	{
		x[len]   = color>>8;
		x[len+1] = color&0xff;
	}
	len = h;
	while (len-->0) 							// Fill window
		SPI_DAT(x, 2*w);
	
	return(0);
}


/*
 * Draw one character on the display
 */
int lcd_putxy(uint16_t x, uint16_t y, char c, sFONT* f, uint16_t fgc, uint16_t bgc)
{
	uint8_t pix[FON_MAXPIX*2];					// Array containing all character pixels
	uint8_t *p, *q, m;
	uint32_t len;
	int i, j;
	
	// Range check
	if ((c<f->First) || (c>f->Last)) return(-1);
	
	// Create window for character
	lcd_window(x, y, f->Width, f->Height);

	// Define parameters
	len = (f->Width>>3);						// Nr of horizontal font char bytes
	if (f->Width&0x7) len++;
	len = len*f->Height;						// Total nr of font char bytes
	p = &(f->table[(c - f->First) * len]);		// Point to first char byte in font table
	q = pix;									// Point to first pixel byte
	
	// Fill the pixel array (can be faster with n*8 Width)
	for (j=0; j<f->Height; j++)					// For each line
	{
		m=0x80;									// Initialize bitmask for new line
		for (i=0; i<f->Width; i++)				// For each bit
		{
			if (*p & m)							// Bit set?
			{
				*q++ = fgc>>8;					// Set Foreground colour
				*q++ = fgc&0xff;				//   and low byte
			}
			else
			{
				*q++ = bgc>>8;					// Set Background colour
				*q++ = bgc&0xff;				//   and low byte
			}
			if (m==0x01) 						// Last bit?
			{
				m = 0x80;						// Reset mask
				p++;							// Next byte
			}
			else
			{
				m >>= 1;						// Shift mask
			}
		}	
		if (m!=0x80) p++;						// incomplete last byte
	}
	
	// Transfer pixels
	len = f->Width*f->Height*2;					// Actual amount of pixel bytes
	SPI_DAT(pix, len);

	return(0);
}


/*
 * Draw a string of characters on display
 */
int lcd_writexy(uint16_t x, uint16_t y, char *s, sFONT* f, uint16_t fgc, uint16_t bgc)
{
	char *pc;
	uint16_t xc;
	
	pc = s;
	xc = x;
	while (*pc != 0)
	{
		if (lcd_putxy(xc, y, *pc, f, fgc, bgc)<0) return(-1);
		xc += f->Width;
		pc++;
	}
	return(0);
}


/*
 * Initialize LCD interface and screen
 * Specific to ILI9143, code based on Waveshare 2.4" driver.
 * Correct settings found after much experimentation...
 */
void lcd_init(void)
{ 
	uint8_t x[64];
	uint8_t i;
	
	// Init output GPIOs, spi is initialized in uSDR.c
	gpio_init_mask(LCD_MASK_OUT);	
	gpio_set_dir_out_masked (LCD_MASK_OUT);
	gpio_set_drive_strength (LCD_DC, GPIO_DRIVE_STRENGTH_8MA);
	gpio_put(LCD_RST, 1);						// Non reset state
	gpio_put(LCD_BL, 1);						// Backlight off
	gpio_put(LCD_DC, 1);						// Data transfer
	gpio_put(LCD_CS, 0);						// Chip selected

	lcd_reset();
	
	x[0] = LCD_SLPOFF; 							// Wake up chip
	SPI_CMD(x);

	/*** Initialisation settings for Waveshare ILI9341 display ***/
// As defined in ESP driver
//  0xEF, 3, 0x03, 0x80, 0x02,
	x[0] = 0xEF; x[1] = 0x03; x[2] = 0x80; x[3] = 0x02; 
	SPI_CMD(x);	SPI_DAT(x+1, 3);	
	
//  0xCF, 3, 0x00, 0xC1, 0x30,
	x[0] = 0xCF; x[1] = 0x00; x[2] = 0xC1; x[3] = 0x30; 
	SPI_CMD(x);	SPI_DAT(x+1, 3);	

//  0xED, 4, 0x64, 0x03, 0x12, 0x81,
	x[0] = 0xED; x[1] = 0x64; x[2] = 0x03; x[3] = 0x12;  x[4] = 0x81;
	SPI_CMD(x);	SPI_DAT(x+1, 4);	
	
//  0xE8, 3, 0x85, 0x00, 0x78,
	x[0] = 0xE8; x[1] = 0x85; x[2] = 0x00; x[3] = 0x79; 
	SPI_CMD(x);	SPI_DAT(x+1, 3);	
	
//  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
	x[0] = 0xCB; x[1] = 0x39; x[2] = 0x2C; x[3] = 0x00;  x[4] = 0x34;  x[5] = 0x02;
	SPI_CMD(x);	SPI_DAT(x+1, 5);	
		
//  0xF7, 1, 0x20,
	x[0] = 0xF7; x[1] = 0x20;
	SPI_CMD(x);	SPI_DAT(x+1, 1);	
	
//  0xEA, 2, 0x00, 0x00,
	x[0] = 0xEA; x[1] = 0x00; x[2] = 0x00;
	SPI_CMD(x);	SPI_DAT(x+1, 2);	


//  ILI9XXX_PWCTR1  , 1, 0x23,
	x[0] = LCD_PWCTL1; 															// 0xC0 Power control 1, 
	x[1] = 0x1D;																// 0x1D; VRH[5:0]
	SPI_CMD(x);	SPI_DAT(x+1, 1);	

//  ILI9XXX_PWCTR2  , 1, 0x10,
	x[0] = LCD_PWCTL2; 															// 0xC1 Power control 2, 
	x[1] = 0x12;																// 0x12; SAP[2:0];BT[3:0]
	SPI_CMD(x);	SPI_DAT(x+1, 1);	

//  ILI9XXX_VMCTR1  , 2, 0x3e, 0x28,
	x[0] = LCD_VCOMCTL1; 														// 0xC5 VCOM control 1
	x[1] = 0x33; 																// 0x33; 
	x[2] = 0x3F;																// 0x3F;
	SPI_CMD(x);	SPI_DAT(x+1, 2);	
	
//  ILI9XXX_VMCTR2  , 1, 0x86,
	x[0] = LCD_VCOMCTL2; 														// 0xC7 VCOM control 2
	x[1] = 0x82;																// 0x92;
	SPI_CMD(x);	SPI_DAT(x+1, 1);	

//  ILI9XXX_PIXFMT  , 1, 0x55,
	x[0] = LCD_PIXFMT; 															// 0x3A Pixel format
	x[1] = 0x55;																// 0x55; 16 bits per pixel, (def=0x66, 18bpp)
	SPI_CMD(x);	SPI_DAT(x+1, 1);	

	x[0] = LCD_IDLEOFF; 														// 0x38 Idle mode off
	SPI_CMD(x);	

//  ILI9XXX_MADCTL  , 1, 0x48,
	x[0] = LCD_MACTL; 															// 0x36 Memory Access Control
	x[1] = 0xE8;																// 0xA0; MY, MX, MV, ML, BGR, MH, (def=0x00)
	SPI_CMD(x);	SPI_DAT(x+1, 1);	

//  ILI9XXX_FRMCTR1 , 2, 0x00, 0x18,			
	x[0] = LCD_FRMCTL1; 														// 0xB1 Frame control (normal mode)
	x[1] = 0x00;																// 0x00; No clock divider, def=0x00
	x[2] = 0x1B;																// 0x12; Frame rate 106Hz, def=0x1B (70Hz)
	SPI_CMD(x);	SPI_DAT(x+1, 2);	

//  ILI9XXX_DFUNCTR , 3, 0x08, 0x82, 0x27,
	x[0] = LCD_DFUNCTL; 														// 0xB6 Display Function Control
	x[1] = 0x08;																// 0x0A; def=0x0A ()
	x[2] = 0x82;																// 0xA2; def=0x82 ()
	x[3] = 0x27;																// none; 
	SPI_CMD(x);	SPI_DAT(x+1, 3);	

//	x[0] = LCD_TEARSCANL; 														// Set tear scanline
//	x[1] = 0x02;																// 0x02; 
//	SPI_CMD(x);	SPI_DAT(x+1, 1);	

//  0xF2, 1, 0x00,
	x[0] = 0xF2; 																// Gamma Function Disable
	x[1] = 0x00;																// 0x00; 
	SPI_CMD(x);	SPI_DAT(x+1, 1);	

//  ILI9XXX_GAMMASET , 1, 0x01,
	x[0] = LCD_GAMMASET; 														// Gamma curve selected
	x[1] = 0x01;																// 0x01; 
	SPI_CMD(x);	SPI_DAT(x+1, 1);	

//  ILI9XXX_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
	x[0] = LCD_PGAMCOR; 														// Positive gamma correction
	x[ 1] = 0x0F; x[ 2] = 0x31; x[ 3] = 0x2b; x[ 4] = 0x0c; x[ 5] = 0x0e; 
	x[ 6] = 0x08; x[ 7] = 0x4e; x[ 8] = 0xf1; x[ 9] = 0x37; x[10] = 0x07; 
	x[11] = 0x10; x[12] = 0x03; x[13] = 0x0e; x[14] = 0x09; x[15] = 0x00;
	SPI_CMD(x);	SPI_DAT(x+1, 15);
	
//  ILI9XXX_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
	x[0] = LCD_NGAMCOR; 														// Negative gamma correction
	x[ 1] = 0x00; x[ 2] = 0x0e; x[ 3] = 0x14; x[ 4] = 0x03; x[ 5] = 0x11; 
	x[ 6] = 0x07; x[ 7] = 0x31; x[ 8] = 0xc1; x[ 9] = 0x48; x[10] = 0x08; 
	x[11] = 0x0f; x[12] = 0x0c; x[13] = 0x31; x[14] = 0x36; x[15] = 0x0f;
	SPI_CMD(x);	SPI_DAT(x+1, 15);

	x[0] = LCD_COLOR; 															// Color set
	SPI_CMD(x);
	for (i=0; i<32; i++) x[i] = i*2+1;											// Red mappings 5bit -> 6bit
	SPI_DAT(x, 32)
	for (i=0; i<64; i++) x[i] = i;												// Green mappings 6bit -> 6bit
	SPI_DAT(x, 64)
	for (i=0; i<32; i++) x[i] = i*2+1;											// Blue mappings 5bit -> 6bit
	SPI_DAT(x, 32)
	
	x[0] = LCD_DISPON; 															// Display on
	SPI_CMD(x);
}


/*
 * Test routine called from serial monitor
 */
char lastchar;
void lcd_test(void)
{
	char chr[64];
	uint16_t i, j, w, h;
	sFONT *f;
	
	// Dump font on display
	f = &FontUB;
	w = LCD_WIDTH/f->Width;
	h = LCD_HEIGHT/f->Height;
	lcd_clear(0, 0, LCD_WIDTH, LCD_HEIGHT, BLACK);
	for (i=0; i<h; i++)
	{
		for(j=0; j<w; j++) 
			chr[j] = (uint8_t)(0x20+w*i+j);
		lastchar = chr[j];
		chr[w] = 0;
		if (0>lcd_writexy(0, i*(f->Height), chr, f, YELLOW, DGRAY)) break;
	}
	
	sleep_ms(1000);
	
	// Dump EGA colors on display
	f = &Font16;
	w = f->Width;
	h = f->Height;
	lcd_clear(0, 0, LCD_WIDTH, LCD_HEIGHT, BLACK);
	i=0;
	lcd_writexy(0, i*h, "BLUE"     , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, BLUE, BLUE); i++;
	lcd_writexy(0, i*h, "GREEN"    , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, GREEN, GREEN); i++;
	lcd_writexy(0, i*h, "CYAN"     , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, CYAN, CYAN); i++;
	lcd_writexy(0, i*h, "RED"      , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, RED, RED); i++;
	lcd_writexy(0, i*h, "MAGENTA"  , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, MAGENTA, MAGENTA); i++;
	lcd_writexy(0, i*h, "BROWN"    , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, BROWN, BROWN); i++;
	lcd_writexy(0, i*h, "LGRAY"    , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, LGRAY, LGRAY); i++;
	lcd_writexy(0, i*h, "DGRAY"    , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, DGRAY, DGRAY); i++;
	lcd_writexy(0, i*h, "LBLUE"    , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, LBLUE, LBLUE); i++;
	lcd_writexy(0, i*h, "LGREEN"   , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, LGREEN, LGREEN); i++;
	lcd_writexy(0, i*h, "LCYAN"    , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, LCYAN, LCYAN); i++;
	lcd_writexy(0, i*h, "ORANGE"   , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, ORANGE, ORANGE); i++;
	lcd_writexy(0, i*h, "LMAGENTA" , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, LMAGENTA, LMAGENTA); i++;
	lcd_writexy(0, i*h, "YELLOW"   , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, YELLOW, YELLOW); i++;
	lcd_writexy(0, i*h, "WHITE"    , f, WHITE, BLACK); lcd_writexy(12*w, i*h, "            ", f, WHITE, WHITE); i++;
}