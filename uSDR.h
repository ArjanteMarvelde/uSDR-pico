#ifndef __USDR_H__
#define __USDR_H__
/* 
 * uSDR.h
 *
 * Created: Aug 2022
 * Author: Arjan te Marvelde
 *
 * This file contains the system-wide definitions and platform services.
 *
 */

#include "hardware/i2c.h"




#define DSP_FFT					1											// Set this to 1 when FFT engine must be used
#define CH_I					0											// Q channel index for ADC, DAC, etc
#define CH_Q					1											// I channel index for ADC, DAC, etc
#define CH_A					2											// A channel index for ADC, DAC, etc


/* GPIO assignments and pinning*/

#define GP_AUX0					0											// Pin  1: Auxiliary 0
#define GP_AUX1					1											// Pin  2: Auxiliary 1

#define GP_AUX2					2											// Pin  4: Auxiliary 2
#define GP_AUX3					3											// Pin  5: Auxiliary 3
#define GP_ENC_A				4											// Pin  6: Encoder channel A
#define GP_ENC_B				5											// Pin  7: Encoder channel B

#define GP_BTN_0				6											// Pin  9: Cancel
#define GP_BTN_1				7											// Pin 10: Right move
#define GP_BTN_2				8											// Pin 11: Left move
#define GP_BTN_3				9											// Pin 12: Accept

#define LCD_BL					10											// Pin 14: LCD backlight, on
#define LCD_RST					11											// Pin 15: LCD reset, on
#define LCD_DC					12											// Pin 16: LCD data/control, on
#define LCD_CS					13											// Pin 17: LCD Chip select (NC, always low)

#define LCD_CLK					14											// Pin 19: LCD SPI clock
#define LCD_MOSI				15											// Pin 20: LCD SPI data

#define I2C0_SDA				16											// Pin 21: I2C channel 0 - data
#define I2C0_SCL				17											// Pin 22: I2C channel 0 - clock

#define GP_PTT_OUT				18											// Pin 24: Output PTT to bus, TX=1, RX=0
#define GP_PTT_IN				19											// Pin 25: Input PTT from Mic, TX=0, RX=1
#define DAC_Q					20											// Pin 26: PWM DAC Q channel
#define DAC_I					21											// Pin 27: PWM DAC I channel

#define DAC_A					22											// Pin 29: PWM DAC A channel
#define PICO_RESET				-1											// Pin 30: CPU Reset, no SW use
#define ADC_I					26											// Pin 31: ADC 0, Q channel
#define ADC_Q					27											// Pin 32: ADC 1, I channel

#define ADC_A					28											// Pin 34: ADC 2, A channel
#define PICO_ADCREF				-1											// Pin 35: ADC reference voltage, no SW use
#define PICO_3V3OUT				-1											// Pin 36: 3V3 supply output, no SW use
#define PICO_3V3EN				-1											// Pin 37: 3V3 enable input, no SW use

#define PICO_VSYS				-1											// Pin 39: Vcc (5V) PSU input, no SW use
#define PICO_VBUS				-1											// Pin 40: Vcc internal, no SW use



/* Timer values */

#define LED_MS					1000										// LED flashing, half cycle duration
#define LOOP_MS					100											// Core 0 main loop timer (see also uSDR.c)


/* I2C addresses */

#define I2C_RX					0x21										// Expander on Rx board
#define I2C_BPF					0x20										// Expander on Filter board
#define I2C_VFO					0x60										// Si5351A


/* I2C wrapper functions (blocking write and read) */

#define I2C_LINGER_US			200											// Linger time added after i2c SDK functions

int i2c_put_data(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop);
int i2c_get_data(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop);


/* LCD type selection (see also lcd.c) */

#define LCD_1804			0												// Type 0: Seeed / Grove
#define LCD_8574_ADA		1												// Type 1: Adafruit I2C backpack
#define LCD_8574_GEN		2												// Type 2: Generic I2C backpack
#define LCD_IL9341			3												// Type 3: Graphical 320x240
#define LCD_TYPE			LCD_IL9341										// Active selection



#endif
