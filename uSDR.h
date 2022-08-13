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


/* Set this to 1 when FFT engine must be used */

#define DSP_FFT					1


/* GPIO (pin) assignments */

#define GP_ENC_A				2											// Pin  4: Encoder channel A
#define GP_ENC_B				3											// Pin  5: Encoder channel B
#define GP_AUX_0				6											// Pin  9: Enter, Confirm
#define GP_AUX_1				7											// Pin 10: Escape, Cancel
#define GP_AUX_2				8											// Pin 11: Left move
#define GP_AUX_3				9											// Pin 12: Right move
#define GP_PTT					15											// Pin 20: PTT line (low is active)
#define I2C0_SDA				16											// Pin 21: I2C channel 0 - data
#define I2C0_SCL				17											// Pin 22: I2C channel 0 - clock
#define I2C1_SDA				18											// Pin 24: I2C channel 1 - data
#define I2C1_SCL				19											// Pin 25: I2C channel 1 - clock
#define DAC_Q					20											// Pin 26: PWM DAC Q channel
#define DAC_I					21											// Pin 27: PWM DAC I channel
#define DAC_A					22											// Pin 29: PWM DAC Audio channel
#define ADC_Q					26											// Pin 31: ADC 0
#define ADC_I					27											// Pin 32: ADC 1
#define ADC_A					28											// Pin 34: ADC 2



/* Timer values */

#define LED_MS					1000										// LED flashing, half cycle duration
#define LOOP_MS					100											// Core 0 main loop timer (see also uSDR.c)


/* I2C addresses */

#define I2C_RX					0x21										// Expander on Rx board
#define I2C_BPF					0x20										// Expander on Filter board
#define I2C_VFO					0x60										// Si5351A
#define I2C_LCD					0x3E										// Grove: 0x3E, 8574 backpack range: 0x20..0x27


/* I2C wrapper functions (blocking write and read) */

#define I2C_LINGER_US			200											// Linger time added after i2c SDK functions

int i2c_put_data(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop);
int i2c_get_data(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop);


/* LCD type selection (see also lcd.c) */

#define LCD_1804			0												// Type 0: Seeed / Grove
#define LCD_8574_ADA		1												// Type 1: Adafruit I2C backpack
#define LCD_8574_GEN		2												// Type 2: Generic I2C backpack
#define LCD_TYPE			LCD_1804										// Active selection



#endif
