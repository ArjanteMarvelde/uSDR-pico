/*
 * relay.c
 *
 * Created: Nov 2021
 * Author: Arjan te Marvelde
 * 
 * Two PCF8574 expanders are on the I2C bus, one on the RX and one on the BPF board.
 * The RX (0x42) bit assignments:
 *  0: Enable -20dB attenuator
 *  1: Enable -10dB attenuator
 *  2: Enable +10dB pre-amplifier
 * The BPF (0x40) bit assignments:
 *  0: Enable LPF  2.5 MHz
 *  1: Enable BPF  2.0 - 6.0 MHz
 *  2: Enable BPF  5.0 -12.0 MHz
 *  3: Enable BPF 10.0 -24.0 MHz
 *  4: Enable BPF 20.0 -40.0 MHz
 * 
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "relay.h"


/* I2C address and pins */
#define I2C_RX 				0x21
#define I2C_BPF				0x20


void relay_setband(uint8_t val)
{
	uint8_t data[2];
	int ret;
	
	data[0] = val&0x1f;
	if (i2c_write_blocking(i2c1, I2C_BPF, data, 1, false) < 0)
		i2c_write_blocking(i2c1, I2C_BPF, data, 1, false);
}

int relay_getband(void)
{
	uint8_t data[2];
	int ret;
	
	ret = i2c_read_blocking(i2c1, I2C_BPF, data, 1, false);
	if (ret>=0) 
		ret=data[0];
	return(ret);
}

void relay_setattn(uint8_t val)
{
	uint8_t data[2];
	
	data[0] = val&0x07;
	if (i2c_write_blocking(i2c1, I2C_RX, data, 1, false) < 0)
		i2c_write_blocking(i2c1, I2C_RX, data, 1, false);
}

int relay_getattn(void)
{
	uint8_t data[2];
	int ret;
	
	ret = i2c_read_blocking(i2c1, I2C_RX, data, 1, false);
	if (ret>=0) 
		ret=data[0];
	return(ret);
}

void relay_init(void)
{ 
	relay_setattn(REL_PRE_10);
	sleep_ms(1);
	relay_setband(REL_BPF12);
}