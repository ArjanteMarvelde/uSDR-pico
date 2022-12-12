/*
 * relay.c
 *
 * Created: Nov 2021
 * Author: Arjan te Marvelde
 * 
 * Two PCF8574 expanders are on the I2C bus, one on the RX and one on the BPF board.
 * The RX (0x42) bit assignments:
 *  0x03: Enable -20dB and -10dB attenuators
 *  0x01: Enable -20dB attenuator
 *  0x02: Enable -10dB attenuator
 *  0x04: Enable +10dB pre-amplifier
 *  0x00: No attenuator or pre-amp
 *
 * The BPF (0x40) bit assignments:
 *  0x01: Enable LPF  2.5 MHz
 *  0x02: Enable BPF  2.0 - 6.0 MHz
 *  0x04: Enable BPF  5.0 -12.0 MHz
 *  0x08: Enable BPF 10.0 -24.0 MHz
 *  0x10: Enable BPF 20.0 -40.0 MHz
 * 
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "uSDR.h"
#include "relay.h"

void relay_setband(int val)
{
	uint8_t data;
	
	data = ((uint8_t)val)&0x1f;
	if (i2c_put_data(i2c1, I2C_BPF, &data, 1, false) < 0)
		i2c_put_data(i2c1, I2C_BPF, &data, 1, false);
	sleep_ms(1);
}

int relay_getband(void)
{
	uint8_t data;
	int ret;
	
	ret = i2c_get_data(i2c1, I2C_BPF, &data, 1, false);
	if (ret>=0)
		ret=data;
	return(ret);
}

void relay_setattn(int val)
{
	uint8_t data;
	
	data = ((uint8_t)val)&0x07;
	if (i2c_put_data(i2c1, I2C_RX, &data, 1, false) < 0)
		i2c_put_data(i2c1, I2C_RX, &data, 1, false);
	sleep_ms(1);
}

int relay_getattn(void)
{
	uint8_t data;
	int ret;
	
	ret = i2c_get_data(i2c1, I2C_RX, &data, 1, false);
	if (ret>=0) 
		ret=data;
	return(ret);
}

void relay_init(void)
{ 
	relay_setattn(REL_PRE_10);
	sleep_ms(1);
	relay_setband(REL_BPF12);
}