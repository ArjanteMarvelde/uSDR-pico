# uSDR-pico
uSDR implementation based on a RP2040 Pi Pico

This is the repository for an experimental implementation of the control and signal processing for a QSD/QSE based transceiver. The platform used is a Pi Pico module with an RP2040 processor. This processor has dual core, running at 125MHz each and very configurable I/O which eases the HW design.

The software consists of a TX branch and an RX branch, each running inside a timer callback function, once every 16 usec. This makes the signal processing rythm 62.5kHz. 
The TX branch 
- samples audio input with ADC2 (rate = 62.5 kHz), 
- applies a low-pass filter Fc=3kHz, 
- reduces sampling by 2 to get better low frequency Hilbert transform (rate = 31.25 kHz), 
- splits into an I-channel 7 sample delay line and a Q-channel 15-tap DHT
- scales and outputs I and Q samples on PWM based DACs towards filters, opamps and QSE

The RX branch
- intermittently samples I and Q channels from QSD on ADC0 and ADC1 (rate = 31.25 kHz)
- corrects for sampling shift between I and Q (average last two I samples)
- applies 15-tap DHT on Q channel and 7 sample delay on I channel
- adds I and Q samples
- scales and outputs audio on an PWM based DAC

The main loop takes care of user I/O and the serial port. There is also a LED flashing timer callback.

The Pico controls an Si5351A clock module to obtain the switching clock for the QSE and QSD. The module outputs two synchronous square wave clocks on ch 0 and 1, whith selectable phase difference (0, 90, 180 or 270 degrees). The clock on ch2 is free to be used for other goals. The module is controlled over the i2c0 channel.
The display is a standard 16x2 LCD, but with an I2C interface. The display is connected through the i2c1 channel.

Open issues:
- take care of processing cycles, by moving parts to the second core
- add some more filtering 
- implement the user I/O bit

