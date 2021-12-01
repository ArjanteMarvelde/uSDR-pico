# uSDR-pico
This Git repository contains a Micro-SDR implementation, based on a RP2040 Pi Pico. The project is highly experimental, foremost intended to investigate how the Pico HW and SDK work with an application like this. It contains the code for an experimental implementation of the control and signal processing for a QSD/QSE based transceiver. 
Furthermore, the repository contains the electronic design of some modules that cover the mixing, filtering and RF amplification.  

Please see the doc folder for a full description.

The platform used is a Pi Pico module with an RP2040 processor. This processor has dual cores, running at 125MHz each and very configurable I/O, which eases the HW design.
The software is distributed over the two cores: core-0 takes care of all user I/O and control functions, while core-1 performs all of the signal processing. The core-1 functionality consists of a TX-branch and an RX-branch, each called from a function that waits for inter-core FIFO words popping out. This happens every 16usec, because on core-0 a 16usec timer callback ISR pushes the RX/TX status into that FIFO. Hence the signal processing rythm on core-1 effectively is 62.5kHz.  
On *core1* the three ADC channels are continuously sampled at maximum speed in round-robin mode. Samples are therefore taken every 6usec for each channel, maximum jitter between I and Q channels is 4usec, which has a negligible effect in the audio domain.  

The TX-branch 
- takes latest audio audio sample input from ADC2 (rate = 62.5 kHz), 
- applies a low-pass filter at Fc=3kHz, 
- reduces sampling by 4 to get better low frequency response Hilbert xform (rate = 15.625 kHz), 
- splits into an I-channel 7 sample delay line and a Q-channel 15-tap Discrete Hilbert Transform
- scales, filters and outputs I and Q samples on PWM based DACs, towards QSE output
 
The RX-branch
- takes latest Q and I samples from QSD on ADC0 and ADC1 (rate = 62.5 kHz)
- applies a low-pass filter at Fc=3kHz, 
- reduces sampling by 4 to get better low frequency response Hilbert xform (rate = 15.625 kHz), 
- demodulates, e.g. SSB:
-- applies 15-tap DHT on Q channel and 7 sample delay on I channel
-- subtracts I and Q samples
- scales, filters and outputs audio on an PWM based DAC, towards audio output

On *core0* the main loop takes care of user I/O, all other controls and the monitor port. There is also a LED flashing timer callback functioning as a heartbeat.

The Pico controls an Si5351A clock module to obtain the switching clock for the QSE and QSD. The module outputs two synchronous square wave clocks on ch 0 and 1, whith selectable phase difference (0, 90, 180 or 270 degrees). The clock on ch2 is free to be used for other goals. The module is controlled over the **i2c0** channel.
The display is a standard 16x2 LCD, but with an I2C interface. The display is connected through the **i2c1** channel, as well as the bus expanders for controlling the various relays.

## Open issues: 
- [x] take care of processing cycles, by moving signal processing parts to the second core
- [x] add some more filtering
- [x] implement the user I/O bit: LCD+Rotary encoder+buttons
- [x] implement AGC 
- [x] implement LSB
- [x] implement AM
- [x] SW based VOX
- [ ] implement RSSI
- [x] design a set of PCBs
- [x] sort out the new HW modules
- [ ] improve speed: better dual-core management for memory and timer 
- [x] add control for new HW: BPF and pre-amp/attenuator switching

## Installing and using the SDK for Windows: 
Please refer to https://github.com/ndabas/pico-setup-windows/releases where the latest installer can be downloaded (e.g. **pico-setup-windows-0.3-x64.exe**).  
Execute the installer to set up the SDK environment, e.g. in **~/Documents/Pico**  (let's call this folder **$PICO**). 

## Building uSDR-pico: 
Clone/copy the uSDR-pico code files into a subdirectory, e.g. **$PICO/uSDR-pico**  
Create the build folder: **$PICO/uSDR-pico/build**  

Before doing any building you need to adapt the file **$PICO/uSDR-pico/CMakeLists.txt**, using your favourite editor, to reflect your own directory structure. Also, select whether you want **stdio** to use the UART on pins 1 and 2 or the USB serial port. The monitor terminal is on **stdio**.  
In **$PICO/** you will find a command to start a Developer Command Prompt (*DCP*, like a "DOS box"), make sure to use this one instead of any other DOS box. Within this *DCP* all environment settings have been properly pre-set to enable building.  
In the *DCP* window, chdir to the **build** folder and execute: **cmake -G "NMake Makefiles" ..**   (do not forget the trailing dots).
Now you have initialized the make environment (for *nmake*) and by executing **nmake** in that same **build** folder, all SDK libraries and finally the Pi Pico loadable file **uSDR.uf2** will be created.  
Rebooting the Pico while the bootsel button is pressed will open a file explorer window with the Pico shown as a Mass Storage Device (e.g. drive E:). Moving the binary to the Pico is as easy as dragging and dropping this uf2 file into that MSD.  

# Background
The folder **$PICO/docs** also contains some manuals, of which the *C-SDK description*, the *RP2040 datasheet* and the *Pico Pinout* are absolute must-reads when you start writing software.  
For calculating filters I have used the free software from Iowa Hills (http://www.iowahills.com/8DownloadPage.html)  
I also used the online FIR filter calculator T-Filter (http://t-filter.engineerjs.com/) 

# Copyright notice
**The code and electronic designs as well as the implementations presented in this repository can be copied and modified freely, for non-commercial use.
Use for commercial purposes is allowed as well, as long as a reference to this repository is included in the product.**


