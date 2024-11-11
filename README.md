![uSDR-Pico 4](http://./doc/uSDR-Pico 4.jpg)  
The new V4.00 is now available:  
- Integrated audio on the CPU board  
- Moved VFO to Mixer board    
- Use discrete MMIC for LNA on RX board  
- New low power TX board  
- New layout BPF board  
- Proper bus connectors for all 5 boards,  
- Increased board size: 2" x 3.7", with the bus connectors this enables a cleaner stack  
- LCD graphical display (ILI9341 based: 320x240)  
- No more RS232, replaced with USB interface which can also be used for programming  
- Recalculated all filters  

The V4.00 schematics are now available in KiCAD format, so open for all to adapt. In the package I included a library with some dedicated definitions. Note that there are some patches needed to make things functional. These are described in the documentation. New software to run on this upgrade is also available as a package. Please update your SDK before building.

Still to be done:  
- Publish V4.10  
- More software updates   

# uSDR-pico
This Git repository contains a Micro-SDR implementation, based on a RP2040 Pi Pico.  

The project is highly experimental, foremost intended to investigate how the Pico HW and SDK work with an application like this. Also, it is a platform to experiment with digital signal processing techniques. The repository contains the code for an experimental implementation of the control and signal processing for a Quadrature Sampling Detector (QSD) and - Exciter (QSE) based transceiver. 
For completeness, the repository contains the electronic design of some modules that cover the mixing, filtering and RF amplification, as I have implemented in my prototype. See the *doc* subdirectory for full documentation.   

The ZIP files contain a consistent package, but the latest code with all the bug fixes and some new features is contained in the files in the main directory.  
Starting with the V3.00 package **uSDR-pico** contains *two signal processing engines*, selectable with a compile switch in uSDR.h. The first engine is the  time domain processor, more or less as in V2.00, and the second engine is a new FFT-based frequency domain processor.  
For a more detailed description of the software and the hardware, again refer to the elaborate documentation.  

The processor platform is a Pi Pico module, with an RP2040 device. This processor has dual cores running at 125MHz each, and a very configurable I/O which eases the HW design enormously. The platform can be overclocked, but some functions seem to become unstable when pushed too far. It is one of the topics for further investigation, although performance-wise not neccessary at the moment.
The software is distributed over the two cores: *core0* takes care of all user I/O and control functions, while *core1* performs all of the signal processing. The *core1* functionality consists of a TX-branch and an RX-branch, each invoked by a function that is synchronized by a timer every 64usec. Hence the signal processing rythm on *core1* effectively is 15.625kHz.  
On *core1* the three ADC channels are continuously sampled at maximum speed in round-robin mode. Samples are therefore taken every 6usec for each channel, maximum jitter between I and Q channels is 2usec, which has a negligible effect in the audio domain.  
For the time domain processing the TX and RX functions are executed within every 64usec timeslot, but for the frequency domain processing the samples are collected until half an FFT buffer is filled (512 samples), and hence this happens every 32msec (in background).  
On *core0* the main loop takes care of user I/O, all other controls and the monitor port. There is also a LED flashing timer callback functioning as a heartbeat.

The Pico controls an Si5351A clock module to obtain the switching clock for the QSE and QSD. The module outputs two synchronous square wave clocks on ch 0 and 1, whith selectable phase difference (0, 90, 180 or 270 degrees). The clock on ch2 is free to be used for other goals. The module is controlled over one of the I2C channels.
The display is a standard 16x2 LCD, but with an I2C interface. The display is connected through the other I2C channel, as well as the bus expanders for controlling the various relays.

## Open issues: 
- [ ] all SW to be tested on new v4.10 HW
- [ ] add simple waterfall over the 7.8kHz FFT band, as tuning assist  
- [ ] upgrade SDK and HW to the Pico 2 board
 

## Installing and using the SDK for Windows: 
For setting up the C/C++ build environment for Windows, you can follow the procedure as described in the Raspberry [Getting Started](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) document. This document also refers to a [setup script](https://github.com/raspberrypi/pico-setup-windows). The link [Download the latest release] (https://github.com/raspberrypi/pico-setup-windows/releases/latest/download/pico-setup-windows-x64-standalone.exe) point to a Windows installer for 64-bit processors. I still find VS Code a disaster and use Notepad++ as text editor and the Developer Command Prompt to start cmake and make manually.  
In case this does not work, please revert to the instructions in the getting started document.  

### Windows installer.  
Doing it manually, first download the latest release installer, for Windows 10 on a 64 bit PC. During the installation process target directory is asked, I use a directory called Pico in my Documents folder (call it **$PICO**) and install the toolchain in an SDKv1.5.1 subfolder. Everything will end up there, except VS Code, which I don't use anyway.   
  
  
## Building uSDR-pico:    
Create a folder **$PICO/uSDR-pico**.  
Clone/copy the uSDR-pico code files into **$PICO/uSDR-pico**.  
Create the build folder: **$PICO/uSDR-pico/build**  
Edit **CMakeLists.txt** to have the correct environment parameter *PICO_SDK_PATH*. Note that every time you change something in **CMakeLists.txt** (like adding another source file to the build) you will have to clean the build folder and re-issue cmake.  
 
All building is using Ninja, which has to be done from a **VS Developer Command Prompt for Pico** (*DCP*) because it sets up the proper build environment. A shortcut to *DCP* is found in the Start menu under the Raspberry Pi Pico SDK folder, and it is best to copy a shortcut in a more convenient place. Then the *startup folder* property of the shortcut can be changed to for example **$PICO**.   

In the *DCP* window, chdir to the **build** folder and execute: **cmake -G "Ninja" ..**   (do not forget the trailing dots, it points to the folder containing **CMakeLists.txt**).  

Now you have initialized the make environment and by executing **Ninja** in that same **build** folder, all SDK libraries and finally the Pi Pico loadable file **uSDR.uf2** will be created.  
*Note that when environment errors are encountered, it may help to empty the build folder and re-issue the cmake command.*   
Rebooting the Pico while the bootsel button is pressed will open a Windows Explorer window with the Pico shown as a Mass Storage Device (e.g. drive E:). Moving **uSDR.uf2** to the Pico is as easy as dragging and dropping this file into that MSD.  
  
## Releases  
Stable packages are archived in zip files. The source files in the root folder are newest and could be used to replace files from the zip archive. There are pre-built UF2 files for three display types, which could be tried. However, there are too many differnt types and addresses, so it is better to build a fresh one for your own implementation.   
The PCB files have been made with Eagle 5.11, and can be modified or otherwise re-used when needed. The CAM files for each board are packaged in separate zips, these can be used as-is to order PCBs.  

# Background
The folder **$PICO/docs** also contains some manuals, of which the *C-SDK description*, the *RP2040 datasheet* and the *Pico Pinout* are absolute must-reads when you start writing software. Note that this folder is only created by the **ndabas** script, after manual installation you should find these on the Raspberry website.  
For calculating filters I have used the free software from [Iowa Hills](http://www.iowahills.com/8DownloadPage.html) (website has been down for a while, but files can be found using [Wayback Machine]( https://web.archive.org/web/20210819042054/http://www.iowahills.com/8DownloadPage.html))  
I also used the online FIR filter calculator [T-Filter](http://t-filter.engineerjs.com/) 

# Copyright notice
**The code and electronic designs as well as the implementations presented in this repository can be copied and modified freely, for non-commercial use.
Use for commercial purposes is allowed as well, as long as a reference to this repository is included in the product.**

See also my [Wave Form Generator](https://github.com/ArjanteMarvelde/uWFG-Pico) project. 
