![uSDR-Pico 4](https://github.com/ArjanteMarvelde/uSDR-pico/blob/main/doc/uSDR-Pico-4.jpg)  

The new V4.0 is now available:  
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

The V4.0 schematics are now available in KiCAD format, so open for all to adapt. In the package I included a library with some dedicated definitions. Note that there are some patches needed to make things functional. These are described in the documentation. New software to run on this upgrade is also available as a package. Please update your SDK before building.

The V4.1 SW package is also available. This version only works when the patches on the HW have been made, as described in the [V4.10](https://github.com/ArjanteMarvelde/uSDR-pico/blob/main/doc/uSDR%20-%20v4.10.pdf) documentation. 

Still to be done:   
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
 

## Installing and using the SDK: 
Forget about collecting all the tooling manually in Windows, and avoid going through the hassle of keeping all things in sync. Instead, use Ubuntu on WSL (Windows Subsystem for Linux) and use the command line interface to generate your build. You can still edit the source files in Windows e.g. with Notepad++.  
### Ubuntu on WSL 
Make sure that virtualization is enabled in BIOS  
open Powershell as administrator  
- wsl --install

restart computer  
open Powershell as administrator  
- wsl.exe --list --online  
(to see the list of distro's)  
- wsl.exe --install [Distro]  
(I have used *Ubuntu-24.04*)

exit Powershell  
run ubuntu from the Start menu  
- sudo apt update  
- sudo apt upgrade  
- sudo apt install cmake gcc-arm-none-eabi build-essential libnewlib-arm-none-eabi git  
### Get SDK and other stuff 
run ubuntu from the Start menu  
- cd ~  
- mkdir pico  
- cd pico  
- git clone -b master https://github.com/raspberrypi/pico-sdk.git  
- cd pico-sdk  
- git submodule update --init   
- cd ~/pico  
- git clone -b master https://github.com/raspberrypi/pico-examples.git  
- cd ~  
- nano .bashrc  
add to the end: *export PICO_SDK_PATH=~/pico/pico-sdk*  
(now the *CMakeLists.txt* file no longer needs to set this)  
- save file and exit nano  
- exit  
### Test with blink example  
run ubuntu from the Start menu  
- cd ~/pico/pico-examples  
- mkdir build  
- cd build  
- cmake ..  
- cd blink  
- make
  
### Some hints  
In Ubuntu the windows C:\ is available under **/mnt/c**  
Likewise, the Documents folder: **/mnt/c/Users/<user>/Documents**  
Use "explorer.exe ." to open a windows explorer in the current directory  
  
## Building uSDR-pico:    
In ubuntu, git clone the uSDR-pico code files into **~/pico/**, "cd ~/pico; git clone https://github.com/ArjanteMarvelde/uSDR-pico".  
Create the build folder: **~/pico/uSDR-pico/build**  
Edit **~/pico/uSDR-pico/CMakeLists.txt** to have the correct environment parameter *PICO_SDK_PATH*, but it should also take it from the environment setting. In line with above installation this will be "~/pico/pico-sdk".  
Now create the build environment, "cd ~/pico/uSDR-pico/build; cmake ..".  
*Note* that every time you change something in **CMakeLists.txt** (like adding another source file to the build) you will have to clean the build folder and execute cmake once more: "cd ~/pico/uSDR-pico/build; rm -rf *; cmake ..".   

Now you have initialized the build environment and by executing **make** in the **build** folder, all SDK libraries and finally the Pi Pico loadable file **uSDR.uf2** will be created.  
*Note that when environment errors are encountered, it may help to empty the build folder and re-issue the cmake command.*   
Rebooting the Pico while the bootsel button is pressed will open a Windows Explorer window with the Pico shown as a Mass Storage Device (e.g. drive E:). Moving **uSDR.uf2** from the build to the Pico drive is as easy as dragging and dropping this file in windows explorer.  
  
## Releases  
Stable packages are archived in zip files. The source files in the root folder are newest and could be used to replace files from the zip archive. There are pre-built UF2 files for three display types, which could be tried. However, there are too many differnt types and addresses, so it is better to build a fresh one for your own implementation.   
The PCB files have been made with Eagle 5.11, and can be modified or otherwise re-used when needed. The CAM files for each board are packaged in separate zips, these can be used as-is to order PCBs.  

# Background
The Raspberry website contains the manuals, of which the *C-SDK description*, the *RP2040 datasheet* and the *Pico Pinout* are absolute must-reads when you start writing software.    
For calculating filters I have used the free software from [Iowa Hills](http://www.iowahills.com/8DownloadPage.html) (website has been down for a while, but files can be found using [Wayback Machine]( https://web.archive.org/web/20210819042054/http://www.iowahills.com/8DownloadPage.html))  
I also used the online FIR filter calculator [T-Filter](http://t-filter.engineerjs.com/) 

# Copyright notice
**The code and electronic designs as well as the implementations presented in this repository can be copied and modified freely, for non-commercial use.
Use for commercial purposes is allowed as well, as long as a reference to this repository is included in the product.**

See also my [Wave Form Generator](https://github.com/ArjanteMarvelde/uWFG-Pico) project. 
