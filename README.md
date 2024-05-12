*Note*: I'm working on V4.0 right now, with some difference in hardware:  
- Integrated audio on the CPU board  
- Moved VFO to Mixer board  
- Applied different way to derive FST3253 clocks, no longer dependent on 90 deg phasing of VFO  
- Use discrete MMIC for LNA on RX board  
- New low power TX board  
- New layout BPF board  
- Proper bus connectors for all 5 boards,  
- Increased board size: 2" x 3.7", with the bus connectors this enables a cleaner stack  
- LCD graphical display (ILI9341 based 320x240)  
- No more RS232, replaced with USB interface which can also be used for programming  
- Recalculated all filters  

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
- [ ] write FFT TX function
- [ ] improve on TX audio quality 
- [ ] implement better AGC 
- [ ] improve FFT filtering  
 

## Installing and using the SDK for Windows: 
For setting up the C/C++ build environment for Windows, you can follow the procedure as described in the Raspberry [Getting Started](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) document. This document also refers to a [setup script](https://github.com/ndabas/pico-setup-windows). In case this does not work, follow the instructions below. I have found that the Visual Studio Code interface is hard to set up correctly, so I recommend to just use [Notepad++](https://notepad-plus-plus.org/downloads/) to edit the source and txt files and simply use NMake from the Developer Command Prompt to build a loadable UF2 file.  

### Manual installation.  
Doing it manually, first download the latest packages, in my case for Windows 10 on a 64 bit PC:  
 
[ARM GNU toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/downloads) (choose the file ending on *arm-non-eabi.exe*)  
[CMake](https://cmake.org/download/)  
[VS Build Tools](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2022)  
[Python](https://www.python.org/downloads/windows/) (I wonder, do we actually need this for C/C++ environment?)  
[Git](https://git-scm.com/download/win)  

I use [Notepad++](https://notepad-plus-plus.org/downloads/) as editor for my source files, since I don't like the VS IDE, so I recommend to install this before anything else.  
The installation, step by step, listing the choices I made:  
  
**-1- ARM GNU toolchain**  
Start the installer 
-  Language: English  
-  Next  
-  I Agree  
-  Folder as proposed, Install  
-  Tick the box: "Add path to environment variable", Finish  

Note: If the installer complains, no worry: this will be done below in step **-7-**.
You could also add the installation folder location manually to the system path, through *System Properties* on the PC, click on Environment Variables in the advanced tab. (The variable is PICO_TOOLCHAIN_PATH and the path should look like "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\11.2 2022.02")  
  
**-2- CMake**  
Start the installer  
- Next  
- Accept, Next  
- Tick the box: "Add CMake to path for all users", Next  
- Folder as proposed, Next  
- Install  


**-3- VS Build Tools (Installer)**  
Start the loader/installer  
- Continue  
- Select: "Desktop development with C++", Install 	(this takes a while...)  

Close the window when done
  
**-4- Python**  
Start the installer  
- Tick the boxes: "Add Python ... to PATH" and "Install for all...", Install now  
  
  
**-5- Git**  
Start installer  
- Next  
- Use proposed path, Next  
- Defaults, Next  
- Default Start menu folder, Next  
- Use Notepad++ as default editor, Next  
- Let Git decide, Next  
- Git from the commandline and 3rd party software, Next  
- Use bundled SSH, Next  
- Use the OpenSSL library, Next  
- Checkout as-is, commit as-is, Next  
- Use Windows default console, Next  
- Default, Next  
- Git Credential manager, Next  
- Enable file system caching, Next  
- Enable experimental support for pseudo consoles, Next  
- Finish  
  
  
**-6- Get Pico SDK and examples from Github**  
Open a Windows command prompt, then use it to setup the folder structure (for example, "C:\Users\name\Documents\Pico"):  
```
mkdir <target folder>   
chdir <target folder>  
git clone -b master https://github.com/raspberrypi/pico-sdk.git  
cd pico-sdk  
git submodule update --init  
cd ..  
git clone -b master https://github.com/raspberrypi/pico-examples.git  
```

  
**-7- Setup the build environment**  
Open a Visual Studio Developer Command Prompt from the Start menu  
Define some environment variables manually (these were not set right during installation)  
```
setx PICO_SDK_PATH "<target folder>\pico-sdk"  
setx PICO_TOOLCHAIN_PATH "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\11.2 2022.02"  
```
*Note that the actual ARM toolchain folder may be different: check it first!*  
Close this VS Developer Command Prompt window  
  
  
## Building uSDR-pico:  
Let's call our *target folder* **$PICO** from now on.  
Create the folder **$PICO/uSDR-pico**  
Clone/copy the uSDR-pico code files into **$PICO/uSDR-pico**  
Copy the file **$PICO/pico-sdk/pico_sdk_import.cmake** into this folder too, it contains the global cmake instructions  
Create the build folder: **$PICO/uSDR-pico/build**  
  
Before the first build you need to check and adapt the file **$PICO/uSDR-pico/CMakeLists.txt**, using your favourite editor, to make sure it reflects your own directory structure. Also in this file, select whether you want **stdio** to use the UART on pins 1 and 2 or the USB serial port. The monitor terminal is on **stdio**. This is needed because CMakeLists.txt directs CMake in the construction of your nmake environment. In fact, every time you change something in CMakeLists.txt (like adding another source file to the build) you will have to swipe the build folder and re-issue cmake.   
All building is using the Visual Studio NMake, so it has to be done from a  **VS Developer Command Prompt for Pico** (*DCP*). This is found in the Start menu under VS 2022, and it is best to copy a shortcut in a more convenient place. Then the startup folder property in the shortcut can be changed to for example **$PICO**. Within this *DCP* all environment settings have been properly set to enable the building process.  
In the *DCP* window, chdir to the **build** folder and execute: **cmake -G "NMake Makefiles" ..**   (do not forget the trailing dots, it points to the folder containing CMakeLists.txt).  
Now you have initialized the make environment (for *nmake*) and by executing **nmake** in that same **build** folder, all SDK libraries and finally the Pi Pico loadable file **uSDR.uf2** will be created.  
*Note that when environment errors are encountered, it may help to empty the build folder and re-issue the cmake command.*   
Rebooting the Pico while the bootsel button is pressed will open a Windows Explorer window with the Pico shown as a Mass Storage Device (e.g. drive E:). Moving **uSDR.uf2** to the Pico is as easy as dragging and dropping this file into that MSD.  
  
## Releases:  
Stable packages are archived in zip files. The source files in the root folder are newest and could be used to replace files from the zip archive. There are pre-built UF2 files for three display types, which could be tried. However, there are too many differnt types and addresses, so it is better to build a fresh one for your own implementation.   
The PCB files have been made with Eagle 5.11, and can be modified or otherwise re-used when needed. The CAM files for each board are packaged in separate zips, these can be used as-is to order PCBs.  

# Background
The folder **$PICO/docs** also contains some manuals, of which the *C-SDK description*, the *RP2040 datasheet* and the *Pico Pinout* are absolute must-reads when you start writing software. Note that this folder is only created by the **ndabas** script, after manual installation you should find these on the Raspberry website.  
For calculating filters I have used the free software from [Iowa Hills](http://www.iowahills.com/8DownloadPage.html) (website has been down for a while)  
I also used the online FIR filter calculator [T-Filter](http://t-filter.engineerjs.com/) 

# Copyright notice
**The code and electronic designs as well as the implementations presented in this repository can be copied and modified freely, for non-commercial use.
Use for commercial purposes is allowed as well, as long as a reference to this repository is included in the product.**

See also my [Wave Form Generator](https://github.com/ArjanteMarvelde/uWFG-Pico) project. 
