# Generated Cmake Pico project file
#
# After changing this file, empty the build folder and execute from there:  
#     cmake -G "Ninja" ..
#

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Users/Arjan/Documents/Pico-new/SDKv1.5.1/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(uSDR-Pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
# uSDR.c	main loop and initialisation of the software
# lcd.c		LCD driver stuff; pay attention, X different HW implementations exist
# si5351.c	The drivers for setting output frequency and phase in the SI5351 chip
# dsp.c		The signal processing stuff, either timedomain or frequency domain
# fix_fft.c	The FFT transformations in fixed point format
# hmi.c		All user interaction, controlling freq, modulation, levels, etc
# monitor.c	A tty shell on a serial interface
# relay.c	Switching for the band filter and attenuator relays
# font?.c	Graphical character fonts
add_executable(uSDR-Pico 
	uSDR.c 
	lcd.c 
	si5351.c 
	dsp.c 
	fix_fft.c 
	hmi.c 
	monitor.c 
	relay.c
	font7spp.c
	font8.c
	font16.c
	font20.c
	font24.c
	fontub.c
	fontubb.c
	fontAR.c
	fontGR.c
	fontSYM32.c
	fontSYM16.c
)

target_compile_options(uSDR-Pico PRIVATE -Wall)
pico_set_program_name(uSDR-Pico "uSDR-Pico")
pico_set_program_version(uSDR-Pico "4.0")

# Pull in our pico_stdlib which aggregates commonly used features
target_link_libraries(uSDR-Pico pico_stdlib)

# Disable uart output, enable usb output
pico_enable_stdio_uart(uSDR-Pico 0)
pico_enable_stdio_usb(uSDR-Pico 1)

# Add any user requested libraries
target_link_libraries(uSDR-Pico
        pico_stdlib
		pico_multicore
		hardware_i2c
		hardware_spi
		hardware_pwm
        hardware_gpio
        hardware_timer
        hardware_clocks
		hardware_pll
		hardware_adc
		hardware_dma
        )
		
# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(uSDR-Pico)

