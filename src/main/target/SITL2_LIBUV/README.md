## SITL2_LIBUV 
SITL (software in the loop) simulator allows you to run betaflight/cleanflight without any hardware.
Currently only tested on Ubuntu 18.04, x86_64, gcc and Windows mingw64 gcc 10.0


## Proposed changes 

* use libuv instead of dyad.c and udplink.c for i/o
* use stdin and/or TCP as rx/gyro/etc input
* use input parameter as time source for scheduler
* use stdout for motor PWM and internal state output
* use blackbox output to file for PIDAnalyzer/PIDToolbox analysis
* add simulated motor RPM data for RPM filters
* add fft support for dynamic filters



UARTx will bind on `tcp://127.0.0.1:576x` when port been open.

`eeprom.bin`, size 8192 Byte, is for config saving.
size can be changed in `src/main/target/SITL/pg.ld` >> `__FLASH_CONFIG_Size`
