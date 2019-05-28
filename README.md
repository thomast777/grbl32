# GRBL32
***

Expanding the venerable Grbl universe of AVR 328p Arduinos to the STM32 platform.  Now running on the STM32F103 "blue pill" and STM32F407 controllers. Breaking out of the memory and clock constraints of the 328p, Grbl32 supports up to 6-axis with an order of magnitude increase in pulse rate. 

The STM32F103 [Arm Cortex M3] will clock in at 250 KHz when running the traditional 3-axis,  150 KHz when running 6-axis. The STM32F407 [Arm Cortex M4] sports the warp speed of 500+KHz running 6-axis.

Some additional M codes supported by Grbl32:
* M62/M63 : digital output.
* M66 : wait on digital input.
* M67 : analog output, additional PWM controls.
* M100 : custom M-code, in code acceleration scaling to something less than 100%.