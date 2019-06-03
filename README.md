# grbl32
***
Please refer to [gnea/grbl](https://github.com/gnea/grbl) for the core GRBL codes. Check the [grbl32 WIKI page](https://github.com/thomast777/grbl32/wiki) for more information 
***

Expanding the venerable GRBL universe of AVR 328p Arduinos to the STM32 platform.  Now running on the STM32F103 "blue pill" and STM32F407 controllers. Breaking out of the memory and clock constraints of the 328p, Grbl32 supports up to 6-axis with an order of magnitude increase in pulse rate. 

  [<img src="https://raw.githubusercontent.com/thomast777/media/master/TR/grbl32-0215-600x600.JPG">](https://shop.tomsrobotics.com/)

### Hightlights:
* Up to 6-axis: XYZ ABC.
* Communication Baud Rate of 921,600. Releases wil still contain 115,200 versions for older software compatibility.
* The STM32F103 [ARM Cortex M3] will output up to 250 KHz for each axis while under 3-axis coordinated motion,  150 KHz when running 6-axis.
* The STM32F407 [ARM Cortex M4] sports the warp speed of up to 500+KHz for each axis while under 6-axis coordinated motion.
