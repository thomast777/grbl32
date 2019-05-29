# Grbl32
***

Expanding the venerable Grbl universe of AVR 328p Arduinos to the STM32 platform.  Now running on the STM32F103 "blue pill" and STM32F407 controllers. Breaking out of the memory and clock constraints of the 328p, Grbl32 supports up to 6-axis with an order of magnitude increase in pulse rate. 
<p align="center">
  <img src="https://raw.githubusercontent.com/thomast777/media/master/TR/grbl32-0215-600x600.JPG">
</p>

### Hightlights:
* Up to 6-axis: XYZ ABC.
* Communication Baud Rate of 921,600. Releases wil still contain 115,200 versions for older software compatibility.
* The STM32F103 [ARM Cortex M3] will clock up to 250 KHz for each axis while under 3-axis coordinated motion,  150 KHz when running 6-axis.
* The STM32F407 [ARM Cortex M4] sports the warp speed of up to 500+KHz for each axis while under 6-axis coordinated motion.

### Development Notes:
* All reasonable attempts were made to keep the Grbl 1.1f source intact. Whenever possible, calls to STM32 hardware are contained in:
  ```
  #ifdef STM32
  ...
  #endif
  ```
* For reference, the older AVR codes are kept in:
  ```
  #ifdef ATMEGA328P
  ...
  #endif
  ```
* ST TrueStudio (Atollic) IDE, with CubeMX for pins/timers/peripherals selection.
* Common /grbl and /stm32 folders for three sub-branches under /Atollic/:
  * F13 : 3-Axis running STM32F103
  * F16 : 6-Axis running STM32F103. Conditional compilation symbols available for compiling 3, 4, 5, 6 axis.
  * F46 : 6-Axis running STM32F407. Conditional compilation symbols available for compiling 3, 4, 5, 6 axis.


### Some additional M codes supported by Grbl32:
* M62/M63 : digital output. Examples:
  * To turn ON output bit 0
    ```
    M62 P0
    ```
  * To turn OFF output bit 1
    ```
    M63 P1
    ```
* M66 : wait on digital input. Examples:
  * Wait for input bit 0 to turn ON (go HIGH), timeout after 10 seconds
    ```
    M66 P0 L3 Q10
    ```
  * Wait for input bit 7 to turn OFF (go LOW), timeout after 12.5 seconds
    ```
    M66 P7 L4 Q12.5
    ```
* M67 : analog output, additional PWM controls. When the PWM timer parameters are configured properly, the following are examples of controlling RC Servo motors:
  * Set PWM Channel 0 (RC Servo 0) to a PPM pulse of 1000 usec (usually half left)
    ```
    M67 E0 Q1000
    ```
  * Set PWM Channel 7 (RC Servo 7) to a PPM pulse of 2500 usec (usually full right)
    ```
    M67 E7 Q2500
    ```
* M100 : custom M-code, in code acceleration scaling to something less than 100%.
  * Change accelleration of X axis to 80% of $120
    ```
    M100 P0 Q0.8
    ```
  * Change acceleration of A axis to 50% of $123
    ```
    M100 P3 Q0.5
    ```
  * Change acceleration of ALL axis back to 100%
    ```
    M100 P255 Q1
    ```