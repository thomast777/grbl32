/*
  stm32_pin_out.h - Header file for shared definitions, variables, and functions
  Part of Grbl32

  Copyright (c) 2018-2019 Thomas Truong

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STM32_Pin_OUT_H_
#define STM32_Pin_OUT_H_


#ifdef STM32F13

  //-- Step Dir Limit ---------------------------------------------------------
  #define LIM_GPIO_Port GPIOB
  #define LIM_MASK        (LIM_X_Pin | LIM_Y_Pin | LIM_Z_Pin) // All limit pins
  #define DIR_GPIO_Port GPIOA
  #define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin) // All direction pins
  #define STEP_GPIO_Port GPIOA
  #define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin) // All step pins
  #define AUX_GPIO_Port GPIOA
  #define AUX_MASK        (AUX_1_Pin | AUX_2_Pin | AUX_3_Pin | AUX_4_Pin) // All aux pins

  /*
  #define SetStepperDisableBit() GPIO_SetBits(STEP_ENABLE_GPIO_Port,LL_GPIO_Pin_15)
  #define ResetStepperDisableBit() GPIO_ResetBits(STEP_ENABLE_GPIO_Port,LL_GPIO_Pin_15)
  */
  #define SetStepperDisableBit() GPIO_SetBits(STEP_ENABLE_GPIO_Port,STEP_ENABLE_Pin)
  #define ResetStepperDisableBit() GPIO_ResetBits(STEP_ENABLE_GPIO_Port,STEP_ENABLE_Pin)

  #define STEP_SET_TIMER    TIM2        //-- Set Timer : Step pulse START - typically rising
  #define STEP_SET_IRQ      TIM2_IRQn
  #define STEP_RESET_TIMER  TIM3        //-- Reset Timer : Step pulse END - typically falling
  #define STEP_RESET_IRQ    TIM3_IRQn

  #define Step_Set_EnableIRQ()        NVIC_EnableIRQ(STEP_SET_IRQ)
  #define Step_Reset_EnableIRQ()      NVIC_EnableIRQ(STEP_RESET_IRQ)
  #define Step_Set_DisableIRQ()       NVIC_DisableIRQ(STEP_SET_IRQ)
  #define Step_Reset_DisableIRQ()     NVIC_DisableIRQ(STEP_RESET_IRQ)

  #define Step_Set_Enable()           { LL_TIM_EnableIT_UPDATE(STEP_SET_TIMER); LL_TIM_EnableCounter(STEP_SET_TIMER); }
  #define Step_Reset_Enable()         { LL_TIM_EnableIT_UPDATE(STEP_RESET_TIMER); LL_TIM_EnableCounter(STEP_RESET_TIMER); }

  #define CON_GPIO_Port GPIOB

  #define OUTPUTS_PWM_FREQUENCY       10000
  #define OUTPUTS_PWM_MAX_VALUE      (1000000 / OUTPUTS_PWM_FREQUENCY)

  //-- Spindle/Laser PWM -------------------------------------------------------
  /* For maximum resolution (Counter_Period), we will use the full timer clock, Pre_Scaler(PSC) will be 0.
   * For the STM32F1, the timer clock will be 72MHz
   * using PWM_FREQUENCY = Timer_Clock / (Pre_Scaler+1) * (Counter_Period+1)
   * for TIM2;    PWM_FREQUENCY = 72000000 / (PSC+1) * (ARR+1)
   * for GRBL, a PWM_FREQUENCY of 10KHz is desirable for a Laser Engraver
   * then           10000 = 72000000 / (ARR+1)
   * makes            ARR = (72000000/10000) - 1
   *                  ARR = 7199
   *                          a little less than 13bit resolution (8192 steps to represent 0 to 5V)
   *                          for a 5000 RPM range spindle, the resolution is about 0.7 RPM
   */
  #define SPINDLE_PWM_MAX_VALUE       7199
  #define SPINDLE_PWM_MIN_VALUE       1   // Must be greater than zero.
  #define SPINDLE_PWM_OFF_VALUE       0
  #define SPINDLE_PWM_RANGE           (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
  #define SetSpindleEnablebit()       GPIO_SetBits(SPIN_EN_GPIO_Port, SPIN_EN_Pin)
  #define ResetSpindleEnablebit()     GPIO_ResetBits(SPIN_EN_GPIO_Port, SPIN_EN_Pin)
  #define SetSpindleDirectionBit()    GPIO_SetBits(SPIN_DIR_GPIO_Port, SPIN_DIR_Pin)
  #define ResetSpindleDirectionBit()  GPIO_ResetBits(SPIN_DIR_GPIO_Port, SPIN_DIR_Pin)

  #define SPINDLE_TIMER     TIM1
  #define SPINDLE_CHANNEL   LL_TIM_CHANNEL_CH1
  #define Spindle_Timer_Init()        { LL_TIM_CC_EnableChannel(SPINDLE_TIMER,SPINDLE_CHANNEL); LL_TIM_DisableAllOutputs(SPINDLE_TIMER); LL_TIM_EnableCounter(SPINDLE_TIMER); } //-- start timer with PWM disabled

//  #define Spindle_Disable()           LL_TIM_DisableAllOutputs(SPINDLE_TIMER)
//  #define Spindle_Enable()            LL_TIM_EnableAllOutputs(SPINDLE_TIMER)
  #define Set_Spindle_Speed(pwmVal)   LL_TIM_OC_SetCompareCH1(SPINDLE_TIMER,pwmVal)


  void Analog_Timer_Init(); //-- does nothing in STM32F1



#endif


#ifdef STM32F16

	//-- Step Dir Limit ---------------------------------------------------------
	#define LIM_GPIO_Port GPIOB
  #define DIR_GPIO_Port GPIOA
  #define STEP_GPIO_Port GPIOA
  #define AUX_GPIO_Port GPIOB
  #define AUX_MASK        (AUX_1_Pin) // All aux pins

	#ifdef STM32F1_3
		#define LIM_MASK        (LIM_X_Pin | LIM_Y_Pin | LIM_Z_Pin) // All limit pins
		#define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin) // All direction pins
		#define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin) // All step pins
	#endif
  #ifdef STM32F1_4
    #define LIM_MASK        (LIM_X_Pin | LIM_Y_Pin | LIM_Z_Pin | LIM_A_Pin) // All limit pins
    #define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin | DIR_A_Pin) // All direction pins
    #define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin | STEP_A_Pin) // All step pins
  #endif
  #ifdef STM32F1_5
    #define LIM_MASK        (LIM_X_Pin | LIM_Y_Pin | LIM_Z_Pin | LIM_A_Pin | LIM_B_Pin) // All limit pins
    #define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin | DIR_A_Pin | DIR_B_Pin) // All direction pins
    #define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin | STEP_A_Pin | STEP_B_Pin) // All step pins
  #endif
  #ifdef STM32F1_6
    #define LIM_MASK        (LIM_X_Pin | LIM_Y_Pin | LIM_Z_Pin | LIM_A_Pin | LIM_B_Pin | LIM_C_Pin ) // All limit pins
    #define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin | DIR_A_Pin | DIR_B_Pin | DIR_C_Pin ) // All direction pins
    #define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin | STEP_A_Pin | STEP_B_Pin | STEP_C_Pin ) // All step pins
  #endif


	/*
	#define SetStepperDisableBit() GPIO_SetBits(STEP_ENABLE_GPIO_Port,LL_GPIO_Pin_15)
	#define ResetStepperDisableBit() GPIO_ResetBits(STEP_ENABLE_GPIO_Port,LL_GPIO_Pin_15)
	*/
	#define SetStepperDisableBit() GPIO_SetBits(STEP_ENABLE_GPIO_Port,STEP_ENABLE_Pin)
	#define ResetStepperDisableBit() GPIO_ResetBits(STEP_ENABLE_GPIO_Port,STEP_ENABLE_Pin)

	#define STEP_SET_TIMER 		TIM2				//-- Set Timer : Step pulse START - typically rising
	#define STEP_SET_IRQ			TIM2_IRQn
	#define STEP_RESET_TIMER	TIM3				//-- Reset Timer : Step pulse END - typically falling
	#define STEP_RESET_IRQ		TIM3_IRQn

	#define Step_Set_EnableIRQ() 				NVIC_EnableIRQ(STEP_SET_IRQ)
	#define Step_Reset_EnableIRQ() 			NVIC_EnableIRQ(STEP_RESET_IRQ)
	#define Step_Set_DisableIRQ() 			NVIC_DisableIRQ(STEP_SET_IRQ)
	#define Step_Reset_DisableIRQ() 		NVIC_DisableIRQ(STEP_RESET_IRQ)

	#define Step_Set_Enable()						{	LL_TIM_EnableIT_UPDATE(STEP_SET_TIMER); LL_TIM_EnableCounter(STEP_SET_TIMER); }
	#define Step_Reset_Enable()					{ LL_TIM_EnableIT_UPDATE(STEP_RESET_TIMER); LL_TIM_EnableCounter(STEP_RESET_TIMER); }

	#define CON_GPIO_Port GPIOB

	#define OUTPUTS_PWM_FREQUENCY       10000
	#define OUTPUTS_PWM_MAX_VALUE      (1000000 / OUTPUTS_PWM_FREQUENCY)

	//-- Spindle/Laser PWM -------------------------------------------------------
	/* For maximum resolution (Counter_Period), we will use the full timer clock, Pre_Scaler(PSC) will be 0.
	 * For the STM32F1, the timer clock will be 72MHz
	 * using PWM_FREQUENCY = Timer_Clock / (Pre_Scaler+1) * (Counter_Period+1)
	 * for TIM2;   	PWM_FREQUENCY = 72000000 / (PSC+1) * (ARR+1)
	 * for GRBL, a PWM_FREQUENCY of 10KHz is desirable for a Laser Engraver
	 * then						10000 = 72000000 / (ARR+1)
	 * makes						ARR = (84000000/10000) - 1
	 * 									ARR = 7199
	 * 													a little less than 13bit resolution (8192 steps to represent 0 to 5V)
	 * 													for a 5000 RPM range spindle, the resolution is about 0.7 RPM
	 */
	#define SPINDLE_PWM_MAX_VALUE       7199
	#define SPINDLE_PWM_MIN_VALUE   		1   // Must be greater than zero.
	#define SPINDLE_PWM_OFF_VALUE     	0
	#define SPINDLE_PWM_RANGE         	(SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
	#define SetSpindleEnablebit()       GPIO_SetBits(SPIN_EN_GPIO_Port, SPIN_EN_Pin)
	#define ResetSpindleEnablebit()     GPIO_ResetBits(SPIN_EN_GPIO_Port, SPIN_EN_Pin)
	#define SetSpindleDirectionBit()    GPIO_SetBits(SPIN_DIR_GPIO_Port, SPIN_DIR_Pin)
	#define ResetSpindleDirectionBit()  GPIO_ResetBits(SPIN_DIR_GPIO_Port, SPIN_DIR_Pin)

	#define SPINDLE_TIMER 		TIM1
	#define SPINDLE_CHANNEL		LL_TIM_CHANNEL_CH1
	#define Spindle_Timer_Init()				{	LL_TIM_CC_EnableChannel(SPINDLE_TIMER,SPINDLE_CHANNEL); LL_TIM_DisableAllOutputs(SPINDLE_TIMER); LL_TIM_EnableCounter(SPINDLE_TIMER); } //-- start timer with PWM disabled

	#define Spindle_Disable()						LL_TIM_DisableAllOutputs(SPINDLE_TIMER)
	#define Spindle_Enable()						LL_TIM_EnableAllOutputs(SPINDLE_TIMER)
	#define Set_Spindle_Speed(pwmVal)   LL_TIM_OC_SetCompareCH1(SPINDLE_TIMER,pwmVal)


	void Analog_Timer_Init();	//-- does nothing in STM32F1
#endif


#ifdef STM32F46

	//-- Step Dir  ---------------------------------------------------------
	#define LIM_GPIO_Port GPIOB
	//#define LIM_MASK  ( INT_LIMITS_Pin | INT_HOMES_Pin | INT_EXPIO_Pin )		//-- IOexpander interrupts
//	#define LIM_MASK  ( INT_LIMITS_Pin | INT_HOMES_Pin)		//-- IOexpander interrupts
	#define LIM_MASK  ( INT_LIMITS_Pin )		//-- IOexpander interrupts
  #define DIR_GPIO_Port GPIOA
  #define STEP_GPIO_Port GPIOA
  #define AUX_GPIO_Port GPIOD
  #define AUX_MASK        (AUX_1_Pin | AUX_2_Pin | AUX_3_Pin | AUX_4_Pin | AUX_5_Pin | AUX_6_Pin | AUX_7_Pin | AUX_8_Pin) // All aux pins

#ifdef STM32F4_3
  #define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin) // All direction pins
  #define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin) // All step pins
#endif
#ifdef STM32F4_4
  #define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin | DIR_A_Pin ) // All direction pins
  #define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin | STEP_A_Pin ) // All step pins
#endif
#ifdef STM32F4_5
  #define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin | DIR_A_Pin | DIR_B_Pin ) // All direction pins
  #define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin | STEP_A_Pin | STEP_B_Pin ) // All step pins
#endif
#ifdef STM32F4_6
  #define DIR_MASK        (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin | DIR_A_Pin | DIR_B_Pin | DIR_C_Pin ) // All direction pins
  #define STEP_MASK       (STEP_X_Pin | STEP_Y_Pin | STEP_Z_Pin | STEP_A_Pin | STEP_B_Pin | STEP_C_Pin ) // All step pins
#endif


	#define SetStepperDisableBit() GPIO_SetBits(STEP_ENABLE_GPIO_Port,STEP_ENABLE_Pin)
	#define ResetStepperDisableBit() GPIO_ResetBits(STEP_ENABLE_GPIO_Port,STEP_ENABLE_Pin)

	#define STEP_SET_TIMER 		TIM5				//-- Set Timer : Step pulse START - typically rising
	#define STEP_SET_IRQ			TIM5_IRQn
	#define STEP_RESET_TIMER	TIM7				//-- Reset Timer : Step pulse END - typically falling
	#define STEP_RESET_IRQ		TIM7_IRQn

/*
#define STEP_SET_TIMER 		TIM9				//-- Set Timer : Step pulse START - typically rising
#define STEP_SET_IRQ			TIM1_BRK_TIM9_IRQn
#define STEP_RESET_TIMER	TIM10				//-- Reset Timer : Step pulse END - typically falling
#define STEP_RESET_IRQ		TIM1_UP_TIM10_IRQn
*/

	#define Step_Set_EnableIRQ() 				NVIC_EnableIRQ(STEP_SET_IRQ)
	#define Step_Reset_EnableIRQ() 			NVIC_EnableIRQ(STEP_RESET_IRQ)
	#define Step_Set_DisableIRQ() 			NVIC_DisableIRQ(STEP_SET_IRQ)
	#define Step_Reset_DisableIRQ() 		NVIC_DisableIRQ(STEP_RESET_IRQ)

	#define Step_Set_Enable()						{	LL_TIM_EnableIT_UPDATE(STEP_SET_TIMER); LL_TIM_EnableCounter(STEP_SET_TIMER); }
	#define Step_Reset_Enable()					{ LL_TIM_EnableIT_UPDATE(STEP_RESET_TIMER); LL_TIM_EnableCounter(STEP_RESET_TIMER); }


	#define CON_GPIO_Port GPIOB


	#define OUTPUTS_PWM_FREQUENCY       10000
	#define OUTPUTS_PWM_MAX_VALUE      (1000000 / OUTPUTS_PWM_FREQUENCY)

	//-- Spindle/Laser PWM -------------------------------------------------------
	/* For maximum resolution (Counter_Period), we will use the full timer clock, Pre_Scaler(PSC) will be 0.
	 * For the STM32F4, the timer clock will be 168MHz for TIM1,8,9,10,11 and 84MHz for TIM2,3,4,5,6,7,12,13,14
	 * using PWM_FREQUENCY = Timer_Clock / (Pre_Scaler+1) * (Counter_Period+1)
	 * for TIM2;   	PWM_FREQUENCY = 84000000 / (PSC+1) * (ARR+1)
	 * for GRBL, a PWM_FREQUENCY of 10KHz is desirable for a Laser Engraver
	 * then						10000 = 84000000 / (ARR+1)
	 * makes						ARR = (84000000/10000) - 1
	 * 									ARR = 8399
	 * 													a little better than 13bit resolution (8192 steps to represent 0 to 5V)
	 * 													for a 5000 RPM range spindle, the resolution is about 0.6 RPM
	 * similarly for TIM1 with a 168MHz clock
	 * 									ARR = 16799
	 * 													better than 14bit resolution (16384 steps to represent 0 to 5V)
	 * 													for a 5000 RPM range spindle, the resolution is about 0.3 RPM
	 */
	#define SPINDLE_PWM_MAX_VALUE       8399
	#define SPINDLE_PWM_MIN_VALUE   		1   // Must be greater than zero.
	#define SPINDLE_PWM_OFF_VALUE     	0
	#define SPINDLE_PWM_RANGE         	(SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
	#define SetSpindleEnablebit()       GPIO_SetBits(SPIN_EN_GPIO_Port, SPIN_EN_Pin)
	#define ResetSpindleEnablebit()     GPIO_ResetBits(SPIN_EN_GPIO_Port, SPIN_EN_Pin)
	#define SetSpindleDirectionBit()    GPIO_SetBits(SPIN_DIR_GPIO_Port, SPIN_DIR_Pin)
	#define ResetSpindleDirectionBit()  GPIO_ResetBits(SPIN_DIR_GPIO_Port, SPIN_DIR_Pin)

	#define SPINDLE_TIMER 		TIM2
	#define SPINDLE_CHANNEL		LL_TIM_CHANNEL_CH1
	#define Spindle_Timer_Init()				{	LL_TIM_CC_EnableChannel(SPINDLE_TIMER,SPINDLE_CHANNEL); LL_TIM_DisableAllOutputs(SPINDLE_TIMER); LL_TIM_EnableCounter(SPINDLE_TIMER); } //-- start timer with PWM disabled


	//#define Spindle_Disable()						LL_TIM_DisableAllOutputs(SPINDLE_TIMER)
	//#define Spindle_Enable()						LL_TIM_EnableAllOutputs(SPINDLE_TIMER)
	#define Set_Spindle_Speed(pwmVal)   LL_TIM_OC_SetCompareCH1(SPINDLE_TIMER,pwmVal)
	#define Spindle_Disable()						LL_TIM_CC_DisableChannel(SPINDLE_TIMER, SPINDLE_CHANNEL)
	#define Spindle_Enable()						LL_TIM_CC_EnableChannel(SPINDLE_TIMER, SPINDLE_CHANNEL)


	#define ANA1_TIMER	TIM1
	#define ANA2_TIMER	TIM1
	#define ANA3_TIMER	TIM1
	#define ANA4_TIMER	TIM1
	#define ANA5_TIMER	TIM3
	#define ANA6_TIMER	TIM3
	#define ANA7_TIMER	TIM3
	#define ANA8_TIMER	TIM3

	#define ANA1_CHANNEL	LL_TIM_CHANNEL_CH1
	#define ANA2_CHANNEL	LL_TIM_CHANNEL_CH2
	#define ANA3_CHANNEL	LL_TIM_CHANNEL_CH3
	#define ANA4_CHANNEL	LL_TIM_CHANNEL_CH4
	#define ANA5_CHANNEL	LL_TIM_CHANNEL_CH1
	#define ANA6_CHANNEL	LL_TIM_CHANNEL_CH2
	#define ANA7_CHANNEL	LL_TIM_CHANNEL_CH3
	#define ANA8_CHANNEL	LL_TIM_CHANNEL_CH4

	void Analog_Timer_Init();

	/*
	 * io expander pins for limits. 3 sets: Positive, Negative, Home.
	 * There are 2 chips, 4 ports total, both chips share the same SPI controls:  SPI_SCK, SPI_MOSI, SPI_MISO, SPI_SEL0
	 * 		Chip 1 address : 000 (each chip has 3 address pins: A0, A1, A2).
	 * 		  For LimX~C, Pos and Neg only, both port A and B ports (8 bit each)
	 * 		    are configured to have share interrupt line, going to INT_LIMITS
	 * 		Chip 2 address : 001
	 * 		  For LimX~CHome (portA) and IOExtensions (portB).
	 * 		  	PortA is configured to interrupt INT_HOME
	 * 		  	PortB is configured to interrupt INT_EXPIO
	 */

#define LIM_XP_Pin 	0x01			//-- pin for Limit X Positive/Negative, ioExpander0 Port B
#define LIM_XN_Pin 	0x02			//-- ...
#define LIM_YP_Pin 	0x04
#define LIM_YN_Pin 	0x08
#define LIM_ZP_Pin 	0x10
#define LIM_ZN_Pin 	0x20
#define LIM_AP_Pin 	0x40
#define LIM_AN_Pin 	0x80

#define LIM_BP_Pin 	0x80			//-- pin for Limit B Positive/Negative, ioExpander0 Port A
#define LIM_BN_Pin 	0x40			//-- ...
#define LIM_CP_Pin 	0x20
#define LIM_CN_Pin 	0x10

#define LIM_XH_Pin	0x01			//-- PortB: pin for Limit X Home, ioExpander1 Port B
#define LIM_YH_Pin	1x02
#define LIM_ZH_Pin	2x04
#define LIM_AH_Pin	3x08
#define LIM_BH_Pin	4x10
#define LIM_CH_Pin	5x20

#define EXPIO_8_Pin 0x01			//-- PortA: pin for Expansion IO 8, ioExpander1 Port A
#define EXPIO_7_Pin 0x02			//-- ...
#define EXPIO_6_Pin 0x04
#define EXPIO_5_Pin 0x08
#define EXPIO_4_Pin 0x10
#define EXPIO_3_Pin 0x20
#define EXPIO_2_Pin 0x40
#define EXPIO_1_Pin 0x80





#endif




#endif /* STM32_Pin_OUT_H_ */
