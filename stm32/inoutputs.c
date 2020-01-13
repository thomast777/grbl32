/*
  inoutputs.c
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

 *  M62 and M63 digital output
 *  M66 digital input
 *  M67 analog output
*/


#include "grbl.h"

#ifdef STM32F1
  #ifdef STM32F13
  const PIN_MASK outputs_pin_mask[N_OUTPUTS_DIG] =
    {
    AUX_1_Pin,AUX_2_Pin,AUX_3_Pin,AUX_4_Pin,
    };
  #endif

  #ifdef STM32F16
  const PIN_MASK outputs_pin_mask[N_OUTPUTS_DIG] =
    {
    AUX_1_Pin,
    };
  #endif
#endif

#ifdef STM32F4
static float pwm_analog_gradient; // Precalulated value to speed up rpm to PWM conversions.
const PIN_MASK outputs_pin_mask[N_OUTPUTS_DIG] =
  { AUX_1_Pin, AUX_2_Pin, AUX_3_Pin, AUX_4_Pin, AUX_5_Pin, AUX_6_Pin, AUX_7_Pin, AUX_8_Pin };
TIM_TypeDef * ana_outputs_timer[N_OUTPUTS_ANA] =
  { ANA1_TIMER, ANA2_TIMER, ANA3_TIMER, ANA4_TIMER, ANA5_TIMER, ANA6_TIMER, ANA7_TIMER, ANA8_TIMER };
const uint32_t ana_outputs_channel[N_OUTPUTS_ANA] =
  { ANA1_CHANNEL, ANA2_CHANNEL, ANA3_CHANNEL, ANA4_CHANNEL, ANA5_CHANNEL, ANA6_CHANNEL, ANA7_CHANNEL, ANA8_CHANNEL };
const PIN_MASK inputs_pin_mask[N_INPUTS_DIG] =
  { EXPIO_1_Pin, EXPIO_2_Pin, EXPIO_3_Pin, EXPIO_4_Pin, EXPIO_5_Pin, EXPIO_6_Pin, EXPIO_7_Pin, EXPIO_8_Pin };

#endif

//------------------------------------------------------------------------
void inoutputs_init()
{
  outputs_digital_reset();
  inputs_digital_init();
  // analog init here
  outputs_analog_init();
  outputs_analog_set(0);

}

// Reset outputs
void outputs_digital_reset()
{
  uint8_t i;
  for (i = 0; i < N_OUTPUTS_DIG; i++)
    GPIO_ResetBits(AUX_GPIO_Port, outputs_pin_mask[i]);
}
// Set outputs
void outputs_digital_set()
{
  uint8_t i;
  for (i = 0; i < N_OUTPUTS_DIG; i++)
    GPIO_SetBits(AUX_GPIO_Port, outputs_pin_mask[i]);
}

// Returns output state as a bit-wise uint8 variable.
uint8_t outputs_get_digital_state()
{
  uint8_t outputs_state = 0;
  uint8_t i;
  for (i = 0; i < N_OUTPUTS_DIG; i++)
    {
    if (bit_istrue(GPIO_ReadOutputData(AUX_GPIO_Port), outputs_pin_mask[i]))
      {
      outputs_state |= (1 << i);
      }
    }
  return outputs_state;
}

void outputs_set_digital(uint8_t bit_index, uint8_t OnOff)
{
  if (bit_index < N_OUTPUTS_DIG)
    {
    if (OnOff)
      GPIO_SetBits(AUX_GPIO_Port, outputs_pin_mask[bit_index]);
    else
      GPIO_ResetBits(AUX_GPIO_Port, outputs_pin_mask[bit_index]);
    }
}

void outputs_digital_action(uint8_t bit_index, uint8_t Action)
{
  protocol_buffer_synchronize();
  if (bit_index == 0xFF)
    {
    if (Action == DIGITAL_CONTROL_ON)
      outputs_digital_set();
    else if (Action == DIGITAL_CONTROL_OFF)
      outputs_digital_reset();
    }
  else if (bit_index < N_OUTPUTS_DIG)
    {
    if (Action == DIGITAL_CONTROL_ON)
      outputs_set_digital(bit_index, 1);
    else if (Action == DIGITAL_CONTROL_OFF)
      outputs_set_digital(bit_index, 0);
    }

}

#ifdef STM32F1
void outputs_analog_init()
  {}
void outputs_analog_set(uint16_t value)
  {}	//-- set value to all channels
void outputs_set_analog(uint8_t channel, uint16_t value)
  {}
uint16_t outputs_compute_pwm_value(float Val)
  {return (0);}
void outputs_analog_action (uint8_t Echannel, float *pQval)
  {}
void inputs_digital_init()
  {}
void wait_on_input_action (uint8_t bit_index, uint8_t Mode,float *pTimeoutS)
  {}

#endif //-- STM32F1

#ifdef STM32F4
//--------------------------------------------------------------------------
/*
 * Sequence: Set PWM value, then Enable
 */
// analog outputs ----------------------------------------------------------
void outputs_analog_init()
{
  Analog_Timer_Init();
  pwm_analog_gradient = OUTPUTS_PWM_MAX_VALUE / settings.analog_max;
}
//--------------------------------------------------------------------------
void outputs_analog_set(uint16_t value)	//-- set value to all channels
{
  uint8_t i;
  for (i = 0; i < N_OUTPUTS_ANA; i++)
    {
    outputs_set_analog(i, value);
    }

}
//--------------------------------------------------------------------------
void outputs_set_analog(uint8_t channel, uint16_t value)
{
  TIM_TypeDef * Timer = ana_outputs_timer[channel];
  uint32_t TimerChannel = ana_outputs_channel[channel];

  if (value)
    {
    switch (TimerChannel)
      {
      case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(Timer, value);
        break;
      case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(Timer, value);
        break;
      case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(Timer, value);
        break;
      case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(Timer, value);
        break;
      }

    //if (LL_TIM_CC_IsEnabledChannel(Timer,TimerChannel) == 0)
    LL_TIM_CC_EnableChannel(Timer, TimerChannel);
    }
  else
    {
    LL_TIM_CC_DisableChannel(Timer, TimerChannel);
    }

}
//--------------------------------------------------------------------------
uint16_t outputs_compute_pwm_value(float Val)
{
  uint16_t pwm_value;
  pwm_value = (uint16_t) (Val * pwm_analog_gradient);
  return (pwm_value);
}
//--------------------------------------------------------------------------
void outputs_analog_action(uint8_t Echannel, float *pQval)
{
  uint16_t value = trunc(*pQval);
  protocol_buffer_synchronize();

  if (Echannel == 0xFF)
    {
    outputs_analog_set(value);	//-- set value to all channels

    }
  else if (Echannel < N_OUTPUTS_ANA)
    {
    outputs_set_analog(Echannel, value);

    }
}

//-- digital input section ---------------------------------------------------
void inputs_digital_init()
{
}

void wait_on_input_action(uint8_t bit_index, uint8_t Mode, float *pTimeoutS)
{
  uint32_t Start = HAL_GetTick(); //-- milliseconds
  uint32_t Now;

  uint32_t Elapsed;
  uint32_t TimeoutMS = *pTimeoutS * 1000;
  uint8_t bitAction = (1 << bit_index);
  uint8_t byInput;
  bool StillWaiting = true;

  protocol_buffer_synchronize();
  while (StillWaiting)
    {
    Now = HAL_GetTick();
    Elapsed = Now - Start;
    if ((Elapsed > TimeoutMS) || (sys.abort))
      {
      StillWaiting = false;
      }
    else
      {
      byInput = ReadInputByte();
      if (Mode == 3) // look for HI
        {
        if (bit_istrue(byInput, bitAction))
          StillWaiting = false;
        }
      else if (Mode == 4) // look for LO
        {
        if (bit_isfalse(byInput, bitAction))
          StillWaiting = false;
        }
      }
    protocol_execute_realtime();
    //HAL_Delay(1);
    //delay_ms(1000);
    }

}
#endif //--STM32F4
