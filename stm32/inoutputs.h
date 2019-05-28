/*
  inoutputs.h - Header file for shared definitions, variables, and functions
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

#ifndef INOUTPUTS_H_
#define INOUTPUTS_H_

// Initialize the IO module
void inoutputs_init();

// ALL digital outputs
void outputs_digital_reset();
void outputs_digital_set();
// Returns output digital state as a bit-wise uint8 variable.
uint8_t outputs_get_digital_state();
void outputs_set_digital(uint8_t bit_index, uint8_t OnOff );
void outputs_digital_action (uint8_t bit_index, uint8_t Action);

// ALL analog outputs
void outputs_analog_init();
void outputs_analog_set(uint16_t pwmvalue);
void outputs_set_analog(uint8_t channel, uint16_t pwmvalue);
uint16_t outputs_compute_pwm_value(float Val);
void outputs_analog_action (uint8_t Echannel, float *pQval);

//-- ALL Digital Inputs
void inputs_digital_init();
void wait_on_input_action (uint8_t bit_index, uint8_t Mode,float *pTimeoutS);


#endif /* INOUTPUTS_H_ */
