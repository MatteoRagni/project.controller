/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright (c) 2018, Matteo Ragni
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by Matteo Ragni.
 * 4. Neither the name of Matteo Ragni nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "Config.h"
#include "Messages.h"
#include "Machine.h"

inline float cmd_saturation(float &bottom, float &value, float &top) {
  return (value >= top ? top : (value <= bottom ? bottom : value));
}

/**
 * The commands are defined in an array of function pointers, all of the same shape.
 * There can be a maximum od 255 commands, all of them must be specified in
 * the command array.
 * An enum indexes our commands, the command itself is a function pointer.
 */

typedef enum CommandCode {
  Hearthbit,                   /**< Command from the screen: it is alive! */
  ManualTemperatureControl,    /**< Overrides (disabling) the temperature control */
  AutomaticTemperatureControl, /**< Enables the pressure control */
  ManualPressureControl,       /**< Overrides (disabling) the pressure control */
  AutomaticPressureControl,    /**< Enables the pressure control */
  TogglePauseCycle,            /**< Pauses / restarts the control system */
  EmergencyStopCycle,          /**< Calls immediately the alarm status */
  ToggleChillerActuation,      /**< Enables chiller actuation */
  ToggleResistanceActuation,   /**< Enables resistance actuator */
  SetTemperature,              /**< Sets temperature setpoint */
  SetPressureHigh,             /**< Sets pressure set point for reference generator */
  SetPressureLow,              /**< Sets pressure set point for reference generator */
  SetPressure,                 /**< Overrides current pressure set point */
  OverridePIControl,           /**< Override PI Control with voltage input */
  SetPIProportionalGain,       /**< Sets PI proportional gain */
  SetPIIntegrativeGain,        /**< Sets PI integrative gain */
  SetMaximumCycleNumber,       /**< Sets target number for cycles */
  SetCurrentCycleNumber,       /**< Override current Cycle number */
  SetReferencePeriod,          /**< Setup reference square wave period */
  SetReferenceDutyCycle,       /**< Setup reference square wave cycle */
  SystemReboot,                /**< Force an immediate system reboot */
  CommandCodeSize              /**< This last one is a size for the array of function pointers */
} CommandCode;

typedef void(*CommandAction)(float value, MachineState *m);

void cmd_hearthbit(float value, MachineState *m) {
  return;
}

void cmd_manual_temperature_control(float value, MachineState *m) {
  m->state->config &= ((0xF ^ ControlEnabler::Chiller) | (0xF ^ ControlEnabler::Resistance));
}

void cmd_automatic_temperature_control(float value, MachineState *m) {
  m->state->config |= (ControlEnabler::Chiller | ControlEnabler::Resistance); 
}

void cmd_manual_pressure_control(float value, MachineState *m) {
  m->state->config &= (0xF ^ ControlEnabler::PActuator);
}

void cmd_automatic_pressure_control(float value, MachineState *m) {
  m->state->config |= ControlEnabler::PActuator;
}

void cmd_toggle_pause_cycle(float value, MachineState *m) {
  if (m->state->state == StateCode::Pause) {
    m->state->state = StateCode::Running;
  } else if (m->state->state == StateCode::Running) {
    m->state->state = StateCode::Pause;
  }
}

void cmd_emergency_stop_cycle(float value, MachineState *m) {
  m->state->error = ErrorMessage::SerialStop;
  m->alarm(m);
}

void cmd_toggle_chiller_actuation(float value, MachineState *m) {
  m->state->config ^= ControlEnabler::Chiller;
}

void cmd_toggle_resistance_actuation(float value, MachineState *m) {
  m->state->config ^= ControlEnabler::Resistance;
}

void cmd_set_temperature(float value, MachineState *m) {
  m->state->t_set = constrain(value, SERIAL_PARSER_TEMP_MIN, SERIAL_PARSER_TEMP_MAX);
}

void cmd_set_pressure_high(float value, MachineState *m) {
  m->p_high = constrain(value, SERIAL_PARSER_PRESSURE_MIN, SERIAL_PARSER_PRESSURE_MAX);
}

void cmd_set_pressure_low(float value, MachineState *m) {
  m->p_low = constrain(value, SERIAL_PARSER_PRESSURE_MIN, SERIAL_PARSER_PRESSURE_MAX);
}

void cmd_set_pressure(float value, MachineState *m) {
  m->state->p_set = constrain(value, SERIAL_PARSER_PRESSURE_MIN, SERIAL_PARSER_PRESSURE_MAX);
}

void cmd_set_override_PI_control(float value, MachineState *m) {
  m->state->u_pres = constrain(value, PRESCTRL_PI_SATMIN, PRESCTRL_PI_SATMAX);
}

void cmd_set_PI_proportional_gain(float value, MachineState *m) {
  m->state->kp = constrain(value, SERIAL_PARSER_KPI_MIN, SERIAL_PARSER_KPI_MAX);
}

void cmd_set_PI_integral_gain(float value, MachineState *m) {
  m->state->ki = constrain(value, SERIAL_PARSER_KPI_MIN, SERIAL_PARSER_KPI_MAX);
}

void cmd_set_maximum_cycle_number(float value, MachineState *m) {
  m->state->max_cycle = (unsigned long)(constrain(value, 0, 1e12));
}

void cmd_set_current_cycle_number(float value, MachineState *m) {
  m->state->cycle = (unsigned long)(constrain(value, 0, 1e12));
}

void cmd_set_reference_period(float value, MachineState *m) {
  m->state->period = constrain(value, SERIAL_PARSER_PERIOD_MIN, SERIAL_PARSER_PERIOD_MAX);
}

void cmd_set_reference_duty_cycle(float value, MachineState *m) {
  m->state->duty_cycle = constrain(value, 0.0, 1.0);
}

void cmd_set_system_reboot() {
  // TODO: To be implemented!
}

// TODO: The order of the function pointers MATTERS!
const CommandAction cmd_ary[CommandCode::CommandCodeSize] = {
  cmd_hearthbit,
  cmd_manual_temperature_control,
  cmd_automatic_temperature_control,
  cmd_manual_pressure_control,
  cmd_automatic_pressure_control,
  cmd_toggle_pause_cycle,
  cmd_emergency_stop_cycle,
  cmd_toggle_chiller_actuation,
  cmd_toggle_resistance_actuation,
  cmd_set_temperature,
  cmd_set_pressure_high,
  cmd_set_pressure_low,
  cmd_set_pressure,
  cmd_set_override_PI_control,
  cmd_set_PI_proportional_gain,
  cmd_set_PI_integral_gain,
  cmd_set_maximum_cycle_number,
  cmd_set_current_cycle_number,
  cmd_set_reference_period,
  cmd_set_reference_duty_cycle,
  cmd_set_system_reboot
};

#endif /* COMMANDS_H_ */