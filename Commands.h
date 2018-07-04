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

#include <avr/wdt.h>
#include "Config.h"
#include "Machine.h"
#include "Messages.h"

#ifdef SIL_SIM
#define SIL_SIM_CONT ,
#else
#define SIL_SIM_CONT };
#endif

/**
 * The commands are defined in an array of function pointers, all of the same shape.
 * There can be a maximum od 255 commands, all of them must be specified in
 * the command array.
 * An enum indexes our commands, the command itself is a function pointer.
 */

typedef enum CommandCode {
  Hearthbeat = 0x00,           /**< Command from the screen: it is alive! */
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
  StartCycle,                  /**< Starts the system (Running state) */
  SaveStorageConfig,           /**< Save current configuration in the EEPROM */
  LoadStorageConfig,           /**< Load storage config from EEPROM. Extremely risky, it may be corrupted data */
  LoadStorageCycle,            /**< Load current cycle number from EEPROM. Extremely Risky it may be corrupted data */
#ifdef SIL_SIM
  SILTempSensFault,
  SILPresActSensFault,
  SILPresAccSensFault,
  SILPresActFault,
  SILTempActFault,
#endif
  CommandCodeSize /**< This last one is a size for the array of function pointers */
} CommandCode;

typedef void (*CommandAction)(float, volatile MachineState *);

void cmd_hearthbeat(float, volatile MachineState *m) { m->serial->send(); }

void cmd_manual_temperature_control(float, volatile MachineState *m) {
  m->state->config &= ((0xFF ^ ControlEnabler::Chiller) | (0xFF ^ ControlEnabler::Resistance));
}

void cmd_automatic_temperature_control(float, volatile MachineState *m) {
  m->state->config |= (ControlEnabler::Chiller | ControlEnabler::Resistance);
}

void cmd_manual_pressure_control(float, volatile MachineState *m) {
  m->state->config &= (0xFF ^ ControlEnabler::PActuator);
}

void cmd_automatic_pressure_control(float, volatile MachineState *m) { m->state->config |= ControlEnabler::PActuator; }

void cmd_toggle_pause_cycle(float, volatile MachineState *m) {
  if (m->state->state == StateCode::Pause) {
    m->state->state = StateCode::Running;
  } else if (m->state->state == StateCode::Running) {
    m->state->state = StateCode::Pause;
  }
}

void cmd_emergency_stop_cycle(float, volatile MachineState *m) {
  m->state->error = ErrorMessage::SerialStop;
  m->alarm(m);
}

void cmd_toggle_chiller_actuation(float, volatile MachineState *m) { m->state->config ^= ControlEnabler::Chiller; }

void cmd_toggle_resistance_actuation(float, volatile MachineState *m) {
  m->state->config ^= ControlEnabler::Resistance;
}

void cmd_set_temperature(float value, volatile MachineState *m) {
  m->state->t_set = constrain(value, SERIAL_PARSER_TEMP_MIN, SERIAL_PARSER_TEMP_MAX);
}

void cmd_set_pressure_high(float value, volatile MachineState *m) {
  m->p_high = constrain(value, SERIAL_PARSER_PRESSURE_MIN, SERIAL_PARSER_PRESSURE_MAX);
}

void cmd_set_pressure_low(float value, volatile MachineState *m) {
  m->p_low = constrain(value, SERIAL_PARSER_PRESSURE_MIN, SERIAL_PARSER_PRESSURE_MAX);
}

void cmd_set_pressure(float value, volatile MachineState *m) {
  m->state->p_set = constrain(value, SERIAL_PARSER_PRESSURE_MIN, SERIAL_PARSER_PRESSURE_MAX);
}

void cmd_set_override_PI_control(float value, volatile MachineState *m) {
  m->state->u_pres = constrain(value, PRESCTRL_PI_SATMIN, PRESCTRL_PI_SATMAX);
}

void cmd_set_PI_proportional_gain(float value, volatile MachineState *m) {
  m->state->kp = constrain(value, SERIAL_PARSER_KPI_MIN, SERIAL_PARSER_KPI_MAX);
}

void cmd_set_PI_integral_gain(float value, volatile MachineState *m) {
  m->state->ki = constrain(value, SERIAL_PARSER_KPI_MIN, SERIAL_PARSER_KPI_MAX);
}

void cmd_set_maximum_cycle_number(float value, volatile MachineState *m) {
  m->max_cycle = (unsigned long)(constrain(value, 0, 1e12));
}

void cmd_set_current_cycle_number(float value, volatile MachineState *m) {
  m->cycle = (unsigned long)(constrain(value, 0, 1e12));
}

void cmd_set_reference_period(float value, volatile MachineState *m) {
  m->state->period = constrain(value, SERIAL_PARSER_PERIOD_MIN, SERIAL_PARSER_PERIOD_MAX);
}

void cmd_set_reference_duty_cycle(float value, volatile MachineState *m) {
  m->state->duty_cycle = constrain(value, 0.0, 1.0);
}

void cmd_system_reboot(float, volatile MachineState *m) {
  m->tempctrl->disable();
  m->presctrl->disable();

  wdt_enable(WDTO_30MS);
  while (1) {
  };
}

void cmd_start_cycle(float, volatile MachineState *m) { m->state->state = StateCode::Running; }

void cmd_save_storage_config(float, volatile MachineState *m) { m->storage->save_config(); }

void cmd_load_storage_config(float, volatile MachineState *m) { m->storage->load_config(); }

void cmd_load_storage_cycle(float, volatile MachineState *m) { m->storage->load_cycle(); }

// TODO: The order of the function pointers MATTERS!
const CommandAction cmd_ary[CommandCode::CommandCodeSize] = {cmd_hearthbeat,
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
                                                             cmd_system_reboot,
                                                             cmd_start_cycle,
                                                             cmd_save_storage_config,
                                                             cmd_load_storage_config,
                                                             cmd_load_storage_cycle SIL_SIM_CONT
#ifdef SIL_SIM
                                                                 cmd_faulty_temperature_sens,
                                                             cmd_faulty_pres_act_sens,
                                                             cmd_faulty_pres_acc_sens,
                                                             cdm_faulty_pres_actuator,
                                                             cdm_faulty_temp_actuator};
#endif

#endif /* COMMANDS_H_ */