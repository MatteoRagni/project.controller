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
#ifndef CONFIG_H_
#define CONFIG_H_

//#define SIL_SIM /**< Enables the Software in the loop simulator for debugging */
#define CONTROLLINO_VERSION /**< Modifications for Controllino Board */
#ifdef CONTROLLINO_VERSION
#define STATE_LED0 CONTROLLINO_D2
#define STATE_LED1 CONTROLLINO_D3
#define EMERGENCY_LED CONTROLLINO_D4
#endif

// TODO Check all the default values!

// Real time loop timing
#define LOOP_TIMING 30 /**< Defines the ms for timing the loop (must be enough for completing the loop) */

// Emergency Button Configuration
#define EMERGENCYBTN_ENABLE /**< Enable Emergency button logic */
#define EMERGENCYBTN_PIN A0 /**< Pin for emergency button, must have Interrupt capabilities */
#define EMERGENCYBTN_MODE \
  FALLING /**< Default mode for emergency button signal: valid are RISING, LOW, CHANGE, FALLING */

// Communication Configurations
#define SERIAL_PORT Serial      /**< Serial port used for communication */
#define SERIAL_PORT_BAUD 115200 /**< Serial port baud speed (bps) */
#define SERIAL_SYNC_CHAR 0x63   /**< Signature for accepting incoming connection */

// Temperature Control Configurations
#define TEMPCTRL_OFFSET 5.0                                  /**< Offset degrees for temperature controller */
#define TEMPCTRL_LIMIT_TIME_ALARM 600.0                      /**< Time accumulator for alarm limit */
#define TEMPCTRL_CHILLER_PIN 30                              /**< Pin for chiller digital out */
#define TEMPCTRL_RESISTANCE_PIN 31                           /**< Pin for resistance digital out */
#define TEMPCTRL_TEMPSENSOR_PIN A12                          /**< Pin for analog temperature sensor */
#define TEMPCTRL_MOVAVG_ALPHA 0.25                           /**< Exponential Moving average trade off */
#define TEMPCTRL_SENSOR_M (80.0 / 773.0)                     /**< Sensor linear coefficient */
#define TEMPCTRL_SENSOR_Q (90.0 - TEMPCTRL_SENSOR_M * 970.0) /**< Sensor offset coefficient */

// Pressure Control Configurations
#define PRESCTRL_DUMPING_GENERATOR                /**< If defined uses SquareWaveGeneratorDump, otherwise uses simple \
                                                     SquareWaveGenerator */
#define PRESCTRL_CTRLALARM_PRES_TH 2.0            /**< Pressure threshold for the Controller Alarm */
#define PRESCTRL_CTRLALARM_TIME_TH 6.0            /**< Time threshold (out-of-bound) for the Controller Alarm */
#define PRESCTRL_CTRLALARM_OVERPRESS_TH 40.0      /**< Over pressure threshold for the Controller Alarm */
#define PRESCTRL_ACCALARM_PRES_TH 2.0             /**< Time threshold (out-of-bound) for the Accumulator Alarm */
#define PRESCTRL_ACCALARM_TIME_TH 6.0             /**< Over pressure threshold for the Accumulator Alarm */
#define PRESCTRL_PI_SATMIN 0                      /**< Minimum value for PI saturation */
#define PRESCTRL_PI_SATMAX 255                    /**< Maximum value for PI saturation */
#define PRESCTRL_PACTUATOR_SENSOR_PIN A13         /**< Pin selection for actuator pressure sensor */
#define PRESCTRL_PACCUMULATOR_SENSOR_PIN A11      /**< Pin selection for accumulator pressure sensors */
#define PRESCTRL_PACT_OUT_PIN 44                  /**< Pin selection for actuator command output */
#define PRESCTRL_ENABLE_PRES_CTRL 11              /**< Pin for enabling PB pressure control */
#define PRESCTRL_ENABLE_PUMP_CTRL 22              /**< Pin for enabling pump */
#define PRESCTRL_SENSOR_ACTUATOR_M (60.0 / 775.8) /**< Sensor linear coefficient */
#define PRESCTRL_SENSOR_ACTUATOR_Q (60.0 - PRESCTRL_SENSOR_ACTUATOR_M * 970.0)      /**< Sensor offset coefficient */
#define PRESCTRL_SENSOR_ACCUMULATOR_M (30.0 / 818.4)                                /**< Sensor linear coefficient */
#define PRESCTRL_SENSOR_ACCUMULATOR_Q (0.0 - PRESCTRL_SENSOR_ACCUMULATOR_M * 102.3) /**< Sensor offset coefficient */
#define PRESCTRL_MOVAVG_ACTUATOR 0.25    /**< Exponential Moving average trade off */
#define PRESCTRL_MOVAVG_ACCUMULATOR 0.25 /**< Exponential Moving average trade off */

// Serial Command parser configurations (=limits in inp ut)
#define SERIAL_PARSER_TEMP_MIN 15.0    /**< Minimum accepted temperature set point in celsius */
#define SERIAL_PARSER_TEMP_MAX 78.0    /**< Maximum accepted temperature set point in celsius */
#define SERIAL_PARSER_PRESSURE_MIN 0.0 /**< Maximum accepted temperature set point in bar */
#define SERIAL_PARSER_PRESSURE_MAX                                                                 \
  PRESCTRL_CTRLALARM_OVERPRESS_TH     /**< Maximum accepted pressureure set point in bar (equal to \
                                         PRESCTRL_CTRLALARM_OVERPRESS_TH)  */
#define SERIAL_PARSER_KPI_MIN -99.0   /**< Minimum value for the PI gain */
#define SERIAL_PARSER_KPI_MAX 99.0    /**< Maximum value for the PI gain */
#define SERIAL_PARSER_PERIOD_MIN 1.0  /**< Minimum value for the square wave generation in secs */
#define SERIAL_PARSER_PERIOD_MAX 30.0 /**< Maximum value for the square wave generation in secs */

// Storage configuration
#define STORAGE_REF_ADDRESS 0 /**< The default storage reference address offset */
#define STORAGE_FREQUENCY \
  1000 /**< Specify when to write current cycle number to EEPROM (1000 means every 1000 cycles) */

// Defaults
#define DEFAULT_T_MEAS 0.0         /**< Default value for measured temeprature at setup */
#define DEFAULT_P_MEAS 0.0         /**< Default value for measured actuator pressure at setup */
#define DEFAULT_Q_MEAS 0.0         /**< Default value for measured accumulator pressure at setup */
#define DEFAULT_KP 27.0            /**< Default value for proportional (PI press. control) gain at setup */
#define DEFAULT_KI 6.0             /**< Default value for integral gain (PI press. control) temeprature at setup */
#define DEFAULT_T_SET 28.0         /**< Default value for temperature set-point (=T_room=T_oil_rested) */
#define DEFAULT_P_SET 24.0         /**< Default value for pressure set point (equal to system pr when pump is on) */
#define DEFAULT_U_PRES 0.0         /**< Default value override for PI press. control output */
#define DEFAULT_PERIOD 10.0        /**< Default value period for square wave */
#define DEFAULT_DUTY_CYCLE 0.3     /**< Default value for duty cycle for square wave */
#define DEFAULT_CYCLE 0            /**< Default value for initial cycle count */
#define DEFAULT_MAX_CYCLE 10000000 /**< Default value for maximum cycle count */
#define DEFAULT_CONFIG 0x0         /**< Default initial configuration (everything off) */
#define DEFAULT_STATE StateCode::SerialSetup /**< Default initial state for the machine */
#define DEFAULT_P_LOW 24.0                   /**< Default low pressure for reference generator */
#define DEFAULT_P_HIGH 32.0                  /**< Default high pressure for reference generator */

#endif /* CONFIG_H_ */
