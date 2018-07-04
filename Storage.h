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

#ifndef STORAGE_H_
#define STORAGE_H_

#include <EEPROM.h>
#include "Config.h"
#include "Machine.h"

/** \brief Data to store in the EEPROM
 *
 * We store a complete set of data from the EEPROM that may
 * be useful to restore a test.
 */
typedef struct storage_s {
  unsigned long max_cycle; /**< Max cycle configuration */
  float kp;                /**< Proportional gain configuration */
  float ki;                /**< Integral gain configuration */
  float period;            /**< Square wave period configuration */
  float duty_cycle;        /**< Square wave duty cycle configuration */
  float t_set;             /**< Temeperature set point configuration */
  float p_low;             /**< Low pressure configuration for reference */
  float p_high;            /**< High pressure configuration for reference */
} storage_s;

/** \brief The storage class handles read write from the EEPROM
 *
 * The class saves and restores some configuration in the state of the
 * machine. The idea is to store and restore only upon user request.
 * The configuration saved are in \p storage_s.
 */
class Storage {
  int config_addr;             /**< Configuration EEPROM address */
  int cycle_addr;              /**< Cycle number EEPROM address */
  storage_s config;            /**< The configuration buffer struct */
  unsigned long current_cycle; /**< The current cycle number buffer */
  volatile MachineState* m;

 public:
  /** \brief Class constructor initializes the address
   *
   * The address are automatically evaluated.
   * \param _m pointer to the machine state
   */
  Storage(volatile MachineState* _m) : m(_m) {
    config_addr = STORAGE_REF_ADDRESS;
    cycle_addr = config_addr + sizeof(storage_s);
  };

  /** \brief Load configuration from memory */
  void load_config() {
    EEPROM.get(config_addr, config);
    m->max_cycle = config.max_cycle;
    m->state->kp = config.kp;
    m->state->ki = config.ki;
    m->state->period = config.period;
    m->state->duty_cycle = config.duty_cycle;
    m->state->t_set = config.t_set;
    m->p_low = config.p_low;
    m->p_high = config.p_high;
  };

  /** \brief Load cycle number from the memory */
  void load_cycle() {
    EEPROM.get(cycle_addr, current_cycle);
    m->cycle = current_cycle;
  };

  /** \brief Save configuration in memory */
  void save_config() {
    config.max_cycle = m->max_cycle;
    config.kp = m->state->kp;
    config.ki = m->state->ki;
    config.period = m->state->period;
    config.duty_cycle = m->state->duty_cycle;
    config.t_set = m->state->t_set;
    config.p_low = m->p_low;
    config.p_high = m->p_high;
    EEPROM.put(config_addr, config);
  };

  /** \brief Save cycle number in configuration */
  void save_cycle() {
    current_cycle = m->cycle;
    EEPROM.put(cycle_addr, current_cycle);
  };

  /** \brief Check if it has to save the cycles, depending on frequency */
  void run() {
    if (m->cycle % STORAGE_FREQUENCY == 0)
      save_cycle();
  }
};

#endif