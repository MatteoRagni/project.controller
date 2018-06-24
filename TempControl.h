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
#ifndef TEMP_CONTROL_H_
#define TEMP_CONTROL_H_

#include "Config.h"
#include "Machine.h"

#ifdef SIL_SIM
#include "SIL.h"
#endif

/**
 * \brief Class for the temperature controller
 *
 * The class handles the control for the temperature. It is actually a system in the form:
 *
 *      x' = f(x, c, r)        x : temperature plant state. We have a sensor for it
 *      c' = 0                 c : chiller state
 *      r' = 0                 r : resistance state
 *      t' = s * LOOP_TIMING   t : counter for alarm state
 *      s' = 0                 s : derivative of the timer
 *
 * This dynamic equation may evolve with jumps if those conditions are reached:
 *
 *      Condition: s != -1 and t ≤ Tset - Tdelta
 *      x+ = x
 *      c+ = 0
 *      r+ = 1 * r_enabled
 *      t+ = 0
 *      s+ = -1
 *
 *      Condition: s != 1 and t ≥ Tset + Tdelta
 *      x+ = x
 *      c+ = 1 * c_enabled
 *      r+ = 0
 *      t+ = 0
 *      s+ = 1
 *
 *      Condition: s != 0 and Tset - Tdelta ≤ t ≤ Tset + Tdelta
 *      x+ = x
 *      c+ = 0
 *      r+ = 0
 *      t+ = 0
 *      s+ = 0
 *
 *      Condition abs(t) ≥ alarm_threshold and TEMP_CTRL = 1
 *      An error is raised in the state machine and all the system is shut down
 */
class TempControl {
 private:
  MachineState *m;       /**< Pointer to the machine struct, a data central repository */
  bool chiller_state;    /**< Chiller on/off state */
  bool resistance_state; /**< Resistance on/off state */
  float alarm_timer;     /**< State for the timer accumulator */
  char condition;        /**< Derivative of the timer accumulator */

  const float delta_t = (float(LOOP_TIMING) / 1000.0); /**< Delta time for integration */
  const float movavg_alpha = TEMPCTRL_MOVAVG_ALPHA;    /**< Moving average scaling factor for filtering */
  const float sens_m = TEMPCTRL_SENSOR_M;              /**< Sensor linear coefficient (pre-evaluated) */
  const float sens_q = TEMPCTRL_SENSOR_Q;              /**< Sensor offset coefficient (pre-evaluated) */

 public:
  /** \brief Temperature control constructor: must run in setup!
   *
   * The temperature control setups the pins and intializes with several readings
   * the exponential moving average filter. This means it must run in Setup, and cannot
   * run outside a function or it will not work!
   * \param m a pointer to the MachineState structure, from wich we take and store measurements and references.
   */
  TempControl(MachineState *_m) : m(_m), chiller_state(false), resistance_state(false), alarm_timer(0), condition(0) {
    // Setting up output pin
    pinMode(TEMPCTRL_CHILLER_PIN, OUTPUT);
    pinMode(TEMPCTRL_RESISTANCE_PIN, OUTPUT);
    digitalWrite(TEMPCTRL_CHILLER_PIN, chiller_state);
    digitalWrite(TEMPCTRL_RESISTANCE_PIN, resistance_state);
    // Initializing moving average filter with n samples
    float x = 0.0;
    const int n = 8;
    for (size_t i = 0; i < n; i++) {
      x += read_sensor();
    }
    x /= n;
    m->state->t_meas = x;
  };

  /** \brief Main loop to run the temperature controller
   *
   * The loop evaluates the system introduced in class description and makes the actuation
   * on the output pin if no alarm is raised.
   */
  void run() {
    // Flowing dynamics
    m->state->t_meas = (1 - movavg_alpha) * m->state->t_meas + movavg_alpha * read_sensor();
    alarm_timer += delta_t * float(condition);

    // Condition helpers
    bool t_below = (m->state->t_meas <= (m->state->t_set - TEMPCTRL_OFFSET));
    bool t_above = (m->state->t_meas >= (m->state->t_set + TEMPCTRL_OFFSET));

    // Jump Dynamics
    if ((condition != -1) && t_below) {
      chiller_state = false;
      resistance_state = bool(m->state->config & ControlEnabler::Resistance);
      alarm_timer = 0;
      condition = -1;
    }
    if ((condition != 0) && (!t_below) && (!t_above)) {
      chiller_state = false;
      resistance_state = false;
      alarm_timer = 0;
      condition = 0;
    }
    if ((condition != 1) && t_above) {
      chiller_state = bool(m->state->config & ControlEnabler::Chiller);
      resistance_state = false;
      alarm_timer = 0;
      condition = 1;
    }

    // Alarm Dynamic
    if (abs(alarm_timer) >= TEMPCTRL_LIMIT_TIME_ALARM) {
      m->state->error = ErrorMessage::TempControlErr;
      m->alarm(m);
      return;
    }
    exec();
  };

  /** \brief Enables the temperature control. Is a placeholder
   *
   * For the temeprature control this function is actually a placeholder;
   */
  void enable() {}

  /** \brief Disables and cuts-off the temeprature output
   *
   * This function must run when we want to cut off the temperature system
   * by putting both the pin to low.
   */
  void disable() {
    digitalWrite(TEMPCTRL_CHILLER_PIN, LOW);
    digitalWrite(TEMPCTRL_RESISTANCE_PIN, LOW);
  }

  /** \brief The alarm function puts the controller in an unrecoverable clean state
   *
   * This function puts the overall controller in a clean state that can be recovered only through
   * a reboot of the microcontroller.
   */
  void alarm() {
    m->state->t_set = 0;
    chiller_state = false;
    resistance_state = false;
    exec();

    m->state->config &= (0xF ^ ControlEnabler::Resistance);
    m->state->config &= (0xF ^ ControlEnabler::Chiller);
  };

 private:
  /** \brief Routine that reads the sensor input */
  float read_sensor() {
#ifdef SIL_SIM
    int read = sil_sim.read_temp();
#else
    int read = analogRead(TEMPCTRL_TEMPSENSOR_PIN);
#endif
    if (read == 0) {
      m->state->error = ErrorMessage::TempSensor;
      m->alarm(m);
    }
    float x = float(read);
    return (sens_m * x + sens_q);
  };

  /** \brief Sends in output the command */
  void exec() {
#ifdef SIL_SIM
    sil_sim.write_chiller(chiller_state);
    sil_sim.write_resistance(resistance_state);
#else
    digitalWrite(TEMPCTRL_CHILLER_PIN, chiller_state ? HIGH : LOW);
    digitalWrite(TEMPCTRL_RESISTANCE_PIN, resistance_state ? HIGH : LOW);
#endif
  };
};

#endif /* TEMP_CONTROL_H_ */