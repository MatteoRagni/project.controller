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
#ifndef PRES_CONTROL_H_
#define PRES_CONTROL_H_

#include "Config.h"
#include "Machine.h"

#ifdef SIL_SIM
#include "SIL.h"
#endif

/** \brief Square wave generator class
 *
 * This class is actually a counter that is able to generate a square wave output
 * inside two limits. The square wave period are defined upon PERIOD and DUTY CYCLE.
 * The dynamical generator is:
 *
 *     t' = 1
 *     s' = 0
 *
 *     Condition: t ≥ duty_cycle * T and s = 0
 *     t+ = t
 *     s+ = 1 - s
 *
 *     Condition: t ≥ T and s = 1
 *     t+ = 0
 *     s+ = 1 - s
 *
 * And the ouput of the dynamical system is:
 *
 *     y = s y1 + (1 - s) y0
 *
 * But to make things faster, some evaluations will be simplified.
 */
class SquareWaveGenerator {
  float t;              /**< Internal timer for the system */
  char s;               /**< Internal logic state for the system */
  float *T;             /**< pointer to the period value */
  float *dc;            /**< pointer to the duty cycle value */
  float *y0;            /**< pointer to the lower value for the output */
  float *y1;            /**< pointer to the upper value for the output */
  float *y_set;         /**< pointer to set point */
  unsigned long *cycle; /**< Pointer to the cycle counter (for updating) */

  const float delta_t = float(LOOP_TIMING) / 1000.0; /**< Timing for the integrator */

 public:
  /** \brief Class constructor.
   *
   * The reference limits are actually two pointer in such a way if the memory address receives a change
   * everything is updated also here.
   * \param period pointer to the period value
   * \param duty_cycle pointer to the duty cycle value
   * \param _y0 pointer to the lower value for the output
   * \param _y1 pointer to the upper value for the output
   * \param _cycle pointer to the cycle value fr updating
   */
  SquareWaveGenerator(float *period, float *duty_cycle, float *_y0, float *_y1, float *_y_set, unsigned long *_cycle)
      : t(0), s(0), T(period), dc(duty_cycle), y0(_y0), y1(_y1), y_set(_y_set), cycle(_cycle){};

  /** \brief Evaluates the new value for the reference
   *
   * Evaluates the new value for the reference
   * \return the reference current value
   */
  void reference() {
    t += delta_t;

    if ((t >= ((*T) * (*dc))) && (s == 0)) {
      s = 1;
    }
    if ((t >= (*T)) && (s == 1)) {
      s = 0;
      t = 0;
      (*cycle) += 1;  // Increment cycle number
    }

    (*y_set) = (s ? (*y1) : (*y0));
  };
};

/** \brief Controller Alarm counter class
 *
 * The Alarm Control is raised when the difference between the reference
 * pressure and the measured accumulator pressure is too distant for too
 * long from the reference pressure OR if accumulator pressure is over
 * a certain threshold.
 * The class responds to the following dynamical system:
 *
 *     t' = s
 *     s' = 0
 *
 *     Condition: abs(p_act - p_ref) ≥ TH and s != 1
 *     t+ = t
 *     s+ = 1
 *
 *     Condition: abs(p_act - p_ref) ≤ TH and s != 0
 *     t+ = 0
 *     s+ = 0
 *
 *     Condition: t ≥ TH_time
 *     Alarm is raised
 *
 *     Condition: p_act ≥ TH_pres
 *     Alarm is raised
 */
class ControllerAlarm {
  float t;       /**< Internal timer for the system */
  char s;        /**< Internal logic state for the system */
  float *p_meas; /**< pointer to the actuator measured pressure */
  float *p_set;  /**< Pressure set point */

  const float p_th = PRESCTRL_CTRLALARM_PRES_TH;      /**< Pressure threshold */
  const float t_th = PRESCTRL_CTRLALARM_TIME_TH;      /**< Time threshold */
  const float q_th = PRESCTRL_CTRLALARM_OVERPRESS_TH; /**< Over pressure safety threshold */

  const float delta_t = float(LOOP_TIMING) / 1000.0; /**< Timing for the integrator */
 public:
  /** \brief Constructor for the alarm
   *
   * The constructor needs only the pointers to the measured and the set point
   * pressures in order to have all the data always updated.
   * \param _p_meas pointer to the measured pressure value
   * \param _p_set pointer to the pressure set point
   */
  ControllerAlarm(float *_p_meas, float *_p_set) : t(0), s(0), p_meas(_p_meas), p_set(_p_set){};

  /** \brief Evaluates if alarm is present
   *
   * Evaluates the new value for the timer and checks for an eventual error.
   * \return true if an error shall be raised
   */
  bool check_alarm() {
    t = delta_t;
    float dp = abs((*p_meas) - (*p_set));

    if ((dp >= p_th) && (s != 1)) {
      s = 1;
    }
    if ((dp <= p_th) && (s != 0)) {
      s = 0;
      t = 0;
    }

    // alarm conditions;
    if (t >= t_th)
      return true;
    if ((*p_meas) > q_th)
      return true;
    return false;
  };
};

/** \brief Accumulator Alarm counter class
 *
 * The Alarm Accumulator is raised when the distance between the measured
 * accumulator pressure and the measured actuator pressure is above a certain
 * threshold for a defined time.
 * The class responds to the following dynamical system:
 *
 *     t' = s
 *     s' = 0
 *
 *     Condition: abs(p_act - p_acc) ≥ TH and s != 1
 *     t+ = t
 *     s+ = 1
 *
 *     Condition: abs(p_act - p_acc) ≤ TH and s != 0
 *     t+ = 0
 *     s+ = 0
 *
 *     Condition: t ≥ TH_time
 *     Alarm is raised
 */
class AccumulatorAlarm {
  float t;       /**< Internal timer for the system */
  char s;        /**< Internal logic state for the system */
  float *p_meas; /**< pointer to the actuator measured pressure */
  float *p_acc;  /**< pointer to the accumulator pressure */

  const float p_th = PRESCTRL_ACCALARM_PRES_TH; /**< Pressure threshold */
  const float t_th = PRESCTRL_ACCALARM_TIME_TH; /**< Time threshold */

  const float delta_t = float(LOOP_TIMING) / 1000.0; /**< Timing for the integrator */
 public:
  /** \brief Constructor for the alarm
   *
   * The constructor needs only the pointers to the measured and the accumulator
   * pressures in order to have all the data always updated.
   * \param _p_meas pointer to the measured pressure value
   * \param _p_acc pointer to the pressure set point
   */
  AccumulatorAlarm(float *_p_meas, float *_p_acc) : t(0), s(0), p_meas(_p_meas), p_acc(_p_acc){};

  /** \brief Evaluates if alarm is present
   *
   * Evaluates the new value for the timer and checks for an eventual error.
   * \return true if an error shall be raised
   */
  bool check_alarm() {
    t = delta_t;
    float dp = abs((*p_meas) - (*p_acc));

    if ((dp >= p_th) && (s != 1)) {
      s = 1;
    }
    if ((dp <= p_th) && (s != 0)) {
      s = 0;
      t = 0;
    }

    // alarm conditions;
    if (t >= t_th)
      return true;
    return false;
  };
};

/** \brief PI Controller for pressure with saturation
 *
 * The controller implements the following closed loop equation
 *
 *     e_i' = e_p = (p_ref - p_meas)
 *     u = SAT( kp * e_p + ki * e_i )
 *
 * The control gain are variable. The saturation limits are not.
 */
class PIController {
  float e_p; /**< Internal state, proportional error */
  float e_i; /**< Internal state, integrated error */

  float *kp;     /**< Pointer to the proportional gain */
  float *ki;     /**< Pointer to the integral gain */
  float *p_meas; /**< Pointer to the measured pressure */
  float *p_set;  /**< Pointer to the set point pressure */

  const float sat_min = float(PRESCTRL_PI_SATMIN); /**< Saturation lower bound */
  const float sat_max = float(PRESCTRL_PI_SATMAX); /**< Saturation upper bound */

  const float delta_t = float(LOOP_TIMING) / 1000.0; /**< Timing for the integrator */

 public:
  /** \brief Constructor for the controller
   *
   * The contructor takes the pointers to the gain, the measure and the set point and
   * initialize the state to zero.
   * \param _kp pointer to the controller proportional gain
   * \param _ki pointer to the controller integral gain
   * \param _p_meas pointer to the measured pressure
   * \param _p_set pointer to the set poin pressure
   */
  PIController(float *_kp, float *_ki, float *_p_meas, float *_p_set)
      : kp(_kp), ki(_ki), p_meas(_p_meas), p_set(_p_set) {
    e_p = 0.0;
    e_i = 0.0;
  };

  /** \brief Executes one loop for the control
   *
   * The control loop evaluates the proportional error, the integral error and the
   * control action, applying the saturation. Since gains and measures are as pointer
   * any change to the gains is reflected in the algorithm.
   * \return Saturated control action
   */
  float control() {
    e_p = ((*p_set) - (*p_meas));
    e_i += delta_t * e_p;

    return sat((*kp) * e_p + (*ki) * e_i);
  }

 private:
  /** \brief Evaluates saturation for the control
   *
   * The saturation limits the output value between two bounds, sat_min and sat_max. In pseudocode
   *
   *     if u <= sat_min then return sat_min
   *     if u >= sat_max then return sat_max
   *     return u
   *
   * \param u unsaturated input for the plant
   * \return saturated input for the plant
   */
  inline float sat(float u) { return ((u >= sat_max) ? sat_max : (u <= sat_min) ? sat_min : u); };
};

/** \brief Pressure control class
 *
 * The pressure control is a PI controller that requires a reference generator
 * as input. The pressure control follows:
 *
 *     +---- +              +---------+     +-------+
 *     | REF |------>o---+->| PI Sat  |---->| PLANT |------+-----> P actuator
 *     +-----+       ^-  |  +---------+     +-------+      |
 *                   |   |   +------------+                |
 *                   |   +-->| ALARM Ctrl |-->STOP         |
 *                   |       +------------+                |
 *                   +-------------------------------------+
 *                   |
 *                   V       +-------------+
 *      P accumul -->o------>| ALARM Accum |-->STOP
 *                           +-------------+
 *
 *
 * The reference generator must create two set point with a square wave
 * that depends upon the period and the duty cycle.
 * The PI control is saturated. The Alarms are hybrid counters that
 * fire an error if the system is for too long away from the references.
 *
 * The Alarm Control is raised when the difference between the reference
 * pressure and the measured accumulator pressure is too distant for too
 * long from the reference pressure OR if accumulator pressure is over
 * a certain threshold.
 *
 * The Alarm Accumulator is raised when the distance between the measured
 * accumulator pressure and the measured actuator pressure is above a certain
 * threshold for a defined time.
 */
class PresControl {
  MachineState *m;          /**< Pointer to the machine */
  SquareWaveGenerator *ref; /**< Square wave generator structure pointer */
  ControllerAlarm *al_ctrl; /**< Alarm control structure pointer */
  AccumulatorAlarm *al_acc; /**< Accumulator alarm structure pointer */
  PIController *ctrl;       /**< PI controller structure pointer */

  const float sens_act_m = float(PRESCTRL_SENSOR_ACTUATOR_M);    /**< Calibration  for the actuator sensor */
  const float sens_act_q = float(PRESCTRL_SENSOR_ACTUATOR_Q);    /**< Offset for the actuator sensor */
  const float sens_acc_m = float(PRESCTRL_SENSOR_ACCUMULATOR_M); /**< Calibration for the accumulator sensor */
  const float sens_acc_q = float(PRESCTRL_SENSOR_ACCUMULATOR_Q); /**< Offset for the accumulator sensor */
  const float mov_avg_act = float(PRESCTRL_MOVAVG_ACTUATOR);     /**< Moving average coefficient for actuator measure */
  const float mov_avg_acc =
      float(PRESCTRL_MOVAVG_ACCUMULATOR); /**< Moving average coefficient for accumulator measure */

 public:
  /** \brief Constructor for the pressure control. Must run in Setup!
   *
   * The pressure control must run in setup mode, since it needs to setup some pins I/O
   * and initialize some pointers in the internal structures. It has also to initialize the
   * measures for the pressure sensors, and this is done during class initialization.
   * \param m a pointer to the MachineState struct that contains all the elements.
   */
  PresControl(MachineState *_m) : m(_m) {
    ref = new SquareWaveGenerator(&(m->state->period), &(m->state->duty_cycle), &(m->p_high), &(m->p_low),
                                  &(m->state->p_set), &(m->cycle));
    al_ctrl = new ControllerAlarm(&(m->state->p_meas), &(m->state->p_set));
    al_acc = new AccumulatorAlarm(&(m->state->p_meas), &(m->state->q_meas));
    ctrl = new PIController(&(m->state->kp), &(m->state->ki), &(m->state->p_meas), &(m->state->p_set));
    // Initialize input and output pin
    pinMode(PRESCTRL_PACTUATOR_SENSOR_PIN, INPUT);
    pinMode(PRESCTRL_PACCUMULATOR_SENSOR_PIN, INPUT);
    pinMode(PRESCTRL_PACT_OUT_PIN, OUTPUT);
    pinMode(PRESCTRL_ENABLE_PRES_CTRL, OUTPUT);
    disable();

    // Initializing exponential moving average filter with n samples
    float x = 0.0;
    const int n = 8;
    for (size_t i = 0; i < n; i++) {
      x += read_p_sensor();
    }
    x /= n;
    m->state->p_meas = x;

    x = 0.0;
    for (size_t i = 0; i < n; i++) {
      x += read_q_sensor();
    }
    x /= n;
    m->state->q_meas = x;
  };

  /** \brief Enables output for the actuator
   *
   * This function is mandatory in automatic mode in order to make it run
   */
  void enable() { digitalWrite(PRESCTRL_ENABLE_PRES_CTRL, HIGH); }

  /** \brief Disables and cuts-off the actuator output
   *
   * This function must run when we want to cut off the actuator
   */
  void disable() {
    digitalWrite(PRESCTRL_ENABLE_PRES_CTRL, LOW);
    analogWrite(PRESCTRL_PACT_OUT_PIN, 0);
  }

  /** \brief Central control loop for the pressure
   *
   * The function updates all the sensor values using an exponential moving average, then
   * evaluates if there are conditions for rising an alarm due to incontrollability
   * or puntures in the accumulator, checks the new reference value and then evaluates the PI control
   * and sends it to the actuator.
   */
  void run() {
    // First off: Let's check that we have not reached the maximum cycles...
    if (m->cycle == m->max_cycle) {
      m->state->error = ErrorMessage::CycleLimit;
      m->alarm(m);
    }

    // Measuring sensors:
    m->state->p_meas = mov_avg_act * read_p_sensor() + (1 - mov_avg_act) * m->state->p_meas;
    m->state->q_meas = mov_avg_acc * read_q_sensor() + (1 - mov_avg_acc) * m->state->q_meas;

    // Evaluating error conditions:
    if (al_ctrl->check_alarm()) {
      m->state->error = ErrorMessage::PresControlErr;
      m->alarm(m);
    }
    if (al_acc->check_alarm()) {
      m->state->error = ErrorMessage::PresAccumulatorErr;
      m->alarm(m);
    }

    // Evaluating PID Control Action
    ref->reference();
    m->state->u_pres = ctrl->control();
    if (m->state->config & ControlEnabler::PActuator) {
#ifdef SIL_SIM
      sil_sim.write_pressure(m->state->u_pres);
#else
      analogWrite(PRESCTRL_PACT_OUT_PIN, m->state->u_pres);
#endif
    } else {
      disable();
    }
  };

  /** \brief The alarm function puts the controller in an unrecoverable clean state
   *
   * This function puts the overall controller in a clean state that can be recovered only through
   * a reboot of the microcontroller.
   */
  void alarm() {
    disable();
    m->state->p_set = 0;
    m->state->u_pres = 0;
    m->state->config &= (0xF ^ ControlEnabler::PActuator);
  };

 private:
  /** \brief Routine that reads the actuator sensor input */
  float read_p_sensor() {
#ifdef SIL_SIM
    int read = sil_sim.read_p_act();
#else
    int read = analogRead(PRESCTRL_PACTUATOR_SENSOR_PIN);
#endif
    if (read == 0) {
      m->state->error = ErrorMessage::PActuatorSensor;
      m->alarm(m);
    }
    float x = float(read);
    return (sens_act_m * x + sens_act_q);
  };

  /** \brief Routine that reads the actuator sensor input */
  float read_q_sensor() {
#ifdef SIL_SIM
    int read = sil_sim.read_p_acc();
#else
    int read = analogRead(PRESCTRL_PACCUMULATOR_SENSOR_PIN);
#endif
    if (read == 0) {
      m->state->error = ErrorMessage::PAccumulatorSensor;
      m->alarm(m);
    }
    float x = float(read);
    return (sens_acc_m * x + sens_acc_q);
  };
};
#endif /* PRES_CONTROL_H_ */