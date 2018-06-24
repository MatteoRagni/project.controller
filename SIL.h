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
#ifndef SIL_H_
#define SIL_H_

/** \brief Software in the loop class
 *
 * This class implements a software in the loop simulation
 * to test the implemented software. The SIL overrides the sensor readings and
 * overrides the actution output, printing in serial all the information regarding
 * the system. Thus SerialParser shall not be included when using Software in the loop
 * simulation.
 */

#include "Config.h"

class SIL {
  float t[3] = {21.0, 21.0, 21.0};
  float p[2] = {0.0, 0.0};

  float u_chil;
  float u_res;
  float u_pres;

  const float dt = float(LOOP_TIMING) / 1000.0;

  const float k12 = 0.5;
  const float k13 = 0.3;
  const float k14 = 0.2;
  const float k24 = 0.2;
  const float k34 = 0.2;
  const float kA = 0.8;
  const float kB = 0.6;
  const float tA = 100.0;
  const float tB = 10.0;
  const float t4 = 21.0;
  const float k1 = -5.0;
  const float k2 = -0.5;
  const float k3 = -1.0;
  const float k4 = (-k2 - k3);

  void sim_temperature() {
    t[0] += dt * (-(k12 + k13 + k14) * t[0] + k12 * t[1] + k13 * t[2] + k14 * t4);
    t[1] += dt * (-(k12 + k24) * t[1] + k12 * t[0] + k24 * t4 + kA * tA * u_res);
    t[2] += dt * (-(k13 + k34) * t[2] + k13 * t[0] + k34 * t4 + kB * tB * u_chil);
  };

  void sim_pressure() {
    p[0] += dt * (k1 * p[0] - k1 * p[1]);
    p[1] += dt * (k2 * p[1] + k3 * p[2] + k4 * u_pres);
  };

 public:
  SIL(){};
  void run() {
    sim_temperature();
    sim_pressure();
    // Serial.print(t[0]);
    // Serial.print(",");
    // Serial.print(p[0]);
    // Serial.print(",");
    // Serial.println(p[1]);
    // Serial.flush();
  };

  int read_temp() { return round(map(t[0], 0.0, 120.0, 1.0, 1024.0)); };
  int read_p_act() { return round(map(p[1], 0.0, 50.0, 1.0, 1024.0)); };
  int read_p_acc() { return round(map(p[0], 0.0, 50.0, 1.0, 1024.0)); };

  void write_chiller(bool c) { u_chil = (c ? 1.0 : 0.0); };
  void write_resistance(bool c) { u_res = (c ? 1.0 : 0.0); };
  void write_pressure(float c) { u_pres = constrain(c, 0.0, 1024.0); };
};

SIL sil_sim;

#endif