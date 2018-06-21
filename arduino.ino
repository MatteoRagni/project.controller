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
#include "Config.h"
#include "Machine.h"
#include "SerialParser.h"
#include "TempControl.h"
#include "PresControl.h"
#include "Storage.h"
#include "Commands.h"

// Globals
unsigned long tic, toc;

MachineState m;

/** \brief Alarm Function: Called when system goes in alarm state
 * 
 * This function is stored in the MachineState and it is called by
 * the various modules when an alarm is raised. It calls a clean condition
 * for each modulus (each modulus has its own alarm function) and then
 * changes the overall state for the system.
 * \param m the MachineState pointer (that is a singleton)
 */
void alarm_fnc(MachineState* m) {
  m->presctrl->alarm();
  m->tempctrl->alarm();
  m->state->state = StateCode::Alarm;
  while (1) { loop(); } // Re-enters in the loop immediately
}


void setup() {
  // The first initialization shall aways be the serial parser
  // since it connects the machine state and the output buffer
  m.serial = new SerialParser(&m);
  m.tempctrl = new TempControl(&m);
  m.presctrl = new PresControl(&m);
  m.storage = new Storage(&m);
  // Configuring the alarm callback
  m.alarm = alarm_fnc;
  m.p_low = DEFAULT_P_LOW;
  m.p_high = DEFAULT_P_HIGH;
  
  m.serial->begin();
  tic = millis();
  toc = tic;
}

void running_loop() {
  m.serial->
}

void loop() {
  toc = millis();
  if (((unsigned long)(toc - tic)) >= LOOP_TIMING) {

  }
}
