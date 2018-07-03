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

#ifdef CONTROLLINO_VERSION
#include <Controllino.h>
#include <SPI.h>
#endif

#include "Machine.h"
#include "PresControl.h"
#include "SerialParser.h"
#include "Storage.h"
#include "TempControl.h"
// Software in the loop simulation
#ifdef SIL_SIM
#include "SIL.h"
#endif
#include "Commands.h"

// Globals
unsigned long tic, toc;

volatile MachineState m;

/** \brief Alarm Function: Called when system goes in alarm state
 *
 * This function is stored in the MachineState and it is called by
 * the various modules when an alarm is raised. It calls a clean condition
 * for each modulus (each modulus has its own alarm function) and then
 * changes the overall state for the system.
 * \param m the MachineState pointer (that is a singleton)
 */
void alarm_fnc(MachineState* m) {
#ifdef CONTROLLINO_VERSION
  digitalWrite(EMERGENCY_LED, HIGH);
#endif
  m->presctrl->alarm();
  m->tempctrl->alarm();
  m->state->state = StateCode::Alarm;
  while (1) {
    loop();
  }  // Re-enters in the loop immediately
}

#ifdef EMERGENCYBTN_ENABLE
/** \brief Emergency button Interrupt routine
 *
 * FIXME: I don't have a better strategy for this. The problem is that we are actually
 * entering an interrupt service routine and we never exit from it. I'm not sure it
 * works well, but we go in any way in a safe state. Probably a manual reboot is
 * required after this and it is not a problem.
 */
void emergency_button_isr() { m.alarm(&m); }
#endif

/** \brief State Machine Running loop
 *
 * Normal mode operation.
 */
void running_loop() {
  m.presctrl->enable();
  m.tempctrl->enable();

  m.presctrl->run();
  m.tempctrl->run();
  m.storage->run();
}

void pause_loop() {
  m.presctrl->disable();
  m.tempctrl->disable();
}

void alarm_loop() {
  m.presctrl->disable();
  m.tempctrl->disable();
}

void serial_loop() {
  m.serial->receive();
  if (m.serial->is_ready()) {
    if (m.state->state == StateCode::Alarm) {
      // The only command we accept in serial on alarm is a reboot
      if (m.command->command == CommandCode::SystemReboot)
        cmd_system_reboot(m.command->value, &m);
      // ... or saving current configuration in EEPROM
      if (m.command->command == CommandCode::SaveStorageConfig)
        cmd_save_storage_config(m.command->value, &m);
      if (m.command->command == CommandCode::Hearthbeat)
        cmd_hearthbeat(m.command->value, &m);
    } else {
      // Here we can execute all the commands
      if (m.command->command < CommandCode::CommandCodeSize)
        cmd_ary[m.command->command](m.command->value, &m);
    }
  }
}

//  ___      _
// / __| ___| |_ _  _ _ __
// \__ \/ -_)  _| || | '_ \
// |___/\___|\__|\_,_| .__/
//                   |_|
void setup() {
#ifdef EMERGENCYBTN_ENABLE
  attachInterrupt(EMERGENCYBTN_PIN, emergency_button_isr, EMERGENCYBTN_MODE);
#endif
#ifdef CONTROLLINO_VERSION
  pinMode(STATE_LED0, OUTPUT);
  pinMode(STATE_LED1, OUTPUT);
  pinMode(EMERGENCY_LED, OUTPUT);
  digitalWrite(STATE_LED0, LOW);
  digitalWrite(STATE_LED1, LOW);
  digitalWrite(EMERGENCY_LED, LOW);
#endif
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
#ifdef CONTROLLINO_VERSION
  digitalWrite(STATE_LED0, HIGH);
#endif
  m.serial->begin();
#ifdef CONTROLLINO_VERSION
  digitalWrite(STATE_LED0, LOW);
  digitalWrite(STATE_LED1, HIGH);
#endif
  tic = millis();
  toc = tic;
}

//  _
// | |   ___  ___ _ __
// | |__/ _ \/ _ \ '_ \
// |____\___/\___/ .__/
//               |_|
void loop() {
  toc = millis();
  if (((unsigned long)(toc - tic)) >= LOOP_TIMING) {
#ifdef SIL_SIM
    sil_sim.run();
#endif
    switch (m.state->state) {
      case (StateCode::Alarm):
        alarm_loop();
        break;
      case (StateCode::Pause):
        pause_loop();
        break;
      case (StateCode::Running):
        running_loop();
        break;
    };
    serial_loop();
    tic = toc;
  }
}
