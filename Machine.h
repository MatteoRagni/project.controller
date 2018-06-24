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
#ifndef MACHINE_H_
#define MACHINE_H_

#include "Config.h"
#include "Messages.h"

// Forward Decalrations:
class SerialParser;
class Storage;
class TempControl;
class PresControl;
class Storage;

typedef struct MachineState MachineState;
typedef void (*alarm_handler)(MachineState* m);

typedef enum ControlEnabler { Chiller = 0x1, Resistance = 0x2, PActuator = 0x4 } ControlEnabler;

typedef enum ErrorMessage {
  NoError = 0x0,
  TempSensor = 0x1,
  PActuatorSensor = 0x2,
  PAccumulatorSensor = 0x3,
  SerialCheck = 0x4,
  ContactOpen = 0x5,
  TempControlErr = 0x6,
  PresControlErr = 0x7,
  PresAccumulatorErr = 0x8,
  CycleLimit = 0x9,
  SerialStop = 0xA
} ErrorMessage;

typedef enum StateCode { Alarm = 0x1, Pause = 0x2, Running = 0x4, Waiting = 0x8, SerialSetup = 0xA } StateCode;

struct MachineState {
  output_s* state;         /**< Current output_s that will be sent on request */
  input_s* command;        /**< Last received command to execute */
  alarm_handler alarm;     /**< Callback to the alarm function */
  float p_high;            /**< High level pressure, this is not on output communication */
  float p_low;             /**< Low level pressure, this is not on output communication */
  unsigned long cycle;     /**< Current cycle number, it is not in output for endianess reason */
  unsigned long max_cycle; /**< Maximum cycle number, it is not in output for endianess reason */
  SerialParser* serial;    /**< Pointer to the serial parser */
  TempControl* tempctrl;   /**< Pointer to the temperature control */
  PresControl* presctrl;   /**< Pointer to the pressure control */
  Storage* storage;        /**< Pointer to the Storage controller */
};

#endif /* MACHINE_H_ */