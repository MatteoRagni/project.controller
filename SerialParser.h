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
#ifndef SERIAL_PARSER_H_
#define SERIAL_PARSER_H_

#include "Config.h"
#include "Messages.h"
#include "Machine.h"

typedef union input_u {
  input_s s;
  char b[input_buffer_size];
} input_u;

typedef union output_u {
  output_s s;
  char b[output_buffer_size];
} output_u;

typedef unsigned int size_t; /**< Type definition for size_t as index in arrays */

/**
 * \brief The serial parser state maxchine
 *
 * A simple implementation of a serial parser state machine
 * that is able to receive and send two structs of dat athat
 * are defined in Messages.h.
 *
 * The input is read in a non blocking manner and it is updated as
 * soon as data is available.
 *
 * The output is written all at the same time, BLOCKING the board
 * in the operation, so we have to make sure that the operation:
 *  1. is not performed in an interrupt routine
 *  2. do not send too much data for a single operation
 */
class SerialParser {
 private:
   MachineState * m; /**< Machine state pointer for some operations */
  input_u input;   /**< Input data holder */
  output_u output; /**< Output data holder */

  char input_buf[input_buffer_size]; /**< Placeholder buffer for input data */
  volatile size_t idx;               /**< Current buffer index for receiving */
  bool data_ready;                   /**< A boolean that tells us if data is ready or not */

 public:
  bool error; /** This boolean tells us if data is in error */

  /** \brief Serial Parser constructor
   *
   * The serial parser constructor requires prepares the buffers
   * for the communication initialization. It will initialize the memory
   * to zero just to be sure (we don't really know what the Arduino compiler does);
   * The state structure is initialized to some default value, and check is made
   * consistent.
   * The constructor also connects the machine state and the input struct in order 
   * to eliminate the need of a copy operation.
   * \param _m a pointer to the MachineState
   */
  SerialParser(MachineState * _m) : m(_m), idx(0), data_ready(false), error(false) {
    m->state = &(output.s);
    m->command = &(input.s);

    memset(input.b, 0, input_buffer_size);
    memset(input_buf, 0, input_buffer_size);
    memset(output.b, 0, output_buffer_size);

    output.s.t_meas = DEFAULT_T_MEAS;
    output.s.p_meas = DEFAULT_P_MEAS;
    output.s.q_meas = DEFAULT_Q_MEAS;
    output.s.kp = DEFAULT_KP;
    output.s.ki = DEFAULT_KI;
    output.s.t_set = DEFAULT_T_SET;
    output.s.p_set = DEFAULT_P_SET;
    output.s.u_pres = DEFAULT_U_PRES;
    output.s.period = DEFAULT_PERIOD;
    output.s.duty_cycle = DEFAULT_DUTY_CYCLE;
    output.s.cycle = DEFAULT_CYCLE;
    output.s.max_cycle = DEFAULT_MAX_CYCLE;
    output.s.config = DEFAULT_CONFIG;
    output.s.state = DEFAULT_STATE;
    output.s.check = lcr_check((const char*)output.b, (size_t)output_size);
  };

  /** \brief Begins and syncronize the serial interface
   *
   * The serial interface is initialized at the speed declared in the configuration
   * file. Once the interface is initialized, there is a short delay and then the
   * signature char is waited. This operation is blocking. The baud is specified in the
   * Config.h file.
   */
  inline void begin() {
    SERIAL_PORT.begin(SERIAL_PORT_BAUD);
    delay(100);  // Waiting the port to set up
    char r = 0x00;
    while (r != SERIAL_SYNC_CHAR) {
      if (SERIAL_PORT.available()) {
        r = SERIAL_PORT.read();
      }
    }
    m->state->state = StateCode::Waiting;
  };

  /** \brief Receives one char at the time, executes when a full command is receive
   *
   * The command receives ALL the character from the serial monitor if they are available
   * and once all the char have been written to a temporary buffer, they are copied to
   * the output structure of the parser. This means that if the transmitting endpoint
   * spams too much char on the serial it will block our receiving endpoint.
   *
   * When data is fully received it is immediately copied in the output structure,
   * and the flag data_ready is set to true. That means: if we are sending too much
   * info from the sending endpoint, we are forcing an overwrite. Last command sent
   * is always more important with respect to the previous ones.
   *
   * If checksum is not respected, data is not copied and error is set to true.
   * Error is reset to false at the first good package received.
   * Received data is expected to be little-endian.
   */
  void receive() {
    while (SERIAL_PORT.available()) {
      input_buf[idx++] = SERIAL_PORT.read();
      if (idx >= input_buffer_size) {
        char r_idx = lcr_check((const char*)input_buf, (size_t)input_size);
        // Checking. If data is correct it is available for the user
        if (r_idx == input_buf[input_size]) {
          // Copying receiving buffer in the output struct
          memcpy((void*)input.b, (void*)input_buf, input_buffer_size);
          error = false;
          data_ready = true;
        } else {
          // Raising the error if the checksum is not correct
          m->state->error = ErrorMessage::SerialCheck;
          m->alarm(m);
        }
        idx = 0;
      }
    }
  }

  /** \brief Send output data: it may be blocking
   *
   * Sends on the serial the output structure. The ouptut structure should be less than 64byte
   * to be sure not to full the output buffer of the Arduino. Longer structure can be used
   * but for sure the operation timing will depend on the length of the structure itself.
   * 
   * The operation is made as blocking in order to avoid chenge in the output structure
   * while writing data to serial.
   * 
   * FIXME: Here I'm doing a blocking operation. It is performed only if requested via serial
   * thus it is stil safe, but if the serial disconnects while sending with flush blocking
   * everything, then I may have some problem. With enough timer one should define a timer
   * for maximum transmission time and raise an alert if the threshold time is reached.
   */
  void send() {
    output.s.check = lcr_check((const char*)output.b, (size_t)output_size);
    SERIAL_PORT.print(output.b);
    SERIAL_PORT.flush();
  }

  /** \brief Utility for data ready
   *
   * The user may use this function to know if data is ready. Is so,
   * he HAS TO read it immediately, since the flag is reset once read.
   * \return true if a command has been received
   */
  bool is_ready() {
    if (data_ready) {
      data_ready = false;
      return true;
    }
    return false;
  };

  /** \brief Pointer to constant input command structure */
  input_s* const get_input_ptr() { return &(input.s); };
  /** \brief Pointer to output structure, that must be filled BEFORE sending */
  output_s* get_output_ptr() { return &(output.s); };
  /** \brief Input size with check */
  const size_t get_input_size() { return input_buffer_size; };
  /** \brief Output size with check */
  const size_t get_output_size() { return output_buffer_size; }

 private:
  /** \brief Checksum for transimitted data
   *
   * The checksum is a simple xor checksum, it may be risky because of
   * collision but it is a good tradeoff with speed. The function
   * evaluates the xor-accumulation of the input bytes.
   */
  inline char lcr_check(const char* buf, const size_t count) {
    char sum = 0x00;
    for (size_t i = 0; i < count; i++) {
      sum ^= buf[i];
    }
    return sum;
  }
};

#endif /* SERIAL_PARSER_H_ */
