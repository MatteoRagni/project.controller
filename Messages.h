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
#ifndef MESSAGES_H_
#define MESSAGES_H_

#ifdef ARDUINO
#define FORCE_PACKED
#else
#define FORCE_PACKED __attribute__((packed))
#endif

/** Input Message Protocol */

typedef struct FORCE_PACKED input_s {
  char command;
  float value;
  char check;
} input_s;

#define input_size (sizeof(input_s) - sizeof(char))
#define input_buffer_size sizeof(input_s)

/** Output Message Protocol */

typedef struct FORCE_PACKED output_s {
  float t_meas;     /**< Temeprature measurements */
  float p_meas;     /**< Actuator Pressure measurements */
  float q_meas;     /**< Accumulator pressure measurements */
  float kp;         /**< Pressure proportional gain */
  float ki;         /**< Pressure Integral gain */
  float t_set;      /**< Temeprature set point */
  float p_set;      /**< Pressure set point for actuator */
  float u_pres;     /**< Current output for pressure control */
  float period;     /**< Square wave generator period */
  float duty_cycle; /**< Duty cycle coefficient */
  float cycle;      /**< Cycles number */
  float max_cycle;  /**< Max cycles Number */
  char config;      /**< Actuators configuration */
  char state;       /**< Current state flag */
  char error;       /**< Last error flag */
  char check;       /**< Communication checksum (accumulated xor) */
} output_s;

#define output_size (sizeof(output_s) - sizeof(char))
#define output_buffer_size sizeof(output_s)

#endif /* MESSAGES_H_ */
