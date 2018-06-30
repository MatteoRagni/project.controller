# project.controller - A controller for Fatigue test bench

The repository holds the code of the firmware for a test bench. The bench tests pressure accumulator with respect to fatigue.
The bench performs a sequence of square wave reference between an upper an lower pressure (PI controller) and maintains the 
temperature using a chiller and a resistance.

The firmware is composed by several systems that cooperates in order to control the hardware device:
 
 * a pressure controller (`PresControl.h`)
 * a temperature controller (`TempControl.h`)
 * a storage system (`Storage.h`)
 * a serial interface parser (`SerialParser.h`)

The main `setup` and `loop` operates on a state machine, and are in the file `arduino.ino`, the core file to be compiled and uploaded
to the controller board. The static configuration and defaults are defined in `Config.h`.

## Emergency Button

It is possible to connect an emergency button to the controller. When the button is pressed an interrupt that puts the controller
in the alarm state is called.

The settings for the emergency button are:

 * `EMERGENCYBTN_PIN`: Pin for emergency button, must have Interrupt capabilities
 * `EMERGENCYBTN_MODE`: Default mode for emergency button signal: valid are RISING, LOW, CHANGE, FALLING. 
    Check on Arduino attachInterrupt documentation

## Pressure Controller

The pressure control is a PI controller that requires a reference generator
as input. The pressure control follows:
```
     +---- +              +---------+     +-------+
     | REF |------>o---+->| PI Sat  |---->| PLANT |------+-----> P actuator
     +-----+       ^-  |  +---------+     +-------+      |
                   |   |   +------------+                |
                   |   +-->| ALARM Ctrl |-->STOP         |
                   |       +------------+                |
                   +-------------------------------------+
                   |
                   V       +-------------+
      P accumul -->o------>| ALARM Accum |-->STOP
                           +-------------+
```

The reference generator must create two set point with a square wave
that depends upon the period and the duty cycle.
The PI control is saturated. The Alarms are hybrid counters that
fire an error if the system is for too long away from the references.

The Alarm Control is raised when the difference between the reference
pressure and the measured accumulator pressure is too distant for too
long from the reference pressure OR if accumulator pressure is over
a certain threshold.

The Alarm Accumulator is raised when the distance between the measured
accumulator pressure and the measured actuator pressure is above a certain
threshold for a defined time.

### PI Controller

The controller implements the following closed loop equation
```
e_i' = e_p = (p_ref - p_meas)
u = SAT( kp * e_p + ki * e_i )
```
The control gain are variable. The saturation limits are not.

The saturation limits the output value between two bounds, sat_min and sat_max. In pseudocode
```
if u <= sat_min then return sat_min
if u >= sat_max then return sat_max
return u
```

### Square Wave Generator

This class is actually a counter that is able to generate a square wave output
inside two limits. The square wave period are defined upon PERIOD and DUTY CYCLE.
The dynamical generator is:
```
t' = 1
s' = 0
```
and has the following jump rules:
```
Condition: t ≥ duty_cycle * T and s = 0
t+ = t
s+ = 1 - s
```
```
Condition: t ≥ T and s = 1
t+ = 0
s+ = 1 - s
```
And the ouput of the dynamical system is:
```
y = s y1 + (1 - s) y0
```
But to make things faster, some evaluations are simplified in the code itself.

### Controller Alarm

The Alarm Control is raised when the difference between the reference
pressure and the measured accumulator pressure is too distant for too
long from the reference pressure OR if accumulator pressure is over
a certain threshold.

The class responds to the following dynamical system:
```
t' = s
s' = 0
```
and to the following jump conditions:
```
Condition: abs(p_act - p_ref) ≥ TH and s != 1
t+ = t
s+ = 1
```
```
Condition: abs(p_act - p_ref) ≤ TH and s != 0
t+ = 0
s+ = 0
```
the actual alarm is raised when the following condition occurs:
```
Condition: t ≥ TH_time
Alarm is raised
```
that is enabled when when system pressure deviates from the control input, 
```
Condition: p_act ≥ TH_pres
Alarm is raised
```
which is really unlikely to occur.

### Accumulator Alarm

The Alarm Accumulator is raised when the distance between the measured
accumulator pressure and the measured actuator pressure is above a certain
threshold for a defined time.
The class responds to the following dynamical system:
```
t' = s
s' = 0
```
and to the following jump equations:
```
Condition: abs(p_act - p_acc) ≥ TH and s != 1
t+ = t
s+ = 1
```
```
Condition: abs(p_act - p_acc) ≤ TH and s != 0
t+ = 0
s+ = 0
```
the actual alarm is raised when the following condition occurs:
```
Condition: t ≥ TH_time
Alarm is raised
```

### Pressure controller defines

 * `PRESCTRL_CTRLALARM_PRES_TH`: Pressure threshold for the Controller Alarm
 * `PRESCTRL_CTRLALARM_TIME_TH`: Time threshold (out-of-bound) for the Controller Alarm
 * `PRESCTRL_CTRLALARM_OVERPRESS_TH`: Over pressure threshold for the Controller Alarm
 * `PRESCTRL_ACCALARM_PRES_TH`: Time threshold (out-of-bound) for the Accumulator Alarm
 * `PRESCTRL_ACCALARM_TIME_TH`: Over pressure threshold for the Accumulator Alarm
 * `PRESCTRL_PI_SATMIN`: Minimum value for PI saturation
 * `PRESCTRL_PI_SATMAX`: Maximum value for PI saturation
 * `PRESCTRL_PACTUATOR_SENSOR_PIN`: Pin selection for actuator pressure sensor
 * `PRESCTRL_PACCUMULATOR_SENSOR_PIN`: Pin selection for accumulator pressure sensors
 * `PRESCTRL_PACT_OUT_PIN`: Pin selection for actuator command output
 * `PRESCTRL_ENABLE_PRES_CTRL`: Pin for enabling overall ressure control
 * `PRESCTRL_SENSOR_ACTUATOR_M`: Sensor linear coefficient
 * `PRESCTRL_SENSOR_ACTUATOR_Q`: Sensor offset coefficient
 * `PRESCTRL_SENSOR_ACCUMULATOR_M`: Sensor linear coefficient
 * `PRESCTRL_SENSOR_ACCUMULATOR_Q`: Sensor offset coefficient
 * `PRESCTRL_MOVAVG_ACTUATOR`: Exponential Moving average trade off
 * `PRESCTRL_MOVAVG_ACCUMULATOR`: Exponential Moving average trade off

## Temperature Controller

The class handles the control for the temperature. It is actually a system in the form:
```
x' = f(x, c, r)        x : temperature plant state. We have a sensor for it
c' = 0                 c : chiller state
r' = 0                 r : resistance state
t' = s * LOOP_TIMING   t : counter for alarm state
s' = 0                 s : derivative of the timer
```
This dynamic equation may evolve with jumps if those conditions are reached:
```
Condition: s != -1 and t ≤ Tset - Tdelta
x+ = x
c+ = 0
r+ = 1 * r_enabled
t+ = 0
s+ = -1
```
`r_enabled` is an additional bit that allows the resistance to be turned on, and can be overridden by the user.
```
Condition: s != 1 and t ≥ Tset + Tdelta
x+ = x
c+ = 1 * c_enabled
r+ = 0
t+ = 0
s+ = 1
```
`v_enabled` is an additional bit that allows the resistance to be turned on, and can be overridden by the user. When 
the temperature is inside a certain hysteresis band all the output are turned off: 
```
Condition: (s == 1 and t ≤ Tset) or (s == -1 and t ≥ Tset) 
x+ = x
c+ = 0
r+ = 0
t+ = 0
s+ = 0
```
This switch condition is not optimal (the temperature osccilates quite a bit) but it allows to save some interrupts on
the relays and to preserve their operative time.
If the system deviates from the set point, after a certain amount of time an alarm is raised.
```
Condition abs(t) ≥ alarm_threshold and TEMP_CTRL = 1
An error is raised in the state machine and all the system is shut down
```

### Temperature controller defines

 * `TEMPCTRL_OFFSET`: Offset degrees for temperature controller
 * `TEMPCTRL_LIMIT_TIME_ALARM`: Time accumulator for alarm limit
 * `TEMPCTRL_CHILLER_PIN`: Pin for chiller digital out
 * `TEMPCTRL_RESISTANCE_PIN`: Pin for resistance digital out
 * `TEMPCTRL_TEMPSENSOR_PIN`: Pin for analog temperature sensor
 * `TEMPCTRL_MOVAVG_ALPHA`: Exponential Moving average trade off
 * `TEMPCTRL_SENSOR_M`: Sensor linear coefficient
 * `TEMPCTRL_SENSOR_Q`: Sensor offset coefficient

## Storage system

The class saves and restores some configuration in the state of the
machine. The idea is to store and restore only upon user request.
The configuration saved are:

 * `unsigned long max_cycle`: Max cycle configuration
 * `float kp`: Proportional gain configuration
 * `float ki`: Integral gain configuration
 * `float period`: Square wave period configuration
 * `float duty_cycle`: Square wave duty cycle configuration
 * `float t_set`: Temeperature set point configuration
 * `float p_low`: Low pressure configuration for reference
 * `float p_high`: High pressure configuration for reference

and the `cycle` count is also saved.

### Storage defines

 * `STORAGE_REF_ADDRESS`: The default storage reference address offset
 * `STORAGE_FREQUENCY`: Specify when to write current cycle number to EEPROM (1000 means every 1000 cycles)

## Serial Parser

A simple implementation of a serial parser state machine
that is able to receive and send two structs of data that
are defined in `Messages.h`.

The input is read in a non blocking manner and it is updated as
soon as data is available.

The output is written all at the same time, **blocking** (through `flush`) the board
in the operation, so we have to make sure that the operation:
 
 1. is not performed in an interrupt routine
 2. do not send too much data for a single operation

### Receiving data algorithm

The command receives ALL the character from the serial monitor if they are available
and once all the char have been written to a temporary buffer, they are copied to
the output structure of the parser. This means that if the transmitting endpoint
spams too much char on the serial it will block our receiving endpoint.

When data is fully received it is immediately copied in the output structure,
and the flag data_ready is set to true. That means: if we are sending too much
info from the sending endpoint, we are forcing an overwrite. Last command sent
is always more important with respect to the previous ones.

If checksum is not respected, data is not copied and error is set to true.
Error is reset to false at the first good package received.
Received data is expected to be little-endian.

The command to be executed are defined in `Commands.h`, in the form of an array
of function pointer.

### Sending Data Algorithm

Sends on the serial the output structure. The ouptut structure should be less than 64byte
to be sure not to full the output buffer of the Arduino. Longer structure can be used
but for sure the operation timing will depend on the length of the structure itself.

The operation is made as blocking in order to avoid chenge in the output structure
while writing data to serial.

> FIXME: Here I'm doing a blocking operation. It is performed only if requested via serial
  thus it is stil safe, but if the serial disconnects while sending with flush blocking
  everything, then I may have some problem. With enough timer one should define a timer
  for maximum transmission time and raise an alert if the threshold time is reached.

### Checksum algorithm

The checksum algorithm is a XOR accumulation:
```
check = 0x00

for each byte in buffer:
  check = check XOR byte

return check
```
If a checksum is failed the system enters the **Alarm** state.

### Input command protocol

The input command protocol is a 6 bytes array of char that is transformed to a little-endian
structure composed by:

 * 1 character: the command to operate
 * 1 float: an input value for the command
 * 1 character: a checksum

```
       0     1   2   3   4     5
     +---+ +---+---+---+---+ +---+  
0x0_ |   | |   |   |   |   | |   |
     +---+ +---+---+---+---+ +---+  
      CMD   VALUE             CHECK
```

The commands are described in the `Commands.h` and currently are:

  | **Byte** | **Command Enum** | Description |
  |--------------|---------------|------------|
  | `0x00` | `Hearthbeat` | **Request an update about the system state** |
  | `0x01` | `ManualTemperatureControl` | Overrides (disabling) the temperature control |
  | `0x02` | `AutomaticTemperatureControl` | Enables the pressure control |
  | `0x03` | `ManualPressureControl` | Overrides (disabling) the pressure control |
  | `0x04` | `AutomaticPressureControl` | Enables the pressure control |
  | `0x05` | `TogglePauseCycle` | Pauses / restarts the control system |
  | `0x06` | `EmergencyStopCycle` | Calls immediately the alarm status |
  | `0x07` | `ToggleChillerActuation` | Enables chiller actuation |
  | `0x08` | `ToggleResistanceActuation` | Enables resistance actuator |
  | `0x09` | `SetTemperature` | Sets temperature setpoint |
  | `0x0A` | `SetPressureHigh` | Sets pressure set point for reference generator |
  | `0x0B` | `SetPressureLow` | Sets pressure set point for reference generator |
  | `0x0C` | `SetPressure` | Overrides current pressure set point |
  | `0x0D` | `OverridePIControl` | Override PI Control with voltage input |
  | `0x0E` | `SetPIProportionalGain` | Sets PI proportional gain |
  | `0x0F` | `SetPIIntegrativeGain` | Sets PI integrative gain |
  | `0x10` | `SetMaximumCycleNumber` | Sets target number for cycles |
  | `0x11` | `SetCurrentCycleNumber` | Override current Cycle number |
  | `0x12` | `SetReferencePeriod` | Setup reference square wave period |
  | `0x13` | `SetReferenceDutyCycle` | Setup reference square wave cycle |
  | `0x14` | `SystemReboot` | Force an immediate system reboot |
  | `0x15` | `StartCycle` | Starts the system (Running state) |
  | `0x16` | `SaveStorageConfig` | Save current configuration in the EEPROM |
  | `0x17` | `LoadStorageConfig` | Load storage config from EEPROM. Extremely risky, it may be corrupted data |
  | `0x18` | `LoadStorageCycle` | Load current cycle number from EEPROM. Extremely Risky it may be corrupted data |
  | `0x19` | `CommandCodeSiz` | This last one is a size for the array of function pointers |

### Output state protocol

The output sends information about the current operating state of the system:

 * `float t_meas`: Temeprature measurements
 * `float p_meas`: Actuator Pressure measurements
 * `float q_meas`: Accumulator pressure measurements
 * `float kp`: Pressure proportional gain
 * `float ki`: Pressure Integral gain
 * `float t_set`: Temeprature set point
 * `float p_set`: Pressure set point for actuator
 * `float u_pres`: Current output for pressure control
 * `float period`: Square wave generator period
 * `float duty_cycle`: Duty cycle coefficient
 * `float cycle`: Cycles number
 * `float max_cycle`: Max cycles Number
 * `char config`: Actuators configuration
 * `char state`: Current state flag
 * `char error`: Last error flag
 * `char check`: Communication checksum (accumulated xor)

```
       0   1   2   3     4   5   6   7     8   9   A   B     C   D   E   F
     +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ 
0x0_ |   |   |   |   | |   |   |   |   | |   |   |   |   | |   |   |   |   | 
     +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ 
      Meas. Temp.       Meas. P Actuat.   Meas. P Accumul.  PI Kp 

       0   1   2   3     4   5   6   7     8   9   A   B     C   D   E   F
     +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ 
0x1_ |   |   |   |   | |   |   |   |   | |   |   |   |   | |   |   |   |   | 
     +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ 
      PI Ki             Set point T       Set point P       PI output

       0   1   2   3     4   5   6   7     8   9   A   B     C   D   E   F
     +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ 
0x2_ |   |   |   |   | |   |   |   |   | |   |   |   |   | |   |   |   |   | 
     +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ +---+---+---+---+ 
      Period            Duty Cycle        Cycle             Cycle Target

       0        1        2         3
     +---+    +---+    +---+     +---+
0x3_ |   |    |   |    |   |     |   |         SIZE = 0x34 = 52 bytes
     +---+    +---+    +---+     +---+
      Config   State    Error     Check 
```

> The update MUST BE REQUESTED through the command `Hearthbeat`!

#### Config table

The char `0x30` reports the system configuration:

| **Byte** | **Config** | Description |
|---|---|---|
| `0x01` | Chiller Enabled | Additional bit to enable the chiller actuation |
| `0x02` | Resistance Enabled | Additional bit to enable the resistance actuation |
| `0x04` | Actuator Enabled | Additional bit to enable the pressure actuation |

#### State Table

The char `0x31` reports the system state:

| **Byte** | **State** | Description |
|---|---|---|
| `0x01` | Alarm | The system is in alarm mode. Only save configuration, hearthbeat and system reboot commands are accepted |
| `0x02` | Pause | The system is in pause between cycles |
| `0x04` | Running | The system is running the cycles |
| `0x08` | Waiting | Waiting for user input, it is actually never used |
| `0x0A` | Serial Setup | Very initial state, it is actually never reported |

### Interfacing

It is possible to interface the device with a raspberry PI using the library [project.controller.interface](https://github.com/MatteoRagni/project.controller.interface)

### Serial Parser defines

 * `SERIAL_PORT`: Serial port used for communication
 * `SERIAL_PORT_BAUD`: Serial port baud speed (bps)
 * `SERIAL_SYNC_CHAR`: Signature for accepting incoming connection
 * `SERIAL_PARSER_TEMP_MIN`: Minimum accepted temperature set point in celsius
 * `SERIAL_PARSER_TEMP_MAX`:  Maximum accepted temperature set point in celsius
 * `SERIAL_PARSER_PRESSURE_MIN`:  Maximum accepted temperature set point in bar
 * `SERIAL_PARSER_PRESSURE_MAX`:  Maximum accepted temperature set point in bar
 * `SERIAL_PARSER_KPI_MIN`:  Minimum value for the PI gain
 * `SERIAL_PARSER_KPI_MAX`:  Maximum value for the PI gain
 * `SERIAL_PARSER_PERIOD_MIN`:  Minimum value for the square wave generation in secs
 * `SERIAL_PARSER_PERIOD_MAX`:  Maximum value for the square wave generation in secs
