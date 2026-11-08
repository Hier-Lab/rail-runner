# `rail-drive` contains the software for the drive board

`BOARD=drive` targets an ak60-powered linear track and flashes to an STM-32 Nucleo F446RE development board.

---

## The AK60-6-V3.0 system

The AK60 operates in **MIT torque mode** at 48V with a max rated speed of 490 rpm. Velocity is commanded through the MIT frame with zero position target, zero position gain, zero feed-forward torque, and a velocity gain Kd that defaults to 0.100.

### Motion

Hardcoded `#define` constants    
- `MAX_ACC` 100 mm/s^2
- `MAX_JRK` 150 mm/s^3
- `MAX_VEL` 32 mm/s
- `MIN_POS` 0 mm
- `MAX_POS` 420 mm
- `POSITION_MARGIN` is 2 mm, so normal motion brakes to the 2--418 mm operating envelope. Inside either margin, only motion back toward the operating envelope is allowed.
- `CAL_VEL` is -2 mm/s (the home-switch approach speed)
- `CAL_BACKOFF_VEL` is 5 mm/s (the back-off speed once the home switch asserts, faster than the approach to minimize dwell time pressing on it)
- `PITCH` is 5 mm/rev
- Safety limits use a time-optimal jerk-limited velocity profile

The AK60-6 V3 uses CubeMars extended-ID CAN at 1 Mbps with motor ID 2 and 500 Hz feedback. The drive must already be configured for MIT torque mode, 48 V, a 5 A current cap, and zero torque after 10 ms without CAN commands. Each active command is one 8-byte packet-type-8 frame containing zero Kp, the selected Kd, zero position, output-shaft velocity in rad/s, and zero feed-forward torque. Deactivated and faulted states send the same frame with all five logical fields zero.

Rail position integrates measured velocity from the existing `0x2902` status feedback at the 1 kHz control rate while feedback is fresh and fault-free. Electrical RPM is converted using 14 pole pairs, 6:1 reduction, and `PITCH`. The 16-bit position field is not used. Calibration establishes a firmware-local origin at the confirmed optical clear edge without changing the motor's internal origin or CAN feedback configuration. Verify travel scale and positive motor direction on hardware.

Comms with ros2 lifecycle node specified in `rail-interface/SPEC.md` occur over a 115200 baud serial interface at a 250 Hz transaction rate:
- requests receive one reply, except that `cal` also emits its completion reply asynchronously
- requests accept LF/CRLF
- replies use LF

Generally, messages take the form:
- `<cmd> <val>`
- `ack <pos> <vel> <acc>` or `<status> <code>`

Requests use the exact grammar `cal`, `dis`, `sp <signed-decimal>` with at most one fractional digit, or `kd <unsigned-decimal>` with at most three fractional digits.
    - mm, mm/s, and mm/s^2 units

### Serial commands

`<rec>` <-- `<reply>`

- `cal` <-- `cal` while calibrating, then `ack <pos> <vel> 0.0` when done
    - `cal` is sent once. Calibration times out after 90 seconds with `err cal`.
    - If the debounced home switch is clear, ramp to `CAL_VEL` (-2 mm/s) until it asserts. If already asserted, begin backing off immediately at `CAL_BACKOFF_VEL` (5 mm/s). The home switch does not fault during calibration; it is the cal target.
    - Reverse at `CAL_BACKOFF_VEL` with the normal jerk/acceleration limits until the home switch has been clear for 5 ms. Set the firmware position origin to the first clear sample in that confirmed interval.
    - Brake to zero commanded velocity and acceleration. Calibration completes in active zero-velocity mode at the resulting positive measured position and does not wait for measured velocity to settle.
    - The far switch still faults (`err est`) even during calibration.
    - Once active, the optical gates (below) act as hard stops; `cal` itself ignores them.
- `dis` <-- `dis` immediately. `dis` is sent once. Continue with a jerk-limited stop, switch to the all-zero MIT command, invalidate calibration, and enter the deactivated state.
- `sp <vel>` <-- `ack <pos> <vel> <acc>`. Valid commands refresh a 50 ms watchdog, but the watchdog is only armed while the last requested velocity is nonzero. A host that goes silent while actively commanding motion times out and triggers `err com`; a host that has already requested zero (holding, including the hold `cal` leaves you in) can disconnect indefinitely without faulting or losing calibration, so a run can be split across several separate host sessions as long as each new session only reconnects while the carriage is holding.
- `kd <gain>` <-- `kd <gain>` formatted to three fractional digits. Values from 0 through 1.000 are accepted in every state, apply immediately, persist until MCU reset, and do not refresh the host watchdog or alter the current state. Malformed and out-of-range values return nonfatal `err kd` without changing the gain.


### Error responses

| Error | Cause | Response | Fatal? |
|---|---|---|---|
| `err cal` | Non-`cal` during calibration | Deactivate | No |
| `err dis` | Non-`cal` while deactivated | Stay deactivated | No |
| `err lim` | Setpoint exceeds velocity limit | Deactivate | Yes |
| `err hrd` | Retired; no longer produced (see below) | n/a | n/a |
| `err est` | Wired E-stop pressed, far switch tripped (any state), or home switch tripped (while active or stopping) | Disable motor | Yes |
| `err pos` | Motion farther outside 2--418 mm | Hold zero; allow inward motion | No |
| `err com` | Host command timeout while still commanding nonzero velocity | Limited stop; deactivate | Yes |
| `err mot` | CubeMars fault | Send all-zero MIT command | Yes |
| `err can` | TX failure or 10 ms feedback loss | Send all-zero MIT command | Yes |
| `err cmd` | Malformed active-state command | Keep current state; don't refresh watchdog | No |
| `err kd` | Invalid `kd` in any state | Keep current state; don't refresh watchdog | No |
| `err sys` | Control overrun or UART overflow | Send all-zero MIT command | Yes |

All fatal faults invalidate calibration. 

Fault priority is estop, system, motor, CAN, velocity limit, communication, then calibration/state/parser errors. A healthy `dis` clears a pending firmware fault without first reporting it, and additionally requires the E-stop, far switch, and home switch to all currently read healthy; motor and CAN faults require fresh fault-free feedback. There is no direct-to-`cal` recovery shortcut from a fault: estop recovery (wired E-stop, far switch, or home switch) always requires physical reset, `dis`, and a new calibration.

The mechanical home and far switches are E-stop-class safety inputs, the same severity and recovery path as the wired E-stop, not a separate hard-limit class:
- The home switch (normally-open, active-low) is sensed on PC8. It is the calibration target — `cal` rides onto and off it to find zero, and it does not fault during calibration — but once active or stopping, pressing it faults `err est`.
- The far switch (normally-open, active-low) is sensed on PC9 and faults `err est` unconditionally, in every state including calibration.
- The normally-closed estop relay chain is sensed on PA1, independent of and in addition to the two switches.

All three switches/relay remove motor power while leaving the Nucleo powered. Emergency inputs are acted on immediately, undebounced; the home switch's calibration-target debounce (5 ms) is separate and only affects when `cal` treats it as settled, not how fast it can fault during active/stopping.

Two optical gates are hard stops, not faults. The home-side gate is sensed on PC10 and the far-side gate on PC11 (both active-high, pulled down so a disconnected or dead sensor reads as clear). Once the carriage has passed fully through a gate moving toward that gate's end, further motion toward that end commands zero — the normal jerk-limited profile brakes it to a halt, in `STATE_ACTIVE`, with no fault and no `err` reply. Motion back away from that end is unrestricted, and `cal` ignores both gates entirely. Reversing direction while still inside a gate's slot is handled: which side of the gate the carriage ends up on is decided by its direction of travel at the moment it leaves the slot, not the direction it entered. Gate inputs are debounced for 5 ms.

### Hotloop

This communication protocol runs inside a single nonblocking hotloop released by an STM32 timer ISR at 1 kHz. The ISR only releases cycles and detects overruns.

1) read protocol
2) read safety info
    - if the E-stop, far switch, or (while active/stopping) home switch is tripped, kill (`err est`)
    - if a fatal fault is present, send the all-zero MIT command
3) read state info
    - read motor feedback position and velocity
    - update linear position and velocity
    - update the two optical gates' debounced state and which side of each the carriage is on
4) control loop
    - set velocity command to the requested velocity, adjusted for acceleration, jerk, gate hard stops, and position limits, and convert from mm/s to output-shaft rad/s
5) reply protocol

### Electrical

The NUCLEO-F446RE connections for `BOARD=drive` are:

| Function | Nucleo connector | MCU pin | Connect to |
|---|---|---|---|
| CAN receive | D15, CN5 pin 10 | PB8 / CAN1_RX | CAN transceiver RXD |
| CAN transmit | D14, CN5 pin 9 | PB9 / CAN1_TX | CAN transceiver TXD |
| Home switch (E-stop class, also the cal target) | CN10 pin 2 | PC8 | Normally-open switch contact to GND |
| Estop chain | CN7 pin 30 | PA1 | Normally-closed relay contact to GND |
| Far switch (E-stop class) | CN10 pin 1 | PC9 | Normally-open switch contact to GND |
| Home-side optical gate (hard stop) | CN7 pin 1 | PC10 | Optical sensor SIG |
| Far-side optical gate (hard stop) | CN7 pin 2 | PC11 | Optical sensor SIG |
| Serial transmit | D1, CN9 pin 2 | PA2 / USART2_TX | Onboard ST-LINK USB virtual COM |
| Serial receive | D0, CN9 pin 1 | PA3 / USART2_RX | Onboard ST-LINK USB virtual COM |
| Logic ground | CN7 pin 20 or 22 | GND | All external logic grounds |

PB8 and PB9 are logic-level CAN signals, not CANH and CANL.
Use a 3.3 V-compatible CAN transceiver between the Nucleo and the motor CAN bus.
The three E-stop-class inputs use internal pull-ups and are high when inactive, low
when their contacts close (normally-open for the two switches) or open (normally-closed
for the estop relay). The two optical gates drive SIG themselves (not a switch contact),
low when idle and high when triggered, and use internal pull-downs so a disconnected or
dead sensor reads as clear rather than floating. Power the optical sensors from 3.3 V to
keep SIG within the Nucleo pins' rated range.
The relay contacts must be voltage-free; never apply motor voltage to a Nucleo pin.

USART2 is connected to the onboard ST-LINK virtual COM port by default and runs at
115200 baud, 8 data bits, no parity, and 1 stop bit.
No jumper wires are required for the normal USB serial connection.
Before using D0 and D1 with an external 3.3 V UART, configure the board's solder bridges
to avoid contention with the ST-LINK virtual COM port.
