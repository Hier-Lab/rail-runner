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
- `MAX_POS` 325 mm is the travel measured on the rig (324.9 mm to the far switch press). Each calibration replaces it with the measured far switch position.
- `POSITION_MARGIN` is 2 mm, so normal motion brakes to an operating envelope 2 mm inside zero and 2 mm inside the measured far switch position. Inside either margin, only motion back toward the operating envelope is allowed.
- `CAL_FAST_VEL` is 20 mm/s and is used only between the two gates (and for parking). `CAL_SLOW_VEL` and `CAL_PROBE_VEL` are 2 mm/s: everything between a gate and its switch, and the blind probe, runs slow regardless of what the run believes about where the gates and switches are (gates can be moved or replaced). `CAL_BACKOFF_VEL` is 5 mm/s (leaving a switch). A missing or misplaced gate on a fast leg ends in an `err est` at the switch at the fast speed.
- `CAL_HOME_ZONE_MM` is 60 mm and `CAL_FAR_ZONE_MM` is 48 mm, the gate-to-switch distances measured on the rig (outer gate edge to switch press). Only the calibration budgets depend on them.
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

Requests use the exact grammar `cal`, `dis`, `sp <signed-decimal>` with at most one fractional digit, `kd <unsigned-decimal>` with at most three fractional digits, or `res`, `res h`, `res f`.
    - mm, mm/s, and mm/s^2 units

### Serial commands

`<rec>` <-- `<reply>`

- `cal` <-- `cal` while calibrating, then `ack <pos> <vel> 0.0` when done
    - `cal` is sent once. The run sweeps the whole rail and touches both switches, in the numbered phases below. Phase 5 of the original proposal is not run.
    - 1, probe: move toward home at `CAL_PROBE_VEL`. The home switch asserting goes to phase 4, the home gate being blocked goes to phase 3, and the far gate clearing or `CAL_PROBE_BUDGET_MM` (80 mm) of travel goes to phase 2. A run that starts on the home switch begins at phase 4, and one that starts with the home gate blocked begins at phase 3.
    - 2, fast home: `CAL_FAST_VEL` toward home until the home gate is blocked.
    - 3, slow home: `CAL_SLOW_VEL` toward home until the home switch asserts.
    - 4, back off home: `CAL_BACKOFF_VEL` away from home until the switch has been clear for 5 ms. Zero is the first clear sample in that confirmed interval.
    - 6, fast far: `CAL_FAST_VEL` away from home until the far gate is blocked.
    - 7, slow far: `CAL_SLOW_VEL` until the far switch asserts. That position is the travel length.
    - 8, back off far: `CAL_BACKOFF_VEL` toward home until the far switch is clear.
    - 9, park: move to 10 mm on the home side of the far gate and brake to zero. Calibration completes there in active zero-velocity mode.
    - A phase fails with `err cal` if it travels past its distance budget, takes longer than 1.5 times that distance should take plus 3 s, or the carriage moves at under a quarter of the commanded speed for 500 ms. The budgets are 80 mm for the probe, 90 mm for a slow phase, 10 mm for a back-off, and 360 mm for a fast phase or the park.
    - The home switch does not fault in phases 1, 3, and 4, nor in phase 6 until the carriage is 2 mm past zero. The far switch does not fault in phases 7 and 8, nor in phase 9 until the carriage is 2 mm off it. In every other phase either switch faults `err est`, which is how a dead gate ends: the fast leg reaches the switch behind it.
    - Once active, the optical gates (below) act as hard stops.
- `res` <-- `res <phase> <fail> <travel> <far-release> <home-press>` for the latest calibration, running, finished, or failed. `<phase>` is the phase number above, or 10 when done. `<fail>` is 0 for none, 1 for a distance budget, 2 for a time budget, 3 for a stall. Positions are in mm from zero with two fractional digits, or `-` if not measured.
- `res h` and `res f` <-- `res h|f <lead-in> <trail-in> <trail-out> <lead-out>`, the positions of the home or far gate's edges. "In" is travel toward that gate's own end, and the leading edge is the one met first on the way in.
    - `res` is accepted in every state, and does not refresh the host watchdog or alter the current state.
- `dis` <-- `dis` immediately. `dis` is sent once. Continue with a jerk-limited stop, switch to the all-zero MIT command, invalidate calibration, and enter the deactivated state.
- `sp <vel>` <-- `ack <pos> <vel> <acc>`. Valid commands refresh a 50 ms watchdog, but the watchdog is only armed while the last requested velocity is nonzero. A host that goes silent while actively commanding motion times out and triggers `err com`; a host that has already requested zero (holding, including the hold `cal` leaves you in) can disconnect indefinitely without faulting or losing calibration, so a run can be split across several separate host sessions as long as each new session only reconnects while the carriage is holding.
- `kd <gain>` <-- `kd <gain>` formatted to three fractional digits. Values from 0 through 1.000 are accepted in every state, apply immediately, persist until MCU reset, and do not refresh the host watchdog or alter the current state. Malformed and out-of-range values return nonfatal `err kd` without changing the gain.


### Error responses

| Error | Cause | Response | Fatal? |
|---|---|---|---|
| `err cal` | Non-`cal` during calibration, or a calibration phase over budget or stalled | Deactivate | No |
| `err dis` | Non-`cal` while deactivated | Stay deactivated | No |
| `err lim` | Setpoint exceeds velocity limit | Deactivate | Yes |
| `err hrd` | Retired; no longer produced (see below) | n/a | n/a |
| `err est` | Wired E-stop pressed, far switch tripped (any state), or home switch tripped (while active or stopping) | Disable motor | Yes |
| `err pos` | Motion farther outside the operating envelope | Hold zero; allow inward motion | No |
| `err com` | Host command timeout while still commanding nonzero velocity | Limited stop; deactivate | Yes |
| `err mot` | CubeMars fault | Send all-zero MIT command | Yes |
| `err can` | TX failure or 10 ms feedback loss | Send all-zero MIT command | Yes |
| `err cmd` | Malformed active-state command | Keep current state; don't refresh watchdog | No |
| `err kd` | Invalid `kd` in any state | Keep current state; don't refresh watchdog | No |
| `err sys` | Control overrun or UART overflow | Send all-zero MIT command | Yes |

All fatal faults invalidate calibration. 

Fault priority is estop, system, motor, CAN, velocity limit, communication, then calibration/state/parser errors. A healthy `dis` clears a pending firmware fault without first reporting it, and additionally requires the E-stop, far switch, and home switch to all currently read healthy; motor and CAN faults require fresh fault-free feedback. There is no direct-to-`cal` recovery shortcut from a fault: estop recovery (wired E-stop, far switch, or home switch) always requires physical reset, `dis`, and a new calibration.

The mechanical home and far switches are E-stop-class safety inputs, the same severity and recovery path as the wired E-stop, not a separate hard-limit class:
- The home switch (normally-open, active-low) is sensed on PC8. `cal` rides onto and off it to find zero, so it does not fault in the calibration phases that approach it or while deactivated. In the other calibration phases and once active or stopping, pressing it faults `err est`.
- The far switch (normally-open, active-low) is sensed on PC9. `cal` rides onto and off it to measure the travel length, so it does not fault in the calibration phases that approach it. In every other state and phase it faults `err est`, and a pressed far switch still blocks starting `cal`.
- The normally-closed estop relay chain is sensed on PA1, independent of and in addition to the two switches.

The wired estop relay chain removes motor power while leaving the Nucleo powered. The home and far switches do not cut motor power in hardware (confirmed 2026-11-17): the firmware is the only thing that acts on them, so they protect only while the control loop is running. Emergency inputs are acted on immediately, undebounced; the home switch's calibration-target debounce (5 ms) is separate and only affects when `cal` treats it as settled, not how fast it can fault during active/stopping.

Two optical gates are hard stops, not faults. The home-side gate is sensed on PC10 and the far-side gate on PC11 (both active-high, pulled down so a disconnected or dead sensor reads as clear). Once the carriage has passed fully through a gate moving toward that gate's end, further motion toward that end commands zero — the normal jerk-limited profile brakes it to a halt, in `STATE_ACTIVE`, with no fault and no `err` reply. Motion back away from that end is unrestricted. `cal` does not hard-stop at the gates; it uses them to switch from its fast speed to its slow speed. Reversing direction while still inside a gate's slot is handled: which side of the gate the carriage ends up on is decided by its direction of travel at the moment it leaves the slot, not the direction it entered. Gate inputs are debounced for 5 ms.

### Hotloop

This communication protocol runs inside a single nonblocking hotloop released by an STM32 timer ISR at 1 kHz. The ISR only releases cycles and detects overruns.

1) read protocol
2) read safety info
    - if the E-stop is tripped, or a switch is tripped outside the calibration phases that ride onto it, kill (`err est`)
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
