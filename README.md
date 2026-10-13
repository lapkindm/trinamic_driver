# trinamic-driver

Python driver for the **Trinamic SIXpack 2** six-axis stepper controller over
RS232/RS485, using its binary 9-byte protocol, plus a Bliss motor controller.

Written from the *SIXpack 2 Manual* (Trinamic, V1.01, May 2006), which is
not included here. The newer TMCL
modules use a different protocol, and this driver does not support them.

## Install

```bash
pip install -e .
```

The package has no runtime dependency. The examples use `pyserial`.

## Quick start

```python
import serial
from trinamic_driver import SixPack

with serial.Serial("/dev/ttyS9", 19200, timeout=0.05) as ser:
    pack = SixPack(ser, address=0)          # unit address = rotary switches
    pack.unit_info()                        # firmware, reset flag, temperature, serial
    pack.position(2)                        # position, action, stop flag of motor 2
    pack.start_ramp(2, 5400)                # trapezoidal ramp to 5400 microsteps
    pack.stop(1 << 2)                       # $2A: target := actual position
```

## Protocol facts the driver relies on

- **Frames.** Every command is 9 bytes: address, command, P0..P6. Values are
  sent least significant byte first, and there is no checksum.
- **No acknowledgements.** Set and motion commands get no reply, and the unit
  silently ignores a bad one. Only queries answer: $20, $21, $28, $30 and $43.
- **Reply format is ambiguous in the manual.** The $20 example reads 8 bytes
  (command, then P0..P6). The command table suggests 9 bytes, with an address
  first. Both are accepted by default and the first reply fixes the format
  (`SixPack(reply_length=...)` forces one). `examples/check_readonly.py`
  reports which one the unit uses.
- **Settings are write-only and volatile.** They cannot be read back and are
  lost on a power cycle. $43 has a reset flag: it reads 1 on the first read
  after a reset, then 0.
- **Read side effects.** Reading $43 clears the reset flag, and reading $20
  clears the stop flag. The driver keeps both.
- **Speed.** f = 20 MHz / (clkdiv+1) · v / 2^(14+div), with v ≤ 511. clkdiv
  is global, div is per motor. During acceleration v grows by amax/64 every
  2 ms, and amax ≤ 64·vstart.
- **Never sent.** The commands that change the unit itself are refused, also as
  raw commands: baud rate, timeout, address, power-down mode, hardware reset,
  flash and EEPROM ($40–$42, $45, $CC, $F2–$F7).

## Bliss

`trinamic_driver.bliss_trinamic.Trinamic` is one controller per unit, with
one axis per motor channel. The XRD-3003TT config is
[bliss_config/xrd3003tt_trinamic.yml](bliss_config/xrd3003tt_trinamic.yml),
translated from spec.

**How it behaves:**

- **Serial port.** A local device (`/dev/...`) is opened with pyserial.
  bliss.comm's Serial lost bytes on this instrument's ViX line, and the
  SIXpack has no acknowledgements to catch a damaged frame. Remote URLs use
  bliss.comm.

- **Programming.** When an axis initialises, the controller programs it; all
  motors must be stopped, otherwise it refuses. By default
  (`program_parameters: true`) that is the currents, speeds, $15 flags
  (`home_method`, spec's 16-bit encoding), reference-search parameters and
  null offset. With `program_parameters: false` it sends only what spec
  sends: the clock divider ($12), the stop limits ($31) and the speeds
  ($13/$14), and the unit keeps its own power-on settings. The controller
  programs the axes again after detecting a unit reset, checking the reset
  flag at most every `check_interval` seconds, like spec.
- **Lost replies.** A query that gets no reply or a garbled one is asked again
  up to twice, on an emptied input buffer, and logged. Commands are never
  repeated, so a move is never sent twice. $43 is not simply asked again,
  because reading it clears the reset flag: after a lost $43 reply the
  controller reads it once more and also checks the positions, since a reset
  puts every motor at 0. A motor that was elsewhere and now reads 0 is
  treated as reset (positions lost).
- **Lost positions.** After a reset the axes report `READY | POSLOST`. Moves
  and jogs are refused until the axis is homed (`axis.home()`, the unit's
  reference search) or its dial is set. The state is not FAULT because Bliss
  refuses `home()` on a FAULT axis. The lost flag is kept in Redis, so a
  Bliss restart cannot forget it.
- **Velocity and acceleration** are converted to v/amax. div is chosen per
  axis from the configured velocity, for the best resolution. A velocity
  outside that divider's range is refused (set `div` to go faster).
  Accelerations above 64·vstart are limited, with a warning.
- **Stop.** $2A sets the target to the current position. A fast motor
  overshoots and ramps back to that point. Jogs (rotation, $25) decelerate
  with amax and are then ended with $23.
- **Limits.** The unit's stop condition (StopNull or the analogue limits) is
  reported as LIMNEG/LIMPOS in the direction of the move, so the move raises
  `AxisOnLimitError`. With `ignore_limits: true` (spec's
  `disable_limit_checks`) it is only logged.
- **Homing** uses the unit's reference search. Its direction comes from the
  `home_method` flags, not from `home(±1)`. `stop_after_search: true` (the
  driver's default) stays at the reference point; the unit's default (false)
  drives back to the old position number. With `program_parameters: false`
  the unit's own reference settings apply.
- **Not supported:** `hw_limit()` (the unit has no limit search),
  `axis.on()`/`off()`, the PI controller, and trajectories.

Custom commands: `axis.trinamic_inputs()` and `axis.trinamic_supply()`.
`raw_write_read("20 02 00")` takes hex bytes (command, P0..) and returns P0..P6.

## On the XRD-3003TT

Since October 2026 the controller runs the monochromator `m1` and the two
absorber wheels `a1` and `a2` (one SIXpack 2, firmware 1.66, RS232 at
19200 baud). Confirmed there:

- **Replies are 9 bytes**, with the response address first.
- **Positions** are the unit's microstep counter; dial = steps / steps_per_unit
  matched spec's dial values.
- **spec sends very little.** A debug capture of spec showed only $40 (baud
  rate) and $31 (stop limits 0/1024) at start-up, $12 (clock divider 23) and,
  per move, $13/$14 (speeds) and $23. The unit otherwise runs on its power-on
  settings, $15 flags 0x0AA0. spec's `motor_par dump` shows 0x60D0 instead,
  which is not what the unit uses: programming those flags made `a2` stop
  between wheel positions, in Bliss and afterwards in spec. The XRD config
  therefore uses `program_parameters: false` and `clkdiv: 23`.
- **Absorber backlash.** A backlash move to wheel position 1 overshoots below 1,
  so the low limit of `a1` and `a2` is 1 minus the backlash (0.984375).

`examples/check_readonly.py /dev/ttyS9` (with spec and Bliss stopped) shows
the reply format, the supply voltage and the positions as dial values, without
moving anything.

## Tests

The tests need no hardware:

```bash
python -m unittest discover -s tests
```

- **Unit tests** check frames against the manual's worked examples, the
  conversions, and the unit layer against a simulator (`tests/fake_sixpack.py`).
- **End-to-end:** `tests/bliss_e2e.py` runs Bliss axes from the XRD config
  against the simulator, served on a pseudo-terminal by `tests/pty_sim.py`
  (setup in the docstring). It covers programming, moves, stop, jog, the
  stop condition, a power cycle with recovery by homing or by setting the
  dial, and raw commands. It passes with both 8- and 9-byte replies.

## Not yet verified on hardware

These come from the manual's text and are tested only against the simulator:

- homing (the reference search) and jogging: the deceleration of $25 with
  velocity 0, and $23 ending a rotation;
- a stop condition during a move;
- detecting a real power cycle of the unit, and the retries after lost or
  garbled replies.
