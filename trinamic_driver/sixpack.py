"""Talk to one Trinamic SIXpack 2 over RS232/RS485.

The unit answers queries only; set and motion commands are never
acknowledged, and a rejected command is silently ignored. Read back what you
can ($20, $21, $28, $30, $43) to confirm.

**Reply format.** The manual's example ($20, chapter 6.2.2.3) reads the reply
as command byte + P0..P6, eight bytes. The command table describes replies
like requests, which would put an address byte first (nine bytes). Not yet
verified on hardware, so by default both are accepted and the first reply
fixes the format: a nine-byte reply is recognised by the command byte in its
second position. Pass ``reply_length=8`` or ``9`` to force one.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Protocol

from .protocol import (
    FORBIDDEN,
    FRAME_LEN,
    PARAM_LEN,
    QUERIES,
    V_MAX,
    Cmd,
    InputsReply,
    PositionReply,
    ProtocolError,
    Reply,
    UnitInfo,
    VelocityReply,
    check_motor,
    decode_activities,
    decode_inputs,
    decode_position,
    decode_unit_info,
    decode_velocity,
    encode,
    i16,
    i32,
    split_home_method,
    u8,
    u16,
)


class SixPackError(Exception):
    """Base class for errors talking to the unit."""


class SixPackTimeout(SixPackError):
    """No (complete) reply in time."""


class Transport(Protocol):
    """Byte transport: pyserial ``Serial`` fits, as does a Bliss adapter."""

    def write(self, data: bytes) -> int | None: ...

    def read(self, size: int = 1) -> bytes:
        """Return up to ``size`` bytes; may return fewer (or none) after a short wait."""
        ...

    def reset_input_buffer(self) -> None: ...


log = logging.getLogger(__name__)


class SixPack:
    """One SIXpack 2, selected by its module address (rotary switches).

    A query that gets no reply or a garbled one is sent again (``retries``
    times) on an emptied input buffer: spec's "Trinamic motor became
    unresponsive" was cured by restarting spec, i.e. by fresh buffers, so a
    lost or partial reply should not stop a long measurement. Not retried:
    commands (no reply; a move is never repeated blindly) and $43, whose
    reset flag is cleared by reading it (see the controller's _unit_info).
    """

    def __init__(
        self,
        transport: Transport,
        address: int = 0,
        *,
        timeout: float = 0.5,
        response_address: int = 0,
        reply_length: int | None = None,
        quiet: float = 0.02,
        lock=None,
        retries: int = 2,
    ):
        if reply_length not in (None, 8, 9):
            raise ValueError("reply_length must be 8, 9 or None (detect)")
        self.transport = transport
        self.address = address
        self.timeout = timeout
        self.response_address = response_address
        self.reply_length = reply_length
        self.quiet = quiet  # wait for a ninth byte this long after the eighth
        self.retries = retries
        self.retried = 0  # queries that needed a retry (diagnostics)
        self._lock = lock if lock is not None else threading.RLock()

    # -- framing -------------------------------------------------------------

    def command(self, cmd: int, params: bytes = b"") -> None:
        """Send a command that has no reply."""
        if cmd in QUERIES:
            raise ProtocolError(f"${cmd:02X} is a query, use query()")
        frame = encode(self.address, cmd, params)
        with self._lock:
            self.transport.write(frame)

    def query(self, cmd: int, params: bytes = b"") -> Reply:
        """Send a query and return its reply (retried, see the class doc)."""
        frame = encode(self.address, cmd, params)
        attempts = 1 if cmd == Cmd.UNIT_INFO else 1 + self.retries
        for attempt in range(attempts):
            try:
                with self._lock:
                    self.transport.reset_input_buffer()
                    self.transport.write(frame)
                    return self._read_reply(cmd)
            except (SixPackTimeout, SixPackError) as exc:
                if attempt == attempts - 1:
                    raise
                self.retried += 1
                log.warning("SIXpack unit %d: %s; asking again", self.address, exc)

    def _read_reply(self, cmd: int) -> Reply:
        buf = b""
        deadline = time.monotonic() + self.timeout
        quiet_until = None
        while True:
            reply = self._match(buf, cmd, final=False)
            if reply is not None:
                return reply
            now = time.monotonic()
            if quiet_until is not None and now >= quiet_until:
                reply = self._match(buf, cmd, final=True)
                if reply is not None:
                    return reply
            if now >= deadline:
                self.transport.reset_input_buffer()
                if buf:
                    raise SixPackTimeout(
                        f"incomplete or unexpected reply to ${cmd:02X} from unit {self.address}: {buf.hex(' ')}"
                    )
                raise SixPackTimeout(f"no reply to ${cmd:02X} from unit {self.address}")
            chunk = self.transport.read(FRAME_LEN - len(buf))
            if chunk:
                buf += chunk
                if len(buf) == 8 and self.reply_length is None:
                    quiet_until = time.monotonic() + self.quiet

    def _match(self, buf: bytes, cmd: int, final: bool) -> Reply | None:
        """Reply in ``buf``, or None if more bytes may still come."""
        n = len(buf)
        if self.reply_length in (None, 9) and n >= 9 and buf[1] == cmd:
            self.reply_length = 9
            return Reply(cmd, buf[2:9])
        if self.reply_length in (None, 8) and n >= 8 and buf[0] == cmd:
            if self.reply_length == 8 or final or n >= 9:
                self.reply_length = 8
                return Reply(cmd, buf[1:8])
        if n >= 9 or (final and n >= 8):
            self.transport.reset_input_buffer()
            raise SixPackError(f"unexpected reply to ${cmd:02X} from unit {self.address}: {buf.hex(' ')}")
        return None

    def raw(self, cmd: int, params: bytes = b"") -> Reply | None:
        """Send any command except the unit-changing ones in ``FORBIDDEN``."""
        if cmd in FORBIDDEN:
            raise ProtocolError(f"${cmd:02X} changes the unit itself (baud rate, address, reset, flash); refused")
        if cmd in QUERIES:
            return self.query(cmd, params)
        frame = encode(self.address, cmd, params)
        with self._lock:
            self.transport.write(frame)
        return None

    # -- queries -------------------------------------------------------------

    def unit_info(self) -> UnitInfo:
        """$43: firmware, reset flag (cleared by this read), temperature, serial."""
        return decode_unit_info(self.query(Cmd.UNIT_INFO, u8(self.response_address)))

    def position(self, motor: int) -> PositionReply:
        """$20: position, action and stop flag (the read clears the flag)."""
        reply = decode_position(self.query(Cmd.QUERY_POSITION, u8(check_motor(motor)) + u8(self.response_address)))
        self._check_echo(reply.motor, motor, Cmd.QUERY_POSITION)
        return reply

    def velocity(self, motor: int) -> VelocityReply:
        """$21: actual velocity value and action."""
        reply = decode_velocity(self.query(Cmd.QUERY_VELOCITY, u8(check_motor(motor)) + u8(self.response_address)))
        self._check_echo(reply.motor, motor, Cmd.QUERY_VELOCITY)
        return reply

    def activities(self) -> tuple[int, ...]:
        """$28 without delayed response: current action of all six motors."""
        return decode_activities(self.query(Cmd.QUERY_ACTIVITIES, u8(self.response_address) + u8(0)))

    def inputs(self, channel: int) -> InputsReply:
        """$30: analogue input of ``channel`` (0..5 motors, 6 external, 7 supply)."""
        channel = int(channel)
        if not 0 <= channel <= 7:
            raise ProtocolError(f"channel={channel} out of range 0..7")
        reply = decode_inputs(self.query(Cmd.QUERY_INPUTS, u8(channel) + u8(self.response_address)))
        self._check_echo(reply.channel, channel, Cmd.QUERY_INPUTS)
        return reply

    def _check_echo(self, got: int, sent: int, cmd: int) -> None:
        if got != sent:
            raise SixPackError(f"reply to ${cmd:02X} is for motor/channel {got}, asked for {sent}")

    # -- configuration (change only with the motors stopped, unless noted) ---

    def set_peak_current(self, motor: int, value: int) -> None:
        """$10. Shared by motor pairs 0+1, 2+3, 4+5; only even motor numbers count."""
        self.command(Cmd.PEAK_CURRENT, u8(check_motor(motor)) + u8(value))

    def set_current_control(self, motor: int, i0: int, i1: int, i2: int, i3: int, t0: int) -> None:
        """$11: current indices (0=100% .. 8=0%) for power-down, standstill,
        constant velocity and acceleration; power-down delay t0 in 2 ms units."""
        for name, index in (("i0", i0), ("i1", i1), ("i2", i2), ("i3", i3)):
            if not 0 <= index <= 8:
                raise ProtocolError(f"{name}={index} out of range 0..8")
        if not 1 <= t0 <= 0xFFFF:
            raise ProtocolError(f"t0={t0} out of range 1..65535")
        self.command(
            Cmd.CURRENT_CONTROL, u8(check_motor(motor)) + bytes([i0, i1, i2, i3]) + u16(t0)
        )

    def set_clock_divider(self, clkdiv: int) -> None:
        """$12: global for all motors."""
        if not 0 <= clkdiv <= 31:
            raise ProtocolError(f"clkdiv={clkdiv} out of range 0..31")
        self.command(Cmd.CLOCK_DIVIDER, u8(clkdiv))

    def set_start_velocity(self, motor: int, vmin: int, vstart: int, div: int) -> None:
        """$13: vmin (reference fine search, 0 = automatic), vstart, divider."""
        if not 0 <= div <= 3:
            raise ProtocolError(f"div={div} out of range 0..3")
        if not 0 <= vmin <= V_MAX or not 1 <= vstart <= V_MAX:
            raise ProtocolError(f"vmin={vmin} / vstart={vstart} out of range")
        self.command(Cmd.START_VELOCITY, u8(check_motor(motor)) + u16(vmin) + u16(vstart) + u8(div))

    def set_amax_vmax(self, motor: int, amax: int, vmax: int) -> None:
        """$14: may be changed while moving."""
        if not 1 <= amax <= 32767 or not 1 <= vmax <= V_MAX:
            raise ProtocolError(f"amax={amax} / vmax={vmax} out of range")
        self.command(Cmd.AMAX_VMAX, u8(check_motor(motor)) + u16(amax) + u16(vmax))

    def set_motor_parameters(self, motor: int, poslimit: int, home_method: int) -> None:
        """$15: position limit (microsteps) and spec-style home_method flags."""
        if not 0 <= poslimit <= 0x7FFFFFFF:
            raise ProtocolError(f"poslimit={poslimit} out of range")
        p5, p6 = split_home_method(home_method)
        self.command(Cmd.MOTOR_PARAMETERS, u8(check_motor(motor)) + i32(poslimit) + u8(p5) + u8(p6))

    def set_reference_parameters(self, motor: int, vrefmax: int, debounce: int, stop_after_search: bool) -> None:
        """$16: fast reference search velocity, de-bounce mask, stop after search."""
        if not 1 <= vrefmax <= V_MAX:
            raise ProtocolError(f"vrefmax={vrefmax} out of range 1..{V_MAX}")
        self.command(
            Cmd.REFERENCE_PARAMETERS,
            u8(check_motor(motor)) + u16(vrefmax) + u16(debounce) + u8(1 if stop_after_search else 0),
        )

    def set_null_offset(self, motor: int, null_offset: int, test_null_range: int) -> None:
        """$18: null point offset (microsteps) and test-null range."""
        self.command(Cmd.NULL_OFFSET, u8(check_motor(motor)) + i32(null_offset) + u16(test_null_range))

    def set_stop_limits(self, channel: int, min_left: int, max_right: int) -> None:
        """$31: analogue stop limits of a channel (0 = no left stop, >= 1023 = no right stop)."""
        if not 0 <= channel <= 7:
            raise ProtocolError(f"channel={channel} out of range 0..7")
        self.command(Cmd.STOP_LIMITS, u8(channel) + u16(min_left) + u16(max_right))

    # -- motion --------------------------------------------------------------

    def start_ramp(self, motor: int, target: int) -> None:
        """$23: trapezoidal ramp to ``target``. Ignored while the motor is
        active; ends a rotation (the motor only stops)."""
        self.command(Cmd.START_RAMP, u8(check_motor(motor)) + i32(target))

    def set_target(self, motor: int, target: int) -> None:
        """$26: change the target of a running ramp."""
        self.command(Cmd.SET_TARGET, u8(check_motor(motor)) + i32(target))

    def rotate(self, motor: int, velocity: int) -> None:
        """$25: constant rotation (-511..511), 0 decelerates to standstill."""
        if not -V_MAX <= velocity <= V_MAX:
            raise ProtocolError(f"rotation velocity={velocity} out of range")
        self.command(Cmd.START_ROTATION, u8(check_motor(motor)) + i16(velocity))

    def stop(self, mask: int) -> None:
        """$2A: set target = actual position of the masked motors. Motors
        faster than vstart overshoot and ramp back."""
        if not 0 <= mask <= 0x3F:
            raise ProtocolError(f"mask={mask:#x} out of range")
        self.command(Cmd.STOP_MOTORS, u8(mask))

    def set_position(self, motor: int, position: int) -> None:
        """$27: set the position counter."""
        self.command(Cmd.SET_POSITION, u8(check_motor(motor)) + i32(position))

    def reference_search(self, motor: int) -> None:
        """$22: start the reference search configured with $15/$16/$18."""
        self.command(Cmd.START_REFERENCE, u8(check_motor(motor)))

    def abort_reference(self, motor: int) -> None:
        """$2B."""
        self.command(Cmd.ABORT_REFERENCE, u8(check_motor(motor)))


__all__ = ["SixPack", "SixPackError", "SixPackTimeout", "Transport", "PARAM_LEN"]
