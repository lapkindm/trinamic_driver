"""Trinamic SIXpack 2 binary protocol.

Written from the *SIXpack 2 Manual* (Trinamic, V1.01, May 5th 2006),
chapters 6.2 and 7 (``docs/Sixpack_2_manual.pdf``).

Every command is a 9-byte frame: module address, command byte and seven
parameter bytes P0..P6 (unused ones sent as 0). Multi-byte values are sent
least significant byte first. There is no checksum and set commands get no
reply; only queries answer, with the command byte, P0..P6.

Nothing here does I/O.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

FRAME_LEN = 9
PARAM_LEN = 7
F_CLK = 20e6  # Hz, SIXpack 2 clock
MAX_STEP_FREQUENCY = 200e3  # Hz, microstep frequency must not exceed this
NUM_MOTORS = 6
V_MAX = 511  # largest velocity value (vmax, vstart, rotation velocity)


class Cmd(IntEnum):
    """Commands used by this driver (manual chapter 7)."""

    PEAK_CURRENT = 0x10
    CURRENT_CONTROL = 0x11
    CLOCK_DIVIDER = 0x12
    START_VELOCITY = 0x13
    AMAX_VMAX = 0x14
    MOTOR_PARAMETERS = 0x15
    REFERENCE_PARAMETERS = 0x16
    NULL_OFFSET = 0x18
    QUERY_POSITION = 0x20
    QUERY_VELOCITY = 0x21
    START_REFERENCE = 0x22
    START_RAMP = 0x23
    START_ROTATION = 0x25
    SET_TARGET = 0x26
    SET_POSITION = 0x27
    QUERY_ACTIVITIES = 0x28
    STOP_MOTORS = 0x2A
    ABORT_REFERENCE = 0x2B
    QUERY_INPUTS = 0x30
    STOP_LIMITS = 0x31
    UNIT_INFO = 0x43


QUERIES = frozenset(
    {Cmd.QUERY_POSITION, Cmd.QUERY_VELOCITY, Cmd.QUERY_ACTIVITIES, Cmd.QUERY_INPUTS, Cmd.UNIT_INFO}
)

# Commands that change the unit itself (baud rate, address, power-down mode,
# hardware reset, flash and EEPROM service functions). The driver never sends
# them, and refuses them as raw commands.
FORBIDDEN = frozenset({0x40, 0x41, 0x42, 0x45, 0xCC, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7})


class Action(IntEnum):
    """Current action of a motor, reported by $20, $21 and $28."""

    INACTIVE = 0
    RAMP = 5
    PI_CONTROLLER = 10
    ROTATION = 15
    REFERENCE = 20  # 20..29: reference switch search
    MECHANICAL_REFERENCE = 30


def is_referencing(action: int) -> bool:
    return 20 <= action <= 30


def action_name(action: int) -> str:
    if is_referencing(action):
        return "mechanical reference" if action == 30 else f"reference search ({action})"
    try:
        return Action(action).name.lower().replace("_", " ")
    except ValueError:
        return f"unknown ({action})"


class ProtocolError(Exception):
    """Malformed frame or value out of range."""


def _check(name: str, value: int, low: int, high: int) -> int:
    value = int(value)
    if not low <= value <= high:
        raise ProtocolError(f"{name}={value} out of range {low}..{high}")
    return value


def check_motor(motor: int) -> int:
    return _check("motor", motor, 0, NUM_MOTORS - 1)


def u8(value: int) -> bytes:
    return struct.pack("<B", _check("byte", value, 0, 0xFF))


def u16(value: int) -> bytes:
    return struct.pack("<H", _check("u16", value, 0, 0xFFFF))


def i16(value: int) -> bytes:
    return struct.pack("<h", _check("i16", value, -0x8000, 0x7FFF))


def i32(value: int) -> bytes:
    return struct.pack("<i", _check("i32", value, -0x80000000, 0x7FFFFFFF))


def encode(address: int, cmd: int, params: bytes = b"") -> bytes:
    """Build a 9-byte frame; params are padded with zeros to 7 bytes."""
    if len(params) > PARAM_LEN:
        raise ProtocolError(f"{len(params)} parameter bytes, at most {PARAM_LEN} allowed")
    return bytes([_check("address", address, 0, 255), _check("command", cmd, 0, 255)]) + params.ljust(
        PARAM_LEN, b"\0"
    )


@dataclass(frozen=True)
class Reply:
    cmd: int
    params: bytes  # P0..P6

    def __post_init__(self):
        if len(self.params) != PARAM_LEN:
            raise ProtocolError(f"reply has {len(self.params)} parameter bytes, expected {PARAM_LEN}")


# -- decoded replies ---------------------------------------------------------


@dataclass(frozen=True)
class PositionReply:
    """$20: position (microsteps), current action, stop condition flag.

    The stop flag is cleared in the unit by the read, so whoever reads it
    must keep it.
    """

    motor: int
    position: int
    action: int
    stop: bool


@dataclass(frozen=True)
class VelocityReply:
    """$21: actual velocity value (-511..511) and current action."""

    motor: int
    velocity: int
    action: int


@dataclass(frozen=True)
class InputsReply:
    """$30: analogue value of one channel and the reference inputs.

    The manual says the reference inputs are reported inverted; ``reference``
    and ``all_references`` are the raw bits.
    """

    channel: int
    analog: int  # 0..1023; channel 7 is the supply voltage (22 per volt)
    reference: bool  # reference input of this channel (raw bit)
    all_references: int  # bit n = motor n, bit 6/7 = jumpers
    ttlio1: bool


@dataclass(frozen=True)
class UnitInfo:
    """$43. The reset flag is 1 on the first read after a reset, then 0."""

    firmware: int  # e.g. 166 = "1.66"
    reset: bool
    temperature: int  # degC
    serial: int

    @property
    def firmware_str(self) -> str:
        return f"{self.firmware // 100}.{self.firmware % 100:02d}"


def decode_position(reply: Reply) -> PositionReply:
    p = reply.params
    return PositionReply(motor=p[0], position=struct.unpack("<i", p[1:5])[0], action=p[5], stop=bool(p[6] & 1))


def decode_velocity(reply: Reply) -> VelocityReply:
    p = reply.params
    return VelocityReply(motor=p[0], velocity=struct.unpack("<h", p[1:3])[0], action=p[3])


def decode_activities(reply: Reply) -> tuple[int, ...]:
    return tuple(reply.params[:NUM_MOTORS])


def decode_inputs(reply: Reply) -> InputsReply:
    p = reply.params
    return InputsReply(
        channel=p[0],
        analog=struct.unpack("<H", p[1:3])[0],
        reference=bool(p[3] & 1),
        all_references=p[4],
        ttlio1=bool(p[5] & 1),
    )


def decode_unit_info(reply: Reply) -> UnitInfo:
    p = reply.params
    return UnitInfo(
        firmware=p[0],
        reset=bool(p[1]),
        temperature=struct.unpack("<b", p[2:3])[0],
        serial=struct.unpack("<H", p[3:5])[0],
    )


# -- velocity and acceleration -----------------------------------------------
#
#   f_microstep = F_CLK / (clkdiv + 1) * v / 2**(14 + div)       (manual 6.3.1)
#
# Checked against the manual's defaults (clkdiv=5): v=511, div=2 -> 26 kHz;
# vstart=5, div=2 -> 254 Hz; vmin=4, div=3 -> 102 Hz.
# During acceleration amax/64 is added to v every 2 ms (500 Hz), so v rises
# by amax * 500/64 per second.


def step_frequency(v: float, div: int, clkdiv: int) -> float:
    """Microstep frequency (steps/s) of velocity value ``v``."""
    return F_CLK / (clkdiv + 1) * v / 2 ** (14 + div)


def velocity_value(frequency: float, div: int, clkdiv: int) -> float:
    """Velocity value for a microstep frequency (not rounded)."""
    return frequency / step_frequency(1, div, clkdiv)


def amax_value(acceleration: float, div: int, clkdiv: int) -> float:
    """amax for an acceleration in steps/s^2 (not rounded)."""
    return acceleration / step_frequency(1, div, clkdiv) * 64 / 500


def acceleration_of(amax: float, div: int, clkdiv: int) -> float:
    """Acceleration (steps/s^2) of an amax value."""
    return amax * 500 / 64 * step_frequency(1, div, clkdiv)


def choose_div(max_frequency: float, clkdiv: int) -> int:
    """Finest divider (largest div, best velocity resolution) that still
    reaches ``max_frequency`` with v <= 511."""
    for div in (3, 2, 1, 0):
        if velocity_value(max_frequency, div, clkdiv) <= V_MAX:
            return div
    raise ProtocolError(
        f"{max_frequency:g} steps/s is above the fastest velocity "
        f"{step_frequency(V_MAX, 0, clkdiv):g} steps/s at clkdiv={clkdiv}"
    )


def max_vstart(div: int) -> int:
    """Largest vstart the firmware accepts: 0 < vstart < (512 >> (3-div)) - 1."""
    return (512 >> (3 - div)) - 2


# -- currents ------------------------------------------------------------------

# $11 current indices 0..8 in percent of the peak current.
CURRENT_PERCENT = (100, 75, 50, 38, 25, 19, 13, 9, 0)


def current_index(percent: float) -> int:
    """Index of the nearest $11 current step."""
    return min(range(len(CURRENT_PERCENT)), key=lambda i: abs(CURRENT_PERCENT[i] - percent))


def peak_current_value(percent: float) -> int:
    """$10 value (0..255) for a peak current in percent of the DIP-switch maximum.

    50 % -> 128, the manual's default.
    """
    return max(0, min(255, round(percent * 256 / 100)))


def debounce_mask(ms: float) -> int:
    """$16 reference switch de-bounce mask: $0001 = 0 ms, $0003 = 2 ms, ... $FFFF = 30 ms."""
    bits = round(_check("debouncing time (ms)", round(ms), 0, 30) / 2) + 1
    return (1 << bits) - 1


# -- motor type flags ($15 P5, P6) ---------------------------------------------
#
# spec's 16-bit "home_method" packs P5 into the low byte and P6 into the high
# byte (spec help "trinamic"); bit 0 (PIMode) is always 0 here.

FLAG_NAMES = {
    0x0002: "RotaryAxis",
    0x0004: "AutoNullCmd",
    0x0008: "TestNull",
    0x0010: "NullLeft",
    0x0020: "NullCenter",
    0x0040: "StopNull",
    0x0080: "FilterSwitch",
    0x0100: "WayOptimization",
    0x0200: "FastRef",
    0x0400: "MechRef",
    0x0800: "DelayTestNull",
    0x1000: "StopSoft",
    0x2000: "StopNoRef",
    0x4000: "NullPositive",
    0x8000: "StopAtFullSteps",
}
NULL_LEFT = 0x0010
STOP_NULL = 0x0040
SPEC_DEFAULT_HOME_METHOD = 0x60D0  # NullLeft|StopNull|FilterSwitch|StopNoRef|NullPositive (spec's *intended* values)
# Power-on default of the unit (manual 6.2.1.8: "motortype= Delayedtest0 | NullCenter |
# Filterswitch | FastRef"). spec never sends $15, so this is what the XRD-3003TT runs with.
POWER_ON_HOME_METHOD = 0x0AA0


def split_home_method(home_method: int) -> tuple[int, int]:
    """(P5, P6) of $15 for a spec-style home_method word."""
    home_method = _check("home_method", home_method, 0, 0xFFFF)
    if home_method & 1:
        raise ProtocolError("home_method bit 0 (PI controller mode) is not supported")
    return home_method & 0xFF, home_method >> 8


def describe_home_method(home_method: int) -> str:
    return "|".join(name for bit, name in FLAG_NAMES.items() if home_method & bit) or "none"
