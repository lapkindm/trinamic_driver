"""Simulated SIXpack 2 for tests (behaviour as described in the manual).

Motion is simplified: ramps and rotations run at their full velocity from the
start (no acceleration). The simulator records every frame it receives, so
tests can check what was programmed.

Test hooks: a frame for address ``SIM_ADDRESS`` is not a command but pokes the
simulator: command 0x01 simulates a power cycle, 0x02 raises the stop
condition on motor P0 (as if the reference switch acted as a limit), 0x04
drops the next N replies (N = first parameter byte), 0x05 drops the next
reply to $43 (the unit still clears its reset flag, as a lost reply would).
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from trinamic_driver.protocol import (  # noqa: E402
    FRAME_LEN,
    NULL_LEFT,
    Action,
    Cmd,
    step_frequency,
)

SIM_ADDRESS = 0xFE
SIM_POWER_CYCLE = 0x01
SIM_STOP_CONDITION = 0x02
SIM_DROP_REPLIES = 0x04
SIM_DROP_UNIT_INFO = 0x05


@dataclass
class Motor:
    position: float = 0.0
    target: int = 0
    action: int = Action.INACTIVE
    stop_flag: bool = False
    rotation: int = 0  # velocity value while rotating
    vmax: int = 511
    vstart: int = 5
    vmin: int = 4
    div: int = 2
    amax: int = 128
    poslimit: int = 6400
    home_method: int = 0
    vrefmax: int = 100
    stop_after_search: bool = False
    null_offset: int = 0
    ref_return_to: int | None = None
    ref_until: float = 0.0
    params: dict = field(default_factory=dict)


class FakeSixPack:
    def __init__(self, address: int = 0, reply_length: int = 8, firmware: int = 166, serial: int = 5816,
                 reference_time: float = 0.3):
        self.address = address
        self.reply_length = reply_length
        self.firmware = firmware
        self.serial = serial
        self.reference_time = reference_time
        self.frames: list[bytes] = []  # every frame received for our address
        self.out = b""
        self._buf = b""
        self._t = time.monotonic()
        self.power_cycle(reset_flag=False)
        self.drop_next = 0  # replies to drop (lost on the line)
        self.drop_unit_info = 0

    # -- simulation ----------------------------------------------------------

    def power_cycle(self, reset_flag: bool = True) -> None:
        self.motors = [Motor() for _ in range(6)]
        self.clkdiv = 5
        self.reset_flag = reset_flag

    def _frequency(self, m: Motor, v: float) -> float:
        return step_frequency(v, m.div, self.clkdiv)

    def advance(self) -> None:
        now = time.monotonic()
        dt, self._t = now - self._t, now
        for m in self.motors:
            if m.action == Action.RAMP:
                step = self._frequency(m, m.vmax) * dt
                delta = m.target - m.position
                if abs(delta) <= step:
                    m.position = m.target
                    m.action = Action.INACTIVE
                else:
                    m.position += step if delta > 0 else -step
            elif m.action == Action.ROTATION:
                m.position += self._frequency(m, m.rotation) * dt
            elif 20 <= m.action <= 30 and now >= m.ref_until:
                m.position = m.null_offset
                if m.ref_return_to is not None and not m.stop_after_search:
                    m.target, m.action = m.ref_return_to, Action.RAMP
                else:
                    m.target, m.action = m.null_offset, Action.INACTIVE

    # -- byte interface ------------------------------------------------------

    def feed(self, data: bytes) -> None:
        self._buf += data
        while len(self._buf) >= FRAME_LEN:
            frame, self._buf = self._buf[:FRAME_LEN], self._buf[FRAME_LEN:]
            self.advance()
            self._handle(frame)

    def _reply(self, cmd: int, params: bytes, response_address: int) -> None:
        if self.drop_next > 0:
            self.drop_next -= 1
            return
        if cmd == 0x43 and self.drop_unit_info > 0:
            self.drop_unit_info -= 1
            return
        params = params.ljust(7, b"\0")
        head = bytes([response_address]) if self.reply_length == 9 else b""
        self.out += head + bytes([cmd]) + params

    def _handle(self, frame: bytes) -> None:
        address, cmd, p = frame[0], frame[1], frame[2:]
        if address == SIM_ADDRESS:
            if cmd == SIM_POWER_CYCLE:
                self.power_cycle()
            elif cmd == SIM_DROP_REPLIES:
                self.drop_next = p[0]
            elif cmd == SIM_DROP_UNIT_INFO:
                self.drop_unit_info = 1
            elif cmd == SIM_STOP_CONDITION:
                m = self.motors[p[0]]
                m.stop_flag = True
                m.target = round(m.position)
                m.action = Action.INACTIVE
            return
        if address != self.address:
            return
        self.frames.append(frame)
        m = self.motors[p[0]] if p[0] < 6 else None
        u16 = lambda i: struct.unpack("<H", p[i:i + 2])[0]  # noqa: E731
        i32 = lambda i: struct.unpack("<i", p[i:i + 4])[0]  # noqa: E731

        if cmd == Cmd.UNIT_INFO:
            self._reply(cmd, bytes([self.firmware, int(self.reset_flag), 41]) + struct.pack("<H", self.serial), p[0])
            self.reset_flag = False
        elif cmd == Cmd.QUERY_POSITION:
            self._reply(cmd, bytes([p[0]]) + struct.pack("<i", round(m.position)) + bytes([m.action, int(m.stop_flag)]),
                        p[1])
            m.stop_flag = False
        elif cmd == Cmd.QUERY_VELOCITY:
            v = m.rotation if m.action == Action.ROTATION else (m.vmax if m.action == Action.RAMP else 0)
            self._reply(cmd, bytes([p[0]]) + struct.pack("<h", v) + bytes([m.action]), p[1])
        elif cmd == Cmd.QUERY_ACTIVITIES:
            self._reply(cmd, bytes(mm.action for mm in self.motors), p[0])
        elif cmd == Cmd.QUERY_INPUTS:
            self._reply(cmd, bytes([p[0]]) + struct.pack("<H", 512) + bytes([0, 0, 0]), p[1])
        elif cmd == Cmd.PEAK_CURRENT:
            m.params["peak"] = p[1]
        elif cmd == Cmd.CURRENT_CONTROL:
            m.params["currents"] = (p[1], p[2], p[3], p[4], u16(5))
        elif cmd == Cmd.CLOCK_DIVIDER:
            self.clkdiv = p[0]
        elif cmd == Cmd.START_VELOCITY:
            m.vmin, m.vstart, m.div = u16(1), u16(3), p[5]
        elif cmd == Cmd.AMAX_VMAX:
            m.amax, m.vmax = u16(1), u16(3)
        elif cmd == Cmd.MOTOR_PARAMETERS:
            m.poslimit, m.home_method = i32(1), p[5] | p[6] << 8
        elif cmd == Cmd.REFERENCE_PARAMETERS:
            m.vrefmax, m.params["debounce"], m.stop_after_search = u16(1), u16(3), bool(p[5] & 1)
        elif cmd == Cmd.NULL_OFFSET:
            m.null_offset, m.params["test_null_range"] = i32(1), u16(5)
        elif cmd == Cmd.STOP_LIMITS:
            self.motors[p[0]].params["stop_limits"] = (u16(1), u16(3))
        elif cmd == Cmd.START_RAMP:
            if m.action == Action.ROTATION:
                m.action, m.rotation = Action.INACTIVE, 0  # "the motor is only stopped"
            elif m.action == Action.INACTIVE:
                m.target, m.action = i32(1), Action.RAMP
        elif cmd == Cmd.SET_TARGET:
            m.target = i32(1)
        elif cmd == Cmd.START_ROTATION:
            m.rotation = struct.unpack("<h", p[1:3])[0]
            m.action = Action.ROTATION
        elif cmd == Cmd.STOP_MOTORS:
            for i, mm in enumerate(self.motors):
                if p[0] & (1 << i) and mm.action == Action.RAMP:
                    mm.target = round(mm.position)
        elif cmd == Cmd.SET_POSITION:
            m.position = m.target = i32(1)
        elif cmd == Cmd.START_REFERENCE:
            m.ref_return_to = m.target if m.action == Action.INACTIVE else None
            m.action = Action.REFERENCE
            m.ref_until = time.monotonic() + self.reference_time
        elif cmd == Cmd.ABORT_REFERENCE:
            if 20 <= m.action <= 30:
                m.action, m.target = Action.INACTIVE, round(m.position)
        # anything else is ignored, like the real unit ignores bad frames


class FakeTransport:
    """In-memory Transport around a FakeSixPack."""

    def __init__(self, sim: FakeSixPack):
        self.sim = sim
        self.drop_replies = False

    def write(self, data: bytes) -> int:
        self.sim.feed(data)
        if self.drop_replies:
            self.sim.out = b""
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if not self.sim.out:
            time.sleep(0.002)
            return b""
        data, self.sim.out = self.sim.out[:size], self.sim.out[size:]
        return data

    def reset_input_buffer(self) -> None:
        self.sim.out = b""

    def close(self) -> None:
        pass


def null_left(home_method: int) -> bool:
    return bool(home_method & NULL_LEFT)
