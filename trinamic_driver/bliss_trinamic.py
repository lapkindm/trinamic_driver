"""Bliss motor controller for the Trinamic SIXpack 2.

One controller per unit (serial line + module address); each axis is one of
the six motor channels. Example YAML::

    - class: Trinamic
      package: trinamic_driver.bliss_trinamic
      serial:
        url: /dev/ttyS9
        baudrate: 19200
      module: 0              # unit address (rotary switches)
      clkdiv: 5              # global clock divider (manual default)
      check_interval: 2      # s; check for a unit reset at most this often (0: off)
      # timeout: 0.5         # s to wait for a reply
      # reply_length: 8      # 8 or 9; detected from the first reply if omitted
      axes:
        - name: m1
          channel: 2                 # motor 0..5
          steps_per_unit: 5400       # microsteps per user unit
          velocity: 1.0              # user units/s
          acceleration: 8.0          # user units/s^2
          base_velocity: 0.037       # start/stop velocity (spec's base rate)
          # Trinamic parameters, as spec's motor_par() names them; the
          # defaults are spec's defaults (spec help "trinamic"):
          peak_current: 50           # % of the DIP-switch maximum (shared by
                                     # channel pairs 0+1, 2+3, 4+5)
          acceleration_current: 100  # % of peak current
          running_current: 75
          standing_current: 50
          power_down_current: 0
          power_down_delay: 1000     # ms
          home_method: 0x60D0        # $15 flags, spec's 16-bit encoding
          pos_limit: 400000          # microsteps, for the reference search
          debouncing_time: 22        # ms
          stop_after_search: true    # stay at the reference point after home()
          null_offset: 0             # microsteps
          test_null_range: 240       # microsteps
          home_velocity: 0.5         # user units/s for the fast reference search
          ignore_limits: false       # spec's disable_limit_checks

The unit cannot report its settings and forgets them on a power cycle; the
controller programs them when an axis initialises and again after a reset is
detected (reset flag of $43). Such a reset also loses the positions: the axes
then report FAULT until they are homed or their dial is set.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace

import gevent.lock

from bliss import global_map
from bliss.comm.exceptions import CommunicationTimeout
from bliss.comm.util import SERIAL, get_comm
from bliss.common.axis.state import AxisState
from bliss.common.logtools import log_debug, log_warning
from bliss.common.utils import object_method
from bliss.config.settings import HashSetting
from bliss.controllers.motor import Controller

from .protocol import (
    NULL_LEFT,
    SPEC_DEFAULT_HOME_METHOD,
    V_MAX,
    Action,
    UnitInfo,
    acceleration_of,
    action_name,
    amax_value,
    choose_div,
    current_index,
    debounce_mask,
    describe_home_method,
    is_referencing,
    max_vstart,
    peak_current_value,
    split_home_method,
    step_frequency,
    velocity_value,
)
from .sixpack import SixPack, SixPackError

# spec defaults (spec help "trinamic", motor_par(mne, "dump"))
DEFAULTS = {
    "peak_current": 50.0,
    "acceleration_current": 100.0,
    "running_current": 75.0,
    "standing_current": 50.0,
    "power_down_current": 0.0,
    "power_down_delay": 1000.0,
    "home_method": SPEC_DEFAULT_HOME_METHOD,
    "pos_limit": 400000,
    "debouncing_time": 22.0,
    "stop_after_search": True,
    "null_offset": 0,
    "test_null_range": 240,
    "ignore_limits": False,
}


class BlissSerialTransport:
    """Adapts a ``bliss.comm`` serial object to the SixPack transport interface."""

    def __init__(self, comm, poll: float = 0.02):
        self._comm = comm
        self._poll = poll

    def write(self, data: bytes) -> None:
        self._comm.write(data)

    def read(self, size: int = 1) -> bytes:
        try:
            return self._comm.raw_read(size, timeout=self._poll)
        except CommunicationTimeout:
            return b""

    def reset_input_buffer(self) -> None:
        self._comm.flush()


def _as_int(value) -> int:
    """YAML may give 0x60D0 as int or as the string '0x60D0'."""
    return int(value, 0) if isinstance(value, str) else int(value)


@dataclass
class _AxisData:
    name: str
    channel: int
    steps_per_unit: float
    div: int
    vstart: int
    peak: int  # $10 value
    currents: tuple  # i0, i1, i2, i3 indices
    t0: int  # power-down delay, 2 ms units
    home_method: int
    pos_limit: int
    debounce: int
    stop_after_search: bool
    null_offset: int
    test_null_range: int
    home_frequency: float  # steps/s
    ignore_limits: bool
    stop_limits: tuple | None = None  # $31 (min_left, max_right), as spec sends
    vmax: int = 0
    amax: int = 0
    velocity_setpoint: float | None = None  # steps/s as requested by Bliss
    acceleration_setpoint: float | None = None
    stop_latched: bool = False  # $20 stop flag, kept until the next move
    direction: int = 0  # of the last move, for LIMNEG/LIMPOS
    jog_stopping: bool = False
    last_steps: int | None = None  # position at the last read (reset check)


class Trinamic(Controller):
    """Bliss controller for one Trinamic SIXpack 2."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._comm = None
        self._pack: SixPack | None = None
        self._data_by_name: dict[str, _AxisData] = {}
        self._clkdiv = 5
        self._check_interval = 2.0
        self._last_check = 0.0
        self._lost = None  # HashSetting: axis name -> 1 while its position is unknown

    # -- initialisation ----------------------------------------------------

    def initialize(self):
        cfg = self.config
        serial_cfg = cfg.config_dict.get("serial", {}) or {}
        url = str(serial_cfg.get("url", ""))
        if url.startswith("/"):
            # Local device: pyserial directly. bliss.comm's Serial lost the
            # first bytes of replies from the ViX drives on this instrument
            # (2026-10-06); pyserial never did, also inside Bliss (gevent
            # patches select, so reads stay cooperative). The SIXpack has no
            # acknowledgements, so a silently damaged frame must not happen.
            import serial

            self._comm = serial.Serial(url, int(serial_cfg.get("baudrate", 19200)), timeout=0.02)
            transport = self._comm
        else:  # ser2net://, rfc2217://, tango://
            self._comm = get_comm(cfg.config_dict, ctype=SERIAL, baudrate=19200, timeout=1.0)
            transport = BlissSerialTransport(self._comm)
        reply_length = cfg.get("reply_length", int)
        self._pack = SixPack(
            transport,
            address=cfg.get("module", int, 0),
            timeout=cfg.get("timeout", float, 0.5),
            reply_length=reply_length,
            lock=gevent.lock.RLock(),
        )
        self._clkdiv = cfg.get("clkdiv", int, 5)
        if not 0 <= self._clkdiv <= 31:
            raise ValueError(f"{self.name}: clkdiv must be 0..31")
        self._check_interval = cfg.get("check_interval", float, 2.0)
        # False: send only what spec sends (clock divider, speeds, stop limits)
        # and leave currents, motor flags ($15), reference search and null
        # offset at the unit's power-on defaults.
        self._program_all = cfg.get("program_parameters", bool, True)
        self._lost = HashSetting(f"{self.name}:positions_lost")
        # All axes from the config, so that a reset can be handled even for
        # axes this process has not initialised.
        for node in cfg.config_dict.get("axes", []):
            data = self._axis_data_from_config(node)
            self._data_by_name[data.name] = data
        self._check_peak_pairs()
        if not url.startswith("/"):
            global_map.register(self, children_list=[self._comm])

    def _axis_data_from_config(self, node) -> _AxisData:
        name = node.get("name")

        def get(key, conv=float):
            value = node.get(key, DEFAULTS.get(key))
            return None if value is None else conv(value)

        channel = node.get("channel")
        if channel is None or not 0 <= int(channel) <= 5:
            raise ValueError(f"{name}: 'channel' (motor 0..5) is required")
        spu = abs(float(node.get("steps_per_unit")))
        velocity = abs(float(node.get("velocity"))) * spu
        home_velocity = node.get("home_velocity")
        home_frequency = abs(float(home_velocity)) * spu if home_velocity is not None else velocity
        div = node.get("div")
        div = int(div) if div is not None else choose_div(max(velocity, home_frequency), self._clkdiv)
        base = node.get("base_velocity")
        base_frequency = abs(float(base)) * spu if base is not None else 200.0  # spec base rate
        vstart = max(1, min(max_vstart(div), round(velocity_value(base_frequency, div, self._clkdiv))))
        currents = tuple(
            current_index(get(key))
            for key in ("power_down_current", "standing_current", "running_current", "acceleration_current")
        )
        home_method = _as_int(node.get("home_method", DEFAULTS["home_method"]))
        split_home_method(home_method)  # validates
        return _AxisData(
            name=name,
            channel=int(channel),
            steps_per_unit=spu,
            div=div,
            vstart=vstart,
            peak=peak_current_value(get("peak_current")),
            currents=currents,
            t0=max(1, round(get("power_down_delay") / 2)),
            home_method=home_method,
            pos_limit=get("pos_limit", int),
            debounce=debounce_mask(get("debouncing_time")),
            stop_after_search=get("stop_after_search", bool),
            null_offset=get("null_offset", int),
            test_null_range=get("test_null_range", int),
            home_frequency=home_frequency,
            ignore_limits=get("ignore_limits", bool),
            stop_limits=tuple(int(v) for v in node.get("stop_limits")) if node.get("stop_limits") else None,
        )

    def _check_peak_pairs(self) -> None:
        by_pair: dict[int, _AxisData] = {}
        channels = set()
        for data in self._data_by_name.values():
            if data.channel in channels:
                raise ValueError(f"{self.name}: channel {data.channel} used by more than one axis")
            channels.add(data.channel)
            other = by_pair.setdefault(data.channel // 2, data)
            if other.peak != data.peak:
                raise ValueError(
                    f"{self.name}: {other.name} and {data.name} share a peak current "
                    f"(channels {data.channel // 2 * 2}+{data.channel // 2 * 2 + 1}) "
                    "but have different peak_current"
                )

    def initialize_hardware(self):
        info = self._unit_info()  # also handles a pending reset flag
        log_debug(self, "SIXpack 2 firmware %s, sn %d, %d C", info.firmware_str, info.serial, info.temperature)
        self._require_stopped("set the clock divider")
        self._pack.set_clock_divider(self._clkdiv)

    def initialize_axis(self, axis):
        if axis.name not in self._data_by_name:
            raise ValueError(f"{axis.name}: not found in the controller config")

    def initialize_hardware_axis(self, axis):
        data = self._data(axis)
        self._require_stopped(f"program {axis.name}")
        self._program(data)

    def close(self):
        if self._comm is not None:
            self._comm.close()

    def _data(self, axis) -> _AxisData:
        return self._data_by_name[axis.name]

    # -- programming the unit ----------------------------------------------

    def _require_stopped(self, what: str) -> None:
        active = [(i, a) for i, a in enumerate(self._pack.activities()) if a != Action.INACTIVE]
        if active:
            busy = ", ".join(f"motor {i}: {action_name(a)}" for i, a in active)
            raise RuntimeError(f"{self.name}: cannot {what} while motors are active ({busy})")

    def _vmax_amax(self, data: _AxisData) -> tuple[int, int]:
        vmax = data.vmax or self._velocity_value(data, self._config_velocity(data))
        amax = data.amax or self._amax(data, self._config_acceleration(data), vmax)
        return vmax, amax

    def _config_velocity(self, data: _AxisData) -> float:
        node = self._axis_node(data.name)
        return abs(float(node.get("velocity"))) * data.steps_per_unit

    def _config_acceleration(self, data: _AxisData) -> float:
        node = self._axis_node(data.name)
        return abs(float(node.get("acceleration"))) * data.steps_per_unit

    def _axis_node(self, name: str):
        for node in self.config.config_dict.get("axes", []):
            if node.get("name") == name:
                return node
        raise KeyError(name)

    def _program(self, data: _AxisData) -> None:
        """Send the settings of one channel (motors must be stopped).

        With ``program_parameters: false`` only speeds and stop limits, as spec.
        """
        pack, ch = self._pack, data.channel
        vmax, amax = self._vmax_amax(data)
        if data.stop_limits is not None:
            pack.set_stop_limits(ch, *data.stop_limits)
        if self._program_all:
            pack.set_peak_current(ch & ~1, data.peak)
            pack.set_current_control(ch, *data.currents, data.t0)
        pack.set_start_velocity(ch, 0, data.vstart, data.div)
        pack.set_amax_vmax(ch, amax, vmax)
        if self._program_all:
            pack.set_motor_parameters(ch, data.pos_limit, data.home_method)
            pack.set_reference_parameters(ch, self._vrefmax(data, vmax), data.debounce, data.stop_after_search)
            pack.set_null_offset(ch, data.null_offset, data.test_null_range)
        data.vmax, data.amax = vmax, amax
        log_debug(
            self, "%s: channel %d programmed: div=%d vstart=%d vmax=%d amax=%d%s",
            data.name, ch, data.div, data.vstart, vmax, amax,
            f" home_method={describe_home_method(data.home_method)}" if self._program_all else " (speeds only)",
        )

    def _vrefmax(self, data: _AxisData, vmax: int) -> int:
        """511 >= vmax >= vrefmax >= vstart (manual, $16)."""
        v = round(velocity_value(data.home_frequency, data.div, self._clkdiv))
        return max(data.vstart, min(v, vmax))

    # -- reset detection ---------------------------------------------------

    def _unit_info(self) -> UnitInfo:
        """$43. Reading it clears the unit's reset flag, so every read goes
        through here and acts on it.

        If a reply is lost, the flag may have gone with it: ask again, and
        also look for the other sign of a reset, motor positions back at 0."""
        try:
            info = self._pack.unit_info()
        except SixPackError as exc:
            log_warning(self, "%s: no valid reply to $43 (%s); asking again and checking the positions",
                        self.name, exc)
            info = self._pack.unit_info()
            if not info.reset and self._positions_look_reset():
                info = replace(info, reset=True)
        self._last_check = time.monotonic()
        if info.reset:
            self._handle_reset()
        return info

    def _positions_look_reset(self) -> bool:
        """After a reset every motor position is 0: True if a motor that was
        somewhere else at its last read now reads 0."""
        for data in self._data_by_name.values():
            if data.last_steps not in (None, 0) and self._pack.position(data.channel).position == 0:
                return True
        return False

    def _check_reset(self) -> None:
        if self._check_interval > 0 and time.monotonic() - self._last_check >= self._check_interval:
            self._unit_info()

    def _handle_reset(self) -> None:
        names = list(self._data_by_name)
        log_warning(
            self,
            "%s: the SIXpack was reset (power cycle?): positions of %s are lost. "
            "Re-programming; home the axes or set their dial before moving.",
            self.name, ", ".join(names),
        )
        for name in names:
            self._lost[name] = 1
        self._pack.set_clock_divider(self._clkdiv)
        for data in self._data_by_name.values():
            data.stop_latched = data.jog_stopping = False
            self._program(data)

    def _is_lost(self, axis) -> bool:
        return self._lost.get(axis.name) is not None

    def _found(self, axis) -> None:
        self._lost.remove(axis.name)

    # -- state & position --------------------------------------------------

    def _read(self, data: _AxisData):
        reply = self._pack.position(data.channel)
        if reply.stop:
            data.stop_latched = True
        data.last_steps = reply.position
        return reply

    def state(self, axis):
        self._check_reset()
        data = self._data(axis)
        reply = self._read(data)
        if self._is_lost(axis):
            # Not FAULT: Bliss refuses home() on a FAULT axis, and homing is
            # how the position is recovered. Moves are refused in prepare_move.
            state = AxisState("MOVING" if reply.action != Action.INACTIVE else "READY")
            state.create_state("POSLOST", "position lost (unit reset): home or set dial")
            state.set("POSLOST")
            return state
        if reply.action == Action.ROTATION and data.jog_stopping:
            if self._pack.velocity(data.channel).velocity == 0:
                # $23 ends a rotation ("the motor is only stopped"), leaving it inactive
                self._pack.start_ramp(data.channel, reply.position)
                data.jog_stopping = False
            return AxisState("MOVING")
        if reply.action != Action.INACTIVE:
            return AxisState("MOVING")
        state = AxisState("READY")
        if data.stop_latched:
            if data.ignore_limits:
                log_warning(self, "%s: stop condition reported (ignored, ignore_limits)", axis.name)
                data.stop_latched = False
            else:
                state.set("LIMNEG" if data.direction < 0 else "LIMPOS")
        return state

    def read_position(self, axis):
        self._check_reset()
        return float(self._read(self._data(axis)).position)

    def set_position(self, axis, new_position):
        data = self._data(axis)
        self._pack.set_position(data.channel, int(round(new_position)))
        self._found(axis)
        data.stop_latched = False
        return float(self._read(data).position)

    # -- velocity & acceleration -------------------------------------------

    def _velocity_value(self, data: _AxisData, frequency: float) -> int:
        v = round(velocity_value(abs(frequency), data.div, self._clkdiv))
        if not 1 <= v <= V_MAX:
            fastest = step_frequency(V_MAX, data.div, self._clkdiv) / data.steps_per_unit
            raise ValueError(
                f"{data.name}: velocity {abs(frequency) / data.steps_per_unit:g} is outside the range "
                f"of divider div={data.div} (max {fastest:g} units/s); set 'div' in the config to go faster"
            )
        return v

    def _amax(self, data: _AxisData, acceleration: float, vmax: int) -> int:
        amax = round(amax_value(abs(acceleration), data.div, self._clkdiv))
        limit = min(32767, data.vstart * 64)  # 0 < amax <= vstart * 64
        if not 1 <= amax <= limit:
            clipped = max(1, min(amax, limit))
            log_warning(
                self, "%s: acceleration %g units/s^2 limited to %g (amax %d, vstart %d)", data.name,
                abs(acceleration) / data.steps_per_unit,
                acceleration_of(clipped, data.div, self._clkdiv) / data.steps_per_unit, clipped, data.vstart,
            )
            amax = clipped
        return amax

    def read_velocity(self, axis):
        data = self._data(axis)
        vmax, _ = self._vmax_amax(data)
        if data.velocity_setpoint is not None and self._velocity_value(data, data.velocity_setpoint) == vmax:
            return data.velocity_setpoint
        return step_frequency(vmax, data.div, self._clkdiv)

    def set_velocity(self, axis, new_velocity):
        data = self._data(axis)
        vmax = self._velocity_value(data, new_velocity)
        _, amax = self._vmax_amax(data)
        self._pack.set_amax_vmax(data.channel, amax, vmax)
        data.vmax, data.velocity_setpoint = vmax, abs(new_velocity)

    def read_acceleration(self, axis):
        data = self._data(axis)
        _, amax = self._vmax_amax(data)
        if (data.acceleration_setpoint is not None
                and round(amax_value(data.acceleration_setpoint, data.div, self._clkdiv)) == amax):
            return data.acceleration_setpoint
        return acceleration_of(amax, data.div, self._clkdiv)

    def set_acceleration(self, axis, new_acc):
        data = self._data(axis)
        vmax, _ = self._vmax_amax(data)
        amax = self._amax(data, new_acc, vmax)
        self._pack.set_amax_vmax(data.channel, amax, vmax)
        data.amax, data.acceleration_setpoint = amax, abs(new_acc)

    # -- motion ------------------------------------------------------------

    def _check_movable(self, axis) -> None:
        self._check_reset()
        if self._is_lost(axis):
            raise RuntimeError(
                f"{axis.name}: the SIXpack was reset and the position is lost; "
                f"home the axis ({axis.name}.home()) or set its dial at a known position"
            )

    def check_ready_to_move(self, axis, state):
        # A jog on a lost axis must be refused here: an exception from
        # start_jog() is raised in a Bliss background task and leaves the jog
        # motion hanging. (Moves are refused in prepare_move with a clearer
        # message; home() must stay possible.)
        if self._is_lost(axis) and self._starting_jog(axis):
            log_warning(self, "%s: position lost (unit reset): home or set the dial before jogging", axis.name)
            return False
        return super().check_ready_to_move(axis, state)

    @staticmethod
    def _starting_jog(axis) -> bool:
        """True while Bliss checks readiness for a jog: GroupMove.start()
        assigns axis._group_move before calling check_ready_to_move()."""
        group_move = getattr(axis, "_group_move", None)
        motions_dict = getattr(group_move, "motions_dict", None) or {}
        return any(
            motion.axis is axis and motion.type == "jog"
            for motions in motions_dict.values()
            for motion in motions
        )

    def prepare_move(self, motion):
        axis = motion.axis
        self._check_movable(axis)
        data = self._data(axis)
        current = self._read(data).position
        data.stop_latched = False
        data.direction = 1 if motion.target_pos > current else -1

    def start_one(self, motion):
        self._pack.start_ramp(self._data(motion.axis).channel, int(round(motion.target_pos)))

    def start_all(self, *motions):
        started = []
        try:
            for motion in motions:
                self.start_one(motion)
                started.append(motion)
        except BaseException:
            self.stop_all(*started)
            raise

    def stop(self, axis):
        data = self._data(axis)
        action = self._read(data).action
        if action == Action.ROTATION:
            self._pack.rotate(data.channel, 0)  # decelerates with amax
            data.jog_stopping = True
        elif is_referencing(action):
            self._pack.abort_reference(data.channel)
        else:
            # target := actual position; a fast motor overshoots and ramps back
            self._pack.stop(1 << data.channel)

    def stop_all(self, *motions):
        for motion in motions:
            try:
                self.stop(motion.axis)
            except SixPackError as exc:
                log_warning(self, "stop failed on %s: %s", motion.axis.name, exc)

    def start_jog(self, axis, velocity, direction):
        self._check_movable(axis)
        data = self._data(axis)
        v = self._velocity_value(data, velocity)
        data.stop_latched = data.jog_stopping = False
        data.direction = 1 if direction > 0 else -1
        self._pack.rotate(data.channel, v * data.direction)

    def stop_jog(self, axis):
        data = self._data(axis)
        self._pack.rotate(data.channel, 0)
        data.jog_stopping = True

    def limit_search(self, axis, limit):
        raise NotImplementedError(
            f"{axis.name}: the SIXpack 2 has no limit search; use {axis.name}.home() (reference search)"
        )

    # -- homing (reference search) -----------------------------------------

    def home_search(self, axis, switch):
        data = self._data(axis)
        vmax, _ = self._vmax_amax(data)
        null_left = bool(data.home_method & NULL_LEFT)
        if (switch > 0) == null_left:
            log_warning(
                self, "%s: the reference search direction is set by home_method (NullLeft=%d), "
                "not by home(%+d); searching %s", axis.name, null_left, switch,
                "negative" if null_left else "positive",
            )
        if self._program_all:
            self._pack.set_reference_parameters(
                data.channel, self._vrefmax(data, vmax), data.debounce, data.stop_after_search
            )
        data.stop_latched = data.jog_stopping = False
        self._pack.reference_search(data.channel)

    def home_state(self, axis):
        data = self._data(axis)
        reply = self._read(data)
        if reply.action != Action.INACTIVE:
            return AxisState("MOVING")
        self._found(axis)
        data.stop_latched = False
        return AxisState("READY")

    # -- information & raw access ------------------------------------------

    def get_id(self, axis):
        info = self._unit_info()
        return (f"Trinamic SIXpack 2 firmware {info.firmware_str}, sn {info.serial}, "
                f"unit {self._pack.address}, motor {self._data(axis).channel}")

    def get_axis_info(self, axis):
        data = self._data(axis)
        info = self._unit_info()
        vmax, amax = self._vmax_amax(data)
        reply = self._read(data)
        lines = [
            f"TRINAMIC SIXPACK 2: unit {self._pack.address}, firmware {info.firmware_str}, "
            f"sn {info.serial}, {info.temperature} C",
            f"    motor {data.channel}: {action_name(reply.action)}, position {reply.position} steps"
            + (", POSITION LOST (home or set dial)" if self._is_lost(axis) else ""),
            f"    clkdiv {self._clkdiv}, div {data.div}, vstart {data.vstart}, vmax {vmax}, amax {amax}",
            f"    peak {data.peak}/255, currents (pd, stand, run, acc) "
            + ", ".join(f"{(100, 75, 50, 38, 25, 19, 13, 9, 0)[i]}%" for i in data.currents)
            + f", power-down delay {data.t0 * 2} ms",
            f"    home_method {data.home_method:#06x} ({describe_home_method(data.home_method)})"
            + ("" if self._program_all else "  [currents/flags: config values, not sent (program_parameters: false)]"),
            f"    pos_limit {data.pos_limit}, null_offset {data.null_offset}, "
            f"test_null_range {data.test_null_range}, debounce {data.debounce:#06x}",
        ]
        return "\n".join(lines)

    @staticmethod
    def _parse_raw(com: str) -> tuple[int, bytes]:
        values = [int(x, 16) for x in com.replace(",", " ").split()]
        if not values:
            raise ValueError("give the command and parameter bytes in hex, e.g. '20 02 00'")
        return values[0], bytes(values[1:])

    def raw_write(self, com):
        """Send e.g. ``"23 02 E8 03 00 00"`` (command, then P0.. in hex)."""
        cmd, params = self._parse_raw(com)
        self._pack.raw(cmd, params)

    def raw_write_read(self, com):
        """Query e.g. ``"20 02 00"``; returns P0..P6 of the reply in hex."""
        cmd, params = self._parse_raw(com)
        if cmd == 0x43:
            raise ValueError("use get_id()/get_axis_info() for $43 (reading it clears the reset flag)")
        reply = self._pack.raw(cmd, params)
        if reply is None:
            raise ValueError(f"${cmd:02X} is not a query")
        return reply.params.hex(" ")

    @object_method(types_info=("None", "str"))
    def trinamic_inputs(self, axis):
        """Analogue input and reference switch bit of the axis' channel (raw)."""
        data = self._data(axis)
        r = self._pack.inputs(data.channel)
        refs = format(r.all_references & 0x3F, "06b")[::-1]
        return f"analog {r.analog}/1023, reference bit {int(r.reference)} (raw; manual: inverted), " \
               f"all reference bits m0..m5: {refs}"

    @object_method(types_info=("None", "str"))
    def trinamic_supply(self, axis):
        """Supply voltage of the unit (channel 7, 22 counts per volt)."""
        return f"{self._pack.inputs(7).analog / 22:.1f} V"

