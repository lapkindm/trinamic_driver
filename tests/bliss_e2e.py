"""End-to-end test of the Bliss controller against the pty simulator.

Needs a Bliss environment, a fresh Beacon whose config is
bliss_config/xrd3003tt_trinamic.yml with ``serial.url`` pointing at the
simulator link, and::

    python tests/pty_sim.py <link> [--reply-length 9]
    BEACON_HOST=localhost:25000 PYTHONPATH=. python tests/bliss_e2e.py <link>
"""

import json
import os
import sys
import time

from bliss.common.axis.error import AxisOnLimitError
from bliss.config.static import get_config

LINK = sys.argv[1]
results = []


def sim(cmd, motor=0):
    fd = os.open(LINK, os.O_WRONLY | os.O_NOCTTY)
    os.write(fd, bytes([0xFE, cmd, motor]) + bytes(6))
    os.close(fd)


def sim_state():
    path = LINK + ".json"
    if os.path.exists(path):
        os.remove(path)
    sim(0x03)
    for _ in range(100):
        if os.path.exists(path):
            with open(path) as f:
                return json.load(f)
        time.sleep(0.02)
    raise RuntimeError("simulator did not dump its state")


def check(name, cond, detail=""):
    results.append(bool(cond))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


def raises(fn, exc=Exception):
    try:
        fn()
    except exc as e:
        return e
    return None


cfg = get_config()
m1, a1, a2 = cfg.get("m1"), cfg.get("a1"), cfg.get("a2")
ctrl = m1.controller
for ax in (m1, a1, a2):
    ax.position  # initialise (programs the unit)
for ax in (m1, a1, a2):
    ax.dial = 0
    ax.position = 0

# -- programming ---------------------------------------------------------------
st = sim_state()
m = st["motors"]
check("clock divider 23, as spec", st["clkdiv"] == 23, st["clkdiv"])
check("m1 speed values", (m[2]["div"], m[2]["vstart"], m[2]["vmax"]) == (2, 16, 425),
      (m[2]["div"], m[2]["vstart"], m[2]["vmax"]))
check("a1 speed values as spec's (div 1, vstart 8, vmax 503)", (m[0]["div"], m[0]["vstart"], m[0]["vmax"]) == (1, 8, 503),
      (m[0]["div"], m[0]["vstart"], m[0]["vmax"]))
check("amax limited to 64*vstart", m[0]["amax"] == 512, m[0]["amax"])
check("stop limits 0/1024 on all three, as spec", all(m[c].get("stop_limits") == [0, 1024] for c in (0, 1, 2)),
      [m[c].get("stop_limits") for c in (0, 1, 2)])
check("only what spec sends: no $10/$11/$15/$16/$18", not set(st["commands"]) & {0x10, 0x11, 0x15, 0x16, 0x18},
      [hex(c) for c in st["commands"]])
check("motor flags untouched (unit default)", m[2]["home_method"] == 0 and "peak" not in m[0],
      (hex(m[2]["home_method"]), m[0].get("peak")))
check("no unit-changing command sent", not set(st["commands"]) & {0x40, 0x41, 0x42, 0x45, 0xCC},
      [hex(c) for c in st["commands"]])
check("velocity read back as configured", abs(m1.velocity - 1.0) < 1e-9 and abs(a1.velocity - 4.0) < 1e-9,
      (m1.velocity, a1.velocity))
check("acceleration a1 reports the limit", abs(a1.acceleration - 31.8) < 0.1, a1.acceleration)

# -- motion --------------------------------------------------------------------
m1.move(0.5)
check("single move", abs(m1.position - 0.5) < 1e-6 and sim_state()["motors"][2]["position"] == 2700,
      m1.position)
from bliss.common.standard import move  # noqa: E402
move(a1, 2, a2, 3)
check("group move", (a1.position, a2.position) == (2.0, 3.0), (a1.position, a2.position))
m1.move(-0.2, relative=True)
check("relative move (backlash)", abs(m1.position - 0.3) < 1e-6, m1.position)

m1.move(10, wait=False)
time.sleep(0.2)
check("MOVING during move", m1.state.MOVING, m1.state)
m1.stop()
check("stop ends move", m1.state.READY and m1.position < 10, f"{m1.state} at {m1.position:.3f}")

m1.jog(0.5)
time.sleep(0.2)
check("jog moving", m1.state.MOVING)
m1.stop()
check("jog stopped", m1.state.READY, m1.state)
m1.move(0.3)
check("move after jog (rotation ended)", abs(m1.position - 0.3) < 1e-6, m1.position)

# -- stop condition (limit) ----------------------------------------------------
m1.move(5, wait=False)
time.sleep(0.1)
sim(0x02, 2)
err = raises(m1.wait_move, AxisOnLimitError)
check("stop condition raises AxisOnLimitError on m1", err is not None, repr(err))
m1.move(0.3)
check("m1 moves again after limit", abs(m1.position - 0.3) < 1e-6)
a1.move(4, wait=False)
time.sleep(0.05)
sim(0x02, 0)
err = raises(a1.wait_move)
check("ignore_limits: no error on a1", err is None, repr(err))

# -- power cycle ---------------------------------------------------------------
sim(0x01)
time.sleep(2.1)  # check_interval
a2.sync_hard()
check("POSLOST after power cycle", "POSLOST" in a2.state.current_states_names, a2.state)
st2 = sim_state()
check("re-programmed after power cycle (clkdiv, speeds)", st2["clkdiv"] == 23 and st2["motors"][0]["vmax"] == 503,
      (st2["clkdiv"], st2["motors"][0]["vmax"]))
err = raises(lambda: a2.move(1))
check("move refused while position lost", err is not None and "position is lost" in str(err), str(err)[:90])
err = raises(lambda: (a2.jog(1), a2.wait_move()))
check("jog refused while position lost", err is not None and "POSLOST" in str(err), str(err)[:80])
check("axis consistent after refused jog", not a2.is_moving and sim_state()["motors"][1]["action"] == 0,
      f"is_moving={a2.is_moving}")
a2.home()
check("home() recovers a2", a2.state.READY and "POSLOST" not in a2.state.current_states_names and a2.position == 0.0,
      f"{a2.state} {a2.position}")
m1.dial = 0
m1.position = 0
check("setting the dial recovers m1", m1.state.READY and "POSLOST" not in m1.state.current_states_names, m1.state)
a1.sync_hard()
check("a1 still lost", "POSLOST" in a1.state.current_states_names, a1.state)
a1.home()

# -- lost replies ----------------------------------------------------------------
a2.move(3)
sim(0x04, 1)  # the next reply is lost on the line
a2.sync_hard()
check("a lost reply is asked again", a2.position == 3 and a2.state.READY, (a2.position, a2.state))
sim(0x01)  # power cycle ...
sim(0x05)  # ... and the $43 reply carrying the reset flag is lost
time.sleep(2.1)  # check_interval
a2.sync_hard()
check("reset caught although its $43 reply was lost (positions back at 0)",
      "POSLOST" in a2.state.current_states_names, a2.state)
a2.home()
m1.dial = 0
m1.position = 0
a1.home()

# -- raw and info --------------------------------------------------------------
check("raw query", ctrl.raw_write_read("20 02 00").startswith("02 "), ctrl.raw_write_read("20 02 00"))
check("hardware reset refused", raises(lambda: ctrl.raw_write("CC 00")) is not None)
check("get_id", "1.66" in ctrl.get_id(m1), ctrl.get_id(m1))
check("axis info", "not sent" in ctrl.get_axis_info(m1))
check("inputs", "analog" in m1.trinamic_inputs())

print(f"\n{sum(results)}/{len(results)} checks passed")
sys.exit(0 if all(results) else 1)
