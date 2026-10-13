"""Read-only check of the SIXpack 2 before the first Bliss session.

Usage (with spec stopped, since only one program can hold the port):

    python examples/check_readonly.py /dev/ttyS9

Sends only queries: $43 unit information, $28 activities, $20 position of
each configured motor, $30 inputs. Nothing moves and no setting is changed,
with two side effects of reading: $43 clears the unit's reset flag and $20
clears a motor's stop flag (as spec's own polling does).

It reports which reply format the unit uses (8 or 9 bytes; the manual is
ambiguous) and the motor positions in user units, to compare with the DIAL
positions of spec's ``wm m1 a1 a2``.
"""

import sys
import time

import serial

sys.path.insert(0, __file__.rsplit("/examples/", 1)[0])
from trinamic_driver import SixPack, SixPackError  # noqa: E402
from trinamic_driver.protocol import action_name  # noqa: E402

# channel -> (axis, steps_per_unit), as in bliss_config/xrd3003tt_trinamic.yml
AXES = {2: ("m1", 5400), 0: ("a1", 3200), 1: ("a2", 3200)}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyS9"
    with serial.Serial(port, 19200, timeout=0.05) as ser:
        pack = SixPack(ser, address=0, timeout=0.5)
        try:
            info = pack.unit_info()
        except SixPackError as exc:
            sys.exit(f"no answer from SIXpack unit 0 on {port}: {exc}")
        print(f"SIXpack 2 on {port}: firmware {info.firmware_str}, sn {info.serial}, {info.temperature} C")
        print(f"reply format: {pack.reply_length} bytes")
        if info.reset:
            print("RESET FLAG SET: the unit was reset (power cycle?) since its last $43 read; "
                  "its parameters are defaults and positions are 0")
        print(f"supply: {pack.inputs(7).analog / 22:.1f} V")
        print("activities:", ", ".join(f"m{i}={action_name(a)}" for i, a in enumerate(pack.activities())))
        print()
        print(f"{'axis':5} {'motor':5} {'steps':>10} {'dial':>11} {'action':>12} {'stop':>4} {'ref':>3} {'analog':>6}")
        for channel, (name, spu) in sorted(AXES.items(), key=lambda kv: kv[1][0]):
            p = pack.position(channel)
            inp = pack.inputs(channel)
            print(f"{name:5} {channel:5} {p.position:10d} {p.position / spu:11.5f} {action_name(p.action):>12} "
                  f"{int(p.stop):4} {int(inp.reference):3} {inp.analog:6}")
        time.sleep(0.1)
        print()
        print("Compare 'dial' with spec's DIAL positions (wm m1 a1 a2). 'ref' is the raw reference")
        print("input bit (the manual says it is inverted).")


if __name__ == "__main__":
    main()
