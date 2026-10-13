"""Serve the simulated SIXpack 2 on a pseudo-terminal.

Usage:  python tests/pty_sim.py LINK [--reply-length 8|9]

Creates a pty, symlinks its device to LINK and answers SIXpack frames on it
until killed, so real serial clients (pyserial, Bliss) can talk to it.

Test hooks are frames for address 0xFE (see fake_sixpack.py), plus command
0x03: write the simulator state as JSON to LINK + ".json".
"""

import argparse
import json
import os
import sys
import termios
import tty

sys.path.insert(0, os.path.dirname(__file__))
from fake_sixpack import SIM_ADDRESS, FakeSixPack  # noqa: E402

SIM_DUMP = 0x03


def dump(sim: FakeSixPack, path: str) -> None:
    sim.advance()
    state = {
        "clkdiv": sim.clkdiv,
        "motors": [
            {
                "position": round(m.position), "action": int(m.action), "vmax": m.vmax, "vstart": m.vstart,
                "vmin": m.vmin, "div": m.div, "amax": m.amax, "poslimit": m.poslimit,
                "home_method": m.home_method, "vrefmax": m.vrefmax, "stop_after_search": m.stop_after_search,
                "null_offset": m.null_offset, **{k: v for k, v in m.params.items()},
            }
            for m in sim.motors
        ],
        "commands": sorted({f[1] for f in sim.frames}),
    }
    with open(path + ".tmp", "w") as f:
        json.dump(state, f)
    os.replace(path + ".tmp", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("link")
    ap.add_argument("--reply-length", type=int, default=8, choices=(8, 9))
    args = ap.parse_args()

    master, slave = os.openpty()
    tty.setraw(slave)
    attrs = termios.tcgetattr(slave)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(slave, termios.TCSANOW, attrs)
    if os.path.lexists(args.link):
        os.remove(args.link)
    os.symlink(os.ttyname(slave), args.link)
    print(f"serving SIXpack 2 on {args.link} -> {os.ttyname(slave)} ({args.reply_length}-byte replies)",
          flush=True)

    sim = FakeSixPack(reply_length=args.reply_length)
    buf = b""
    while True:
        buf += os.read(master, 1024)
        while len(buf) >= 9:
            frame, buf = buf[:9], buf[9:]
            if frame[0] == SIM_ADDRESS and frame[1] == SIM_DUMP:
                dump(sim, args.link + ".json")
                continue
            sim.feed(frame)
            if sim.out:
                os.write(master, sim.out)
                sim.out = b""


if __name__ == "__main__":
    main()
