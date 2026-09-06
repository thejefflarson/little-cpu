#!/usr/bin/env python3
"""Read the iCESugar-Pro's UART for a bounded window and print what arrived.

The counterpart to soc/ftread.c, and much smaller for one reason: the UPduino's
FT232H is both the serial port and the programmer, so iceprog leaves it in MPSSE
mode with no /dev node and ftread has to talk libftdi. This board's serial is a
CDC on the iCELink debugger, a device node of its own that nothing takes away,
so the only things left worth writing down are the ones that cost a run here.

  * The window is BOUNDED and the transcript is written as bytes arrive. A run
    that is interrupted, or a program that never prints its last line, still
    leaves what it did say on disk.
  * `os.read` returning b"" is END OF FILE, not "nothing yet" -- the debugger
    re-enumerates and the descriptor goes stale. Read as idle it spins on a dead
    fd for the whole window and reports silence from a board that was talking.
    Counted and reopened instead.
  * The iCELink writes its own chatter onto this line (`@cdone:0`, an overflow
    notice). It is the debugger's, not the core's, and is left in rather than
    filtered: a transcript that quietly drops bytes is the wrong tool for
    finding out why a board said nothing.
"""

import argparse
import glob
import os
import sys
import termios
import time


def open_port(pattern):
    ports = sorted(glob.glob(pattern))
    if not ports:
        return None
    try:
        fd = os.open(ports[0], os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200
        attrs[2] = (attrs[2] | termios.CS8 | termios.CREAD | termios.CLOCAL) & ~(
            termios.PARENB | termios.CSTOPB
        )
        attrs[0] = attrs[1] = attrs[3] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except termios.error:
        os.close(fd)
        return None
    return fd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--port", default="/dev/cu.usbmodem*")
    parser.add_argument("--out", help="write the transcript here as it arrives")
    parser.add_argument("--until", help="stop early once this text has arrived")
    args = parser.parse_args()

    buf = bytearray()
    fd = open_port(args.port)
    eofs = 0
    started = time.time()
    while time.time() - started < args.seconds:
        if fd is None:
            fd = open_port(args.port)
            if fd is None:
                time.sleep(0.1)
                continue
            eofs = 0
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            eofs = 0
            time.sleep(0.01)
            continue
        except OSError:
            os.close(fd)
            fd = None
            time.sleep(0.15)
            continue
        if chunk:
            eofs = 0
            buf.extend(chunk)
            if args.out:
                with open(args.out, "wb") as handle:
                    handle.write(bytes(buf))
            if args.until and args.until.encode() in buf:
                break
        else:
            eofs += 1
            if eofs > 200:
                os.close(fd)
                fd = None
            time.sleep(0.01)
    if fd is not None:
        os.close(fd)

    sys.stdout.write(bytes(buf).decode("ascii", "replace"))
    sys.stdout.write("\n")
    if not buf:
        print("*** nothing arrived on %s in %.0fs." % (args.port, args.seconds))
        print("*** The board is configured by `make icesugar-prog`, which loads")
        print("*** SRAM over JTAG and starts the design immediately. Writing the")
        print("*** flash instead leaves the part unconfigured until it is")
        print("*** power-cycled, and a silent line is what that looks like.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
