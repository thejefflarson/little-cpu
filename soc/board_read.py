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
  * The port is the iCELink's, resolved by the USB VID/PID the loader is pinned to
    (soc/board_port.py). Zero or several matches are refused rather than the first
    one opened, and a reopen after re-enumeration goes through the same lookup.
  * Bytes reach the terminal escaped, every one that is not printable ASCII, `\\n` or
    `\\t`: the UART is the program's, and an ESC sequence in it would otherwise retitle
    the window, rewrite earlier output or set the clipboard. `--out` keeps the raw bytes.
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

import board_port

KEEP = frozenset(b"\n\t") | frozenset(range(0x20, 0x7F))


def escape_display(data):
    return "".join(chr(b) if b in KEEP else "\\x%02x" % b for b in data)


def stamped_lines(buf, tail, arrived):
    """(`seconds<TAB>line` for each line completed since offset tail, the new tail)."""
    lines = []
    while (end := buf.find(b"\n", tail)) >= 0:
        lines.append("%.4f\t%s" % (arrived, escape_display(buf[tail:end].rstrip(b"\r"))))
        tail = end + 1
    return lines, tail


def candidates(override):
    return sorted(glob.glob(override)) if override else board_port.find_ports()


def refusal(nodes, override):
    if len(nodes) == 1:
        return None
    want = override or "USB %04x:%04x" % (board_port.VID, board_port.PID)
    return "%d serial nodes match %s%s; name the one to read with --port" % (
        len(nodes), want, " (%s)" % ", ".join(nodes) if nodes else "")


def open_port(path):
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
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
        termios.tcflush(fd, termios.TCIFLUSH)
    except termios.error:
        os.close(fd)
        return None
    return fd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--port", help="a serial node or glob, instead of the iCELink found by VID/PID")
    parser.add_argument("--stamps", help="write `seconds<TAB>line` for each line, stamped as it arrived")
    parser.add_argument("--out", help="write the transcript here as it arrives")
    parser.add_argument("--until", help="stop early once this text has arrived")
    args = parser.parse_args()

    nodes = candidates(args.port)
    if refusal(nodes, args.port):
        sys.exit("*** " + refusal(nodes, args.port))
    node = nodes[0]
    print("board_read: reading %s" % node, flush=True)

    buf = bytearray()
    fd = open_port(node)
    eofs = 0
    lines, tail = [], 0
    started = time.monotonic()
    while time.monotonic() - started < args.seconds:
        if fd is None:
            nodes = candidates(args.port)
            if len(nodes) > 1:
                sys.exit("*** " + refusal(nodes, args.port))
            again = nodes[0] if nodes else None
            fd = open_port(again) if again else None
            if fd is None:
                time.sleep(0.1)
                continue
            if again != node:
                print("board_read: the board re-enumerated as %s" % again, flush=True)
                node = again
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
            arrived = time.monotonic() - started
            buf.extend(chunk)
            if args.out:
                with open(args.out, "wb") as handle:
                    handle.write(bytes(buf))
            if args.stamps:
                fresh, tail = stamped_lines(buf, tail, arrived)
                lines += fresh
                with open(args.stamps, "w") as handle:
                    handle.write("".join(line + "\n" for line in lines))
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

    sys.stdout.write(escape_display(buf))
    sys.stdout.write("\n")
    if not buf:
        print("*** nothing arrived on %s in %.0fs." % (node, args.seconds))
        print("*** The board is configured by `make icesugar-prog`, which loads")
        print("*** SRAM over JTAG and starts the design immediately. Writing the")
        print("*** flash instead leaves the part unconfigured until it is")
        print("*** power-cycled, and a silent line is what that looks like.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
