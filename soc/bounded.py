#!/usr/bin/env python3
"""Run a command under a wall-clock limit: bounded.py SECONDS COMMAND [ARG...].

macOS has no timeout(1), and a stalled CMSIS-DAP transfer never returns, so a loader run unbounded hangs
the whole script around it. On expiry the command's process group gets SIGTERM, then SIGKILL if it outlives GRACE_S, and the exit
status is 124, timeout(1)'s own, so a caller can tell a stall from a refusal. Otherwise the command's status
is returned. SIGTERM goes first because sudo relays it to a root child and cannot relay SIGKILL, which would
orphan an iceprog in mid-write.
"""

import os
import signal
import subprocess
import sys

TIMED_OUT = 124
GRACE_S = 10


def run(seconds, argv, grace=GRACE_S):
    proc = subprocess.Popen(argv, start_new_session=True)
    try:
        return proc.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        sys.stderr.write("*** %s still running after %gs; stopped\n" % (argv[0], seconds))
        return TIMED_OUT


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: bounded.py SECONDS COMMAND [ARG...]")
    try:
        seconds = float(sys.argv[1])
    except ValueError:
        sys.exit("*** bounded.py: %r is not a number of seconds" % sys.argv[1])
    try:
        return run(seconds, sys.argv[2:])
    except OSError as err:
        sys.stderr.write("*** %s: %s\n" % (sys.argv[2], err.strerror))
        return 127


if __name__ == "__main__":
    sys.exit(main())
