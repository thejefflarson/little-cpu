#!/usr/bin/env python3
"""Turn soc/clock_check.S's `T` lines into a measured core frequency and grade it.

  clock_grade.py STAMPS --clock-hz HZ [--tol-ppm N] [--min-intervals N]

STAMPS is soc/board_read.py's `--stamps` file. The program prints a `T` every PERIOD `mcycle` ticks, so the
mean spacing of the `T`s on the host's wall clock is PERIOD / f. The mean runs from the first `T` to the
last, over every interval between them, which is what keeps one late UART chunk from standing for the whole
run. Fewer than --min-intervals is a refusal rather than a figure, and so is a stamps file with no `T`.
"""

import argparse
import sys

PERIOD = 150_000_000


def measure(lines):
    """(count of T lines, measured Hz) from `seconds<TAB>text` lines; Hz is None under two T lines."""
    times = []
    for line in lines:
        stamp, _, text = line.rstrip("\n").partition("\t")
        if text == "T":
            times.append(float(stamp))
    if len(times) < 2:
        return len(times), None
    return len(times), PERIOD * (len(times) - 1) / (times[-1] - times[0])


def grade(lines, clock_hz, tol_ppm, min_intervals):
    """(ok, message) for a stamps file against the clock the design claims."""
    count, hz = measure(lines)
    if count - 1 < min_intervals:
        return False, "%d T lines give %d intervals; %d are needed to grade a clock" % (
            count, max(count - 1, 0), min_intervals)
    error = (hz - clock_hz) / clock_hz * 1e6
    ok = abs(error) <= tol_ppm
    return ok, "clock: %.4f MHz measured over %d intervals; expected %.4f MHz; %+.0f ppm; tolerance %.0f ppm: %s" % (
        hz / 1e6, count - 1, clock_hz / 1e6, error, tol_ppm, "ok" if ok else "OUT OF TOLERANCE")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stamps")
    parser.add_argument("--clock-hz", type=float, required=True)
    parser.add_argument("--tol-ppm", type=float, default=1000.0)
    parser.add_argument("--min-intervals", type=int, default=4)
    args = parser.parse_args()
    with open(args.stamps) as handle:
        ok, message = grade(handle, args.clock_hz, args.tol_ppm, args.min_intervals)
    print(message if ok else "*** " + message)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
