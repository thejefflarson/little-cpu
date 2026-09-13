#!/usr/bin/env python3
"""Turns nano-sim --bench's one output line into DMIPS/MHz or CoreMark/MHz.

Reads the `BENCH marks=<n> cycles=<n> verdict=<n> writes=<n>` line
soc/compare/dhry_monitor.v's marker mechanism produces (nano/tb/nano_testbench.v prints it
via nano/tb/nano_cxxrtl.cc), and requires two markers (a run that reached both ends of the
timed region) and verdict 1 (the benchmark's own self-check, not this script's).

nano has no placed clock, so this prints only the per-MHz figure -- never an absolute
DMIPS or CoreMark figure, and never beside littlecpu's numbers, which were measured on a
different design with a different memory system.
"""

import argparse
import re
import sys

FACT = re.compile(
    r"^BENCH marks=(?P<marks>\d+) cycles=(?P<cycles>\d+) "
    r"verdict=(?P<verdict>\d+) writes=(?P<writes>\d+)"
)
VAX_DHRYSTONES_PER_SEC = 1757.0


def parse(path):
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read the simulation log: {exc}")
    for line in text.splitlines():
        match = FACT.match(line.strip())
        if match:
            return {k: int(v) for k, v in match.groupdict().items()}
    sys.exit(
        f"no BENCH result line in {path}. The simulation printed nothing this script\n"
        "understands, which is a run that did not happen rather than a run with no result."
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="nano-sim --bench's output")
    parser.add_argument("--kind", choices=["dhrystone", "coremark"], required=True)
    parser.add_argument("--runs", type=int, required=True,
                         help="Dhrystone runs, or CoreMark iterations")
    args = parser.parse_args()

    if args.runs <= 0:
        sys.exit(f"--runs is {args.runs}; nothing was measured.")

    facts = parse(args.log)
    if facts["marks"] != 2:
        sys.exit(
            f"nano published {facts['marks']} marker(s), not 2. One is a run that "
            "reached\nthe start of the measured loop and never the end of it; none is a "
            f"run\nthat never started. It spent {facts['writes']} cycles writing, which "
            "is how\nfar it got."
        )
    if facts["cycles"] <= 0:
        sys.exit(f"nano measured {facts['cycles']} cycles, which is not a run.")
    if facts["verdict"] != 1:
        sys.exit(
            f"the benchmark's own self-check word is {facts['verdict']}, not 1: it says "
            "it\ndid not compute the published results, so this cycle count describes a "
            "run\nthat was not correct and means nothing."
        )

    cycles = facts["cycles"]
    per_run = cycles / args.runs
    if args.kind == "dhrystone":
        per_mhz = (args.runs * 1e6 / cycles) / VAX_DHRYSTONES_PER_SEC
        print(f"cycles       : {cycles} ({per_run:.1f} cycles/dhrystone, {args.runs} runs)")
        print(f"DMIPS/MHz    : {per_mhz:.3f}")
    else:
        per_mhz = args.runs * 1e6 / cycles
        print(f"cycles       : {cycles} ({per_run:.1f} cycles/iteration, {args.runs} iterations)")
        print(f"CoreMark/MHz : {per_mhz:.3f}")
    print(
        "No clock is placed for nano yet, so this is a per-MHz figure only -- never an\n"
        "absolute figure, and never comparable to littlecpu's own numbers, which were\n"
        "measured on a different design with a different memory system."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
