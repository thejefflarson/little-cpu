#!/usr/bin/env python3
"""Grade soc/compare/dhry_tb.v's run and turn its cycle counts into DMIPS.

The testbench prints raw facts and nothing else; every comparison that can call
a run bad is here, in a file test/probe_gates.sh can drive against fixture text
without a simulator. Five of this repo's recorded defects were comparisons whose
failure path had never once run, and a benchmark harness is exactly where that
hides: a core that faults on its first instruction reports a cycle count too.

What a run has to satisfy:

  - both cores present, each with two markers. One marker is a run that reached
    the start of the measured loop and not the end of it.
  - a positive cycle count, and the self-check word 1: that is the benchmark's
    own verdict that it computed the published results, 3 is its verdict that it
    did not, and 0 is a program that never got there.
  - the two data RAMs identical at the end of the run. Same image, same
    memories, no interrupt on either side -- the two RAMs hold the same 16 KB or
    one of the cores computed something else.

DMIPS/MHz is Dhrystones per second per MHz divided by 1757, the VAX 11/780 rate,
which is what test/bench/dhry_port.c computes on-target for the other
measurement. Absolute DMIPS needs a clock, and this script will not invent one:
pass `--mhz core=value` from a placement, or get only the per-MHz column.
"""

import argparse
import re
import sys

FACT = re.compile(
    r"^DHRY core=(?P<core>\S+) marks=(?P<marks>\d+) cycles=(?P<cycles>\d+) "
    r"verdict=(?P<verdict>\d+) writes=(?P<writes>\d+)"
)
RAMDIFF = re.compile(r"^DHRY ramdiff=(\d+) of=(\d+) words")
VAX_DHRYSTONES_PER_SEC = 1757.0


def parse(path):
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read the simulation log: {exc}")
    cores = {}
    ramdiff = None
    for line in text.splitlines():
        line = line.strip()
        match = FACT.match(line)
        if match:
            fields = match.groupdict()
            core = fields.pop("core")
            if core in cores:
                sys.exit(f"{core} reported twice; there is no single run to grade.")
            cores[core] = {name: int(value) for name, value in fields.items()}
            continue
        match = RAMDIFF.match(line)
        if match:
            ramdiff = (int(match.group(1)), int(match.group(2)))
    if not cores:
        sys.exit(
            f"no DHRY result lines in {path}. The simulation printed nothing this\n"
            "script understands, which is a run that did not happen rather than a\n"
            "run with no result -- there is nothing to report."
        )
    if ramdiff is None:
        sys.exit(
            f"no ramdiff line in {path}. That comparison is the only thing saying "
            "the\ntwo cores computed the same thing, and a run that did not report "
            "it is a\nrun whose cross-core check did not happen."
        )
    return cores, ramdiff


def grade(cores, ramdiff, want):
    missing = [core for core in want if core not in cores]
    if missing:
        sys.exit(
            f"no result for {', '.join(missing)}. A cross-core figure needs both\n"
            "sides of it, and one side alone is the incomparability this harness\n"
            "exists to remove."
        )
    for core in want:
        facts = cores[core]
        if facts["marks"] != 2:
            sys.exit(
                f"{core} published {facts['marks']} marker(s), not 2. One is a run\n"
                "that reached the start of the measured loop and never the end of\n"
                f"it; none is a run that never started. It spent {facts['writes']}\n"
                "cycles writing, which is how far it got."
            )
        if facts["cycles"] <= 0:
            sys.exit(f"{core} measured {facts['cycles']} cycles, which is not a run.")
        if facts["verdict"] != 1:
            sys.exit(
                f"{core}'s self-check word is {facts['verdict']}, not 1. Dhrystone\n"
                "says it did not compute the published results, so its cycle count\n"
                "describes a run that was not correct and means nothing."
            )
    differing, total = ramdiff
    if total <= 0:
        sys.exit(
            f"the RAM comparison covered {total} words, so it could not have failed."
        )
    if differing != 0:
        sys.exit(
            f"the two cores' data RAMs differ in {differing} of {total} words. They\n"
            "ran the same image on the same memories, so one of them computed\n"
            "something else and neither cycle count is a measurement."
        )


def read_clocks(specs):
    clocks = {}
    for spec in specs:
        if "=" not in spec:
            sys.exit(f"--mhz wants core=value, got '{spec}'")
        core, value = spec.split("=", 1)
        try:
            clocks[core] = float(value)
        except ValueError:
            sys.exit(f"--mhz {core} is '{value}', which is not a number of MHz.")
        if clocks[core] <= 0:
            sys.exit(f"--mhz {core} is {value}; a design at zero MHz is not placed.")
    return clocks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="soc/compare/dhry_tb.v's output")
    parser.add_argument("--runs", type=int, required=True)
    parser.add_argument(
        "--cores",
        default="littlecpu,vexriscv",
        help="the cores a complete run reports, comma separated",
    )
    parser.add_argument(
        "--mhz",
        action="append",
        default=[],
        metavar="CORE=MHZ",
        help="that core's placed clock, for the absolute DMIPS column",
    )
    args = parser.parse_args()

    if args.runs <= 0:
        sys.exit(f"--runs is {args.runs}; nothing was measured.")
    want = [core for core in args.cores.split(",") if core]
    if len(want) != 2:
        sys.exit(f"--cores wants exactly two names, got '{args.cores}'")

    cores, ramdiff = parse(args.log)
    grade(cores, ramdiff, want)
    clocks = read_clocks(args.mhz)
    for core in clocks:
        if core not in want:
            sys.exit(f"--mhz names {core}, which is not one of the cores graded.")

    print(f"{'':<12}{'cycles':>12}{'cyc/dhry':>10}{'DMIPS/MHz':>12}{'DMIPS':>10}")
    for core in want:
        cycles = cores[core]["cycles"]
        per_run = cycles / args.runs
        dhrystones_per_sec_per_mhz = args.runs * 1e6 / cycles
        dmips_per_mhz = dhrystones_per_sec_per_mhz / VAX_DHRYSTONES_PER_SEC
        if core in clocks:
            absolute = f"{dmips_per_mhz * clocks[core]:.2f}"
        else:
            absolute = "-"
        print(
            f"{core:<12}{cycles:>12}{per_run:>10.1f}{dmips_per_mhz:>12.3f}"
            f"{absolute:>10}"
        )

    ratio = cores[want[1]]["cycles"] / cores[want[0]]["cycles"]
    print(
        f"\n{want[1]} takes {ratio:.3f}x {want[0]}'s cycles for the same work, and "
        f"the\ntwo data RAMs are identical in all {ramdiff[1]} words afterwards."
    )
    if not clocks:
        print(
            "No clock was given, so the DMIPS column is empty. It is each core's own\n"
            "worst placement from soc/compare/sweep.sh, and nothing here will guess "
            "one."
        )
    else:
        print(
            "THE DMIPS COLUMN MULTIPLIES A CLOCK MEASURED AT THE PLACED 4 KB/2 KB\n"
            "GEOMETRY BY CYCLES MEASURED AT A LARGER SIMULATED ONE, because no ice40\n"
            "in this flow has the block RAM to hold Dhrystone. It is a projection.\n"
            "Neither side is a shipped design either: this core is RV32IMC + Zicsr\n"
            "with traps, that one is RV32IC with a branch predictor, no CSR file and\n"
            "no traps, and the image is the ISA they share."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
