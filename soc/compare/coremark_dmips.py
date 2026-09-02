#!/usr/bin/env python3
"""Grade soc/compare/coremark_tb.v's run and turn its cycle counts into
CoreMark/MHz.

The testbench prints raw facts and nothing else; every comparison that can
call a run bad is here, in a file test/probe_gates.sh can drive against
fixture text without a simulator, the same split soc/compare/dhry_dmips.py
uses for Dhrystone.

What a run has to satisfy:

  - both cores present, each with two markers. One marker is a run that
    reached the start of the measured section and not the end of it.
  - a positive cycle count, and the self-check word 1: CoreMark's own verdict
    that its list/matrix/state CRCs matched EEMBC's published values for the
    2K performance run, 3 is its verdict that they did not, 5 is a run that
    validated a DIFFERENT configuration than the one this port checks for,
    and 0 is a program that never got there.
  - the two data RAMs identical at the end of the run. Same image, same
    memories, no interrupt on either side -- the two RAMs hold the same bytes
    or one of the cores computed something else.

CoreMark/MHz is iterations * 1e6 / cycles -- frequency cancels out of the
ratio, the same way it does for DMIPS/MHz, which is why this is safe to
compute with no clock at all. Absolute CoreMark needs one; this script will
not invent it: pass `--mhz core=value` from a placement, or get only the
per-MHz column.
"""

import argparse
import re
import sys

FACT = re.compile(
    r"^COREMARK core=(?P<core>\S+) marks=(?P<marks>\d+) cycles=(?P<cycles>\d+) "
    r"verdict=(?P<verdict>\d+) writes=(?P<writes>\d+)"
)
RAMDIFF = re.compile(r"^COREMARK ramdiff=(\d+) of=(\d+) words")
WAIT = re.compile(r"^COREMARK core=(?P<core>\S+) wait_cycles=(?P<n>\d+)")


def parse(path):
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read the simulation log: {exc}")
    cores = {}
    ramdiff = None
    waits = {}
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
            continue
        match = WAIT.match(line)
        if match:
            waits[match.group("core")] = int(match.group("n"))
    if not cores:
        sys.exit(
            f"no COREMARK result lines in {path}. The simulation printed nothing "
            "this\nscript understands, which is a run that did not happen rather "
            "than a\nrun with no result -- there is nothing to report."
        )
    if ramdiff is None:
        sys.exit(
            f"no ramdiff line in {path}. That comparison is the only thing saying "
            "the\ntwo cores computed the same thing, and a run that did not report "
            "it is a\nrun whose cross-core check did not happen."
        )
    return cores, ramdiff, waits


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
                "that reached the start of the measured section and never the end\n"
                f"of it; none is a run that never started. It spent {facts['writes']}\n"
                "cycles writing, which is how far it got."
            )
        if facts["cycles"] <= 0:
            sys.exit(f"{core} measured {facts['cycles']} cycles, which is not a run.")
        if facts["verdict"] != 1:
            sys.exit(
                f"{core}'s self-check word is {facts['verdict']}, not 1. CoreMark\n"
                "says it did not validate the 2K performance run, so its cycle "
                "count\ndescribes a run that was not correct and means nothing."
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
    parser.add_argument("log", help="soc/compare/coremark_tb.v's output")
    parser.add_argument("--iterations", type=int, required=True)
    parser.add_argument(
        "--cores",
        default="littlecpu,hazard3",
        help="the cores a complete run reports, comma separated",
    )
    parser.add_argument(
        "--mhz",
        action="append",
        default=[],
        metavar="CORE=MHZ",
        help="that core's placed clock, for the absolute CoreMark column",
    )
    args = parser.parse_args()

    if args.iterations <= 0:
        sys.exit(f"--iterations is {args.iterations}; nothing was measured.")
    want = [core for core in args.cores.split(",") if core]
    if len(want) != 2:
        sys.exit(f"--cores wants exactly two names, got '{args.cores}'")

    cores, ramdiff, waits = parse(args.log)
    grade(cores, ramdiff, want)
    clocks = read_clocks(args.mhz)
    for core in clocks:
        if core not in want:
            sys.exit(f"--mhz names {core}, which is not one of the cores graded.")

    print(f"{'':<12}{'cycles':>12}{'cyc/iter':>10}{'CoreMark/MHz':>14}{'CoreMark':>12}")
    for core in want:
        cycles = cores[core]["cycles"]
        per_iter = cycles / args.iterations
        coremark_per_mhz = args.iterations * 1e6 / cycles
        if core in clocks:
            absolute = f"{coremark_per_mhz * clocks[core]:.2f}"
        else:
            absolute = "-"
        print(
            f"{core:<12}{cycles:>12}{per_iter:>10.1f}{coremark_per_mhz:>14.3f}"
            f"{absolute:>12}"
        )

    ratio = cores[want[1]]["cycles"] / cores[want[0]]["cycles"]
    print(
        f"\n{want[1]} takes {ratio:.3f}x {want[0]}'s cycles for the same work, and "
        f"the\ntwo data RAMs are identical in all {ramdiff[1]} words afterwards."
    )

    for core in want:
        if core in waits:
            share = 100.0 * waits[core] / cores[core]["cycles"]
            print(
                f"\n{core} spends {waits[core]} of its {cores[core]['cycles']} "
                f"measured cycles ({share:.2f}%) in a bus wait state the other "
                "core here does not pay -- disclosed, not corrected."
            )

    if not clocks:
        print(
            "No clock was given, so the CoreMark column is empty. It is each\n"
            "core's own worst placement from soc/compare/sweep.sh, and nothing "
            "here\nwill guess one."
        )
    else:
        print(
            "THE COREMARK COLUMN MULTIPLIES A CLOCK MEASURED AT THE PLACED 4 KB/2 "
            "KB\nGEOMETRY BY CYCLES MEASURED AT A LARGER SIMULATED ONE, because no "
            "ice40\nin this flow has the block RAM to hold CoreMark. It is a "
            "projection.\nThe image is RV32IMA, the ISA littlecpu and Hazard3's "
            "iCE40 build share --\nneither core's own C extension or lack of one "
            "is exercised here."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
