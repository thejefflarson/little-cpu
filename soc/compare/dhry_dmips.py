#!/usr/bin/env python3
"""Grade a soc/compare/ Dhrystone testbench's run and turn its cycle counts
into DMIPS.

Reads soc/compare/dhry_tb.v's three-core log or soc/compare/dhry_solo_tb.v's
one-core log -- both print the same per-core fact line, so this script does not
care which produced the file. The testbench prints raw facts and nothing else;
every comparison that can call a run bad is here, in a file test/probe_gates.sh
can drive against fixture text without a simulator. Five of this repo's recorded
defects were comparisons whose failure path had never once run, and a benchmark
harness is exactly where that hides: a core that faults on its first instruction
reports a cycle count too.

What a run has to satisfy:

  - every requested core present, each with two markers. One marker is a run
    that reached the start of the measured loop and not the end of it.
  - a positive cycle count, and the self-check word 1: that is the benchmark's
    own verdict that it computed the published results, 3 is its verdict that it
    did not, and 0 is a program that never got there.
  - with more than one core requested, every core but the first (the reference)
    reports its data RAM identical to the reference's. Same image, same
    memories, no interrupt on any side -- agreement is transitive, so comparing
    every other core against one reference is the same claim a full round-robin
    would be, for fewer full-RAM scans. A single-core run has no reference to
    compare against and is graded without this check.

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
RAMDIFF = re.compile(r"^DHRY ramdiff core=(?P<core>\S+) diff=(?P<diff>\d+) of=(?P<total>\d+) words")
WAIT = re.compile(r"^DHRY core=(?P<core>\S+) wait_cycles=(?P<n>\d+)")
VAX_DHRYSTONES_PER_SEC = 1757.0


def parse(path):
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read the simulation log: {exc}")
    cores = {}
    ramdiff = {}
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
            ramdiff[match.group("core")] = (
                int(match.group("diff")),
                int(match.group("total")),
            )
            continue
        match = WAIT.match(line)
        if match:
            waits[match.group("core")] = int(match.group("n"))
    if not cores:
        sys.exit(
            f"no DHRY result lines in {path}. The simulation printed nothing this\n"
            "script understands, which is a run that did not happen rather than a\n"
            "run with no result -- there is nothing to report."
        )
    return cores, ramdiff, waits


def grade(cores, ramdiff, want):
    missing = [core for core in want if core not in cores]
    if missing:
        sys.exit(
            f"no result for {', '.join(missing)}. A cross-core figure needs every\n"
            "side of it, and one side missing is the incomparability this harness\n"
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
    if len(want) < 2:
        return
    reference = want[0]
    for core in want[1:]:
        if core not in ramdiff:
            sys.exit(
                f"no ramdiff line for {core} in the log. That comparison is the "
                "only\nthing saying it and "
                f"{reference} computed the same thing, and a run that\ndid not "
                "report it is a run whose cross-core check did not happen."
            )
        differing, total = ramdiff[core]
        if total <= 0:
            sys.exit(
                f"the RAM comparison for {core} covered {total} words, so it could "
                "not have failed."
            )
        if differing != 0:
            sys.exit(
                f"{reference} and {core}'s data RAMs differ in {differing} of "
                f"{total} words.\nThey ran the same image on the same memories, so "
                "one of them computed\nsomething else and neither cycle count is a "
                "measurement."
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
    parser.add_argument("log", help="the testbench's output")
    parser.add_argument("--runs", type=int, required=True)
    parser.add_argument(
        "--cores",
        default="littlecpu,vexriscv,hazard3",
        help="the cores a complete run reports, comma separated; the first is "
        "the reference the rest are RAM-compared against",
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
    if not want:
        sys.exit(f"--cores named no core at all, got '{args.cores}'")

    cores, ramdiff, waits = parse(args.log)
    grade(cores, ramdiff, want)
    clocks = read_clocks(args.mhz)
    for core in clocks:
        if core not in want:
            sys.exit(f"--mhz names {core}, which is not one of the cores graded.")

    print(f"{'':<12}{'cycles':>12}{'cyc/dhry':>10}{'DMIPS/MHz':>12}{'DMIPS':>10}")
    dmips_per_mhz = {}
    for core in want:
        cycles = cores[core]["cycles"]
        per_run = cycles / args.runs
        dhrystones_per_sec_per_mhz = args.runs * 1e6 / cycles
        dmips_per_mhz[core] = dhrystones_per_sec_per_mhz / VAX_DHRYSTONES_PER_SEC
        if core in clocks:
            absolute = f"{dmips_per_mhz[core] * clocks[core]:.2f}"
        else:
            absolute = "-"
        print(
            f"{core:<12}{cycles:>12}{per_run:>10.1f}{dmips_per_mhz[core]:>12.3f}"
            f"{absolute:>10}"
        )

    if len(want) >= 2:
        print()
        reference = want[0]
        for core in want[1:]:
            ratio = cores[core]["cycles"] / cores[reference]["cycles"]
            print(f"{core} takes {ratio:.3f}x {reference}'s cycles for the same work.")
        checked = ", ".join(f"{core} against {reference}" for core in want[1:])
        print(f"Data RAMs identical for {checked}.")

    for core in want:
        if core in waits and cores[core]["cycles"] > 0:
            share = 100.0 * waits[core] / cores[core]["cycles"]
            print(
                f"\n{core} spends {waits[core]} of its {cores[core]['cycles']} "
                f"measured cycles ({share:.2f}%) in a bus wait state the other "
                "cores here do not pay -- disclosed, not corrected."
            )

    if not clocks:
        print(
            "\nNo clock was given, so the DMIPS column is empty. It is each core's"
            " own\nworst placement from soc/compare/sweep.sh, and nothing here "
            "will guess one."
        )
    else:
        print(
            "\nTHE DMIPS COLUMN MULTIPLIES A CLOCK MEASURED AT THE PLACED 4 KB/2 KB\n"
            "GEOMETRY BY CYCLES MEASURED AT A LARGER SIMULATED ONE, because no ice40\n"
            "in this flow has the block RAM to hold Dhrystone. It is a projection.\n"
            "None of these are shipped designs: this core is RV32IMAC + Zicsr with\n"
            "traps, VexRiscv here is RV32IC with a branch predictor, no CSR file and\n"
            "no traps, Hazard3's iCE40 build is RV32IMA with no CSR counters and no\n"
            "interrupt, and the image is whichever ISA the caller compiled it at."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
