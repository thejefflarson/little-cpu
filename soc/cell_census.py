#!/usr/bin/env python3
"""Check yosys's own cell census in a synth log against a declared count.

Replaces the Makefile's `soc_expect_cells` define so `test/probe_gates.sh`
can drive `make soc-timing`'s SPRAM/EBR census against a fixture log without a
real yosys run -- the same reason `soc/timing_split.py` carries
`make soc-timing`'s frequency ratchet instead of a second parser in the
recipe.

Matched by cell NAME, not by presence: yosys logs a cell's library definition
in the same "Number of cells" table whether or not it actually instantiates
one, so `grep -q SB_SPRAM256KA` passed on a build that inferred zero of them.

Two flows read this: `make soc-timing`'s ice40 census and `make ecp5-timing`'s.
`--gate` and `--declared` are what let a failure name the target that stopped
and the variable to move, instead of pointing every reader at the other flow's.
"""

import argparse
import re
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="yosys synth_ice40 log, e.g. soc.synth.log")
    parser.add_argument("cell", help="cell type, e.g. SB_SPRAM256KA")
    parser.add_argument("expected", type=int, help="the declared, exact count")
    parser.add_argument("reason", help="what a mismatch means, printed on failure")
    parser.add_argument(
        "--gate",
        default="make soc-timing",
        help="the target this census runs under, so a failure names the flow "
        "that stopped. Diagnostic only.",
    )
    parser.add_argument(
        "--declared",
        default="SOC_EXPECT_SPRAM / SOC_EXPECT_EBR",
        help="the Makefile variable holding the expected count, so a deliberate "
        "change is pointed at the line to move. Diagnostic only.",
    )
    args = parser.parse_args()

    try:
        with open(args.log) as handle:
            lines = handle.read().splitlines()
    except FileNotFoundError:
        sys.exit(f"{args.log}: no such file")

    # The LAST match, matching the Makefile's `tail -1`: yosys prints this
    # table after more than one pass, and the final one is the whole-design
    # total.
    pattern = re.compile(r"^\s+(\d+)\s+" + re.escape(args.cell) + r"\s*$")
    got = 0
    for line in lines:
        m = pattern.match(line)
        if m:
            got = int(m.group(1))

    if got != args.expected:
        sys.exit(
            f"*** {args.gate}: {got} {args.cell} cells, expected {args.expected}.\n"
            f"*** {args.reason}.\n"
            f"*** Read {args.log}. If the change was deliberate, update\n"
            f"*** {args.declared} in the Makefile in the same commit."
        )
    print(f"{args.cell}: {got}, as declared")


if __name__ == "__main__":
    main()
