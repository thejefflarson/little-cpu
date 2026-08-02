#!/usr/bin/env python3
"""Check yosys's own cell census in a synth log against a declared count.

Replaces the Makefile's `soc_expect_cells` define so `test/probe_gates.sh`
can drive `make soc-timing`'s SPRAM/EBR census against a fixture log without a
real yosys run (ADR-0053) -- the same reason `soc/timing_split.py` carries
`make soc-timing`'s frequency ratchet instead of a second parser in the
recipe.

Matched by cell NAME, not by presence: yosys logs a cell's library definition
in the same "Number of cells" table whether or not it actually instantiates
one, so `grep -q SB_SPRAM256KA` passed on a build that inferred zero of them
(ADR-0054).
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
            f"*** make soc-timing: {got} {args.cell} cells, expected {args.expected}.\n"
            f"*** {args.reason}.\n"
            f"*** Read {args.log}. If the change was deliberate, update\n"
            "*** SOC_EXPECT_SPRAM / SOC_EXPECT_EBR in the Makefile in the same commit."
        )
    print(f"{args.cell}: {got}, as declared")


if __name__ == "__main__":
    main()
