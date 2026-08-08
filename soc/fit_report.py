#!/usr/bin/env python3
"""Read nextpnr's fit.log, print the utilisation table, and apply the
FIT_MAX_LC ratchet.

Replaces the Makefile's inline grep/sed/awk chain so `test/probe_gates.sh`
can drive both checks against a fixture log without running nextpnr --
the same reason `soc/timing_split.py` carries `make soc-timing`'s ratchet
instead of a second parser in the recipe.

`make fit` EXPECTS placement to fail: `littlecpu` with its memories external
presents 231 SB_IO against sg48's 39, so nextpnr always errors out on a pad,
after printing the utilisation table -- which is the measurement. nextpnr's
exit status carries no signal either way; the presence of the table is what
says a measurement was taken at all.
"""

import argparse
import re
import sys

UTIL_START = "Info: Device utilisation:"
LC_LINE = re.compile(r"ICESTORM_LC:\s+(\d+)/\s*(\d+)\s+(\S+)")

# Both spellings, because nextpnr words this differently depending on which
# phase gives up: "Unable to place cell ... no BELs remaining" when a BEL
# class is exhausted, "Unable to find a placement location for cell" when it
# cannot site a constrained IO. Matching only the first made a normal 80%-full
# run print the "read fit.log before quoting this" warning.
PLACEMENT_ERROR = re.compile(r"^ERROR: Unable to (place cell|find a placement location for cell)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="nextpnr's fit.log")
    parser.add_argument("--max-lc", type=int, required=True, help="FIT_MAX_LC budget")
    args = parser.parse_args()

    try:
        with open(args.log) as handle:
            lines = handle.read().splitlines()
    except FileNotFoundError:
        sys.exit(f"{args.log}: no such file")

    if not any("ICESTORM_LC:" in line for line in lines):
        tail = "\n".join(lines[-20:])
        sys.exit(
            "*** make fit: nextpnr printed no utilisation table, so NO measurement\n"
            f"*** was taken. That is a failure, not a 0% fit. Tail of {args.log}:\n{tail}"
        )

    in_table = False
    for line in lines:
        if line.startswith(UTIL_START):
            in_table = True
        if not in_table:
            continue
        if line.strip() == "":
            break
        if line.startswith("Info: "):
            print(line[len("Info: "):])

    # The last match, matching the Makefile's `tail -1`: yosys/nextpnr can
    # print more than one utilisation block, and the final one is the
    # whole-design total.
    lc = total = pct = None
    for line in lines:
        m = LC_LINE.search(line)
        if m:
            lc, total, pct = m.group(1), m.group(2), m.group(3)

    print(f"\nLOGIC CELLS: {lc} of {total} ({pct})  --  up5k / sg48 / littlecpu, memories external")

    if any(PLACEMENT_ERROR.match(line) for line in lines):
        print("Placement failed on IO, which is the expected state (ADR-0038 decision 1a);")
        print("the utilisation above is the measurement. Full log: fit.log")
    else:
        print("Placement did not report an IO error -- read fit.log before quoting this.")

    got = int(lc)
    if got > args.max_lc:
        sys.exit(
            f"\n*** make fit: {got} logic cells is over the {args.max_lc}-cell budget.\n"
            "*** This is a ratchet (ADR-0042), not a suggestion. Find what grew;\n"
            "*** raising FIT_MAX_LC in the Makefile needs a reason in the commit."
        )
    print(f"\nRATCHET: {got} of {args.max_lc} cells budgeted -- OK")


if __name__ == "__main__":
    main()
